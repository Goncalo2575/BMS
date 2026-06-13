/**
 * @file    bms_relays.c
 * @brief   BMS - Máquina de estados de segurança + actuação de relés (implementação)
 *
 *  Porta fiel da lógica da BMS do ano passado (ver main.c PL455), adaptada à
 *  BMS actual (BQ796xx):
 *    - BMS_OK         <- derivado de hbms->fault_flags + nfault_pending
 *    - tensão do bus  <- hbms->inverter_voltage_mv (GPIO4 / sensor HV)
 *    - tensão do pack <- hbms->pack_voltage_mv (soma das células)
 *    - monitores      <- polling com debounce (sem EXTI; evita conflito com
 *                        a EXTI9_5 do NFAULT)
 *    - atraso de pré-carga (750 ms) NÃO-bloqueante (o IWDG ~500 ms proíbe
 *      HAL_Delay longos no super-loop)
 *
 * @version 3.2.0
 */

#include "bms_relays.h"

/* =========================================================================
 * MACROS DE ACTUAÇÃO (active-high: SET = relé fechado / LED aceso)
 * ========================================================================= */
#define RLY_CLOSE(port, pin)  HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define RLY_OPEN(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)

/* =========================================================================
 * ESTADO INTERNO
 * ========================================================================= */
static BMS_RelayState_t s_state;
static bool             s_state_change;

static bool             s_pre_charge_enable;
static bool             s_dis_charge_enable;
static bool             s_charger_relay_enable;
static bool             s_bms_relay_open_latched;

static uint32_t         s_engaged_tick;     /* p/ atraso de pré-carga */

/* --- Debounce dos monitores --- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       last_raw;
    uint8_t       stable;     /* 1 = activo (pino HIGH) */
    uint32_t      t;          /* início da contagem de debounce */
} MonCh_t;

typedef enum {
    MON_IMD = 0,   /* PB0  */
    MON_TSMS,      /* PC8  */
    MON_ESDB,      /* PC6  */
    MON_ESDB_CHG,  /* PB14 */
    MON_CHARGER,   /* PB12 */
    MON_COUNT
} MonId_t;

static MonCh_t s_mon[MON_COUNT];

/* =========================================================================
 * UTILITÁRIOS
 * ========================================================================= */
static void mon_seed(MonId_t id, GPIO_TypeDef *port, uint16_t pin)
{
    s_mon[id].port     = port;
    s_mon[id].pin      = pin;
    s_mon[id].last_raw = 0U;
    s_mon[id].stable   = 0U;   /* arranque "não activo" -> fail-safe */
    s_mon[id].t        = 0U;
}

static void mon_update(MonId_t id, uint32_t now_ms)
{
    MonCh_t *d = &s_mon[id];
    uint8_t raw = (HAL_GPIO_ReadPin(d->port, d->pin) == GPIO_PIN_SET) ? 1U : 0U;

    if (raw != d->stable)
    {
        if (raw != d->last_raw)
        {
            d->last_raw = raw;
            d->t        = now_ms;          /* nova transição -> reinicia debounce */
        }
        else if ((now_ms - d->t) >= BMS_RELAY_DEBOUNCE_MS)
        {
            d->stable = raw;               /* estável durante o tempo de debounce */
        }
    }
    else
    {
        d->last_raw = raw;
    }
}

/* =========================================================================
 * INICIALIZAÇÃO
 * ========================================================================= */
void BMS_Relays_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* --- Saídas: relés + LEDs --- */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = BMS_PRE_CHARGE_PIN;  HAL_GPIO_Init(BMS_PRE_CHARGE_PORT,  &g);
    g.Pin = BMS_CHARGE_RELAY_PIN;HAL_GPIO_Init(BMS_CHARGE_RELAY_PORT,&g);
    g.Pin = BMS_DISCHARGE_PIN;   HAL_GPIO_Init(BMS_DISCHARGE_PORT,   &g);
    g.Pin = BMS_RELAY_PIN;       HAL_GPIO_Init(BMS_RELAY_PORT,       &g);
    g.Pin = BMS_BMSCHARGE_PIN;   HAL_GPIO_Init(BMS_BMSCHARGE_PORT,   &g);
    g.Pin = BMS_LED_GREEN_PIN;   HAL_GPIO_Init(BMS_LED_GREEN_PORT,   &g);
    g.Pin = BMS_LED_RED_PIN;     HAL_GPIO_Init(BMS_LED_RED_PORT,     &g);
    g.Pin = BMS_LED_BLUE_PIN;    HAL_GPIO_Init(BMS_LED_BLUE_PORT,    &g);

    /* --- Entradas: monitores (pull-down fail-safe) --- */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    g.Pin = BMS_IMD_STATUS_PIN;  HAL_GPIO_Init(BMS_IMD_STATUS_PORT,  &g);
    g.Pin = BMS_TSMS_PIN;        HAL_GPIO_Init(BMS_TSMS_PORT,        &g);
    g.Pin = BMS_ESDB_PIN;        HAL_GPIO_Init(BMS_ESDB_PORT,        &g);
    g.Pin = BMS_ESDB_CHG_PIN;    HAL_GPIO_Init(BMS_ESDB_CHG_PORT,    &g);
    g.Pin = BMS_CHARGER_SIG_PIN; HAL_GPIO_Init(BMS_CHARGER_SIG_PORT, &g);

    /* --- Estado seguro de arranque (igual ao ano passado) --- */
    RLY_CLOSE(BMS_RELAY_PORT,       BMS_RELAY_PIN);       /* BMS relay fechado */
    RLY_CLOSE(BMS_BMSCHARGE_PORT,   BMS_BMSCHARGE_PIN);   /* BMS charge fechado */
    RLY_OPEN (BMS_CHARGE_RELAY_PORT,BMS_CHARGE_RELAY_PIN);/* charger relay aberto */
    RLY_OPEN (BMS_PRE_CHARGE_PORT,  BMS_PRE_CHARGE_PIN);
    RLY_OPEN (BMS_DISCHARGE_PORT,   BMS_DISCHARGE_PIN);

    RLY_OPEN (BMS_LED_RED_PORT,   BMS_LED_RED_PIN);
    RLY_OPEN (BMS_LED_GREEN_PORT, BMS_LED_GREEN_PIN);
    RLY_OPEN (BMS_LED_BLUE_PORT,  BMS_LED_BLUE_PIN);

    /* --- Estado lógico --- */
    s_state                  = BMS_RLY_NOT_SAFE;
    s_state_change           = true;
    s_pre_charge_enable      = false;
    s_dis_charge_enable      = false;
    s_charger_relay_enable   = false;
    s_bms_relay_open_latched = false;
    s_engaged_tick           = 0U;

    mon_seed(MON_IMD,      BMS_IMD_STATUS_PORT,  BMS_IMD_STATUS_PIN);
    mon_seed(MON_TSMS,     BMS_TSMS_PORT,        BMS_TSMS_PIN);
    mon_seed(MON_ESDB,     BMS_ESDB_PORT,        BMS_ESDB_PIN);
    mon_seed(MON_ESDB_CHG, BMS_ESDB_CHG_PORT,    BMS_ESDB_CHG_PIN);
    mon_seed(MON_CHARGER,  BMS_CHARGER_SIG_PORT, BMS_CHARGER_SIG_PIN);
}

/* =========================================================================
 * TAREFA PRINCIPAL
 * ========================================================================= */
void BMS_Relays_Task(BMS_Handle_t *hbms, uint32_t now_ms)
{
    if (hbms == NULL) { return; }

    /* --- 1) Amostragem + debounce dos monitores --- */
    for (uint8_t i = 0U; i < (uint8_t)MON_COUNT; i++)
    {
        mon_update((MonId_t)i, now_ms);
    }

    bool imd_ok        = (s_mon[MON_IMD].stable != 0U);
    bool tsms_wd       = (s_mon[MON_TSMS].stable != 0U);
    bool esdb_wd       = (s_mon[MON_ESDB].stable != 0U);
    bool charger_opto  = (s_mon[MON_CHARGER].stable != 0U);
    /* MON_ESDB_CHG é amostrado (telemetria); na lógica do ano passado o
     * ESDB_charge_flag era forçado a true, pelo que não entra na decisão. */

    /* --- 2) BMS_OK (derivado da BMS actual: faults + NFAULT de hardware) --- */
    bool bms_ok = (hbms->fault_flags == 0U) &&
                  (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) == 0U);

    /* --- 3) Tensões (mV) para pré-carga/descarga --- */
    uint32_t bus_mv  = hbms->inverter_voltage_mv;
    uint32_t pack_mv = hbms->pack_voltage_mv;
    bool bus_ge_min  = (bus_mv >= BMS_BUS_MIN_MV);
    bool bus_le_min  = (bus_mv <= BMS_BUS_MIN_MV);
    uint32_t pre_thr = (pack_mv > 0U)
                     ? (pack_mv * BMS_PRECHARGE_PCT_NUM) / BMS_PRECHARGE_PCT_DEN
                     : 0U;
    bool bus_ge_90   = (pack_mv > 0U) && (bus_mv >= pre_thr);
    bool bus_lt_90   = (pack_mv > 0U) && (bus_mv <  pre_thr);

    /* --- 4) Relé do CARREGADOR: segue o optoacoplador (charger ligado) --- */
    if (charger_opto)
    {
        if (!s_charger_relay_enable)
        {
            RLY_CLOSE(BMS_CHARGE_RELAY_PORT, BMS_CHARGE_RELAY_PIN);
            s_charger_relay_enable = true;
        }
    }
    else
    {
        if (s_charger_relay_enable)
        {
            RLY_OPEN(BMS_CHARGE_RELAY_PORT, BMS_CHARGE_RELAY_PIN);
            s_charger_relay_enable = false;
        }
    }

    /* --- 5) BMS relay + BMS charge: abrir em falha (latch, como ano passado) --- */
    if (!bms_ok || !imd_ok)
    {
        RLY_OPEN(BMS_RELAY_PORT,     BMS_RELAY_PIN);
        RLY_OPEN(BMS_BMSCHARGE_PORT, BMS_BMSCHARGE_PIN);
        s_bms_relay_open_latched = true;
    }
#if (BMS_RELAY_AUTO_RECLOSE != 0)
    else if (s_bms_relay_open_latched)
    {
        RLY_CLOSE(BMS_RELAY_PORT,     BMS_RELAY_PIN);
        RLY_CLOSE(BMS_BMSCHARGE_PORT, BMS_BMSCHARGE_PIN);
        s_bms_relay_open_latched = false;
    }
#endif

    /* --- 6) ESDB / TSMS (lógica idêntica ao ano passado) --- */
    bool esdb_flag = (esdb_wd == (bms_ok && imd_ok));  /* evita erro ESDB com BMS/IMD NOK */
    bool tsms_flag = esdb_flag && tsms_wd;

    /* --- 7) Auto-abertura de descarga / pré-carga por tensão do bus --- */
    if (s_dis_charge_enable && bus_le_min)
    {
        RLY_OPEN(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
        s_dis_charge_enable = false;
    }
    if (s_pre_charge_enable && bus_ge_90)
    {
        RLY_OPEN(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
        s_pre_charge_enable = false;
    }

    /* --- 8) Selecção de estado --- */
    BMS_RelayState_t ns;
    if (!bms_ok || !imd_ok || !esdb_flag)
    {
        ns = BMS_RLY_NOT_SAFE;
    }
    else if (charger_opto)
    {
        ns = BMS_RLY_CHARGING;        /* carregador ligado -> nunca SAFE/ENGAGED */
    }
    else if (!tsms_flag)
    {
        ns = BMS_RLY_SAFE;
    }
    else
    {
        ns = BMS_RLY_ENGAGED;
    }

    if (ns != s_state)
    {
        s_state        = ns;
        s_state_change = true;
    }

    /* --- 9) Acções de entrada de estado --- */
    if (s_state_change)
    {
        switch (s_state)
        {
            case BMS_RLY_NOT_SAFE:
            case BMS_RLY_SAFE:
            case BMS_RLY_CHARGING:
                /* pré-carga desligada; descarga (bleed) ligada se houver bus */
                RLY_OPEN(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
                s_pre_charge_enable = false;
                if (!s_dis_charge_enable && bus_ge_min)
                {
                    RLY_CLOSE(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
                    s_dis_charge_enable = true;
                }
                break;

            case BMS_RLY_ENGAGED:
                /* descarga desligada; pré-carga adiada BMS_PRECHARGE_DELAY_MS */
                RLY_OPEN(BMS_DISCHARGE_PORT, BMS_DISCHARGE_PIN);
                s_dis_charge_enable = false;
                s_engaged_tick      = now_ms;
                break;

            default:
                break;
        }
        s_state_change = false;
    }

    /* --- 10) Pré-carga adiada do estado ENGAGED (não-bloqueante) --- */
    if ((s_state == BMS_RLY_ENGAGED) &&
        (!s_pre_charge_enable) &&
        ((now_ms - s_engaged_tick) >= BMS_PRECHARGE_DELAY_MS) &&
        bus_lt_90)
    {
        RLY_CLOSE(BMS_PRE_CHARGE_PORT, BMS_PRE_CHARGE_PIN);
        s_pre_charge_enable = true;
    }

    /* --- 11) LED cluster (conduzido pelo estado) --- */
    bool red = false, green = false, blue = false;
    switch (s_state)
    {
        case BMS_RLY_NOT_SAFE: red   = true;                          break;
        case BMS_RLY_SAFE:     green = ((now_ms % 1000U) < 500U);     break; /* 1 Hz */
        case BMS_RLY_ENGAGED:  green = true;                          break;
        case BMS_RLY_CHARGING: blue  = true;                          break;
        default:                                                      break;
    }
    HAL_GPIO_WritePin(BMS_LED_RED_PORT,   BMS_LED_RED_PIN,   red   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BMS_LED_GREEN_PORT, BMS_LED_GREEN_PIN, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BMS_LED_BLUE_PORT,  BMS_LED_BLUE_PIN,  blue  ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* =========================================================================
 * ACESSORES
 * ========================================================================= */
BMS_RelayState_t BMS_Relays_GetState(void)
{
    return s_state;
}

const char *BMS_Relays_GetStateString(void)
{
    switch (s_state)
    {
        case BMS_RLY_SAFE:     return "SAFE";
        case BMS_RLY_ENGAGED:  return "ENGAGED";
        case BMS_RLY_CHARGING: return "CHARGING";
        case BMS_RLY_NOT_SAFE: return "NOT_SAFE";
        default:               return "UNKNOWN";
    }
}

void BMS_Relays_GetMonitors(BMS_RelayMonitors_t *out)
{
    if (out == NULL) { return; }

    out->imd_ok              = (s_mon[MON_IMD].stable      != 0U);
    out->tsms                = (s_mon[MON_TSMS].stable     != 0U);
    out->esdb                = (s_mon[MON_ESDB].stable     != 0U);
    out->esdb_charger        = (s_mon[MON_ESDB_CHG].stable != 0U);
    out->charger             = (s_mon[MON_CHARGER].stable  != 0U);

    out->pre_charge_closed   = s_pre_charge_enable;
    out->discharge_closed    = s_dis_charge_enable;
    out->charge_relay_closed = s_charger_relay_enable;
    out->bms_relay_closed    = !s_bms_relay_open_latched;
}
