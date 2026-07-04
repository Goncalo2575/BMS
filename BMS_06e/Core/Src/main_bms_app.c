/**
 * @file    main_bms_app.c
 * @brief   BMS - Aplicação Principal e Ponto de Entrada (Arquitectura Centralizada)
 *
 *  Este STM32F446RET6 monitoriza o pack (BQ79600 + 2x BQ79616) E actua a
 *  malha de segurança/relés (lógica portada da BMS do ano passado — ver
 *  bms_relays.c). A máquina de estados SAFE/ENGAGED/CHARGING/NOT_SAFE comanda
 *  pré-carga, descarga, relé do carregador, BMS_relay/BMS_charge e o LED cluster.
 *
 *  ⚠ Isto SUPERSEDE a nota "decide e reporta, sem actuação": este MCU
 *  passou a ser o actuador dos relés.
 *
 *  PINOUT (ver bms_relays.h para o mapa completo e conflitos a confirmar):
 *    PA0/PA1 UART4 (bridge)   PA2/PA3 USART2 (debug TX)   PA8 NFAULT
 *    Relés:  PC0 pré-carga  PC1 charge  PC2 descarga  PC4 BMS_relay  PA6 BMS_charge
 *    LED:    PA15 verde  PC11 vermelho  PC12 azul
 *    Monitor:PB0 IMD  PC8 TSMS  PC6 ESDB  PB14 ESDB_chg  PB12 charger_sig
 */

#include "bq796xx_bms.h"
#include "bms_master_comm.h"
#include "bms_relays.h"
#include <stdio.h>
#include <stdbool.h>

/* =========================================================================
 * VARIÁVEIS GLOBAIS DA APLICAÇÃO
 * ========================================================================= */

static BMS_Handle_t      g_bms;
BMS_MasterComm_t         g_master_comm;

extern UART_HandleTypeDef  huart4;   /* UART4 — BQ79600 bridge (PA0/PA1) */
extern UART_HandleTypeDef  huart2;   /* USART2 (PA2/PA3) — debug/telemetria TX-only */
extern TIM_HandleTypeDef   htim2;    /* TIM2  — Delay µs (contador livre) */

#define BMS_TASK_PERIOD_MS   100U

/* =========================================================================
 * FUNÇÕES DE APOIO À APLICAÇÃO
 * ========================================================================= */

/**
 * @brief  Mantém a expectativa interna de "tração activa" (contactor_closed)
 *
 * Para que serve: mantém contactor_closed coerente com o estado do BMS. NÃO
 * comanda contactores nem pinos — o Line_contactor é fechado pelo INVERSOR
 * (Sevcon Gen4 Size 6) quando B+ atinge o limiar programado. Esta flag serve só
 * para (a) gating do cálculo de SoC e (b) telemetria/reporte por evento.
 *
 * Como funciona: se houver NFAULT pendente sai logo. Caso contrário, marca
 * "tração activa" em operação normal (MONITORING ou BALANCING, sem falhas) e
 * desde que NÃO esteja a carregar — passando pelas barreiras de
 * BMS_ContactorClose — e "inactiva" em qualquer outro caso (incluindo CHARGING,
 * por separação total tração/carga).
 */
static void BMS_ContactorControl(BMS_Handle_t *hbms)
{
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return;
    }

    bool charging = (BMS_Relays_GetState() == BMS_RLY_CHARGING);

    if (!charging &&
        ((hbms->state == BMS_STATE_MONITORING) ||
         (hbms->state == BMS_STATE_BALANCING)) &&
        (hbms->fault_flags == BMS_FAULT_NONE))
    {
        if (!hbms->contactor_closed)
        {
            BMS_ContactorClose(hbms);   /* Barreiras internas reforçam aqui */
        }
    }
    else
    {
        if (hbms->contactor_closed)
        {
            BMS_ContactorOpen(hbms);    /* inclui CHARGING → tração inactiva */
        }
    }
}

/**
 * @brief  Limpa os registos de latch de falha em todos os dispositivos
 *
 * Para que serve: passo obrigatório antes de tentar voltar a MONITORING após
 * uma falha. Os BQ79616/BQ79600 retêm (latch) as falhas em hardware; sem este
 * clear, o NFAULT voltaria a disparar de imediato.
 *
 * Como funciona: escreve 0xFF em FAULT_RST1/RST2 por broadcast (slaves) e, como
 * o broadcast não chega à bridge, repete em single-write para a bridge.
 */
static BMS_Status_t BMS_ClearHardwareFaultLatches(BMS_Handle_t *hbms)
{
    uint8_t      clear_val = FAULT_CLEAR_VAL;  /* 0xFF */
    BMS_Status_t status;

    status = BMS_WriteBroadcast(hbms, REG_FAULT_RST1, &clear_val, 1U);
    if (status != BMS_OK) { return status; }

    status = BMS_WriteBroadcast(hbms, REG_FAULT_RST2, &clear_val, 1U);
    if (status != BMS_OK) { return status; }

    /* Latches da Bridge (Broadcast não chega à bridge — precisa Single) */
    (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                REG_FAULT_RST1, &clear_val, 1U);
    (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                REG_FAULT_RST2, &clear_val, 1U);

    return BMS_OK;
}

/**
 * @brief  Tentativa de recuperação automática a partir do estado FAULT
 *
 * Para que serve: dá ao sistema a hipótese de recuperar de falhas transitórias
 * (ex.: ring break recuperável) sem intervenção humana, mas SEM nunca religar
 * sobre uma condição perigosa.
 *
 * Como funciona:
 *  - Limita as tentativas: à 4ª (retry > 3) desiste e adormece o pack em
 *    segurança (BMS_EnterSleep), evitando o ciclo infinito FAULT→SHUTDOWN→FAULT.
 *  - Reavalia tensões/temperaturas e marca como NÃO recuperável se a grandeza
 *    em falha ainda está perto do limite (OV/UV/OT), ou se a falha é física
 *    (open-wire) ou de comunicação total.
 *  - Só se recuperável: limpa os latches de hardware, zera as flags por célula
 *    e regressa a MONITORING (reabilitando a decisão de fechar o contactor).
 */
static void BMS_FaultRecoveryAttempt(BMS_Handle_t *hbms,
                                      uint32_t *retry_counter)
{
    (*retry_counter)++;

    if (*retry_counter > 3U)
    {
        /* Adormece fisicamente o barramento; o guard de SLEEP em
         * BMS_Task_100ms quebra o ciclo infinito FAULT→SHUTDOWN→FAULT. */
        BMS_EnterSleep(hbms);
        return;
    }

    BMS_ReadAllCellVoltages(hbms);
    BMS_ReadAllTemperatures(hbms);

    bool recoverable = true;

    if ((hbms->fault_flags & BMS_FAULT_OV) &&
        (hbms->max_cell_mv > (CELL_OV_MV - 50U)))
    {
        recoverable = false;
    }
    if ((hbms->fault_flags & BMS_FAULT_UV) &&
        (hbms->min_cell_mv < (CELL_UV_MV + 50U)))
    {
        recoverable = false;
    }
    if ((hbms->fault_flags & BMS_FAULT_OT) &&
        (hbms->max_temp_c > (int16_t)(CELL_TEMP_MAX_C - 5)))
    {
        recoverable = false;
    }
    /* Open Wire: falha física — não recuperável por software */
    if (hbms->fault_flags & BMS_FAULT_OPEN_WIRE)
    {
        recoverable = false;
    }
    /* Falha de comunicação total: não recuperável sem re-init */
    if ((hbms->fault_flags & BMS_FAULT_COMM) &&
        !(hbms->fault_flags & BMS_FAULT_RING_BREAK))
    {
        recoverable = false;
    }

    if (recoverable)
    {
        /* Limpar latches de hardware ANTES de transitar para MONITORING. */
        if (BMS_ClearHardwareFaultLatches(hbms) != BMS_OK)
        {
            hbms->fault_flags |= BMS_FAULT_COMM;
            return;
        }

        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            hbms->slave[s].fault_flags = 0U;
            for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
            {
                hbms->slave[s].ov_cell[c] = false;
                hbms->slave[s].uv_cell[c] = false;
            }
        }

        hbms->fault_flags &= ~(BMS_FAULT_OV  |
                                BMS_FAULT_UV  |
                                BMS_FAULT_OT  |
                                BMS_FAULT_HB_FAIL |
                                BMS_FAULT_CRC);

        hbms->state    = BMS_STATE_MONITORING;
        *retry_counter = 0U;
    }
}

/* =========================================================================
 * PONTO DE ENTRADA DA APLICAÇÃO BMS
 * ========================================================================= */

/**
 * @brief  Ponto de entrada da aplicação BMS (chamado de main.c, USER CODE BEGIN 2)
 *
 * Para que serve: é o "main" funcional do BMS. Põe o hardware em estado seguro,
 * inicializa a daisy-chain, corre o auto-teste e entra no super-loop que
 * monitoriza o pack e actua a malha de segurança/relés indefinidamente.
 *
 * Como funciona (fases):
 *  FASE 1  — Relés/LED em estado seguro (ANTES de tudo), depois BMS_Init. Se o
 *            init falhar, fica num loop que NÃO refresca o IWDG de propósito:
 *            o watchdog (~500 ms) reseta e re-tenta (auto-recuperação Via B).
 *  FASE 1b — Telemetria USART2 (TX-only).
 *  FASE 1c — IWDG já armado pelo CubeMX; aqui só se anuncia.
 *  FASE 1d — POST (CRC/Comm/ADC/NFAULT).
 *  FASE 2  — Leituras iniciais (incl. tensão do bus) e decisão lógica inicial
 *            de contactor; 1ª passagem da malha de relés + reporte inicial.
 *  FASE 3  — Super-loop: em TODAS as iterações refresca o IWDG e corre
 *            BMS_Relays_Task (debounce, blink 1 Hz, pré-carga não-bloqueante);
 *            processa NFAULT fora da cadência (propagando logo aos relés);
 *            a cada 100 ms corre BMS_Task_100ms + decisão de contactor, reporta
 *            por evento (mudança de contactor/BMS_OK), gere o recovery em FAULT
 *            (50 ciclos, máx. 3 tentativas) e envia o heartbeat de telemetria a 1 Hz.
 */
void BMS_Main(void)
{
    BMS_Status_t status;
    uint32_t     fault_cycle_count  = 0U;   /* ciclos em FAULT desde a última transição */
    uint32_t     fault_retry_count  = 0U;   /* tentativas de recovery efectuadas */
    uint32_t     debug_print_div    = 0U;
    uint32_t     last_task_tick;            /* timestamp do último ciclo de 100 ms */
    bool         last_contactor_decision;   /* p/ reporte por evento (FTTI) */
    bool         last_bms_ok;

    /* ------------------------------------------------------------------
     * FASE 1: Inicialização do BMS
     * ------------------------------------------------------------------ */
    printf("\r\n[BMS] Firmware v%s\r\n", BMS_FW_VERSION_STRING);
    printf("[BMS] Config: %u slaves x %u cells = %u total\r\n",
           BMS_NUM_SLAVES, BMS_CELLS_PER_SLAVE, BMS_TOTAL_CELLS);

    /* Relés + LED em estado seguro ANTES de tudo (BMS relay fechado, contactores
     * abertos, LEDs apagados). */
    BMS_Relays_Init();
    printf("[BMS] Relays/LED init OK (safe-state)\r\n");

    status = BMS_Init(&g_bms, &huart4, &htim2);
    if (status != BMS_OK)
    {
        printf("[BMS] FATAL: Initialization failed! Code: %d\r\n", (int)status);
        /* AUTO-RECUPERAÇÃO (Via B): este loop NÃO refresca o IWDG de propósito.
         * O watchdog (armado pelo CubeMX) dispara ao fim de ~500 ms e reinicia
         * o MCU, que volta a tentar BMS_Init. NÃO adicionar BMS_IWDG_Refresh. */
        while (1U)
        {
            BMS_DelayMs(200U);
        }
    }

    printf("[BMS] Initialization OK. State: %s\r\n",
           BMS_GetStateString(g_bms.state));
    printf("[BMS] Ring intact: %s\r\n", g_bms.ring_intact ? "YES" : "NO");

    /* ------------------------------------------------------------------
     * FASE 1b: Inicializar saída de debug/telemetria (USART2, TX-only)
     * ------------------------------------------------------------------ */
    BMS_MasterComm_Init(&g_master_comm, &huart2);
    printf("[BMS] Telemetry/debug init OK (USART2 PA2/PA3, TX-only)\r\n");

    /* ------------------------------------------------------------------
     * FASE 1c: IWDG Watchdog (ASIL-D) — armado pelo CubeMX (MX_IWDG_Init).
     * ------------------------------------------------------------------ */
    printf("[BMS] IWDG active (CubeMX MX_IWDG_Init, timeout ~500 ms)\r\n");

    /* ------------------------------------------------------------------
     * FASE 1d: Power-On Self Test (POST)
     * ------------------------------------------------------------------ */
    status = BMS_PowerOnSelfTest(&g_bms);
    if (status != BMS_OK)
    {
        printf("[BMS] POST FAILED! Fault: %s\r\n",
               BMS_GetFaultString(g_bms.fault_flags));
        g_bms.state = BMS_STATE_FAULT;
    }
    else
    {
        printf("[BMS] POST passed (CRC, Comm, ADC, NFAULT all OK)\r\n");
    }

    /* ------------------------------------------------------------------
     * FASE 2: Decisão lógica inicial de contactor (gating de SoC/telemetria;
     *          actuação física é do módulo de relés).
     * ------------------------------------------------------------------ */
    BMS_DelayMs(100U);
    BMS_ReadAllCellVoltages(&g_bms);
    BMS_ReadAllTemperatures(&g_bms);
    BMS_ReadInverterVoltage(&g_bms);          /* tensão do bus p/ pré-carga */
    BMS_UpdateHardwareInterlocks(&g_bms);     /* calcula bms_ok inicial */

    if ((g_bms.fault_flags == BMS_FAULT_NONE) && (g_bms.post_passed))
    {
        printf("[BMS] Conditions OK at startup.\r\n");
        BMS_ContactorClose(&g_bms);
    }
    else
    {
        printf("[BMS] Startup with fault: %s\r\n",
               BMS_GetFaultString(g_bms.fault_flags));
    }

    /* Primeira passagem da malha de segurança/relés + reporte inicial */
    BMS_Relays_Task(&g_bms, HAL_GetTick());
    BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
    last_contactor_decision = g_bms.contactor_closed;
    last_bms_ok             = g_bms.bms_ok;

    /* ------------------------------------------------------------------
     * FASE 3: Super-loop de operação
     * ------------------------------------------------------------------ */
    printf("[BMS] Entering monitoring loop. Safety state: %s\r\n",
           BMS_Relays_GetStateString());

    last_task_tick = HAL_GetTick();

    while (1U)
    {
        /* IWDG: refrescar em TODAS as iterações (independente do tick). */
        BMS_IWDG_Refresh();

        /* MALHA DE SEGURANÇA / RELÉS + LED: corre em TODAS as iterações
         * (debounce dos monitores, blink 1 Hz, pré-carga não-bloqueante). */
        BMS_Relays_Task(&g_bms, HAL_GetTick());

        /* NFAULT fora do tick — processamento imediato. */
        if (__atomic_load_n(&g_bms.nfault_pending, __ATOMIC_SEQ_CST) != 0U)
        {
            BMS_ProcessFaults(&g_bms);

            /* Propagar de imediato à malha de relés (abre BMS relay etc.). */
            BMS_Relays_Task(&g_bms, HAL_GetTick());

            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            last_contactor_decision = g_bms.contactor_closed;
            last_bms_ok             = g_bms.bms_ok;
            debug_print_div         = 0U;
        }

        /* Cadência de 100 ms via HAL_GetTick (base TIM6). */
        if ((HAL_GetTick() - last_task_tick) < BMS_TASK_PERIOD_MS)
        {
            BMS_DelayMs(1U);
            continue;
        }
        last_task_tick += BMS_TASK_PERIOD_MS;   /* cadência sem deriva acumulada */

        /* TAREFA PRINCIPAL DO BMS @ 100 ms */
        status = BMS_Task_100ms(&g_bms);

        /* Decisão LÓGICA de contactor (gating de SoC/telemetria) */
        BMS_ContactorControl(&g_bms);

        /* REPORTE POR EVENTO: se a decisão de contactor ou o BMS_OK mudaram. */
        if ((g_bms.contactor_closed != last_contactor_decision) ||
            (g_bms.bms_ok           != last_bms_ok))
        {
            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            last_contactor_decision = g_bms.contactor_closed;
            last_bms_ok             = g_bms.bms_ok;
            debug_print_div         = 0U;
        }

        /* GESTÃO DE RECOVERY EM ESTADO FAULT */
        if (g_bms.state == BMS_STATE_FAULT)
        {
            fault_cycle_count++;

            if ((fault_cycle_count >= 50U) && (fault_retry_count <= 3U))
            {
                fault_cycle_count = 0U;
                BMS_FaultRecoveryAttempt(&g_bms, &fault_retry_count);
            }
        }
        else
        {
            fault_cycle_count = 0U;
            fault_retry_count = 0U;
        }

        /* HEARTBEAT DE TELEMETRIA @ 1 Hz (a cada 10 ciclos de 100 ms) */
        debug_print_div++;
        if (debug_print_div >= 10U)
        {
            debug_print_div = 0U;
            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
        }
    }
}
