/**
 * @file    bms_relays.h
 * @brief   BMS - Máquina de estados de segurança + actuação de relés + LED cluster
 *
 *  Porta a lógica de relés da BMS do ano passado (PL455) para a BMS actual
 *  (BQ796xx). Este MCU passa a ACTUAR fisicamente os relés/contactores:
 *  pré-carga, descarga, charge (carregador), BMS_relay e BMS_charge — e a
 *  conduzir o LED cluster a partir do estado de segurança.
 *
 *  ⚠ ARQUITECTURA (v3.3): isto SUPERSEDE a nota "decide e reporta, sem actuação"
 *    da v3.2. Este MCU é agora o actuador dos relés. Actualizar o FMEA/FTA.
 *
 *  ── ESTADOS (regulamento FS/TS) ─────────────────────────────────────────
 *    SAFE      : verde intermitente 1 Hz   (TSMS aberto, relés todos fechados)
 *    ENGAGED   : verde contínuo            (TSMS fechado -> pré-carga/condução)
 *    CHARGING  : azul contínuo             (carregador ligado)
 *    NOT_SAFE  : vermelho contínuo         (BMS/IMD/ESDB aberto) [prioridade]
 *  Fail-safe: na dúvida, NOT_SAFE / LEDs apagados (= Not Safe por regulamento).
 *
 *  ── MAPA DE PINOS (CONFIRMAR polaridade real contra o esquema) ──────────
 *  SAÍDAS (relés, active-high: SET = relé fechado/energizado):
 *    PC0  pré-carga        PC1  charge (relé do carregador)
 *    PC2  descarga (ctr)   PC4  BMS_relay        PA6  BMS_charge
 *  SAÍDAS (LED):
 *    PA15 verde   PC11 vermelho   PC12 azul
 *    (PA15 = JTDI; libertado quando CubeMX SYS Debug = "Trace Asynchronous SW",
 *     que reserva PA13=SWDIO, PA14=SWCLK, PB3=SWO para o conector de programação)
 *  ENTRADAS (monitor, pull-down fail-safe, active-high: SET = activo/fechado):
 *    PB0  IMD status   PC8  TSMS   PC6  ESDB   PB14 ESDB charger   PB12 charger signal
 *
 * @version 3.3.0
 */

#ifndef BMS_RELAYS_H
#define BMS_RELAYS_H

#include "bq796xx_bms.h"

/* =========================================================================
 * CONFIGURAÇÃO DE PINOS  (editar aqui se o hardware diferir)
 * ========================================================================= */
/* --- Saídas: relés/contactores --- */
#define BMS_PRE_CHARGE_PORT     GPIOC
#define BMS_PRE_CHARGE_PIN      GPIO_PIN_0
#define BMS_CHARGE_RELAY_PORT   GPIOC          /* relé do CARREGADOR (charge) */
#define BMS_CHARGE_RELAY_PIN    GPIO_PIN_1
#define BMS_DISCHARGE_PORT      GPIOC          /* ctr_discharge */
#define BMS_DISCHARGE_PIN       GPIO_PIN_2
#define BMS_RELAY_PORT          GPIOC          /* (atual: PC4; ano passado: PB10) */
#define BMS_RELAY_PIN           GPIO_PIN_4
#define BMS_BMSCHARGE_PORT      GPIOA          /* (atual: PA6; ano passado: PC4) */
#define BMS_BMSCHARGE_PIN       GPIO_PIN_6

/* --- Saídas: LED cluster --- */
#define BMS_LED_GREEN_PORT      GPIOA
#define BMS_LED_GREEN_PIN       GPIO_PIN_15
#define BMS_LED_RED_PORT        GPIOC
#define BMS_LED_RED_PIN         GPIO_PIN_11
#define BMS_LED_BLUE_PORT       GPIOC
#define BMS_LED_BLUE_PIN        GPIO_PIN_12

/* --- Entradas: monitorização --- */
#define BMS_IMD_STATUS_PORT     GPIOB
#define BMS_IMD_STATUS_PIN      GPIO_PIN_0
#define BMS_TSMS_PORT           GPIOC          /* (atual: PC8; ano passado: KSI PC5) */
#define BMS_TSMS_PIN            GPIO_PIN_8
#define BMS_ESDB_PORT           GPIOC
#define BMS_ESDB_PIN            GPIO_PIN_6
#define BMS_ESDB_CHG_PORT       GPIOB
#define BMS_ESDB_CHG_PIN        GPIO_PIN_14
#define BMS_CHARGER_SIG_PORT    GPIOB          /* charger signal em PB12 */
#define BMS_CHARGER_SIG_PIN     GPIO_PIN_12

/* =========================================================================
 * PARÂMETROS  (iguais ao ano passado, salvo onde indicado)
 * ========================================================================= */
#define BMS_RELAY_DEBOUNCE_MS       100U    /* debounce dos monitores */
#define BMS_PRECHARGE_DELAY_MS      750U    /* atraso ENGAGED->pré-carga (era HAL_Delay) */
#define BMS_BUS_MIN_MV              5000U   /* limiar "bus com tensão" (5 V) */
#define BMS_PRECHARGE_PCT_NUM       9U      /* 90% = 9/10 da tensão do pack */
#define BMS_PRECHARGE_PCT_DEN       10U

/* Reclose automático do BMS_relay após a falha desaparecer.
 * 0 = latch aberto (igual ao ano passado: requer reset). 1 = re-fecha. */
#define BMS_RELAY_AUTO_RECLOSE      0

/* =========================================================================
 * TIPOS
 * ========================================================================= */
typedef enum {
    BMS_RLY_SAFE = 0,
    BMS_RLY_ENGAGED,
    BMS_RLY_CHARGING,
    BMS_RLY_NOT_SAFE
} BMS_RelayState_t;

/** Snapshot dos sinais de monitorização (debounced) e dos relés (para telemetria) */
typedef struct {
    /* Entradas (monitores) */
    bool imd_ok;               /* PB0  */
    bool tsms;                 /* PC8  */
    bool esdb;                 /* PC6  */
    bool esdb_charger;         /* PB14 */
    bool charger;              /* PB12 */
    /* Saídas (estado lógico dos relés) */
    bool pre_charge_closed;    /* PC0  */
    bool discharge_closed;     /* PC2  */
    bool charge_relay_closed;  /* PC1  */
    bool bms_relay_closed;     /* PC4/PA6 (false = latched aberto por falha) */
} BMS_RelayMonitors_t;

/* =========================================================================
 * API
 * ========================================================================= */

/**
 * @brief  Configura GPIO (relés + LEDs como saída; monitores como entrada
 *         pull-down) e coloca o sistema em estado seguro de arranque:
 *         BMS_relay/BMS_charge FECHADOS, restantes relés ABERTOS, LEDs apagados.
 *         Liga os clocks de GPIOA/B/C. Chamar uma vez no arranque.
 */
void BMS_Relays_Init(void);

/**
 * @brief  Tarefa de segurança: lê monitores (debounced), calcula BMS_OK a
 *         partir do estado da BMS (faults + NFAULT), actua os relés e o LED
 *         cluster e actualiza o estado SAFE/ENGAGED/CHARGING/NOT_SAFE.
 *         Não-bloqueante. Chamar em TODAS as iterações do super-loop.
 */
void BMS_Relays_Task(BMS_Handle_t *hbms, uint32_t now_ms);

/** @brief  Estado de segurança actual. */
BMS_RelayState_t BMS_Relays_GetState(void);

/** @brief  Nome legível do estado (para telemetria/debug). */
const char *BMS_Relays_GetStateString(void);

/** @brief  Snapshot dos monitores + estado dos relés (para telemetria). */
void BMS_Relays_GetMonitors(BMS_RelayMonitors_t *out);

#endif /* BMS_RELAYS_H */
