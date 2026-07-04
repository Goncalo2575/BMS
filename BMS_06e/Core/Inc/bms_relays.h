/**
 * @file    bms_relays.h
 * @brief   BMS - Máquina de estados de segurança + actuação de relés + LED cluster
 *
 *  ── O MCU COMANDA 5 RELÉS ────────────────────────────────────────────────
 *    PC0 pre_charge_relay · PC2 discharge_relay (bleed) · PC1 charge_relay ·
 *    PA6 BMS_charge_relay · PC4 BMS_relay   (BMS_relay e BMS_charge_relay têm
 *    a MESMA lógica: abrem em falha do BMS e ficam em latch).
 *
 *  ── ESTADOS ─────────────────────────────────────────
 *    SAFE      : verde intermitente 1 Hz   (TSMS aberto, relés todos fechados)
 *    ENGAGED   : verde contínuo            (TSMS fechado -> pré-carga/condução)
 *    CHARGING  : azul contínuo             (carregador ligado)
 *    NOT_SAFE  : vermelho contínuo         (BMS/IMD/ESDB aberto) [prioridade]
 *  Fail-safe: na dúvida, NOT_SAFE / LEDs apagados (= Not Safe por regulamento).
 *
 *  ── MAPA DE PINOS (CONFIRMAR polaridade real contra o esquema) ──────────
 *  SAÍDAS (relés, active-high: SET = relé fechado/energizado):
 *    PC0  pré-carga        PC1  charge (relé do carregador)
 *    PC2  descarga    PC4  BMS_relay        PA6  BMS_charge
 *  SAÍDAS (LED):
 *    PA15 verde   PC11 vermelho   PC12 azul
 *    
 *  ENTRADAS (monitor, pull-down fail-safe, active-high: SET = activo/fechado):
 *    PB0  IMD status   PC8  TSMS   PC6  ESDB   PB14 ESDB charger   PB12 charger signal                 /////////////////////juntar o IMDmonitor/////////////////////
 *
 */

#ifndef BMS_RELAYS_H
#define BMS_RELAYS_H

#include "bq796xx_bms.h"

/* =========================================================================
 * CONFIGURAÇÃO DE PINOS  
 * ========================================================================= */
/* --- Saídas: relés --- */
#define BMS_PRE_CHARGE_PORT     GPIOC          
#define BMS_PRE_CHARGE_PIN      GPIO_PIN_0
#define BMS_CHARGE_RELAY_PORT   GPIOC          
#define BMS_CHARGE_RELAY_PIN    GPIO_PIN_1
#define BMS_DISCHARGE_PORT      GPIOC          
#define BMS_DISCHARGE_PIN       GPIO_PIN_2
#define BMS_RELAY_PORT          GPIOC          
#define BMS_RELAY_PIN           GPIO_PIN_4
#define BMS_BMSCHARGE_PORT      GPIOA          
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
#define BMS_TSMS_PORT           GPIOC         
#define BMS_TSMS_PIN            GPIO_PIN_8
#define BMS_ESDB_PORT           GPIOC
#define BMS_ESDB_PIN            GPIO_PIN_6
#define BMS_ESDB_CHG_PORT       GPIOB
#define BMS_ESDB_CHG_PIN        GPIO_PIN_14
#define BMS_CHARGER_SIG_PORT    GPIOB         
#define BMS_CHARGER_SIG_PIN     GPIO_PIN_12

/* =========================================================================
 * PARÂMETROS  
 * ========================================================================= */
#define BMS_RELAY_DEBOUNCE_MS       100U    /* debounce dos monitores */
#define BMS_PRECHARGE_DELAY_MS      750U    /* atraso ENGAGED->pré-carga */
#define BMS_BUS_MIN_MV              5000U   /* limiar "bus com tensão" (5 V) */
#define BMS_PRECHARGE_PCT_NUM       9U      /* 90%  da tensão do pack */
#define BMS_PRECHARGE_PCT_DEN       10U

/* Reclose automático do BMS_relay após a falha desaparecer.
 * 0 = latch aberto (igual ao ano passado: requer reset). 1 = re-fecha. */
#define BMS_RELAY_AUTO_RECLOSE      0



//// !!!!!!!!!!!!!!!!!!!!!!! VERRRR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




/* Comportamento do relé de DESCARGA/BLEED (PC2) durante CHARGING.
 * ⚠ DEPENDE DO LAYOUT FÍSICO — confirmar no esquema onde está o resistor:
 *   1 = HOLD: bleed FECHADO de forma contínua → mantém o bus de tração a 0 V
 *       durante todo o carregamento (D.5.3.7). SÓ É SEGURO se o resistor de
 *       bleed estiver no lado do INVERSOR (depois do Line_contactor): com o
 *       contactor aberto o bus está a 0 V, P = V²/R = 0 W.
 *   0 = SAFE-like: bleed transitório (fecha se bus>5 V, o passo 7 abre a ≤5 V).
 * NOTA: se o resistor estiver no lado do PACK/carga (vê 126 V em carga), NENHUMA
 * destas opções o protege — em SAFE-like também ficaria fechado para sempre
 * (o pack nunca desce a 5 V) e queimaria. Aliás, a lógica existente (abrir a
 * ≤5 V) só funciona se o bus PUDER chegar a ~0 V, o que implica bleed no lado
 * do inversor. Um bleed do lado do pack é um ERRO DE HARDWARE, não de software.
 * Default = 1 (assume bleed no lado do inversor, requisito para cumprir D.5.3.7). */
#define BMS_BLEED_HOLD_IN_CHARGING  1



////////////////////////////////////////////////////////////////


/* =========================================================================
 * estruturas
 * ========================================================================= */
typedef enum {
    BMS_RLY_SAFE = 0,
    BMS_RLY_ENGAGED,
    BMS_RLY_CHARGING,
    BMS_RLY_NOT_SAFE
} BMS_RelayState_t;


typedef struct {
    /* Entradas  */
    bool imd_ok;               /* PB0  */
    bool tsms;                 /* PC8  */
    bool esdb;                 /* PC6  */
    bool esdb_charger;         /* PB14 */
    bool charger;              /* PB12 */


    /* Saídas  */
    bool pre_charge_closed;    /* PC0  */
    bool discharge_closed;     /* PC2  */
    bool charge_relay_closed;  /* PC1  */
    bool bms_relay_closed;     /* PC4/PA6  */
} BMS_RelayMonitors_t;

/* =========================================================================
 * fUNÇOES
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
