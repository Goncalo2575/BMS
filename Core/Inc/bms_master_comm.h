/**
 * @file    bms_master_comm.h
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 *  Arquitectura CENTRALIZADA (v3.3): este STM32F446 é o único MCU — decide,
 *  actua os relés (via bms_relays) e reporta. Esta interface envia
 *  periodicamente (1 Hz) e POR EVENTO uma linha de texto legível com o estado
 *  essencial do pack para um terminal/registador ligado a PA2 (USART2_TX).
 *
 *  A linha inclui o estado do BMS + as decisões lógicas (contactor/BMS_OK/
 *  precharge) E, no fim, o estado da máquina de segurança (SAFE/ENGAGED/
 *  CHARGING/NOT_SAFE), os sinais dos monitores (IMD/TSMS/ESDB/ESDB_charger/
 *  charger) e o estado real dos relés — obtidos do módulo bms_relays.
 *
 *  O pino PA3 (USART2_RX) fica livre — reservado para comandos simples futuros,
 *  mas NÃO é usado nem monitorizado (sem recepção por interrupção).
 *
 *  ⚠ Esta linha periódica NÃO é o canal de segurança de baixa latência: a
 *  abertura crítica dos relés é feita localmente por bms_relays.c (não depende
 *  deste link). A perda da telemetria não compromete a segurança do pack.
 *
 *  PINOUT USART2:
 *    PA2 (USART2_TX) ──► terminal de debug / registador
 *    PA3 (USART2_RX) ◄── (livre)
 *    Baud: 115200, 8N1
 */

#ifndef BMS_MASTER_COMM_H
#define BMS_MASTER_COMM_H

#include "bq796xx_bms.h"

/* =========================================================================
 * HANDLE DO MÓDULO DE DEBUG
 * ========================================================================= */
typedef struct {
    UART_HandleTypeDef  *huart;     /* Handle USART2 (TX de debug) */
    uint32_t             tx_count;  /* Nº de linhas de debug enviadas */
} BMS_MasterComm_t;

/* =========================================================================
 * API PÚBLICA
 * ========================================================================= */

/**
 * @brief  Inicializa a saída de debug. Apenas guarda o handle do USART2.
 *         NÃO arma qualquer recepção (TX-only). Chamar após MX_USART2_UART_Init().
 */
void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart2);

/**
 * @brief  Envia uma linha de debug legível com o estado essencial do BMS,
 *         a máquina de segurança e o estado dos relés/monitores.
 *         Bloqueante (HAL_UART_Transmit, sem DMA). Chamar a 1 Hz e por evento.
 */
void BMS_MasterComm_PrintDebug(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms);

#endif /* BMS_MASTER_COMM_H */
