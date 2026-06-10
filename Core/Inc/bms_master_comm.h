/**
 * @file    bms_master_comm.h
 * @brief   BMS → Saída de DEBUG (USART2, PA2/PA3) — TX-only
 *
 *  Arquitectura CENTRALIZADA: não há VCU externa nem heartbeat.
 *  Esta interface envia periodicamente (1 Hz) uma linha de texto legível
 *  com o estado essencial do BMS para um terminal/registador ligado a
 *  PA2 (USART2_TX). O pino PA3 (USART2_RX) fica livre — reservado para
 *  comandos simples futuros (ex.: pedir status), mas NÃO é usado nem
 *  monitorizado (sem recepção por interrupção, sem watchdog).
 *
 *  Toda a segurança (protecção térmica, OV/UV, contactor, NFAULT) é
 *  interna ao STM32; a ausência deste link NUNCA provoca falha.
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
 * @brief  Envia uma linha de debug legível com o estado essencial do BMS.
 *         Bloqueante (HAL_UART_Transmit, sem DMA). Chamar a 1 Hz.
 */
void BMS_MasterComm_PrintDebug(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms);

#endif /* BMS_MASTER_COMM_H */
