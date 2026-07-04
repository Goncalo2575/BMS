/**
 * @file    bms_master_comm.h
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 */

#ifndef BMS_MASTER_COMM_H
#define BMS_MASTER_COMM_H

#include "bq796xx_bms.h"

typedef struct {
    UART_HandleTypeDef  *huart;     /* Handle USART2 (TX de debug) */
    uint32_t             tx_count;  /* Nº de linhas de debug enviadas */
} BMS_MasterComm_t;


/**
 * @brief  Inicializa a saída de debug. Apenas guarda o handle do USART2.
 *         NÃO arma qualquer recepção (TX-only). Chamar após MX_USART2_UART_Init().
 */
void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart2);

/**
 * @brief  Envia uma linha de debug legível com o estado essencial do BMS,
 *         a máquina de segurança e o estado dos relés/monitores. Chamar a 1 Hz e por evento.
 */
void BMS_MasterComm_PrintDebug(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms);

#endif /* BMS_MASTER_COMM_H */
