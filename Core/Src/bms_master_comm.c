/**
 * @file    bms_master_comm.c
 * @brief   BMS → Saída de DEBUG (USART2, PA2/PA3) — TX-only
 *
 *  Substitui o antigo módulo de telemetria+heartbeat (arquitectura
 *  distribuída) por uma simples emissão periódica de texto legível.
 *  Sem pacote binário, sem checksum, sem watchdog, sem link_ok.
 */

#include "bms_master_comm.h"
#include <stdio.h>     /* snprintf */
#include <string.h>

void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart2)
{
    if ((hcomm == NULL) || (huart2 == NULL)) { return; }

    hcomm->huart    = huart2;
    hcomm->tx_count = 0U;
    /* TX-only: nada de HAL_UART_Receive_IT. PA3 (RX) fica livre. */
}

void BMS_MasterComm_PrintDebug(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms)
{
    if ((hcomm == NULL) || (hbms == NULL) || (hcomm->huart == NULL)) { return; }

    char line[160];

    /* Linha única, legível, com os campos essenciais. snprintf garante
     * truncagem segura dentro do buffer. */
    int n = snprintf(line, sizeof(line),
        "[BMS] st=%s flt=%s(0x%08lX) pack=%lumV min=%umV max=%umV "
        "dV=%umV Tmax=%dC SoC=%u%% HV=%lumV ring=%s ctor=%s cerr=%lu crc=%lu\r\n",
        BMS_GetStateString(hbms->state),
        BMS_GetFaultString(hbms->fault_flags),
        (unsigned long)hbms->fault_flags,
        (unsigned long)hbms->pack_voltage_mv,
        (unsigned)hbms->min_cell_mv,
        (unsigned)hbms->max_cell_mv,
        (unsigned)hbms->delta_cell_mv,
        (int)hbms->max_temp_c,
        (unsigned)hbms->soc_percent,
        (unsigned long)hbms->inverter_voltage_mv,
        hbms->ring_intact ? "OK" : "BROKEN",
        hbms->contactor_closed ? "CLOSED" : "OPEN",
        (unsigned long)hbms->comm_error_count,
        (unsigned long)hbms->crc_error_count);

    if (n < 0) { return; }
    uint16_t len = (n >= (int)sizeof(line)) ? (uint16_t)(sizeof(line) - 1U)
                                            : (uint16_t)n;

    /* TX bloqueante (frame curto: ~150 bytes @ 115200 ≈ 13 ms). Sem DMA. */
    (void)HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, len, 50U);
    hcomm->tx_count++;
}
