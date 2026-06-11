/**
 * @file    bms_master_comm.c
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 *  Arquitectura CENTRALIZADA: este STM32F446 (Master) reporta o estado e as
 *  DECISÕES de segurança (contactor abrir/fechar, BMS_OK, PRECHARGE_OK) ao
 *  estágio de potência / registador via uma linha de texto legível a 1 Hz.
 *
 *  ⚠ Esta linha periódica NÃO é o canal de segurança de baixa latência:
 *  a decisão crítica de abrir o contactor é também emitida POR EVENTO no
 *  super-loop (ver main_bms_app.c) para cumprir o FTTI. Ver a NOTA DE
 *  SEGURANÇA em bq796xx_bms_monitor.c (Secção 9).
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

    char line[192];

    /* Linha única, legível, com os campos essenciais + interlocks reportados
     * ao master (ok=BMS_OK, pre=PRECHARGE_OK, ctor=decisão de contactor).
     * snprintf garante truncagem segura dentro do buffer. */
    int n = snprintf(line, sizeof(line),
        "[BMS] st=%s flt=%s(0x%08lX) pack=%lumV min=%umV max=%umV "
        "dV=%umV Tmax=%dC SoC=%u%% HV=%lumV ring=%s ctor=%s ok=%u pre=%u "
        "cerr=%lu crc=%lu\r\n",
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
        hbms->contactor_closed ? "CLOSE" : "OPEN",   /* DECISÃO p/ o master */
        (unsigned)(hbms->bms_ok ? 1U : 0U),          /* interlock BMS_OK */
        (unsigned)(hbms->precharge_ready ? 1U : 0U), /* interlock PRECHARGE_OK */
        (unsigned long)hbms->comm_error_count,
        (unsigned long)hbms->crc_error_count);

    if (n < 0) { return; }
    uint16_t len = (n >= (int)sizeof(line)) ? (uint16_t)(sizeof(line) - 1U)
                                            : (uint16_t)n;

    /* TX bloqueante (frame curto @ 115200). Sem DMA. */
    (void)HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, len, 50U);
    hcomm->tx_count++;
}
