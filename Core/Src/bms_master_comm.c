/**
 * @file    bms_master_comm.c
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 *  Arquitectura CENTRALIZADA: este STM32F446 (Master) reporta o estado e as
 *  DECISÕES de segurança ao estágio de potência / registador via uma linha de
 *  texto legível a 1 Hz (+ reporte por evento).
 *
 *  A linha inclui agora o estado da máquina de segurança (SAFE/ENGAGED/
 *  CHARGING/NOT_SAFE) e os sinais dos monitores (IMD/TSMS/ESDB/ESDB_charger/
 *  charger) e o estado dos relés, vindos do módulo bms_relays.
 *
 *  ⚠ Esta linha periódica NÃO é o canal de segurança de baixa latência:
 *  a decisão crítica de abrir o contactor é também emitida POR EVENTO no
 *  super-loop (ver main_bms_app.c). Ver a NOTA DE SEGURANÇA em
 *  bq796xx_bms_monitor.c (Secção 9).
 */

#include "bms_master_comm.h"
#include "bms_relays.h"     /* estado de segurança + snapshot dos monitores */
#include <stdio.h>
#include <string.h>


/* =========================================================================
 * API PÚBLICA
 * ========================================================================= */
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

    char line[384];

    /* Snapshot da malha de segurança/relés (módulo bms_relays). */
    BMS_RelayMonitors_t mon;
    BMS_Relays_GetMonitors(&mon);

    /* Linha única, legível. Campos da BMS + (no fim) estado de segurança,
     * sinais dos monitores e estado dos relés. snprintf trunca em segurança. */
    int n = snprintf(line, sizeof(line),
        "[BMS] st=%s flt=%s(0x%08lX) pack=%lumV min=%umV max=%umV "
        "dV=%umV Tmax=%dC SoC=%u%% HV=%lumV ring=%s ctor=%s ok=%u pre=%u "
        "cerr=%lu crc=%lu "
        "sfty=%s imd=%u tsms=%u esdb=%u esdbC=%u chgsig=%u "
        "rly[pre=%u dis=%u chg=%u bms=%u]\r\n",
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
        hbms->contactor_closed ? "CLOSE" : "OPEN",   /* DECISÃO lógica interna */
        (unsigned)(hbms->bms_ok ? 1U : 0U),          /* interlock BMS_OK */
        (unsigned)(hbms->precharge_ready ? 1U : 0U), /* interlock PRECHARGE_OK */
        (unsigned long)hbms->comm_error_count,
        (unsigned long)hbms->crc_error_count,
        /* --- malha de segurança / relés --- */
        BMS_Relays_GetStateString(),
        (unsigned)(mon.imd_ok ? 1U : 0U),
        (unsigned)(mon.tsms ? 1U : 0U),
        (unsigned)(mon.esdb ? 1U : 0U),
        (unsigned)(mon.esdb_charger ? 1U : 0U),
        (unsigned)(mon.charger ? 1U : 0U),
        (unsigned)(mon.pre_charge_closed ? 1U : 0U),
        (unsigned)(mon.discharge_closed ? 1U : 0U),
        (unsigned)(mon.charge_relay_closed ? 1U : 0U),
        (unsigned)(mon.bms_relay_closed ? 1U : 0U));

    if (n < 0) { return; }
    uint16_t len = (n >= (int)sizeof(line)) ? (uint16_t)(sizeof(line) - 1U)
                                            : (uint16_t)n;

    /* TX bloqueante (frame curto @ 115200). Sem DMA. */
    (void)HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, len, 50U);
    hcomm->tx_count++;
}
