/**
 * @file    bms_master_comm.c
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 */

#include "bms_master_comm.h"
#include "bms_relays.h"     
#include <stdio.h>
#include <string.h>


/**
 * @brief  Inicializa a saída de debug/telemetria (USART2)
 */

void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart2)
{
    if ((hcomm == NULL) || (huart2 == NULL)) { return; }

    hcomm->huart    = huart2;
    hcomm->tx_count = 0U;
    
}

/**
 * @brief  Compõe e envia uma linha única de telemetria/debug pelo USART2
 */

void BMS_MasterComm_PrintDebug(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms)
{
    if ((hcomm == NULL) || (hbms == NULL) || (hcomm->huart == NULL)) { return; }

    char line[384];

    BMS_RelayMonitors_t mon;
    BMS_Relays_GetMonitors(&mon);

    int n = snprintf(line, sizeof(line),
        "[BMS] st=%s flt=%s(0x%08lX) pack=%lumV min=%umV max=%umV "
        "dV=%umV Tmax=%dC SoC=%u%% HV=%lumV ring=%s ctor=%s ok=%u pre=%u "
        "cerr=%lu crc=%lu "
        "sfty=%s imd=%u tsms=%u esdb=%u esdbC=%u chgsig=%u "
        "rly[pre=%u dis=%u chg=%u bms=%u]\r\n",

        /* --- coisas que vêm dos BQs --- */
        BMS_GetStateString(hbms->state),                //state
        BMS_GetFaultString(hbms->fault_flags),          //fault_flags
        (unsigned long)hbms->fault_flags,           
        (unsigned long)hbms->pack_voltage_mv,           //pack_voltage_mv
        (unsigned)hbms->min_cell_mv,                    //min_cell_mv
        (unsigned)hbms->max_cell_mv,                    //max_cell_mv
        (unsigned)hbms->delta_cell_mv,                  //delta_cell_mv
        (int)hbms->max_temp_c,                          //max_temp_c
        (unsigned)hbms->soc_percent,                    //soc_percent
        (unsigned long)hbms->inverter_voltage_mv,       //inverter_voltage_mv (B+)
        hbms->ring_intact ? "OK" : "BROKEN",            //ring_intact
        hbms->contactor_closed ? "CLOSE" : "OPEN",      // DECISÃO lógica interna //////////////achoq ue se pode tirar///////////////////
        (unsigned)(hbms->bms_ok ? 1U : 0U),             // interlock BMS_OK
        (unsigned)(hbms->precharge_ready ? 1U : 0U),    // interlock PRECHARGE_OK
        (unsigned long)hbms->comm_error_count,          // comm_error_count
        (unsigned long)hbms->crc_error_count,           // crc_error_count



        /* --- malha de segurança / relés --- */
        BMS_Relays_GetStateString(),                        // estado de segurança (SAFE/ENGAGED/CHARGING/NOT_SAFE)
        (unsigned)(mon.imd_ok ? 1U : 0U),                   // imd ok
        (unsigned)(mon.tsms ? 1U : 0U),                     // tsms ok
        (unsigned)(mon.esdb ? 1U : 0U),                     // esdb ok
        (unsigned)(mon.esdb_charger ? 1U : 0U),             // esdb charger ok
        (unsigned)(mon.charger ? 1U : 0U),                  // charger ok
        (unsigned)(mon.pre_charge_closed ? 1U : 0U),        // pre-charge relay aberto/fechado
        (unsigned)(mon.discharge_closed ? 1U : 0U),         // discharge relay aberto/fechado
        (unsigned)(mon.charge_relay_closed ? 1U : 0U),      // charge relay aberto/fechado
        (unsigned)(mon.bms_relay_closed ? 1U : 0U));        // BMS relay aberto/fechado

    if (n < 0) { return; }
    uint16_t len = (n >= (int)sizeof(line)) ? (uint16_t)(sizeof(line) - 1U)
                                            : (uint16_t)n;

    
    (void)HAL_UART_Transmit(hcomm->huart, (uint8_t *)line, len, 50U);
    hcomm->tx_count++;
}
