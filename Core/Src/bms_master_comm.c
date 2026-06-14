/**
 * @file    bms_master_comm.c
 * @brief   BMS → Saída de DEBUG/Telemetria (USART2, PA2/PA3) — TX-only
 *
 *  Arquitectura CENTRALIZADA (v3.3): este STM32F446 decide, ACTUA os relés
 *  (via bms_relays) e reporta o estado a um terminal/registador por uma linha
 *  de texto legível a 1 Hz (+ reporte por evento).
 *
 *  A linha inclui o estado do BMS, as decisões lógicas (contactor/BMS_OK/
 *  precharge) e, no fim, o estado da máquina de segurança (SAFE/ENGAGED/
 *  CHARGING/NOT_SAFE), os sinais dos monitores (IMD/TSMS/ESDB/ESDB_charger/
 *  charger) e o estado real dos relés — vindos do módulo bms_relays.
 *
 *  ⚠ Esta linha periódica NÃO é o canal de segurança de baixa latência: a
 *  abertura crítica dos relés é feita localmente por bms_relays.c. Ver a NOTA
 *  DE SEGURANÇA em bq796xx_bms_monitor.c (Secção 9).
 *
 * @version 3.3.0
 */

#include "bms_master_comm.h"
#include "bms_relays.h"     /* estado de segurança + snapshot dos monitores */
#include <stdio.h>
#include <string.h>


/* =========================================================================
 * API PÚBLICA
 * ========================================================================= */

/**
 * @brief  Inicializa a saída de debug/telemetria (USART2)
 *
 * Para que serve: guarda o handle do USART2 e zera o contador de linhas. NÃO
 * arma qualquer recepção (TX-only) — o PA3/RX fica livre. Chamar uma vez após
 * MX_USART2_UART_Init().
 */
void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart2)
{
    if ((hcomm == NULL) || (huart2 == NULL)) { return; }

    hcomm->huart    = huart2;
    hcomm->tx_count = 0U;
    /* TX-only: nada de HAL_UART_Receive_IT. PA3 (RX) fica livre. */
}

/**
 * @brief  Compõe e envia uma linha única de telemetria/debug pelo USART2
 *
 * Para que serve: dá visibilidade externa do sistema num formato legível por
 * humanos e fácil de fazer parse por uma máquina. A linha leva, por esta ordem:
 *   - estado do BMS, falha activa, tensões pack/min/max/delta, Tmax, SoC, HV;
 *   - decisões lógicas: ring, contactor (decisão), BMS_OK, PRECHARGE_OK;
 *   - contadores de erro (comm/crc);
 *   - (NO FIM, para não partir parsers antigos) estado de segurança, sinais dos
 *     monitores e estado real dos relés, obtidos via BMS_Relays_GetMonitors().
 *
 * Como funciona: snprintf para um buffer de 384 B (trunca em segurança), depois
 * TX bloqueante a 115200 (frame curto, sem DMA). É chamada a 1 Hz e por evento
 * (quando a decisão de contactor ou o BMS_OK mudam — ver main_bms_app.c).
 *
 * NOTA de parsing: o campo 'ctor' é a DECISÃO lógica do BMS destinada ao
 * inversor por CAN (a implementar) — não comanda o contactor principal aqui.
 * Em CHARGING é sempre 'OPEN' (separação total tração/carga). O campo
 * 'rly[bms=..]' é o estado FÍSICO real do BMS_relay de segurança (0 = latch
 * aberto por falha ou estado inactivo) e pode estar a 1 mesmo durante o
 * carregamento (o relé de segurança não é o contactor principal).
 */
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
