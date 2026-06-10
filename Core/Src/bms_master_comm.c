/**
 * @file    bms_master_comm.c
 * @brief   BMS → Master (VCU) Communication — UART3 telemetria + watchdog
 *
 *  Fluxo de operação:
 *    1. BMS_MasterComm_Init(): inicializa estrutura e arma RX IT para heartbeat
 *    2. BMS_MasterComm_Task_100ms(): serializa pacote, transmite, verifica watchdog
 *    3. BMS_MasterComm_RxCallback(): reset do watchdog ao receber heartbeat 0x55
 *    4. Se watchdog expirar → fault no handle BMS principal
 */

#include "bms_master_comm.h"
#include <string.h>

/* =========================================================================
 * FUNÇÕES PRIVADAS
 * ========================================================================= */

/**
 * @brief  Calcula checksum XOR sobre 'len' bytes
 */
uint8_t BMS_MasterComm_CalcChecksum(uint8_t *data, uint16_t len)
{
    uint8_t xor_acc = 0U;
    for (uint16_t i = 0U; i < len; i++)
    {
        xor_acc ^= data[i];
    }
    return xor_acc;
}

/**
 * @brief  Preenche o pacote de telemetria com dados actuais do BMS
 */
static void BMS_MasterComm_BuildPacket(BMS_TelemetryPacket_t *pkt,
                                        BMS_Handle_t *hbms)
{
    memset(pkt, 0, sizeof(BMS_TelemetryPacket_t));

    pkt->header[0]         = TELEMETRY_HEADER_1;
    pkt->header[1]         = TELEMETRY_HEADER_2;
    pkt->bms_state         = (uint8_t)hbms->state;
    pkt->fault_flags       = hbms->fault_flags;
    pkt->pack_voltage_mv   = hbms->pack_voltage_mv;
    pkt->min_cell_mv       = hbms->min_cell_mv;
    pkt->max_cell_mv       = hbms->max_cell_mv;
    pkt->max_temp_c        = (int8_t)hbms->max_temp_c;
    pkt->precharge_ready   = hbms->precharge_ready ? 1U : 0U;
    pkt->balance_active    = hbms->is_balancing    ? 1U : 0U;
    /* Tensão inversor em décimas de V (ex: 1134 = 113.4V) */
    pkt->inverter_voltage_v = (uint16_t)(hbms->inverter_voltage_mv / 100UL);

    /* Checksum: XOR de todos os bytes desde header até inverter_voltage_v */
    uint16_t cs_len = (uint16_t)(offsetof(BMS_TelemetryPacket_t, checksum));
    pkt->checksum = BMS_MasterComm_CalcChecksum((uint8_t *)pkt, cs_len);
    pkt->footer   = TELEMETRY_FOOTER;
}

/* =========================================================================
 * API PÚBLICA
 * ========================================================================= */

/**
 * @brief  Inicialização do módulo de comunicação com a Master
 *
 *  Arma o receptor IT para capturar o byte de heartbeat da VCU.
 *  Deve ser chamado após HAL_Init() e configuração de USART3.
 *
 * @param  hcomm    Handle do módulo de comunicação
 * @param  huart3   Handle do USART3 (distinto do USART2 do BQ79600)
 */
void BMS_MasterComm_Init(BMS_MasterComm_t *hcomm, UART_HandleTypeDef *huart3)
{
    if ((hcomm == NULL) || (huart3 == NULL)) { return; }

    memset(hcomm, 0, sizeof(BMS_MasterComm_t));
    hcomm->huart       = huart3;
    hcomm->last_hb_tick = HAL_GetTick();
    hcomm->link_ok     = false;

    /* Armar receptor para captura do heartbeat (1 byte em modo IT) */
    (void)HAL_UART_Receive_IT(hcomm->huart,
                               (uint8_t *)hcomm->hb_rx_buf, 1U);
}

/**
 * @brief  Tarefa cíclica a 100 ms — envia telemetria e verifica watchdog
 *
 *  WATCHDOG: Se não houver heartbeat da VCU em MASTER_HB_TIMEOUT_MS (500 ms),
 *  a ligação é declarada perdida (link_ok = false), o fault BMS_FAULT_COMM
 *  é assinalado e o BMS transita para FAULT + abertura de contactores.
 *
 *  TRANSMISSÃO: Packet de 22 bytes enviado de forma bloqueante (frame curto,
 *  latência < 2 ms a 115200 bps). Para sistemas RTOS, substituir por DMA TX.
 *
 * @param  hcomm    Handle do módulo de comunicação
 * @param  hbms     Handle principal do BMS (para leitura de dados e fault)
 */
void BMS_MasterComm_Task_100ms(BMS_MasterComm_t *hcomm, BMS_Handle_t *hbms)
{
    if ((hcomm == NULL) || (hbms == NULL)) { return; }

    /* ---------------------------------------------------------------
     * 1. VERIFICAÇÃO DO WATCHDOG DE HEARTBEAT
     * --------------------------------------------------------------- */
    uint32_t now_ms     = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - hcomm->last_hb_tick;

    if (elapsed_ms >= MASTER_HB_TIMEOUT_MS)
    {
        /* Heartbeat em falta há >= 500 ms: ligação perdida OU nunca estabelecida.
         * Fail-safe: declara fault em qualquer dos casos. A janela de 500 ms
         * (a contar de last_hb_tick, semeado no boot) dá margem para a VCU
         * arrancar e enviar o primeiro heartbeat. */
        if (hcomm->link_ok)
        {
            /* Conta apenas a transição ok→perdido (evita spam do contador) */
            hcomm->hb_timeout_count++;
        }
        hcomm->link_ok      = false;
        hbms->fault_flags  |= BMS_FAULT_COMM;
        hbms->state         = BMS_STATE_FAULT;
        BMS_ContactorOpen(hbms);
    }
    /* NOTA: link_ok é posto a TRUE EXCLUSIVAMENTE em BMS_MasterComm_RxCallback
     * quando um heartbeat 0x55 válido é recebido. A versão anterior tinha aqui
     * um ramo 'else' que punha link_ok=true sempre que elapsed < timeout —
     * como last_hb_tick é semeado com o tempo de boot, isso declarava a ligação
     * saudável durante os primeiros 500 ms mesmo com a VCU morta (falso positivo). */

    /* ---------------------------------------------------------------
     * 2. SERIALIZAÇÃO E ENVIO DO PACOTE DE TELEMETRIA
     * --------------------------------------------------------------- */
    BMS_TelemetryPacket_t pkt;
    BMS_MasterComm_BuildPacket(&pkt, hbms);

    (void)HAL_UART_Transmit(hcomm->huart,
                             (uint8_t *)&pkt,
                             TELEMETRY_PACKET_SIZE,
                             20U);  /* Timeout 20 ms: 22 bytes @ 115200 = ~1.9 ms */
    hcomm->tx_count++;
}

/**
 * @brief  Retorna o estado da ligação com a Master
 * @return TRUE se heartbeat recebido nos últimos MASTER_HB_TIMEOUT_MS
 */
bool BMS_MasterComm_IsLinkOk(BMS_MasterComm_t *hcomm)
{
    return (hcomm != NULL) ? hcomm->link_ok : false;
}

/**
 * @brief  Callback a chamar a partir de HAL_UART_RxCpltCallback para USART3
 *
 *  Verifica se o byte recebido é o heartbeat válido (0x55) e actualiza
 *  o timestamp. Re-arma o receptor para o próximo byte.
 *
 *  Integração no projecto:
 *    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *        BMS_MasterComm_RxCallback(&g_master_comm, huart);
 *        // ... outros handlers UART ...
 *    }
 */
void BMS_MasterComm_RxCallback(BMS_MasterComm_t *hcomm,
                                 UART_HandleTypeDef *huart)
{
    if ((hcomm == NULL) || (huart != hcomm->huart)) { return; }

    if (hcomm->hb_rx_buf[0] == MASTER_HEARTBEAT_BYTE)
    {
        hcomm->last_hb_tick = HAL_GetTick();
        hcomm->link_ok      = true;
    }
    /* Re-armar receptor independentemente da validade do byte */
    (void)HAL_UART_Receive_IT(hcomm->huart,
                               (uint8_t *)hcomm->hb_rx_buf, 1U);
}
