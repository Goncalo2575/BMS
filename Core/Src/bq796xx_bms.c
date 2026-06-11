/**
 * @file    bq796xx_bms.c
 * @brief   BMS Driver - Implementação completa
 *          STM32F446 + BQ79600-Q1 Bridge + 2x BQ79616-Q1 Slaves (15S cada)
 *
 *  ARQUITECTURA DE COMUNICAÇÃO:
 *  ┌─────────┐  UART 1Mbps   ┌──────────────┐  Daisy-Chain Isolado
 *  │ STM32F4 │◄─────────────►│ BQ79600 Bridge│◄──────────────────────┐
 *  └─────────┘               └──────────────┘                        │
 *                                    │ DIR0 →  Slave1 → Slave2        │
 *                                    └── DIR1 ← Slave1 ← Slave2 ────┘
 *                                              (Ring/Anel)
 *
 * @version 3.2.0
 */

#include "bq796xx_bms.h"

/* =========================================================================
 * VARIÁVEIS ESTÁTICAS E DECLARAÇÕES EXTERNAS DE ÂMBITO DE FICHEIRO
 * ========================================================================= */
static BMS_Handle_t    *g_hbms_irq   = NULL;

/* =========================================================================
 * SECÇÃO 1: UTILITÁRIOS - DELAY E CRC
 * ========================================================================= */

/**
 * @brief  Atraso em milissegundos usando HAL_Delay
 */
void BMS_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief  Atraso em microssegundos usando Timer de hardware em modo free-running
 *
 *  O timer (TIM2) é iniciado uma única vez em BMS_Init e esta função apenas
 *  lê o contador por subtracção. Requer 1 µs/tick (no CubeMX: PSC tal que o
 *  timer-clock de APB1 dê 1 MHz — a 84 MHz com APB1÷2 o timer corre a 84 MHz,
 *  logo PSC=83). Wraparound do contador de 32 bits a ~51 s — muito acima do
 *  delay máximo usado (40 ms no HW reset).
 */
void BMS_DelayUs(BMS_Handle_t *hbms, uint32_t us)
{
    if (hbms->htim_delay != NULL)
    {
        /* Leitura do contador free-running sem Start/Stop */
        uint32_t start = __HAL_TIM_GET_COUNTER(hbms->htim_delay);
        while ((__HAL_TIM_GET_COUNTER(hbms->htim_delay) - start) < us);
    }
    else
    {
        /* Fallback: loop calibrado (aproximado) para STM32F446 a 84 MHz.
         * Só é atingido se htim_delay==NULL — caminho não usado em operação. */
        volatile uint32_t count = us * 21U;
        while (count--);
    }
}

/**
 * @brief  Calcula CRC-16-IBM (polinómio 0x8005 reflexo = 0xA001), init 0xFFFF, LSB-first
 */
uint16_t BMS_CalculateCRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = CRC16_INIT;   /* 0xFFFF */

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8U; j++)
        {
            if (crc & 0x0001U)
            {
                crc = (crc >> 1U) ^ CRC16_POLY_LSB_FIRST;  /* 0xA001 */
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/* =========================================================================
 * SECÇÃO 2: CONSTRUÇÃO E TRANSMISSÃO DE FRAMES
 * ========================================================================= */

/**
 * @brief  Transmite um frame e recebe a resposta via DMA (Full-Duplex Async)
 */
static BMS_Status_t BMS_Transceive(BMS_Handle_t *hbms,
                                    uint8_t *tx_data, uint16_t tx_len,
                                    uint8_t *rx_data, uint16_t rx_len)
{
    HAL_StatusTypeDef hal_status;

    if ((rx_data != NULL) && (rx_len > 0U))
    {
        /* PASSO 1: Armar DMA RX antes de transmitir */
        __atomic_store_n(&hbms->dma_rx_done, 0U, __ATOMIC_SEQ_CST);

        hal_status = HAL_UART_Receive_DMA(hbms->huart, rx_data, rx_len);
        if (hal_status != HAL_OK)
        {
            hbms->comm_error_count++;
            return BMS_ERR_COMM;
        }

        /* PASSO 2: Transmitir frame de comando (bloqueante — frames curtos) */
        hal_status = HAL_UART_Transmit(hbms->huart, tx_data, tx_len,
                                        BMS_UART_TIMEOUT_MS);
        if (hal_status != HAL_OK)
        {
            HAL_UART_DMAStop(hbms->huart);
            hbms->comm_error_count++;
            return BMS_ERR_COMM;
        }

        /* PASSO 3: Aguardar flag DMA TC com timeout calculado */
        uint32_t timeout_us = (uint32_t)rx_len * 10UL + BMS_DAISY_CHAIN_LATENCY_US;
        uint32_t elapsed_us = 0U;

        while (__atomic_load_n(&hbms->dma_rx_done, __ATOMIC_SEQ_CST) == 0U)
        {
            BMS_DelayUs(hbms, 1U);
            elapsed_us++;
            if (elapsed_us >= timeout_us)
            {
                HAL_UART_DMAStop(hbms->huart);
                hbms->comm_error_count++;
                BMS_CommClear(hbms);
                return BMS_ERR_TIMEOUT;
            }
        }

        /* Verificar e limpar ORE residual */
        if (__HAL_UART_GET_FLAG(hbms->huart, UART_FLAG_ORE))
        {
            __HAL_UART_CLEAR_OREFLAG(hbms->huart);
            hbms->comm_error_count++;
            return BMS_ERR_COMM;
        }
    }
    else
    {
        /* Escrita sem resposta: TX bloqueante simples */
        hal_status = HAL_UART_Transmit(hbms->huart, tx_data, tx_len,
                                        BMS_UART_TIMEOUT_MS);
        if (hal_status != HAL_OK)
        {
            hbms->comm_error_count++;
            return BMS_ERR_COMM;
        }
    }

    return BMS_OK;
}

/**
 * @brief  Callback HAL para a recepção DMA do BQ79600 (UART4)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* UART4: BQ79600 DMA Transfer Complete.
     * (A telemetria de debug em USART2 é TX-only — sem recepção IT.) */
    if ((g_hbms_irq != NULL) && (huart == g_hbms_irq->huart))
    {
        __atomic_store_n(&g_hbms_irq->dma_rx_done, 1U, __ATOMIC_SEQ_CST);
    }
}

/**
 * @brief  Escrita Single Device (com DEV_ADR no frame)
 *         Formato: INIT(1) + DEV_ADR(1) + REG_ADR(2) + DATA(n) + CRC(2)
 */
BMS_Status_t BMS_WriteSingleDevice(BMS_Handle_t *hbms, uint8_t dev_addr,
                                    uint16_t reg_addr, uint8_t *data,
                                    uint8_t data_len)
{
    uint8_t  frame[BMS_MAX_FRAME_SIZE];
    uint16_t idx = 0U;
    uint16_t crc;

    if ((data == NULL) || (data_len == 0U) || (data_len > 8U) ||
        (data_len > (BMS_MAX_FRAME_SIZE - 6U)))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    frame[idx++] = BMS_INIT(INIT_BASE_SINGLE_WRITE, data_len);
    frame[idx++] = dev_addr & 0x7FU;
    frame[idx++] = (uint8_t)(reg_addr >> 8U);
    frame[idx++] = (uint8_t)(reg_addr & 0xFFU);
    for (uint8_t i = 0U; i < data_len; i++)
    {
        frame[idx++] = data[i];
    }
    crc = BMS_CalculateCRC16(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFFU);
    frame[idx++] = (uint8_t)(crc >> 8U);

    return BMS_Transceive(hbms, frame, idx, NULL, 0U);
}

/**
 * @brief  Leitura Single Device
 *    TX: [INIT_BASE_SINGLE_READ][DEV][REG_H][REG_L][(N-1)][CRC_L][CRC_H] = 7 bytes
 */
BMS_Status_t BMS_ReadSingleDevice(BMS_Handle_t *hbms, uint8_t dev_addr,
                                   uint16_t reg_addr, uint8_t *rx_data,
                                   uint8_t data_len)
{
    uint8_t  tx_frame[7U];
    uint8_t  rx_frame[BMS_MAX_RESPONSE_PAYLOAD];
    uint16_t tx_idx = 0U;
    uint16_t crc_calc, crc_recv;
    BMS_Status_t status;

    if ((rx_data == NULL) || (data_len == 0U))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    uint16_t rx_total = (uint16_t)(1U + 1U + 2U + data_len + 2U);
    if (rx_total > (uint16_t)BMS_MAX_RESPONSE_PAYLOAD)
    {
        return BMS_ERR_INVALID_PARAM;
    }

    tx_frame[tx_idx++] = INIT_BASE_SINGLE_READ;           /* 0x90, SIZE=000 */
    tx_frame[tx_idx++] = dev_addr & 0x7FU;
    tx_frame[tx_idx++] = (uint8_t)(reg_addr >> 8U);
    tx_frame[tx_idx++] = (uint8_t)(reg_addr & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(data_len - 1U);        /* Payload: bytes a ler - 1 */

    crc_calc = BMS_CalculateCRC16(tx_frame, tx_idx);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc >> 8U);

    status = BMS_Transceive(hbms, tx_frame, tx_idx, rx_frame, rx_total);
    if (status != BMS_OK) { return status; }

    crc_calc = BMS_CalculateCRC16(rx_frame, rx_total - 2U);
    crc_recv = (uint16_t)rx_frame[rx_total - 2U] |
               ((uint16_t)rx_frame[rx_total - 1U] << 8U);
    if (crc_calc != crc_recv)
    {
        hbms->crc_error_count++;
        return BMS_ERR_CRC;
    }

    for (uint8_t i = 0U; i < data_len; i++)
    {
        rx_data[i] = rx_frame[4U + i];
    }
    return BMS_OK;
}

/**
 * @brief  Escrita Broadcast (sem DEV_ADR)
 */
BMS_Status_t BMS_WriteBroadcast(BMS_Handle_t *hbms, uint16_t reg_addr,
                                 uint8_t *data, uint8_t data_len)
{
    uint8_t  frame[BMS_MAX_FRAME_SIZE];
    uint16_t idx = 0U;
    uint16_t crc;

    if ((data == NULL) || (data_len == 0U) || (data_len > 8U) ||
        (data_len > (BMS_MAX_FRAME_SIZE - 5U)))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    frame[idx++] = BMS_INIT(INIT_BASE_BROADCAST_WRITE, data_len);
    frame[idx++] = (uint8_t)(reg_addr >> 8U);
    frame[idx++] = (uint8_t)(reg_addr & 0xFFU);
    for (uint8_t i = 0U; i < data_len; i++)
    {
        frame[idx++] = data[i];
    }
    crc = BMS_CalculateCRC16(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFFU);
    frame[idx++] = (uint8_t)(crc >> 8U);

    return BMS_Transceive(hbms, frame, idx, NULL, 0U);
}

/**
 * @brief  Leitura Broadcast
 *    TX: [INIT_BASE_BROADCAST_READ][REG_H][REG_L][(N-1)][CRC_L][CRC_H] = 6 bytes
 */
BMS_Status_t BMS_ReadBroadcast(BMS_Handle_t *hbms, uint16_t reg_addr,
                                uint8_t *rx_data, uint8_t data_len_per_dev)
{
    uint8_t  tx_frame[6U];
    uint16_t tx_idx = 0U;
    uint16_t crc_calc;
    uint16_t per_dev_size = (uint16_t)(1U + 1U + 2U + (uint16_t)data_len_per_dev + 2U);
    uint16_t rx_total     = (uint16_t)(BMS_NUM_SLAVES * per_dev_size);
    uint8_t  rx_raw[BMS_MAX_RESPONSE_PAYLOAD];
    BMS_Status_t status;

    if ((rx_data == NULL) || (data_len_per_dev == 0U))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    tx_frame[tx_idx++] = INIT_BASE_BROADCAST_READ;             /* 0x80, SIZE=000 */
    tx_frame[tx_idx++] = (uint8_t)(reg_addr >> 8U);
    tx_frame[tx_idx++] = (uint8_t)(reg_addr & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(data_len_per_dev - 1U);     /* Payload: N-1 */

    crc_calc = BMS_CalculateCRC16(tx_frame, tx_idx);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc >> 8U);

    status = BMS_Transceive(hbms, tx_frame, tx_idx, rx_raw, rx_total);
    if (status != BMS_OK) { return status; }

    uint16_t offset = 0U;
    for (uint8_t wire_idx = 0U; wire_idx < BMS_NUM_SLAVES; wire_idx++)
    {
        uint8_t logical_idx;
        if (hbms->ring_using_reverse)
        {
            logical_idx = wire_idx;  /* DIR1: ordem directa */
        }
        else
        {
            logical_idx = (uint8_t)(BMS_NUM_SLAVES - 1U - wire_idx);  /* DIR0: invertido */
        }

        crc_calc = BMS_CalculateCRC16(&rx_raw[offset], per_dev_size - 2U);
        uint16_t crc_recv = (uint16_t)rx_raw[offset + per_dev_size - 2U] |
                            ((uint16_t)rx_raw[offset + per_dev_size - 1U] << 8U);
        if (crc_calc != crc_recv)
        {
            hbms->slave[logical_idx].comm_ok = false;
            hbms->crc_error_count++;
            return BMS_ERR_CRC;
        }
        for (uint8_t b = 0U; b < data_len_per_dev; b++)
        {
            rx_data[logical_idx * data_len_per_dev + b] = rx_raw[offset + 4U + b];
        }
        hbms->slave[logical_idx].comm_ok = true;
        offset += per_dev_size;
    }
    return BMS_OK;
}

/**
 * @brief  Escrita Broadcast Reversa para configuração do caminho DIR1
 */
BMS_Status_t BMS_WriteBroadcastReverse(BMS_Handle_t *hbms, uint16_t reg_addr,
                                        uint8_t init_base, uint8_t *data,
                                        uint8_t data_len)
{
    uint8_t  frame[BMS_MAX_FRAME_SIZE];
    uint16_t idx = 0U;
    uint16_t crc;

    if ((data == NULL) || (data_len == 0U) || (data_len > 8U))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    frame[idx++] = BMS_INIT(init_base, data_len);
    frame[idx++] = (uint8_t)(reg_addr >> 8U);
    frame[idx++] = (uint8_t)(reg_addr & 0xFFU);
    for (uint8_t i = 0U; i < data_len; i++)
    {
        frame[idx++] = data[i];
    }
    crc = BMS_CalculateCRC16(frame, idx);
    frame[idx++] = (uint8_t)(crc & 0xFFU);
    frame[idx++] = (uint8_t)(crc >> 8U);

    return BMS_Transceive(hbms, frame, idx, NULL, 0U);
}

/* =========================================================================
 * SECÇÃO 3: SEQUÊNCIA DE INICIALIZAÇÃO COMPLETA
 * ========================================================================= */

/**
 * @brief  Pulso WAKE - força TX (PA0) ao nível LOW para acordar a Bridge
 */
static void BMS_SendWakePulse(BMS_Handle_t *hbms)
{
    HAL_UART_DeInit(hbms->huart);

    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin = BMS_BRIDGE_TX_PIN;       /* PA0 = UART4_TX */
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &gpio_cfg);

    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);  /* LOW */
    BMS_DelayUs(hbms, DELAY_WAKE_PULSE_US);                /* 2500 µs */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);    /* HIGH */

    HAL_UART_Init(hbms->huart);
    BMS_DelayMs(DELAY_OSC_STAB_MS);  /* 2 ms para osciladores da bridge */
}

/**
 * @brief  Sincronização DLL - 8 dummy stack writes para ECC_DATA1..8
 */
static BMS_Status_t BMS_SyncDLL(BMS_Handle_t *hbms)
{
    uint8_t dummy = 0x00U;
    BMS_Status_t status = BMS_OK;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        status = BMS_WriteBroadcast(hbms, REG_ECC_DATA1 + i, &dummy, 1U);
        if (status != BMS_OK)
        {
            return status;
        }
    }
    return BMS_OK;
}

/**
 * @brief  Endereçamento automático da rede daisy-chain
 */
BMS_Status_t BMS_AutoAddressing(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t data;

    /* PASSO 1: Hardware Reset - Pulso WAKE */
    BMS_SendWakePulse(hbms);

    /* PASSO 2: WAKE Tone para a stack de slaves */
    data = CTRL1_SEND_WAKE;  /* 0x20 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }
    BMS_DelayMs(DELAY_WAKE_PROPAGATION_MS);  /* 5 ms - propagação obrigatória */

    /* PASSO 3: Sincronização DLL */
    status = BMS_SyncDLL(hbms);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* PASSO 4: Endereçamento do Caminho Principal (DIR_SEL = 0) */
    data = CTRL1_ADDR_WR;  /* 0x01 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = BMS_ADDR_BRIDGE;  /* 0x00 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_DIR0_ADDR, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = BMS_ADDR_SLAVE1;  /* 0x01 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_DIR0_ADDR, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = BMS_ADDR_SLAVE2;  /* 0x02 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_DIR0_ADDR, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* PASSO 5: Definição de Papéis no Caminho Principal */
    data = COMM_CTRL_STACK_DEV;  /* 0x02 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_COMM_CTRL, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = COMM_CTRL_TOP_STACK;  /* 0x03 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_SLAVE2,
                                    REG_BRIDGE_COMM_CTRL, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* PASSO 6: Configuração do Anel (Caminho Reverso DIR_SEL = 1) */
    data = CTRL1_DIR_SEL;  /* 0x02 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }
    BMS_DelayUs(hbms, DELAY_DIR_SEL_SWITCH_US);  /* 100 µs */

    data = 0x80U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_CONTROL1,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* PASSO 7: Endereçamento do Caminho Reverso */
    data = 0x81U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_CONTROL1,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = BMS_ADDR_BRIDGE;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_DIR1_ADDR,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = 0x01U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_DIR1_ADDR,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = 0x02U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_DIR1_ADDR,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    data = COMM_CTRL_TOP_STACK;  /* 0x03 */
    status = BMS_WriteSingleDevice(hbms, 0x02U,
                                    REG_BRIDGE_COMM_CTRL, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Regista endereços no handle */
    hbms->slave[0].address     = BMS_ADDR_SLAVE1;
    hbms->slave[0].address_rev = 0x02U;
    hbms->slave[1].address     = BMS_ADDR_SLAVE2;
    hbms->slave[1].address_rev = 0x01U;
    hbms->ring_intact          = true;

    /* SANEAMENTO PÓS-ENDEREÇAMENTO (TI ref: AutoAddress() post-sequence) */
    {
        uint8_t dummy_rx[BMS_NUM_SLAVES];
        for (uint8_t i = 0U; i < 8U; i++)
        {
            (void)BMS_ReadBroadcast(hbms, REG_ECC_DATA1 + i, dummy_rx, 1U);
        }
        BMS_DelayMs(2U);

        uint8_t clear_comm = 0x03U;   /* bit0=COMM_ERR1, bit1=COMM_ERR2 */
        (void)BMS_WriteBroadcast(hbms, REG_FAULT_RST2, &clear_comm, 1U);
        (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_FAULT_RST2, &clear_comm, 1U);
    }

    return BMS_OK;
}

/**
 * @brief  Configura ambos os slaves para 15 células
 */
BMS_Status_t BMS_ConfigureSlaves(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t data;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        uint8_t slave_addr = hbms->slave[s].address;

        data = ACTIVE_CELL_15S;  /* 0x0F */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_ACTIVE_CELL, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = OV_THRESH_VAL;  /* 0x24 → 3600 mV */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_OV_THRESH, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = UV_THRESH_VAL;  /* 0x24 ≈ 3000 mV */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_UV_THRESH, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = 0x02U;  /* LPF corner frequency padrão */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_ADC_CONF1, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = OVUV_CTRL_ENABLE;  /* 0x06 */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_OVUV_CTRL, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = BAL_CTRL1_TIMER_10MIN;
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_BAL_CTRL1, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = BAL_CTRL2_AUTOSTOP_EN;
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_BAL_CTRL2, &data, 1U);
        if (status != BMS_OK) { return status; }

        {
            uint8_t zero[2U] = {0x00U, 0x00U};
            status = BMS_WriteSingleDevice(hbms, slave_addr,
                                            REG_CB_CELL1_CTRL, zero, 2U);
            if (status != BMS_OK) { return status; }
        }

        data = ADC_CTRL1_CONTINUOUS_LPF;  /* 0x2E */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_ADC_CTRL1, &data, 1U);
        if (status != BMS_OK) { return status; }
    }

    BMS_DelayMs(DELAY_ADC_SETTLE_MS);
    return BMS_OK;
}

/**
 * @brief  Inicialização completa do BMS
 */
BMS_Status_t BMS_Init(BMS_Handle_t *hbms, UART_HandleTypeDef *huart,
                       TIM_HandleTypeDef *htim)
{
    BMS_Status_t status;

    if ((hbms == NULL) || (huart == NULL))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    memset(hbms, 0, sizeof(BMS_Handle_t));

    hbms->huart        = huart;
    hbms->htim_delay   = htim;
    hbms->state        = BMS_STATE_INITIALIZING;
    hbms->ring_intact  = false;
    hbms->nfault_pending = 0U;

    /* Iniciar timer em modo free-running (1 µs/tick — ver PSC no CubeMX) */
    if (hbms->htim_delay != NULL)
    {
        HAL_TIM_Base_Start(hbms->htim_delay);
    }

    g_hbms_irq = hbms;

    status = BMS_AutoAddressing(hbms);
    if (status != BMS_OK)
    {
        hbms->state = BMS_STATE_FAULT;
        hbms->fault_flags |= BMS_FAULT_COMM;
        return BMS_ERR_INIT_FAILED;
    }

    status = BMS_ConfigureSlaves(hbms);
    if (status != BMS_OK)
    {
        hbms->state = BMS_STATE_FAULT;
        return BMS_ERR_INIT_FAILED;
    }

    hbms->state = BMS_STATE_MONITORING;
    return BMS_OK;
}

/* =========================================================================
 * GESTÃO DE ENERGIA — PULSOS DE CONTROLO DE ESTADO (linha TX PA0)
 * ========================================================================= */

void BMS_SendShutdownPulse(BMS_Handle_t *hbms)
{
    HAL_UART_DeInit(hbms->huart);

    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin = BMS_BRIDGE_TX_PIN;          /* PA0 = UART4_TX */
    gpio_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull  = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &gpio_cfg);

    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);  /* LOW */
    BMS_DelayUs(hbms, DELAY_SHUTDOWN_PULSE_US);             /* 9 000 µs */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);    /* HIGH */

    HAL_UART_Init(hbms->huart);
    hbms->state = BMS_STATE_SLEEP;
}

void BMS_SendHardwareReset(BMS_Handle_t *hbms)
{
    HAL_UART_DeInit(hbms->huart);

    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin = BMS_BRIDGE_TX_PIN;
    gpio_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull  = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &gpio_cfg);

    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);
    BMS_DelayUs(hbms, DELAY_HWRESET_PULSE_US);              /* 40 000 µs */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);

    HAL_UART_Init(hbms->huart);
    hbms->state = BMS_STATE_UNINITIALIZED;   /* Requer BMS_Init() após reset */
}

void BMS_EnterSleep(BMS_Handle_t *hbms)
{
    (void)BMS_StopAllBalancing(hbms);

    BMS_ContactorOpen(hbms);
    BMS_DelayMs(DELAY_SLEEP_CONTACTOR_MS);  /* Aguardar abertura mecânica (no master) */

    BMS_SendShutdownPulse(hbms);
    /* state = BMS_STATE_SLEEP (set em BMS_SendShutdownPulse) */
}

/* =========================================================================
 * HAL_GPIO_EXTI_Callback — Captura NFAULT (EXTI8, PA8, Falling Edge)
 * =========================================================================
 * g_hbms_irq é static neste ficheiro — daí o callback residir aqui.
 * PA8 → linha EXTI8 → vector EXTI9_5_IRQHandler (gerado pelo CubeMX). */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMS_NFAULT_PIN)   /* PA8 */
    {
        if (g_hbms_irq != NULL)
        {
            BMS_NFAULT_IRQHandler(g_hbms_irq);
        }
    }
}

/* =========================================================================
 * SECÇÃO: WATCHDOG INDEPENDENTE (IWDG) — corre do LSI, independente do SYSCLK
 * ========================================================================= */

void BMS_IWDG_Init(void)
{
    IWDG_HandleTypeDef hiwdg = {0};
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = BMS_IWDG_PRESCALER;   /* IWDG_PRESCALER_64 */
    hiwdg.Init.Reload    = BMS_IWDG_RELOAD;       /* 250 → ~500 ms */
    (void)HAL_IWDG_Init(&hiwdg);
}

void BMS_IWDG_Refresh(void)
{
    IWDG_HandleTypeDef hiwdg = {0};
    hiwdg.Instance = IWDG;
    (void)HAL_IWDG_Refresh(&hiwdg);
}

/* =========================================================================
 * SECÇÃO: COMM CLEAR (Reset da State Machine do Receptor UART)
 * ========================================================================= */

void BMS_CommClear(BMS_Handle_t *hbms)
{
    HAL_UART_DeInit(hbms->huart);

    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin = BMS_BRIDGE_TX_PIN;   /* PA0 = UART4_TX */
    gpio_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull  = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &gpio_cfg);

    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);
    BMS_DelayUs(hbms, BMS_COMM_CLEAR_BITS);   /* 18 µs @ 1 Mbps */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);

    HAL_UART_Init(hbms->huart);
    BMS_DelayUs(hbms, 50U);
}

/* =========================================================================
 * SECÇÃO: POWER-ON SELF TEST (POST)
 * ========================================================================= */

BMS_Status_t BMS_PowerOnSelfTest(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t test_buf[2U];

    /* TESTE 1: Integridade do algoritmo CRC ("123456789" → 0x4B37) */
    {
        uint8_t crc_test[] = {'1','2','3','4','5','6','7','8','9'};
        uint16_t crc = BMS_CalculateCRC16(crc_test, 9U);
        if (crc != 0x4B37U)
        {
            hbms->fault_flags |= BMS_FAULT_CRC;
            return BMS_ERR_INIT_FAILED;
        }
    }

    /* TESTE 2: Comunicação com cada slave (lê ACTIVE_CELL, espera 0x0F) */
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        status = BMS_ReadSingleDevice(hbms, hbms->slave[s].address,
                                       REG_ACTIVE_CELL, test_buf, 1U);
        if (status != BMS_OK)
        {
            hbms->fault_flags |= BMS_FAULT_COMM;
            return BMS_ERR_INIT_FAILED;
        }
        if (test_buf[0] != ACTIVE_CELL_15S)
        {
            hbms->fault_flags |= BMS_FAULT_COMM;
            return BMS_ERR_INIT_FAILED;
        }
    }

    /* TESTE 3: Sanidade do ADC (1000..4500 mV por célula) */
    status = BMS_ReadAllCellVoltages(hbms);
    if (status != BMS_OK)
    {
        return BMS_ERR_INIT_FAILED;
    }
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t mv = hbms->slave[s].cell_voltage_mv[c];
            if ((mv < 1000U) || (mv > 4500U))
            {
                hbms->fault_flags |= BMS_FAULT_OPEN_WIRE;
                return BMS_ERR_INIT_FAILED;
            }
        }
    }

    /* TESTE 4: NFAULT (PA8) deve estar HIGH (sem faults residuais) */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }

    hbms->post_passed = true;
    return BMS_OK;
}
