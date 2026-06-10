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
 * @version 2.0.0
 */

#include "bq796xx_bms.h"
#include "bms_master_comm.h"

/* =========================================================================
 * VARIÁVEIS ESTÁTICAS E DECLARAÇÕES EXTERNAS DE ÂMBITO DE FICHEIRO
 * =========================================================================
 * OBS-02: Declarações extern movidas para âmbito de ficheiro.
 * Declarar extern dentro de uma função não é erro em C, mas é má prática:
 *   - o compilador reavalia a ligação em cada invocação da função (ISR)
 *   - dificulta a análise estática (linters, MISRA C Rule 8.5)
 *   - oculta dependências que devem ser visíveis ao nível do módulo
 * Aqui: g_hbms_irq é definido neste ficheiro (static); g_master_comm é
 * definido em main_bms_app.c e partilhado pelo callback unificado. */
static BMS_Handle_t    *g_hbms_irq   = NULL;
extern BMS_MasterComm_t g_master_comm;

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
 *  D9 CORRIGIDO: a versão anterior chamava HAL_TIM_Base_Start/Stop em cada
 *  invocação. No polling loop do DMA (~1 chamada/µs × centenas de iterações),
 *  cada par Start/Stop adicionava ~2-5 µs de overhead ao registo CR1 do timer,
 *  distorcendo a contagem de elapsed_us e causando timeouts prematuros.
 *
 *  Solução: o timer deve ser iniciado uma única vez (em BMS_Init, ou via
 *  CubeMX com auto-start), e esta função apenas lê o contador via subtracção.
 *  Com TIM2 de 32 bits (ARR=0xFFFFFFFF) e prescaler para 1 µs/tick,
 *  o wraparound ocorre a cada ~71 minutos — muito acima do delay máximo.
 *
 *  NOTA: Garantir que HAL_TIM_Base_Start(htim2) é chamado no BMS_Init ou
 *        que o timer é configurado com auto-start no CubeMX.
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
        /* Fallback: loop calibrado para STM32F446 a 180 MHz */
        volatile uint32_t count = us * 45U;
        while (count--);
    }
}

/**
 * @brief  Calcula CRC-16-IBM (polinómio 0x8005 reflexo = 0xA001)
 *         Inicialização: 0xFFFF | UART LSB-first
 *
 * @param  data     Ponteiro para os dados
 * @param  length   Comprimento em bytes
 * @return CRC calculado (16 bits)
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
                /* Polinómio x^16 + x^15 + x^2 + 1, formato LSB-first reflexo */
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
 *
 *  TOPOLOGIA FÍSICA: Full-Duplex Standard Asynchronous (PA0=TX, PA1=RX, UART4)
 *  Configuração CubeMX obrigatória:
 *    UART4 -> Mode: Asynchronous  (NÃO Half-Duplex — ver nota abaixo)
 *    DMA RX -> DMA2 Stream5, Normal, Low priority, Byte
 *    DMA TX -> DMA1 Stream6, Normal, Low priority, Byte
 *    NVIC   -> DMA RX TC interrupt habilitado
 *
 *  NOTA HALF-DUPLEX vs FULL-DUPLEX:
 *    Em Half-Duplex nativo do STM32, TX e RX são multiplexados no mesmo pino
 *    físico. O diagrama de hardware do projecto usa PA0 (TX) e PA1 (RX) — UART4
 *    separados -> Full-Duplex obrigatório. Configurar Half-Duplex desactiva
 *    o receptor em PA1 e quebra toda a comunicação.
 *
 *  PORQUÊ DMA E NÃO _IT:
 *    A 1 Mbps, receber 128 bytes via _IT gera 128 interrupções em ~1.2 ms,
 *    saturando o CPU. O DMA transfere o bloco completo para SRAM com uma
 *    única interrupção Transfer Complete (TC), libertando o CPU durante a
 *    transferência.
 *
 *  MECANISMO ASSÍNCRONO (sem polling bloqueante):
 *    O BQ79600 só envia a resposta após receber o frame completo do MCU.
 *    O fluxo correcto é:
 *      1. Armar DMA RX (receptor pronto antes de qualquer byte chegar)
 *      2. Transmitir via TX (bloqueante — frame curto, < 10 bytes)
 *      3. Aguardar flag dma_rx_done (set na TC ISR) com timeout calculado
 *    O "wait" é um polling leve de uma flag em SRAM, não de registo USART.
 *
 * @param  hbms     Handle do BMS
 * @param  tx_data  Frame a transmitir
 * @param  tx_len   Comprimento TX
 * @param  rx_data  Buffer de recepção (NULL se escrita sem resposta)
 * @param  rx_len   Bytes esperados (0 se sem resposta)
 * @return BMS_OK ou código de erro
 */
static BMS_Status_t BMS_Transceive(BMS_Handle_t *hbms,
                                    uint8_t *tx_data, uint16_t tx_len,
                                    uint8_t *rx_data, uint16_t rx_len)
{
    HAL_StatusTypeDef hal_status;

    if ((rx_data != NULL) && (rx_len > 0U))
    {
        /* -----------------------------------------------------------------
         * PASSO 1: Armar DMA RX antes de transmitir
         * HAL_UART_Receive_DMA configura o canal DMA e habilita o receptor.
         * Uma única interrupção Transfer Complete (TC) sinaliza o fim.
         * Ao contrário de _IT (1 ISR/byte), DMA não consome CPU durante RX.
         * ----------------------------------------------------------------- */
        __atomic_store_n(&hbms->dma_rx_done, 0U, __ATOMIC_SEQ_CST);

        hal_status = HAL_UART_Receive_DMA(hbms->huart, rx_data, rx_len);
        if (hal_status != HAL_OK)
        {
            hbms->comm_error_count++;
            return BMS_ERR_COMM;
        }

        /* -----------------------------------------------------------------
         * PASSO 2: Transmitir frame de comando (bloqueante — frames curtos)
         * O BQ79600 não transmite resposta enquanto estiver a receber,
         * portanto o DMA RX ficará inactivo durante a TX — sem overrun.
         * ----------------------------------------------------------------- */
        hal_status = HAL_UART_Transmit(hbms->huart, tx_data, tx_len,
                                        BMS_UART_TIMEOUT_MS);
        if (hal_status != HAL_OK)
        {
            HAL_UART_DMAStop(hbms->huart);
            hbms->comm_error_count++;
            return BMS_ERR_COMM;
        }

        /* -----------------------------------------------------------------
         * PASSO 3: Aguardar flag DMA TC (set em HAL_UART_RxCpltCallback)
         * BUG-05 CORRIGIDO: margem de 200 µs era insuficiente para latência
         * de propagação na rede isolada de 3 ICs (estimativa ~900 µs).
         * BMS_DAISY_CHAIN_LATENCY_US = 2000 µs: conservador e seguro.
         * ----------------------------------------------------------------- */
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
                /* Destravar a state machine UART dos slaves após timeout.
                 * Sem COMM CLEAR, um frame parcialmente recebido pelo BQ79600/BQ79616
                 * deixa o receptor num estado intermédio — os próximos frames
                 * serão rejeitados como FMT_ERR até um reset explícito. */
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
 * @brief  Callback HAL unificado — router para UART4 (bridge) e USART3 (VCU)
 *
 *  Este callback substitui a implementação __weak do HAL e serve
 *  dois periféricos em simultâneo:
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │ UART4  (BQ79600 DMA RX)    → sinaliza dma_rx_done = true      │
 *  │ USART3 (Master/VCU HB IT)  → delega para BMS_MasterComm_RxCB  │
 *  └─────────────────────────────────────────────────────────────────┘
 *
 *  ROUTING: o ponteiro huart identifica univocamente o periférico.
 *  Não usar if-else encadeados — ambos os blocos são independentes
 *  para que um callback inválido (outro periférico UART) seja ignorado
 *  sem afectar o estado dos dois drivers.
 *
 *  Integração CubeMX:
 *    O CubeMX gera HAL_UART_RxCpltCallback como __weak.
 *    NÃO definir este símbolo no user code gerado — o linker selecciona
 *    esta implementação forte automaticamente.
 *    Para projectos RTOS: mover para uma tarefa dedicada com notificação
 *    por semáforo a partir desta ISR.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* --- 1. UART4: BQ79600 DMA Transfer Complete --- */
    if ((g_hbms_irq != NULL) && (huart == g_hbms_irq->huart))
    {
        __atomic_store_n(&g_hbms_irq->dma_rx_done, 1U, __ATOMIC_SEQ_CST);
    }

    /* --- 2. USART3: Master/VCU Heartbeat (IT, 1 byte) --- */
    if ((g_master_comm.huart != NULL) && (huart == g_master_comm.huart))
    {
        BMS_MasterComm_RxCallback(&g_master_comm, huart);
    }
}

/**
 * @brief  Escrita Single Device (com DEV_ADR no frame)
 *         Formato: INIT(1) + DEV_ADR(1) + REG_ADR(2) + DATA(n) + CRC(2)
 *
 * @param  hbms         Handle do BMS
 * @param  dev_addr     Endereço do dispositivo (0x00..0x7F)
 * @param  reg_addr     Endereço do registo (16 bits)
 * @param  data         Dados a escrever
 * @param  data_len     Número de bytes de dados
 * @return BMS_OK ou código de erro
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

    /* INIT byte dinâmico: bits[2:0] = (data_len-1) codificam o tamanho do payload.
     * Com INIT estático 0x90 (size=0 → 1 byte), multi-byte writes falhavam:
     * o IC lia o 2º byte de dados como CRC → FMT_ERR e frame rejeitado. */
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
 *
 *  FRAME TX CORRECTO — protocolo TI BQ79616-Q1:
 *    [INIT_BASE_SINGLE_READ] [DEV_ADR] [REG_H] [REG_L] [(data_len-1)] [CRC_L] [CRC_H]
 *    = 7 bytes fixos; CRC cobre 5 bytes.
 *
 *  INTERPRETAÇÃO DO CAMPO SIZE NO INIT:
 *    SIZE=000 no INIT indica que o frame de comando contém 1 byte de payload.
 *    Esse 1 byte é (data_len - 1), que diz ao IC quantos bytes devolver (N-1).
 *    Esta estrutura suporta leituras de 1 a 255 bytes — não limitada a 8.
 *
 *  ERRO DA VERSÃO ANTERIOR (v2.5):
 *    Usava BMS_INIT(base, data_len) que codificava (data_len-1) em bits[2:0].
 *    Para data_len=30: (30-1)&0x07 = 5 → INIT=0x95.
 *    O IC interpretava: "frame de ESCRITA com 6 bytes de dados seguidos."
 *    O MCU enviava apenas REG_ADR + CRC → FMT_ERR garantido pelo IC.
 */
BMS_Status_t BMS_ReadSingleDevice(BMS_Handle_t *hbms, uint8_t dev_addr,
                                   uint16_t reg_addr, uint8_t *rx_data,
                                   uint8_t data_len)
{
    uint8_t  tx_frame[7U];   /* INIT + DEV + REG×2 + (N-1) + CRC×2 = 7 bytes */
    uint8_t  rx_frame[BMS_MAX_RESPONSE_PAYLOAD];
    uint16_t tx_idx = 0U;
    uint16_t crc_calc, crc_recv;
    BMS_Status_t status;

    if ((rx_data == NULL) || (data_len == 0U))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    /* OBS-01: guard de overflow do buffer de recepção.
     * rx_total = 6 + data_len; se data_len for muito grande, rx_raw[rx_total]
     * ultrapassa BMS_MAX_RESPONSE_PAYLOAD → stack corruption silenciosa. */
    uint16_t rx_total = (uint16_t)(1U + 1U + 2U + data_len + 2U);
    if (rx_total > (uint16_t)BMS_MAX_RESPONSE_PAYLOAD)
    {
        return BMS_ERR_INVALID_PARAM;
    }

    /* INIT base fixo: SIZE=000 → anuncia exactamente 1 byte de payload
     * (o byte data_len-1 que vem a seguir). Bit7=1 assegura que o IC
     * reconhece o frame como comando do host, não como resposta. */
    tx_frame[tx_idx++] = INIT_BASE_SINGLE_READ;           /* 0x90, SIZE=000 */
    tx_frame[tx_idx++] = dev_addr & 0x7FU;
    tx_frame[tx_idx++] = (uint8_t)(reg_addr >> 8U);
    tx_frame[tx_idx++] = (uint8_t)(reg_addr & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(data_len - 1U);        /* Payload: bytes a ler - 1 */

    /* CRC cobre 5 bytes: INIT + DEV_ADR + REG_H + REG_L + (data_len-1) */
    crc_calc = BMS_CalculateCRC16(tx_frame, tx_idx);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc >> 8U);

    /* rx_total já calculado e validado no guard OBS-01 acima */
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

    /* Dados a offset 4: após INIT + DEV + REG_H + REG_L */
    for (uint8_t i = 0U; i < data_len; i++)
    {
        rx_data[i] = rx_frame[4U + i];
    }
    return BMS_OK;
}

/**
 * @brief  Escrita Broadcast (sem DEV_ADR)
 *         Formato: INIT(1) + REG_ADR(2) + DATA(n) + CRC(2)
 *
 * @param  hbms         Handle do BMS
 * @param  reg_addr     Endereço do registo
 * @param  data         Dados a escrever
 * @param  data_len     Número de bytes de dados
 * @return BMS_OK ou código de erro
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

    /* INIT dinâmico: base Broadcast Write + tamanho do payload */
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
 *
 *  FRAME TX CORRECTO:
 *    [INIT_BASE_BROADCAST_READ] [REG_H] [REG_L] [(data_len-1)] [CRC_L] [CRC_H]
 *    = 6 bytes; CRC cobre 4 bytes (INIT + REG×2 + data_len-1).
 *
 *  ERRO DA VERSÃO ANTERIOR (v2.5):
 *    BMS_INIT(INIT_BASE_BROADCAST_READ, N) para N>8 truncava bits[2:0],
 *    causando FMT_ERR no IC. Sem byte explícito, o IC não sabia quantos
 *    bytes devolver.
 */
BMS_Status_t BMS_ReadBroadcast(BMS_Handle_t *hbms, uint16_t reg_addr,
                                uint8_t *rx_data, uint8_t data_len_per_dev)
{
    uint8_t  tx_frame[6U];   /* INIT + REG×2 + (N-1) + CRC×2 = 6 bytes */
    uint16_t tx_idx = 0U;
    uint16_t crc_calc;
    /* Resposta por dispositivo — protocolo BQ79600 inclui DEV_ADDR em TODAS
     * as respostas (single E broadcast):
     *   [INIT(1)] [DEV_ADDR(1)] [REG_H(1)] [REG_L(1)] [DATA(N)] [CRC_L(1)] [CRC_H(1)]
     *   = 6 + N bytes por dispositivo
     *
     * BUG-03 CORRIGIDO: versão anterior usava 5+N (omitia DEV_ADDR) → CRC
     * calculado sobre N-1 bytes errados → todas as leituras broadcast
     * falhavam com CRC mismatch → BMS_ProcessFaults nunca funcionava. */
    uint16_t per_dev_size = (uint16_t)(1U + 1U + 2U + (uint16_t)data_len_per_dev + 2U);
    uint16_t rx_total     = (uint16_t)(BMS_NUM_SLAVES * per_dev_size);
    uint8_t  rx_raw[BMS_MAX_RESPONSE_PAYLOAD];
    BMS_Status_t status;

    if ((rx_data == NULL) || (data_len_per_dev == 0U))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    /* INIT base: SIZE=000 → 1 byte de payload (o byte data_len-1 seguinte) */
    tx_frame[tx_idx++] = INIT_BASE_BROADCAST_READ;             /* 0x80, SIZE=000 */
    tx_frame[tx_idx++] = (uint8_t)(reg_addr >> 8U);
    tx_frame[tx_idx++] = (uint8_t)(reg_addr & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(data_len_per_dev - 1U);     /* Payload: N-1 */

    /* CRC cobre 4 bytes: INIT + REG_H + REG_L + (data_len-1) */
    crc_calc = BMS_CalculateCRC16(tx_frame, tx_idx);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc & 0xFFU);
    tx_frame[tx_idx++] = (uint8_t)(crc_calc >> 8U);

    status = BMS_Transceive(hbms, tx_frame, tx_idx, rx_raw, rx_total);
    if (status != BMS_OK) { return status; }

    /* Parse com correcção topológica dependente da direcção:
     *
     * DIR0 (Normal): Bridge→Slave1→Slave2. Slave2 (TOP_STACK) responde primeiro.
     *   wire_idx=0 → Slave2 → logical_idx = NUM_SLAVES-1-0 = 1 ✓
     *
     * DIR1 (Reverso): Bridge→Slave2→Slave1. Slave1 (fim da linha) responde primeiro.
     *   wire_idx=0 → Slave1 → logical_idx = 0 (mapeamento directo) ✓
     *
     * BUG CORRIGIDO: inversão estática em ambos os modos cruzava telemetria
     * dos Packs durante ring recovery → balanceamento na célula errada. */
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
            /* Offset +4: após INIT(1) + DEV_ADDR(1) + REG_H(1) + REG_L(1) */
            rx_data[logical_idx * data_len_per_dev + b] = rx_raw[offset + 4U + b];
        }
        hbms->slave[logical_idx].comm_ok = true;
        offset += per_dev_size;
    }
    return BMS_OK;
}

/**
 * @brief  Escrita Broadcast Reversa para configuração do caminho DIR1
 *
 * @param  hbms         Handle do BMS
 * @param  reg_addr     Endereço do registo
 * @param  init_base    Base INIT — usar sempre INIT_BASE_BCAST_REV_WRITE (0xF8).
 *                     INIT_BASE_BCAST_REV_ADDR (0xF9) foi REMOVIDO: o bit0=1
 *                     anunciava 2 bytes de payload mas DIR1_ADDR tem 8 bits →
 *                     o IC absorvia o byte baixo do CRC como 2º dado → Ring Break.
 * @param  data         Dados a escrever
 * @param  data_len     Número de bytes de dados (1-8)
 * @return BMS_OK ou código de erro
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

    /* INIT dinâmico: base reversa + payload size */
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
 * @brief  Pulso WAKE - força TX ao nível LOW para acordar a Bridge do SHUTDOWN
 *         UART TX é temporariamente controlado como GPIO
 *
 * @param  hbms     Handle do BMS
 */
static void BMS_SendWakePulse(BMS_Handle_t *hbms)
{
    /* Desactiva a UART para controlo manual do pino */
    HAL_UART_DeInit(hbms->huart);

    /* Configura TX como GPIO Output e força LOW */
    GPIO_InitTypeDef gpio_cfg = {0};
    /* NOTA: O pino TX deve ser definido na configuração do projecto.
     * Pino TX da bridge: PA0 = UART4_TX (definido em BMS_BRIDGE_TX_PIN).
     * Ajustar conforme o hardware real. */
    gpio_cfg.Pin = BMS_BRIDGE_TX_PIN;       /* Ajustar para o pino TX real */
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &gpio_cfg);  /* Ajustar para o porto real */

    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);  /* LOW */
    BMS_DelayUs(hbms, DELAY_WAKE_PULSE_US);                /* 2500 µs */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);    /* HIGH */

    /* Reínicializa a UART */
    HAL_UART_Init(hbms->huart);
    BMS_DelayMs(DELAY_OSC_STAB_MS);  /* 2 ms para osciladores da bridge */
}

/**
 * @brief  Sincronização DLL - 8 dummy stack writes para ECC_DATA1..8
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro
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
 *         Configura DIR0 (caminho normal) e DIR1 (caminho reverso)
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou BMS_ERR_INIT_FAILED
 */
BMS_Status_t BMS_AutoAddressing(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t data;

    /* ------------------------------------------------------------------ */
    /* PASSO 1: Hardware Reset - Pulso WAKE */
    /* ------------------------------------------------------------------ */
    BMS_SendWakePulse(hbms);

    /* ------------------------------------------------------------------ */
    /* PASSO 2: WAKE Tone para a stack de slaves */
    /* ------------------------------------------------------------------ */
    data = CTRL1_SEND_WAKE;  /* 0x20 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }
    BMS_DelayMs(DELAY_WAKE_PROPAGATION_MS);  /* 5 ms - propagação obrigatória */

    /* ------------------------------------------------------------------ */
    /* PASSO 3: Sincronização DLL */
    /* ------------------------------------------------------------------ */
    status = BMS_SyncDLL(hbms);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* ------------------------------------------------------------------ */
    /* PASSO 4: Endereçamento do Caminho Principal (DIR_SEL = 0) */
    /* ------------------------------------------------------------------ */
    /* Ativar modo ADDR_WR */
    data = CTRL1_ADDR_WR;  /* 0x01 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Bridge: DIR0_ADDR = 0x00 */
    data = BMS_ADDR_BRIDGE;  /* 0x00 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_DIR0_ADDR, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Slave 1: DIR0_ADDR = 0x01 */
    data = BMS_ADDR_SLAVE1;  /* 0x01 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_DIR0_ADDR, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Slave 2: DIR0_ADDR = 0x02 */
    data = BMS_ADDR_SLAVE2;  /* 0x02 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_DIR0_ADDR, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* ------------------------------------------------------------------ */
    /* PASSO 5: Definição de Papéis no Caminho Principal */
    /* ------------------------------------------------------------------ */
    /* Todos os dispositivos: STACK_DEV = 1 */
    data = COMM_CTRL_STACK_DEV;  /* 0x02 */
    status = BMS_WriteBroadcast(hbms, REG_BRIDGE_COMM_CTRL, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Slave 2 (topo físico): TOP_STACK = 1 */
    data = COMM_CTRL_TOP_STACK;  /* 0x03 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_SLAVE2,
                                    REG_BRIDGE_COMM_CTRL, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* ------------------------------------------------------------------ */
    /* PASSO 6: Configuração do Anel (Caminho Reverso DIR_SEL = 1) */
    /* ------------------------------------------------------------------ */
    /* Trocar DIR_SEL na Bridge */
    data = CTRL1_DIR_SEL;  /* 0x02 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }
    BMS_DelayUs(hbms, DELAY_DIR_SEL_SWITCH_US);  /* 100 µs */

    /* Inverter direcção de escuta das escravas (Broadcast Rev Write, dado = 0x80) */
    data = 0x80U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_CONTROL1,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* ------------------------------------------------------------------ */
    /* PASSO 7: Endereçamento do Caminho Reverso */
    /* ------------------------------------------------------------------ */
    /* Ativar escrita invertida — usa INIT_BASE_BCAST_REV_WRITE (0xF8, 1 byte) */
    data = 0x81U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_CONTROL1,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Bridge: DIR1_ADDR = 0x00 */
    data = BMS_ADDR_BRIDGE;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_DIR1_ADDR,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Slave 2 (primeiro na volta): DIR1_ADDR = 0x01 */
    data = 0x01U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_DIR1_ADDR,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Slave 1: DIR1_ADDR = 0x02 */
    data = 0x02U;
    status = BMS_WriteBroadcastReverse(hbms, REG_BRIDGE_DIR1_ADDR,
                                        INIT_BASE_BCAST_REV_WRITE, &data, 1U);
    if (status != BMS_OK) { return BMS_ERR_INIT_FAILED; }

    /* Slave 1 (endereço reverso 0x02): novo TOP_STACK no anel reverso */
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

    /* ------------------------------------------------------------------ */
    /* SANEAMENTO PÓS-ENDEREÇAMENTO (TI ref: AutoAddress() post-sequence) */
    /* ------------------------------------------------------------------ */
    /* A sequência de endereçamento é electromagneticamente instável:
     * mudanças de direcção, oscilações de DIR_SEL e transições bidirec-
     * cionais geram FMT_ERR e comm faults nos latches do BQ79616.
     * Se não forem limpos, o pino NFAULT transita para LOW imediatamente
     * após a inicialização, forçando um falso BMS_FaultRecoveryAttempt
     * logo no primeiro ciclo de monitorização.
     *
     * TI faz 2 acções após AutoAddress():
     *   1. 8 leituras dummy de OTP_ECC (purga dos buffers internos do IC)
     *   2. FAULT_RST2 = 0x03 (limpa latches de COMM_ERR1 e COMM_ERR2)
     *
     * Implementamos com 8 writes broadcast (equivalente funcional para
     * purga de buffers) + FAULT_RST2 em todos os slaves e bridge. */
    {
        /* Purga via READ obriga escoamento do TX FIFO dos slaves.
         * Writes apenas preenchem os RX FIFOs; se o ruído do DIR_SEL
         * gerou tramas encravadas no TX FIFO dos BQ79616, apenas um
         * comando de leitura os força a emitir e esvaziar a linha. */
        uint8_t dummy_rx[BMS_NUM_SLAVES];
        for (uint8_t i = 0U; i < 8U; i++)
        {
            (void)BMS_ReadBroadcast(hbms, REG_ECC_DATA1 + i, dummy_rx, 1U);
        }
        BMS_DelayMs(2U);  /* settling após purga */

        /* Limpar latches de COMM_ERR gerados durante endereçamento */
        uint8_t clear_comm = 0x03U;   /* bit0=COMM_ERR1, bit1=COMM_ERR2 */
        (void)BMS_WriteBroadcast(hbms, REG_FAULT_RST2, &clear_comm, 1U);
        (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_FAULT_RST2, &clear_comm, 1U);
    }

    return BMS_OK;
}

/**
 * @brief  Configura ambos os slaves para 15 células
 *         IMPORTANTE: Requer curto-circuito de hardware VC16->VC15 e CB16->CB15
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro
 */
BMS_Status_t BMS_ConfigureSlaves(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t data;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        uint8_t slave_addr = hbms->slave[s].address;

        /* --- Configurar 15 células activas --- */
        data = ACTIVE_CELL_15S;  /* 0x0F */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_ACTIVE_CELL, &data, 1U);
        if (status != BMS_OK) { return status; }

        /* --- Threshold Sobretensão: 0x24 → 3600 mV (2700 + 36×25 mV) --- */
        data = OV_THRESH_VAL;  /* 0x24 */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_OV_THRESH, &data, 1U);
        if (status != BMS_OK) { return status; }

        /* --- Threshold Subtensão: 0x24 ≈ 3000 mV --- */
        data = UV_THRESH_VAL;  /* 0x24 */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_UV_THRESH, &data, 1U);
        if (status != BMS_OK) { return status; }

        /* --- Filtro passa-baixo ADC --- */
        data = 0x02U;  /* LPF corner frequency padrão */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_ADC_CONF1, &data, 1U);
        if (status != BMS_OK) { return status; }

        /* --- Activar protecções hardware OVUV --- */
        data = OVUV_CTRL_ENABLE;  /* 0x06 */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_OVUV_CTRL, &data, 1U);
        if (status != BMS_OK) { return status; }

        /* --- Configurar temporizador de auto-stop de balanceamento --- */
        data = BAL_CTRL1_TIMER_10MIN;
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_BAL_CTRL1, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = BAL_CTRL2_AUTOSTOP_EN;
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_BAL_CTRL2, &data, 1U);
        if (status != BMS_OK) { return status; }

        /* --- Garantir balanceamento desligado ao arranque --- */
        {
            uint8_t zero[2U] = {0x00U, 0x00U};
            status = BMS_WriteSingleDevice(hbms, slave_addr,
                                            REG_CB_CELL1_CTRL, zero, 2U);
            if (status != BMS_OK) { return status; }
        }

        /* --- Arrancar ADC em modo contínuo com LPF --- */
        /* MAIN_GO=1, MAIN_MODE=0b10, LPF_EN=1 -> 0x2E */
        data = ADC_CTRL1_CONTINUOUS_LPF;
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_ADC_CTRL1, &data, 1U);
        if (status != BMS_OK) { return status; }
    }

    BMS_DelayMs(DELAY_ADC_SETTLE_MS);  /* Aguardar estabilização ADC */
    return BMS_OK;
}

/**
 * @brief  Inicialização completa do BMS
 *         Executa a sequência completa: WAKE -> Addressing -> Config
 *
 * @param  hbms     Handle do BMS (pré-alocado pela aplicação)
 * @param  huart    Handle UART do HAL
 * @param  htim     Handle Timer do HAL (para delays µs)
 * @return BMS_OK ou BMS_ERR_INIT_FAILED
 */
BMS_Status_t BMS_Init(BMS_Handle_t *hbms, UART_HandleTypeDef *huart,
                       TIM_HandleTypeDef *htim)
{
    BMS_Status_t status;

    if ((hbms == NULL) || (huart == NULL))
    {
        return BMS_ERR_INVALID_PARAM;
    }

    /* Inicialização a zero da estrutura */
    memset(hbms, 0, sizeof(BMS_Handle_t));

    /* Atribuir handles HAL */
    hbms->huart        = huart;
    hbms->htim_delay   = htim;
    hbms->state        = BMS_STATE_INITIALIZING;
    hbms->ring_intact  = false;
    hbms->nfault_pending = 0U;

    /* D9: Iniciar timer em modo free-running (counter runs indefinidamente).
     * BMS_DelayUs usa subtracção de contador, não Start/Stop por chamada.
     * Com TIM2 de 32 bits, ARR=0xFFFFFFFF e 1 µs/tick, wraparound a ~71 min. */
    if (hbms->htim_delay != NULL)
    {
        HAL_TIM_Base_Start(hbms->htim_delay);
    }

    /* Regista ponteiro global para a ISR */
    g_hbms_irq = hbms;

    /* Sequência de endereçamento automático */
    status = BMS_AutoAddressing(hbms);
    if (status != BMS_OK)
    {
        hbms->state = BMS_STATE_FAULT;
        hbms->fault_flags |= BMS_FAULT_COMM;
        return BMS_ERR_INIT_FAILED;
    }

    /* Configuração dos slaves (15S) */
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
 * GESTÃO DE ENERGIA — PULSOS DE CONTROLO DE ESTADO
 * =========================================================================
 * Todas as funções abaixo reutilizam a técnica de BMS_SendWakePulse:
 * desinicializam a UART, controlam o pino TX (PA0) como GPIO puro com o pulso adequado,
 * e reinicializam a UART. O pino TX (BMS_BRIDGE_TX_PIN) corresponde a UART4_TX (PA0).
 * ========================================================================= */

/**
 * @brief  Envia pulso de Shutdown (9 ms LOW) → transição WAKE → SHUTDOWN
 *
 *  Num veículo com chave OFF, o BMS não pode permanecer em MONITORING
 *  a consumir dezenas de mA — esgota a bateria auxiliar de 12V e causa
 *  descarga profunda do pack de HV. Este pulso coloca todos os ICs em
 *  low-power SHUTDOWN mode (consumo < 1 µA por IC).
 */
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

/**
 * @brief  Envia pulso de Hardware Reset (40 ms LOW) → reset completo da rede
 *
 *  Força a reinicialização completa de todos os ICs na linha daisy-chain.
 *  Útil após falhas graves de comunicação irrecuperáveis ou para diagnóstico.
 *  Após este pulso é obrigatório executar BMS_Init() novamente.
 */
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

/**
 * @brief  Sequência completa de entrada em Sleep
 *         Para balanceamento → abre contactor → aguarda → pulso shutdown
 */
void BMS_EnterSleep(BMS_Handle_t *hbms)
{
    /* 1. Parar balanceamento imediatamente */
    (void)BMS_StopAllBalancing(hbms);

    /* 2. Abrir contactor de potência */
    BMS_ContactorOpen(hbms);
    BMS_DelayMs(DELAY_SLEEP_CONTACTOR_MS);  /* Aguardar abertura mecânica */

    /* 3. Enviar pulso de shutdown para todos os ICs */
    BMS_SendShutdownPulse(hbms);
    /* state = BMS_STATE_SLEEP (set em BMS_SendShutdownPulse) */
}

/* =========================================================================
 * HAL_GPIO_EXTI_Callback — Captura NFAULT (EXTI13 Falling Edge)
 * =========================================================================
 * D2 CORRIGIDO: movido de bq796xx_bms_monitor.c para aqui.
 * g_hbms_irq é static neste ficheiro — era inacessível na outra unidade
 * de tradução, causando 'undefined reference' no linker.
 * Consistente com o HAL_UART_RxCpltCallback que já reside neste ficheiro. */

#define NFAULT_GPIO_PIN  GPIO_PIN_13  /* PC13 — ajustar conforme hardware */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NFAULT_GPIO_PIN)
    {
        if (g_hbms_irq != NULL)
        {
            BMS_NFAULT_IRQHandler(g_hbms_irq);
        }
    }
}

/* =========================================================================
 * SECÇÃO: WATCHDOG INDEPENDENTE (IWDG)
 * =========================================================================
 * O IWDG usa o oscilador LSI (~32 kHz), independente do HSE/PLL.
 * Se o CPU pendurar, o IWDG não é refrescado e reseta o MCU inteiro.
 * A inicialização é irreversível — uma vez activado, não pode ser desligado.
 *
 * Configuração: PSC=64 → tick=2ms, RLR=250 → timeout≈500ms.
 * Refresh obrigatório em CADA iteração do super-loop (100 ms cadência).
 * 5 ciclos de margem antes do reset. */

void BMS_IWDG_Init(void)
{
    IWDG_HandleTypeDef hiwdg = {0};
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = BMS_IWDG_PRESCALER;   /* IWDG_PRESCALER_64 */
    hiwdg.Init.Reload    = BMS_IWDG_RELOAD;       /* 250 → ~500 ms */
    (void)HAL_IWDG_Init(&hiwdg);
    /* A partir deste ponto, HAL_IWDG_Refresh DEVE ser chamado a cada <500ms */
}

void BMS_IWDG_Refresh(void)
{
    IWDG_HandleTypeDef hiwdg = {0};
    hiwdg.Instance = IWDG;
    (void)HAL_IWDG_Refresh(&hiwdg);
}

/* =========================================================================
 * SECÇÃO: COMM CLEAR (Reset da State Machine do Receptor UART)
 * =========================================================================
 * Quando uma leitura excede tWAIT_READ_MAX, o receptor UART do BQ79600/BQ79616
 * pode ficar num estado intermédio (à espera de bytes que não chegam).
 * Forçar TX a LOW por 18 períodos de bit (18 µs a 1 Mbps) gera um break
 * condition que reseta a state machine receptora de todos os ICs na linha.
 *
 * Este procedimento é descrito na datasheet do BQ79600-Q1 como obrigatório
 * após qualquer timeout de comunicação (secção "COMM CLEAR"). */

void BMS_CommClear(BMS_Handle_t *hbms)
{
    /* Desinicializar UART para controlo GPIO directo */
    HAL_UART_DeInit(hbms->huart);

    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin = BMS_BRIDGE_TX_PIN;   /* PA0 = UART4_TX */
    gpio_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull  = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BMS_BRIDGE_TX_PORT, &gpio_cfg);

    /* Forçar TX a LOW por 18 períodos de bit = 18 µs @ 1 Mbps */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_RESET);
    BMS_DelayUs(hbms, BMS_COMM_CLEAR_BITS);   /* 18 µs */
    HAL_GPIO_WritePin(BMS_BRIDGE_TX_PORT, BMS_BRIDGE_TX_PIN, GPIO_PIN_SET);

    /* Reinicializar UART */
    HAL_UART_Init(hbms->huart);
    BMS_DelayUs(hbms, 50U);  /* Estabilização pós-UART init */
}

/* =========================================================================
 * SECÇÃO: DETECÇÃO DE SOLDADURA DO CONTACTOR (NÃO-BLOQUEANTE)
 * =========================================================================
 * AUDITORIA v3.1: A versão anterior usava 3 leituras com 15 ms de delay
 * cada (45 ms bloqueante), consumindo quase metade do ciclo de 100 ms.
 * Isto induzia jitter na telemetria e podia disparar falsos alarmes no
 * watchdog da VCU.
 *
 * CORRECÇÃO: paradigma não-bloqueante com máquina de estados finita.
 * A cada ciclo de 100 ms lê-se o pin UMA vez (0 µs de delay bloqueante).
 * Três ciclos consecutivos com leitura HIGH → soldadura confirmada.
 * Uma leitura LOW reinicia o contador — debounce natural pelo período
 * do superloop (100 ms >> tempo de bounce mecânico de ~2 ms). */

bool BMS_CheckContactorWeld(void)
{
    /* Variável estática: persiste entre chamadas — conta ciclos consecutivos */
    static uint8_t consecutive_high = 0U;

    if (HAL_GPIO_ReadPin(CONTACTOR_WELD_PORT, CONTACTOR_WELD_PIN) == GPIO_PIN_SET)
    {
        if (consecutive_high < 255U) { consecutive_high++; }
    }
    else
    {
        consecutive_high = 0U;   /* Reset: pin LOW → contactor abriu correctamente */
    }

    /* Maioria 3/3: três ciclos consecutivos (300 ms) com pin HIGH → weld */
    return (consecutive_high >= 3U);
}

/* =========================================================================
 * SECÇÃO: POWER-ON SELF TEST (POST)
 * =========================================================================
 * Sequência de auto-diagnóstico executada ANTES do arranque operacional.
 * Verifica a integridade do CRC, a comunicação com cada slave, a sanidade
 * do ADC e o estado do pino NFAULT. Se qualquer teste falhar, o sistema
 * NÃO transita para MONITORING — permanece em FAULT até resolução.
 *
 * Referência: ISO 26262 Part 5 — Hardware Design Verification */

BMS_Status_t BMS_PowerOnSelfTest(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t test_buf[2U];

    /* ---------------------------------------------------------------
     * TESTE 1: Integridade do algoritmo CRC
     * Verifica o vector padrão "123456789" → 0x4B37 (CRC-16/MODBUS)
     * Se falhar, corrupção de código/RAM — MCU comprometido.
     * --------------------------------------------------------------- */
    {
        uint8_t crc_test[] = {'1','2','3','4','5','6','7','8','9'};
        uint16_t crc = BMS_CalculateCRC16(crc_test, 9U);
        if (crc != 0x4B37U)
        {
            hbms->fault_flags |= BMS_FAULT_CRC;
            return BMS_ERR_INIT_FAILED;
        }
    }

    /* ---------------------------------------------------------------
     * TESTE 2: Comunicação com cada slave (lê ACTIVE_CELL)
     * Valor esperado: 0x0F (15 células, configurado em BMS_ConfigureSlaves)
     * Se falhar: slave inacessível, cabo partido, ou IC danificado.
     * --------------------------------------------------------------- */
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
            /* Registo não corresponde à configuração — IC não inicializado */
            hbms->fault_flags |= BMS_FAULT_COMM;
            return BMS_ERR_INIT_FAILED;
        }
    }

    /* ---------------------------------------------------------------
     * TESTE 3: Sanidade do ADC (tensões celulares no range físico)
     * Cada célula LiIon deve estar entre 1000 mV e 4500 mV.
     * Fora deste range: sensor desligado, open wire, ou ADC danificado.
     * --------------------------------------------------------------- */
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

    /* ---------------------------------------------------------------
     * TESTE 4: NFAULT deve estar HIGH (sem faults residuais)
     * Após limpeza dos latches em BMS_AutoAddressing, o pin deve estar
     * desassertado. Se LOW: IC com fault permanente ou hardware danificado.
     * --------------------------------------------------------------- */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }

    hbms->post_passed = true;
    return BMS_OK;
}
