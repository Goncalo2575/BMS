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
 *  Esta camada trata da comunicação com a daisy-chain e calcula as decisões
 *  LÓGICAS de segurança. A ACTUAÇÃO FÍSICA dos relés/LED é do módulo
 *  bms_relays.c  — ver bq796xx_bms.h .
 *
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
 *
 * Para que serve: pausa simples baseada no SysTick (1 ms de resolução),
 * usada nas sequências de arranque/configuração onde a precisão de ms basta.
 * NÃO usar para pausas longas dentro do super-loop (o IWDG ~500 ms rebenta).
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
 *
 *
 * O protocolo de comunicação dos chips BQ796xx da Texas Instruments exige
 * timings elétricos muito rigorosos para acordar ou adormecer a rede.
 * Por exemplo, o pulso de WAKE tem de durar 2500 µs (2.5 milissegundos).
 * Usar a função HAL_Delay() baseada no SysTick (que apenas tem precisão de 1 milissegundo)
 * seria demasiado impreciso. A função BMS_DelayUs, ao ler diretamente o hardware
 * de um temporizador a rodar a 1 MHz (1 tick = 1 µs), consegue fazer essa pausa
 * com precisão cirúrgica sem criar o "overhead" (peso no processador) de estar a gerir interrupções.
 * Além disso, o texto acerta ao mencionar o fallback (o ciclo while cego)
 * caso o temporizador falhe ou não seja inicializado.
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
 *
 * Calcula uma "assinatura matemática" (Cyclic Redundancy Check) para garantir a
 * integridade das mensagens enviadas e recebidas pela Daisy-Chain. Num ambiente de veículo elétrico,
 * o ruído eletromagnético dos inversores e motores pode facilmente alterar (corromper) um bit de
 * dados nos cabos de comunicação. Sem o CRC, um bit corrompido poderia transformar uma leitura de
 * tensão normal numa falsa leitura de sobretensão, forçando o BMS a desligar o carro indevidamente.
 *
 * O emissor calcula este valor de 16 bits para todos os
 * bytes do pacote e anexa-o no final. O recetor recalcula-o e compara-o com o recebido. Se divergirem,
 * o pacote é descartado de imediato (silenciosamente protegido contra lixo informático).
 * O algoritmo é o CRC-16-IBM, que é um padrão comum e é o mesmo usado pelos chips BQ796xx.
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
 *
 *
 *
 * Como funciona (Passo a passo):
 * 1. Preparação (DMA RX): Se espera resposta, "arma" a receção via DMA antes sequer de falar.
 *    Isto garante que o HW já está à escuta e não perde o 1º byte que chega a alta velocidade.
 * 2. Transmissão (TX): Envia a mensagem a 1 Mbps.
 * 3. Espera com Timeout: Fica num ciclo a aguardar que o DMA receba todos os bytes (a flag
 *    dma_rx_done muda para 1). Tem um limite de tempo estrito; se a linha cair (cabo cortado),
 *    ele aborta, regista o erro e limpa a linha (CommClear) para não bloquear o sistema.
 * 4. Validação HW: Verifica se houve "Overrun" (hardware engasgado com excesso de dados).
 * 5. Escrita simples: Se for um comando sem resposta (ex: Broadcast Write), apenas transmite.
 *
 * @note Resumo da lógica: Esta função recebe a estrutura BMSHandle que contém as
 * informações das flags, DMA, UART, contagem de erros, e os dados de TX e RX com o
 * seu respetivo comprimento. Estes dados não vão para o USART2 (exterior/debug), mas sim
 * para a UART4 que fala diretamente com a Bridge e Slaves. Para isso, a função começa
 * por se colocar à escuta ativando o DMA e metendo a flag dma_rx_done a 0. A seguir
 * envia os dados (TX) e entra num 'while' onde o processador engonha à espera da resposta.
 * Como já se conhecem os tempos normais de resposta, há um "timeout" rigoroso; se não
 * for cumprido, a transmissão aborta, adiciona um erro de comunicação e limpa a linha.
 * No fim, há ainda uma verificação da flag ORE para garantir que o hardware não se
 * "engasgou" com informação a chegar rápido demais. Se vier lixo, descarta e avança!
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
 *
 * Para que serve: É a "campainha" do hardware. Esta função não é chamada
 * pelo código principal, mas sim disparada automaticamente pelo próprio hardware
 * (STM32) quando o DMA termina de receber exatamente o número de bytes pedidos.
 *
 * Como funciona: Quando o último byte da resposta chega da bateria, a
 * execução do CPU salta para aqui. Ela verifica se o aviso vem da UART correta e
 * altera a flag `dma_rx_done` para 1, libertando o `while` na função `BMS_Transceive`.
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
 *
 * Para que serve: Serve para enviar uma ordem ou comando para APENAS UM chip
 * específico da rede, ignorando todos os outros.
 *
 * Exemplos práticos no projeto:
 * - Balanceamento: Mandar apenas o Slave 2 ligar a resistência para descarregar
 *   a sua Célula 3, deixando o Slave 1 inalterado.
 * - Endereçamento (AutoAddressing): Dizer exclusivamente ao último chip da rede que
 *   ele é o "Fim da Linha" (TOP_STACK), para que o cabo de comunicação não encrave.
 *
 * Como funciona (Construção do Pacote/Frame):
 * 1. Validação: Verifica se os dados são válidos (não nulos e de 1 a 8 bytes).
 * 2. Cabeçalho (INIT): Cria o byte inicial que diz "Isto é uma escrita singular".
 * 3. Endereço (DEV_ADR): Adiciona a 'morada' do chip alvo (ex: 0x01).
 * 4. Registo (REG_ADR): Adiciona em qual "gaveta" (registo) de memória do chip quer escrever.
 * 5. Dados e Assinatura: Junta os dados, calcula o CRC-16 para segurança e entrega
 *    tudo à função `BMS_Transceive` para fazer o envio físico pelo cabo.
 *
 * Write: Constrói pacote com dados -> Envia -> Vai à sua vida.
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
 *
 * Para que serve: Interroga APENAS UM chip específico e fica à espera que
 * ele devolva a resposta. É usado, por exemplo, para ler a tensão global
 * do inversor (HV) que está ligada apenas ao Slave 1.
 *
 * Como funciona (O processo de Pergunta-Resposta):
 * 1. Constrói a Pergunta (TX): Em vez de enviar dados, envia um byte extra `(data_len - 1)`
 *    que diz ao chip quantos bytes de informação o STM32 quer receber de volta.
 * 2. Transceive: Envia a pergunta e arma o DMA para ficar ativamente à escuta da resposta.
 * 3. Barreira de Segurança (CRC): Quando a resposta chega, a primeira coisa que faz é
 *    recalcular a assinatura matemática. Se a mensagem foi corrompida por ruído, aborta!
 * 4. Extração: O chip responde ecoando o cabeçalho (4 bytes). A função ignora esse eco
 *    e extrai apenas a "carne" (os dados reais) guardando-os no buffer do utilizador.
 *
 * Read: Constrói pacote com o número de bytes que quer -> Envia -> Fica à escuta ->
 *  -> Recebe -> Verifica Segurança (CRC) -> Extrai a informação útil.
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
 *
 * Para que serve: Serve para enviar uma ordem para TODOS as slaves da rede
 * em simultâneo. É o equivalente a falar num megafone para toda a fábrica.
 * contudo nao fica a espera de resposta tal como o BMS_WriteSingleDevice)
 *
 * Exemplos práticos no projeto:
 * - Limpeza de Falhas: Enviar o comando para limpar os latches de erro (FAULT_RST)
 *   em todos os slaves com apenas uma mensagem ultra-rápida.
 *
 * Como funciona (A magia da eficiência):
 * O pacote construído AQUI NÃO TEM o byte da Morada (DEV_ADR). Como a mensagem
 * começa com o byte especial de Broadcast (0x88), os chips já sabem que não há
 * destinatário e o byte seguinte é logo o Registo. O pacote fica mais curto e rápido!
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
 *
 * Para que serve: Faz uma pergunta a TODOS os slaves da rede simultaneamente e
 * recolhe as respostas de todos num único grande "comboio" de dados. É incrivelmente
 * rápido para ler falhas de todo o pack da bateria de uma só vez. Ficam a espera de resposta,
 * mas o processo é otimizado para ser o mais eficiente possível.
 *
 * Como funciona (A Inversão Topológica):
 * 1. O STM32 envia o pedido (sem morada).
 * 2. O chip mais distante (TOP_STACK / Slave 2) responde primeiro. O Slave 1 anexa a sua
 *    resposta a seguir, criando um fluxo contínuo de bytes.
 * 3. A função recebe este bloco gigante via DMA.
 * 4. Ao desempacotar, como o Slave 2 chegou primeiro, a função faz uma inversão matemática
 *     para colocar os dados do Slave 2 no
 *    índice correto do array lógico, e valida o CRC de CADA slave individualmente.
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
 *
 * Para que serve: Envia uma mensagem global, mas "pela porta das traseiras" (DIR1).
 * É fundamental para a topologia em Anel (Ring). Se o cabo principal partir, o
 * STM32 consegue falar com os slaves pelo caminho inverso.
 *
 * Onde é usada: Principalmente na fase de Inicialização (BMS_AutoAddressing).
 * O Cérebro usa esta função para pré-configurar os endereços reversos de todos
 * os chips, preparando-os para o pior cenário.
 *
 * O detalhe técnico: Ao contrário da Broadcast normal que tem o byte INIT
 * fixo (0x88), esta recebe o init_base como parâmetro, porque o comando reverso
 * exige um byte mágico especial (0xF8) para os chips saberem de onde vem a ordem.
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
 *
 * Para que serve: Quando o carro está desligado, os chips da bateria entram
 * em sono profundo (SHUTDOWN) para não consumirem energia. Para os acordar,
 * o protocolo da TI exige um pulso elétrico muito específico de 2.5 milissegundos.
 *
 * O Truque de Hardware: Uma porta série (UART) envia dados e não consegue gerar
 * facilmente um pulso contínuo de 2.5ms. Por isso, esta função "rouba" o pino TX
 * à UART, transforma-o num pino de saída normal (GPIO), puxa-o para LOW (0V)
 * durante exatamente 2500 µs, volta a pô-lo em HIGH e, no fim, devolve o pino à UART.
 * Aguarda ainda 2 ms para que os osciladores internos da Bridge estabilizem.
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
 *
 * Para que serve: É o "Teste de Relógio" da rede. Após os chips acordarem, os
 * seus relógios internos precisam de se calibrar à velocidade de 1 Mbps da UART.
 *
 * Como funciona: O STM32 envia 8 mensagens Broadcast consecutivas (de valor 0x00)
 * para os registos seguros ECC_DATA (que não afetam a configuração do chip).
 * Estas transições elétricas no cabo permitem à DLL (Delay-Locked Loop) de cada
 * chip afinar o seu recetor para não perder nenhum bit nas mensagens seguintes,
 * que já serão ordens críticas (como atribuir moradas).
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
 *
 * Para que serve: É a "Chamada de Presenças" e atribuição de lugares.
 * Quando o carro liga, os chips não sabem quem são nem onde estão no cabo.
 * Esta função acorda-os, afina-os e dá a cada um uma "morada" oficial.
 *
 * Como funciona (O Truque do ADDR_WR):
 * 1. O STM32 ativa o modo "Address Write" (ADDR_WR).
 * 2. Envia um Broadcast com a morada 0x00. A Bridge (1º chip) apanha-a, guarda para si
 *    e "fecha a sua porta" a novos endereços.
 * 3. Envia Broadcast 0x01. A Bridge já tem morada, por isso passa para a frente. O Slave 1
 *    apanha-a, guarda para si e fecha a porta. E assim sucessivamente.
 * 4. Fim da Linha: Diz ao último Slave que ele é o topo da pilha (TOP_STACK) para ele saber
 *    que tem de devolver o sinal para trás.
 * 5. Via de Emergência (DIR1): A Bridge muda a direção da agulha para o cabo traseiro e
 *    repete a chamada ao contrário (usando os comandos Reverse), preparando
 *    os endereços alternativos caso o cabo principal parta no futuro!
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
 * @brief  Configura ambos os slaves para 15 células e define proteções (Regras do Jogo)
 *
 * Para que serve: Define os limites de segurança de hardware, os filtros de ruído,
 * os temporizadores de balanceamento e manda os conversores (ADCs) iniciarem as leituras.
 *
 * Como funciona (Passo a Passo por Slave):
 * 1. ACTIVE_CELL: Avisa o slave que só tem 15 células ligadas.
 * 2. OV_THRESH / UV_THRESH: Define os limites físicos de OV (4250mV) e UV (3000mV).
 * 3. ADC_CONF / OVUV_CTRL: Liga os filtros passa-baixo e ativa as proteções autónomas.
 * 4. BAL_CTRL: Define segurança térmica (ex: autostop do balanceamento aos 10 minutos).
 * 5. ADC_CTRL1: Dá o comando MAIN_GO para iniciar as medições ADC em ciclo contínuo.
 * 6. Aguarda 10ms no final para garantir que as primeiras amostras do ADC estão prontas.
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

        data = OV_THRESH_VAL;  /* 0x3E → 4250 mV (NMC: 2700 + 62×25 mV) */
        status = BMS_WriteSingleDevice(hbms, slave_addr,
                                        REG_OV_THRESH, &data, 1U);
        if (status != BMS_OK) { return status; }

        data = UV_THRESH_VAL;  /* 0x24 → 3000 mV (2100 + 36×25 mV) */
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
 * @brief  Inicialização completa do BMS (O Maestro do Arranque)
 *
 * Para que serve: Liga a estrutura de software do BMS ao hardware do STM32,
 * limpa todo o lixo de memória e orquestra a sequência de arranque da Daisy-Chain.
 *
 * Como funciona (Passo a Passo):
 * 1. Limpeza: Faz um memset a 0 a todo o handle para apagar "lixo" na RAM.
 * 2. Setup HW: Associa a UART e liga o Timer de microssegundos (TIM2) em roda livre.
 * 3. Ponte IRQ: Regista o handle em `g_hbms_irq` para as interrupções de HW (DMA
 *    e EXTI/NFAULT) saberem com quem falar.
 * 4. Acordar e Mapear: Executa a auto-configuração de endereços (AutoAddressing).
 * 5. Proteger e Ligar: Envia as proteções e ativa as leituras ADC (ConfigureSlaves).
 * 6. Sucesso: Se os passos 4 e 5 não derem erro, muda o estado para MONITORING.
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

    /* IWDG (~500 ms) já armado pelo CubeMX antes do BMS_Main. O BMS_Init tem
     * fases bloqueantes (wake/propagação/settle + dezenas de transacções UART
     * que, com latências/timeouts, podem aproximar-se do timeout). Refrescar
     * em pontos estratégicos (cada fase é limitada no tempo — não mascara hang)
     * evita um reset prematuro a meio do arranque. NÃO refrescar no loop de
     * falha de init (esse é o mecanismo de auto-recuperação "Via B"). */
    BMS_IWDG_Refresh();

    status = BMS_AutoAddressing(hbms);
    if (status != BMS_OK)
    {
        hbms->state = BMS_STATE_FAULT;
        hbms->fault_flags |= BMS_FAULT_COMM;
        return BMS_ERR_INIT_FAILED;
    }

    /* Refresco estratégico entre as duas fases pesadas (endereçamento ↔
     * configuração), conforme análise de temporização IWDG vs boot. */
    BMS_IWDG_Refresh();

    status = BMS_ConfigureSlaves(hbms);
    if (status != BMS_OK)
    {
        hbms->state = BMS_STATE_FAULT;
        return BMS_ERR_INIT_FAILED;
    }

    BMS_IWDG_Refresh();
    hbms->state = BMS_STATE_MONITORING;
    return BMS_OK;
}

/* =========================================================================
 * GESTÃO DE ENERGIA — PULSOS DE CONTROLO DE ESTADO (linha TX PA0)
 * ========================================================================= */
/**
 * @brief  Pulso SHUTDOWN - Força TX ao nível LOW por 9ms para adormecer a rede
 *
 * Para que serve: Coloca todos os chips da bateria em modo de sono profundo
 * (baixo consumo) para poupar a bateria do carro quando este está
 * desligado ou após uma falha crítica.
 *
 * Como funciona: Tal como o pulso de WAKE, "rouba" o pino da UART para o
 * controlar manualmente. A diferença fundamental é o tempo: no protocolo da
 * Texas Instruments, um pulso LOW de exatamente 9 ms significa "Shutdown".
 * No final, devolve o pino e atualiza a máquina de estados para SLEEP.
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
 * @brief  Adormece o pack de bateria com segurança absoluta
 *
 * Para que serve: Desliga o sistema de forma controlada. Em vez de simplesmente
 * "ir dormir", garante que o hardware fica num estado seguro a longo prazo.
 *
 * Como funciona (Passo a Passo de Segurança):
 * 1. Pára o Balanceamento: Evita que as células continuem a descarregar enquanto o BMS dorme.
 * 2. Abre o Contactor: Dá ordem para isolar a Alta Tensão (decisão lógica; a actuação
 *    física do BMS_relay é feita por bms_relays.c ao detectar bms_ok=false/FAULT).
 * 3. Espera Mecânica: Aguarda 100ms para garantir que o relé físico tem tempo de abrir.
 * 4. Põe a Rede a Dormir: Envia o pulso de 9ms para os chips pouparem a bateria de 12V.
 */
void BMS_EnterSleep(BMS_Handle_t *hbms)
{
    (void)BMS_StopAllBalancing(hbms);

    BMS_ContactorOpen(hbms);
    BMS_DelayMs(DELAY_SLEEP_CONTACTOR_MS);  /* Aguardar abertura mecânica do relé */

    BMS_SendShutdownPulse(hbms);
    /* state = BMS_STATE_SLEEP (set em BMS_SendShutdownPulse) */
}

/* =========================================================================
 * HAL_GPIO_EXTI_Callback — Captura NFAULT (EXTI8, PA8, Falling Edge)
 * =========================================================================
 * g_hbms_irq é static neste ficheiro — daí o callback residir aqui.
 * PA8 → linha EXTI8 → vector EXTI9_5_IRQHandler (gerado pelo CubeMX). */

/**
 * @brief  Callback de Interrupção de Hardware - O Alarme de Incêndio (NFAULT)
 *
 * Para que serve: Lida com emergências detetadas autonomamente pelo hardware da bateria
 * (ex: sobretensão gravíssima). Reage instantaneamente sem esperar pelo ciclo normal do código.
 *
 * Como funciona: Quando um chip escravo deteta perigo, ele puxa fisicamente o cabo NFAULT
 * para 0V. O STM32 deteta essa queda (Falling Edge) no pino PA8 e salta imediatamente para
 * esta função, pausando tudo o resto. A função confirma a origem do alarme e chama o
 * tratador (BMS_NFAULT_IRQHandler) que regista a decisão de abrir o contactor; a abertura
 * física é propagada de imediato no super-loop (BMS_Relays_Task após processar o NFAULT).
 *
 * O detalhe técnico: Esta função é uma "Callback" de EXTI (External Interrupt) definida pela HAL da STMicroelectronics.
 * Esta função usa o mecanismo de "Funções Fracas" (__weak) da HAL. Quando o alarme
 * de HW dispara, o STM32 salta internamente para a ISR (EXTI9_5_IRQHandler), limpa a
 * flag de hardware e chama automaticamente esta Callback. Ao definirmos esta
 * função aqui, "sobrepomos" a função vazia da biblioteca da STMicroelectronics,
 * garantindo um tempo de reação de microssegundos de forma 100% invisível ao ciclo main!
 *
 * NOTA: só o NFAULT (PA8) usa EXTI. Os monitores de segurança (IMD/TSMS/ESDB/...)
 * em bms_relays.c são lidos por polling com debounce, para não colidir com esta
 * mesma linha EXTI9_5.
 */
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
 * =========================================================================
 * Via B: o IWDG é INICIALIZADO pelo CubeMX (MX_IWDG_Init em main.c, com
 * Prescaler=64 e Reload=250 → ~500 ms). A aplicação NÃO o inicializa; apenas
 * o refresca aqui. BMS_IWDG_Refresh é self-contained (hiwdg local): o
 * HAL_IWDG_Refresh só usa o campo .Instance (escreve a chave de reload no
 * registo KR), pelo que não precisa do handle global do main.c nem dos
 * parâmetros .Init. Mantém o driver desacoplado do main.c.
 *
 *
 * @brief  Alimenta o "Cão de Guarda" (IWDG - Independent Watchdog)
 *
 * Para que serve: Barreira de segurança ASIL-D suprema. O IWDG é um temporizador
 * de hardware isolado que reinicia o STM32 se o software "encravar" (freeze).
 *
 * Como funciona: O temporizador está configurado para "morder" ao fim de ~500ms.
 * O ciclo principal do BMS tem de chamar esta função constantemente para fazer o
 * reset ao relógio. A genialidade aqui é usar uma estrutura local `hiwdg`, pois a
 * HAL só precisa do ponteiro para o registo do hardware para enviar o comando de
 * refresh, mantendo este ficheiro independente do `main.c`!
 */
void BMS_IWDG_Refresh(void)
{
    IWDG_HandleTypeDef hiwdg = {0};
    hiwdg.Instance = IWDG;
    (void)HAL_IWDG_Refresh(&hiwdg);
}

/* =========================================================================
 * SECÇÃO: COMM CLEAR (Reset da State Machine do Receptor UART)
 * ========================================================================= */

/**
 * @brief  Comm Clear - Envia um sinal de "Break" para desencravar a comunicação
 *
 * Para que serve: É o "limpar a garganta" do sistema. Se houver ruído no cabo e os
 * chips perderem a sincronia dos bytes (ficando à espera de um fim de mensagem que
 * nunca chega), esta função força-os a limpar os buffers e recomeçar do zero.
 *
 * Como funciona: Transforma o pino TX num interruptor manual e puxa-o a 0V durante 18 µs.
 * A 1 Mbps, cada bit demora 1 µs. O protocolo UART normal usa no máximo ~9 bits seguidos a zero.
 * Ao forçar 18 zeros seguidos ("Break Condition"), os chips percebem a violação do
 * protocolo, abortam qualquer receção a meio e preparam-se para a próxima mensagem.
 */
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

/**
 * @brief  Power-On Self Test (POST) - Exame médico de admissão do BMS
 *
 * Para que serve: Barreira de segurança ASIL-D obrigatória antes de ligar a Alta Tensão.
 * O BMS testa os seus próprios componentes físicos e matemáticos para provar que está
 * saudável antes de sequer considerar fechar o contactor.
 *
 * Como funciona (Os 4 Testes):
 * 1. CRC Sanity: Verifica se a ALU do STM32 está sã, calculando um CRC conhecido ("123456789" = 0x4B37).
 * 2. Comms Sanity: Lê o registo ACTIVE_CELL de todos os slaves para garantir resposta e valor correto.
 * 3. ADC Sanity: Lê as 30 tensões. Se encontrar algo absurdo (< 1V ou > 4.5V), assume cabo partido.
 * 4. NFAULT Sanity: Garante que o pino físico de hardware alarm não disparou durante o arranque.
 * Se passar tudo, levanta a flag `post_passed = true`.
 */
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
