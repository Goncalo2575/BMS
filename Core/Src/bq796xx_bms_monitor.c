/**
 * @file    bq796xx_bms_monitor.c
 * @brief   BMS - Módulo de Monitorização, Protecções e Tratamento de Falhas
 *          Leitura de tensões, temperaturas, gestão de faults e recuperação de anel
 *
 * @version 2.0.0
 */

#include "bq796xx_bms.h"

/* =========================================================================
 * SECÇÃO 4: LEITURA DE TENSÕES DAS CÉLULAS
 * ========================================================================= */

/**
 * @brief  Converte os dois bytes raw do ADC para tensão em mV
 *         Resolução BQ79616: LSB = 190.73 µV
 *         Fórmula: V_mV = (raw * 1907) / 10000
 *         Erro máximo: < 1 mV em toda a gama (validado: 3000/3300/3600 mV)
 *
 * @param  hi_byte  Byte alto do registo VCx_HI
 * @param  lo_byte  Byte baixo do registo VCx_LO
 * @return Tensão em mV
 */
static inline uint16_t BMS_RawToMillivolts(uint8_t hi_byte, uint8_t lo_byte)
{
    uint16_t raw = ((uint16_t)hi_byte << 8U) | (uint16_t)lo_byte;
    /* 190.73 µV/LSB -> (raw * 1907) / 10000
     * Usa aritmética de 32 bits para evitar overflow (max raw=65535):
     * 65535 * 1907 = 124,985,285 -> dentro de uint32_t (max 4,294,967,295) */
    return (uint16_t)(((uint32_t)raw * 1907UL) / 10000UL);
}

/**
 * @brief  Lê as tensões de todas as células de um slave via Single Device Read
 *
 *  ENDEREÇO CORRECTO: REG_VCELL15_HI = 0x056A
 *  ERRO ANTERIOR:     REG_VC1_HI = 0x0042 — pertence ao BQ79606A-Q1, não ao
 *                     BQ79616-Q1. Ler de 0x0042 retornava dados de registos de
 *                     configuração sem qualquer relação com tensões de células.
 *
 *  MAPEAMENTO INVERTIDO OBRIGATÓRIO:
 *  Os registos ADC do BQ79616-Q1 estão em ordem DESCENDENTE no mapa de memória:
 *    rx_buf[0..1]   = VCELL15_HI/LO (célula 15 = topo activo)
 *    rx_buf[2..3]   = VCELL14_HI/LO
 *    ...
 *    rx_buf[28..29] = VCELL1_HI/LO  (célula 1 = base)
 *
 *  O array cell_voltage_mv[] usa índice ascendente (índice 0 = célula 1):
 *    cell_voltage_mv[c] = BMS_RawToMillivolts(rx_buf[(14-c)*2], rx_buf[(14-c)*2+1])
 *
 * @param  hbms         Handle do BMS
 * @param  slave_idx    Índice do slave (0 ou 1)
 * @return BMS_OK ou código de erro
 */
static BMS_Status_t BMS_ReadSlaveVoltages(BMS_Handle_t *hbms, uint8_t slave_idx)
{
    uint8_t      rx_buf[BMS_VCELL_READ_BYTES];  /* 30 bytes */
    BMS_Status_t status;
    uint8_t      slave_addr = hbms->slave[slave_idx].address;

    /* Lê 30 bytes contíguos a partir de VCELL15_HI (0x056A)
     * Cobre células 15 (topo activo) descendo até célula 1 (base).
     * VCELL16 em 0x0568 é omitido pois está fisicamente curto-circuitado a VC15. */
    status = BMS_ReadSingleDevice(hbms, slave_addr,
                                   REG_VCELL15_HI, rx_buf,
                                   (uint8_t)BMS_VCELL_READ_BYTES);
    if (status != BMS_OK)
    {
        hbms->slave[slave_idx].comm_ok = false;
        return status;
    }

    /* Mapeamento invertido: registos em ordem descendente (VCELL15 primeiro),
     * array em ordem ascendente (índice 0 = Cell 1).
     * Índice c=0 → Cell 1 → rx_buf[28..29] (byte offset = (14-0)*2 = 28)
     * Índice c=14 → Cell 15 → rx_buf[0..1]  (byte offset = (14-14)*2 = 0)  */
    for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
    {
        uint8_t buf_idx = (uint8_t)((BMS_CELLS_PER_SLAVE - 1U - c) * 2U);
        hbms->slave[slave_idx].cell_voltage_mv[c] =
            BMS_RawToMillivolts(rx_buf[buf_idx], rx_buf[buf_idx + 1U]);
    }
    hbms->slave[slave_idx].comm_ok = true;
    return BMS_OK;
}

/**
 * @brief  Lê tensões de todos os slaves e calcula métricas globais
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro (falha parcial não bloqueia)
 */
BMS_Status_t BMS_ReadAllCellVoltages(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint32_t pack_sum  = 0UL;  /* uint32 obrigatório: 30S × 3600 mV = 108 000 mV > UINT16_MAX */
    uint16_t min_mv    = 0xFFFFU;
    uint16_t max_mv    = 0U;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        status = BMS_ReadSlaveVoltages(hbms, s);
        if (status != BMS_OK)
        {
            /* BUG PROTOCOLO CORRIGIDO: A versão anterior tentava ler com
             * address_rev como "fallback" sem alterar DIR_SEL na bridge.
             * A bridge BQ79600 transmite estritamente via DIR0 ou DIR1
             * conforme o bit DIR_SEL no registo CONTROL1 — NÃO conforme
             * o endereço pedido pelo MCU.
             *
             * Consequência: address_rev do Slave1 = 0x02 = address DIR0 do
             * Slave2. O pedido com endereço 0x02 via DIR0 extraía dados do
             * Slave2, guardava-os na struct do Slave1 → telemetria cruzada
             * + colisão UART que corrompia state machines dos ADCs.
             *
             * Correcção: registar o erro, zerar dados stale, e deixar o
             * BMS_ProcessFaults → BMS_RingRecovery tratar a comutação de
             * DIR_SEL de forma atómica e completa. */
            hbms->fault_flags |= BMS_FAULT_COMM;
            memset(hbms->slave[s].cell_voltage_mv, 0,
                   sizeof(hbms->slave[s].cell_voltage_mv));
            continue;
        }

        /* Acumula métricas globais */
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t mv = hbms->slave[s].cell_voltage_mv[c];
            pack_sum += mv;
            if (mv < min_mv) { min_mv = mv; }
            if (mv > max_mv) { max_mv = mv; }
        }
    }

    hbms->pack_voltage_mv = pack_sum;
    hbms->min_cell_mv     = (min_mv == 0xFFFFU) ? 0U : min_mv;
    hbms->max_cell_mv     = max_mv;
    hbms->delta_cell_mv   = (max_mv > hbms->min_cell_mv) ?
                             (max_mv - hbms->min_cell_mv) : 0U;

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 5: LEITURA DE TEMPERATURAS
 * ========================================================================= */

/**
 * @brief  Tabela NTC corrigida — NTC 10 kΩ, β=3950 K, R_pull=10 kΩ, V_TSREF=5 V
 *         Pares {ratio_x10000 , temperatura_c}
 *         ratio = V_GPIO / V_TSREF = R_NTC / (R_NTC + R_pull)
 *
 *  VERIFICAÇÃO DO PONTO NOMINAL (25 °C):
 *    R_NTC(25°C) = 10 000 Ω  (valor nominal do componente)
 *    ratio = 10000 / (10000 + 10000) = 0,5000
 *    ratio_x10000 = 5000   ← entrada correcta na tabela abaixo
 *
 *  ERRO DA VERSÃO ANTERIOR:
 *    A tabela anterior tinha ratio_x10000=2207 para 25°C.
 *    Isso corresponde a R_pull ≈ 35,3 kΩ, não a 10 kΩ.
 *    Com R_pull=10 kΩ real, o ADC leria ratio≈5000; a pesquisa
 *    atingia o limite de clamping e reportava perpetuamente -20°C.
 *
 *  GERADA COM:
 *    R_NTC(T) = 10000 * exp(3950 * (1/T[K] - 1/298.15))
 *    ratio(T) = R_NTC(T) / (R_NTC(T) + 10000)
 *    Passo: 5°C de +80°C a -20°C — 21 pontos
 *
 *  COMO ADAPTAR A OUTRO HARDWARE:
 *    Substituir R_nominal, R_pull e beta pelos valores reais do esquemático
 *    e regenerar com o script Python incluído em bms_test_validation.c
 *    (função test_ntc_table_generation).
 */
typedef struct { uint16_t ratio_x10000; int8_t temp_c; } NTC_Point_t;

static const NTC_Point_t g_ntc_table[BMS_NTC_TABLE_SIZE] =
{
    /*  ratio_x10000   temp_°C       R_NTC (Ω)    */
    {  1127U,    80  },   /* R_NTC=     1270       */
    {  1298U,    75  },   /* R_NTC=     1492       */
    {  1496U,    70  },   /* R_NTC=     1760       */
    {  1726U,    65  },   /* R_NTC=     2086       */
    {  1991U,    60  },   /* R_NTC=     2486       */
    {  2295U,    55  },   /* R_NTC=     2978       */
    {  2641U,    50  },   /* R_NTC=     3588       */
    {  3030U,    45  },   /* R_NTC=     4348       */
    {  3465U,    40  },   /* R_NTC=     5301       */
    {  3941U,    35  },   /* R_NTC=     6506       */
    {  4456U,    30  },   /* R_NTC=     8037       */
    {  5000U,    25  },   /* R_NTC=    10000  <- ponto nominal */
    {  5563U,    20  },   /* R_NTC=    12535       */
    {  6130U,    15  },   /* R_NTC=    15837       */
    {  6686U,    10  },   /* R_NTC=    20175       */
    {  7216U,     5  },   /* R_NTC=    25925       */
    {  7708U,     0  },   /* R_NTC=    33621       */
    {  8149U,    -5  },   /* R_NTC=    44026       */
    {  8535U,   -10  },   /* R_NTC=    58246       */
    {  8862U,   -15  },   /* R_NTC=    77898       */
    {  9133U,   -20  },   /* R_NTC=   105385       */
};

/**
 * @brief  Converte medições ratiométricas GPIO/TSREF para temperatura em °C
 *
 *  MÉTODO CORRECTO (conforme especificação do fabricante):
 *  1. Ler V_GPIO  (canal NTC: pino polarizado por R_pull ligado a TSREF)
 *  2. Ler V_TSREF (referência interna de 5 V do integrado)
 *  3. ratio = V_GPIO_raw / V_TSREF_raw  (elimina erros de offset e ganho)
 *  4. Interpolar na tabela Steinhart-Hart para obter temperatura
 *
 *  Porque NÃO usar factor de linearidade simples:
 *  A curva R_NTC(T) é logarítmica (equação de Steinhart-Hart).
 *  Linearizar introduz erros de até ±10°C nas extremidades da gama.
 *
 * @param  gpio_raw   Leitura ADC do pino GPIO (16 bits)
 * @param  tsref_raw  Leitura ADC da referência TSREF (16 bits)
 * @return Temperatura em °C; INT8_MIN (-128) se conversão inválida
 */
static int16_t BMS_RawToTemperature_Ratiometric(uint16_t gpio_raw,
                                                  uint16_t tsref_raw)
{
    if (tsref_raw == 0U)
    {
        return INT8_MIN;   /* TSREF inválido - sensor desligado */
    }

    /* ratio = V_GPIO / V_TSREF, escalado x10000 para aritmética inteira */
    uint32_t ratio_x10000 = ((uint32_t)gpio_raw * 10000UL) / (uint32_t)tsref_raw;

    /* BUG TÉRMICO ASIL-D CORRIGIDO: Detecção de NTC desconectado / cabo partido.
     * Com pull-up de 10 kΩ ligado a TSREF, um NTC aberto puxa o GPIO para ~5V.
     * ratio = GPIO/TSREF ≈ 1.0 → ratio_x10000 ≈ 10000.
     * A tabela NTC só vai até 9133 (-20°C). Sem esta barreira, o clamp
     * inferior reportava -20°C → BMS pensava estar em ambiente gelado →
     * nenhuma protecção térmica → risco de Thermal Runaway em pista.
     *
     * Limiar 9500: corresponde a R_NTC > 190 kΩ, ou seja T < -30°C.
     * Fisicamente impossível num veículo em operação (excluindo Antárctida).
     * Retornar 127°C garante que BMS_CheckProtections detecta OT imediato. */
    if (ratio_x10000 > 9500U)
    {
        return (int16_t)127;   /* Força OT → contactor abre */
    }

    /* Clamp para os extremos da tabela */
    if (ratio_x10000 <= (uint32_t)g_ntc_table[0].ratio_x10000)
    {
        return (int16_t)g_ntc_table[0].temp_c;
    }
    if (ratio_x10000 >= (uint32_t)g_ntc_table[BMS_NTC_TABLE_SIZE - 1U].ratio_x10000)
    {
        return (int16_t)g_ntc_table[BMS_NTC_TABLE_SIZE - 1U].temp_c;
    }

    /* Pesquisa binária + interpolação linear entre dois pontos da tabela */
    uint8_t lo = 0U;
    uint8_t hi = BMS_NTC_TABLE_SIZE - 1U;

    while ((hi - lo) > 1U)
    {
        uint8_t mid = (uint8_t)((lo + hi) / 2U);
        if (ratio_x10000 < (uint32_t)g_ntc_table[mid].ratio_x10000)
        {
            hi = mid;
        }
        else
        {
            lo = mid;
        }
    }

    /* Interpolação linear entre g_ntc_table[lo] e g_ntc_table[hi] */
    uint32_t r_lo = (uint32_t)g_ntc_table[lo].ratio_x10000;
    uint32_t r_hi = (uint32_t)g_ntc_table[hi].ratio_x10000;
    int32_t  t_lo = (int32_t)g_ntc_table[lo].temp_c;
    int32_t  t_hi = (int32_t)g_ntc_table[hi].temp_c;

    /* temp = t_lo + (t_hi - t_lo) * (ratio - r_lo) / (r_hi - r_lo) */
    int32_t temp_c = t_lo + ((t_hi - t_lo) * (int32_t)(ratio_x10000 - r_lo)) /
                              (int32_t)(r_hi - r_lo);

    return (int16_t)temp_c;
}

/**
 * @brief  Lê temperaturas de 3 NTCs por slave + tensão HV do barramento
 *
 *  Leitura única de 10 bytes a partir de GPIO1_HI (0x0588):
 *    bytes[0:1] = GPIO1 (NTC sensor 1)
 *    bytes[2:3] = GPIO2 (NTC sensor 2)
 *    bytes[4:5] = GPIO3 (NTC sensor 3)
 *    bytes[6:7] = GPIO4 (HV bus, usado por BMS_ReadInverterVoltage)
 *    bytes[8:9] = TSREF (referência 5V raciométrica)
 *
 *  Todos os NTCs partilham o mesmo TSREF → conversão raciométrica consistente.
 *  max_temp_c = máximo de todas as 6 leituras (3 × 2 slaves).
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro
 */
BMS_Status_t BMS_ReadAllTemperatures(BMS_Handle_t *hbms)
{
    uint8_t  aux_buf[BMS_AUX_READ_BYTES];  /* 10 bytes: GPIO1-4 + TSREF */
    int16_t  max_temp = -100;
    BMS_Status_t status;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        /* Uma única transacção lê GPIO1, GPIO2, GPIO3, GPIO4, TSREF */
        status = BMS_ReadSingleDevice(hbms, hbms->slave[s].address,
                                       REG_GPIO1_HI, aux_buf,
                                       BMS_AUX_READ_BYTES);
        if (status != BMS_OK) { continue; }

        /* TSREF raw (bytes 8:9) — partilhado por todos os canais */
        uint16_t tsref_raw = ((uint16_t)aux_buf[8] << 8U) | (uint16_t)aux_buf[9];

        /* Conversão NTC para cada um dos 3 sensores */
        for (uint8_t ch = 0U; ch < BMS_NUM_TEMP_SENSORS; ch++)
        {
            uint16_t gpio_raw = ((uint16_t)aux_buf[ch * 2U] << 8U) |
                                 (uint16_t)aux_buf[ch * 2U + 1U];
            hbms->slave[s].temperatures_c[ch] =
                BMS_RawToTemperature_Ratiometric(gpio_raw, tsref_raw);

            if (hbms->slave[s].temperatures_c[ch] > max_temp)
            {
                max_temp = hbms->slave[s].temperatures_c[ch];
            }
        }
    }
    hbms->max_temp_c = max_temp;
    return BMS_OK;
}

/**
 * @brief  Lê a tensão do barramento HV via GPIO4 do Slave 1
 *
 *  GPIO4 recebe a saída de um divisor resistivo que atem a tensão HV
 *  para a gama 0-5V do ADC do BQ79616. A leitura usa o mesmo bloco
 *  aux_buf já processado em BMS_ReadAllTemperatures; aqui relemos o
 *  canal individualmente para isolamento lógico.
 *
 *  V_HV_mV = (gpio4_raw × 1907 / 10000) × HV_BUS_ATTENUATION_RATIO
 *
 *  Nota: A leitura é feita no Slave 1 (base da pilha, input do bus HV).
 *        Ajustar para o slave correcto conforme o layout do hardware.
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro
 */
BMS_Status_t BMS_ReadInverterVoltage(BMS_Handle_t *hbms)
{
    uint8_t buf[2U];
    BMS_Status_t status;

    /* Lê apenas GPIO4 do Slave 1 */
    status = BMS_ReadSingleDevice(hbms, hbms->slave[0].address,
                                   REG_GPIO4_HI, buf, 2U);
    if (status != BMS_OK) { return status; }

    uint16_t raw = ((uint16_t)buf[0] << 8U) | (uint16_t)buf[1];
    /* Converter raw ADC → tensão em mV com a resolução do BQ79616 */
    uint32_t vadc_mv = (uint32_t)(((uint32_t)raw * 1907UL) / 10000UL);
    /* Aplicar rácio de atenuação do hardware */
    hbms->inverter_voltage_mv = vadc_mv * (uint32_t)HV_BUS_ATTENUATION_RATIO;

    hbms->precharge_ready =
        (hbms->inverter_voltage_mv >= PRECHARGE_THRESHOLD_MV);

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 6: VERIFICAÇÃO DE PROTECÇÕES SOFTWARE
 * ========================================================================= */

/**
 * @brief  Verifica limites de tensão e temperatura por software
 *         Complementa as protecções de hardware do BQ79616
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK se tudo OK, BMS_ERR_FAULT_ACTIVE se há fault
 */
BMS_Status_t BMS_CheckProtections(BMS_Handle_t *hbms)
{
    bool fault_detected = false;

    /* D10 NOTA DE DESIGN: fault_flags acumulam via OR sem reset cíclico.
     * Isto é INTENCIONAL (latch-by-design):
     * - Uma célula que leia momentaneamente 3601 mV por ruído ADC activa
     *   BMS_FAULT_OV e transita para FAULT, exigindo recovery explícito.
     * - Numa arquitectura ASIL-D, um transitório térmico/eléctrico real
     *   é indistinguível de ruído sem análise temporal — o sistema opta
     *   pela segurança máxima (fail-safe → recovery explícito).
     * - Se for necessário protecção reactiva (fault dura apenas enquanto a
     *   condição persistir), descomentar o bloco de limpeza abaixo:
     *
     * // hbms->fault_flags &= ~(BMS_FAULT_OV | BMS_FAULT_UV | BMS_FAULT_OT);
     * // for (s = 0; s < BMS_NUM_SLAVES; s++)
     * //     hbms->slave[s].fault_flags &= ~(BMS_FAULT_OV|BMS_FAULT_UV|BMS_FAULT_OT);
     */

    /* --- Verificação por célula --- */
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t mv = hbms->slave[s].cell_voltage_mv[c];

            /* Over-Voltage */
            if (mv >= CELL_OV_MV)
            {
                hbms->slave[s].ov_cell[c] = true;
                hbms->slave[s].fault_flags |= BMS_FAULT_OV;
                hbms->fault_flags |= BMS_FAULT_OV;
                fault_detected = true;
            }
            else
            {
                hbms->slave[s].ov_cell[c] = false;
            }

            /* Under-Voltage
             * BUG ASIL-D CORRIGIDO: removido o guard (mv > 0U).
             * Com ACTIVE_CELL=15, todos os canais ADC são activos.
             * Uma leitura de 0 mV indica célula morta, curto-circuito total
             * ou cabo de sensor cortado — condição letal que DEVE despoletar
             * a abertura do contactor. O guard anterior mascarava esta falha. */
            if (mv <= CELL_UV_MV)
            {
                hbms->slave[s].uv_cell[c] = true;
                hbms->slave[s].fault_flags |= BMS_FAULT_UV;
                hbms->fault_flags |= BMS_FAULT_UV;
                fault_detected = true;
            }
            else
            {
                hbms->slave[s].uv_cell[c] = false;
            }
        }

        /* Over-Temperature */
        int16_t slave_max_t = hbms->slave[s].temperatures_c[0];
        for (uint8_t t = 1U; t < BMS_NUM_TEMP_SENSORS; t++) {
            if (hbms->slave[s].temperatures_c[t] > slave_max_t)
                slave_max_t = hbms->slave[s].temperatures_c[t];
        }
        if (slave_max_t >= (int16_t)CELL_TEMP_MAX_C)
        {
            hbms->slave[s].fault_flags |= BMS_FAULT_OT;
            hbms->fault_flags |= BMS_FAULT_OT;
            fault_detected = true;
        }
    }

    /* --- Verificação de desequilíbrio --- */
    if (hbms->delta_cell_mv > CELL_IMBALANCE_MV)
    {
        /* Desequilíbrio grave - regista mas não actua imediatamente */
        /* Pode acionar cell balancing se implementado */
    }

    if (fault_detected)
    {
        /* BUG CRÍTICO CORRIGIDO: A versão anterior apenas transitava para
         * BMS_STATE_FAULT sem actuar no hardware. Faults de software (OT,
         * UV por leitura ADC) não disparam NFAULT — são invisíveis ao ISR.
         * Sem esta linha, o contactor permanecia fechado com corrente a fluir
         * num pack a 65°C → Thermal Runaway.
         * BMS_EmergencyShutdown: abre contactor + deasserta BMS_OK_PIN. */
        BMS_EmergencyShutdown(hbms);
        hbms->state = BMS_STATE_FAULT;
        return BMS_ERR_FAULT_ACTIVE;
    }

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 7: TRATAMENTO DE FALHAS - ISR E PROCESSAMENTO
 * ========================================================================= */

/**
 * @brief  Callback de interrupção NFAULT (EXTI, nível baixo, não mascarável)
 *         Chamada directamente pelo HAL_GPIO_EXTI_Callback
 *
 *  SEGURANÇA ASIL-D: o contactor deve abrir com latência < 24 µs após NFAULT.
 *  A abertura DIRECTA aqui (sem esperar pela tarefa de 100 ms) é OBRIGATÓRIA
 *  para cumprir o ciclo de detecção OV/UV de 8 ms dos comparadores HW.
 */
void BMS_NFAULT_IRQHandler(BMS_Handle_t *hbms)
{
    if (hbms != NULL)
    {
        /* BUG-06: Desassertar BMS_OK_PIN imediatamente na ISR (latência < 1 µs).
         * Anteriormente o pino só era actualizado em BMS_UpdateHardwareInterlocks()
         * no ciclo de 100 ms → até 100 ms de sinal "OK" falso para a VCU após fault.
         * ASIL-D exige propagação imediata do estado de falha ao hardware interlock. */
        HAL_GPIO_WritePin(BMS_OK_PORT, BMS_OK_PIN, GPIO_PIN_RESET);

        /* Abre o contactor imediatamente */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        hbms->contactor_closed = false;

        /* OBS-04: __atomic_store_n para consistência com a doutrina atómica
         * do codebase (nfault_pending é lido com __atomic_load_n e limpo com
         * __atomic_exchange_n — a escrita deve usar a mesma barreira). */
        __atomic_store_n(&hbms->nfault_pending, 1U, __ATOMIC_SEQ_CST);
        hbms->fault_count++;
    }
}

/**
 * @brief  Lê e interpreta os registos FAULT_SUMMARY de todos os dispositivos
 *         Deve ser chamada no contexto de tarefa (não na ISR)
 *
 *  ACESSO ATÓMICO À FLAG nfault_pending:
 *  Problema de race condition sem protecção:
 *    1. Thread lê nfault_pending == 1 (condição verdadeira)
 *    2. ISR dispara -> escreve nfault_pending = 1 (novo evento)
 *    3. Thread escreve nfault_pending = 0  <- NOVO EVENTO PERDIDO silenciosamente
 *
 *  Solução Cortex-M4: LDREX/STREX (Load/Store Exclusive) implementado por
 *  __atomic_exchange_n() do GCC, que gera exactamente estas instruções.
 *  Equivalente a um compare-and-swap atómico.
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou BMS_ERR_FAULT_ACTIVE
 */
BMS_Status_t BMS_ProcessFaults(BMS_Handle_t *hbms)
{
    /* Leitura e limpeza atómica: se a flag estava a 1 retorna 1 e coloca 0
     * Se entretanto uma ISR colocar a 1 durante esta instrução, STREX falha
     * e a instrução repete; o valor 1 não se perde em nenhum cenário.
     * __atomic_exchange_n() compila para LDREX/STREX no Cortex-M4 com GCC. */
    uint32_t was_pending = __atomic_exchange_n(&hbms->nfault_pending, 0U,
                                                __ATOMIC_SEQ_CST);
    if (was_pending == 0U)
    {
        return BMS_OK;
    }

    /* Variáveis locais declaradas após o guard atómico (C89 compat) */
    uint8_t      fault_buf[4U];
    BMS_Status_t status;
    bool         critical_fault = false;

    /* --- Leitura FAULT_SUMMARY da Bridge --- */
    status = BMS_ReadSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                   REG_BRIDGE_FAULT_SUMMARY, fault_buf, 2U);
    if (status == BMS_OK)
    {
        uint8_t bridge_fault_comm2 = fault_buf[1];

        /* Bit de Ring Break */
        if (bridge_fault_comm2 & 0x01U)
        {
            hbms->fault_flags |= BMS_FAULT_RING_BREAK;
            hbms->ring_intact   = false;
        }

        /* Heartbeat Fail */
        if (bridge_fault_comm2 & 0x40U)
        {
            hbms->fault_flags |= BMS_FAULT_HB_FAIL;
        }
    }

    /* --- Broadcast Read FAULT_SUMMARY dos Slaves ---
     * BUG CRÍTICO CORRIGIDO: A versão anterior lia 3 bytes a partir de 0x052D:
     *   sf[0] = FAULT_SUMMARY (0x052D) ← correcto
     *   sf[1] = FAULT_OV (0x052E)      ← ERRADO: máscara OV das células 1-8
     *   sf[2] = FAULT_UV (0x052F)      ← ERRADO: máscara UV das células 1-8
     * A instrução `sf[2] & 0x10U` testava o bit 4 do FAULT_UV = "Célula 5 com UV"
     * e interpretava-o como "COMM fault / ring break". Se a Célula 5 caísse
     * abaixo de 3.0V, o BMS declarava rotura de cabo e invertia o anel!
     *
     * Correcção: ler apenas 1 byte (FAULT_SUMMARY) e usar os bits correctos:
     *   BQ79616 FAULT_SUMMARY: Bit2=OV, Bit3=UV, Bit4=COMM, Bit5=SYS, etc.
     *   (os bits 0x01 e 0x02 usados antes correspondem a reservados/outros). */
    uint8_t slave_fault_data[BMS_NUM_SLAVES * 1U];
    status = BMS_ReadBroadcast(hbms, REG_SLAVE_FAULT_SUMMARY,
                                slave_fault_data, 1U);
    if (status == BMS_OK)
    {
        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            uint8_t sf = slave_fault_data[s];
            hbms->slave[s].fault_summary_raw[0] = sf;

            if (sf & 0x04U)   /* Bit 2: OV fault */
            {
                hbms->slave[s].fault_flags |= BMS_FAULT_OV;
                hbms->fault_flags |= BMS_FAULT_OV;
                critical_fault = true;
            }
            if (sf & 0x08U)   /* Bit 3: UV fault */
            {
                hbms->slave[s].fault_flags |= BMS_FAULT_UV;
                hbms->fault_flags |= BMS_FAULT_UV;
                critical_fault = true;
            }
            if (sf & 0x10U)   /* Bit 4: COMM fault → possível ring break */
            {
                hbms->slave[s].fault_flags |= BMS_FAULT_COMM;
                hbms->fault_flags |= BMS_FAULT_RING_BREAK;
            }
        }
    }
    else
    {
        /* Falha na leitura Broadcast -> possível rotura do anel */
        hbms->fault_flags |= BMS_FAULT_RING_BREAK;
        hbms->ring_intact   = false;
    }

    /* --- Verificação de Open Wire (FAULT_COMP_VCOW) ---
     * BUG-1 CORRIGIDO: leitura de 2U bytes a partir de 0x0530 capturava também
     * o byte 0x0531 (FAULT_COMM). Uma falha transiente de CRC activa FAULT_COMM;
     * o código confundia-o com cabo partido → BMS_FAULT_OPEN_WIRE → bloqueio
     * permanente (Open Wire é não-recuperável em BMS_FaultRecoveryAttempt).
     * Solução: ler apenas 1 byte — exactamente FAULT_VCOW, sem adjacências. */
    uint8_t vcow_buf[BMS_NUM_SLAVES * 1U];
    if (BMS_ReadBroadcast(hbms, REG_SLAVE_FAULT_VCOW, vcow_buf, 1U) == BMS_OK)
    {
        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            if (vcow_buf[s] != 0U)
            {
                hbms->slave[s].fault_flags |= BMS_FAULT_OPEN_WIRE;
                hbms->fault_flags          |= BMS_FAULT_OPEN_WIRE;
            }
        }
    }

    /* --- Acções de Segurança --- */
    if (critical_fault ||
        (hbms->fault_flags & (BMS_FAULT_OV | BMS_FAULT_UV | BMS_FAULT_OT)))
    {
        BMS_EmergencyShutdown(hbms);
    }
    else if (hbms->fault_flags & BMS_FAULT_RING_BREAK)
    {
        /* Tentar recuperação do anel antes de desligar */
        BMS_Status_t ring_status = BMS_RingRecovery(hbms);
        if (ring_status != BMS_OK)
        {
            BMS_EmergencyShutdown(hbms);
        }
    }

    hbms->state = (hbms->fault_flags != 0U) ? BMS_STATE_FAULT : BMS_STATE_MONITORING;

    return (hbms->fault_flags != 0U) ? BMS_ERR_FAULT_ACTIVE : BMS_OK;
}

/* =========================================================================
 * SECÇÃO 8: RECUPERAÇÃO DE ANEL (RING BREAK RECOVERY)
 * ========================================================================= */

/**
 * @brief  Recuperação após rotura do anel de comunicação
 *
 *  PARADOXO LÓGICO CORRIGIDO:
 *  A versão anterior usava ring_intact para duas finalidades opostas:
 *   (A) Sinalizar que o anel está partido (ring_intact = false)
 *   (B) Guardar a tentativa de leitura pelo caminho reverso
 *       (if (hbms->ring_intact) { ... usa address_rev ... })
 *  Resultado: quando ring_intact = false (anel partido), a condição (B)
 *  era FALSE e o caminho reverso NUNCA era tentado -> estado inacessível.
 *
 *  SOLUÇÃO: Dois campos separados no handle:
 *   ring_intact       = estado físico do anel completo (true/false)
 *   ring_using_reverse = indica que o firmware está a usar o caminho reverso
 *
 *  A leitura de tensões usa address[] se !ring_using_reverse,
 *  ou address_rev[] se ring_using_reverse, independentemente de ring_intact.
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK se pelo menos um slave for acessível pelo caminho reverso
 */
BMS_Status_t BMS_RingRecovery(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t data;

    hbms->state = BMS_STATE_RING_RECOVERY;
    hbms->ring_recovery_count++;
    hbms->ring_intact = false;   /* Confirma rotura física */

    /* Comuta DIR_SEL para o caminho reverso na Bridge */
    data = CTRL1_DIR_SEL;  /* 0x02 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK)
    {
        /* Bridge inacessível - falha total */
        return BMS_ERR_COMM;
    }
    BMS_DelayUs(hbms, DELAY_DIR_SEL_SWITCH_US);  /* 100 µs */

    /* Testa comunicação com ambos os slaves pelo caminho reverso
     * NOTA: usa address_rev[] directamente - NÃO condicionado a ring_intact */
    uint8_t test_buf[1U];
    bool slave0_rev_ok = (BMS_ReadSingleDevice(hbms, hbms->slave[0].address_rev,
                                                REG_ACTIVE_CELL, test_buf, 1U) == BMS_OK);
    bool slave1_rev_ok = (BMS_ReadSingleDevice(hbms, hbms->slave[1].address_rev,
                                                REG_ACTIVE_CELL, test_buf, 1U) == BMS_OK);

    if (slave0_rev_ok || slave1_rev_ok)
    {
        /* Caminho reverso funcional: activar routing reverso permanente
         * As funções de leitura passam a usar address_rev[] até próxima init */
        hbms->ring_using_reverse = true;
        hbms->fault_flags &= ~BMS_FAULT_RING_BREAK;
        /* BUG-3 CORRIGIDO: remover injecção de BMS_FAULT_COMM aqui.
         * O ring foi restabelecido com sucesso via DIR1 — a visibilidade
         * celular está restaurada. Injectar FAULT_COMM forçava a avaliação
         * final em BMS_ProcessFaults (fault_flags != 0) a colocar o estado
         * em FAULT, impedindo o contactor de se manter fechado e o veículo
         * de operar, anulando todo o propósito do Ring Recovery.
         * O modo degradado (anel físico partido) é comunicado à VCU pelo
         * campo ring_intact=false no pacote de telemetria, não por fault_flag. */

        /* Actualizar endereços activos para o caminho reverso */
        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            uint8_t tmp         = hbms->slave[s].address;
            hbms->slave[s].address     = hbms->slave[s].address_rev;
            hbms->slave[s].address_rev = tmp;  /* Guardar original para re-init */
        }

        hbms->state = BMS_STATE_MONITORING;
        return BMS_OK;
    }

    /* Ambos os caminhos inacessíveis - falha total */
    hbms->ring_using_reverse = false;
    hbms->fault_flags |= BMS_FAULT_COMM;
    hbms->state = BMS_STATE_FAULT;
    return BMS_ERR_COMM;
}

/* =========================================================================
 * SECÇÃO 9: CONTROLO DE POTÊNCIA E CONTACTOR
 * ========================================================================= */

/**
 * @brief  Abre o contactor de potência (desliga a bateria)
 *         O pino do contactor deve ser configurado na inicialização do projecto
 *         Exemplo: GPIOB_PIN_12 ligado ao relay/MOSFET gate do contactor
 */
void BMS_ContactorOpen(BMS_Handle_t *hbms)
{
    /* Parar balanceamento ANTES de abrir o contactor.
     * Com o contactor aberto, as células ficam isoladas do exterior.
     * Continuar a balancear sem via de dissipação térmica é inseguro. */
    if (hbms->is_balancing)
    {
        (void)BMS_StopAllBalancing(hbms);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    hbms->contactor_closed = false;
}

/**
 * @brief  Fecha o contactor de potência (liga a bateria)
 *
 *  PRÉ-CONDIÇÕES DE SEGURANÇA (todas têm de ser verdadeiras):
 *
 *  [1] nfault_pending == 0  — BARREIRA CRÍTICA ASIL-D
 *      Impede o fecho quando existe um evento de hardware NÃO processado.
 *
 *      CENÁRIO DE FALHA SEM ESTA BARREIRA (race condition no superloop):
 *        t0: BMS_Task_100ms() termina sem falhas. state=MONITORING, fault_flags=0.
 *        t1: Evento OV físico → EXTI13 → BMS_NFAULT_IRQHandler():
 *              abre contactor, escreve nfault_pending=1.
 *        t2: Superloop chama BMS_ContactorControl():
 *              state==MONITORING && fault_flags==0 → tenta fechar.
 *        t3: SEM esta barreira → BMS_ContactorClose() executaria HAL_GPIO_WritePin
 *              HIGH → contactor fecha sobre barramento OV instável → thermal runaway.
 *        t4: Só no ciclo seguinte é que BMS_ProcessFaults() leria o NFAULT.
 *            Janela de religamento indevido: até 100 ms. Suficiente para
 *            embalamento térmico num pack de 15 células em série.
 *
 *      COM esta barreira: t2 deteta nfault_pending!=0 e retorna imediatamente.
 *      O contactor permanece aberto até BMS_ProcessFaults() dissecar o evento.
 *
 *  [2] fault_flags == 0      — sem falhas mapeadas pelo software
 *  [3] state == MONITORING   — máquina de estados em operação normal
 *  [4] min_cell_mv >= CELL_UV_MV && max_cell_mv <= CELL_OV_MV — tensões OK
 */
void BMS_ContactorClose(BMS_Handle_t *hbms)
{
    /* ---------------------------------------------------------------
     * BARREIRA 1 (CRÍTICA): evento NFAULT de hardware ainda por processar
     * A leitura atómica não limpa a flag — apenas verifica a presença.
     * Enquanto nfault_pending != 0, o fecho é incondicionalmente proibido,
     * independentemente de qualquer outro estado do sistema.
     * --------------------------------------------------------------- */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return;   /* Abortar: evento de hardware não dissecado */
    }

    /* BARREIRA 2: sem falhas mapeadas pelo software */
    if (hbms->fault_flags != 0U)
    {
        return;
    }

    /* BARREIRA 3: máquina de estados em operação normal */
    if (hbms->state != BMS_STATE_MONITORING)
    {
        return;
    }

    /* BARREIRA 4: tensões celulares dentro dos limites operacionais */
    if ((hbms->min_cell_mv < CELL_UV_MV) ||
        (hbms->max_cell_mv > CELL_OV_MV))
    {
        return;
    }

    /* Todas as barreiras ultrapassadas — fecho autorizado */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    hbms->contactor_closed = true;
}

/**
 * @brief  Shutdown de emergência - abre contactor e sinaliza estado FAULT
 *         Chamado em condições críticas (OV, UV, OT, Ring fail total)
 *
 * @param  hbms     Handle do BMS
 */
void BMS_EmergencyShutdown(BMS_Handle_t *hbms)
{
    /* Abre o contactor imediatamente (pára balanceamento + GPIO RESET) */
    BMS_ContactorOpen(hbms);

    /* Desassertar BMS_OK_PIN directamente.
     * BUG DE SEGURANÇA CORRIGIDO: para faults de software (ex: OT, que não tem
     * comparador de hardware), BMS_CheckProtections chama esta função e retorna
     * BMS_ERR_FAULT_ACTIVE — BMS_Task_100ms retorna cedo, saltando
     * BMS_UpdateHardwareInterlocks. Nos ciclos seguintes, o guard de FAULT
     * volta a saltar essa função. Sem esta linha, BMS_OK_PIN ficaria preso em
     * HIGH após um fault de software e a VCU veria "BMS OK" com o contactor
     * aberto. Espelha o comportamento da ISR NFAULT para faults de hardware. */
    HAL_GPIO_WritePin(BMS_OK_PORT, BMS_OK_PIN, GPIO_PIN_RESET);

    /* Actualiza estado */
    hbms->state = BMS_STATE_FAULT;

    /* Opcional: sinalização LED ou log de evento */
    /* BMS_Log_Event(hbms, "EMERGENCY SHUTDOWN", hbms->fault_flags); */
}

/* =========================================================================
 * SECÇÃO 10: TAREFA CÍCLICA PRINCIPAL (100 ms)
 * ========================================================================= */

/**
 * @brief  Tarefa de monitorização cíclica a 100 ms
 *         Deve ser chamada periodicamente (RTOS task, timer callback, ou superloop)
 *
 *  Sequência por ciclo:
 *  1. Verificar flag NFAULT pendente
 *  2. Ler tensões de todas as células
 *  3. Ler temperaturas
 *  4. Verificar protecções por software
 *  5. Controlo do contactor
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro
 */
BMS_Status_t BMS_Task_100ms(BMS_Handle_t *hbms)
{
    BMS_Status_t status = BMS_OK;

    if (hbms == NULL) { return BMS_ERR_INVALID_PARAM; }

    /* --- Processar NFAULT pendente (prioridade máxima) ---
     * Usa __atomic_load_n para leitura consistente da flag volatile.
     * NÃO limpa aqui — a limpeza atómica é feita dentro de
     * BMS_ProcessFaults() via __atomic_exchange_n, garantindo que
     * um evento que chegue entre esta leitura e o processamento
     * não é perdido silenciosamente. */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        BMS_ProcessFaults(hbms);
        /* Contactor já foi aberto na ISR. Se estado=FAULT, interrompe ciclo. */
        if (hbms->state == BMS_STATE_FAULT)
        {
            return BMS_ERR_FAULT_ACTIVE;
        }
    }

    /* --- Estado FAULT / SHUTDOWN / SLEEP / UNINITIALIZED: não efectuar polling ---
     * BUG-2 CORRIGIDO: a versão anterior só excluía BMS_STATE_FAULT.
     * Com SHUTDOWN apenas como flag de variável (sem hardware dormido),
     * BMS_CheckProtections detectava a falha, repunha state=FAULT e o
     * super-loop reiniciava os contadores → ciclo infinito de tentativas.
     * SLEEP e UNINITIALIZED têm ADCs inactivos: medir seria ler lixo. */
    if ((hbms->state == BMS_STATE_FAULT)        ||
        (hbms->state == BMS_STATE_SHUTDOWN)     ||
        (hbms->state == BMS_STATE_SLEEP)        ||
        (hbms->state == BMS_STATE_UNINITIALIZED))
    {
        return BMS_ERR_FAULT_ACTIVE;
    }

    /* --- Leitura de tensões (ÚNICA — sem leitura redundante UART) ---
     * AUDITORIA v3.1: A "leitura redundante" (ler 2× o mesmo registo ADC) foi
     * removida. O ADC do BQ79616-Q1 em modo contínuo latches o resultado a uma
     * taxa fixa de dizimação interna. Duas leituras UART consecutivas separadas
     * por ~3 µs extraem fisicamente a MESMA conversão — a comparação valida
     * apenas a integridade do link UART, que o CRC-16/MODBUS já garante.
     * Redundância ASIL-D real requer o ADC de diagnóstico secundário do silício
     * (canal independente), não a releitura do mesmo registo. */
    status = BMS_ReadAllCellVoltages(hbms);
    if (status != BMS_OK)
    {
        hbms->comm_error_count++;
    }

    /* --- Leitura de temperaturas --- */
    status = BMS_ReadAllTemperatures(hbms);

    /* --- Tensão HV do barramento (pré-carga) --- */
    (void)BMS_ReadInverterVoltage(hbms);

    /* --- Verificação de protecções (dados RAW obrigatórios) ---
     * AUDITORIA v3.1: BMS_CheckProtections opera sobre dados crus (não filtrados).
     * O filtro IIR (Y = (X + 3×Y_prev)/4) introduz atraso de grupo que pode
     * mascarar transitórios reais. Se o comparador OV de hardware (NFAULT) disparar
     * por um pico que o filtro IIR suaviza, a telemetria SW não corrobora o evento
     * → mismatch HW/SW nos registos de diagnóstico post-mortem.
     * FTTI (Fault Tolerant Time Interval) exige que SW reaja ao mesmo transitório
     * que o HW, sem filtragem adicional na cadeia de segurança. */
    status = BMS_CheckProtections(hbms);
    if (status == BMS_ERR_FAULT_ACTIVE)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }

    /* --- Filtro IIR (APÓS protecções — apenas para telemetria e display) ---
     * Os dados filtrados são usados por:
     *   - BMS_RunPassiveBalancing (decisão de delta suavizada)
     *   - BMS_MasterComm_Task_100ms (telemetria VCU, ecrã de cabina)
     *   - BMS_PrintStatus (debug)
     * NÃO são usados por BMS_CheckProtections (que já executou com raw acima). */
    BMS_ApplyVoltageFilter(hbms);

    /* --- SoC (apenas em relaxação termodinâmica) ---
     * AUDITORIA v3.1: A curva OCV do LiFePO4 é assintoticamente plana entre
     * 20-80% de SoC com histerese significativa. Durante operação (contactor
     * fechado), V_terminal = OCV ± (I × R_interna). A queda IR sob carga faz
     * o SoC OCV oscilar caoticamente (ex: aceleração → -30% instantâneo).
     *
     * SOLUÇÃO: OCV lookup só é válido em relaxação (corrente ≈ 0 durante
     * período mínimo de ~30 minutos). Com contactor fechado, mantém o
     * último SoC válido. Para estimação dinâmica sob carga, implementar
     * Coulomb Counting ou Extended Kalman Filter (requer sensor de corrente).
     *
     * Proxy sem sensor de corrente: contactor_closed == false implica I ≈ 0. */
    if (!hbms->contactor_closed)
    {
        if ((hbms->min_cell_mv > 0U) && (hbms->max_cell_mv > 0U))
        {
            uint16_t avg_mv = (uint16_t)(hbms->pack_voltage_mv /
                                          (uint32_t)BMS_TOTAL_CELLS);
            hbms->soc_percent = BMS_EstimateSoC(avg_mv);
        }
    }
    /* Com contactor fechado: hbms->soc_percent mantém o último valor válido */

    /* --- Balanceamento celular passivo --- */
    (void)BMS_RunPassiveBalancing(hbms);

    /* --- Actualizar saídas digitais de interlock (BMS_OK, PRECHARGE_OK) --- */
    BMS_UpdateHardwareInterlocks(hbms);

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 11: UTILITÁRIOS DE DIAGNÓSTICO
 * ========================================================================= */

/**
 * @brief  Retorna string legível para o estado do BMS
 */
const char *BMS_GetStateString(BMS_State_t state)
{
    switch (state)
    {
        case BMS_STATE_UNINITIALIZED:  return "UNINITIALIZED";
        case BMS_STATE_INITIALIZING:   return "INITIALIZING";
        case BMS_STATE_IDLE:           return "IDLE";
        case BMS_STATE_MONITORING:     return "MONITORING";
        case BMS_STATE_BALANCING:      return "BALANCING";
        case BMS_STATE_FAULT:          return "FAULT";
        case BMS_STATE_SHUTDOWN:       return "SHUTDOWN";
        case BMS_STATE_SLEEP:          return "SLEEP";
        case BMS_STATE_RING_RECOVERY:  return "RING_RECOVERY";
        default:                       return "UNKNOWN";
    }
}

/**
 * @brief  Retorna string legível para os flags de fault (primeiro activo)
 */
const char *BMS_GetFaultString(uint32_t fault_flags)
{
    if (fault_flags == BMS_FAULT_NONE)    { return "NONE"; }
    if (fault_flags & BMS_FAULT_OV)       { return "OVER_VOLTAGE"; }
    if (fault_flags & BMS_FAULT_UV)       { return "UNDER_VOLTAGE"; }
    if (fault_flags & BMS_FAULT_OT)       { return "OVER_TEMPERATURE"; }
    if (fault_flags & BMS_FAULT_RING_BREAK) { return "RING_BREAK"; }
    if (fault_flags & BMS_FAULT_OPEN_WIRE) { return "OPEN_WIRE"; }
    if (fault_flags & BMS_FAULT_COMM)     { return "COMM_ERROR"; }
    if (fault_flags & BMS_FAULT_HB_FAIL)  { return "HEARTBEAT_FAIL"; }
    if (fault_flags & BMS_FAULT_CRC)      { return "CRC_ERROR"; }
    if (fault_flags & BMS_FAULT_CONTACTOR){ return "CONTACTOR_FAULT"; }
    return "UNKNOWN_FAULT";
}

/* =========================================================================
 * HAL_GPIO_EXTI_Callback MOVIDO PARA bq796xx_bms.c (D2 CORRIGIDO)
 * g_hbms_irq é static nesse ficheiro → linkagem falhava com undefined reference.
 * Consistente com o padrão já usado para HAL_UART_RxCpltCallback.
 * ========================================================================= */

/* =========================================================================
 * SECÇÃO: BALANCEAMENTO CELULAR PASSIVO
 * =========================================================================
 * O BQ79616-Q1 integra MOSFETs de descarga individuais por célula.
 * Activar o bit correspondente em CB_CELL1_CTRL/CB_CELL9_CTRL liga a
 * resistência de balanceamento interna para essa célula, dissipando
 * excesso de carga até ao nível das células mais baixas.
 *
 * ESTRATÉGIA DE DECISÃO:
 *   Só balancear se delta_cell_mv >= CELL_BALANCE_DELTA_MV (20 mV).
 *   Balancear células com tensão > (min_cell_mv + CELL_BALANCE_HYSTERESIS_MV).
 *   Nunca balancear células abaixo de CELL_BALANCE_MIN_MV (segurança UV).
 *   Parar imediatamente em qualquer fault ou quando delta < CELL_BALANCE_STOP_MV.
 *   Parar antes de abrir o contactor (ver BMS_ContactorOpen).
 * ========================================================================= */

/**
 * @brief  Configura o balanceamento de células de um slave individual
 *
 *  Escreve a bitmask de balanceamento nos registos CB_CELL1_CTRL (células 1-8)
 *  e CB_CELL9_CTRL (células 9-15) via escrita de 2 bytes contíguos.
 *
 *  BITMASK: bit0=célula1, bit1=célula2, ..., bit14=célula15, bit15=0 (inactivo).
 *  Bit=1 → MOSFET de descarga activado; bit=0 → MOSFET desligado.
 *
 * @param  hbms         Handle do BMS
 * @param  slave_addr   Endereço DIR0 do slave
 * @param  cell_mask    Bitmask de 16 bits (bits 0-14 = células 1-15)
 * @return BMS_OK ou código de erro
 */
BMS_Status_t BMS_SetCellBalancing(BMS_Handle_t *hbms, uint8_t slave_addr,
                                   uint16_t cell_mask)
{
    uint8_t data[2U];
    /* byte[0] → CB_CELL1_CTRL: células 1-8 (bits 0-7)
     * byte[1] → CB_CELL9_CTRL: células 9-15 (bits 0-6; bit7 = célula 16 = não activa) */
    data[0] = (uint8_t)(cell_mask & 0xFFU);
    data[1] = (uint8_t)((cell_mask >> 8U) & 0x7FU);  /* Bit 7 = 0: VC16 inactivo */
    return BMS_WriteSingleDevice(hbms, slave_addr, REG_CB_CELL1_CTRL, data, 2U);
}

/**
 * @brief  Para o balanceamento em todos os slaves
 *         Chamado em: ContactorOpen, NFAULT ISR, EnterSleep, fault recovery
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro (falha parcial não bloqueia)
 */
BMS_Status_t BMS_StopAllBalancing(BMS_Handle_t *hbms)
{
    uint8_t zero[2U] = {0x00U, 0x00U};
    BMS_Status_t final_status = BMS_OK;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        BMS_Status_t r = BMS_WriteSingleDevice(hbms, hbms->slave[s].address,
                                               REG_CB_CELL1_CTRL, zero, 2U);
        if (r != BMS_OK) { final_status = r; }
        hbms->slave[s].balance_mask = 0U;
    }
    hbms->is_balancing = false;
    return final_status;
}

/**
 * @brief  Executa um ciclo de balanceamento passivo baseado nas tensões actuais
 *
 *  Algoritmo:
 *  1. Verificar pré-condições (sem fault, estado MONITORING ou BALANCING)
 *  2. Se delta < STOP_MV → parar balanceamento e retornar
 *  3. Se delta >= DELTA_MV → calcular máscara por slave:
 *     Para cada célula: if (V > min_cell + HYSTERESIS) && (V > MIN_MV) → set bit
 *  4. Aplicar máscara apenas se mudou (evitar escritas desnecessárias)
 *  5. Actualizar estado para BMS_STATE_BALANCING ou MONITORING conforme activo
 *
 *  Chamada pelo BMS_Task_100ms a cada 100 ms.
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK ou código de erro
 */
BMS_Status_t BMS_RunPassiveBalancing(BMS_Handle_t *hbms)
{
    /* Pré-condição de segurança */
    if ((hbms->state != BMS_STATE_MONITORING) &&
        (hbms->state != BMS_STATE_BALANCING))
    {
        if (hbms->is_balancing) { return BMS_StopAllBalancing(hbms); }
        return BMS_OK;
    }
    if (hbms->fault_flags != 0U)
    {
        return BMS_StopAllBalancing(hbms);
    }
    /* Não balancear se temperatura crítica */
    if (hbms->max_temp_c >= (int16_t)CELL_TEMP_WARN_C)
    {
        return BMS_StopAllBalancing(hbms);
    }

    /* Verificar se desequilíbrio justifica balanceamento */
    if (hbms->delta_cell_mv < CELL_BALANCE_STOP_MV)
    {
        if (hbms->is_balancing)
        {
            BMS_StopAllBalancing(hbms);
            hbms->state = BMS_STATE_MONITORING;
        }
        return BMS_OK;
    }
    if (hbms->delta_cell_mv < CELL_BALANCE_DELTA_MV)
    {
        return BMS_OK;   /* Abaixo do limiar de início — manter estado actual */
    }

    /* Calcular e aplicar máscaras */
    bool any_active = false;
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        uint16_t new_mask = 0U;
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t mv = hbms->slave[s].cell_voltage_mv[c];
            if ((mv > (hbms->min_cell_mv + CELL_BALANCE_HYSTERESIS_MV)) &&
                (mv > CELL_BALANCE_MIN_MV))
            {
                new_mask |= (uint16_t)(1U << c);
            }
        }
        /* Só escrever se máscara mudou (reduz tráfego UART) */
        if (new_mask != hbms->slave[s].balance_mask)
        {
            hbms->slave[s].balance_mask = new_mask;
            (void)BMS_SetCellBalancing(hbms, hbms->slave[s].address, new_mask);
        }
        if (new_mask != 0U) { any_active = true; }
    }

    hbms->is_balancing = any_active;
    if (any_active)
    {
        hbms->balance_cycle_count++;
        hbms->state = BMS_STATE_BALANCING;
    }
    else
    {
        hbms->state = BMS_STATE_MONITORING;
    }
    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO: INTERLOCK HARDWARE E COMUNICAÇÃO DE ESTADO
 * ========================================================================= */

/**
 * @brief  Actualiza os pinos de interlock hardware em tempo real
 *
 *  BMS_OK (PB13):       HIGH apenas se fault_flags==0 E nfault_pending==0
 *  PRECHARGE_OK (PB14): HIGH se tensão HV barramento >= PRECHARGE_THRESHOLD_MV
 *
 *  Estes pinos permitem à VCU (Vehicle Control Unit) tomar decisões de
 *  segurança directamente em hardware, sem depender da ligação UART.
 *  São actualizados em cada ciclo de 100 ms após todas as leituras.
 *
 *  Regulamentação Formula Student / ISO 26262:
 *  O fecho do contactor principal não deve depender exclusivamente de
 *  software — estes pinos implementam o "Hardware Interlock" exigido.
 */
void BMS_UpdateHardwareInterlocks(BMS_Handle_t *hbms)
{
    /* BUG-5 CORRIGIDO: adicionar verificação de estado operacional.
     * Anteriormente: em UNINITIALIZED ou SLEEP, fault_flags==0 e
     * nfault_pending==0 → BMS_OK ficava HIGH com ADCs inactivos.
     * Correcção: só MONITORING e BALANCING garantem aquisição activa. */
    bool bms_ok = (hbms->fault_flags == 0U) &&
                  (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) == 0U) &&
                  ((hbms->state == BMS_STATE_MONITORING) ||
                   (hbms->state == BMS_STATE_BALANCING));

    HAL_GPIO_WritePin(BMS_OK_PORT, BMS_OK_PIN,
                      bms_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* PRECHARGE_OK: tensão HV é medição física, independente do estado */
    HAL_GPIO_WritePin(PRECHARGE_OK_PORT, PRECHARGE_OK_PIN,
                      hbms->precharge_ready ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* =========================================================================
 * SECÇÃO: FILTRO DE TENSÃO (Média Móvel Exponencial — IIR)
 * =========================================================================
 * Reduz ruído ADC e elimina picos espúrios de 1 ciclo que causavam
 * transições OV/UV falsas no comparador de software.
 *
 * Fórmula: filtered = (raw + (ALPHA-1) × filtered) / ALPHA
 * Com ALPHA=4: 25% peso para leitura nova, 75% história.
 *
 * Na primeira leitura (filter_primed=false), o filtro é semeado com
 * os valores raw directos — sem isso, o transitório de arranque
 * produziria valores falsos de ~0 mV durante os primeiros ALPHA ciclos.
 *
 * Após o filtro, as tensões em cell_voltage_mv são SUBSTITUÍDAS pelos
 * valores filtrados. As funções a jusante (CheckProtections, Balancing)
 * operam sobre dados já filtrados sem alteração. */

void BMS_ApplyVoltageFilter(BMS_Handle_t *hbms)
{
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t raw = hbms->slave[s].cell_voltage_mv[c];

            if (!hbms->filter_primed)
            {
                /* Seed: primeira leitura → filtro inicializado com raw */
                hbms->filtered_mv[s][c] = raw;
            }
            else
            {
                /* IIR: new = (raw + (ALPHA-1) × old) / ALPHA */
                uint32_t acc = (uint32_t)raw +
                               (uint32_t)(BMS_VOLTAGE_FILTER_ALPHA - 1U) *
                               (uint32_t)hbms->filtered_mv[s][c];
                hbms->filtered_mv[s][c] =
                    (uint16_t)(acc / (uint32_t)BMS_VOLTAGE_FILTER_ALPHA);
            }
            /* Substituir raw pelo valor filtrado */
            hbms->slave[s].cell_voltage_mv[c] = hbms->filtered_mv[s][c];
        }
    }
    hbms->filter_primed = true;
}

/* BMS_ReadCellVoltagesRedundant REMOVIDA (MISRA C:2012 Rule 2.1 — código morto).
 * Auditoria v3.1: duas leituras consecutivas do mesmo registo ADC em modo
 * contínuo extraem a mesma conversão latched. O CRC-16/MODBUS já valida
 * integridade UART. Redundância ASIL-D requer o ADC de diagnóstico secundário
 * do BQ79616 (canal independente), não releitura do mesmo registo. */

/* =========================================================================
 * SECÇÃO: ESTIMAÇÃO DE ESTADO DE CARGA (SoC — OCV Lookup)
 * =========================================================================
 * Tabela OCV (Open Circuit Voltage) para LiFePO4:
 *   - Plataforma nominal ~3.2-3.3V (SoC 20-80%)
 *   - Descida rápida abaixo de 3.0V e acima de 3.5V
 *
 * LIMITAÇÃO: OCV é válida apenas em repouso (sem corrente).
 * Para operação dinâmica, implementar Coulomb Counting ou Extended Kalman.
 *
 * NOTA: Substituir pela tabela fornecida pelo fabricante das células.
 * A tabela abaixo é genérica para LiFePO4 e serve como ponto de partida. */

typedef struct { uint16_t mv; uint8_t soc; } SoC_Point_t;

static const SoC_Point_t g_soc_table[BMS_SOC_TABLE_SIZE] =
{
    { 3000U,   0U },   /* UV cutoff */
    { 3100U,   5U },
    { 3200U,  10U },
    { 3250U,  20U },
    { 3270U,  30U },
    { 3280U,  40U },
    { 3300U,  50U },   /* Plataforma nominal LFP */
    { 3310U,  60U },
    { 3330U,  70U },
    { 3350U,  80U },
    { 3400U,  90U },
    { 3600U, 100U },   /* OV cutoff */
};

/**
 * @brief  Estima SoC a partir da tensão média celular (OCV lookup + interpolação)
 *
 * @param  avg_cell_mv  Tensão média das células em mV
 * @return SoC em percentagem (0-100)
 */
uint8_t BMS_EstimateSoC(uint16_t avg_cell_mv)
{
    /* Clamp aos extremos da tabela */
    if (avg_cell_mv <= g_soc_table[0].mv)
    {
        return g_soc_table[0].soc;
    }
    if (avg_cell_mv >= g_soc_table[BMS_SOC_TABLE_SIZE - 1U].mv)
    {
        return g_soc_table[BMS_SOC_TABLE_SIZE - 1U].soc;
    }

    /* Pesquisa linear + interpolação entre dois pontos adjacentes */
    for (uint8_t i = 1U; i < BMS_SOC_TABLE_SIZE; i++)
    {
        if (avg_cell_mv <= g_soc_table[i].mv)
        {
            uint16_t mv_lo  = g_soc_table[i - 1U].mv;
            uint16_t mv_hi  = g_soc_table[i].mv;
            uint8_t  soc_lo = g_soc_table[i - 1U].soc;
            uint8_t  soc_hi = g_soc_table[i].soc;

            /* Interpolação linear: soc = soc_lo + (soc_hi-soc_lo) × (mv-mv_lo) / (mv_hi-mv_lo) */
            uint32_t delta_mv  = (uint32_t)(avg_cell_mv - mv_lo);
            uint32_t range_mv  = (uint32_t)(mv_hi - mv_lo);
            uint32_t delta_soc = (uint32_t)(soc_hi - soc_lo);

            if (range_mv == 0U) { return soc_lo; }
            return (uint8_t)(soc_lo + (uint8_t)((delta_mv * delta_soc) / range_mv));
        }
    }
    return 100U;  /* Shouldn't reach here */
}
