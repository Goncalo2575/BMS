/**
 * @file    bq796xx_bms_monitor.c
 * @brief   BMS - Módulo de Monitorização, Protecções e Tratamento de Falhas
 *          Leitura de tensões, temperaturas, gestão de faults e recuperação de anel
 *
 *  ARQUITECTURA (v3.3 — ACTUALIZADA):
 *  Este STM32F446RET6 é o único microcontrolador que governa o pack. Centraliza
 *  a máquina de estados do BMS, a interface com o IMD (Insulation Monitoring
 *  Device) e a telemetria (USART2). Esta camada DECIDE (abrir/fechar contactor,
 *  BMS_OK, PRECHARGE_OK) — decisões LÓGICAS guardadas no handle. A ACTUAÇÃO
 *  FÍSICA dos relés/contactores e do LED cluster é feita pelo módulo
 *  bms_relays.c, que corre NESTE MESMO MCU (ver Secção 9 e bms_relays.h).
 *
 * @version 3.3.0
 */

#include "bq796xx_bms.h"
#include "bms_relays.h"   /* BMS_Relays_GetState(): bloquear contactor/SoC em CHARGING */

/* =========================================================================
 * SECÇÃO 4: LEITURA DE TENSÕES DAS CÉLULAS
 * ========================================================================= */

/**
 * @brief  Converte os dois bytes raw do ADC para tensão em mV
 *
 * Para que serve: tradutor base de toda a leitura de tensão. Os BQ79616
 * devolvem 16 bits por célula; este helper aplica a resolução do ADC.
 *         Resolução BQ79616: LSB = 190.73 µV → V_mV = (raw * 1907) / 10000
 */
static inline uint16_t BMS_RawToMillivolts(uint8_t hi_byte, uint8_t lo_byte)
{
    uint16_t raw = ((uint16_t)hi_byte << 8U) | (uint16_t)lo_byte;
    return (uint16_t)(((uint32_t)raw * 1907UL) / 10000UL);
}

/**
 * @brief  Lê as tensões de todas as células de um slave via Single Device Read
 *
 * Para que serve: obtém as 15 tensões celulares de UM slave e mapeia-as para o
 * array lógico (Cell 1..15). Os registos VCELL vêm em ordem descendente, daí o
 * mapeamento invertido.
 *         Endereço VCELL15_HI=0x056A; registos em ordem DESCENDENTE → mapeamento invertido.
 */
static BMS_Status_t BMS_ReadSlaveVoltages(BMS_Handle_t *hbms, uint8_t slave_idx)
{
    uint8_t      rx_buf[BMS_VCELL_READ_BYTES];  /* 30 bytes */
    BMS_Status_t status;
    uint8_t      slave_addr = hbms->slave[slave_idx].address;

    status = BMS_ReadSingleDevice(hbms, slave_addr,
                                   REG_VCELL15_HI, rx_buf,
                                   (uint8_t)BMS_VCELL_READ_BYTES);
    if (status != BMS_OK)
    {
        hbms->slave[slave_idx].comm_ok = false;
        return status;
    }

    /* Mapeamento invertido: registos descendentes (VCELL15 primeiro),
     * array ascendente (índice 0 = Cell 1). c=0→Cell1→rx_buf[28..29]. */
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
 * Para que serve: ponto único de leitura de tensões do pack. Percorre os 2
 * slaves, soma a tensão total (pack_voltage_mv), e calcula min/max/delta para
 * protecções, balanceamento e telemetria. Em falha de comunicação de um slave,
 * marca BMS_FAULT_COMM, zera as células desse slave e continua (não aborta).
 */
BMS_Status_t BMS_ReadAllCellVoltages(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint32_t pack_sum  = 0UL;  /* uint32 obrigatório: 30S × 4250 mV > UINT16_MAX */
    uint16_t min_mv    = 0xFFFFU;
    uint16_t max_mv    = 0U;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        status = BMS_ReadSlaveVoltages(hbms, s);
        if (status != BMS_OK)
        {
            /* Não tentar address_rev sem comutar DIR_SEL — deixar o
             * BMS_ProcessFaults → BMS_RingRecovery tratar a comutação. */
            hbms->fault_flags |= BMS_FAULT_COMM;
            memset(hbms->slave[s].cell_voltage_mv, 0,
                   sizeof(hbms->slave[s].cell_voltage_mv));
            continue;
        }

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
                             (uint16_t)(max_mv - hbms->min_cell_mv) : 0U;

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 5: LEITURA DE TEMPERATURAS
 * ========================================================================= */

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
 * Para que serve: traduz a leitura do NTC (ratiométrica face ao TSREF) para °C
 * por interpolação na tabela. Inclui uma barreira de segurança: se o rácio for
 * ≈1.0 (NTC desconectado/cabo partido) força OT (127°C) em vez de reportar frio
 * falso, garantindo que um sensor partido leva à abertura do contactor.
 */
static int16_t BMS_RawToTemperature_Ratiometric(uint16_t gpio_raw,
                                                  uint16_t tsref_raw)
{
    if (tsref_raw == 0U)
    {
        return INT8_MIN;   /* TSREF inválido - sensor desligado */
    }

    uint32_t ratio_x10000 = ((uint32_t)gpio_raw * 10000UL) / (uint32_t)tsref_raw;

    /* Detecção de NTC desconectado / cabo partido: ratio≈1.0 → força OT.
     * Sem esta barreira, NTC aberto reportaria -20°C e mascararia OT real. */
    if (ratio_x10000 > 9500U)
    {
        return (int16_t)127;   /* Força OT → decisão de abrir contactor */
    }

    if (ratio_x10000 <= (uint32_t)g_ntc_table[0].ratio_x10000)
    {
        return (int16_t)g_ntc_table[0].temp_c;
    }
    if (ratio_x10000 >= (uint32_t)g_ntc_table[BMS_NTC_TABLE_SIZE - 1U].ratio_x10000)
    {
        return (int16_t)g_ntc_table[BMS_NTC_TABLE_SIZE - 1U].temp_c;
    }

    uint8_t lo = 0U;
    uint8_t hi = BMS_NTC_TABLE_SIZE - 1U;

    while (((uint8_t)(hi - lo)) > 1U)
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

    uint32_t r_lo = (uint32_t)g_ntc_table[lo].ratio_x10000;
    uint32_t r_hi = (uint32_t)g_ntc_table[hi].ratio_x10000;
    int32_t  t_lo = (int32_t)g_ntc_table[lo].temp_c;
    int32_t  t_hi = (int32_t)g_ntc_table[hi].temp_c;

    int32_t temp_c = t_lo + ((t_hi - t_lo) * (int32_t)(ratio_x10000 - r_lo)) /
                              (int32_t)(r_hi - r_lo);

    return (int16_t)temp_c;
}

/**
 * @brief  Lê temperaturas de 3 NTCs por slave + tensão HV do barramento
 *
 * Para que serve: num único Single Read por slave (bloco GPIO1-4 + TSREF), obtém
 * as 3 temperaturas NTC e regista o máximo global (max_temp_c) para protecção
 * térmica. O GPIO4 (sensor HV) é lido à parte por BMS_ReadInverterVoltage.
 */
BMS_Status_t BMS_ReadAllTemperatures(BMS_Handle_t *hbms)
{
    uint8_t  aux_buf[BMS_AUX_READ_BYTES];  /* 10 bytes: GPIO1-4 + TSREF */
    int16_t  max_temp = -100;
    BMS_Status_t status;

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        status = BMS_ReadSingleDevice(hbms, hbms->slave[s].address,
                                       REG_GPIO1_HI, aux_buf,
                                       BMS_AUX_READ_BYTES);
        if (status != BMS_OK) { continue; }

        uint16_t tsref_raw = ((uint16_t)aux_buf[8] << 8U) | (uint16_t)aux_buf[9];

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
 * Para que serve: mede a tensão do barramento de alta tensão (após o divisor
 * resistivo externo) para a lógica de pré-carga. O resultado (inverter_voltage_mv)
 * é consumido por bms_relays.c (limiar dinâmico de 90% do pack) e gera o interlock
 * lógico precharge_ready.
 *
 * BUG-FIX (v3.3): atenuação aplicada em ponto-fixo (NUM/DEN = 2711/100 = 27.11).
 * O inteiro anterior (27) introduzia ~0.4% de erro por defeito na tensão do bus.
 */
BMS_Status_t BMS_ReadInverterVoltage(BMS_Handle_t *hbms)
{
    uint8_t buf[2U];
    BMS_Status_t status;

    status = BMS_ReadSingleDevice(hbms, hbms->slave[0].address,
                                   REG_GPIO4_HI, buf, 2U);
    if (status != BMS_OK) { return status; }

    uint16_t raw = ((uint16_t)buf[0] << 8U) | (uint16_t)buf[1];
    uint32_t vadc_mv = (uint32_t)(((uint32_t)raw * 1907UL) / 10000UL);

    /* Ponto-fixo 27.11: V_bus = V_adc × 2711 / 100 (sem overflow em uint32). */
    hbms->inverter_voltage_mv = (vadc_mv * (uint32_t)HV_BUS_ATTENUATION_NUM)
                              / (uint32_t)HV_BUS_ATTENUATION_DEN;

    hbms->precharge_ready =
        (hbms->inverter_voltage_mv >= PRECHARGE_THRESHOLD_MV);

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 6: VERIFICAÇÃO DE PROTECÇÕES SOFTWARE
 * ========================================================================= */

/**
 * @brief  Verifica limites de tensão e temperatura por software
 *
 * Para que serve: segunda barreira de protecção (a primeira são os comparadores
 * de hardware OV/UV dos BQ79616 via NFAULT). Percorre todas as células e NTCs;
 * se alguma viola OV/UV/OT, marca as flags (latched-by-design ASIL-D) e dispara
 * BMS_EmergencyShutdown (decisão de abrir + bms_ok=false), passando a FAULT.
 */
BMS_Status_t BMS_CheckProtections(BMS_Handle_t *hbms)
{
    bool fault_detected = false;

    /* fault_flags acumulam via OR sem reset cíclico (latch-by-design ASIL-D). */

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t mv = hbms->slave[s].cell_voltage_mv[c];

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

            /* UV sem guard (mv>0): 0 mV = célula morta/cabo cortado → fault. */
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

    if (hbms->delta_cell_mv > CELL_IMBALANCE_MV)
    {
        /* Desequilíbrio grave - regista mas não actua imediatamente */
    }

    if (fault_detected)
    {
        /* Faults de software (OT, UV por ADC) não disparam NFAULT.
         * BMS_EmergencyShutdown regista a DECISÃO de abrir + bms_ok=false. */
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
 * @brief  Callback de interrupção NFAULT (EXTI8/PA8, flanco descendente)
 *         Chamada por HAL_GPIO_EXTI_Callback (em bq796xx_bms.c).
 *
 * Para que serve: regista, com latência mínima, a DECISÃO de abrir o contactor
 * quando o hardware da bateria sinaliza falha crítica (OV/OT/...) pelo NFAULT.
 *
 *  ACTUAÇÃO (v3.3): esta ISR não acciona o pino do contactor; marca
 *  contactor_closed=false e bms_ok=false e levanta nfault_pending. A abertura
 *  FÍSICA do BMS_relay é feita logo a seguir no super-loop: após
 *  BMS_ProcessFaults corre BMS_Relays_Task, que abre o relé (latch). Ver a NOTA
 *  DE SEGURANÇA na Secção 9.
 */
void BMS_NFAULT_IRQHandler(BMS_Handle_t *hbms)
{
    if (hbms != NULL)
    {
        hbms->bms_ok           = false;  /* interlock lógico: NOK imediato */
        hbms->contactor_closed = false;  /* DECISÃO: contactor deve ABRIR */

        __atomic_store_n(&hbms->nfault_pending, 1U, __ATOMIC_SEQ_CST);
        hbms->fault_count++;
    }
}

/**
 * @brief  Lê e interpreta os registos FAULT_SUMMARY de todos os dispositivos
 *
 * Para que serve: chamada quando há NFAULT pendente, faz o diagnóstico completo:
 * lê o FAULT_SUMMARY da bridge (ring break/heartbeat) e dos slaves (OV/UV/COMM),
 * deteta open-wire (VCOW) e decide a acção — shutdown de emergência em falhas
 * críticas, ou tentativa de recuperação de anel em ring break.
 */
BMS_Status_t BMS_ProcessFaults(BMS_Handle_t *hbms)
{
    uint32_t was_pending = __atomic_exchange_n(&hbms->nfault_pending, 0U,
                                                __ATOMIC_SEQ_CST);
    if (was_pending == 0U)
    {
        return BMS_OK;
    }

    uint8_t      fault_buf[4U];
    BMS_Status_t status;
    bool         critical_fault = false;

    /* --- Leitura FAULT_SUMMARY da Bridge --- */
    status = BMS_ReadSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                   REG_BRIDGE_FAULT_SUMMARY, fault_buf, 2U);
    if (status == BMS_OK)
    {
        uint8_t bridge_fault_comm2 = fault_buf[1];

        if (bridge_fault_comm2 & 0x01U)
        {
            hbms->fault_flags |= BMS_FAULT_RING_BREAK;
            hbms->ring_intact   = false;
        }
        if (bridge_fault_comm2 & 0x40U)
        {
            hbms->fault_flags |= BMS_FAULT_HB_FAIL;
        }
    }

    /* --- Broadcast Read FAULT_SUMMARY dos Slaves (1 byte, bits correctos) --- */
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
        hbms->fault_flags |= BMS_FAULT_RING_BREAK;
        hbms->ring_intact   = false;
    }

    /* --- Verificação de Open Wire (FAULT_VCOW, 1 byte) --- */
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
 * @brief  Recupera a comunicação após rotura do anel, comutando para o caminho reverso (DIR1)
 *
 * Para que serve: tolerância a falha do cabo de comunicação (topologia em anel).
 * Se o caminho principal (DIR0) partir, comuta a bridge para DIR1 e testa se os
 * slaves respondem pelos endereços reversos. Se sim, troca os endereços lógicos
 * (passa a operar em modo degradado mas funcional) e volta a MONITORING. Se nem
 * o caminho reverso responder, marca COMM e fica em FAULT.
 */
BMS_Status_t BMS_RingRecovery(BMS_Handle_t *hbms)
{
    BMS_Status_t status;
    uint8_t data;

    hbms->state = BMS_STATE_RING_RECOVERY;
    hbms->ring_recovery_count++;
    hbms->ring_intact = false;

    data = CTRL1_DIR_SEL;  /* 0x02 */
    status = BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                    REG_BRIDGE_CONTROL1, &data, 1U);
    if (status != BMS_OK)
    {
        return BMS_ERR_COMM;
    }
    BMS_DelayUs(hbms, DELAY_DIR_SEL_SWITCH_US);  /* 100 µs */

    uint8_t test_buf[1U];
    bool slave0_rev_ok = (BMS_ReadSingleDevice(hbms, hbms->slave[0].address_rev,
                                                REG_ACTIVE_CELL, test_buf, 1U) == BMS_OK);
    bool slave1_rev_ok = (BMS_ReadSingleDevice(hbms, hbms->slave[1].address_rev,
                                                REG_ACTIVE_CELL, test_buf, 1U) == BMS_OK);

    if (slave0_rev_ok || slave1_rev_ok)
    {
        hbms->ring_using_reverse = true;
        hbms->fault_flags &= ~BMS_FAULT_RING_BREAK;
        /* Modo degradado (anel físico partido) comunicado ao registador por
         * ring_intact=false na telemetria — NÃO por fault_flag. */

        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            uint8_t tmp         = hbms->slave[s].address;
            hbms->slave[s].address     = hbms->slave[s].address_rev;
            hbms->slave[s].address_rev = tmp;
        }

        hbms->state = BMS_STATE_MONITORING;
        return BMS_OK;
    }

    hbms->ring_using_reverse = false;
    hbms->fault_flags |= BMS_FAULT_COMM;
    hbms->state = BMS_STATE_FAULT;
    return BMS_ERR_COMM;
}

/* =========================================================================
 * SECÇÃO 9: DECISÃO DE CONTACTOR E INTERLOCKS (LÓGICA — ACTUAÇÃO EM bms_relays)
 * =========================================================================
 * ⚠⚠⚠  NOTA DE SEGURANÇA — ABERTURA DO CONTACTOR (ISO 26262 / ASIL-D)  ⚠⚠⚠
 * -------------------------------------------------------------------------
 * ARQUITECTURA (v3.3): este STM32F446RET6 é a ÚNICA autoridade de decisão de
 * segurança do pack E TAMBÉM O ACTUADOR. Concentra a máquina de estados do BMS,
 * a interface com o IMD e a telemetria. Não existe um segundo MCU de supervisão.
 *
 * SEPARAÇÃO DE RESPONSABILIDADES (a constar do FMEA/FTA do projecto):
 *   • Esta camada (bq796xx_bms_monitor.c) CALCULA as decisões LÓGICAS de
 *     segurança e guarda-as no handle: contactor_closed, bms_ok, precharge_ready.
 *   • O módulo bms_relays.c (mesmo MCU) CONSOME essas decisões + a tensão do
 *     bus/pack e ACCIONA fisicamente os RELÉS auxiliares/de segurança
 *     (PC0 pré-carga, PC1 charge, PC2 descarga/bleed, PC4 BMS_relay, PA6
 *     BMS_charge) e o LED cluster, com a máquina SAFE/ENGAGED/CHARGING/NOT_SAFE.
 *   • Os 2 CONTACTORES NÃO são actuados por este MCU:
 *       - charge_contactor : segue o sinal combinado charge_relay + BMS_charge_relay;
 *       - Line_contactor   : fechado AUTONOMAMENTE pelo INVERSOR (Sevcon Gen4
 *         Size 6) quando B+ atinge a tensão programada no inversor. O MCU não o
 *         comanda; só fornece pré-carga/bleed. contactor_closed é apenas a
 *         expectativa interna de "tração activa" (gating de SoC/telemetria).
 *
 * RISCOS RESIDUAIS QUE TÊM DE SER TRATADOS NA ANÁLISE DE SEGURANÇA:
 *
 *   1) LATÊNCIA vs FTTI
 *      A decisão "ABRIR" tem de ser actuada dentro do Fault Tolerant Time
 *      Interval. MITIGAÇÃO: o NFAULT é processado FORA da cadência de 100 ms
 *      (imediato no super-loop) e seguido logo de BMS_Relays_Task, que abre o
 *      BMS_relay no mesmo ciclo. A pré-carga longa (750 ms) é não-bloqueante
 *      para não comprometer o refresh do IWDG.
 *
 *   2) PONTO ÚNICO DE FALHA (SPOF) — MCU único
 *      Sendo o mesmo MCU a decidir e a actuar, um hang do CPU deixaria os relés
 *      no último estado. MITIGAÇÕES: (a) IWDG ~500 ms reseta o MCU; o
 *      BMS_Relays_Init repõe o estado seguro de arranque (BMS_relay/BMS_charge
 *      fechados, contactores abertos, LEDs apagados) — confirmar que este
 *      estado de arranque é o adequado ao veículo; (b) recomenda-se manter o
 *      NFAULT da bridge ramificado por HARDWARE para um corte independente do
 *      software, como defense-in-depth.
 *
 *   3) FAIL-SAFE DOS MONITORES
 *      As entradas de monitorização (IMD/TSMS/ESDB/...) usam pull-down: a
 *      ausência de sinal lê-se como "não activo" → tende a NOT_SAFE. Confirmar
 *      a polaridade real do hardware contra esta convenção.
 *
 * Esta arquitectura (MCU único decisor+actuador) É uma decisão de projecto e
 * DEVE ser registada e justificada na análise de segurança (FMEA/FTA).
 * ========================================================================= */

/**
 * @brief  Limpa a expectativa interna de "tração activa" (contactor_closed=false)
 *
 *  Esta camada não comanda contactores nem pinos. Aqui apenas:
 *   - pára o balanceamento (evita dissipar com o pack a ser isolado);
 *   - regista contactor_closed=false (flag interna p/ gating de SoC/telemetria).
 *  A abertura física dá-se nos RELÉS de segurança (BMS_relay, por bms_relays.c)
 *  e o Line_contactor abre quando o inversor deixa de o manter fechado (B+ cai).
 */
void BMS_ContactorOpen(BMS_Handle_t *hbms)
{
    if (hbms->is_balancing)
    {
        (void)BMS_StopAllBalancing(hbms);
    }
    hbms->contactor_closed = false;   /* expectativa interna: tração inactiva */
}

/**
 * @brief  Marca a expectativa interna de "tração activa" (contactor_closed=true)
 *
 *  NÃO comanda o Line_contactor — esse é fechado pelo INVERSOR (Sevcon Gen4)
 *  quando B+ atinge o limiar programado. Esta flag é só a expectativa interna
 *  do BMS (gating de SoC/telemetria) e só é marcada se TODAS as barreiras
 *  abaixo passarem, garantindo que nunca indicamos "tração activa" numa
 *  condição insegura:
 *   [1] nfault_pending == 0   — evento de hardware NÃO processado (BARREIRA CRÍTICA)
 *   [2] fault_flags == 0      — sem falhas mapeadas pelo software
 *   [3] state == MONITORING ou BALANCING — operação normal (HV ligada)
 *   [4] min_cell_mv >= CELL_UV_MV && max_cell_mv <= CELL_OV_MV — tensões OK
 *   [5] NÃO está em CHARGING   — separação total tração/carga (D.5.3.7)
 */
void BMS_ContactorClose(BMS_Handle_t *hbms)
{
    /* BARREIRA 1 (CRÍTICA): evento NFAULT de hardware ainda por processar */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return;
    }
    /* BARREIRA 2: sem falhas mapeadas pelo software */
    if (hbms->fault_flags != 0U)
    {
        return;
    }
    /* BARREIRA 3: máquina de estados em operação normal (MONITORING ou
     * BALANCING — o balanceamento é operação normal com HV ligada; tratá-lo
     * como "abrir" fazia a decisão lógica divergir do estado real dos relés). */
    if ((hbms->state != BMS_STATE_MONITORING) &&
        (hbms->state != BMS_STATE_BALANCING))
    {
        return;
    }
    /* BARREIRA 4: tensões celulares dentro dos limites operacionais */
    if ((hbms->min_cell_mv < CELL_UV_MV) ||
        (hbms->max_cell_mv > CELL_OV_MV))
    {
        return;
    }
    /* BARREIRA 5: carregador ligado → separação total tração/carga.
     * O circuito de tração tem de estar isolado e a 0 V durante TODO o
     * carregamento; não indicar "tração activa" nesta condição. */
    if (BMS_Relays_GetState() == BMS_RLY_CHARGING)
    {
        return;
    }

    /* Todas as barreiras ultrapassadas — DECISÃO: fechar */
    hbms->contactor_closed = true;
}

/**
 * @brief  Shutdown de emergência - decisão de abrir + interlock NOK
 *         Chamado em condições críticas (OV, UV, OT, Ring fail total)
 *
 * Para que serve: ponto único de "desligar em segurança" por software. Espelha
 * o comportamento da ISR NFAULT para faults que não têm comparador de hardware
 * (OT, UV por ADC): regista a decisão de abrir, força bms_ok=false e passa a FAULT.
 * A abertura física do BMS_relay segue em bms_relays.c.
 */
void BMS_EmergencyShutdown(BMS_Handle_t *hbms)
{
    /* Decisão de abrir (pára balanceamento + contactor_closed=false) */
    BMS_ContactorOpen(hbms);

    /* Interlock lógico BMS_OK → NOK. */
    hbms->bms_ok = false;

    hbms->state = BMS_STATE_FAULT;
}

/* =========================================================================
 * SECÇÃO 10: TAREFA CÍCLICA PRINCIPAL (100 ms)
 * ========================================================================= */

/**
 * @brief  Tarefa periódica de monitorização do BMS (cadência de 100 ms)
 *
 * Para que serve: é o "batimento cardíaco" da monitorização. Por ordem:
 * 1. Processa NFAULT pendente (prioridade máxima); se ficar em FAULT, sai.
 * 2. Salta a aquisição em estados sem ADC (FAULT/SHUTDOWN/SLEEP/UNINIT).
 * 3. Lê tensões, temperaturas e tensão HV do barramento.
 * 4. Corre as protecções sobre dados RAW (FTTI) — antes do filtro.
 * 5. Aplica o filtro IIR (só telemetria/balanceamento), estima SoC (em
 *    relaxação), corre o balanceamento passivo e actualiza os interlocks lógicos.
 * Nota: a actuação dos relés/LED NÃO é feita aqui — é BMS_Relays_Task, chamada
 * em todas as iterações do super-loop (ver main_bms_app.c).
 */
BMS_Status_t BMS_Task_100ms(BMS_Handle_t *hbms)
{
    BMS_Status_t status = BMS_OK;

    if (hbms == NULL) { return BMS_ERR_INVALID_PARAM; }

    /* --- Processar NFAULT pendente (prioridade máxima) --- */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        BMS_ProcessFaults(hbms);
        if (hbms->state == BMS_STATE_FAULT)
        {
            return BMS_ERR_FAULT_ACTIVE;
        }
    }

    /* --- Estados sem polling de ADC --- */
    if ((hbms->state == BMS_STATE_FAULT)        ||
        (hbms->state == BMS_STATE_SHUTDOWN)     ||
        (hbms->state == BMS_STATE_SLEEP)        ||
        (hbms->state == BMS_STATE_UNINITIALIZED))
    {
        return BMS_ERR_FAULT_ACTIVE;
    }

    /* --- Leitura de tensões (dados RAW para protecções) --- */
    status = BMS_ReadAllCellVoltages(hbms);
    if (status != BMS_OK)
    {
        hbms->comm_error_count++;
    }

    /* --- Leitura de temperaturas --- */
    status = BMS_ReadAllTemperatures(hbms);

    /* --- Tensão HV do barramento (pré-carga) --- */
    (void)BMS_ReadInverterVoltage(hbms);

    /* --- Verificação de protecções sobre dados RAW (FTTI) --- */
    status = BMS_CheckProtections(hbms);
    if (status == BMS_ERR_FAULT_ACTIVE)
    {
        return BMS_ERR_FAULT_ACTIVE;
    }

    /* --- Filtro IIR (APÓS protecções — só telemetria/display/balanceamento) --- */
    BMS_ApplyVoltageFilter(hbms);

    /* --- SoC (apenas em relaxação verdadeira: contactor aberto, sem bleed de
     *     balanceamento E sem carregamento — qualquer corrente falseia a OCV) --- */
    if (!hbms->contactor_closed && !hbms->is_balancing &&
        (BMS_Relays_GetState() != BMS_RLY_CHARGING))
    {
        if ((hbms->min_cell_mv > 0U) && (hbms->max_cell_mv > 0U))
        {
            uint16_t avg_mv = (uint16_t)(hbms->pack_voltage_mv /
                                          (uint32_t)BMS_TOTAL_CELLS);
            hbms->soc_percent = BMS_EstimateSoC(avg_mv);
        }
    }

    /* --- Balanceamento celular passivo --- */
    (void)BMS_RunPassiveBalancing(hbms);

    /* --- Actualizar interlocks lógicos (BMS_OK, PRECHARGE_OK) --- */
    BMS_UpdateHardwareInterlocks(hbms);

    return BMS_OK;
}

/* =========================================================================
 * SECÇÃO 11: UTILITÁRIOS DE DIAGNÓSTICO
 * ========================================================================= */

/**
 * @brief  Devolve o nome legível do estado do BMS (para telemetria/debug)
 *
 * Para que serve: traduz o enum BMS_State_t para texto, usado na linha de
 * telemetria USART2 e nos prints de arranque.
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
 * @brief  Devolve o nome legível da falha de maior prioridade (para telemetria/debug)
 *
 * Para que serve: traduz o bitfield fault_flags para texto. Como podem coexistir
 * várias falhas, devolve a de maior prioridade pela ordem de teste (OV→UV→OT→...).
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
    return "UNKNOWN_FAULT";
}

/* =========================================================================
 * SECÇÃO: BALANCEAMENTO CELULAR PASSIVO
 * ========================================================================= */

/**
 * @brief  Programa a máscara de balanceamento de um slave (que células drenar)
 *
 * Para que serve: escreve o bitmask das células a balancear num slave. Cada bit
 * a 1 liga a resistência de descarga (bleed) dessa célula. O bit 7 do byte alto
 * fica a 0 (VC16 inactivo na config 15S).
 */
BMS_Status_t BMS_SetCellBalancing(BMS_Handle_t *hbms, uint8_t slave_addr,
                                   uint16_t cell_mask)
{
    uint8_t data[2U];
    data[0] = (uint8_t)(cell_mask & 0xFFU);
    data[1] = (uint8_t)((cell_mask >> 8U) & 0x7FU);  /* Bit 7 = 0: VC16 inactivo */
    return BMS_WriteSingleDevice(hbms, slave_addr, REG_CB_CELL1_CTRL, data, 2U);
}

/**
 * @brief  Pára o balanceamento em todos os slaves
 *
 * Para que serve: desliga todas as resistências de bleed (escreve 0) em todos os
 * slaves e limpa o estado is_balancing. Usado antes de dormir, em falha, ou
 * quando o desequilíbrio já está dentro do limite.
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
 * @brief  Executa um ciclo de balanceamento passivo (top-balancing)
 *
 * Para que serve: equaliza as células drenando as mais altas. Só corre em
 * MONITORING/BALANCING, sem falhas e abaixo do aviso térmico. Selecciona as
 * células acima de (min + histerese) e acima da tensão mínima de balanceamento,
 * programa as máscaras e gere a transição de estado MONITORING↔BALANCING. Pára
 * quando o delta desce abaixo do limiar.
 */
BMS_Status_t BMS_RunPassiveBalancing(BMS_Handle_t *hbms)
{
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
    if (hbms->max_temp_c >= (int16_t)CELL_TEMP_WARN_C)
    {
        return BMS_StopAllBalancing(hbms);
    }

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
        return BMS_OK;
    }

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
 * SECÇÃO: INTERLOCKS LÓGICOS E COMUNICAÇÃO DE ESTADO
 * ========================================================================= */

/**
 * @brief  Calcula os interlocks lógicos BMS_OK e PRECHARGE_OK
 *
 *  BMS_OK:       true só se fault_flags==0 E nfault_pending==0 E estado activo
 *  PRECHARGE_OK: calculado em BMS_ReadInverterVoltage (tensão HV ≥ limiar)
 *
 *  Estes valores são consumidos pela telemetria e pelo módulo bms_relays
 *  (decide a actuação física). Ver NOTA DE SEGURANÇA (Secção 9).
 */
void BMS_UpdateHardwareInterlocks(BMS_Handle_t *hbms)
{
    /* Só MONITORING e BALANCING garantem aquisição activa (ADCs a correr). */
    hbms->bms_ok = (hbms->fault_flags == 0U) &&
                   (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) == 0U) &&
                   ((hbms->state == BMS_STATE_MONITORING) ||
                    (hbms->state == BMS_STATE_BALANCING));

    /* precharge_ready já é calculado em BMS_ReadInverterVoltage (medição física). */
}

/* =========================================================================
 * SECÇÃO: FILTRO DE TENSÃO (Média Móvel Exponencial — IIR)
 * ========================================================================= */

/**
 * @brief  Aplica o filtro IIR exponencial às tensões celulares
 *
 * Para que serve: suaviza o ruído das leituras para telemetria/display e para o
 * balanceamento (NÃO para as protecções — essas correm sobre dados RAW, antes
 * deste filtro). Na primeira passagem faz "seed" (filtrado = raw) para evitar
 * arranque enviesado.
 */
void BMS_ApplyVoltageFilter(BMS_Handle_t *hbms)
{
    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            uint16_t raw = hbms->slave[s].cell_voltage_mv[c];

            if (!hbms->filter_primed)
            {
                hbms->filtered_mv[s][c] = raw;
            }
            else
            {
                uint32_t acc = (uint32_t)raw +
                               (uint32_t)(BMS_VOLTAGE_FILTER_ALPHA - 1U) *
                               (uint32_t)hbms->filtered_mv[s][c];
                hbms->filtered_mv[s][c] =
                    (uint16_t)(acc / (uint32_t)BMS_VOLTAGE_FILTER_ALPHA);
            }
            hbms->slave[s].cell_voltage_mv[c] = hbms->filtered_mv[s][c];
        }
    }
    hbms->filter_primed = true;
}

/* =========================================================================
 * SECÇÃO: ESTIMAÇÃO DE ESTADO DE CARGA (SoC — OCV Lookup)
 * ========================================================================= */

typedef struct { uint16_t mv; uint8_t soc; } SoC_Point_t;

static const SoC_Point_t g_soc_table[BMS_SOC_TABLE_SIZE] =
{
    { 3000U,   0U },   /* 3.00 V ->   0% (Subtensão Absoluta) */
    { 3150U,   5U },   /* 3.15 V ->   5% */
    { 3300U,  10U },   /* 3.30 V ->  10% (Fim do "Joelho" de descarga) */
    { 3350U,  15U },   /* 3.35 V ->  15% */
    { 3400U,  20U },   /* 3.40 V ->  20% */
    { 3450U,  25U },   /* 3.45 V ->  25% */
    { 3500U,  30U },   /* 3.50 V ->  30% */
    { 3550U,  35U },   /* 3.55 V ->  35% */
    { 3600U,  40U },   /* 3.60 V ->  40% */
    { 3650U,  45U },   /* 3.65 V ->  45% */
    { 3700U,  50U },   /* 3.70 V ->  50% (Tensão Nominal Termodinâmica) */
    { 3750U,  55U },   /* 3.75 V ->  55% */
    { 3800U,  60U },   /* 3.80 V ->  60% */
    { 3850U,  65U },   /* 3.85 V ->  65% */
    { 3900U,  70U },   /* 3.90 V ->  70% */
    { 3950U,  75U },   /* 3.95 V ->  75% */
    { 4000U,  80U },   /* 4.00 V ->  80% */
    { 4050U,  85U },   /* 4.05 V ->  85% */
    { 4100U,  90U },   /* 4.10 V ->  90% */
    { 4150U,  95U },   /* 4.15 V ->  95% */
    { 4200U, 100U }    /* 4.20 V -> 100% (Totalmente Carregada) */
};

/**
 * @brief  Estima o SoC (%) a partir da tensão média por célula (OCV lookup)
 *
 * Para que serve: converte a tensão média em circuito aberto numa percentagem de
 * carga, por interpolação linear na tabela OCV. Só é fiável em relaxação
 * (contactor aberto, corrente ≈ 0) — daí ser chamado nessa condição em
 * BMS_Task_100ms. Faz clamp nos extremos da tabela.
 */
uint8_t BMS_EstimateSoC(uint16_t avg_cell_mv)
{
    if (avg_cell_mv <= g_soc_table[0].mv)
    {
        return g_soc_table[0].soc;
    }
    if (avg_cell_mv >= g_soc_table[BMS_SOC_TABLE_SIZE - 1U].mv)
    {
        return g_soc_table[BMS_SOC_TABLE_SIZE - 1U].soc;
    }

    for (uint8_t i = 1U; i < BMS_SOC_TABLE_SIZE; i++)
    {
        if (avg_cell_mv <= g_soc_table[i].mv)
        {
            uint16_t mv_lo  = g_soc_table[i - 1U].mv;
            uint16_t mv_hi  = g_soc_table[i].mv;
            uint8_t  soc_lo = g_soc_table[i - 1U].soc;
            uint8_t  soc_hi = g_soc_table[i].soc;

            uint32_t delta_mv  = (uint32_t)(avg_cell_mv - mv_lo);
            uint32_t range_mv  = (uint32_t)(mv_hi - mv_lo);
            uint32_t delta_soc = (uint32_t)(soc_hi - soc_lo);

            if (range_mv == 0U) { return soc_lo; }
            return (uint8_t)(soc_lo + (uint8_t)((delta_mv * delta_soc) / range_mv));
        }
    }
    return 100U;
}
