/**
 * @file    bq796xx_bms.h
 * @brief   BMS Driver - STM32F446 + BQ79600-Q1 Bridge + 2x BQ79616-Q1 Slaves
 *          Topologia: Anel Daisy-Chain Isolado (Ring)
 *          Configuração: 2 Slaves x 15 células = 30 células totais (NMC, 4.20 V)
 *
 *  ARQUITECTURA DE ACTUAÇÃO (v3.3 — ACTUALIZADA):
 *  Esta camada (driver BQ796xx) continua a ser um MONITOR que DECIDE e
 *  REPORTA: calcula as decisões LÓGICAS de segurança (contactor_closed,
 *  bms_ok, precharge_ready) e guarda-as no handle; NÃO acciona pinos.
 *  A ACTUAÇÃO FÍSICA dos relés/contactores e do LED cluster é feita pelo
 *  módulo bms_relays.c, que corre NESTE MESMO MCU — já NÃO existe um módulo
 *  MASTER externo. A máquina de segurança SAFE/ENGAGED/CHARGING/NOT_SAFE
 *  (bms_relays) lê estas decisões + a tensão do bus/pack e comanda o hardware.
 *
 *  ⚠ Isto SUPERSEDE a nota da v3.2 ("sem actuação GPIO neste MCU"). Esta
 *    alteração de arquitectura DEVE ser registada no FMEA/FTA do projecto.
 *
 *  PINOS USADOS NESTE MCU:
 *    PA0/PA1        UART4  (bridge BQ79600, 1 Mbps, DMA)
 *    PA2/PA3        USART2 (telemetria/debug TX-only, 115200)
 *    PA8            NFAULT (entrada, EXTI8 falling, pull-up)
 *    PA13/PA14/PB3  SWD/SWO (programação + trace; CubeMX "Trace Asynchronous SW")
 *    Relés:   PC0 pré-carga  PC1 charge  PC2 descarga  PC4 BMS_relay  PA6 BMS_charge
 *    LED:     PA15 verde  PC11 vermelho  PC12 azul
 *    Monitor: PB0 IMD  PC8 TSMS  PC6 ESDB  PB14 ESDB_chg  PB12 charger_sig
 *  (mapa completo, polaridades e conflitos resolvidos: ver bms_relays.h)
 *
 * @version 3.3.0
 */

#ifndef BQ796XX_BMS_H
#define BQ796XX_BMS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* =========================================================================
 * RASTREABILIDADE DE BUILD
 * ========================================================================= */
#define BMS_FW_VERSION_MAJOR        3U
#define BMS_FW_VERSION_MINOR        3U
#define BMS_FW_VERSION_PATCH        0U
#define BMS_FW_VERSION_STRING       "3.3.0"

/* =========================================================================
 * CONFIGURAÇÃO GERAL DO SISTEMA
 * ========================================================================= */
#define BMS_NUM_SLAVES              2U
#define BMS_CELLS_PER_SLAVE         15U
#define BMS_TOTAL_CELLS             (BMS_NUM_SLAVES * BMS_CELLS_PER_SLAVE)  /* 30 */
#define BMS_UART_BAUDRATE           1000000UL   /* 1 Mbps */
#define BMS_UART_TIMEOUT_MS         50U
#define BMS_MAX_FRAME_SIZE          64U
#define BMS_MAX_RESPONSE_PAYLOAD    128U

/* =========================================================================
 * WATCHDOG INDEPENDENTE (IWDG) — ASIL-D OBRIGATÓRIO
 * =========================================================================
 * Se o super-loop bloquear (hang do CPU, deadlock, corrupção de stack),
 * o IWDG do STM32 (oscilador LSI ~32 kHz, independente do clock principal)
 * reseta o MCU se não for refrescado.
 *
 * Configuração: PSC=IWDG_PRESCALER_64, RLR = (500ms × 32000Hz) / 64 = 250
 * Timeout real: ~500 ms. BMS_IWDG_Refresh() em CADA iteração do super-loop.
 * Independente do SYSCLK (84 MHz) — corre do LSI.
 *
 * INICIALIZAÇÃO (Via B): o IWDG é inicializado pelo CubeMX (MX_IWDG_Init em
 * main.c), com estes mesmos parâmetros (Prescaler=64, Reload=250). Fica armado
 * logo no arranque, ANTES de BMS_Main. Como o boot é curto (~170 ms) face ao
 * timeout (~500 ms), não há risco de reset prematuro no caminho normal. Se o
 * arranque falhar e o MCU ficar preso, o IWDG reseta e RE-TENTA o init
 * (auto-recuperação de falhas transitórias do BQ79600). Os defines abaixo
 * servem de referência única da configuração e são usados na verificação.
 *
 * NOTA (actuação dos relés): a temporização longa da pré-carga (750 ms) NÃO
 * pode usar HAL_Delay no super-loop — excederia o timeout do IWDG. Em
 * bms_relays.c essa espera é NÃO-BLOQUEANTE (baseada em HAL_GetTick). */
#define BMS_IWDG_PRESCALER          IWDG_PRESCALER_64
#define BMS_IWDG_RELOAD             250U     /* ~500 ms timeout */

/* =========================================================================
 * FILTRO DE MÉDIA MÓVEL PARA TENSÕES CELULARES
 * =========================================================================
 * IIR exponencial: filtered = (raw + (ALPHA-1) × filtered) / ALPHA
 * ALPHA=4 → 25% peso para leitura nova, 75% história. */
#define BMS_VOLTAGE_FILTER_ALPHA    4U

/* =========================================================================
 * ESTIMAÇÃO DE ESTADO DE CARGA (SoC) — OCV Lookup
 * ========================================================================= */
#define BMS_SOC_TABLE_SIZE          21U

/* =========================================================================
 * COMM CLEAR (Protocolo BQ79600)
 * ========================================================================= */
#define BMS_COMM_CLEAR_BITS         18U   /* 18 períodos de bit = 18 µs */

/* Endereços dos dispositivos */
#define BMS_ADDR_BRIDGE             0x00U
#define BMS_ADDR_SLAVE1             0x01U
#define BMS_ADDR_SLAVE2             0x02U

/* =========================================================================
 * BYTES DE INICIALIZAÇÃO (INIT BYTE) — BQ79600-Q1 Protocol Layer
 * ========================================================================= */
#define INIT_BASE_SINGLE_WRITE      0x98U   /* bit7=1, scope=Single, W=1 */
#define INIT_BASE_SINGLE_READ       0x90U   /* bit7=1, scope=Single, W=0 */
#define INIT_BASE_BROADCAST_WRITE   0x88U   /* bit7=1, scope=Broadcast, W=1 */
#define INIT_BASE_BROADCAST_READ    0x80U   /* bit7=1, scope=Broadcast, W=0 */
#define INIT_BASE_STACK_WRITE       0xA8U   /* bit7=1, scope=Stack, W=1 */
#define INIT_BASE_BCAST_REV_WRITE   0xF8U   /* bit7=1, scope=BcastRev, W=1, size=000=1byte */

/**
 * Constrói dinamicamente o INIT byte codificando o tamanho do payload.
 * @param base  Constante INIT_BASE_* adequada ao tipo de frame
 * @param N     Número de bytes de dados (1..8)
 * @return      INIT byte completo com size bits codificados
 */
#define BMS_INIT(base, N)  ((uint8_t)((base) | (((uint8_t)(N) - 1U) & 0x07U)))

/* =========================================================================
 * REGISTOS DA BRIDGE (BQ79600-Q1)
 * ========================================================================= */
#define REG_BRIDGE_DIR0_ADDR        0x0306U  /* Endereçamento caminho principal */
#define REG_BRIDGE_DIR1_ADDR        0x0307U  /* Endereçamento caminho reverso */
#define REG_BRIDGE_COMM_CTRL        0x0308U  /* Controlo comunicação */
#define REG_BRIDGE_CONTROL1         0x0309U  /* Controlo geral (SEND_WAKE, DIR_SEL, ADDR_WR) */

#define REG_BRIDGE_FAULT_COMM1      0x0520U
#define REG_BRIDGE_FAULT_COMM2      0x0521U  /* Ring break */
#define REG_BRIDGE_FAULT_COMM3      0x0522U  /* Heartbeat fail */
#define REG_BRIDGE_FAULT_SUMMARY    0x2100U

#define REG_ECC_DATA1               0x0343U
#define REG_ECC_DATA8               0x034AU

/* =========================================================================
 * REGISTOS DOS SLAVES (BQ79616-Q1)
 * ========================================================================= */
#define REG_ACTIVE_CELL             0x0003U  /* Número de células activas */

#define REG_OV_THRESH               0x0009U  /* Limiar sobretensão */
#define REG_UV_THRESH               0x000AU  /* Limiar subtensão */
#define REG_OVUV_CTRL               0x032CU  /* Controlo protecção OV/UV */

#define REG_ADC_CONF1               0x0007U  /* Filtro passa-baixo */
#define REG_ADC_CTRL1               0x030DU  /* Controlo ADC (MAIN_GO, MAIN_MODE, LPF) */

/* Leitura de Tensão das Células — BQ79616-Q1 ADC Result Registers
 *   VCELL16_HI = 0x0568 (topo) ... VCELL1_HI = 0x0586 (base)
 *   FÓRMULA: VCELL_N_HI = REG_VCELL16_HI + (16 - N) * 2
 *   CONFIG 15S: VC16 curto a VC15; ler 15 células a partir de VCELL15_HI. */
#define REG_VCELL16_HI              0x0568U  /* VCELL16 = topo, endereço base */
#define REG_VCELL_STRIDE            2U        /* 2 bytes por célula (HI + LO) */
#define REG_VCELL15_HI  (REG_VCELL16_HI + 1U * REG_VCELL_STRIDE)  /* 0x056A */
#define REG_VCELL1_HI   (REG_VCELL16_HI + 15U * REG_VCELL_STRIDE) /* 0x0586 */
#define BMS_VCELL_READ_BYTES        (BMS_CELLS_PER_SLAVE * REG_VCELL_STRIDE)

/* Leitura de Temperatura — bloco contiguo 0x0588-0x0591 (GPIO1-4 + TSREF) */
#define REG_GPIO1_HI                0x0588U
#define REG_GPIO1_LO                0x0589U
#define REG_GPIO2_HI                0x058AU
#define REG_GPIO2_LO                0x058BU
#define REG_GPIO3_HI                0x058CU
#define REG_GPIO3_LO                0x058DU
#define REG_GPIO4_HI                0x058EU  /* HV bus sensor */
#define REG_GPIO4_LO                0x058FU
#define REG_TSREF_HI                0x0590U
#define REG_TSREF_LO                0x0591U

#define BMS_NUM_TEMP_SENSORS        3U    /* GPIO1, GPIO2, GPIO3 por slave */
#define BMS_AUX_READ_BYTES          10U   /* GPIO1+GPIO2+GPIO3+GPIO4+TSREF = 5x2 */

/* =========================================================================
 * PRÉ-CARGA / TENSÃO HV DO BARRAMENTO (GPIO4 do Slave 1)
 * =========================================================================
 * BUG-FIX (v3.3): o divisor resistivo do barramento HV é 27.11, não 27. Usar
 * um inteiro (27U) introduzia ~0.4% de erro por defeito (a 126 V lia ~125.5 V),
 * o que desloca o limiar de pré-carga. Passou a ponto-fixo NUM/DEN:
 *     V_bus_mV = V_adc_mV × HV_BUS_ATTENUATION_NUM / HV_BUS_ATTENUATION_DEN
 * (sem overflow: V_adc_mV ≤ ~12500 mV → ×2711 ≈ 3.4e7, cabe em uint32). */
#define HV_BUS_ATTENUATION_NUM      2711U     /* 27.11 × 100 */
#define HV_BUS_ATTENUATION_DEN      100U
#define PRECHARGE_THRESHOLD_MV      113400UL  /* 90% × 126 V (30S × 4.20 V) */

/* =========================================================================
 * INTERLOCKS LÓGICOS (calculados aqui) + ACTUAÇÃO (módulo bms_relays)
 * =========================================================================
 * Esta camada calcula e guarda no handle as decisões LÓGICAS de segurança:
 *   hbms->contactor_closed, hbms->bms_ok, hbms->precharge_ready.
 * NÃO acciona pinos. O módulo bms_relays.c (mesmo MCU) consome estes valores
 * e a tensão do bus/pack para actuar fisicamente os relés e o LED cluster.
 * A telemetria USART2 reporta tanto a DECISÃO lógica (ctor/ok/pre) como o
 * ESTADO REAL dos relés (rly[...]) — ver bms_master_comm.c.
 *
 * Os antigos #define BMS_OK_PORT/PIN, PRECHARGE_OK_PORT/PIN e o contactor
 * em GPIOB foram removidos desta camada; a pinagem física de actuação vive
 * agora em bms_relays.h. */

/* =========================================================================
 * PINO DE TX DA BRIDGE (controlo GPIO directo para pulsos WAKE/SHUTDOWN/RESET/COMM CLEAR)
 * =========================================================================
 * Durante os pulsos de gestão de energia e o COMM CLEAR, a linha TX é
 * temporariamente desligada do periférico UART e controlada como GPIO puro.
 *
 * HARDWARE ACTUAL: UART4 em PA0 (TX) / PA1 (RX) — STM32F446RET6, AF8.
 *   PA0 (pino 14) = UART4_TX     PA1 (pino 15) = UART4_RX */
#define BMS_BRIDGE_TX_PORT          GPIOA
#define BMS_BRIDGE_TX_PIN           GPIO_PIN_0   /* PA0 = UART4_TX (AF8) */
#define BMS_BRIDGE_RX_PIN           GPIO_PIN_1   /* PA1 = UART4_RX (AF8) */

/* =========================================================================
 * PINO DE NFAULT (entrada da bridge → EXTI)
 * =========================================================================
 * HARDWARE ACTUAL: NFAULT da BQ79600 ligado a PA8.
 *   EXTI linha 8 → vector EXTI9_5_IRQHandler (gerado pelo CubeMX).
 *   Configurar PA8 como GPIO_EXTI8, Falling edge, Pull-up, NVIC prioridade 0.
 *   (A versão original usava PC13/EXTI13; alterado para PA8.)
 *
 * NOTA: os monitores de segurança (IMD/TSMS/ESDB/...) em bms_relays.c são
 * lidos por POLLING com debounce — NÃO usam EXTI — para não colidir com a
 * linha EXTI9_5 partilhada pelo NFAULT. */
#define BMS_NFAULT_PORT             GPIOA
#define BMS_NFAULT_PIN              GPIO_PIN_8   /* PA8 = NFAULT (EXTI8) */

/* Parametrização NTC Steinhart-Hart (NTC 10 kΩ, β=3950 K, R1=10 kΩ) */
#define BMS_NTC_TABLE_SIZE          21U

/* Fault Summary dos Slaves */
#define REG_SLAVE_FAULT_SUMMARY     0x052DU
#define REG_SLAVE_FAULT_OV          0x052EU  /* Over-voltage por célula */
#define REG_SLAVE_FAULT_UV          0x052FU  /* Under-voltage por célula */
#define REG_SLAVE_FAULT_VCOW        0x0530U  /* Open wire detection */
#define REG_SLAVE_FAULT_COMM        0x0531U

/* Registos de limpeza de latch (hardware latching clear) */
#define REG_FAULT_RST1              0x0331U  /* Reset OV/UV/COMP/VCOW latches */
#define REG_FAULT_RST2              0x0332U  /* Reset COMM fault latches */
#define FAULT_CLEAR_VAL             0xFFU    /* Valor de escrita para limpeza total */

/* =========================================================================
 * VALORES E MÁSCARAS DE CONFIGURAÇÃO
 * ========================================================================= */
#define CTRL1_SEND_WAKE             0x20U    /* Bit SEND_WAKE */
#define CTRL1_ADDR_WR               0x01U    /* Bit ADDR_WR */
#define CTRL1_DIR_SEL               0x02U    /* Bit DIR_SEL */

#define COMM_CTRL_STACK_DEV         0x02U    /* STACK_DEV = 1 */
#define COMM_CTRL_TOP_STACK         0x03U    /* TOP_STACK = 1 */

#define ADC_CTRL1_CONTINUOUS_LPF    0x2EU    /* MAIN_GO=1, MAIN_MODE=0b10, LPF_EN=1 */

#define OVUV_CTRL_ENABLE            0x06U

#define ACTIVE_CELL_15S             0x0FU

/* Thresholds de tensão dos comparadores autónomos de hardware (NMC)
 * V_OV = 2700 mV + OV_THRESH × 25 mV → 0x3E (62) = 4250 mV
 * V_UV = 2100 mV + UV_THRESH × 25 mV → 0x24 (36) = 3000 mV */
#define OV_THRESH_VAL               0x3EU   /* 4250 mV - Falha hardware de Sobretensão */
#define UV_THRESH_VAL               0x24U   /* 3000 mV - Falha hardware de Subtensão */

/* =========================================================================
 * BALANCEAMENTO CELULAR PASSIVO (BQ79616-Q1)
 * ========================================================================= */
#define REG_CB_CELL1_CTRL           0x0030U  /* bits[7:0] = células 1-8 */
#define REG_CB_CELL9_CTRL           0x0031U  /* bits[7:0] = células 9-16 */
#define REG_BAL_CTRL1               0x002EU  /* Timer de auto-stop de balanceamento */
#define REG_BAL_CTRL2               0x002FU  /* Enable auto-stop + período de cool-down */

#define BAL_CTRL1_TIMER_10MIN       0x2AU
#define BAL_CTRL2_AUTOSTOP_EN       0x01U

#define CELL_BALANCE_DELTA_MV       20U     /* Desequilíbrio mínimo para iniciar */
#define CELL_BALANCE_HYSTERESIS_MV  10U     /* Balancear células > min + histerese */
#define CELL_BALANCE_MIN_MV         3800U   /* Tensão mínima para iniciar balanceamento (NMC Top-Balancing) */
#define CELL_BALANCE_STOP_MV        5U      /* Parar quando delta < 5 mV */

/* =========================================================================
 * GESTÃO DE ENERGIA — PULSOS DE LINHA TX (PA0/UART4)
 * ========================================================================= */
#define DELAY_WAKE_PULSE_US         2500U   /* SHUTDOWN → WAKE (2.5 ms LOW) */
#define DELAY_SHUTDOWN_PULSE_US     9000U   /* WAKE → SHUTDOWN (9 ms LOW) */
#define DELAY_HWRESET_PULSE_US      40000U  /* Reset completo da rede (40 ms LOW) */
#define DELAY_SLEEP_CONTACTOR_MS    100U    /* Aguardar abertura mecânica do contactor */
#define BMS_DAISY_CHAIN_LATENCY_US  2000U
#define DELAY_OSC_STAB_MS           2U       /* Estabilização osciladores */
#define DELAY_WAKE_PROPAGATION_MS   5U       /* Propagação WAKE pela rede */
#define DELAY_DIR_SEL_SWITCH_US     100U     /* Estabilização após troca DIR_SEL */
#define DELAY_ADC_SETTLE_MS         10U      /* Estabilização ADC */

/* =========================================================================
 * LIMITES DE TENSÃO E TEMPERATURA (Software)
 * ========================================================================= */
#define CELL_OV_MV                  4250U   /* Falha por Sobretensão absoluta (Over-Voltage): 4.25 V */
#define CELL_UV_MV                  3000U   /* Falha por Subtensão absoluta (Under-Voltage): 3.00 V */
#define CELL_WARN_UV_MV             3100U    /* 3100 mV - aviso subtensão (reservado) */
#define CELL_WARN_OV_MV             4150U    /* 4150 mV - aviso de sobretensão (aproximação do limite) */
#define CELL_TEMP_MAX_C             60U      /* 60°C - temperatura máxima */
#define CELL_TEMP_WARN_C            55U      /* 55°C - aviso temperatura */
#define CELL_IMBALANCE_MV           50U      /* 50 mV - desequilíbrio máximo */

/* =========================================================================
 * CRC-16-IBM
 * ========================================================================= */
#define CRC16_INIT                  0xFFFFU
#define CRC16_POLY_LSB_FIRST        0xA001U  /* 0x8005 reflectido para UART LSB-first */

/* =========================================================================
 * ENUMERAÇÕES
 * ========================================================================= */

/** Estado geral do BMS */
typedef enum {
    BMS_STATE_UNINITIALIZED = 0,
    BMS_STATE_INITIALIZING,
    BMS_STATE_IDLE,
    BMS_STATE_MONITORING,
    BMS_STATE_BALANCING,       /* Monitorização activa + balanceamento passivo */
    BMS_STATE_FAULT,
    BMS_STATE_SHUTDOWN,
    BMS_STATE_SLEEP,           /* Pack em SHUTDOWN mode de baixo consumo */
    BMS_STATE_RING_RECOVERY
} BMS_State_t;

/** Tipo de operação de comunicação */
typedef enum {
    BMS_CMD_SINGLE_WRITE = 0,
    BMS_CMD_SINGLE_READ,
    BMS_CMD_BROADCAST_WRITE,
    BMS_CMD_BROADCAST_READ,
    BMS_CMD_STACK_WRITE,
    BMS_CMD_STACK_READ,
    BMS_CMD_BROADCAST_WRITE_REV
} BMS_CmdType_t;

/** Flags de fault */
typedef enum {
    BMS_FAULT_NONE          = 0x00000000U,
    BMS_FAULT_OV            = 0x00000001U,  /* Over-voltage */
    BMS_FAULT_UV            = 0x00000002U,  /* Under-voltage */
    BMS_FAULT_OT            = 0x00000004U,  /* Over-temperature */
    BMS_FAULT_COMM          = 0x00000008U,  /* Falha comunicação */
    BMS_FAULT_RING_BREAK    = 0x00000010U,  /* Rotura anel */
    BMS_FAULT_OPEN_WIRE     = 0x00000020U,  /* Circuito aberto */
    BMS_FAULT_HB_FAIL       = 0x00000040U,  /* Heartbeat fail */
    BMS_FAULT_CRC           = 0x00000080U,  /* Erro CRC */
} BMS_FaultFlag_t;

/** Resultado das operações */
typedef enum {
    BMS_OK = 0,
    BMS_ERR_TIMEOUT,
    BMS_ERR_CRC,
    BMS_ERR_COMM,
    BMS_ERR_INVALID_PARAM,
    BMS_ERR_FAULT_ACTIVE,
    BMS_ERR_INIT_FAILED
} BMS_Status_t;

/* =========================================================================
 * ESTRUTURAS DE DADOS
 * ========================================================================= */

/** Dados de um slave individual */
typedef struct {
    uint8_t     address;
    uint8_t     address_rev;
    uint16_t    cell_voltage_mv[BMS_CELLS_PER_SLAVE];
    int16_t     temperatures_c[BMS_NUM_TEMP_SENSORS];  /* [0]=GPIO1 [1]=GPIO2 [2]=GPIO3 */
    uint32_t    fault_flags;
    uint8_t     fault_summary_raw[4];
    bool        ov_cell[BMS_CELLS_PER_SLAVE];
    bool        uv_cell[BMS_CELLS_PER_SLAVE];
    bool        comm_ok;
    uint16_t    balance_mask;   /* Bitmask de células em balanceamento activo */
} BMS_SlaveData_t;

/** Estrutura principal do BMS */
typedef struct {
    /* Estado */
    BMS_State_t         state;
    uint32_t            fault_flags;            /* OR de todos os slaves */
    bool                ring_intact;            /* Anel fisicamente completo */
    bool                ring_using_reverse;     /* Testemunha: firmware a usar DIR1 após ring break */

    /* DECISÕES LÓGICAS (consumidas por bms_relays para actuar) */
    volatile bool       contactor_closed;       /* DECISÃO: contactor deve estar fechado?
                                                 * (escrito na ISR NFAULT → volatile) */
    bool                bms_ok;                 /* INTERLOCK lógico BMS_OK */

    /* Dados dos Slaves */
    BMS_SlaveData_t     slave[BMS_NUM_SLAVES];

    /* Métricas globais */
    uint32_t            pack_voltage_mv;        /* Soma das tensões (uint32: 30S×4250mV>UINT16_MAX) */
    uint16_t            min_cell_mv;
    uint16_t            max_cell_mv;
    uint16_t            delta_cell_mv;          /* Desequilíbrio */
    int16_t             max_temp_c;

    /* Contadores */
    uint32_t            fault_count;
    uint32_t            ring_recovery_count;
    uint32_t            comm_error_count;
    uint32_t            crc_error_count;

    /* Handles HAL (inicializados externamente) */
    UART_HandleTypeDef  *huart;
    TIM_HandleTypeDef   *htim_delay;           /* Timer µs */

    /* Pre-carga e tensão HV */
    uint32_t            inverter_voltage_mv;
    bool                precharge_ready;        /* PRECHARGE_OK lógico */

    /* Balanceamento passivo */
    bool                is_balancing;
    uint32_t            balance_cycle_count;

    /* Estimação de Estado de Carga (OCV lookup) */
    uint8_t             soc_percent;            /* 0-100% */

    /* Filtro de tensão (média móvel IIR por célula) */
    uint16_t            filtered_mv[BMS_NUM_SLAVES][BMS_CELLS_PER_SLAVE];
    bool                filter_primed;          /* TRUE após 1ª leitura (seed do filtro) */

    /* POST (Power-On Self Test) */
    bool                post_passed;            /* TRUE se POST completou sem erros */

    /* Flags de interrupção - acesso atómico obrigatório */
    volatile uint32_t   nfault_pending;
    volatile uint32_t   dma_rx_done;

    uint8_t             tx_buf[BMS_MAX_FRAME_SIZE];
    uint8_t             rx_buf[BMS_MAX_RESPONSE_PAYLOAD];
} BMS_Handle_t;

/* =========================================================================
 * PROTÓTIPOS DE FUNÇÕES PÚBLICAS
 * ========================================================================= */

/* --- Inicialização e Configuração --- */
BMS_Status_t BMS_Init(BMS_Handle_t *hbms, UART_HandleTypeDef *huart,
                       TIM_HandleTypeDef *htim);
BMS_Status_t BMS_AutoAddressing(BMS_Handle_t *hbms);
BMS_Status_t BMS_ConfigureSlaves(BMS_Handle_t *hbms);

/* --- Monitorização Cíclica --- */
BMS_Status_t BMS_Task_100ms(BMS_Handle_t *hbms);
BMS_Status_t BMS_ReadAllCellVoltages(BMS_Handle_t *hbms);
BMS_Status_t BMS_ReadAllTemperatures(BMS_Handle_t *hbms);
BMS_Status_t BMS_ReadInverterVoltage(BMS_Handle_t *hbms);
BMS_Status_t BMS_CheckProtections(BMS_Handle_t *hbms);

/* --- Balanceamento Celular Passivo --- */
BMS_Status_t BMS_SetCellBalancing(BMS_Handle_t *hbms, uint8_t slave_addr,
                                   uint16_t cell_mask);
BMS_Status_t BMS_StopAllBalancing(BMS_Handle_t *hbms);
BMS_Status_t BMS_RunPassiveBalancing(BMS_Handle_t *hbms);

/* --- Tratamento de Falhas --- */
void         BMS_NFAULT_IRQHandler(BMS_Handle_t *hbms);
BMS_Status_t BMS_ProcessFaults(BMS_Handle_t *hbms);
BMS_Status_t BMS_RingRecovery(BMS_Handle_t *hbms);
void         BMS_EmergencyShutdown(BMS_Handle_t *hbms);

/* --- Gestão de Potência e Estado (decisões lógicas — sem GPIO) --- */
void         BMS_ContactorOpen(BMS_Handle_t *hbms);
void         BMS_ContactorClose(BMS_Handle_t *hbms);
void         BMS_SendShutdownPulse(BMS_Handle_t *hbms);
//void         BMS_SendHardwareReset(BMS_Handle_t *hbms);   //nao e usada
void         BMS_EnterSleep(BMS_Handle_t *hbms);
void         BMS_UpdateHardwareInterlocks(BMS_Handle_t *hbms);

/* --- Segurança Funcional (ASIL-D) --- */
/* IWDG inicializado pelo CubeMX (MX_IWDG_Init em main.c). A aplicação apenas
 * o refresca no super-loop. NÃO existe BMS_IWDG_Init — ver nota na secção
 * WATCHDOG INDEPENDENTE (IWDG) mais acima. */
void         BMS_IWDG_Refresh(void);
BMS_Status_t BMS_PowerOnSelfTest(BMS_Handle_t *hbms);
void         BMS_CommClear(BMS_Handle_t *hbms);

/* --- Processamento de Sinal e SoC --- */
void         BMS_ApplyVoltageFilter(BMS_Handle_t *hbms);
uint8_t      BMS_EstimateSoC(uint16_t avg_cell_mv);

/* --- Comunicação de Baixo Nível --- */
BMS_Status_t BMS_WriteSingleDevice(BMS_Handle_t *hbms, uint8_t dev_addr,
                                    uint16_t reg_addr, uint8_t *data,
                                    uint8_t data_len);
BMS_Status_t BMS_ReadSingleDevice(BMS_Handle_t *hbms, uint8_t dev_addr,
                                   uint16_t reg_addr, uint8_t *rx_data,
                                   uint8_t data_len);
BMS_Status_t BMS_WriteBroadcast(BMS_Handle_t *hbms, uint16_t reg_addr,
                                 uint8_t *data, uint8_t data_len);
BMS_Status_t BMS_ReadBroadcast(BMS_Handle_t *hbms, uint16_t reg_addr,
                                uint8_t *rx_data, uint8_t data_len_per_dev);
BMS_Status_t BMS_WriteBroadcastReverse(BMS_Handle_t *hbms, uint16_t reg_addr,
                                        uint8_t init_byte, uint8_t *data,
                                        uint8_t data_len);

/* --- Utilitários --- */
uint16_t     BMS_CalculateCRC16(uint8_t *data, uint16_t length);
const char  *BMS_GetStateString(BMS_State_t state);
const char  *BMS_GetFaultString(uint32_t fault_flags);
void         BMS_DelayMs(uint32_t ms);
void         BMS_DelayUs(BMS_Handle_t *hbms, uint32_t us);

#endif /* BQ796XX_BMS_H */
