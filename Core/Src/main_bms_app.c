/**
 * @file    main_bms_app.c
 * @brief   BMS - Aplicação Principal e Ponto de Entrada (Arquitectura Centralizada)
 *
 *  ARQUITECTURA CENTRALIZADA:
 *  Este STM32F446RET6 (placa Master) é o único MCU que governa o pack.
 *  Centraliza a máquina de estados do BMS, a interface com o IMD (Insulation
 *  Monitoring Device) e a telemetria (CAN/UART). DECIDE e REPORTA; a actuação
 *  física do contactor é delegada no estágio de potência.
 *
 *  PINOUT ACTUAL (apenas PA0-PA3 + PA8):
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │  PA0 (UART4_TX, AF8) ──────► BQ79600 UART_RX                 │
 *  │  PA1 (UART4_RX, AF8) ◄────── BQ79600 UART_TX                 │
 *  │  PA2 (USART2_TX) ──► Debug/Telemetria TX-only ao master      │
 *  │  PA3 (USART2_RX) ── livre (reservado p/ comandos futuros)    │
 *  │  PA8 (EXTI8 FALLING, Pull-Up) ◄── BQ79600 NFAULT (entrada)   │
 *  │  TIM2 (free-running µs)   TIM6 (HAL timebase 1 ms)           │
 *  │  DMA1 (UART4_TX)   DMA1 (UART4_RX)                           │
 *  └──────────────────────────────────────────────────────────────┘
 *  Sem pinos de contactor/interlock neste MCU — as decisões viajam para o
 *  estágio de potência via telemetria + reporte por evento (ver abaixo).
 *
 *  CONFIGURAÇÃO CubeMX OBRIGATÓRIA (clock 84 MHz):
 *    RCC    -> HSI -> PLL: PLLM=16, PLLN=336, PLLP=4 -> SYSCLK 84 MHz
 *    Bus    -> AHB÷1 (84 MHz), APB1÷2 (42 MHz), APB2÷1 (84 MHz)
 *    UART4  -> Asynchronous, 1000000 8N1, DMA RX+TX Normal
 *    USART2 -> Asynchronous, 115200 8N1 (debug/telemetria TX-only)
 *    TIM2   -> Internal Clock, Prescaler=83 (1 µs/tick), Period=0xFFFFFFFF, sem IT
 *    EXTI8  -> PA8 GPIO_EXTI8, Falling edge, Pull-up; NVIC EXTI9_5 prioridade 0
 *    HAL Timebase Source -> TIM6 (alimenta HAL_GetTick — cadência de 100 ms)
 *    IWDG   -> ACTIVADO no CubeMX (MX_IWDG_Init): Prescaler=64, Reload=250
 *              (~500 ms). A aplicação só faz BMS_IWDG_Refresh no super-loop.
 *
 *  ⚠ NOTA DE SEGURANÇA (resumo — detalhe em bq796xx_bms_monitor.c, Secção 9):
 *  A decisão de abrir o contactor é tomada aqui mas actuada no estágio de
 *  potência. Para cumprir o FTTI, a decisão é emitida POR EVENTO (reporte
 *  imediato quando contactor_closed/bms_ok mudam), não apenas na telemetria
 *  de 1 Hz. Recomenda-se interlock BMS_OK de HARDWARE dedicado (ASIL-D).
 *
 * @version 3.2.0
 */

#include "bq796xx_bms.h"
#include "bms_master_comm.h"
#include <stdio.h>
#include <stdbool.h>

/* =========================================================================
 * VARIÁVEIS GLOBAIS DA APLICAÇÃO
 * ========================================================================= */

static BMS_Handle_t      g_bms;
BMS_MasterComm_t         g_master_comm;

extern UART_HandleTypeDef  huart4;   /* UART4 — BQ79600 bridge (PA0/PA1) */
extern UART_HandleTypeDef  huart2;   /* USART2 (PA2/PA3) — debug/telemetria TX-only */
extern TIM_HandleTypeDef   htim2;    /* TIM2  — Delay µs (contador livre) */

#define BMS_TASK_PERIOD_MS   100U

/* =========================================================================
 * FUNÇÕES DE APOIO À APLICAÇÃO
 * ========================================================================= */

/**
 * @brief  Lógica de controlo do contactor (decisão lógica reportada ao master)
 */
static void BMS_ContactorControl(BMS_Handle_t *hbms)
{
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return;
    }

    if (hbms->state == BMS_STATE_MONITORING &&
        hbms->fault_flags == BMS_FAULT_NONE)
    {
        if (!hbms->contactor_closed)
        {
            BMS_ContactorClose(hbms);   /* Barreiras internas reforçam aqui */
        }
    }
    else
    {
        if (hbms->contactor_closed)
        {
            BMS_ContactorOpen(hbms);
        }
    }
}

/**
 * @brief  Limpeza física dos registos de latch de fault em todos os dispositivos
 */
static BMS_Status_t BMS_ClearHardwareFaultLatches(BMS_Handle_t *hbms)
{
    uint8_t      clear_val = FAULT_CLEAR_VAL;  /* 0xFF */
    BMS_Status_t status;

    status = BMS_WriteBroadcast(hbms, REG_FAULT_RST1, &clear_val, 1U);
    if (status != BMS_OK) { return status; }

    status = BMS_WriteBroadcast(hbms, REG_FAULT_RST2, &clear_val, 1U);
    if (status != BMS_OK) { return status; }

    /* Latches da Bridge (Broadcast não chega à bridge — precisa Single) */
    (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                REG_FAULT_RST1, &clear_val, 1U);
    (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                REG_FAULT_RST2, &clear_val, 1U);

    return BMS_OK;
}

/**
 * @brief  Tentativa de recuperação e reset de faults após shutdown
 */
static void BMS_FaultRecoveryAttempt(BMS_Handle_t *hbms,
                                      uint32_t *retry_counter)
{
    (*retry_counter)++;

    if (*retry_counter > 3U)
    {
        /* Adormece fisicamente o barramento; o guard de SLEEP em
         * BMS_Task_100ms quebra o ciclo infinito FAULT→SHUTDOWN→FAULT. */
        BMS_EnterSleep(hbms);
        return;
    }

    BMS_ReadAllCellVoltages(hbms);
    BMS_ReadAllTemperatures(hbms);

    bool recoverable = true;

    if ((hbms->fault_flags & BMS_FAULT_OV) &&
        (hbms->max_cell_mv > (CELL_OV_MV - 50U)))
    {
        recoverable = false;
    }
    if ((hbms->fault_flags & BMS_FAULT_UV) &&
        (hbms->min_cell_mv < (CELL_UV_MV + 50U)))
    {
        recoverable = false;
    }
    if ((hbms->fault_flags & BMS_FAULT_OT) &&
        (hbms->max_temp_c > (int16_t)(CELL_TEMP_MAX_C - 5)))
    {
        recoverable = false;
    }
    /* Open Wire: falha física — não recuperável por software */
    if (hbms->fault_flags & BMS_FAULT_OPEN_WIRE)
    {
        recoverable = false;
    }
    /* Falha de comunicação total: não recuperável sem re-init */
    if ((hbms->fault_flags & BMS_FAULT_COMM) &&
        !(hbms->fault_flags & BMS_FAULT_RING_BREAK))
    {
        recoverable = false;
    }

    if (recoverable)
    {
        /* Limpar latches de hardware ANTES de transitar para MONITORING. */
        if (BMS_ClearHardwareFaultLatches(hbms) != BMS_OK)
        {
            hbms->fault_flags |= BMS_FAULT_COMM;
            return;
        }

        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            hbms->slave[s].fault_flags = 0U;
            for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
            {
                hbms->slave[s].ov_cell[c] = false;
                hbms->slave[s].uv_cell[c] = false;
            }
        }

        hbms->fault_flags &= ~(BMS_FAULT_OV  |
                                BMS_FAULT_UV  |
                                BMS_FAULT_OT  |
                                BMS_FAULT_HB_FAIL |
                                BMS_FAULT_CRC);

        hbms->state    = BMS_STATE_MONITORING;
        *retry_counter = 0U;
    }
}

/* =========================================================================
 * PONTO DE ENTRADA DA APLICAÇÃO BMS
 * ========================================================================= */

void BMS_Main(void)
{
    BMS_Status_t status;
    uint32_t     fault_cycle_count  = 0U;   /* ciclos em FAULT desde a última transição */
    uint32_t     fault_retry_count  = 0U;   /* tentativas de recovery efectuadas */
    uint32_t     debug_print_div    = 0U;
    uint32_t     last_task_tick;            /* timestamp do último ciclo de 100 ms */
    bool         last_contactor_decision;   /* p/ reporte por evento (FTTI) */
    bool         last_bms_ok;

    /* ------------------------------------------------------------------
     * FASE 1: Inicialização do BMS
     * ------------------------------------------------------------------ */
    printf("\r\n[BMS] Firmware v%s\r\n", BMS_FW_VERSION_STRING);
    printf("[BMS] Config: %u slaves x %u cells = %u total\r\n",
           BMS_NUM_SLAVES, BMS_CELLS_PER_SLAVE, BMS_TOTAL_CELLS);

    status = BMS_Init(&g_bms, &huart4, &htim2);
    if (status != BMS_OK)
    {
        printf("[BMS] FATAL: Initialization failed! Code: %d\r\n", (int)status);
        /* AUTO-RECUPERAÇÃO (Via B): este loop NÃO refresca o IWDG de propósito.
         * O watchdog (armado pelo CubeMX) dispara ao fim de ~500 ms e reinicia
         * o MCU, que volta a tentar BMS_Init. Cobre falhas transitórias de
         * arranque do BQ79600 (ruído, bridge ainda a estabilizar). NÃO
         * adicionar BMS_IWDG_Refresh aqui — isso eliminaria a recuperação e
         * deixaria o MCU preso sem nunca re-tentar. */
        while (1U)
        {
            BMS_DelayMs(200U);
        }
    }

    printf("[BMS] Initialization OK. State: %s\r\n",
           BMS_GetStateString(g_bms.state));
    printf("[BMS] Ring intact: %s\r\n", g_bms.ring_intact ? "YES" : "NO");

    /* ------------------------------------------------------------------
     * FASE 1b: Inicializar saída de debug/telemetria (USART2, TX-only)
     * ------------------------------------------------------------------ */
    BMS_MasterComm_Init(&g_master_comm, &huart2);
    printf("[BMS] Telemetry/debug init OK (USART2 PA2/PA3, TX-only)\r\n");

    /* ------------------------------------------------------------------
     * FASE 1c: IWDG Watchdog (ASIL-D)
     * O IWDG já foi armado pelo CubeMX (MX_IWDG_Init em main.c) ANTES de
     * BMS_Main. Aqui nada se inicializa — o refresh é feito no super-loop.
     * Se o arranque tivesse falhado, o watchdog teria resetado e re-tentado.
     * ------------------------------------------------------------------ */
    printf("[BMS] IWDG active (CubeMX MX_IWDG_Init, timeout ~500 ms)\r\n");

    /* ------------------------------------------------------------------
     * FASE 1d: Power-On Self Test (POST)
     * ------------------------------------------------------------------ */
    status = BMS_PowerOnSelfTest(&g_bms);
    if (status != BMS_OK)
    {
        printf("[BMS] POST FAILED! Fault: %s\r\n",
               BMS_GetFaultString(g_bms.fault_flags));
        g_bms.state = BMS_STATE_FAULT;
    }
    else
    {
        printf("[BMS] POST passed (CRC, Comm, ADC, NFAULT all OK)\r\n");
    }

    /* ------------------------------------------------------------------
     * FASE 2: Decisão de fechar contactor se POST e condições iniciais OK
     * ------------------------------------------------------------------ */
    BMS_DelayMs(100U);
    BMS_ReadAllCellVoltages(&g_bms);
    BMS_ReadAllTemperatures(&g_bms);
    BMS_UpdateHardwareInterlocks(&g_bms);   /* calcula bms_ok inicial */

    if ((g_bms.fault_flags == BMS_FAULT_NONE) && (g_bms.post_passed))
    {
        printf("[BMS] Conditions OK. Decision: CLOSE contactor (master actuates).\r\n");
        BMS_ContactorClose(&g_bms);
    }
    else
    {
        printf("[BMS] Decision: keep contactor OPEN: %s\r\n",
               BMS_GetFaultString(g_bms.fault_flags));
    }

    /* Reportar estado inicial ao master e semear os detectores de evento */
    BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
    last_contactor_decision = g_bms.contactor_closed;
    last_bms_ok             = g_bms.bms_ok;

    /* ------------------------------------------------------------------
     * FASE 3: Super-loop de operação
     * ------------------------------------------------------------------ */
    printf("[BMS] Entering monitoring loop.\r\n");

    last_task_tick = HAL_GetTick();

    while (1U)
    {
        /* IWDG: refrescar em TODAS as iterações (independente do tick). */
        BMS_IWDG_Refresh();

        /* NFAULT fora do tick — processamento imediato. */
        if (__atomic_load_n(&g_bms.nfault_pending, __ATOMIC_SEQ_CST) != 0U)
        {
            BMS_ProcessFaults(&g_bms);

            /* REPORTE POR EVENTO (FTTI): a decisão de abrir tem de chegar ao
             * estágio de potência de imediato, não no próximo ciclo de 1 Hz. */
            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            last_contactor_decision = g_bms.contactor_closed;
            last_bms_ok             = g_bms.bms_ok;
            debug_print_div         = 0U;
        }

        /* Cadência de 100 ms via HAL_GetTick (base TIM6). Subtracção unsigned
         * segura no wraparound de uwTick. */
        if ((HAL_GetTick() - last_task_tick) < BMS_TASK_PERIOD_MS)
        {
            BMS_DelayMs(1U);
            continue;
        }
        last_task_tick += BMS_TASK_PERIOD_MS;   /* cadência sem deriva acumulada */

        /* TAREFA PRINCIPAL DO BMS @ 100 ms */
        status = BMS_Task_100ms(&g_bms);

        /* GESTÃO DA DECISÃO DE CONTACTOR */
        BMS_ContactorControl(&g_bms);

        /* REPORTE POR EVENTO: se a DECISÃO de contactor ou o BMS_OK mudaram,
         * emitir imediatamente ao master (mitigação de latência — ver nota
         * de segurança). Não espera pelo heartbeat de 1 Hz. */
        if ((g_bms.contactor_closed != last_contactor_decision) ||
            (g_bms.bms_ok           != last_bms_ok))
        {
            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
            last_contactor_decision = g_bms.contactor_closed;
            last_bms_ok             = g_bms.bms_ok;
            debug_print_div         = 0U;
        }

        /* GESTÃO DE RECOVERY EM ESTADO FAULT */
        if (g_bms.state == BMS_STATE_FAULT)
        {
            fault_cycle_count++;

            if ((fault_cycle_count >= 50U) && (fault_retry_count <= 3U))
            {
                fault_cycle_count = 0U;
                BMS_FaultRecoveryAttempt(&g_bms, &fault_retry_count);
            }
        }
        else
        {
            fault_cycle_count = 0U;
            fault_retry_count = 0U;
        }

        /* HEARTBEAT DE TELEMETRIA @ 1 Hz (a cada 10 ciclos de 100 ms) */
        debug_print_div++;
        if (debug_print_div >= 10U)
        {
            debug_print_div = 0U;
            BMS_MasterComm_PrintDebug(&g_master_comm, &g_bms);
        }
    }
}
