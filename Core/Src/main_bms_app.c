/**
 * @file    main_bms_app.c
 * @brief   BMS - Aplicação Principal e Ponto de Entrada
 *          Exemplo de integração completa no STM32F446
 *
 *  PINOUT REFERÊNCIA (hardware actual STM32F446RET6):
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │  STM32F446RET6 — UART4 Full-Duplex Asynchronous + DMA        │
 *  │                                                              │
 *  │  PA0  (UART4_TX, pino 14, AF8) ──────► BQ79600 UART_RX      │
 *  │  PA1  (UART4_RX, pino 15, AF8) ◄────── BQ79600 UART_TX      │
 *  │  PC13 (EXTI13 FALLING, Pull-Up) ◄── BQ79600 NFAULT          │
 *  │  PB12 (GPIO_OUT PP)  ──────► Contactor Gate Driver          │
 *  │  PB13 (GPIO_OUT PP)  ──────► BMS_OK (interlock VCU)         │
 *  │  PB14 (GPIO_OUT PP)  ──────► PRECHARGE_OK (interlock VCU)   │
 *  │  PB15 (GPIO_IN  PD)  ◄────── Contactor aux (weld detect)    │
 *  │  PB10 (USART3_TX) ──► VCU RX   PB11 (USART3_RX) ◄── VCU TX  │
 *  │  TIM2 (free-running µs + 100 ms tick)                       │
 *  │  DMA1 (UART4_TX)   DMA1 (UART4_RX)                          │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *  CONFIGURAÇÃO CubeMX OBRIGATÓRIA:
 *    UART4  -> Asynchronous (NÃO Half-Duplex)
 *    UART4  -> Baud Rate: 1000000, Word Length: 8, Parity: None, Stop: 1
 *    DMA    -> UART4_RX e UART4_TX em modo Normal
 *    USART3 -> Asynchronous, 115200 8N1 (telemetria + heartbeat VCU)
 *    TIM2   -> Internal Clock, free-running 1 µs/tick + interrupção 100 ms
 *    EXTI13 -> GPIO_EXTI13, Falling edge, Pull-up, Priority 0 (máxima)
 *    HAL Timebase Source -> TIM6 (não TIM2 — evitar conflito)
 *    IWDG   -> activar (ou deixar BMS_IWDG_Init configurar)
 *
 *  NOTA PCB OBRIGATÓRIA PARA 15S:
 *  Nos dois slaves BQ79616-Q1:
 *    - VC16 ligado a VC15 (curto-circuito físico na PCB)
 *    - CB16 ligado a CB15 (curto-circuito físico na PCB)
 *  Sem este requisito surgem falsas detecções Open Wire permanentes!
 *
 * @version 2.2.0
 */

#include "bq796xx_bms.h"
#include "bms_master_comm.h"
#include <stdio.h>

/* =========================================================================
 * VARIÁVEIS GLOBAIS DA APLICAÇÃO
 * ========================================================================= */

/* Handle principal do BMS */
static BMS_Handle_t g_bms;

/* Handle do módulo de comunicação BMS → Master/VCU */
BMS_MasterComm_t g_master_comm;

/* Handles HAL */
extern UART_HandleTypeDef  huart4;   /* UART4 — BQ79600 bridge (PA0/PA1) */
extern UART_HandleTypeDef  huart3;   /* USART3 — Master/VCU (telemetria + heartbeat) */
extern TIM_HandleTypeDef   htim2;    /* TIM2  — Delay µs */

/* Flag de ciclo de 100 ms (set por timer interrupt ou SysTick) */
static volatile bool g_tick_100ms = false;

/* =========================================================================
 * FUNÇÕES DE APOIO À APLICAÇÃO
 * ========================================================================= */

/**
 * @brief  Imprime sumário do estado do BMS via UART de debug
 *         Na prática usar ITM printf ou UART3/LPUART para debug
 */
static void BMS_PrintStatus(BMS_Handle_t *hbms)
{
    /* Na plataforma real, redirigir para UART de debug ou ITM */
    /* Exemplo com printf (requer retargeting de _write) */
    printf("\r\n=== BMS STATUS ===\r\n");
    printf("State     : %s\r\n", BMS_GetStateString(hbms->state));
    printf("Fault     : %s (0x%08lX)\r\n",
           BMS_GetFaultString(hbms->fault_flags), hbms->fault_flags);
    printf("Pack V    : %lu mV\r\n", hbms->pack_voltage_mv);
    printf("Min Cell  : %u mV\r\n", hbms->min_cell_mv);
    printf("Max Cell  : %u mV\r\n", hbms->max_cell_mv);
    printf("Delta     : %u mV\r\n", hbms->delta_cell_mv);
    printf("Max Temp  : %d degC\r\n", hbms->max_temp_c);
    printf("SoC       : %u%%\r\n", (unsigned)hbms->soc_percent);
    printf("HV Bus    : %lu mV (precharge %s)\r\n",
           hbms->inverter_voltage_mv,
           hbms->precharge_ready ? "READY" : "not ready");
    printf("Ring OK   : %s\r\n", hbms->ring_intact ? "YES" : "NO");
    printf("Contactor : %s\r\n", hbms->contactor_closed ? "CLOSED" : "OPEN");

    for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
    {
        printf("\r\n  Slave %u (Addr 0x%02X):\r\n", s + 1U,
               hbms->slave[s].address);
        for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
        {
            printf("    Cell%02u: %4u mV%s%s\r\n",
                   c + 1U,
                   hbms->slave[s].cell_voltage_mv[c],
                   hbms->slave[s].ov_cell[c] ? " [OV]" : "",
                   hbms->slave[s].uv_cell[c] ? " [UV]" : "");
        }
        printf("    Temp  : %d/%d/%d degC\r\n",
               hbms->slave[s].temperatures_c[0],
               hbms->slave[s].temperatures_c[1],
               hbms->slave[s].temperatures_c[2]);
    }
    printf("Comm Errs : %lu | CRC Errs: %lu\r\n",
           hbms->comm_error_count, hbms->crc_error_count);
    printf("Ring Recs : %lu\r\n", hbms->ring_recovery_count);
    printf("==================\r\n");
}

/**
 * @brief  Lógica de controlo do contactor baseada no estado do BMS
 *
 *  DEFESA EM PROFUNDIDADE (defense-in-depth):
 *  A barreira primária contra religamento indevido encontra-se em
 *  BMS_ContactorClose(). Esta função acrescenta uma camada exterior:
 *  não tenta sequer chamar Close() enquanto existir um evento NFAULT
 *  por processar, evitando chamadas de função desnecessárias e tornando
 *  a intenção de segurança explícita ao nível do controlador.
 */
static void BMS_ContactorControl(BMS_Handle_t *hbms)
{
    /* Camada exterior: não actuar sobre o contactor enquanto existir
     * um evento de hardware não processado. Qualquer estado de tensão
     * ou temperatura lido antes do BMS_ProcessFaults() escoar o evento
     * pode estar desactualizado face à condição real do pack. */
    if (__atomic_load_n(&hbms->nfault_pending, __ATOMIC_SEQ_CST) != 0U)
    {
        return;
    }

    if (hbms->state == BMS_STATE_MONITORING &&
        hbms->fault_flags == BMS_FAULT_NONE)
    {
        if (!hbms->contactor_closed)
        {
            BMS_ContactorClose(hbms);   /* Barreira interna reforça aqui */
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
 *
 *  CONTEXTO CRÍTICO (hardware latching):
 *  Os registos FAULT_* do BQ79616 são de retenção de estado (latch).
 *  Quando um comparador deteta OV/UV/OW, conduz NFAULT a LOW e mantém-no
 *  nesse estado indefinidamente até receber um comando de limpeza explícito.
 *
 *  CONSEQUÊNCIA SEM ESTA LIMPEZA:
 *  Se o software limpar apenas as suas variáveis internas (fault_flags) sem
 *  enviar o comando UART, o pino NFAULT permanece fisicamente preso em LOW.
 *  Como a EXTI13 está configurada para flanco descendente (Falling edge),
 *  um novo evento catastrófico não consegue gerar novo flanco — a ISR de
 *  segurança sub-24 µs fica cega para todas as falhas subsequentes e o
 *  sistema recai numa dependência insegura da varredura de 100 ms.
 *
 *  SEQUÊNCIA DE LIMPEZA:
 *  1. Broadcast write 0xFF -> FAULT_SUMMARY  (limpa registo sumário)
 *  2. Broadcast write 0xFF -> FAULT_OV       (liberta comparador OV)
 *  3. Broadcast write 0xFF -> FAULT_UV       (liberta comparador UV)
 *  4. Broadcast write 0xFF -> FAULT_VCOW     (liberta detecção Open Wire)
 *  5. Broadcast write 0xFF -> FAULT_COMM     (liberta flags de comm)
 *  6. Single write 0xFF   -> Bridge FAULT_COMM* (liberta bridge)
 *  Após estes writes, o BQ79616 reavalia os comparadores. Se as condições
 *  físicas se resolveram, NFAULT volta a HIGH — pronto para novo flanco.
 *
 * @param  hbms     Handle do BMS
 * @return BMS_OK se limpeza enviada com sucesso
 */
static BMS_Status_t BMS_ClearHardwareFaultLatches(BMS_Handle_t *hbms)
{
    uint8_t      clear_val = FAULT_CLEAR_VAL;  /* 0xFF */
    BMS_Status_t status;

    /* Limpar latches dos slaves via Broadcast
     * (propaga-se através da barreira galvânica para BQ79616) */
    status = BMS_WriteBroadcast(hbms, REG_FAULT_RST1, &clear_val, 1U);
    if (status != BMS_OK) { return status; }

    status = BMS_WriteBroadcast(hbms, REG_FAULT_RST2, &clear_val, 1U);
    if (status != BMS_OK) { return status; }

    /* BUG-4 CORRIGIDO: Limpeza mandatória dos latches da Bridge BQ79600-Q1.
     * Comandos Broadcast NÃO chegam à bridge — param nos slaves.
     * Se a falha for nativa da interface UART da bridge (FAULT_COMM1/2),
     * os latches ficam retidos e NFAULT permanece LOW indefinidamente.
     * A bridge requer escritas Single Device dedicadas (endereço 0x00). */
    (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                REG_FAULT_RST1, &clear_val, 1U);
    (void)BMS_WriteSingleDevice(hbms, BMS_ADDR_BRIDGE,
                                REG_FAULT_RST2, &clear_val, 1U);

    return BMS_OK;
}

/**
 * @brief  Tentativa de recuperação e reset de faults após shutdown
 *
 *  CORRECÇÕES APLICADAS:
 *
 *  [A] Limpeza física dos latches de hardware ANTES de transitar para MONITORING.
 *      Sem BMS_ClearHardwareFaultLatches(), o pino NFAULT permanece LOW após
 *      a recuperação, impedindo novos flancos descendentes e cegando a ISR.
 *
 *  [B] Open Wire adicionado às condições NÃO recuperáveis.
 *      Cabo partido é uma falha física irreparável por software. Sem esta
 *      condição, o sistema transitava para MONITORING com BMS_FAULT_OPEN_WIRE
 *      activa, ficando preso num estado paradoxal: state=MONITORING mas
 *      BMS_ContactorControl recusando fechar por fault_flags != 0.
 *
 *  [C] Limpeza das fault_flags individuais de cada slave em paralelo com
 *      a flag global. Sem este passo, hbms->slave[s].fault_flags acumula
 *      flags perpétuas inconsistentes com o estado holístico do sistema.
 *
 * @param  hbms             Handle do BMS
 * @param  retry_counter    Ponteiro para contador de tentativas
 */
static void BMS_FaultRecoveryAttempt(BMS_Handle_t *hbms,
                                      uint32_t *retry_counter)
{
    (*retry_counter)++;

    if (*retry_counter > 3U)
    {
        /* BUG-2 CORRIGIDO: adormece fisicamente o barramento em vez de
         * apenas alterar a variável (que o polling revertia para FAULT).
         * BMS_EnterSleep: para balanceamento, abre contactor, envia pulso
         * UART 9 ms → state=SLEEP. O guard de SLEEP em BMS_Task_100ms
         * quebra o ciclo infinito FAULT→SHUTDOWN→FAULT. */
        BMS_EnterSleep(hbms);
        return;
    }

    /* Leitura actualizada das condições físicas */
    BMS_ReadAllCellVoltages(hbms);
    BMS_ReadAllTemperatures(hbms);

    bool recoverable = true;

    /* Tensão OV ainda acima da janela de histérése */
    if ((hbms->fault_flags & BMS_FAULT_OV) &&
        (hbms->max_cell_mv > (CELL_OV_MV - 50U)))
    {
        recoverable = false;
    }
    /* Tensão UV ainda abaixo da janela de histérése */
    if ((hbms->fault_flags & BMS_FAULT_UV) &&
        (hbms->min_cell_mv < (CELL_UV_MV + 50U)))
    {
        recoverable = false;
    }
    /* Temperatura ainda acima do limiar seguro */
    if ((hbms->fault_flags & BMS_FAULT_OT) &&
        (hbms->max_temp_c > (int16_t)(CELL_TEMP_MAX_C - 5)))
    {
        recoverable = false;
    }
    /* [B] Open Wire: falha física — não recuperável por software */
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
        /* [A] Limpar latches de hardware ANTES de transitar para MONITORING.
         * Se esta escrita falhar, não prosseguimos — NFAULT ficaria preso. */
        if (BMS_ClearHardwareFaultLatches(hbms) != BMS_OK)
        {
            /* Falha de comunicação durante limpeza — não é seguro continuar */
            hbms->fault_flags |= BMS_FAULT_COMM;
            return;
        }

        /* [C] Limpar flags individuais de cada slave */
        for (uint8_t s = 0U; s < BMS_NUM_SLAVES; s++)
        {
            hbms->slave[s].fault_flags = 0U;
            for (uint8_t c = 0U; c < BMS_CELLS_PER_SLAVE; c++)
            {
                hbms->slave[s].ov_cell[c] = false;
                hbms->slave[s].uv_cell[c] = false;
            }
        }

        /* Limpar flags globais recuperáveis */
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

/**
 * @brief  Função principal do BMS
 *         No contexto STM32/CubeMX, este código vai para dentro do main()
 *         após a inicialização do HAL e dos periféricos gerada pelo CubeMX.
 *
 *  Estrutura típica no main.c do CubeMX:
 *
 *  int main(void) {
 *      HAL_Init();
 *      SystemClock_Config();
 *      MX_GPIO_Init();
 *      MX_UART4_Init();
 *      MX_TIM2_Init();
 *      BMS_Main();   // <-- Chamar aqui
 *  }
 */
void BMS_Main(void)
{
    BMS_Status_t status;
    /* BUG-04 CORRIGIDO: dois contadores separados com responsabilidades distintas.
     * fault_cycle_count: número de ciclos de 100ms desde a entrada em FAULT
     *   → controla o TEMPO entre tentativas de recovery (5 s = 50 ciclos)
     * fault_retry_count: número de tentativas de recovery efectivamente feitas
     *   → controla o LIMITE de tentativas (máx 3); passado ao BMS_FaultRecoveryAttempt
     *
     * Problema da versão anterior: um único contador servia ambos.
     * Após BMS_FaultRecoveryAttempt incrementar o contador para 3 (limite atingido)
     * e fazer state=SHUTDOWN, o módulo 50 voltava a disparar aos ciclos 50, 100...
     * e chamava BMS_FaultRecoveryAttempt de novo, sobrepondo o estado SHUTDOWN. */
    uint32_t     fault_cycle_count  = 0U;   /* ciclos em FAULT desde a última transição */
    uint32_t     fault_retry_count  = 0U;   /* tentativas de recovery efectuadas */
    uint32_t     debug_print_div    = 0U;

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
        while (1U)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            BMS_DelayMs(200U);
        }
    }

    printf("[BMS] Initialization OK. State: %s\r\n",
           BMS_GetStateString(g_bms.state));
    printf("[BMS] Ring intact: %s\r\n", g_bms.ring_intact ? "YES" : "NO");

    /* ------------------------------------------------------------------
     * FASE 1b: Inicializar comunicação com a VCU (USART3)
     * ------------------------------------------------------------------ */
    BMS_MasterComm_Init(&g_master_comm, &huart3);
    printf("[BMS] Master comm init OK (USART3, watchdog=%u ms)\r\n",
           MASTER_HB_TIMEOUT_MS);

    /* ------------------------------------------------------------------
     * FASE 1c: IWDG Watchdog (ASIL-D)
     * A partir deste ponto, BMS_IWDG_Refresh() DEVE ser chamado em cada
     * iteração do super-loop. Se omitido → reset MCU em ~500 ms.
     * ------------------------------------------------------------------ */
    BMS_IWDG_Init();
    printf("[BMS] IWDG init OK (timeout ~500 ms)\r\n");

    /* ------------------------------------------------------------------
     * FASE 1d: Power-On Self Test (POST)
     * Verifica CRC, comunicação, ADC e NFAULT antes de permitir operação.
     * Se POST falhar, o sistema NÃO fecha o contactor.
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
     * FASE 2: Fechar contactor se POST e condições iniciais OK
     * ------------------------------------------------------------------ */
    BMS_DelayMs(100U);
    BMS_ReadAllCellVoltages(&g_bms);
    BMS_ReadAllTemperatures(&g_bms);

    if ((g_bms.fault_flags == BMS_FAULT_NONE) && (g_bms.post_passed))
    {
        printf("[BMS] Conditions OK. Closing contactor...\r\n");
        BMS_ContactorClose(&g_bms);
    }
    else
    {
        printf("[BMS] Cannot close contactor: %s\r\n",
               BMS_GetFaultString(g_bms.fault_flags));
    }

    /* ------------------------------------------------------------------
     * FASE 3: Super-loop de operação
     * ------------------------------------------------------------------ */
    printf("[BMS] Entering monitoring loop.\r\n");

    while (1U)
    {
        /* ----------------------------------------------------------
         * VERIFICAÇÃO NFAULT — FORA DO TICK DE 100 ms
         * Um evento de hardware deve ser processado imediatamente,
         * não condicionado ao período de amostragem.
         * Latência máxima de detecção aqui: 1 ms (delay do idle poll).
         * ----------------------------------------------------------  */
        if (__atomic_load_n(&g_bms.nfault_pending, __ATOMIC_SEQ_CST) != 0U)
        {
            BMS_ProcessFaults(&g_bms);
            /* Contactor já foi aberto na ISR. Garante sincronia do estado. */
        }

        /* Aguarda tick de 100 ms (set pelo TIM2 Period Elapsed callback) */
        if (!g_tick_100ms)
        {
            BMS_DelayMs(1U);
            continue;
        }
        g_tick_100ms = false;

        /* ----------------------------------------------------------
         * TAREFA PRINCIPAL DO BMS @ 100 ms
         * ---------------------------------------------------------- */
        status = BMS_Task_100ms(&g_bms);

        /* ----------------------------------------------------------
         * TAREFA DE COMUNICAÇÃO COM A VCU @ 100 ms
         * Envia pacote de telemetria e verifica watchdog do heartbeat.
         * Deve ser chamada APÓS BMS_Task_100ms para que os dados
         * enviados reflictam o estado mais recente do ciclo actual.
         * ---------------------------------------------------------- */
        BMS_MasterComm_Task_100ms(&g_master_comm, &g_bms);

        /* ----------------------------------------------------------
         * GESTÃO DO CONTACTOR
         * A barreira nfault_pending em BMS_ContactorControl e em
         * BMS_ContactorClose garante que não há fecho indevido mesmo
         * que um evento ocorra entre BMS_Task_100ms e este ponto.
         * ---------------------------------------------------------- */
        BMS_ContactorControl(&g_bms);

        /* ----------------------------------------------------------
         * GESTÃO DE RECOVERY EM ESTADO FAULT
         * ---------------------------------------------------------- */
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
            /* Fora de FAULT: reiniciar ambos os contadores */
            fault_cycle_count = 0U;
            fault_retry_count = 0U;
        }

        /* ----------------------------------------------------------
         * DEBUG PRINT @ 1 Hz (a cada 10 ciclos de 100 ms)
         * ---------------------------------------------------------- */
        debug_print_div++;
        if (debug_print_div >= 10U)
        {
            debug_print_div = 0U;
            BMS_PrintStatus(&g_bms);
        }

        /* ----------------------------------------------------------
         * SINALIZAÇÃO LED STATUS
         * ---------------------------------------------------------- */
        switch (g_bms.state)
        {
            case BMS_STATE_MONITORING:
            case BMS_STATE_BALANCING:
                if (debug_print_div == 0U)
                {
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                }
                break;

            case BMS_STATE_FAULT:
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                break;

            case BMS_STATE_RING_RECOVERY:
                /* BUG-07 CORRIGIDO: BMS_DelayMs(50) removido.
                 * O delay bloqueante de 50 ms no superloop atrasava o
                 * processamento de faults e o watchdog da VCU em cada
                 * ciclo durante o ring recovery. Substituído por toggle
                 * simples — o período de 100 ms já providencia visibilidade. */
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                break;

            default:
                break;
        }

        /* ----------------------------------------------------------
         * DETECÇÃO DE SOLDADURA DO CONTACTOR (não-bloqueante)
         * Executa em TODOS os estados, MAS apenas quando o contactor
         * deveria estar aberto. Com contactor fechado (condução normal),
         * o pin auxiliar está naturalmente HIGH → seria falso positivo.
         * ---------------------------------------------------------- */
        if (!g_bms.contactor_closed)
        {
            if (BMS_CheckContactorWeld())
            {
                g_bms.contactor_weld_detected = true;
                g_bms.fault_flags |= BMS_FAULT_CONTACTOR;
                g_bms.state = BMS_STATE_FAULT;
            }
        }

        /* ----------------------------------------------------------
         * IWDG REFRESH — OBRIGATÓRIO EM CADA ITERAÇÃO
         * Se esta linha for removida ou se o CPU pendurar antes de a
         * atingir, o MCU reseta automaticamente em ~500 ms.
         * ---------------------------------------------------------- */
        BMS_IWDG_Refresh();
    }
}

/* =========================================================================
 * CALLBACK DO TIMER HARDWARE (100 ms tick)
 * ========================================================================= */

/**
 * @brief  Callback invocado pelo HAL quando o TIM2 transborda (Period Elapsed)
 *
 *  CONFIGURAÇÃO CubeMX OBRIGATÓRIA para TIM2 @ 100 ms exactos:
 *    Clock source : Internal Clock
 *    Prescaler    : (SystemCoreClock / 10000) - 1   -> tick = 100 µs
 *    Counter Period (ARR) : 999                      -> 999 * 100 µs = 99,9 ms
 *                                                       ≈ 100 ms
 *
 *  Exemplo para STM32F446 a 180 MHz:
 *    Prescaler = 17999  (180 000 000 / 10 000 - 1)
 *    ARR       = 999
 *    Verificação: 180e6 / (17999+1) / (999+1) = 10 Hz -> T = 100 ms ✓
 *
 *  PORQUÊ NÃO usar HAL_GetTick() aqui:
 *    Esta função é chamada precisamente quando o hardware de TIM2 transborda.
 *    Adicionar uma subtracção de SysTick introduz jitter (a ISR pode chegar
 *    alguns SysTick depois do overflow real) e ignora o periférico dedicado.
 *    O corpo desta função deve ser mínimo e determinístico.
 *
 *  NOTA: HAL usa TIM1 ou TIM8 para HAL_GetTick() por defeito. Configurar
 *        o TIM base do HAL para um timer diferente do TIM2 no CubeMX
 *        (Project -> Advanced Settings -> HAL Timebase Source).
 */
void BMS_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Guarda para garantir que só TIM2 dispara o tick do BMS.
     * Outros timers (ex: TIM1 usado pelo HAL internamente) não interferem. */
    if (htim->Instance == TIM2)
    {
        g_tick_100ms = true;
    }
}
