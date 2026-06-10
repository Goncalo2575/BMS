/**
 * @file    bms_master_comm.h
 * @brief   BMS → Master (VCU) Communication Module
 *
 *  Interface dedicada (USART3, distinta da USART2 do BQ79600) para envio
 *  cíclico de telemetria e recepção de Heartbeat da VCU.
 *
 *  PROTOCOLO:
 *  ┌──────────┬─────────────────────────────────────────────────┬───────┐
 *  │ BMS→VCU │ Pacote de 22 bytes a cada 100 ms (cyclic TX)    │ UART3 │
 *  │ VCU→BMS │ Heartbeat: 1 byte (0x55) a cada ≤ 500 ms       │ UART3 │
 *  └──────────┴─────────────────────────────────────────────────┴───────┘
 *
 *  WATCHDOG: Se não receber Heartbeat em MASTER_HB_TIMEOUT_MS (500 ms),
 *  o BMS transita para FAULT e abre os contactores.
 *
 *  PINOUT UART3 (ajustar conforme hardware):
 *    PB10 (USART3_TX) ──► Master RX
 *    PB11 (USART3_RX) ◄── Master TX (Heartbeat)
 *    Baud: 115200, 8N1
 */

#ifndef BMS_MASTER_COMM_H
#define BMS_MASTER_COMM_H

#include "bq796xx_bms.h"

/* =========================================================================
 * CONFIGURAÇÃO DO PROTOCOLO
 * ========================================================================= */
#define TELEMETRY_HEADER_1      0xBEU   /* Marcador de início byte 1 */
#define TELEMETRY_HEADER_2      0xEFU   /* Marcador de início byte 2 */
#define TELEMETRY_FOOTER        0xAAU   /* Marcador de fim */
#define MASTER_HEARTBEAT_BYTE   0x55U   /* Byte de heartbeat da VCU */
#define MASTER_HB_TIMEOUT_MS    500U    /* Timeout para perda de ligação */
#define MASTER_UART_BAUD        115200U

/* =========================================================================
 * ESTRUTURA DO PACOTE DE TELEMETRIA
 * =========================================================================
 * Payload mínimo exigido:
 *   BMS_STATE       — máquina de estados (para diagnóstico VCU)
 *   FAULT_FLAGS     — flags de erro (para luzes de aviso painel)
 *   PACK_VOLTAGE    — tensão total do pack (para torque derating)
 *   MIN/MAX CELL    — tensões extremas (para limites de corrente)
 *   MAX_TEMP        — temperatura máxima (para torque derating térmico)
 *   PRECHARGE_READY — autorização de fecho do contactor principal
 *   BALANCE_ACTIVE  — estado do balanceamento (informativo)
 *   INVERTER_V      — tensão no lado do inversor (pré-carga)
 *
 * Checksum: XOR acumulado de todos os bytes desde HEADER_1 até INVERTER_V. */
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];          /* TELEMETRY_HEADER_1, TELEMETRY_HEADER_2 */
    uint8_t  bms_state;          /* BMS_State_t (1 byte) */
    uint32_t fault_flags;        /* OR de todos os fault flags */
    uint32_t pack_voltage_mv;    /* Tensão total em mV — 30S max = 108 000 mV > UINT16_MAX */
    uint16_t min_cell_mv;        /* Menor tensão celular */
    uint16_t max_cell_mv;        /* Maior tensão celular */
    int8_t   max_temp_c;         /* Temperatura máxima em °C */
    uint8_t  precharge_ready;    /* 1 se tensão HV >= limiar */
    uint8_t  balance_active;     /* 1 se balanceamento em curso */
    uint16_t inverter_voltage_v; /* Tensão inversor em décimas de V (×10) */
    uint8_t  checksum;           /* XOR de todos os bytes anteriores */
    uint8_t  footer;             /* TELEMETRY_FOOTER */
} BMS_TelemetryPacket_t;
#pragma pack(pop)

/* Tamanho do pacote (verificar = sizeof(BMS_TelemetryPacket_t)) */
#define TELEMETRY_PACKET_SIZE   ((uint16_t)sizeof(BMS_TelemetryPacket_t))

/* =========================================================================
 * HANDLE DO MÓDULO DE COMUNICAÇÃO
 * ========================================================================= */
typedef struct {
    UART_HandleTypeDef  *huart;          /* Handle USART3 */
    uint32_t             last_hb_tick;   /* Timestamp do último heartbeat (ms) */
    bool                 link_ok;        /* TRUE: ligação activa */
    uint32_t             tx_count;       /* Pacotes enviados */
    uint32_t             hb_timeout_count; /* Timeouts de heartbeat */
    volatile uint8_t     hb_rx_buf[1];  /* Buffer RX para heartbeat */
} BMS_MasterComm_t;

/* =========================================================================
 * PROTÓTIPOS
 * ========================================================================= */
void         BMS_MasterComm_Init(BMS_MasterComm_t *hcomm,
                                  UART_HandleTypeDef *huart3);
void         BMS_MasterComm_Task_100ms(BMS_MasterComm_t *hcomm,
                                        BMS_Handle_t *hbms);
bool         BMS_MasterComm_IsLinkOk(BMS_MasterComm_t *hcomm);
uint8_t      BMS_MasterComm_CalcChecksum(uint8_t *data, uint16_t len);

/* Callback a chamar no HAL_UART_RxCpltCallback para USART3 */
void         BMS_MasterComm_RxCallback(BMS_MasterComm_t *hcomm,
                                        UART_HandleTypeDef *huart);

#endif /* BMS_MASTER_COMM_H */
