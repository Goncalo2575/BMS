/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pre_charge_Pin GPIO_PIN_0
#define pre_charge_GPIO_Port GPIOC
#define charge_Pin GPIO_PIN_1
#define charge_GPIO_Port GPIOC
#define ctr_discharge_Pin GPIO_PIN_2
#define ctr_discharge_GPIO_Port GPIOC
#define UART_TX_slaves_Pin GPIO_PIN_0
#define UART_TX_slaves_GPIO_Port GPIOA
#define UART_RX_slaves_Pin GPIO_PIN_1
#define UART_RX_slaves_GPIO_Port GPIOA
#define WAKEUP_Pin GPIO_PIN_7
#define WAKEUP_GPIO_Port GPIOA
#define BMS_charge_Pin GPIO_PIN_4
#define BMS_charge_GPIO_Port GPIOC
#define KSI_monitor_Pin GPIO_PIN_5
#define KSI_monitor_GPIO_Port GPIOC
#define KSI_monitor_EXTI_IRQn EXTI9_5_IRQn
#define status_Pin GPIO_PIN_0
#define status_GPIO_Port GPIOB
#define status_EXTI_IRQn EXTI0_IRQn
#define measure_Pin GPIO_PIN_1
#define measure_GPIO_Port GPIOB
#define BMS_relay_Pin GPIO_PIN_10
#define BMS_relay_GPIO_Port GPIOB
#define charger_signal_Pin GPIO_PIN_12
#define charger_signal_GPIO_Port GPIOB
#define charger_signal_EXTI_IRQn EXTI15_10_IRQn
#define ESDB_charger_monitor_Pin GPIO_PIN_14
#define ESDB_charger_monitor_GPIO_Port GPIOB
#define ESDB_charger_monitor_EXTI_IRQn EXTI15_10_IRQn
#define ESDB_monitor_Pin GPIO_PIN_6
#define ESDB_monitor_GPIO_Port GPIOC
#define ESDB_monitor_EXTI_IRQn EXTI9_5_IRQn
#define CAN_RxD_Pin GPIO_PIN_11
#define CAN_RxD_GPIO_Port GPIOA
#define CAN_TxD_Pin GPIO_PIN_12
#define CAN_TxD_GPIO_Port GPIOA
#define led_green_Pin GPIO_PIN_15
#define led_green_GPIO_Port GPIOA
#define led_red_Pin GPIO_PIN_11
#define led_red_GPIO_Port GPIOC
#define led_blue_Pin GPIO_PIN_12
#define led_blue_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
