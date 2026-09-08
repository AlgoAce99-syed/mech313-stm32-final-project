/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  *
  *                   This header is CubeMX-generated. Do not modify.
  *                   The HAL include and GPIO pin definitions below are used
  *                   by every module (drv8323, encoder, current_adc, ...).
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define drv_mosi_Pin GPIO_PIN_13
#define drv_mosi_GPIO_Port GPIOC
#define motor_enable_Pin GPIO_PIN_14
#define motor_enable_GPIO_Port GPIOC
#define motor_hiz_Pin GPIO_PIN_15
#define motor_hiz_GPIO_Port GPIOC
#define motor1_Pin GPIO_PIN_0
#define motor1_GPIO_Port GPIOA
#define motor2_Pin GPIO_PIN_1
#define motor2_GPIO_Port GPIOA
#define motor3_Pin GPIO_PIN_2
#define motor3_GPIO_Port GPIOA
#define drv_cs_Pin GPIO_PIN_0
#define drv_cs_GPIO_Port GPIOB
#define drv_fault_Pin GPIO_PIN_13
#define drv_fault_GPIO_Port GPIOB
#define drv_sclk_Pin GPIO_PIN_10
#define drv_sclk_GPIO_Port GPIOC
#define drv_miso_Pin GPIO_PIN_11
#define drv_miso_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define enc_cs_Pin GPIO_PIN_2
#define enc_cs_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
