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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "global.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define speed_calc_freq motor_speed_calc_freq
#define pwm_freq motor_pwm_freq
#define METER_0_Pin GPIO_PIN_0
#define METER_0_GPIO_Port GPIOA
#define METER_1_Pin GPIO_PIN_1
#define METER_1_GPIO_Port GPIOA
#define MT6701_CSN_Pin GPIO_PIN_4
#define MT6701_CSN_GPIO_Port GPIOA
#define ENCODER_SCK_Pin GPIO_PIN_5
#define ENCODER_SCK_GPIO_Port GPIOA
#define ENCODER_MISO_Pin GPIO_PIN_6
#define ENCODER_MISO_GPIO_Port GPIOA
#define ENCODER_MOSI_Pin GPIO_PIN_7
#define ENCODER_MOSI_GPIO_Port GPIOA
#define MOTOR_BREAK_Pin GPIO_PIN_12
#define MOTOR_BREAK_GPIO_Port GPIOB
#define MOTOR_BREAK_EXTI_IRQn EXTI15_10_IRQn
#define USER_LED_Pin GPIO_PIN_15
#define USER_LED_GPIO_Port GPIOB
#define MOTOR_A_Pin GPIO_PIN_8
#define MOTOR_A_GPIO_Port GPIOA
#define MOTOR_B_Pin GPIO_PIN_9
#define MOTOR_B_GPIO_Port GPIOA
#define MOTOR_C_Pin GPIO_PIN_10
#define MOTOR_C_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
