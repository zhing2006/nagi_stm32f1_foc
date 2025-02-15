/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "nagi_foc_motor.h"
#include "kalman_filter_1d.h"
#include "low_pass_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
nagi_foc_motor_t g_motor;
kalman_filter_1d_t g_speed_filter;
low_pass_filter_t g_current_d_filter;
low_pass_filter_t g_current_q_filter;
control_context_t g_control_context = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_pwm_duty(float u, float v, float w) {
  u = CLAMP(u, 0.0f, 0.9f);
  v = CLAMP(v, 0.0f, 0.9f);
  w = CLAMP(w, 0.0f, 0.9f);
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, u * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, v * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, w * htim1.Instance->ARR);
  __enable_irq();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // Initialize motor.
  const nagi_foc_motor_config_t motor_config = {
    .set_pwm_duty_fn = set_pwm_duty,
    .delay_fn = HAL_Delay,
    .pole_pairs = MOTOR_POLE_PAIRS,
    .voltage_limit = MOTOR_VOLTAGE_LIMIT,
    .position_cycle = MOTOR_POSITION_CYCLE,
    .kp_position = 1.0f / M_PI_4, // 45 degrees output 100% current.
    .ki_position = 0.0f,
    .kd_position = 0.0f,
    .kp_speed = 1.0f / (24.0f * 2.0f * M_PI), // 24 RPM output 100% current.
    .ki_speed = 0.0005f, // Update at 4KHz. So how 0.5s to reach 100% current.
    .kd_speed = 0.0f,
    .kp_current_d = 1.2f,
    .ki_current_d = 0.01f,
    .kd_current_d = 0.0f,
    .kp_current_q = 1.2f,
    .ki_current_q = 0.01f,
    .kd_current_q = 0.0f,
  };
  nagi_foc_error_t err = nagi_foc_motor_init(&g_motor, &motor_config);
  if (err != NAGI_FOC_MOTOR_OK) {
    printf("Motor initialization failed.\n");
  }
  assert(err == NAGI_FOC_MOTOR_OK && "Motor initialization failed.");

  // Initialize filters.
  kalman_filter_1d_init(&g_speed_filter, 0.001f, 0.1f);
  low_pass_filter_init_with_alpha(&g_current_d_filter, 0.1f);
  low_pass_filter_init_with_alpha(&g_current_q_filter, 0.1f);

  // Start encoder reading.
  HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Receive_DMA(&hspi1, g_mt6701_rx_data, sizeof(g_mt6701_rx_data));
  HAL_Delay(10);
  HAL_TIM_Base_Start_IT(&htim3);

  // Start PWM.
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // Start current reading.
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  HAL_Delay(100);

  // Start motor.
  err = nagi_foc_motor_calibrate(&g_motor);
  if (err != NAGI_FOC_MOTOR_OK) {
    printf("Motor calibration failed.\n");
  }
  assert(err == NAGI_FOC_MOTOR_OK && "Motor calibration failed.");
  printf("Motor Zero Angle: %f\n", g_motor.zero_angle);

  // Initialize control context.
  // g_control_context.position = 360.0f * DEG_TO_RAD;
  // g_control_context.type = CONTROL_TYPE_POSITION;

  g_control_context.speed = 10.0f * 2.0f * M_PI;
  g_control_context.type = CONTROL_TYPE_SPEED;

  // g_control_context.current_norm_d = 0.0f;
  // g_control_context.current_norm_q = 0.2f;
  // g_control_context.type = CONTROL_TYPE_CURRENT;

  // g_control_context.speed = 10.0f * 2.0f * PI;
  // g_control_context.max_current = 0.2f;
  // g_control_context.type = CONTROL_TYPE_SPEED_CURRENT;

  // g_control_context.position = 360.0f * DEG_TO_RAD;
  // g_control_context.max_speed = 20.0f * 2.0f * PI;
  // g_control_context.max_current = 0.8f;
  // g_control_context.type = CONTROL_TYPE_POSITION_SPEED_CURRENT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100);
    printf(
      "%f,%f,%f,%f\n",
      g_motor.logical_angle,
      g_motor.speed,
      g_motor.i_d,
      g_motor.i_q
    );
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Implement __io_getchar for USART2.
int __io_getchar(void) {
  uint8_t c;
  HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY);
  return c;
}

// Implement __io_putchar for USART2.
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
