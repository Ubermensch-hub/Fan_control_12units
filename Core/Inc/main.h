/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAN_Tech_IN_11_Pin GPIO_PIN_11
#define FAN_Tech_IN_11_GPIO_Port GPIOC
#define MC_Button_Pin GPIO_PIN_13
#define MC_Button_GPIO_Port GPIOC
#define FAN_Tach_IN_0_Pin GPIO_PIN_0
#define FAN_Tach_IN_0_GPIO_Port GPIOC
#define FAN_Tach_IN_1_Pin GPIO_PIN_1
#define FAN_Tach_IN_1_GPIO_Port GPIOC
#define FAN_Tach_IN_2_Pin GPIO_PIN_2
#define FAN_Tach_IN_2_GPIO_Port GPIOC
#define FAN_Tach_IN_3_Pin GPIO_PIN_3
#define FAN_Tach_IN_3_GPIO_Port GPIOC
#define MC_LED_Pin GPIO_PIN_5
#define MC_LED_GPIO_Port GPIOA
#define FAN_Tech_IN_4_Pin GPIO_PIN_4
#define FAN_Tech_IN_4_GPIO_Port GPIOC
#define FAN_Tech_IN_5_Pin GPIO_PIN_5
#define FAN_Tech_IN_5_GPIO_Port GPIOC
#define MC_UART_TX_Pin GPIO_PIN_9
#define MC_UART_TX_GPIO_Port GPIOA
#define FAN_Tech_IN_6_Pin GPIO_PIN_6
#define FAN_Tech_IN_6_GPIO_Port GPIOC
#define FAN_Tech_IN_7_Pin GPIO_PIN_7
#define FAN_Tech_IN_7_GPIO_Port GPIOC
#define MC_UART_RX_Pin GPIO_PIN_10
#define MC_UART_RX_GPIO_Port GPIOA
#define MC_I2C_SCL_Pin GPIO_PIN_11
#define MC_I2C_SCL_GPIO_Port GPIOA
#define MC_I2C_SDA_Pin GPIO_PIN_12
#define MC_I2C_SDA_GPIO_Port GPIOA
#define MC_SWDIO_Pin GPIO_PIN_13
#define MC_SWDIO_GPIO_Port GPIOA
#define MC_SWCLK_BOOT0_Pin GPIO_PIN_14
#define MC_SWCLK_BOOT0_GPIO_Port GPIOA
#define FAN_Tech_IN_8_Pin GPIO_PIN_8
#define FAN_Tech_IN_8_GPIO_Port GPIOC
#define FAN_Tech_IN_9_Pin GPIO_PIN_9
#define FAN_Tech_IN_9_GPIO_Port GPIOC
#define Buf_PWM_IN_0_Pin GPIO_PIN_0
#define Buf_PWM_IN_0_GPIO_Port GPIOD
#define Buf_PWM_IN_1_Pin GPIO_PIN_1
#define Buf_PWM_IN_1_GPIO_Port GPIOD
#define Buf_PWM_IN_2_Pin GPIO_PIN_2
#define Buf_PWM_IN_2_GPIO_Port GPIOD
#define Buf_PWM_IN_3_Pin GPIO_PIN_3
#define Buf_PWM_IN_3_GPIO_Port GPIOD
#define Buf_PWM_IN_4_Pin GPIO_PIN_4
#define Buf_PWM_IN_4_GPIO_Port GPIOD
#define Buf_PWM_IN_5_Pin GPIO_PIN_5
#define Buf_PWM_IN_5_GPIO_Port GPIOD
#define MC_PWM_OUT_0_Pin GPIO_PIN_3
#define MC_PWM_OUT_0_GPIO_Port GPIOB
#define MC_PWM_OUT_1_Pin GPIO_PIN_4
#define MC_PWM_OUT_1_GPIO_Port GPIOB
#define MC_PWM_OUT_2_Pin GPIO_PIN_5
#define MC_PWM_OUT_2_GPIO_Port GPIOB
#define MC_PWM_OUT_3_Pin GPIO_PIN_6
#define MC_PWM_OUT_3_GPIO_Port GPIOB
#define MC_PWM_OUT_4_Pin GPIO_PIN_7
#define MC_PWM_OUT_4_GPIO_Port GPIOB
#define MC_PWM_OUT_5_Pin GPIO_PIN_8
#define MC_PWM_OUT_5_GPIO_Port GPIOB
#define FAN_Tech_IN_10_Pin GPIO_PIN_10
#define FAN_Tech_IN_10_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
