/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t count; // Счетчик импульсов
  uint32_t last_time; // Время последнего импульса
  uint32_t period; // Период сигнала
} Tachometer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LedOn	 HAL_GPIO_WritePin(MC_LED_GPIO_Port, MC_LED_Pin, GPIO_PIN_SET)
#define LedOff	 HAL_GPIO_WritePin(MC_LED_GPIO_Port, MC_LED_Pin, GPIO_PIN_RESET)
Tachometer tachos[12];
volatile uint32_t min_period = UINT32_MAX;
#define OUTPUT_SIGNAL_COUNT 6

volatile uint32_t tachCounts[NUM_TACHOMETERS] = {0}; // Счетчики для тахометров
uint32_t lastTime[NUM_TACHOMETERS] = {0}; // Хранение времени для расчета частоты
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


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Обработчик прерывания EXTI для тахометров
void EXTI0_1_IRQHandler(void)
{
  uint8_t pin;
  if (EXTI->PR1 & EXTI_PR1_P0) { pin = 0; } //Обрабатываем прерывание с пина 0 и так далее
  else if (EXTI->PR1 & EXTI_PR1_P1) { pin = 1; }
  else {return; } //Обработка неизвестного прерывания


  uint32_t current_time = HAL_GetTick(); //Получаем текущее время
  tachos[pin].count++;
  tachos[pin].period = current_time - tachos[pin].last_time;
  tachos[pin].last_time = current_time;
  EXTI->PR1 |= (1 << pin); // Сброс флага прерывания
}

void EXTI2_3_IRQHandler(void)
{
  uint8_t pin;
  if (EXTI->PR1 & EXTI_PR1_P2) { pin = 2; } //Обрабатываем прерывание с пина 0 и так далее
  else if (EXTI->PR1 & EXTI_PR1_P3) { pin = 3; }
  else {return; } //Обработка неизвестного прерывания


  uint32_t current_time = HAL_GetTick(); //Получаем текущее время
  tachos[pin].count++;
  tachos[pin].period = current_time - tachos[pin].last_time;
  tachos[pin].last_time = current_time;
  EXTI->PR1 |= (1 << pin); // Сброс флага прерывания
}

void EXTI4_15_IRQHandler(void)
{
  uint8_t pin;
  if (EXTI->PR1 & EXTI_PR1_P4) { pin = 4; } //Обрабатываем прерывание с пина 0 и так далее
  else if (EXTI->PR1 & EXTI_PR1_P5) { pin = 5; }
  else if (EXTI->PR1 & EXTI_PR1_P6) { pin = 6; }
  else if (EXTI->PR1 & EXTI_PR1_P7) { pin = 7; }
  else if (EXTI->PR1 & EXTI_PR1_P8) { pin = 8; }
  else if (EXTI->PR1 & EXTI_PR1_P9) { pin = 9; }
  else if (EXTI->PR1 & EXTI_PR1_P10) { pin = 10; }
  else if (EXTI->PR1 & EXTI_PR1_P11) { pin = 11; }
  else{return; } //Обработка неизвестного прерывания


  uint32_t current_time = HAL_GetTick(); //Получаем текущее время
  tachos[pin].count++;
  tachos[pin].period = current_time - tachos[pin].last_time;
  tachos[pin].last_time = current_time;
  EXTI->PR1 |= (1 << pin); // Сброс флага прерывания
}

#define NUM_PINS 6

int count_connected_devices(void)
{
  int count = 0;
  uint32_t timeout = 1000; // примерное время для определения активного пина
  uint32_t start_time = HAL_GetTick();



  for (int i = 0; i < NUM_PINS; i++) {
    uint16_t pin = (1 << i);
    while (HAL_GPIO_ReadPin(GPIOD, pin) == GPIO_PIN_SET && HAL_GetTick() - start_time < timeout); // Ждем, пока пин станет низким
    if (HAL_GPIO_ReadPin(GPIOD, pin) == GPIO_PIN_SET) { // если пин остается высоким после таймаута - считаем его активным
      count++;
    }
    start_time = HAL_GetTick(); // обновляем время для следующего пина
  }

  return count;
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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // запуск PWM на необходимых каналах таймеров
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);

  int pulse_width_MC_OUT_PWM_0 = 25;
  int pulse_width_MC_OUT_PWM_1 = 25;
  int pulse_width_MC_OUT_PWM_2 = 60;
  int pulse_width_MC_OUT_PWM_3 = 75;
  int pulse_width_MC_OUT_PWM_4 = 59;
  int pulse_width_MC_OUT_PWM_5 = 40;

  setup();
  void set_PWM()
  {
  	pulse_width_MC_OUT_PWM_0 = 50;
  	pulse_width_MC_OUT_PWM_1 = 50;
  	pulse_width_MC_OUT_PWM_2 = 50;
  	pulse_width_MC_OUT_PWM_3 = 50;
  	pulse_width_MC_OUT_PWM_4 = 50;
  	pulse_width_MC_OUT_PWM_5 = 50;

  }
  void adjust_PWM()
  {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_width_MC_OUT_PWM_0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_width_MC_OUT_PWM_3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width_MC_OUT_PWM_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_width_MC_OUT_PWM_2);
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse_width_MC_OUT_PWM_5);
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pulse_width_MC_OUT_PWM_4);
  }
  int num_devices = count_connected_devices();
  if (num_devices == 6)
  {
	  pulse_width_MC_OUT_PWM_0 =  pulse_width_Buf_IN_PWM_0;
	  pulse_width_MC_OUT_PWM_1 =  pulse_width_Buf_IN_PWM_1;
	  pulse_width_MC_OUT_PWM_2 =  pulse_width_Buf_IN_PWM_2;
	  pulse_width_MC_OUT_PWM_3 =  pulse_width_Buf_IN_PWM_3;
	  pulse_width_MC_OUT_PWM_4 =  pulse_width_Buf_IN_PWM_4;
	  pulse_width_MC_OUT_PWM_5 =  pulse_width_Buf_IN_PWM_5;
	  printf("num of connected devices = 6");

  } else if (num_devices < 6 & num_devices >= 3)
  {
	  pulse_width_MC_OUT_PWM_0 =  pulse_width_MC_OUT_PWM_1 = (pulse_width_Buf_IN_PWM_0 > pulse_width_Buf_IN_PWM_1) ? pulse_width_Buf_IN_PWM_0 : pulse_width_Buf_IN_PWM_1;
	  pulse_width_MC_OUT_PWM_2 =  pulse_width_MC_OUT_PWM_3 = (pulse_width_Buf_IN_PWM_2 > pulse_width_Buf_IN_PWM_3) ? pulse_width_Buf_IN_PWM_2 : pulse_width_Buf_IN_PWM_3;
	  pulse_width_MC_OUT_PWM_4 =  pulse_width_MC_OUT_PWM_5 = (pulse_width_Buf_IN_PWM_4 > pulse_width_Buf_IN_PWM_5) ? pulse_width_Buf_IN_PWM_4 : pulse_width_Buf_IN_PWM_5;
	  printf("num of connected devices = 3-5");
  }else if (num_devices == 2)
  {
	  if (pulse_width_Buf_IN_PWM_0 > pulse_width_Buf_IN_PWM_1 & pulse_width_Buf_IN_PWM_0 > pulse_width_Buf_IN_PWM_2)
	  {
			  pulse_width_MC_OUT_PWM_0 =  pulse_width_MC_OUT_PWM_1 = pulse_width_MC_OUT_PWM_2 = pulse_width_Buf_IN_PWM_0;

	  }else if (pulse_width_Buf_IN_PWM_1 > pulse_width_Buf_IN_PWM_0 & pulse_width_Buf_IN_PWM_1 > pulse_width_Buf_IN_PWM_2)
	  {
			  pulse_width_MC_OUT_PWM_0 =  pulse_width_MC_OUT_PWM_1 = pulse_width_MC_OUT_PWM_2 = pulse_width_Buf_IN_PWM_1;

	  }else if (pulse_width_Buf_IN_PWM_2 > pulse_width_Buf_IN_PWM_0 & pulse_width_Buf_IN_PWM_2 > pulse_width_Buf_IN_PWM_1)
	  {
	  		  pulse_width_MC_OUT_PWM_0 =  pulse_width_MC_OUT_PWM_1 = pulse_width_MC_OUT_PWM_2 = pulse_width_Buf_IN_PWM_2;
	  }
	  if (pulse_width_Buf_IN_PWM_3 > pulse_width_Buf_IN_PWM_4 & pulse_width_Buf_IN_PWM_3 > pulse_width_Buf_IN_PWM_5)
	  {
		  	  pulse_width_MC_OUT_PWM_3 =  pulse_width_MC_OUT_PWM_4 = pulse_width_MC_OUT_PWM_5 = pulse_width_Buf_IN_PWM_3;

	  }else if (pulse_width_Buf_IN_PWM_4 > pulse_width_Buf_IN_PWM_3 & pulse_width_Buf_IN_PWM_4 > pulse_width_Buf_IN_PWM_5)
	  {
	  		  pulse_width_MC_OUT_PWM_3 =  pulse_width_MC_OUT_PWM_4 = pulse_width_MC_OUT_PWM_5 = pulse_width_Buf_IN_PWM_4;

	  }else if (pulse_width_Buf_IN_PWM_5 > pulse_width_Buf_IN_PWM_3 & pulse_width_Buf_IN_PWM_5 > pulse_width_Buf_IN_PWM_4)
	  {
	  		  pulse_width_MC_OUT_PWM_3 =  pulse_width_MC_OUT_PWM_4 = pulse_width_MC_OUT_PWM_5 = pulse_width_Buf_IN_PWM_5;
	  }
	  printf("num of connected devices = 2");
  } else if (num_devices == 1)
  {
	  pulse_width_MC_OUT_PWM_0 =  pulse_width_Buf_IN_PWM_0;
	  pulse_width_MC_OUT_PWM_1 =  pulse_width_Buf_IN_PWM_1;
	  pulse_width_MC_OUT_PWM_2 =  pulse_width_Buf_IN_PWM_2;
	  pulse_width_MC_OUT_PWM_3 =  pulse_width_Buf_IN_PWM_3;
	  pulse_width_MC_OUT_PWM_4 =  pulse_width_Buf_IN_PWM_4;
	  pulse_width_MC_OUT_PWM_5 =  pulse_width_Buf_IN_PWM_5;
	  printf("num of connected devices = 1");
  } else
  {
	  pulse_width_MC_OUT_PWM_0 = pulse_width_MC_OUT_PWM_1 = pulse_width_MC_OUT_PWM_2 = pulse_width_MC_OUT_PWM_3 = pulse_width_MC_OUT_PWM_4 = pulse_width_MC_OUT_PWM_5 = 99;
	  printf("WARNING! num of connected devices = 0");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  set_PWM();
	  adjust_PWM();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


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
