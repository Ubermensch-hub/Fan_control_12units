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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LedOn	 HAL_GPIO_WritePin(MC_LED_GPIO_Port, MC_LED_Pin, GPIO_PIN_SET)
#define LedOff	 HAL_GPIO_WritePin(MC_LED_GPIO_Port, MC_LED_Pin, GPIO_PIN_RESET)
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


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
	USART2->TDR = ch;
	while ((USART2->ISR & USART_ISR_TC) == 0)
		;
	return ch;
}
int __io_getchar(void)
{
	while ((USART2->ISR & USART_ISR_RXNE_RXFNE) == 0)
		;
	return USART2->RDR;
}
uint32_t t0, p0;
uint16_t portD;
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) // Обработчик прерывания нарастающего фронта
{
	uint16_t port;
	uint16_t t = TIM2->CNT & 0x0000FFFF; // Получение текущего значения счётчика TIM2
	port = GPIOD->IDR; // Чтение порта GPIOD
	if ((port ^ portD) != 0) // Проверка на изменения в статусе порта
	{
		portD = port; // Обновление сохраненной информации о состоянии порта
		GPIOB->BSRR = GPIO_Pin; // Установка соответствующего бита в GPIOB (1)
		printf("portD+ %u\r\n", t); // логирование
	}
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) // Обработчик прерывания спадающего фронта
{
	uint16_t t = TIM2->CNT & 0x0000FFFF;// Получение текущего значения счётчика TIM2
	uint16_t port;
	port = GPIOD->IDR; //Чтение порта GPIOD
	if ((port ^ portD) != 0)// Проверка на изменения в статусе порта
	{
		portD = port;
		GPIOB->BRR = GPIO_Pin;// Установка соответствующего бита в GPIOB (0)
		printf("portD- %u\r\n", t);// логирование
	}
}
uint16_t LedMask;
void HAL_IncTick(void)
{
	uwTick += (uint32_t) uwTickFreq;
//	if ((uwTick % 64) == 0)
	{
		if (LedMask & 1)
		{
			LedOn;
			LedMask >>= 1;
			LedMask |= (1 << 15);
		}
		else
		{
			LedOff;
			LedMask >>= 1;
		}
	}
}
#define StopFan	60000
uint16_t timeB6_11[12] =
{ 40000, 40000, 40000, 40000, 40000, 40000, 40000, 40000, 40000, 40000, 40000,
		40000 };
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) // Обработчик таймера
{
	int isr;
	uint16_t period;
	if (htim->Instance == TIM1)
	{
		isr = TIM1->SR; // получение флага прерывания TIM1
		TIM1->SR = 0; // Сброс флагов прерываний
		printf("Timer1 %u %u\r\n", timeB6_11[0], timeB6_11[1]);
		printf("Timer2 %u %u\r\n", timeB6_11[10], timeB6_11[11]);
		if (isr & (1 << 1))
		{
			if (timeB6_11[0] > timeB6_11[1])
				period = timeB6_11[0];
			else
				period = timeB6_11[1];
			if (period < StopFan)
			{
				TIM1->CCR1 += period;
				GPIOB->ODR ^= (1 << 6);
			}
		}
		if (isr & (1 << 2))
		{
			if (timeB6_11[2] > timeB6_11[3])
				period = timeB6_11[2];
			else
				period = timeB6_11[3];
			if (period < StopFan)
			{
				TIM1->CCR2 += period;
				GPIOB->ODR ^= (1 << 7);
			}
		}
		if (isr & (1 << 3))
		{
			if (timeB6_11[4] > timeB6_11[5])
				period = timeB6_11[4];
			else
				period = timeB6_11[5];
			if (period < StopFan)
			{
				TIM1->CCR3 += period;
				GPIOB->ODR ^= (1 << 8);
			}
		}
		if (isr & (1 << 4))
		{
			if (timeB6_11[6] > timeB6_11[7])
				period = timeB6_11[6];
			else
				period = timeB6_11[7];
			if (period < StopFan)
			{
				TIM1->CCR4 += period;
				GPIOB->ODR ^= (1 << 9);
			}
		}
	}
	else if (htim->Instance == TIM2)
	{
		isr = TIM2->SR;
		TIM2->SR = 0;
		printf("Timer2 CMP %u\r\n", (uint16_t) (TIM2->CNT & 0x0000FFFF));
		if (isr & (1 << 1))
		{
			if (timeB6_11[8] > timeB6_11[9])
				period = timeB6_11[8];
			else
				period = timeB6_11[9];
			if (period < StopFan)
			{
				TIM2->CCR1 += period;
				GPIOB->ODR ^= (1 << 10);
			}
		}
		if (isr & (1 << 2))
		{
			if (timeB6_11[10] > timeB6_11[11])
				period = timeB6_11[10];
			else
				period = timeB6_11[11];
			if (period < StopFan)
			{
				TIM2->CCR2 += period;
				GPIOB->ODR ^= (1 << 11);
			}
		}
	}
}
uint16_t portC;
uint16_t timeCstart[12];
void checkTah(uint16_t ch, uint16_t ticks, uint16_t change)
{
	uint16_t v;
	v = ticks - timeCstart[ch]; // Расчёт времени, прошедшего с момента последнего изменения
	if ((change & (1 << ch)) || (v >= StopFan)) //Если произошли изменения или время превышает порог
	{
		timeCstart[ch] = ticks;// Обновление времени старта
		timeB6_11[ch] = v; // Сохранение времени
	}
}
void Tahometr(void)
{
	uint16_t port, cport, ticks;
	port = GPIOC->IDR; // Чтение состояния порта GPIOC
	cport = port ^ portC; // Определение изменения состояния
	portC = port; // Обновление сохранённого состояния
	/***** FAN 1...7 */
	ticks = TIM1->CNT & 0x0000FFFF;
	/** check FAN 1 & 2 */
	checkTah(0, ticks, cport);
	checkTah(1, ticks, cport);
	/** check FAN 3 & 4 */
	checkTah(2, ticks, cport);
	checkTah(3, ticks, cport);
	/** check FAN 5 & 6 */
	checkTah(4, ticks, cport);
	checkTah(5, ticks, cport);
	/** check FAN 7 & 8 */
	checkTah(6, ticks, cport);
	checkTah(7, ticks, cport);
	/***** FAN 9...12 */
	ticks = TIM2->CNT & 0x0000FFFF;
	/** check FAN 9 & 10 */
	checkTah(8, ticks, cport);
	checkTah(9, ticks, cport);
	/** check FAN 11 & 12 */
	checkTah(10, ticks, cport);
	checkTah(11, ticks, cport);
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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // запуск PWM на необходимых каналах таймеров
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  Tahometr();
    /* USER CODE BEGIN 3 */

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
