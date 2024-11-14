/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
int MC_PWM_OUT_0_PW = 75;
int MC_PWM_OUT_1_PW = 75;
int MC_PWM_OUT_2_PW = 75;
int MC_PWM_OUT_3_PW = 150;
int MC_PWM_OUT_4_PW = 75;
int MC_PWM_OUT_5_PW = 75;

#include <stdbool.h>
#include<stdio.h>

typedef struct
    {
	uint32_t lastCapture; // Последнее значение счётчика таймера при захвате
	uint32_t captureCount; // Счётчик захватов
	float pulse_width; // Коэффициент заполнения
	uint8_t signalPresent; // Признак наличия сигнала
	bool is_on;
    } PWM_INPUT;

PWM_INPUT BUF_PWM_IN_5, BUF_PWM_IN_4, BUF_PWM_IN_3, BUF_PWM_IN_2, BUF_PWM_IN_1, BUF_PWM_IN_0; // @suppress("Multiple variable declaration")


/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 880;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MC_PWM_OUT_3_PW;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = MC_PWM_OUT_1_PW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = MC_PWM_OUT_0_PW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = MC_PWM_OUT_2_PW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 880;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MC_PWM_OUT_5_PW;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = MC_PWM_OUT_4_PW;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 880;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
/* TIM16 init function */
void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */

    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = Buf_PWM_IN_5_Pin|Buf_PWM_IN_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */

    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = Buf_PWM_IN_3_Pin|Buf_PWM_IN_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Buf_PWM_IN_1_Pin|Buf_PWM_IN_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspInit 0 */

  /* USER CODE END TIM16_MspInit 0 */

    /* TIM16 clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();
  /* USER CODE BEGIN TIM16_MspInit 1 */

  /* USER CODE END TIM16_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA11 [PA9]     ------> TIM1_CH4
    PB3     ------> TIM1_CH2
    PB6     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = MC_PWM_OUT_3_Pin|MC_PWM_OUT_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MC_PWM_OUT_1_Pin|MC_PWM_OUT_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = MC_PWM_OUT_5_Pin|MC_PWM_OUT_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4
    */
    HAL_GPIO_DeInit(GPIOA, Buf_PWM_IN_5_Pin|Buf_PWM_IN_4_Pin|MC_PWM_OUT_5_Pin|MC_PWM_OUT_4_Pin);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4
    */
    HAL_GPIO_DeInit(GPIOA, Buf_PWM_IN_3_Pin|Buf_PWM_IN_2_Pin);

    HAL_GPIO_DeInit(GPIOB, Buf_PWM_IN_1_Pin|Buf_PWM_IN_0_Pin);

  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspDeInit 0 */

  /* USER CODE END TIM16_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM16_CLK_DISABLE();
  /* USER CODE BEGIN TIM16_MspDeInit 1 */

  /* USER CODE END TIM16_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    {
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_1);
	BUF_PWM_IN_5.signalPresent = 1; // Сигнал присутствует
	BUF_PWM_IN_5.is_on = true;

	if (BUF_PWM_IN_5.lastCapture != 0)
	    {
	    uint32_t period = currentCapture - BUF_PWM_IN_5.lastCapture;

	    //Обработка переполнения
	    if (period < 0)
		{
		period += 0xFFFFFFFF; //Добавляем максимальное значение 32-битного числа
		}

	    uint32_t highTime = currentCapture - BUF_PWM_IN_5.lastCapture; // Высокий уровень импульса
	    BUF_PWM_IN_5.pulse_width = (float) highTime / period; //Вычисляем коэффициент заполнения
	    }
	BUF_PWM_IN_5.lastCapture = currentCapture;
	}
    if (htim->Instance == TIM2
	    && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
	uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_2);
	BUF_PWM_IN_4.signalPresent = 1; // Сигнал присутствует
	BUF_PWM_IN_4.is_on = true;

	if (BUF_PWM_IN_4.lastCapture != 0)
	    {
	    uint32_t period = currentCapture - BUF_PWM_IN_4.lastCapture;

	    //Обработка переполнения
	    if (period < 0)
		{
		period += 0xFFFFFFFF; //Добавляем максимальное значение 32-битного числа
		}

	    uint32_t highTime = currentCapture - BUF_PWM_IN_4.lastCapture; // Высокий уровень импульса
	    BUF_PWM_IN_4.pulse_width = (float) highTime / period; //Вычисляем коэффициент заполнения
	    }
	BUF_PWM_IN_4.lastCapture = currentCapture;
	}
    if (htim->Instance == TIM3
	    && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_1);
	BUF_PWM_IN_3.signalPresent = 1; // Сигнал присутствует
	BUF_PWM_IN_3.is_on = true;

	if (BUF_PWM_IN_3.lastCapture != 0)
	    {
	    uint32_t period = currentCapture - BUF_PWM_IN_3.lastCapture;

	    //Обработка переполнения
	    if (period < 0)
		{
		period += 0xFFFFFFFF; //Добавляем максимальное значение 32-битного числа
		}

	    uint32_t highTime = currentCapture - BUF_PWM_IN_3.lastCapture; // Высокий уровень импульса
	    BUF_PWM_IN_3.pulse_width = (float) highTime / period; //Вычисляем коэффициент заполнения
	    }
	BUF_PWM_IN_3.lastCapture = currentCapture;
	}
    if (htim->Instance == TIM3
	    && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
	uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_2);
	BUF_PWM_IN_2.signalPresent = 1; // Сигнал присутствует
	BUF_PWM_IN_2.is_on = true;

	if (BUF_PWM_IN_2.lastCapture != 0)
	    {
	    uint32_t period = currentCapture - BUF_PWM_IN_2.lastCapture;

	    //Обработка переполнения
	    if (period < 0)
		{
		period += 0xFFFFFFFF; //Добавляем максимальное значение 32-битного числа
		}

	    uint32_t highTime = currentCapture - BUF_PWM_IN_2.lastCapture; // Высокий уровень импульса
	    BUF_PWM_IN_2.pulse_width = (float) highTime / period; //Вычисляем коэффициент заполнения
	    }
	BUF_PWM_IN_2.lastCapture = currentCapture;
	}
    if (htim->Instance == TIM3
	    && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
	uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_3);
	BUF_PWM_IN_1.signalPresent = 1; // Сигнал присутствует
	BUF_PWM_IN_1.is_on = true;

	if (BUF_PWM_IN_1.lastCapture != 0)
	    {
	    uint32_t period = currentCapture - BUF_PWM_IN_1.lastCapture;

	    //Обработка переполнения
	    if (period < 0)
		{
		period += 0xFFFFFFFF; //Добавляем максимальное значение 32-битного числа
		}

	    uint32_t highTime = currentCapture - BUF_PWM_IN_1.lastCapture; // Высокий уровень импульса
	    BUF_PWM_IN_1.pulse_width = (float) highTime / period; //Вычисляем коэффициент заполнения
	    }
	BUF_PWM_IN_1.lastCapture = currentCapture;
	}
    if (htim->Instance == TIM3
	    && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
	uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_4);
	BUF_PWM_IN_0.signalPresent = 1; // Сигнал присутствует
	BUF_PWM_IN_0.is_on = true;

	if (BUF_PWM_IN_0.lastCapture != 0)
	    {
	    uint32_t period = currentCapture - BUF_PWM_IN_0.lastCapture;

	    //Обработка переполнения
	    if (period < 0)
		{
		period += 0xFFFFFFFF; //Добавляем максимальное значение 32-битного числа
		}

	    uint32_t highTime = currentCapture - BUF_PWM_IN_0.lastCapture; // Высокий уровень импульса
	    BUF_PWM_IN_0.pulse_width = (float) highTime / period; //Вычисляем коэффициент заполнения
	    }
	BUF_PWM_IN_0.lastCapture = currentCapture;
	}
    }



void adjust_PWM()
    {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MC_PWM_OUT_3_PW);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MC_PWM_OUT_1_PW);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MC_PWM_OUT_0_PW);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MC_PWM_OUT_2_PW);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MC_PWM_OUT_5_PW);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MC_PWM_OUT_4_PW);
    }

int count_connected_devices()
    {
    int count = 0;
    if (BUF_PWM_IN_5.is_on == true)
	count++;
    if (BUF_PWM_IN_4.is_on == true)
	count++;
    if (BUF_PWM_IN_3.is_on == true)
	count++;
    if (BUF_PWM_IN_2.is_on == true)
	count++;
    if (BUF_PWM_IN_1.is_on == true)
	count++;
    if (BUF_PWM_IN_0.is_on == true)
	count++;
    return count;
    }



void set_PWM(int num_devices)
    {

    if (num_devices == 6)
	{
	MC_PWM_OUT_0_PW = BUF_PWM_IN_0.pulse_width;
	MC_PWM_OUT_1_PW = BUF_PWM_IN_1.pulse_width;
	MC_PWM_OUT_2_PW = BUF_PWM_IN_2.pulse_width;
	MC_PWM_OUT_3_PW = BUF_PWM_IN_3.pulse_width;
	MC_PWM_OUT_4_PW = BUF_PWM_IN_4.pulse_width;
	MC_PWM_OUT_5_PW = BUF_PWM_IN_5.pulse_width;
	printf("num of connected devices = 6");
	}
    else if (num_devices == 5)
	{
	MC_PWM_OUT_0_PW = BUF_PWM_IN_0.pulse_width;
	MC_PWM_OUT_1_PW = BUF_PWM_IN_1.pulse_width;
	MC_PWM_OUT_2_PW = BUF_PWM_IN_2.pulse_width;
	MC_PWM_OUT_3_PW = BUF_PWM_IN_3.pulse_width;
	MC_PWM_OUT_4_PW = BUF_PWM_IN_4.pulse_width;
	MC_PWM_OUT_5_PW = BUF_PWM_IN_5.pulse_width;
	if (BUF_PWM_IN_0.is_on == 0)
	    {
	    MC_PWM_OUT_0_PW = BUF_PWM_IN_1.pulse_width;
	    }
	else if (BUF_PWM_IN_1.is_on == 0)
	    {
	    MC_PWM_OUT_1_PW = BUF_PWM_IN_0.pulse_width;
	    }
	else if (BUF_PWM_IN_2.is_on == 0)
	    {
	    MC_PWM_OUT_2_PW = BUF_PWM_IN_1.pulse_width;
	    }
	else if (BUF_PWM_IN_3.is_on == 0)
	    {
	    MC_PWM_OUT_3_PW = BUF_PWM_IN_2.pulse_width;
	    }
	else if (BUF_PWM_IN_4.is_on == 0)
	    {
	    MC_PWM_OUT_4_PW = BUF_PWM_IN_3.pulse_width;
	    }
	else if (BUF_PWM_IN_5.is_on == 0)
	    {
	    MC_PWM_OUT_5_PW = BUF_PWM_IN_4.pulse_width;
	    }
	printf("num of connected devices = 5");
	}
    else if (num_devices == 4)
	{
	MC_PWM_OUT_0_PW = BUF_PWM_IN_0.pulse_width;
	MC_PWM_OUT_1_PW = BUF_PWM_IN_1.pulse_width;
	MC_PWM_OUT_2_PW = BUF_PWM_IN_2.pulse_width;
	MC_PWM_OUT_3_PW = BUF_PWM_IN_3.pulse_width;
	MC_PWM_OUT_4_PW = BUF_PWM_IN_4.pulse_width;
	MC_PWM_OUT_5_PW = BUF_PWM_IN_5.pulse_width;
	if (BUF_PWM_IN_0.is_on == 0)
	    {
	    MC_PWM_OUT_0_PW = BUF_PWM_IN_1.pulse_width;
	    }
	else if (BUF_PWM_IN_1.is_on == 0)
	    {
	    MC_PWM_OUT_1_PW = BUF_PWM_IN_0.pulse_width;
	    }
	else if (BUF_PWM_IN_2.is_on == 0)
	    {
	    MC_PWM_OUT_2_PW = BUF_PWM_IN_1.pulse_width;
	    }
	else if (BUF_PWM_IN_3.is_on == 0)
	    {
	    MC_PWM_OUT_3_PW = BUF_PWM_IN_2.pulse_width;
	    }
	else if (BUF_PWM_IN_4.is_on == 0)
	    {
	    MC_PWM_OUT_4_PW = BUF_PWM_IN_3.pulse_width;
	    }
	else if (BUF_PWM_IN_5.is_on == 0)
	    {
	    MC_PWM_OUT_5_PW = BUF_PWM_IN_4.pulse_width;
	    }
	printf("num of connected devices = 4");
	}
    else if (num_devices == 3)
	{
	MC_PWM_OUT_0_PW = BUF_PWM_IN_0.pulse_width;
	MC_PWM_OUT_1_PW = BUF_PWM_IN_1.pulse_width;
	MC_PWM_OUT_2_PW = BUF_PWM_IN_2.pulse_width;
	MC_PWM_OUT_3_PW = BUF_PWM_IN_3.pulse_width;
	MC_PWM_OUT_4_PW = BUF_PWM_IN_4.pulse_width;
	MC_PWM_OUT_5_PW = BUF_PWM_IN_5.pulse_width;
	printf("num of connected devices = 3");
	}
    else if (num_devices == 2)
	{

	printf("num of connected devices = 2");
	}
    else if (num_devices == 1)
	{
	if (BUF_PWM_IN_0.is_on == 1)
	    {
	    MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = BUF_PWM_IN_0.pulse_width;
	    } else if (BUF_PWM_IN_1.is_on == 1)
		{
		MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = BUF_PWM_IN_1.pulse_width;
		} else if (BUF_PWM_IN_2.is_on == 1)
		    {
		    MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = BUF_PWM_IN_2.pulse_width;
		    } else if (BUF_PWM_IN_3.is_on == 1)
			{
			MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = BUF_PWM_IN_3.pulse_width;
			} else if (BUF_PWM_IN_4.is_on == 1)
			    {
			    MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = BUF_PWM_IN_4.pulse_width;
			    } else if (BUF_PWM_IN_5.is_on == 1)
				{
				MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = BUF_PWM_IN_5.pulse_width;
				}
	printf("num of connected devices = 1");
	}
    else
	{
	MC_PWM_OUT_0_PW = MC_PWM_OUT_1_PW = MC_PWM_OUT_2_PW = MC_PWM_OUT_3_PW = MC_PWM_OUT_4_PW = MC_PWM_OUT_5_PW = 100;
	printf("WARNING! num of connected devices = 0");
	}
    }
/* USER CODE END 1 */
