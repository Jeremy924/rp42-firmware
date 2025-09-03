/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "state.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim15;

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;

  if (clock_state & CLOCK_HIGH)
	  htim16.Init.Prescaler = 49999; // Results in a 10kHz timer clock (0.1ms per tick)
  else htim16.Init.Prescaler = 999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Set a default period, e.g., 50ms. This can be changed later.
  // 50ms / 0.1ms_per_tick = 500 ticks. Period = 500 - 1 = 499.
  htim16.Init.Period = 499;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

void MX_TIM15_Init(void) {
	  /* USER CODE BEGIN TIM16_Init 0 */

	  /* USER CODE END TIM16_Init 0 */

	  /* USER CODE BEGIN TIM16_Init 1 */

	  /* USER CODE END TIM16_Init 1 */
	  htim15.Instance = TIM15;

	  if (clock_state & CLOCK_HIGH)
		  htim15.Init.Prescaler = 29999; // Results in a 10kHz timer clock (0.1ms per tick)
	  else htim15.Init.Prescaler = 999;
	  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	  // Set a default period, e.g., 50ms. This can be changed later.
	  // 50ms / 0.1ms_per_tick = 500 ticks. Period = 500 - 1 = 499.
	  htim15.Init.Period = 499;
	  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	  htim15.Init.RepetitionCounter = 0;
	  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM16_Init 2 */

	  /* USER CODE END TIM16_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspInit 0 */

  /* USER CODE END TIM16_MspInit 0 */
    /* TIM16 clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* TIM16 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM16_MspInit 1 */

  /* USER CODE END TIM16_MspInit 1 */
  } else if (tim_baseHandle->Instance == TIM15) {
	  __HAL_RCC_TIM15_CLK_ENABLE();

	  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspDeInit 0 */

  /* USER CODE END TIM16_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* TIM16 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM16_MspDeInit 1 */

  /* USER CODE END TIM16_MspDeInit 1 */
  } else if (tim_baseHandle->Instance==TIM15) {
	  __HAL_RCC_TIM15_CLK_DISABLE();

	  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
