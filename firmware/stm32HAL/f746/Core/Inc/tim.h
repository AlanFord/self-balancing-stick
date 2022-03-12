/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/**
 * Note:  The timers used in this application are as follows:
 * Timer 1: PWM signal on Channel 1 for the Right motor
 * Timer 2: PWM signal on Channel 1 for the Left motor
 * Timer 3: Encoder, Channels 1 and 2, for the Right motor
 * Timer 4: Encoder, Channels 1 and 2, for the Left motor
 * Timer 5: Clock counter in microseconds
 */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN Private defines */
/**
  * @brief  Get the elapsed microseconds from Timer5.
  * @retval 16-bit or 32-bit value of the timer counter register (TIMx_CNT)
  */
#define __MICROS()  (__HAL_TIM_GET_COUNTER(&htim5))

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

