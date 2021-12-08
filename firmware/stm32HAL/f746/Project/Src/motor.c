/*
 * motor.c
 *
 *  Created on: Dec 4, 2021
 *      Author: alan
 */
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "motor.h"
#include <stdlib.h>

#define   voltage_Offset 0 // Defines amount of voltage added to compensate for motor stiction, [0 - 255].

void initialize_motor(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); //clockwise
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);  // full stop
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

void set_Motor_Voltage(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int voltage) { // Switched directions (0's and 1's) from original
	if (voltage == 0) {
		//analogWrite(pin_left_PMW, 0);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
	} else {
		if (voltage < 0) {
			//digitalWrite(pin_left_dir, 1);  //clockwise
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		}
		else {
			//digitalWrite(pin_left_dir, 0);  //counterclockwise
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		}
		//analogWrite(pin_left_PMW, abs(left_Voltage) + voltage_Offset);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,
				(abs(voltage) + voltage_Offset));
	}
}

