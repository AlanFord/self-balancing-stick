/*
 * motor.c
 *
 *  Created on: Dec 4, 2021
 *      Author: alan
 */
#include <motor.hpp>
#include "tim.h"
#include "gpio.h"
#include <stdlib.h>


/**
 * @brief  Initializes a motor driver
 * @param  htim TIM Encoder Interface handle
 * @param  GPIOx where x can be (A..K) to select the GPIO peripheral.
 * @param  GPIO_Pin specifies the port bit to be written.
 *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
 */
Motor::Motor(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	this->htim = htim;
	this->GPIOx = GPIOx;
	this->GPIO_Pin = GPIO_Pin;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); //clockwise
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);  // full stop
}
HAL_StatusTypeDef Motor::start(){
	return HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

HAL_StatusTypeDef Motor::stop(){
	return HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
}

int Motor::get_voltage(){
	return voltage;
}

void Motor::set_max_voltage(int voltage){
	voltage_Max = voltage;
}

int Motor::get_max_voltage(){
	return voltage_Max;
}

void Motor::set_voltage_offset(int voltage){
	voltage_Offset = voltage;
}

int Motor::get_voltage_offset(){
	return voltage_Offset;
}



/**
 * @brief  Sets the motor voltage and direction
 * @param  htim TIM Encoder Interface handle
 * @param  GPIOx where x can be (A..K) to select the GPIO peripheral.
 * @param  GPIO_Pin specifies the port bit to be written.
 *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
 * @param  voltage specifies the motor PWM value between -255 and 255. Nagative values
 *         indicate clockwise rotation and positive values indicate counterclockwise rotation
 */
void Motor::set_voltage(int32_t voltage) { // Switched directions (0's and 1's) from original
	this->voltage = voltage;
	if (voltage == 0) {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
	} else {
		if (voltage < 0) {
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, MOTOR_REVERSE);
		}
		else {
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, MOTOR_FORWARD);
		}
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,
				(abs(voltage) + voltage_Offset));
	}
}

