/*
 * motor.h
 *
 *  Created on: Dec 6, 2021
 *      Author: alan
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif
#include "common.h"

#ifdef __cplusplus
}
#endif

#define MOTOR_FORWARD GPIO_PIN_RESET
#define MOTOR_REVERSE GPIO_PIN_SET
#define MAX_MOTOR_PWM_KHZ 50

class Motor {
	int voltage_Max = 0;
	int voltage_Offset = 0; // Defines amount of voltage added to compensate for motor stiction, [0 - 255].
	int voltage = 0;
	TIM_HandleTypeDef *htim;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
public:
	Motor(TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
	HAL_StatusTypeDef start();
	HAL_StatusTypeDef stop();
	void set_voltage(int32_t voltage);
	int get_voltage();
	void set_max_voltage(int voltage);
	int get_max_voltage();
	void set_voltage_offset(int voltage);
	int get_voltage_offset();
};

#endif /* INC_MOTOR_HPP_ */
