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
	// Defines a friction offset to be added to the calculated motor voltage to compensate for motor stiction.
	// The other option to use is the controller friction_Value.
	// voltage_Offset is ALWAYS used, while friction_Value is applied at low speeds.
	// friction value was used in the original source.
	int voltage_Offset = 0;
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
