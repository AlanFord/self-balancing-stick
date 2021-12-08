/*
 * motor.h
 *
 *  Created on: Dec 6, 2021
 *      Author: alan
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

void initialize_motor(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void set_Motor_Voltage(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int voltage);


#endif /* INC_MOTOR_H_ */
