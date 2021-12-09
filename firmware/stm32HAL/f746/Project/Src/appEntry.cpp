#include <motor.hpp>
#include "main.h"
#include "tim.h"
#include "imu.hpp"
#include "appEntry.hpp"
#include "terminal.h"
#include "encoder.h"

void app_entry(void) {
	HAL_StatusTypeDef rcode;
	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);
	//initialize right motor encoder
	rcode = initialize_encoder (&htim3);
	if (rcode != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}
	//initialize left motor encoder
	rcode = initialize_encoder (&htim4);
	if (rcode != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}
	//initialize right motor
	Motor rightMotor(&htim1, R_MOTOR_DIR_GPIO_Port, R_MOTOR_DIR_Pin);
	if (rightMotor.start() != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}
	//initialize left motor
	Motor leftMotor(&htim2, L_MOTOR_DIR_GPIO_Port, L_MOTOR_DIR_Pin);
	if (leftMotor.start() != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}
	//initialize imu
	IMU imu(&hi2c1);
	if (imu.get_Status()!= TRUE) {
		printf("imu initialization failure");
		Error_Handler();
	}
	Terminal_Init();
	/////////////////////////////////////
	//
	//  TEST FRAMEWORK
	//
	leftMotor.set_voltage(50);

	/////////////////////////////////////
	while (1) {
		// Everything from here down should be non-blocking
		Terminal_Process();
		//TODO: Do Stuff!
	}

}
