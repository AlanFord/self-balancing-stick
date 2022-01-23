#include <encoder.hpp>
#include <motor.hpp>
#include "main.h"
#include "tim.h"
#include "imu.hpp"
#include "controller.hpp"
#include "appEntry.hpp"
#include "terminal.h"

void app_entry(void) {
	HAL_StatusTypeDef rcode;
	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);

	//initialize right motor encoder
	Encoder rightEncoder(&htim3);
	addCallback(&htim3, &rightEncoder);
	rcode = rightEncoder.initEncoder();
	if (rcode != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}

	//initialize left motor encoder
	Encoder leftEncoder(&htim4);
	addCallback(&htim4, &leftEncoder);
	rcode = leftEncoder.initEncoder();
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
	if (imu.get_Status() != TRUE) {
		printf("imu initialization failure");
		Error_Handler();
	}
	float theta_Kp = 1200;
	float theta_Ki = 0;
	float theta_Kd = 130;
	float theta_Ks = -25;

	float omega_Kp = 1600;
	float omega_Ki = 0;
	float omega_Kd = 130;
	float omega_Ks = -25;

	float friction_Value = 10;

	//initialize right controller (omega)
	Controller rightController(omega, omega_Kp, omega_Ki, omega_Kd, omega_Ks,
			&imu, &rightEncoder, &rightMotor, friction_Value);

	//initialize left controller (theta)
	Controller leftController(theta, theta_Kp, theta_Ki, theta_Kd, theta_Ks,
			&imu, &leftEncoder, &leftMotor, friction_Value);

	Terminal_Init();

	/////////////////////////////////////
	while (1) {
		// Everything from here down should be non-blocking
		Terminal_Process();
		bool update_available = imu.update_IMU_values();
		// don't mess with motor voltages if no new imu angles are available
		if (update_available) {
			rightMotor.set_voltage(rightController.get_PID_Voltage_Value());
			leftMotor.set_voltage(leftController.get_PID_Voltage_Value());
		}
	}

}
