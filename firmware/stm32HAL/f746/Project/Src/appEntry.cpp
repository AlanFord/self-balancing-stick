#include "appEntry.hpp"
#include "terminal.h"
#include "hardware.hpp"


// the hardware!
IMU imu(&hi2c1);
Motor rightMotor(&htim1, R_MOTOR_DIR_GPIO_Port, R_MOTOR_DIR_Pin);
Motor leftMotor(&htim2, L_MOTOR_DIR_GPIO_Port, L_MOTOR_DIR_Pin);
Encoder leftEncoder(&htim4);
Encoder rightEncoder(&htim3);
bool left_controller_active = false;
bool right_controller_active = false;

void app_entry(void) {
	//PID constants for the theta motor
	constexpr float theta_Kp = 1200;
	constexpr float theta_Ki = 0;
	constexpr float theta_Kd = 130;
	constexpr float theta_Ks = -25;

	//PID constants for the omega motor
	constexpr float omega_Kp = 1600;
	constexpr float omega_Ki = 0;
	constexpr float omega_Kd = 130;
	constexpr float omega_Ks = -25;

	constexpr float friction_Value = 10;  // accomodate stiction

	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);

	//initialize right motor encoder
	addCallback(&htim3, &rightEncoder);
	HAL_StatusTypeDef rcode = rightEncoder.initEncoder();
	if (rcode != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}

	//initialize left motor encoder
	addCallback(&htim4, &leftEncoder);
	rcode = leftEncoder.initEncoder();
	if (rcode != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}

	//initialize right motor
	if (rightMotor.start() != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}
	//initialize left motor
	if (leftMotor.start() != HAL_OK) {
		printf("motor initialization failure");
		Error_Handler();
	}
	//initialize imu
	if (imu.get_Status() != TRUE) {
		printf("imu initialization failure");
		Error_Handler();
	}

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
			if (left_controller_active)
				leftMotor.set_voltage(leftController.get_PID_Voltage_Value());
			if (right_controller_active)
				rightMotor.set_voltage(rightController.get_PID_Voltage_Value());
		}
	}

}
