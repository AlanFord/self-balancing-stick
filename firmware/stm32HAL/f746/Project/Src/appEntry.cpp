#include "main.h"
#include "tim.h"
#include "motor.h"
#include "imu.hpp"
#include "appEntry.hpp"
#include "terminal.h"
#include "encoder.h"

void app_entry(void) {
	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);
	//initialize right motor encoder
	initialize_encoder (&htim3);
	//initialize left motor encoder
	initialize_encoder (&htim4);
	//initialize right motor
	initialize_motor(&htim1, R_MOTOR_DIR_GPIO_Port, R_MOTOR_DIR_Pin);
	//initialize left motor
	initialize_motor(&htim2, L_MOTOR_DIR_GPIO_Port, L_MOTOR_DIR_Pin);
	//initialize imu
	IMU imu(&hi2c1);
	Terminal_Init();
	while (1) {
		// Everything from here down should be non-blocking
		Terminal_Process();
		//TODO: Do Stuff!
	}

}
