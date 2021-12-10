/*
 * imu.h
 *
 *  Created on: Dec 6, 2021
 *      Author: alan
 */

#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#include "common.h"
#include "MPU6050_6Axis_motionApps20.h"

#ifdef __cplusplus
extern "C" {
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

class IMU {
	uint16_t dmpReady = FALSE;     // set true if DMP init was successful
	uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
	uint8_t fifoBuffer[64];       // FIFO storage buffer
	MPU6050_6Axis_MotionApps20 mpu;            // Creating object 'mpu', I think

	/* these are used to pass data to the controllers */
	float theta_Now;
	float theta_Zero;
	float theta_Integral;
	float theta_Speed_Now;
	float left_Speed_RPM;
	float omega_Now;
	float omega_Zero;
	float omega_Integral;
	float omega_Speed_Now;
	float right_Speed_RPM;

public:
	IMU(I2C_HandleTypeDef *hi2c, uint8_t address = MPU6050_DEFAULT_ADDRESS);
	void get_IMU_values(void);
	void get_theta_values(float &theta_Now, float &theta_Integral,
			float &theta_Speed_Now);
	void get_omega_values(float &omega_Now, float &omega_Integral,
			float &omega_Speed_Now);
	uint16_t get_Status(void) {
		return dmpReady;
	}
};

#endif /* INC_IMU_HPP_ */
