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

/* these are used to pass data to the controllers */
extern float theta_Now;
extern float theta_Zero;
extern float theta_Integral;
extern float theta_Speed_Now;
extern float left_Speed_RPM;
extern float omega_Now;
extern float omega_Zero;
extern float omega_Integral;
extern float omega_Speed_Now;
extern float right_Speed_RPM;

#ifdef __cplusplus
	extern "C"
	{
#endif

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
	}
#endif

class IMU {
	uint16_t dmpReady = FALSE;     // set true if DMP init was successful
	uint16_t packetSize;           // expected DMP packet size (default is 42 bytes)
	uint8_t fifoBuffer[64];       // FIFO storage buffer
	MPU6050_6Axis_MotionApps20 mpu;                  // Creating object 'mpu', I think
public:
	IMU(I2C_HandleTypeDef * hi2c, uint8_t address);
	void get_IMU_values(void);
};

#endif /* INC_IMU_HPP_ */
