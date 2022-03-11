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
#include "filters.hpp"

#define _BV(n) (1 << n)


#ifdef __cplusplus
extern "C" {
#endif
// the callback must have "C" linkage
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#ifdef __cplusplus
}
#endif

enum class imu_angle { THETA, OMEGA };


class IMU {
	uint16_t dmpReady = false;     // set true if DMP init was successful
	uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
	uint8_t fifoBuffer[64];       // FIFO storage buffer
	MPU6050_6Axis_MotionApps20 mpu;            // Creating object 'mpu', I think
	ExponentialFilter thetaFilter;
	ExponentialFilter thetaSpeedFilter;
	ExponentialFilter omegaFilter;
	ExponentialFilter omegaSpeedFilter;

	float theta_Now;
	float theta_Speed_Now;
	float omega_Now;
	float omega_Speed_Now;
	uint32_t deltaTime;

	bool update_ypr_values(float (&ypr)[3], uint32_t *timestamp);

public:
	IMU(I2C_HandleTypeDef *hi2c, uint8_t address = MPU6050_DEFAULT_ADDRESS);
	bool update_IMU_values(void);
	void get_values(imu_angle angle, float &angle_Now,
			float &angle_Speed_Now, uint32_t& deltaTime);
	bool GetCurrentFIFOPacket(uint8_t *data, uint8_t length, uint32_t *timestamp);
	uint16_t get_Status(void) {
		return dmpReady;
	}
};

#endif /* INC_IMU_HPP_ */
