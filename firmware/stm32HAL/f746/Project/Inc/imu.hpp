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

typedef enum { THETA, OMEGA } imu_angle;


class IMU {
	uint16_t dmpReady = false;     // set true if DMP init was successful
	uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
	uint8_t fifoBuffer[64];       // FIFO storage buffer
	MPU6050_6Axis_MotionApps20 mpu;            // Creating object 'mpu', I think
	ExponentialFilter thetaFilter;
	ExponentialFilter thetaSpeedFilter;
	ExponentialFilter omegaFilter;
	ExponentialFilter omegaSpeedFilter;
	ExponentialFilter thetaAverageFilter;
	ExponentialFilter thetaSmoothedFilter;
	ExponentialFilter thetaZeroFilter;


	/* these are used to pass data to the controllers in the "get ... values" methods*/
	float theta_Now;
	float theta_Zero;
	float theta_Integral = 0;
	float theta_Speed_Now;
	float left_Speed_RPM;
	float omega_Now;
	float omega_Zero;
	float omega_Integral = 0;
	float omega_Speed_Now;
	float right_Speed_RPM;
	int p = 0;
	int p_Prev = 0;
	float angle_Average_Filter = 0.970;
	float angle_Smoothed_Filter = 0.997;
	float theta_Zero_Filter = 0.995;
	float omega_Zero_Filter = 0.986;
	float theta_Kt = 0.6;
	float theta_Ktd = 0;
	float omega_Kt = 0.6;
	float omega_Ktd = 1.0;

	bool update_ypr_values(float (&ypr)[3], uint32_t *timestamp);

public:
	IMU(I2C_HandleTypeDef *hi2c, uint8_t address = MPU6050_DEFAULT_ADDRESS);
	void set_angle_Average_Filter(float filter_value);
	float get_angle_Average_Filter(void);
	void set_angle_Smoothed_Filter(float filter_value);
	float get_angle_Smoothed_Filter(void);
	void set_Zero_Filter(imu_angle angle, float filter_value);
	float get_Zero_Filter(imu_angle angle);
	void set_Zero(imu_angle angle, float filter_value);
	float get_Zero(imu_angle angle);
	void set_theta_Zero(imu_angle angle, float value);
	float get_theta_Zero(imu_angle angle);
	void set_Ktd(imu_angle angle, float value);
	float get_Ktd(imu_angle angle);
	void set_Kt(imu_angle angle, float value);
	float get_Kt(imu_angle angle);
	bool update_IMU_values(void);
	void get_values(imu_angle angle, float &angle_Now, float &angle_Integral,
			float &angle_Speed_Now, float& angle_Zero);
	bool GetCurrentFIFOPacket(uint8_t *data, uint8_t length, uint32_t *timestamp);
	uint16_t get_Status(void) {
		return dmpReady;
	}
};

#endif /* INC_IMU_HPP_ */
