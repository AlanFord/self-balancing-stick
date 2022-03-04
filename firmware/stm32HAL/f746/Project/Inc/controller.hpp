/*
 * controller.h
 *
 *  Created on: Dec 8, 2021
 *      Author: alan
 */

#ifndef INC_CONTROLLER_HPP_
#define INC_CONTROLLER_HPP_

#include "common.h"
#include "encoder.hpp"
#include "imu.hpp"
#include "motor.hpp"

typedef enum { OFF, MANUAL, AUTO } controller_mode;

class Controller {
	const float stiction_speed_threshold = 0.5;  // RPM
	int PID_Voltage = 0;
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;
	float Ks = 0;
	// Defines amount of voltage added to compensate for motor stiction, [0 - 255].
	// The other option to use is the motor voltage_Offset.
	// voltage_Offset is ALWAYS used, while friction_Value is applied at low speeds.
	// friction value was used in the original source.
	float friction_Value = 0;
	controller_mode mode = OFF;
	int default_voltage = 0;

	Encoder *encoder;
	IMU *imu;
	Motor *motor;
	imu_angle angle;
public:
	Controller(imu_angle angle, float Kp, float Ki, float Kd, float Ks,
			IMU *imu, Encoder *encoder, Motor *motor, float friction = 10.);
	int get_PID_Voltage_Value();
	void set_Kp(float value);
	void set_Ki(float value);
	void set_Kd(float value);
	void set_Ks(float value);
	void set_friction(float value);
	float get_Kp(void);
	float get_Ki(void);
	float get_Kd(void);
	float get_Ks(void);
	float get_friction(void);
	void set_mode(controller_mode new_mode);
	controller_mode get_mode(void);
	void set_default_voltage(int voltage);
	int get_defult_voltage(void);
};
#endif /* INC_CONTROLLER_HPP_ */
