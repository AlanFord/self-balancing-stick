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

enum class controller_mode { OFF, MANUAL, AUTO };

class Controller {
	float angle_Average_Filter;
	float angle_Smoothed_Filter;
	float angle_Zero_Filter;
	ExponentialFilter angleAverageFilter;
	ExponentialFilter angleSmoothedFilter;
	ExponentialFilter angleZeroFilter;
	const float stiction_speed_threshold{0.5};  // RPM
	float angle_Zero;
	int PID_Voltage{0};
	float Kp{0};
	float Ki{0};
	float Kd{0};
	float Ks{0};
	float Kt{0.6};
	float Ktd{0};
	float angle_Integral{0};
	const float angle_Speed_Filter{0.7};
	const float angle_Integral_Max{3.0};
	float angle_Smoothed{0};
	float angle_Smoothed_Speed{0};
	// Defines amount of voltage added to compensate for motor stiction, [0 - 255].
	// The other option to use is the motor voltage_Offset.
	// voltage_Offset is ALWAYS used, while friction_Value is applied at low speeds.
	// friction value was used in the original source.
	float friction_Value{0};
	controller_mode mode{controller_mode::OFF};
	int default_voltage{0};

	Encoder *encoder;
	IMU *imu;
	Motor *motor;
	imu_angle angle;
public:
	Controller(imu_angle angle, float Kp, float Ki, float Kd, float Ks,
			IMU *imu, Encoder *encoder, Motor *motor, float friction = 10.);
	int get_PID_Voltage_Value();
	void set_angle_Average_Filter(float filter_value);
	float get_angle_Average_Filter(void);
	void set_angle_Smoothed_Filter(float filter_value);
	float get_angle_Smoothed_Filter(void);
	void set_Zero_Filter(float filter_value);
	float get_Zero_Filter(void);
	void set_Zero(float filter_value);
	float get_Zero(void);
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
	void set_Ktd(float value);
	float get_Ktd(void);
	void set_Kt(float value);
	float get_Kt(void);
	void set_mode(controller_mode new_mode);
	controller_mode get_mode(void);
	void set_default_voltage(int voltage);
	int get_defult_voltage(void);
};
#endif /* INC_CONTROLLER_HPP_ */
