/*
 * controller.h
 *
 *  Created on: Dec 8, 2021
 *      Author: alan
 */

#ifndef INC_CONTROLLER_HPP_
#define INC_CONTROLLER_HPP_

#include "motor.hpp"
#include "imu.hpp"

//int get_left_PID_Voltage_Value();
//int get_right_PID_Voltage_Value();

class Controller {
	float           angle_Kp;
	float           angle_Ki;
	float           angle_Kd;
	float           angle_Ks;
	float     		friction_Value;
	Motor *motor;
	IMU *imu;
public:
	Controller(float Kp, float Ki, float Kd, float Ks, float friction = 10.);
	int get_PID_Voltage_Value();
	void set_Kp(float value);
	void set_Ki(float value);
	void set_Kd(float value);
	void set_Ks(float value);
	void set_friction(float value);
};
#endif /* INC_CONTROLLER_HPP_ */
