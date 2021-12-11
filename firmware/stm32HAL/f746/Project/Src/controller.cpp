/*
 * controller.c
 *
 *  Created on: Dec 8, 2021
 *      Author: alan
 */

// Purpose:  Implements the Control Functions
//      void get_left_PID_Voltage_Value() -   sets left_PID_Voltage
//      void get_right_PID_Voltage_Value() -  sets right_PID_Voltage
/////////////////////////////////////////////////////////
#include <controller.hpp>
#include <motor.hpp>
#include "imu.hpp"
#include "encoder.hpp"
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))



Controller::Controller(Direction angle, float Kp, float Ki, float Kd, float Ks,
		IMU *imu, Encoder *encoder, Motor *motor, float friction) {
	this->angle = angle;
	angle_Kp = Kp;
	angle_Ki = Ki;
	angle_Kd = Kd;
	angle_Ks = Ks;
	friction_Value = friction;
	this->imu = imu;
	this->encoder = encoder;
	this->motor = motor;
}

int Controller::get_PID_Voltage_Value() {
	float angle_Now, angle_Integral, angle_Speed_Now, angle_Zero;
	if (angle == theta) {
		imu->get_theta_values(angle_Now, angle_Integral, angle_Speed_Now, angle_Zero);
	} else {
		imu->get_omega_values(angle_Now, angle_Integral, angle_Speed_Now, angle_Zero);
	}
	float speed_RPM, accel;
	encoder->get_Encoder_Speeds(speed_RPM, accel);
	float P_Accel = angle_Kp * (angle_Now - angle_Zero);
	float I_Accel = angle_Ki * angle_Integral;
	float D_Accel = angle_Kd * angle_Speed_Now;
	float S_Accel = angle_Ks * speed_RPM / 1000.;
	float PID_Accel = P_Accel + I_Accel + D_Accel + S_Accel;

	float friction = 0;
	if (speed_RPM > 0.5) {
		friction = friction_Value;
	} else if (speed_RPM < -0.5) {
		friction = -friction_Value;
	}

	float voltage = (PID_Accel + 0.30 * speed_RPM + friction) / 9.4; // Equation measured from Acceleration Motor Tests
	int voltage_limit = motor->get_max_voltage();
	int PID_Voltage = round(constrain(voltage, -voltage_limit, voltage_limit));
	return PID_Voltage;
}

void Controller::set_Kp(float value) {
	angle_Kp = value;
}
void Controller::set_Ki(float value) {
	angle_Ki = value;
}
void Controller::set_Kd(float value) {
	angle_Kd = value;
}
void Controller::set_Ks(float value) {
	angle_Ks = value;
}
void Controller::set_friction(float value) {
	friction_Value = value;
}

