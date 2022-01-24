/*
 * @file controller.cpp
 * @author Alan Ford
 * @date   8 dec 2021
 * @brief Implements PID Controller functionality
 */

#include <controller.hpp>
#include <motor.hpp>
#include "imu.hpp"
#include "encoder.hpp"
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


/*
 * @brief initializes a new controller object
 * @param angle The direction associated with this controller, theta or omega
 * @param Kp PID imu proportional gain
 * @param Ki PID imu integral gain
 * @param Kd PID imu derivative gain
 * @param Ks PID rotor speed gain
 * @param imu pointer to an imu object
 * @param encoder pointer to an encoder object associated with this controller
 * @param motor pointer to a motor object associated with this controller
 * @param friction
 *
 */
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

/*
 * @brief calculate a motor voltage based on encoder, motor, and imu data
 * @returns motor voltage (between -255 and 255)
 */
int Controller::get_PID_Voltage_Value() {
	float angle_Now, angle_Integral, angle_Speed_Now, angle_Zero;
	if (angle == theta) {
		imu->get_theta_values(angle_Now, angle_Integral, angle_Speed_Now, angle_Zero);
	} else {
		imu->get_omega_values(angle_Now, angle_Integral, angle_Speed_Now, angle_Zero);
	}
	float speed;  //speed in RPM
	float accel;
	encoder->get_Encoder_Speeds(speed, accel);
	float P_Accel = angle_Kp * (angle_Now - angle_Zero);
	float I_Accel = angle_Ki * angle_Integral;
	float D_Accel = angle_Kd * angle_Speed_Now;
	float S_Accel = angle_Ks * speed / 1000.;
	float PID_Accel = P_Accel + I_Accel + D_Accel + S_Accel;

	float friction = 0;
	if (speed > stiction_speed_threshold) {
		friction = friction_Value;
	} else if (speed < -stiction_speed_threshold) {
		friction = -friction_Value;
	}

	float voltage = (PID_Accel + 0.30 * speed + friction) / 9.4; // Equation measured from Acceleration Motor Tests
	int voltage_limit = motor->get_max_voltage();
	PID_Voltage = round(constrain(voltage, -voltage_limit, voltage_limit));
	return PID_Voltage;
}

/*
 * @brief Updates Kp, the PID imu angle proportional gain
 * @param new value of Kp
 */
void Controller::set_Kp(float value) {
	angle_Kp = value;
}

/*
 * @brief Updates Ki, the PID imu angle integral gain
 * @param new value of Ki
 */
void Controller::set_Ki(float value) {
	angle_Ki = value;
}

/*
 * @brief Updates Kd, the PID imu angle derivative gain
 * @param new value of Kd
 */
void Controller::set_Kd(float value) {
	angle_Kd = value;
}

/*
 * @brief Updates Ks, the rotor speed proportional gain
 * @param new value of Ks
 */
void Controller::set_Ks(float value) {
	angle_Ks = value;
}

/*
 * @brief Updates friction factor
 * @param new value of friction factor
 */
void Controller::set_friction(float value) {
	friction_Value = value;
}

