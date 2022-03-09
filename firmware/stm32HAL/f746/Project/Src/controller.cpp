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
Controller::Controller(imu_angle angle, float Kp, float Ki, float Kd, float Ks,
		IMU *imu, Encoder *encoder, Motor *motor, float friction) {
	this->angle = angle;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->Ks = Ks;
	this->friction_Value = friction;
	this->imu = imu;
	this->encoder = encoder;
	this->motor = motor;
}

/*
 * @brief set the operating mode of the controller
 * @param new_mode ZERO, MANUAL, or AUTO
 */
void Controller::set_mode(controller_mode new_mode) {
	if ((new_mode == AUTO) && (new_mode != this->mode)) {
		// we are starting AUTO mode, so reset integrals
		angle_Integral = 0;
	}
	this->mode = new_mode;
}
/*
 * @brief returns the operating mode of the controller
 * @return the controller mode of type controller_mode, ZERO, MANUAL, or AUTO
 */
controller_mode Controller::get_mode(void) {
	return mode;
}

/*
 * @brief calculate a motor voltage based on encoder, motor, and imu data
 * @returns motor voltage (between -255 and 255)
 */
int Controller::get_PID_Voltage_Value() {
	int return_voltage;
	float angle_Now;
	float angle_Speed_Now;
	float angle_Zero;
	uint32_t deltaTime;
	imu->get_values(angle, angle_Now, angle_Speed_Now, angle_Zero, deltaTime);

	// calculate angle error in degrees
	float angle_Error = angle_Now - angle_Zero;

	// integral angle error in degree*seconds
	angle_Integral += angle_Error * deltaTime / 1000000.0;
	if (angle_Integral > angle_Integral_Max) {
		angle_Integral = angle_Integral_Max;
	} else if (angle_Integral < -angle_Integral_Max) {
		angle_Integral = -angle_Integral_Max;
	}

	float speed;  //speed in RPM
	float accel;
	encoder->get_Encoder_Speeds(speed, accel);
	float P_Accel = Kp * angle_Error;
	float I_Accel = Ki * angle_Integral;
	float D_Accel = Kd * angle_Speed_Now;
	float S_Accel = Ks * speed / 1000.;
	float PID_Accel = P_Accel + I_Accel + D_Accel + S_Accel;

	float friction = 0;
	if (speed > stiction_speed_threshold) {
		friction = friction_Value;
	} else if (speed < -stiction_speed_threshold) {
		friction = -friction_Value;
	}
	//FIXME: figure out where this equation came from
	float voltage = (PID_Accel + 0.30 * speed + friction) / 9.4; // Equation measured from Acceleration Motor Tests
	int voltage_limit = motor->get_max_voltage();
	PID_Voltage = round(constrain(voltage, -voltage_limit, voltage_limit));
	switch (mode) {
	case AUTO:
		return_voltage = PID_Voltage;
		break;
	case MANUAL:
		return_voltage = default_voltage;
		break;
	case OFF:
	default:
		return_voltage = 0;
		break;
	}
	return return_voltage;
}

/*
 * @brief Updates Kp, the PID imu angle proportional gain
 * @param new value of Kp
 */
void Controller::set_Kp(float value) {
	Kp = value;
}

/*
 * @brief Updates Ki, the PID imu angle integral gain
 * @param new value of Ki
 */
void Controller::set_Ki(float value) {
	Ki = value;
}

/*
 * @brief Updates Kd, the PID imu angle derivative gain
 * @param new value of Kd
 */
void Controller::set_Kd(float value) {
	Kd = value;
}

/*
 * @brief Updates Ks, the rotor speed proportional gain
 * @param new value of Ks
 */
void Controller::set_Ks(float value) {
	Ks = value;
}

/*
 * @brief Updates friction factor
 * @param new value of friction factor
 */
void Controller::set_friction(float value) {
	friction_Value = value;
}

/*
 * @brief Updates Kp, the PID imu angle proportional gain
 * @param new value of Kp
 */
float Controller::get_Kp(void) {
	return Kp;
}

/*
 * @brief Updates Ki, the PID imu angle integral gain
 * @param new value of Ki
 */
float Controller::get_Ki(void) {
	return Ki;
}

/*
 * @brief Updates Kd, the PID imu angle derivative gain
 * @param new value of Kd
 */
float Controller::get_Kd(void) {
	return Kd;
}

/*
 * @brief Updates Ks, the rotor speed proportional gain
 * @param new value of Ks
 */
float Controller::get_Ks(void) {
	return Ks;
}

/*
 * @brief Updates friction factor
 * @param new value of friction factor
 */
float Controller::get_friction(void) {
	return friction_Value;
}

/*
 * @brief updates the default output voltage used in MANUAL mode
 * @param voltage the new default voltage
 */
void Controller::set_default_voltage(int voltage){
	default_voltage = voltage;
}

/*
 * @brief returns the current default output voltage used in MANUAL mode
 * @return int default voltage (-255 to 255)
 */
int Controller::get_defult_voltage(void){
	return default_voltage;
}


