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
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/*
float           theta_Kp = 1200;
float           theta_Ki = 0;
float           theta_Kd = 130;
float           theta_Ks = -25;

float           omega_Kp = 1600;
float           omega_Ki = 0;
float           omega_Kd = 130;
float           omega_Ks = -25;

float     friction_Value        = 10;

int get_left_PID_Voltage_Value() {

  float left_P_Accel = theta_Kp * (theta_Now - theta_Zero);
  float left_I_Accel = theta_Ki * theta_Integral;
  float left_D_Accel = theta_Kd * theta_Speed_Now;
  float left_S_Accel = theta_Ks * left_Speed_RPM / 1000.;
  float left_PID_Accel = left_P_Accel + left_I_Accel + left_D_Accel + left_S_Accel;

  float friction = 0;
  if (left_Speed_RPM > 0.5) {
    friction = friction_Value;
  } else if (left_Speed_RPM < -0.5) {
    friction = -friction_Value;
  }

  float voltage = (left_PID_Accel + 0.303 * left_Speed_RPM + friction) / 9.4;     // Equation measured from Acceleration Motor Tests

  int left_PID_Voltage = round(constrain(voltage, -voltage_Max, voltage_Max));
  return left_PID_Voltage;
}


int get_right_PID_Voltage_Value() {

  float right_P_Accel = omega_Kp * (omega_Now - omega_Zero);
  float right_I_Accel = omega_Ki * omega_Integral;
  float right_D_Accel = omega_Kd * omega_Speed_Now;
  float right_S_Accel = omega_Ks * right_Speed_RPM / 1000.;
  float right_PID_Accel = right_P_Accel + right_I_Accel + right_D_Accel + right_S_Accel;

  float friction = 0;
  if (right_Speed_RPM > 0.5) {
    friction = friction_Value;
  } else if (right_Speed_RPM < -0.5) {
    friction = -friction_Value;
  }

  float voltage = (right_PID_Accel + 0.295 * right_Speed_RPM + friction) / 9.4;      // Equation measured from Acceleration Motor Tests


  int right_PID_Voltage = round(constrain(voltage, -voltage_Max, voltage_Max));
  return right_PID_Voltage;
}
*/

Controller::Controller(float Kp, float Ki, float Kd, float Ks, float friction):
		angle_Kp(Kp), angle_Ki(Ki), angle_Kd(Kd), angle_Ks(Ks), friction_Value(friction) {
}


int Controller::get_PID_Voltage_Value(){
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

	  float voltage = (PID_Accel + 0.30 * speed_RPM + friction) / 9.4;     // Equation measured from Acceleration Motor Tests
	  int voltage_limit = motor->get_max_voltage();
	  int PID_Voltage = round(constrain(voltage, -voltage_limit, voltage_limit));
	  return PID_Voltage;
}

void Controller::set_Kp(float value){
	angle_Kp=value;
}
void Controller::set_Ki(float value){
	angle_Ki=value;
}
void Controller::set_Kd(float value){
	angle_Kd=value;
}
void Controller::set_Ks(float value){
	angle_Ks=value;
}
void Controller::set_friction(float value){
	friction_Value=value;
}




