/*
 * hardware.hpp
 *
 *  Created on: Jan 27, 2022
 *      Author: alan
 */

#ifndef INC_HARDWARE_HPP_
#define INC_HARDWARE_HPP_


#include "main.h"
#include "tim.h"
#include "imu.hpp"
#include <encoder.hpp>
#include <motor.hpp>
#include "controller.hpp"

// the hardware!
extern IMU imu;
extern Motor rightMotor;
extern Motor leftMotor;
extern Encoder leftEncoder;
extern Encoder rightEncoder;

extern bool left_controller_active;
extern bool right_controller_active;

#endif /* INC_HARDWARE_HPP_ */
