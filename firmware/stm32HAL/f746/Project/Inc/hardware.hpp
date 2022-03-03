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
extern IMU *imu_ptr;
extern Motor *rightMotor_ptr;
extern Motor *leftMotor_ptr;
extern Encoder *leftEncoder_ptr;
extern Encoder *rightEncoder_ptr;
extern Controller *rightController_ptr;
extern Controller *leftController_ptr;

extern bool left_controller_active;
extern bool right_controller_active;

#endif /* INC_HARDWARE_HPP_ */
