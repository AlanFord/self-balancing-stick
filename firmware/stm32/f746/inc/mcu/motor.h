

#ifndef MOTOR_H
#define MOTOR_H

#include "nodate.h"

enum MotorDirection {
	FORWARD,
	BACKWARD,
	STOPPED
};

class Motor {
	
public:
	Motor(Timer pwmGenerator);
	bool forward(uint32_t speed);
	bool backward(uint32_t speed);
	bool stop();
	MotorDirection get_direction();
	~Motor();
	
private:
	bool initialized = false;
	MotorDirection direction = STOPPED;
};



#endif  // MOTOR_H

