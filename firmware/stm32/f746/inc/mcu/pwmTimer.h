/*
	pwmTimer.h - Header to provide access to Timer functionality.
	
	Features:
			- Core timer (SysTick) for delay functionality.
			- Peripheral timers features for timing, PWM, etc.
	For background, see:
 https://micromouseonline.com/2013/02/16/quadrature-encoders-with-the-stm32f4/
*/


#ifndef PWM_TIMER_H
#define PWM_TIMER_H


#include "common.h"
#include <nodate.h>

#include <functional>


class PwmTimer : Timer {
	//
	
public:
	PwmTimer(TimerDevice device, GPIO_pin pinA, uint16_t psc, uint32_t arr, uint32_t ccr);
	bool setDutyCycle(uint32_t arr, uint32_t ccr);

private:
};

#endif // PWM_TIMER_H
