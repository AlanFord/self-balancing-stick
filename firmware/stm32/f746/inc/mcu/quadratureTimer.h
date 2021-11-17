/*
	timer.h - Header to provide access to Timer functionality.
	
	Features:
			- Core timer (SysTick) for delay functionality.
			- Peripheral timers features for timing, PWM, etc.
	For background, see:
 https://micromouseonline.com/2013/02/16/quadrature-encoders-with-the-stm32f4/
*/


#ifndef QUADRATURE_TIMER_H
#define QUADRATURE_TIMER_H


#include "common.h"
#include <nodate.h>

#include <functional>

enum EncoderMode {
	Mode1,
	Mode2,
	Mode3
};


class QuadratureTimer : Timer {
	//
	
public:
	QuadratureTimer(TimerDevice device, GPIO_pin pinA, uint16_t psc, uint32_t arr, uint32_t ccr, EncoderMode mode);
	~QuadratureTimer();

private:
};

#endif  // QUADRATURE_TIMER_H
