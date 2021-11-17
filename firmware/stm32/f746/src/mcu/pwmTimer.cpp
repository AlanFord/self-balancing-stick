/*
	pwmTimer.cpp - Implementation file for the PWM Timer class.
*/

#include <nodate.h>


#include "pwmTimer.h"


/*---------------------------------------------------------------------------*/
/** @brief TIMER PWM Timer Initialization
 
This configures a timer perpheral in PWM output mode.
 
*/

PwmTimer::PwmTimer(TimerDevice device, GPIO_pin pinA, uint32_t psc, uint32_t arr) : Timer(device) {
	this->device = device;
	Timer_device &instance = TimerList[this->device];
	// make sure arr isn't too large for a 16-bit timer
	// 0xFFFFFFFF is ok as it is the reset value for the 32-bit register (upper 16-bits are ignored)
	if (!(IS_TIM_32B_COUNTER_INSTANCE(instance.regs)) && (arr != 0xFFFFFFFF)) {
		if (arr > 0xFFFF) {
			instance.active = false;
			return;
		}
	}
	// configure the appropriate GPIO pins
	if (!GPIO::set_af(pinA.port, pinA.pin, pinA.af)) {
		Rcc::disablePort((RccPort) pinA.port);
		instance.active = false;
	}
	// Start timer clock if needed.
	if (!instance.active) {
		if (Rcc::enable(instance.per)) {
			instance.active = true;
		}
	}
	// Configure the timer
	
	// turn the counter off
	instance.regs->CR1 &= ~(TIM_CR1_CEN);
	// reset the peripheral
	Rcc::reset(instance.per);
	// set the prescaler and autoreload values
	instance.regs->PSC   = psc;
	instance.regs->ARR   = arr;
	// setup for pwm
	instance.regs->CCR1 = ccr;   // defines the duty cycle
	instance.regs->CCMR2 &= ~(TIM_CCMR2_CC1S); // CC1 channel is configured as output
	instance.regs->CCER &= ~(TIM_CCER_CC1P);   // Output Polarity set to Active High
	instance.regs->CCMR1 &= ~(TIM_CCMR1_OC1M); // Output Compare 1 Mode set as PWM Mode 1.
	instance.regs->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR2_OC1M_2; // Output Compare 4 Mode set as PWM Mode 1.
	instance.regs->CCMR1 |= TIM_CCMR1_OC1PE; // Enable CCR1 preload register
	instance.regs->CCER |= TIM_CCER_CC1E; // Capture/Compare 1 Output Enable
	instance.regs->EGR |= TIM_EGR_UG;    // Before starting the counter, you have to initialize all the registers
	// instance.regs->BDTR |= TIM_BDTR_MOE;  // only available on advanced timers
	instance.regs->CR1 |= TIM_CR1_CEN; // Start Timer

	
	// select both TI1 and TI2 inputs
	instance.regs->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk);
	instance.regs->CCMR1 |= 0x01 << TIM_CCMR1_CC1S_Pos;
	instance.regs->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk);
	instance.regs->CCMR1 |= 0x01 << TIM_CCMR1_CC2S_Pos;
	// set input polarities
	instance.regs->CCER &= ~(TIM_CCER_CC1P_Msk);
	instance.regs->CCER &= ~(TIM_CCER_CC1NP_Msk);
	instance.regs->CCER &= ~(TIM_CCER_CC2P_Msk);
	instance.regs->CCER &= ~(TIM_CCER_CC1NP_Msk);
	// set encoder mode
	switch (mode) {
		case Mode1:
			instance.regs->SMCR &=  ~(TIM_SMCR_SMS_Msk);
			instance.regs->SMCR |= 0x01 << TIM_SMCR_SMS_Pos;;
		case Mode2:
			instance.regs->SMCR &=  ~(TIM_SMCR_SMS_Msk);
			instance.regs->SMCR |= 0x10 << TIM_SMCR_SMS_Pos;;
		case Mode3:
			instance.regs->SMCR &=  ~(TIM_SMCR_SMS_Msk);
			instance.regs->SMCR |= 0x11 << TIM_SMCR_SMS_Pos;;
		default:
			instance.active = false;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief TIMER Destructor
 
This sets the output pwm duty cycle for the timer.
 
*/
bool PwmTimer::setDutyCycle(uint32_t arr, uint32_t ccr) {
	Timer_device &instance = TimerList[this->device];
	// ensure ccr is less than arr
	if (ccr > arr) {
		return false;
	}
	// setup for pwm
	instance.regs->CCR1 = ccr;
	return true;
}
