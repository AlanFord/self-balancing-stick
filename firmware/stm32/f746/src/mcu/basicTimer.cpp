/*
 basicTimer.cpp - Implementation file for a basic timer class.
 */

#include <basicTimer.h>

count int timer_count = 8;

// --- basicTimer devices ---
Basic_timer_device* BasicTimer_list() {
	Basic_timer_device item;
	static Basic_timer_device timer_devices[timer_count];
	for (int i = 0; i < timer_count; ++i) {
		timer_devices[i] = item;
	}
	
#ifdef TIM1
	timer_devices[TIMER_1].regs = TIM1;
	timer_devices[TIMER_1].per  = RCC_TIM1;
	timer_devices[TIMER_1].reset = RCC_APB2RSTR_TIM1RST;
#endif
#ifdef TIM2
	timer_devices[TIMER_2].regs = TIM2;
	timer_devices[TIMER_2].per  = RCC_TIM2;
	timer_devices[TIMER_2].reset = RCC_APB1RSTR_TIM2RST;
#endif
#ifdef TIM3
	timer_devices[TIMER_3].regs = TIM3;
	timer_devices[TIMER_3].per  = RCC_TIM3;
	timer_devices[TIMER_3].reset = RCC_APB1RSTR_TIM3RST;
#endif
#ifdef TIM4
	timer_devices[TIMER_4].regs = TIM4;
	timer_devices[TIMER_4].per  = RCC_TIM4;
	timer_devices[TIMER_4].reset = RCC_APB1RSTR_TIM4RST;
#endif
#ifdef TIM5
	timer_devices[TIMER_5].regs = TIM5;
	timer_devices[TIMER_5].per  = RCC_TIM5;
	timer_devices[TIMER_5].reset = RCC_APB1RSTR_TIM5RST;
#endif
#ifdef TIM6
	timer_devices[TIMER_6].regs = TIM6;
	timer_devices[TIMER_6].per  = RCC_TIM6;
	timer_devices[TIMER_6].reset = RCC_APB1RSTR_TIM6RST;
#endif
#ifdef TIM7
	timer_devices[TIMER_7].regs = TIM7;
	timer_devices[TIMER_7].per  = RCC_TIM7;
	timer_devices[TIMER_7].reset = RCC_APB1RSTR_TIM7RST;
#endif
#ifdef TIM8
	timer_devices[TIMER_8].regs = TIM8;
	timer_devices[TIMER_8].per  = RCC_TIM8;
	timer_devices[TIMER_8].reset = RCC_APB2RSTR_TIM8RST;

	return timer_devices;
}

Basic_timer_device* basicTimerList = BasicTimer_list();


// ---- Initialize Timer ------
bool BasicTimer::initialize(BasicTimerDevice_t device) {
	Basic_timer_device &instance = basicTimerList[device];
	// Start timer device if needed.
	if (!instance.active) {
		if (!Rcc::enable(instance.per)) {
			return false;
		}
		instance.active = true;
	}
	return true;
}

// ---- Start Timer ------
bool BasicTimer::start(BasicTimerDevice_t device) {
	Basic_timer_device &instance = basicTimerList[device];
	if (instance.active){
		// turn the counter off
		instance.regs->CR1 &= ~(TIM_CR1_CEN);
		// reset the peripheral
		//TODO: add reset to the RCC structure
		// set the prescaler/autoreload registers
		//TODO: fix these
		instance.regs->PSC   = core_clock_hz / 1000;
		instance.regs->ARR   = ms;
		// send update event to reset the timer and apply settings
		instance.regs->EGR |= TIM_EGR_UG;
		// enable interrupt if necessary
		//instance.regs->DIER |= TIM_DIER_UIE;
		// enable the timer
		instance.regs->CR1 |= TIM_CR1_CEN;
		return true;
	}
	else return false;
}

// ---- Stop Timer ------
bool BasicTimer::stop(BasicTimerDevice_t device) {
	Basic_timer_device &instance = basicTimerList[device];
	if (instance.active){
		// turn the counter off
		instance.regs->CR1 &= ~(TIM_CR1_CEN);
		// clear the 'pending update interrupt' flag, just in case
		instance.regs->SR &= ~(TIM_SR_UIF);
		return true;
	}
	else return false;
}

bool BasicTimer::setInterrupt(BasicTimerDevice_t device, std::function<void(uint8_t)> callback) {
	Basic_timer_device &instance = basicTimerList[device];
	instance.callback = callback;
	if (instance.active){
		uint32_t enable_status = instance.regs->CR1 & TIM_CR1_CEN;
		// turn the counter off
		instance.regs->CR1 &= ~(TIM_CR1_CEN);
		// enable interrupt if necessary
		instance.regs->DIER |= TIM_DIER_UIE;
		// enable the timer if it was turned on originally
		if (enable_status) {
			instance.regs->CR1 |= TIM_CR1_CEN;
		}
		return true;
	}
	else return false;
}

bool BasicTimer::clearInterrupt(BasicTimerDevice_t device) {
	Basic_timer_device &instance = basicTimerList[device];
	if (instance.active){
		uint32_t enable_status = instance.regs->CR1 & TIM_CR1_CEN;
		// turn the counter off
		instance.regs->CR1 &= ~(TIM_CR1_CEN);
		// disable interrupt if necessary
		instance.regs->DIER &= ~(TIM_DIER_UIE);
		//TODO: avoid enabling time if not initially enabled
		// enable the timer if it was turned on originally
		if (enable_status) {
			instance.regs->CR1 |= TIM_CR1_CEN;
		}
		return true;
	}
	else return false;
}

// ---- Return Timer Counter ----
uint32_t BasicTimer::get_counter(BasicTimerDevice_t device) {
	uint32_t retVal;
	Basic_timer_device &instance = basicTimerList[device];
	if (!(instance.active)) {
		retVal = 0;
	}
	else if (IS_TIM_32B_COUNTER_INSTANCE(instance.regs) ) {
		retVal =  instance.regs->CNT;
	}
	else {
		retVal = (uint32_t) instance.regs->CNT &0x0000FFFFUL;
	}
	return retVal;
}

bool BasicTimer::reset_counter(BasicTimerDevice_t device) {
	Basic_timer_device &instance = basicTimerList[device];
	if (instance.active) {
		uint32_t enable_status = instance.regs->CR1 & TIM_CR1_CEN;
		// turn the counter off
		instance.regs->CR1 &= ~(TIM_CR1_CEN);
		// send update event to reset the timer and apply settings
		instance.regs->EGR |= TIM_EGR_UG;
		// enable the timer if it was turned on originally
		if (enable_status) {
			instance.regs->CR1 |= TIM_CR1_CEN;
		}
		return true;
	}
	return false;
}

