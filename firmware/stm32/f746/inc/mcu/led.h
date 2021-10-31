////////////////////////////////////////////////////////////////////////////////
/// \file led.h
/// Author: Alan Ford
/// 	Derived from work by Ronald Sousa (@Opticalworm)
////////////////////////////////////////////////////////////////////////////////

#ifndef __LED_H__
#define __LED_H__

#include "universal.h"
#include "Nodate.h"

class LED {
	uint8_t 	led_pin;
	GPIO_ports 	led_port;
	bool        pin_state = false;
public:
	LED(GPIO_ports led_port, uint8_t led_pin);
	void On(void);
	void Off(void);
	void Toggle(void);
};

#endif /* __LED_H__ */
