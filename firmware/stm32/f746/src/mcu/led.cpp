/////////////////////////////////////////////////////////////////////////
///	\file led.cpp
///	\brief this is the LED hardware interface layer.
///
///	Author: Alan Ford
/////////////////////////////////////////////////////////////////////////
#include "nodate.h"
#include "mcu/led.h"

/////////////////////////////////////////////////////////////////////////
///	\brief defines the LED pin number
/////////////////////////////////////////////////////////////////////////
//uint8_t 	led_pin = 0;
//GPIO_ports 	led_port = GPIO_PORT_B;
//bool        pin_state = false;

/////////////////////////////////////////////////////////////////////////
///	\brief	Turns on the LED
/////////////////////////////////////////////////////////////////////////
void LED::On(void)
{
	GPIO::write(led_port, led_pin, GPIO_LEVEL_HIGH);
	pin_state = true;
}

/////////////////////////////////////////////////////////////////////////
///	\brief	Turns off the LED
/////////////////////////////////////////////////////////////////////////
void LED::Off(void)
{
	GPIO::write(led_port, led_pin, GPIO_LEVEL_LOW);
	pin_state = false;
}

/////////////////////////////////////////////////////////////////////////
/// \brief  Toggle the LED state
/////////////////////////////////////////////////////////////////////////
void LED::Toggle(void)
{
    // read the IO state. if set the turn the led off else turn it on
    if(pin_state)
    {
        Off();
    }
    else
    {
        On();
    }
}
/////////////////////////////////////////////////////////////////////////
///	\brief	Setup the LED IO
/////////////////////////////////////////////////////////////////////////
LED::LED(GPIO_ports led_port, uint8_t led_pin) {
	this->led_pin = led_pin;
	this->led_port = led_port;
	GPIO::set_output(led_port, led_pin, GPIO_FLOATING);
}

