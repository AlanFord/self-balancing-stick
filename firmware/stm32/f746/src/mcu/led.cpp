/////////////////////////////////////////////////////////////////////////
///	\file led.c
///	\brief this is the LED hardware interface layer.
///
///	Author: Alan Ford
/////////////////////////////////////////////////////////////////////////
#include "nodate.h"
#include "mcu/led.h"

/////////////////////////////////////////////////////////////////////////
///	\brief defines the LED pin number
/////////////////////////////////////////////////////////////////////////
uint8_t 	led_pin = 5;
GPIO_ports 	led_port = GPIO_PORT_A;
bool        pin_state = false;

/////////////////////////////////////////////////////////////////////////
///	\brief	Turns on the LED
/////////////////////////////////////////////////////////////////////////
void Led_On(void)
{
	//GPIOB->BSRR |= ((uint32_t)1 << LED_PIN);
	GPIO::write(led_port, led_pin, GPIO_LEVEL_HIGH);
	pin_state = true;
}

/////////////////////////////////////////////////////////////////////////
///	\brief	Turns off the LED
/////////////////////////////////////////////////////////////////////////
void Led_Off(void)
{
	//GPIOB->BRR |= ((uint32_t)1 << LED_PIN);
	GPIO::write(led_port, led_pin, GPIO_LEVEL_LOW);
	pin_state = false;
}

/////////////////////////////////////////////////////////////////////////
/// \brief  Toggle the LED state
/////////////////////////////////////////////////////////////////////////
void Led_Toggle(void)
{
    // read the IO state. if set the turn the led off else turn it on
    if(pin_state)
    {
        Led_Off();
    }
    else
    {
        Led_On();
    }
}
/////////////////////////////////////////////////////////////////////////
///	\brief	Setup the LED IO
/////////////////////////////////////////////////////////////////////////
void Led_Init(void)
{
	//RCC->AHBENR |= ((uint32_t)RCC_AHBENR_GPIOBEN); // enable port B clock

	// configure port
	//GPIOB->MODER   &= ~((uint32_t)GPIO_MODER_MODER10);     // clear the related bits before we set them. Comment the next call will set all bits.
	//GPIOB->MODER   |=  ((uint32_t)GPIO_MODER_MODER10_0);   // the pin as output
	//GPIOB->OTYPER  &= ~((uint32_t)GPIO_OTYPER_OT_10); 	   // set it to push-pull
	//GPIOB->OSPEEDR |=  ((uint32_t)GPIO_OSPEEDR_OSPEEDR10); // set to 50Mhz
	//GPIOB->PUPDR   &= ~((uint32_t)GPIO_PUPDR_PUPDR10);	   // no pull up/down resistor

	//Led_Off();
	// TODO: see if pull up is really needed.
	GPIO::set_output(led_port, led_pin, GPIO_FLOATING);
}
