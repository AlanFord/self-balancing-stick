/***************************************************************************************************
 * Precision microdrives example for 103-001 back EMF measure
 *
 *   29 april 2014
 ****************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "speed_BEMF.h"



int main(void){

	InitPorts();		// init output ports
	InitTimer1();		// PWM and interrupt for ADC trigger
	InitUSART();		// for data sending via usart
	InitADC();			// Initialize ADC

	sei();

	usart_puts("********** PMD BEMF METER ****************\r\n");

	OCR1A = 100;			// set PWM value

	while(1){

		/* calculate average value of BEMF*/
		bemf = 0;
		for (unsigned char ind = N_LOST_SAMPLES; ind < N_SAMPLES;  ind++)	bemf += t_bemf[ind];
		bemf = 1024 - bemf / (N_SAMPLES - N_LOST_SAMPLES);		// because BEMF is measured relative to ground result, needs to be inverted

		//--------------------------Send data --------------------------------------

		if (!Timer1){
			Timer1 = 30;
			usart_puts("BEMF: ");
			usart_integer(bemf);
			usart_puts("  \r");
		}

	}// while
}// main


//------------------------------------------------------------------------------------------------
//------------------- Timer 1 compare A  - adc trigger on falling edge for BEMF measuring --------


ISR(TIMER1_COMPA_vect){

	if (!(PINB & (1<<PB1)))	adc_get_bemf();   //if pinb4 is high, motor rotating freely - read voltage on motor
}

//----------------- Overflow timer1 -- as systick constans frequency 244Hz ~ 4ms

ISR(TIMER1_OVF_vect){

	register char x;

	x = Timer1;
	if(x) Timer1= --x;

}

