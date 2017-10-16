/***************************************************************************************************
 * Precision Microdrives example for 103-001 back EMF measure
 *
 *  Author Marcin Pergol
 *   2014
 *************************************************************************************** */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#include "speed_BEMF.h"

//**********************************************************************

volatile char Timer1;				// software timer for sys tick
int bemf;							// BACK EMF signal
unsigned int t_bemf[N_SAMPLES];		// BACK EMF samples table

//**********************************************************************

void InitPorts(void){

//---------- PORT B
	DDRB  |= (1<<DDB1); 				// config PB1 as output for PWM
}

//------------------------ Init Timer 1 as pwm --------------------------------------------

void InitTimer1(void){

 TCCR1A = (1<<COM1A1)|(1<<WGM11);	// Toggle OC1A on compare 9Bit
 TCCR1B = (1<<WGM12);				// 9bit pwm
 TCCR1B = (1<<CS10)|(1<<CS11);		// clk/64  8Mhz/64 = 125kHz    125kHz/9bit = 244Hz

 TIMSK1 |= (1<<OCIE1A);				// on compare to trigger ADC

 TIMSK1 |= (1<<TOIE1);				// for constans frequency systick timer

 OCR1A = 0;							// PWM = 0

}

//---------------------------- Init ADC ---------------------------------------------------


void InitADC(void){

	ADCSRA  |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // enable ADC, prescaller  8000000 hz / 64 = 125kHz
	ADMUX   |= (1<<REFS0);				  		 // reference AVCC
    DIDR0    = (1<<ADC0D);						  // disable digital input (1<<ADC0D)
}

//------------------------------ Init USART -----------------------------------------------

void InitUSART(void)
{
 UBRR0L  = BAUD_PRESCALE;
 UBRR0H  = BAUD_PRESCALE>>8;
 UCSR0B  |= USART_DIRECTION;
 UCSR0C  |= BIT_CONFIG;
}

//------------------------- Send character via usart --------------------------------------
void usart_putc(char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}
//------------------------ Send string via USART ----------------------------------------------

void usart_puts( char *str )
{
	while (*str) 	usart_putc(*str++);
}

//---------------------------- Send integer value via USART --------------------------------
void usart_integer(int val){
 char intbuff[8];
 usart_puts(itoa(val,intbuff,10));
}

//----------------------------- Get BEMF voltage ------------------------------------------

void adc_get_bemf(void)
{
unsigned char index;


	ADMUX &= 0xF8; 						// Clear multiplexer
	ADMUX |= 0x0; 						// Set multiplexer for Channel 0

	for (index=0; index < N_SAMPLES; index++){

		ADCSRA |= (1<<ADSC);				// Start conversion

		while (!(ADCSRA & (1<<ADIF))){}; 	// Wait for conversion end

		t_bemf[index] = ADCW;
	}

}
