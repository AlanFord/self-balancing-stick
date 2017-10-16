/***************************************************************************************************
 * Precision microdrives example for 103-001 back EMF measure
 *
 *  Author Marcin Pergol
 *   2014
 ****************************************************************************************/

#ifndef DYNO_H_
#define DYNO_H_

#define MOTOR 				PB1
#define MOTOR_OFF			PORTB&=~(1<<MOTOR)
#define MOTOR_ON			PORTB|=(1<<MOTOR)
#define MOTOR_TOGG			PORTB^=(1<<MOTOR)

#define MOTOR_PIN_LOW		!(PINB&MOTOR)

#define BAUD_PRESCALE 25 	//19200 8Mhz

#define USART_DIRECTION (1<<TXEN0)						// Enable TX
#define BIT_CONFIG		(1<<UCSZ01) | (1<<UCSZ00) 		// 8 bits, 1 stop bit, no parity

#define N_SAMPLES 5		// number of samples for bemf
#define N_LOST_SAMPLES 2	// number of lost samples until BEMF is stable

//*********************************************************************************************

extern volatile char Timer1; //software timer
extern int bemf;
extern unsigned int t_bemf[N_SAMPLES];

//*********************************************************************************************

void InitPorts(void);
void InitTimer1(void);
void InitINT0(void);
void InitADC(void);
void InitUSART(void);

void adc_get_bemf(void);
void usart_putc(char c);
void usart_puts(char *str);
void usart_integer(int val);


#endif /* DYNO_H_ */
