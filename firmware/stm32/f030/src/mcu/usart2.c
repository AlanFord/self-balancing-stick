/////////////////////////////////////////////////////////////////////////
///	\file usart2.c
///	\brief STM32 serial2 MCU hardware interface layer. to maintain
///	code portability, the hardware related code is split from the main logic.
///
///	Author: Ronald Sousa (Opticalworm)
/////////////////////////////////////////////////////////////////////////
#include "MCU/usart2.h"
#include "FIFO.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief alternative function set bit 1 for AFR2
///////////////////////////////////////////////////////////////////////////////
#define GPIO_AFRL_AFR2_0 ((uint32_t) 0x00000100)

///////////////////////////////////////////////////////////////////////////////
/// \brief alternative function set bit 1 for AFR3
///////////////////////////////////////////////////////////////////////////////
#define GPIO_AFRL_AFR3_0 ((uint32_t) 0x00001000)

/////////////////////////////////////////////////////////////////////////
/// \brief enable 16x oversampling. Used to reduce the baudrate calculation
/// error.
/////////////////////////////////////////////////////////////////////////
//#define USART_OVER_SAMPLE_16

///////////////////////////////////////////////////////////////////////////////
/// \brief Keeps track if the serial port is configure and open
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t IsOpenFlag = FALSE;

///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial open state
///
/// \return true = the serial port is open else false
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t IsSerialOpen(void)
{
    return IsOpenFlag;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Open the serial port
///
/// \return true = success else port is already open
///////////////////////////////////////////////////////////////////////////////
static void Close(void)
{
	USART2->CR1 &= ~(USART_CR1_UE);
	NVIC_DisableIRQ(USART2_IRQn);
}

/////////////////////////////////////////////////////////////////////////
///	\brief	Set usart baudrate. can be called at any time.
///
///	\param baud the desire baudrate
///
///	\note setting baudrate will effect any data currently been sent.susb
///		make sure that you check that the write buffer is empty
/////////////////////////////////////////////////////////////////////////
static void Setbaudrate(const uint32_t baud)
{
	uint_fast8_t WasUartEnable = FALSE;

	uint16_t BaudrateTemp = 0;

	if (IsOpenFlag)
	{
		WasUartEnable = TRUE;
		Close();
	}

#ifdef USART_OVER_SAMPLE_16
	BaudrateTemp = (SystemCoreClock) / (baud);
#else
	BaudrateTemp = (2 * SystemCoreClock) / (baud);
	BaudrateTemp = ((BaudrateTemp & 0xFFFFFFF0) | ((BaudrateTemp >> 1) & 0x00000007));
#endif

	USART2->BRR = BaudrateTemp;


	if(WasUartEnable)
	{
		NVIC_EnableIRQ(USART2_IRQn); // enable interrupt
		USART2->CR1 |=  USART_CR1_UE;
	}

}

/////////////////////////////////////////////////////////////////////////
///	\brief	you can use this function to check if the write buffer is
///	empty and ready for new data
///
///	\return TRUE = Busy else ready. else false
/////////////////////////////////////////////////////////////////////////
static uint_fast8_t IsWriteBusy(void)
{
	if ( USART2->ISR & USART_ISR_TXE )
	{
		return FALSE;
	}

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Open the serial port.
///
/// \param baudrate set the serial port baud rate
///
/// \return true = success else port is already open
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t Open(const uint32_t baudrate)
{

	if(!IsOpenFlag)
	{
		// reset the FIFO
	    FIFO_Initialiser();

		RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // en USART clock

		USART2->CR1 = 0; // reset the usart CR1 register

		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // en GPIOA clock

		GPIOA->MODER &= (~(GPIO_MODER_MODER2) | ~(GPIO_MODER_MODER3)); // clear PA2 moder
		GPIOA->MODER  |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // Set PA2 to alter function

		GPIOA->AFR[0] &= (~(GPIO_AFRL_AFRL2) | ~(GPIO_AFRL_AFRL3)); // clear PA2 config
		GPIOA->AFR[0] |= GPIO_AFRL_AFR2_0 | GPIO_AFRL_AFR3_0;

		// set baudrate
		Setbaudrate(baudrate);

		NVIC_SetPriority(USART2_IRQn, 0); // set the USART to the highest interrupt priority

		NVIC_EnableIRQ(USART2_IRQn); 	// enable interrupt

		// enable interrupt for framing, overrun and noise;
		USART2->CR3 |= USART_CR3_EIE;

		// enable interrupt for PE and RX
		USART2->CR1 |= USART_CR1_PEIE | USART_CR1_RXNEIE;

		USART2->CR1 |= USART_CR1_OVER8 | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;



		IsOpenFlag = TRUE;
	}

	// This Setbaudrate is for testing if it does turn of and off the uart while is setting the baudrate.
	Setbaudrate(baudrate);

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Send a single byte
///
///	\param source the character to send via serial
///
/// \return true = success else port is not open
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t SendByte(const uint8_t source)
{
	if(IsOpenFlag)
	{
		while( IsWriteBusy() );

		USART2->TDR = source;

		return TRUE;
	}

    return FALSE;
}
///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial receive byte buffer state
///
/// \return      1 = we have data
///              0 = no data to read
///             ERROR = Port is not open
///////////////////////////////////////////////////////////////////////////////
static int_fast8_t DoesReceiveBufferHaveData(void)
{
	if(IsOpenFlag)
	{
		if(FIFO_CounnterBufferCount())
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
	}

    return ERROR;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief serial port for reading serial byte.
///
///	\note Data is still return even if corrupted. Make sure that you check the function return state.
///
/// \param destination pointer to return the newly read byte.
///
/// \return      	TRUE = success on reading a byte
///              	FALSE = no data to read or
///             	ERROR = Port is not open 	or the destination pointer is invalid
///												or if the data is corrupted (ie, framing error or buffer overflow)
///					ERROR_INVALID_POINTER Invalid pointer
///////////////////////////////////////////////////////////////////////////////
static int_fast8_t GetByte(uint8_t *destination)
{
	int_fast8_t Result = ERROR;

	if(IsOpenFlag)
	{
		Result = FIFO_Read(destination);
	}

	return Result;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief internal function for handling the RX interrupt routing
///////////////////////////////////////////////////////////////////////////////
static inline void InterruptRead(void)
{
	uint8_t DummyRead;

	if(USART2->ISR & USART_ISR_RXNE)
	{
		DummyRead = USART2->RDR;
		FIFO_Write(DummyRead);
	}

	if (USART2->ISR & USART_ISR_ORE)
	{
		USART2->ICR |= USART_ICR_ORECF;
	}

	if (USART2->ISR & USART_ISR_FE)
	{
		USART2->ICR |= USART_ICR_FECF;
	}

	if (USART2->ISR & USART_ISR_NE)
	{
		USART2->ICR |= USART_ICR_NCF;
	}

	if (USART2->ISR & USART_ISR_PE)
	{
		USART2->ICR |= USART_ICR_PECF;
	}
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Write a string
///
/// \param source pointer to the string to write. must end with null
///
/// \return true = success else either the port is not open or the pointer
/// to the array is invalid.
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t SendString(const char *source)
{
    if (IsOpenFlag && source)
    {
        while(*source)
        {
            if (!SendByte(*source) )
            {
                return FALSE;
            }
            source++;
        }
        return TRUE;
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Write an array of bytes
///
/// \param source pointer to the array to transmit.
/// \param length is the size of the array
///
/// \return true = success else either the port is not open or the pointer
/// to the array is invalid.
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t SendArray(const uint8_t *source, uint32_t length)
{
    if (IsOpenFlag && source)
    {
        for ( ; length ; length--)
        {
            if ( !SendByte(*source) )
            {
                return FALSE;
            }
            source++;
        }
        return TRUE;
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief the USART 2 interrupt handler.
///
///	\todo Need to implement the TX interrupt
///////////////////////////////////////////////////////////////////////////////
void USART2_IRQHandler(void)
{
	InterruptRead();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Defines the standard serial functions for usart 2
///
/// \sa SerialInterface
///////////////////////////////////////////////////////////////////////////////
SerialInterface SerialPort2 = {
                                    IsSerialOpen,
                                    Open,
                                    Close,
                                    SendByte,
                                    SendString,
                                    SendArray,
                                    DoesReceiveBufferHaveData,
                                    GetByte
                                };
