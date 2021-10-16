/////////////////////////////////////////////////////////////////////////
///	\file SerialPort.cpp
///	\brief STM32 serial port MCU hardware interface layer.
///
///	Author: Alan Ford
///		Derived from work by Ronald Sousa (Opticalworm)
/////////////////////////////////////////////////////////////////////////
#include "nodate.h"
#include "printf.h"
#include "mcu/SerialPort.h"
#include "fifo.h"

// nucleo-f746zg
#define ACTIVE_UART USART_3
#define UART_TX_PORT GPIO_PORT_D
#define UART_TX_PIN  8
#define UART_TX_AF   7
#define UART_RX_PORT GPIO_PORT_D
#define UART_RX_PIN  9
#define UART_RX_AF   7


///////////////////////////////////////////////////////////////////////////////
/// \brief Keeps track if the serial port is configure and open
///////////////////////////////////////////////////////////////////////////////
static uint_fast8_t IsOpenFlag = U_FALSE;

///////////////////////////////////////////////////////////////////////////////
/// \brief internal function for handling the RX interrupt routing
///////////////////////////////////////////////////////////////////////////////
static inline void InterruptRead(char ch) {
	FIFO_Write(ch);

}

/////////////////////////////////////////////////////////////////////////
///	\brief	you can use this function to check if the write buffer is
///	empty and ready for new data
///
///	\return TRUE = Busy else ready. else false
/////////////////////////////////////////////////////////////////////////
uint_fast8_t SerialPort::IsWriteBusy(void)
{
	if ( USART3->ISR & USART_ISR_TXE )
	{
		return U_FALSE;
	}

	return U_TRUE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial open state
///
/// \return true = the serial port is open else false
///////////////////////////////////////////////////////////////////////////////
uint_fast8_t SerialPort::IsSerialOpen(void)
{
    return IsOpenFlag;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Open the serial port.
///
/// \param baudrate set the serial port baud rate
///
/// \return true = success else port is already open
///////////////////////////////////////////////////////////////////////////////
uint_fast8_t SerialPort::Open(const uint32_t baudrate)
{

	if(!IsOpenFlag)
	{
		USART::startUart(ACTIVE_UART, UART_TX_PORT, UART_TX_PIN, UART_TX_AF, UART_RX_PORT, UART_RX_PIN, UART_RX_AF, baudrate, InterruptRead);
		IO::setStdOutTarget(USART_3);
		IsOpenFlag = U_TRUE;
	}

	return U_FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Open the serial port
///
/// \return true = success else port is already open
///////////////////////////////////////////////////////////////////////////////
void SerialPort::Close(void)
{
	//USART2->CR1 &= ~(USART_CR1_UE);
	//NVIC_DisableIRQ(USART2_IRQn);
	USART::stopUart(ACTIVE_UART);
	IsOpenFlag = U_FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Send a single byte
///
///	\param source the character to send via serial
///
/// \return true = success else port is not open
///////////////////////////////////////////////////////////////////////////////
uint_fast8_t SerialPort::SendByte(const uint8_t source)
{
	if(IsOpenFlag)
	{
		while( IsWriteBusy() );

		USART3->TDR = source;

		return U_TRUE;
	}

    return U_FALSE;
}
///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial receive byte buffer state
///
/// \return      1 = we have data
///              0 = no data to read
///             ERROR = Port is not open
///////////////////////////////////////////////////////////////////////////////
int_fast8_t SerialPort::DoesReceiveBufferHaveData(void)
{
	if(IsOpenFlag)
	{
		if(FIFO_CounnterBufferCount())
		{
			return U_TRUE;
		}
		else
		{
			return U_FALSE;
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
int_fast8_t SerialPort::GetByte(uint8_t *destination)
{
	int_fast8_t Result = ERROR;

	if(IsOpenFlag)
	{
		Result = FIFO_Read(destination);
	}

	return Result;
}


///////////////////////////////////////////////////////////////////////////////
/// \brief Write a string
///
/// \param source pointer to the string to write. must end with null
///
/// \return true = success else either the port is not open or the pointer
/// to the array is invalid.
///////////////////////////////////////////////////////////////////////////////
uint_fast8_t SerialPort::SendString(const char *source)
{
    if (IsOpenFlag && source)
    {
        while(*source)
        {
            if (!SendByte(*source) )
            {
                return U_FALSE;
            }
            source++;
        }
        return U_TRUE;
    }
    return U_FALSE;
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
uint_fast8_t SerialPort::SendArray(const uint8_t *source, uint32_t length)
{
    if (IsOpenFlag && source)
    {
        for ( ; length ; length--)
        {
            if ( !SendByte(*source) )
            {
                return U_FALSE;
            }
            source++;
        }
        return U_TRUE;
    }
    return U_FALSE;
}
