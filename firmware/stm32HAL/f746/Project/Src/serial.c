/////////////////////////////////////////////////////////////////////////
///	\file serial.c
///	\brief STM32 serial2 MCU hardware interface layer. to maintain
///	code portability, the hardware related code is split from the main logic.
///
///	Author: Alan Ford
/////////////////////////////////////////////////////////////////////////
#include <serial.h>
#include "usart.h"
#include "FIFO.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief Keeps track if the serial port is configure and open
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT IsOpenFlag = IO_FALSE;

///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial open state
///
/// \return true = the serial port is open else false
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT IsSerialOpen(void) {
	return IsOpenFlag;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Close the serial port
///
///////////////////////////////////////////////////////////////////////////////
static void Close(void) {
	LL_USART_Disable(USART3);
	NVIC_DisableIRQ(USART3_IRQn);
	FIFO_Deinitialize();
	IsOpenFlag = IO_FALSE;
}

/////////////////////////////////////////////////////////////////////////
///	\brief	you can use this function to check if the write buffer is
///	empty and ready for new data
///
///	\return TRUE = Busy else false
/////////////////////////////////////////////////////////////////////////
static IO_RESULT IsWriteBusy(void) {
	if (LL_USART_IsActiveFlag_TXE(USART3)) {
		return IO_FALSE;
	}

	return IO_TRUE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Open the serial port.
///
/// Open results in the following:
///    - Initializing the FIFO read buffer
///    - Setting the baud rate in the huart struct
///    - Deinit the uart if previously init
///    - Init the uart
///
/// \param baudrate set the serial port baud rate
///
/// \return true = success else port is already open
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT Open(void) {

	if (!IsOpenFlag) {
		// reset the FIFO
		FIFO_Initialize();
		if (!LL_USART_IsEnabled(USART3)) {
			MX_USART3_UART_Init();
		}

		IsOpenFlag = IO_TRUE;
		LL_USART_EnableIT_RXNE(USART3);
		return IO_TRUE;
	}
	return IO_FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Send a single byte
///
///	\param source the character to send via serial
///
/// \return IO_TRUE = success else port is not open
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT SendByte(const uint8_t source) {
	if (IsOpenFlag) {
		while (IsWriteBusy())
			;

		LL_USART_TransmitData8(USART3, source);

		return IO_TRUE;
	}

	return IO_FALSE;
}
///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial receive byte buffer state
///
/// \return      IO_TRUE = we have data
///              IO_FALSE = no data to read
///              IO_ERROR = Port is not open
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT DoesReceiveBufferHaveData(void) {
	if (IsOpenFlag) {
		if (FIFO_CounnterBufferCount()) {
			return IO_TRUE;
		} else {
			return IO_FALSE;
		}
	}

	return IO_ERROR;
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
static IO_RESULT GetByte(uint8_t *destination) {
	IO_RESULT Result = IO_ERROR;

	if (IsOpenFlag == IO_TRUE) {
		Result = FIFO_Read(destination);
	}

	return Result;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief internal function for handling the RX interrupt routing
///////////////////////////////////////////////////////////////////////////////
inline void InterruptRead(void) {
	uint8_t DummyRead;

	if (LL_USART_IsActiveFlag_RXNE(USART3)) {
		DummyRead = LL_USART_ReceiveData8(USART3);
		FIFO_Write(DummyRead);
	}

	if (LL_USART_IsActiveFlag_ORE(USART3)) {
		LL_USART_ClearFlag_ORE(USART3);
	}

	if (LL_USART_IsActiveFlag_FE(USART3)) {
		LL_USART_ClearFlag_FE(USART3);
	}

	if (LL_USART_IsActiveFlag_NE(USART3)) {
		LL_USART_ClearFlag_NE(USART3);
	}

	if (LL_USART_IsActiveFlag_PE(USART3)) {
		LL_USART_ClearFlag_PE(USART3);
	}
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Write a string
///
/// \param source pointer to the string to write. must end with null
///
/// \return IO_TRUE = success else either the port is not open or the pointer
/// to the array is invalid.
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT SendString(const char *source) {
	if ((IsOpenFlag == IO_TRUE) && source) {
		while (*source) {
			if (!SendByte(*source)) {
				return IO_FALSE;
			}
			source++;
		}
		return IO_TRUE;
	}
	return IO_FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Write an array of bytes
///
/// \param source pointer to the array to transmit.
/// \param length is the size of the array
///
/// \return IO_TRUE = success else either the port is not open or the pointer
/// to the array is invalid.
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT SendArray(const uint8_t *source, uint32_t length) {
	if ((IsOpenFlag == IO_TRUE) && source) {
		for (; length; length--) {
			if (SendByte(*source) != IO_TRUE) {
				return IO_FALSE;
			}
			source++;
		}
		return IO_TRUE;
	}
	return IO_FALSE;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Defines the standard serial functions for usart 3
///
/// \sa SerialInterface
///////////////////////////////////////////////////////////////////////////////
SerialInterface SerialPort = { IsSerialOpen, Open, Close, SendByte, SendString,
		SendArray, DoesReceiveBufferHaveData, GetByte };
