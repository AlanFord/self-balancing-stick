///////////////////////////////////////////////////////////////////////////////
/// \file FIFO.c
///
///	\brief This is our FIFO library
///
///	\author Ronald Sousa @Opticalworm
///////////////////////////////////////////////////////////////////////////////
#include "common.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief Keeps track if the serial port is configure and open
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT IsInitializedFlag = IO_FALSE;

////////////////////////////////////////////////////////
///	\brief defines the fifo buffer max size.
////////////////////////////////////////////////////////
#define FIFO_BUFFER_MAX_SIZE 2000

////////////////////////////////////////////////////////
///	\brief Is our fifo buffer.
///
///	\note The fifo data type is unsigned 8bit
////////////////////////////////////////////////////////
static uint8_t Buffer[FIFO_BUFFER_MAX_SIZE];

////////////////////////////////////////////////////////
///	\brief keep track of the number of data in buffer
///
///	\sa Buffer[]
////////////////////////////////////////////////////////
static uint32_t BufferCurrentSize = 0;

////////////////////////////////////////////////////////
///	\brief keep track of the write buffer position
///
///	\sa Buffer[] BufferCurrentSize FIFO_BUFFER_MAX_SIZE
////////////////////////////////////////////////////////
static uint32_t WritePosition = 0;

////////////////////////////////////////////////////////
///	\brief keep track of the read buffer position
///
///	\sa Buffer[] BufferCurrentSize FIFO_BUFFER_MAX_SIZE
////////////////////////////////////////////////////////
static uint32_t ReadPosition = 0;

////////////////////////////////////////////////////////
///	\brief return the number of bytes in buffer
////////////////////////////////////////////////////////
uint32_t FIFO_CounnterBufferCount(void) {
	/// \FIXME Implement interrupt save code. ie enable and disable interrupt

	return BufferCurrentSize;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief return the serial open state
///
/// \return true = the serial port is open else false
///////////////////////////////////////////////////////////////////////////////
static IO_RESULT IsFifoInitialized(void) {
	return IsInitializedFlag;
}

////////////////////////////////////////////////////////
///	\brief This will init the fifo variables and
///	clear the buffer
///
////////////////////////////////////////////////////////
void FIFO_Initialize(void) {
	if (!IsInitializedFlag) {
		uint32_t BufferIndex;

		WritePosition = 0;
		ReadPosition = 0;
		BufferCurrentSize = 0;

		for (BufferIndex = 0; BufferIndex < FIFO_BUFFER_MAX_SIZE;
				BufferIndex++) {
			Buffer[BufferIndex] = 0;
		}
		IsInitializedFlag = IO_TRUE;
	}
}

////////////////////////////////////////////////////////
///	\brief This will deinit the fifo variables
///
////////////////////////////////////////////////////////
void FIFO_Deinitialize(void) {
	IsInitializedFlag = IO_FALSE;
}

////////////////////////////////////////////////////////
/// \brief Read one byte from the buffer. Return false
///	if we didn't.
///
///	\param outputDataPointer pointer to return the read value.
///
///	\return TRUE = successfully read a byte rom buffer
///			FALSE = no data to read
///			ERROR_INVALID_POINTER = Invalid outputDataPointer pointer
///
////////////////////////////////////////////////////////
IO_RESULT FIFO_Read(uint8_t *outputDataPointer) {
	IO_RESULT Result = IO_FALSE;

	// check pointer is valid and not set to zero
	if (outputDataPointer) {
		if (BufferCurrentSize > 0) {
			// we have data to read

			// Pass the data back
			*outputDataPointer = Buffer[ReadPosition];

			ReadPosition++;

			BufferCurrentSize--;

			if (ReadPosition == FIFO_BUFFER_MAX_SIZE) {
				ReadPosition = 0;
			}

			Result = IO_TRUE;
		}
	} else {
		// pointer error
		Result = ERROR_INVALID_POINTER;
	}

	return Result;
}

////////////////////////////////////////////////////////
///	\brief Write inputData into our buffer.
///
///	\param inputData copy of the data we want to store
///
///	\return IO_TRUE = successfully writing data to our buffer
///			IO_FALSE = No space in buffer
////////////////////////////////////////////////////////
IO_RESULT FIFO_Write(uint8_t inputData) {
	IO_RESULT Result = IO_FALSE;

	if (BufferCurrentSize < FIFO_BUFFER_MAX_SIZE) {
		// We have space

		Buffer[WritePosition] = inputData;

		BufferCurrentSize++;

		WritePosition++;

		// check to see if we need to reset the write position index counter.
		// this will ensure that the buffer is circulating
		if ( FIFO_BUFFER_MAX_SIZE == WritePosition) {
			WritePosition = 0;
		}

		Result = IO_TRUE;
	}

	return Result;
}

