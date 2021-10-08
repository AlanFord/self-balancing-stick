///////////////////////////////////////////////////////////////////////////////
/// \file FIFO.h
///
///	\author Ronald Sousa @Opticalworm
///////////////////////////////////////////////////////////////////////////////

#ifndef  __FIFO_H__
#define __FIFO_H__

	void FIFO_Initialiser(void);
	uint32_t FIFO_CounnterBufferCount(void);
	uint_fast8_t FIFO_Write(uint8_t inputData);
	uint_fast8_t FIFO_Read(uint8_t *outputDataPointer);

#endif /* __FIFO_H__ */
