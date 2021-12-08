///////////////////////////////////////////////////////////////////////////////
/// \file FIFO.h
///
///	\author Ronald Sousa @Opticalworm
///////////////////////////////////////////////////////////////////////////////

#ifndef  __FIFO_H__
#define __FIFO_H__

uint_fast8_t IsFifoInitialized(void);
void FIFO_Initialize(void);
void FIFO_Deinitialize(void);
uint32_t FIFO_CounnterBufferCount(void);
uint_fast8_t FIFO_Write(uint8_t inputData);
uint_fast8_t FIFO_Read(uint8_t *outputDataPointer);

#endif /* __FIFO_H__ */