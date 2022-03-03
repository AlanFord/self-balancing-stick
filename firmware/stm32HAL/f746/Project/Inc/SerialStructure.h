///////////////////////////////////////////////////////////////////////////////
/// \file SerialStructure.h
/// \author Ronald Sousa
/// \website www.HashDefineElectronics.com
/// \company Hash Define Electronics Ltd
///
/// \brief  define the serial interface layer structure
///////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_INTERFACE_STRUCTURE_H__
#define __SERIAL_INTERFACE_STRUCTURE_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "common.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief define the standard serial interface
///////////////////////////////////////////////////////////////////////////////
typedef struct {
	IO_RESULT (*IsSerialOpen)(void);  ///< return the serial connection state
	IO_RESULT (*Open)(void);                 ///< opens the serial connection
	void (*Close)(void);                           ///< closes serial connection
	IO_RESULT (*SendByte)(const uint8_t source);      ///< send a single byte
	IO_RESULT (*SendString)(const char *source); ///< send a string that. The string should be terminated by null character
	IO_RESULT (*SendArray)(const uint8_t *source, uint32_t length); ///< send an array of data
	IO_RESULT (*DoesReceiveBufferHaveData)(void); ///< return the state of the serial receive buffer
	IO_RESULT (*GetByte)(uint8_t *destination); ///< get a single byte from the serial
} SerialInterface;

#ifdef __cplusplus
}
#endif


#endif

