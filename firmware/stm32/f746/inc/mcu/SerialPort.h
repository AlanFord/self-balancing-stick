///////////////////////////////////////////////////////////////////////////////
/// \file SerialPort.h
/// \author Alan Ford
///
/// \brief  define the serial interface
///////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__
    #include "../universal.h"

class SerialPort {
	static uint_fast8_t    IsWriteBusy(void);
public:
	static uint_fast8_t    IsSerialOpen(void);                                 ///< return the serial connection state
	static uint_fast8_t    Open(const uint32_t baudrate);                            ///< opens the serial connection
	static void            Close(void);                                        ///< closes serial connection
	static uint_fast8_t    SendByte(const uint8_t source);                           ///< send a single byte
	static int_fast8_t     DoesReceiveBufferHaveData(void);                    ///< return the state of the serial receive buffer
	static int_fast8_t     GetByte(uint8_t *destination);                     ///< get a single byte from the serial
	static uint_fast8_t    SendString(const char *source);                 ///< send a string that. The string should be terminated by null character
	static uint_fast8_t    SendArray(const uint8_t *source, uint32_t length); ///< send an array of data
};

#endif /* __SERIAL_PORT_H__ */

