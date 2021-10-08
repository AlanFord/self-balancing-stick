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
    #include "../common.h"

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief define the standard serial interface
    ///////////////////////////////////////////////////////////////////////////////
    typedef struct {
        uint_fast8_t    (*IsSerialOpen)(void);                                 ///< return the serial connection state
        uint_fast8_t    (*Open)(const uint32_t baudrate);                            ///< opens the serial connection
        void            (*Close)(void);                                        ///< closes serial connection
        uint_fast8_t    (*SendByte)(const uint8_t source);                           ///< send a single byte
        uint_fast8_t    (*SendString) (const char *source);                 ///< send a string that. The string should be terminated by null character
        uint_fast8_t    (*SendArray) (const uint8_t *source, uint32_t length); ///< send an array of data
        int_fast8_t     (*DoesReceiveBufferHaveData)(void);                    ///< return the state of the serial receive buffer
        int_fast8_t     (*GetByte) (uint8_t *destination);                     ///< get a single byte from the serial
    }SerialInterface;

#endif

