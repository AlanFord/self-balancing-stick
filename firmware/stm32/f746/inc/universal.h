////////////////////////////////////////////////////////////////////////////////
/// \file	common.h
///	\brief Holds all common code definitions
///
///	Author: Alan Ford
////////////////////////////////////////////////////////////////////////////////
#ifndef __COMMON_H__
#define __COMMON_H__
    #include <stdio.h>
    #include <stdlib.h>
	#include <stdint.h>
	//#include "stm32f0xx.h"

	enum U_return_codes {
		U_TRUE = 1,
		U_FALSE = 0,
		U_ERROR = -1,
		U_ERROR_INVALID_POINTER = -2
	};

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Firmware version
    /// D = development version of the firmware. Should only be used for testing purposes
    /// C = concession version. This version of the firmware is usual custom for a customer. see CONCESSION_NUMBER
    /// P = production version
    ///
    /// \sa CONCESSION_NUMBER
    ///////////////////////////////////////////////////////////////////////////////
    #define FIRMWARE_VERSION "00.0001D"

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Hardware version
    ///////////////////////////////////////////////////////////////////////////////
    #define HARDWARE_VERSION "00"

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Hardware version
    ///////////////////////////////////////////////////////////////////////////////
    #define COMPILED_DATA_TIME "[" __DATE__ " " __TIME__ "]"

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief Enables the debug interface and all debug message associated
    ///////////////////////////////////////////////////////////////////////////////
    #define EN_DEBUG_INTERFACE

    ///////////////////////////////////////////////////////////////////////////////
    /// \brief define the union type used to convert between types.
    ///////////////////////////////////////////////////////////////////////////////
    typedef union {
        long double d64_t;      ///< 64bit IEEE floating point number
        float       f32_t[2];   ///< 32bit IEEE float point number
        uint32_t    ui32_t[2];  ///< unsigned 32bit.
        int32_t     i32_t[2];   ///< signed 32bit.
        uint16_t    ui16_t[4];  ///< unsigned 16bit.
        int16_t     i16_t[4];   ///< signed 16bit.
        uint8_t     ui8_t[8];   ///< unsigned 8bit.
        int8_t      i8_t[8];    ///< singed 8bit.
    }DataConverter;

#endif /* __COMMON_H__ */
