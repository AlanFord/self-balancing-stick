////////////////////////////////////////////////////////////////////////////////
/// \file serial.h
///	Author: Alan Ford
////////////////////////////////////////////////////////////////////////////////

#ifndef __SERIAL_H__
#define __SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "common.h"
#include "SerialStructure.h"

void InterruptRead(void);
extern SerialInterface SerialPort;

#ifdef __cplusplus
}
#endif


#endif
