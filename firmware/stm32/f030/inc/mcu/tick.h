////////////////////////////////////////////////////////////////////////////////
/// \file tick.h
/// Author: Ronald Sousa (@Opticalworm)
////////////////////////////////////////////////////////////////////////////////

#ifndef __TICK_H__
#define __TICK_H__
#include "common.h"


    /////////////////////////////////////////////////////////////////////////
    /// \brief defines a non-blocking delay data type.
    /////////////////////////////////////////////////////////////////////////
    typedef struct {
        uint32_t StartMs;       ///< do not modify directly. Use Tick_DelayMs_NonBlocking
        uint32_t DelayMs;       ///< Set the desire delay
    } TickType;


    void Tick_init(void);
    uint32_t Tick_GetMs(void);
    int_fast8_t Tick_DelayMs_NonBlocking(uint_fast8_t reset, TickType * config);
    void Tick_DelayMs(uint32_t delayMs);

#endif
