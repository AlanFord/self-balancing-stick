///////////////////////////////////////////////////////////////////////////////
/// \file Terminal.h
///
///	\author Ronald Sousa @Opticalworm
///////////////////////////////////////////////////////////////////////////////

#ifndef __TERMINAL_H__
#define __TERMINAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "common.h"

void Terminal_Init(void);
IO_RESULT Terminal_Process(void);
void DisplaySystemInformation(void);

#ifdef __cplusplus
}
#endif


#endif /* __TERMINAL_H__ */
