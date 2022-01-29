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

void Terminal_Init(void);
int_fast8_t Terminal_Process(void);
void DisplaySystemInformation(void);

#ifdef __cplusplus
}
#endif


#endif /* __TERMINAL_H__ */
