/////////////////////////////////////////////////////////////////////////
///	\file main.c
///	\brief This is the main program code.
///
///	\author Alan Ford
/////////////////////////////////////////////////////////////////////////
#include "universal.h"
#include "terminal.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


int main(void)
{
    Terminal_Init();

	while (1) {
		// Everything from here down should be non-blocking
    	Terminal_Process();
		// Do Stuff!
    }
}
