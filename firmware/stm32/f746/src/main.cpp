/////////////////////////////////////////////////////////////////////////
///	\file main.c
///	\brief This is the main program code.
///
///	\author Alan Ford
/////////////////////////////////////////////////////////////////////////
#include "universal.h"
#include "terminal.h"
#include "imu.h"

int main(void)
{
    Terminal_Init();
	// initialize motor drivers
	// initialize interrupts for encoders
	// initialize interrupt for imu
	initialize_imu();

	while (1) {
		// Everything from here down should be non-blocking
    	Terminal_Process();
		// Do Stuff!
    }
}
