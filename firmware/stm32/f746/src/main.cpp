/////////////////////////////////////////////////////////////////////////
///	\file main.c
///	\brief This is the main program code.
///
///	\author Alan Ford
/////////////////////////////////////////////////////////////////////////
#include "universal.h"
#include "terminal.h"


int main(void)
{
    Terminal_Init();

	while (1) {
		// Everything from here down should be non-blocking
    	Terminal_Process();
		// Do Stuff!
    }
}
