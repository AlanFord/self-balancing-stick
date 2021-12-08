#include "main.h"
#include "tim.h"
#include "appEntry.hpp"
#include "terminal.h"
#include "encoder.h"

void app_entry(void) {
	initialize_encoder (&htim4);
	initialize_encoder (&htim5);
	Terminal_Init();
	//initialize_imu();
	//initialize_left_motor();
	//initialize_right_motor();
	while (1) {
		// Everything from here down should be non-blocking
		Terminal_Process();
		//TODO: Do Stuff!
	}

}
