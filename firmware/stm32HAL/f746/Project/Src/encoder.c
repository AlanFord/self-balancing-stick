/*
 * encoder.c
 *
 *  Created on: Dec 4, 2021
 *      Author: alan
 */
#include "main.h"
#include "tim.h"
#include "encoder.h"

// encoder_Tick_Resolution was 10 for Mikes version
#define   encoder_Tick_Resolution   1 //10                // Defines how many ticks to 'wait' until calculating encoder positions and time measurments. Higher values lead to decreased ISR times, but decreased encoder accuracy.
#define   encoder_Speed_Time_Limit  100                   // Defines how long to wait for a new tick before assuming wheel is stopped [ms]
#define   left_Speed_Filter         0.8
#define   right_Speed_Filter        0.8

/**
 * What the interrupt callback calculates and saves:
 * - the time of the last encoder interrupt in us
 * - the elapsed time between the last two interrupts
 * - the direction the encoder was moving at last interrupt
 * - While direction and time should be sufficient to subsequently determine velocity and acceleration,
 *      the absolute time is used to ensure we aren't moving so slowly that we have overrun the clock period
 *      between encoder interrupts (possible, but unlikely unless the motor is stopped, as we are using
 *      a 32-bit timer with a period of just over an hour)
 */
volatile unsigned long left_Time_Now = 0; // Time at current absolute tick change, [us]
volatile unsigned long right_Time_Now = 0; // Time at current absolute tick change, [us]
volatile unsigned long left_Time_Dif = 0;
volatile unsigned long right_Time_Dif = 0;
volatile int left_Encoder_Direction; // Encoder measured direction, [-1 or 1]
volatile int right_Encoder_Direction; // Encoder measured direction, [-1 or 1]
volatile long left_Abs_Tick_Count = 0;      // Absolute total of encoder ticks
volatile long right_Abs_Tick_Count = 0;     // Absolute total of encoder ticks

float encoder_Resolution_Size; // Encoder Revolution per each encrment in resolution * 100000, calculated in setup

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	static unsigned int left_Rel_Tick_Count = 0; // Encoder ticks untill resolution
	static unsigned int right_Rel_Tick_Count = 0; // Encoder ticks untill resolution

	int left_Encoder_Direction_Now; // Encoder measured direction, [-1 or 1]
	int left_Encoder_Direction_Prev; // Encoder measured direction, [-1 or 1]
	unsigned long left_Time_Prev; // Time at previous absolute tick change, [us]

	int right_Encoder_Direction_Now; // Encoder measured direction, [-1 or 1]
	int right_Encoder_Direction_Prev; // Encoder measured direction, [-1 or 1]
	unsigned long right_Time_Prev; // Time at previous absolute tick change, [us]

	if (htim->Instance == TIM3) {
		// Right Encoder
		right_Rel_Tick_Count++;
		if (right_Rel_Tick_Count >= encoder_Tick_Resolution) {

			//right_Encoder_Direction_Now = digitalReadFast (pin_right_EncoderB) ? -1 : +1;
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				right_Encoder_Direction_Now = -1;
			} else {
				//TODO: verify "UP" is +1 in old code
				right_Encoder_Direction_Now = +1;
			}

			///////////////
			// Not sure the following is necessary at lower PPR values from the Pololu motors.
			// Also, it might be easier to just check the accel/decel rate to determine if an invalid reversal has occurred
			//////////////
			right_Encoder_Direction = right_Encoder_Direction_Now;
			//    if (right_Encoder_Direction_Now == right_Encoder_Direction_Prev) {                // Needs to register to ticks in the same directions. Sometimes the encoder will incorretly read wrong direction for one tick.
			//      right_Encoder_Direction = right_Encoder_Direction_Now;
			//    }

			right_Encoder_Direction_Prev = right_Encoder_Direction_Now;
			right_Abs_Tick_Count += right_Encoder_Direction;

			right_Time_Prev = right_Time_Now;
			right_Time_Now = __MICROS();
			right_Time_Dif = right_Time_Now - right_Time_Prev;

			right_Rel_Tick_Count = 0;
		}
	} else if (htim->Instance == TIM4) {
		// Left Encoder
		left_Rel_Tick_Count++;
		if (left_Rel_Tick_Count >= encoder_Tick_Resolution) {

			//left_Encoder_Direction_Now = digitalReadFast (pin_left_EncoderB) ? -1 : +1;
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				left_Encoder_Direction_Now = -1;
			} else {
				//TODO: verify "UP" is +1 in old code
				left_Encoder_Direction_Now = +1;
			}

			///////////////
			// Not sure the following is necessary at lower PPR values from the Pololu motors.
			// Also, it might be easier to just check the accel/decel rate to determine if an invalid reversal has occurred
			//////////////
			left_Encoder_Direction = left_Encoder_Direction_Now;
			//    if (left_Encoder_Direction_Now == left_Encoder_Direction_Prev) {                     // Needs to register to ticks in the same directions. Sometimes the encoder will incorretly read wrong direction for one tick.
			//      left_Encoder_Direction = left_Encoder_Direction_Now;
			//    }

			left_Encoder_Direction_Prev = left_Encoder_Direction_Now;
			left_Abs_Tick_Count += left_Encoder_Direction;

			left_Time_Prev = left_Time_Now;
			left_Time_Now = __MICROS();
			left_Time_Dif = left_Time_Now - left_Time_Prev;

			left_Rel_Tick_Count = 0;
		}
	} else {
		//TODO: throw an error??
	}
}

void initialize_encoder(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
}

void deinitialize_encoder(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Stop_IT(htim, TIM_CHANNEL_ALL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////  SPEED  ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_left_Encoder_Speeds(float *speed, float *accel) {
	//static int left_Encoder_Direction = 0; // Encoder measured direction, [-1 or 1]
	//static unsigned long left_Time_Now = 0; // Time at current absolute tick change, [us]
	static unsigned long left_Time_Acc_Now = 0; // Time at current absolute tick change, [us]
	static float left_Speed_RPM = 0;

	//volatile unsigned int left_Rel_Tick_Count = 0; // Encoder ticks untill resolution
	//volatile long left_Abs_Tick_Count = 0;    // Absolute total of encoder ticks
	//volatile long left_Abs_Tick_Count_Prev = 0; // Previous Absolute total of encoder ticks
	//volatile int left_Encoder_Direction_Prev = 0; // Encoder measured direction, [-1 or 1]
	//volatile int left_Encoder_Direction_Now = 0; // Encoder measured direction, [-1 or 1]
	//volatile int left_Encoder_Direction_Debug = 0; // Encoder measured direction, [-1 or 1]
	//volatile unsigned long left_Time_Prev = 0; // Time at previous absolute tick change, [us]
	//volatile unsigned long left_Time_Dif = 0;
	//volatile unsigned long left_Time_Acc_Prev = 0; // Time at previous absolute tick change, [us]
	//unsigned long left_Time_Age = 0; // Time at current absolute tick change, [us]
	//unsigned long left_Time_Saved = 0; // Time at current absolute tick change, [us]

	//float left_Speed_RPM_Prev = 0;
	//float left_Speed_RPM_Alt = 0;
	//float left_Speed_RPM_Alt_Prev = 0;
	//float left_Acceleration = 0;
	//int left_Current_Max = 0;
	//int left_Current_Avg = 0;
	//long left_Current_Total = 0;
	//int left_Current;
	//int left_Voltage = 0;

	float left_Speed_RPM_Prev = left_Speed_RPM;
	//left_Speed_RPM_Alt_Prev = left_Speed_RPM_Alt;

	unsigned long left_Time_Saved = left_Time_Now;
	unsigned long left_Time_Age = __MICROS();

	int left_Encoder_Direction_Debug = left_Encoder_Direction;

	unsigned long left_Time_Acc_Prev = left_Time_Acc_Now;
	left_Time_Acc_Now = __MICROS();

	float left_Speed_RPM_Unfiltered;
	if ((left_Time_Age - left_Time_Saved)
			> (encoder_Speed_Time_Limit * 1000L)) { // If last time measurment was taken too long ago, it means wheel has stopped
		left_Speed_RPM = 0;
		left_Speed_RPM_Unfiltered = 0;
	} else {
		left_Speed_RPM_Unfiltered = encoder_Resolution_Size / left_Time_Dif * 60
				* left_Encoder_Direction_Debug;    // Speed measurement
	}

	long left_Abs_Tick_Count_Prev = left_Abs_Tick_Count;
	left_Speed_RPM = ((1 - left_Speed_Filter) * left_Speed_RPM_Unfiltered)
			+ (left_Speed_Filter * left_Speed_RPM_Prev);
	float left_Acceleration = (left_Speed_RPM - left_Speed_RPM_Prev)
			/ (left_Time_Acc_Now - left_Time_Acc_Prev) * 1000000;

	*speed = left_Speed_RPM;
	*accel = left_Acceleration;
}

void get_right_Encoder_Speeds(float *speed, float *accel) {
	//static int right_Encoder_Direction; // Encoder measured direction, [-1 or 1]
	//static unsigned long right_Time_Now = 0; // Time at current absolute tick change, [us]
	static unsigned long right_Time_Acc_Now = 0; // Time at current absolute tick change, [us]
	static float right_Speed_RPM = 0;

	//volatile unsigned int right_Rel_Tick_Count = 0; // Encoder ticks untill resolution
	//volatile long right_Abs_Tick_Count = 0;   // Absolute total of encoder ticks
	//volatile int right_Encoder_Direction_Prev = 0; // Encoder measured direction, [-1 or 1]
	//volatile int right_Encoder_Direction_Now = 0; // Encoder measured direction, [-1 or 1]
	//volatile unsigned long right_Time_Prev = 0; // Time at previous absolute tick change, [us]
	//volatile unsigned long right_Time_Dif;
	volatile unsigned long right_Time_Acc_Prev; // Time at previous absolute tick change, [us]
	unsigned long right_Time_Age; // Time at current absolute tick change, [us]
	static unsigned long right_Time_Saved = 0; // Time at current absolute tick change, [us]

	float right_Speed_RPM_Prev;
	float right_Acceleration;
	//int right_Current_Max = 0;
	//int right_Current_Avg = 0;
	//long right_Current_Total = 0;
	//int right_Current;
	//int right_Voltage = 0;

	right_Speed_RPM_Prev = right_Speed_RPM;
	right_Time_Acc_Prev = right_Time_Acc_Now;
	right_Time_Acc_Now = __MICROS();

	right_Time_Saved = right_Time_Now;
	right_Time_Age = __MICROS();

	if ((right_Time_Age - right_Time_Saved)
			> (encoder_Speed_Time_Limit * 1000L)) { // If last time measurment was taken too long ago, it means wheel has stopped
		right_Speed_RPM = 0;
	} else {
		right_Speed_RPM = encoder_Resolution_Size / right_Time_Dif * 60
				* right_Encoder_Direction;    // Speed measurement

	}

	right_Speed_RPM = ((1 - right_Speed_Filter) * right_Speed_RPM)
			+ (right_Speed_Filter * right_Speed_RPM_Prev);
	right_Acceleration = (right_Speed_RPM - right_Speed_RPM_Prev)
			/ (right_Time_Acc_Now - right_Time_Acc_Prev) * 1000000;

	*speed = right_Speed_RPM;
	*accel = right_Acceleration;
}

