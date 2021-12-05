/*
 * encoder.c
 *
 *  Created on: Dec 4, 2021
 *      Author: alan
 */
#include "main.h"
#include "tim.h"

// encoder_Tick_Resolution was 10 for Mikes version
#define   encoder_Tick_Resolution   1 //10                // Defines how many ticks to 'wait' until calculating encoder positions and time measurments. Higher values lead to decreased ISR times, but decreased encoder accuracy.

volatile unsigned int   left_Rel_Tick_Count = 0;           // Encoder ticks untill resolution
volatile long           left_Abs_Tick_Count = 0;           // Absolute total of encoder ticks
volatile int            left_Encoder_Direction_Now = 0;    // Encoder measured direction, [-1 or 1]
volatile int            left_Encoder_Direction_Prev = 0;   // Encoder measured direction, [-1 or 1]
volatile int            left_Encoder_Direction = 0;        // Encoder measured direction, [-1 or 1]
volatile unsigned long  left_Time_Prev = 0;                // Time at previous absolute tick change, [us]
volatile unsigned long  left_Time_Now = 0;                 // Time at current absolute tick change, [us]
volatile unsigned long  left_Time_Dif = 0;

volatile unsigned int   right_Rel_Tick_Count = 0;          // Encoder ticks untill resolution
volatile long           right_Abs_Tick_Count = 0;          // Absolute total of encoder ticks
volatile int            right_Encoder_Direction_Now = 0;   // Encoder measured direction, [-1 or 1]
volatile int            right_Encoder_Direction_Prev = 0;  // Encoder measured direction, [-1 or 1]
volatile int            right_Encoder_Direction = 0;       // Encoder measured direction, [-1 or 1]
volatile unsigned long  right_Time_Prev = 0;               // Time at previous absolute tick change, [us]
volatile unsigned long  right_Time_Now = 0;                // Time at current absolute tick change, [us]
volatile unsigned long  right_Time_Dif = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3){
		// Right Encoder
		  right_Rel_Tick_Count++;
		  if (right_Rel_Tick_Count >= encoder_Tick_Resolution) {

		    //right_Encoder_Direction_Now = digitalReadFast (pin_right_EncoderB) ? -1 : +1;
			  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				  right_Encoder_Direction_Now = -1;
			  }
			  else {
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
	}
	else if (htim->Instance == TIM4){
		// Left Encoder
		  left_Rel_Tick_Count++;
		  if (left_Rel_Tick_Count >= encoder_Tick_Resolution) {

		    //left_Encoder_Direction_Now = digitalReadFast (pin_left_EncoderB) ? -1 : +1;
			  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				  left_Encoder_Direction_Now = -1;
			  }
			  else {
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
	}
	else {
		//TODO: throw an error??
	}
}

void initialize_encoder(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
}

void deinitialize_encoder(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Stop_IT(htim, TIM_CHANNEL_ALL);
}

