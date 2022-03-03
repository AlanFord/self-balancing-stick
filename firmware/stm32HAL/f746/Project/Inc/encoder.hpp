/*
 * encoder.h
 *
 *  Created on: Dec 5, 2021
 *      Author: alan
 */
#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

//#include "common.h"
#include "tim.h"


//////////////////////////////////////////////////////////////////
//  Hardware Notes!!!
//
//  Note: See "hardware interface design notes.txt" for a derivation of encoder_PPR
// Encoder pulse per revolution (was 334 in Mike's version of the software)
#define   encoder_PPR               211.2
// encoder_Tick_Resolution was 10 for Mikes version
#define   encoder_Tick_Resolution   6 //10                // how many ticks to 'wait' until calculating encoder positions and time measurments. Higher values lead to decreased ISR times, but decreased encoder accuracy.
#define   encoder_Speed_Time_Limit  100                   // how long to wait for a new tick before assuming wheel is stopped [ms]
#define   Speed_Filter         0.8
//
//
//////////////////////////////////////////////////////////////////

class Encoder {
	float encoder_Resolution_Size;
	TIM_HandleTypeDef *htim;
	unsigned int Rel_Tick_Count = 0; // Encoder ticks until resolution
	unsigned long Time_Acc_Now = 0; // Time at current absolute tick change, [us]
	float Speed_RPM = 0;
	volatile unsigned long Time_Now = 0; // Time at current absolute tick change, [us]
	volatile unsigned long Time_Dif = 0;
	volatile int Encoder_Direction;     // Encoder measured direction, [-1 or 1]
	volatile long Abs_Tick_Count = 0;    // Absolute total of encoder ticks
public:
	friend void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
	Encoder(TIM_HandleTypeDef *htim);
	HAL_StatusTypeDef initEncoder();
	HAL_StatusTypeDef deinitEncoder();
	void get_Encoder_Speeds(float &speed, float &accel);
	unsigned long get_Time_Now() {
		return Time_Now;
	}
	unsigned long get_Time_Dif() {
		return Time_Dif;
	}
	int get_Encoder_Direction() {
		return Encoder_Direction;
	}
	long get_Abs_Tick_Count() {
		return Abs_Tick_Count;
	}

};

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	TIM_HandleTypeDef *htim;
	Encoder *encoder;
} EncoderCallbackData;

extern uint32_t callBackCount;
extern EncoderCallbackData callbackList[];

bool addCallback(TIM_HandleTypeDef *htim, Encoder *encoder);


#ifdef __cplusplus
}
#endif

#endif /* INC_ENCODER_HPP_ */
