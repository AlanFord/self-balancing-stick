/*
 * encoder.c
 *
 *  Created on: Dec 4, 2021
 *      Author: alan
 */
#include <encoder.hpp>

#define maxCallBacks 10
uint32_t callBackCount = 0;
EncoderCallbackData callbackList[maxCallBacks];

bool addCallback(TIM_HandleTypeDef *htim, Encoder *encoder) {
	if (callBackCount < maxCallBacks) {
		callbackList[callBackCount].htim = htim;
		callbackList[callBackCount].encoder = encoder;
		callBackCount++;
		return true;
	}
	return false;
}

// encoder_Tick_Resolution was 10 for Mikes version
#define   encoder_Tick_Resolution   1 //10                // Defines how many ticks to 'wait' until calculating encoder positions and time measurments. Higher values lead to decreased ISR times, but decreased encoder accuracy.
#define   encoder_Speed_Time_Limit  100                   // Defines how long to wait for a new tick before assuming wheel is stopped [ms]
#define   Speed_Filter         0.8

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
//volatile unsigned long left_Time_Now = 0; // Time at current absolute tick change, [us]
//volatile unsigned long right_Time_Now = 0; // Time at current absolute tick change, [us]
//volatile unsigned long left_Time_Dif = 0;
//volatile unsigned long right_Time_Dif = 0;
//volatile int left_Encoder_Direction; // Encoder measured direction, [-1 or 1]
//volatile int right_Encoder_Direction; // Encoder measured direction, [-1 or 1]
//volatile long left_Abs_Tick_Count = 0;      // Absolute total of encoder ticks
//volatile long right_Abs_Tick_Count = 0;     // Absolute total of encoder ticks
//float encoder_Resolution_Size; // Encoder Revolution per each encrment in resolution * 100000, calculated in setup
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// hurry up and get the data that can change!
	unsigned long grab_the_time = __MICROS();

	// determine which encoder to use
	Encoder *currentEncoder;
	for (uint32_t i = 0; i < callBackCount; i++) {
		if (callbackList[i].htim == htim) {
			currentEncoder = callbackList[i].encoder;
			break;
		}
	}

	int Encoder_Direction_Now; // Encoder measured direction, [-1 or 1]
	int Encoder_Direction_Prev; // Encoder measured direction, [-1 or 1]
	unsigned long Time_Prev; // Time at previous absolute tick change, [us]

	currentEncoder->Rel_Tick_Count++;
	if (currentEncoder->Rel_Tick_Count >= encoder_Tick_Resolution) {

		//right_Encoder_Direction_Now = digitalReadFast (pin_right_EncoderB) ? -1 : +1;
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
			Encoder_Direction_Now = -1;
		} else {
			//TODO: verify "UP" is +1 in old code
			Encoder_Direction_Now = +1;
		}

		///////////////
		// Not sure the following is necessary at lower PPR values from the Pololu motors.
		// Also, it might be easier to just check the accel/decel rate to determine if an invalid reversal has occurred
		//////////////
		currentEncoder->Encoder_Direction = Encoder_Direction_Now;
		//    if (right_Encoder_Direction_Now == right_Encoder_Direction_Prev) {                // Needs to register to ticks in the same directions. Sometimes the encoder will incorretly read wrong direction for one tick.
		//      right_Encoder_Direction = right_Encoder_Direction_Now;
		//    }

		Encoder_Direction_Prev = Encoder_Direction_Now;
		currentEncoder->Abs_Tick_Count += currentEncoder->Encoder_Direction;

		Time_Prev = currentEncoder->Time_Now;
		currentEncoder->Time_Now = grab_the_time;
		currentEncoder->Time_Dif = currentEncoder->Time_Now - Time_Prev;

		currentEncoder->Rel_Tick_Count = 0;
	}

}

Encoder::Encoder(TIM_HandleTypeDef *htim) {
	this->htim = htim;
	encoder_Resolution_Size =
			(encoder_Tick_Resolution * 1000000. / encoder_PPR);
}

HAL_StatusTypeDef Encoder::initEncoder() {
	return HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
}

HAL_StatusTypeDef Encoder::deinitEncoder() {
	return HAL_TIM_Encoder_Stop_IT(htim, TIM_CHANNEL_ALL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////  SPEED  ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Encoder::get_Encoder_Speeds(float &speed, float &accel) {
	//static int left_Encoder_Direction = 0; // Encoder measured direction, [-1 or 1]
	//static unsigned long left_Time_Now = 0; // Time at current absolute tick change, [us]

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

	float Speed_RPM_Prev = Speed_RPM;
	//left_Speed_RPM_Alt_Prev = left_Speed_RPM_Alt;

	unsigned long Time_Saved = Time_Now;
	unsigned long Time_Age = __MICROS();

	int Encoder_Direction_Debug = Encoder_Direction;

	unsigned long Time_Acc_Prev = Time_Acc_Now;
	Time_Acc_Now = __MICROS();

	float Speed_RPM_Unfiltered;
	if ((Time_Age - Time_Saved) > (encoder_Speed_Time_Limit * 1000L)) { // If last time measurment was taken too long ago, it means wheel has stopped
		Speed_RPM = 0;
		Speed_RPM_Unfiltered = 0;
	} else {
		Speed_RPM_Unfiltered = encoder_Resolution_Size / Time_Dif * 60
				* Encoder_Direction_Debug;    // Speed measurement
	}

	long Abs_Tick_Count_Prev = Abs_Tick_Count;
	Speed_RPM = ((1 - Speed_Filter) * Speed_RPM_Unfiltered)
			+ (Speed_Filter * Speed_RPM_Prev);
	float Acceleration = (Speed_RPM - Speed_RPM_Prev)
			/ (Time_Acc_Now - Time_Acc_Prev) * 1000000;

	speed = Speed_RPM;
	accel = Acceleration;
}

