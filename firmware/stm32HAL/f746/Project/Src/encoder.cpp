/*
 * encoder.c
 *
 *  Created on: Dec 4, 2021
 *      Author: alan
 */
#include <encoder.hpp>

const uint32_t maxCallBacks = 10;
uint32_t callBackCount = 0;
EncoderCallbackData callbackList[maxCallBacks];

/*
 * @brief adds a timer handle and associated encoder object to the callback list
 * @param htim pointer to a timer handle
 * @encoder pointer to an encoder object
 * @return true if a collback list entry is made, false otherwise
 */
bool addCallback(TIM_HandleTypeDef *htim, Encoder *encoder) {
	if (callBackCount < maxCallBacks) {
		callbackList[callBackCount].htim = htim;
		callbackList[callBackCount].encoder = encoder;
		callBackCount++;
		return true;
	}
	return false;
}


/*
 * @brief callback handler for timers associated with encoder objects
 * @param htim pointer to a timer handle
 *
 * The timer periperal
 */
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
			//FIXME: verify "UP" is +1 in old code
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
/*
 * @brief initializes an encoder peripheral
 * @param a timer peripheral handle
 */
Encoder::Encoder(TIM_HandleTypeDef *htim) {
	this->htim = htim;
	encoder_Resolution_Size =
			(encoder_Tick_Resolution * 1000000. / encoder_PPR);
}

/*
 * @brief starts the encoder operation
 * @returns a HAL status
 */
HAL_StatusTypeDef Encoder::initEncoder() {
	return HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
}

/*
 * @brief stops the encoder operation
 * @returns a HAL status
 */
HAL_StatusTypeDef Encoder::deinitEncoder() {
	return HAL_TIM_Encoder_Stop_IT(htim, TIM_CHANNEL_ALL);
}

/*
 * @brief calculates the motor speed and acceleration
 * @param speed  motor speed in RPM
 * @param accel  motor acceleration in RPM/sec
 */
void Encoder::get_Encoder_Speeds(float &speed, float &accel) {
	float Speed_RPM_Prev = Speed_RPM;
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

