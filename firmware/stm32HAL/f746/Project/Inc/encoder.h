/*
 * encoder.h
 *
 *  Created on: Dec 5, 2021
 *      Author: alan
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

void initialize_encoder(TIM_HandleTypeDef *htim);
void deinitialize_encoder(TIM_HandleTypeDef *htim);
void get_right_Encoder_Speeds(float *speed, float *accel);
void get_left_Encoder_Speeds(float *speed, float *accel);


#endif /* INC_ENCODER_H_ */
