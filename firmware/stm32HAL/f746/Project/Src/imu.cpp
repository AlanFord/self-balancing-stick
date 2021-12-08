/*
 * imu.c
 *
 *  Created on: Dec 6, 2021
 *      Author: alan
 */

#include "common.h"
#include "tim.h"
#include "imu.hpp"
#include "MPU6050_6Axis_motionApps20.h"
#include <math.h>
#include <stdio.h>

#define   right_Speed_Filter        0.8
#define   encoder_Speed_Time_Limit  100                   // Defines how long to wait for a new tick before assuming wheel is stopped [ms]
#define   angle_Rounding_Value      1000.                  // Defines what value to multiple by before round theta and omega. Cuts down on noise, [10 = x.1,  100 = x.x1 , 1(Default)]
#define   theta_Filter              0.7
#define   theta_Integral_Max        3.0
#define   theta_Speed_Filter        0.7
float   angle_Average_Filter      = 0.970;
float   angle_Smoothed_Filter     = 0.997;

volatile uint16_t mpuInterrupt = FALSE; // indicates whether MPU interrupt pin has gone high
uint16_t dmpReady = FALSE;     // set true if DMP init was successful
uint16_t packetSize;           // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];       // FIFO storage buffer
MPU6050 mpu;                  // Creating object 'mpu', I think

// these are determined here but used by the controller
float           theta_Zero = 0;
float           theta_Now = 0;
float           theta_Integral = 0;
float           theta_Speed_Now = 0;
float           omega_Zero = 0;
float           omega_Now = 0;
float           omega_Integral = 0;
float           omega_Speed_Now = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == IMU_INT_Pin) {
		if (__HAL_GPIO_EXTI_GET_FLAG(IMU_INT_Pin)) {
			mpuInterrupt = TRUE;
		}
	}
}

void IMU_init() {
	// IMU Setup
	//Wire.begin();       // Join I2C bus
	//TWBR = twbr_Value;  // 12 = 400kHz I2C clock (200kHz if CPU is 8MHz)
	mpu.initialize();   // Initialize IMU

	printf(
			mpu.testConnection() ?
					"MPU6050 connection successful\n" :
					"MPU6050 connection failed\n"); // verify connection
	uint8_t devStatus = mpu.dmpInitialize();               // load and configure the DMP

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(233);
	mpu.setYGyroOffset(96);
	mpu.setZGyroOffset(18);
	mpu.setZAccelOffset(1434);          // 1688 factory default for my test chip

	if (devStatus == 0) {     // make sure it worked (devStatus returns 0 if so)
		mpu.setDMPEnabled(true);         // turn on the DMP, now that it's ready
		mpu.getIntStatus(); // enable Arduino interrupt detection (Remove attachInterrupt function here)
		dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
		packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code ");
		printf("%u",devStatus);
		printf(")");
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// IMU ////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Discussion of IMU values and motor orientation
//
//   -----------------------------------
//   |         Left Motor/Hub          |
//   |                                 |
//   -----------------------------------
//                  |        IMU top,front      ------
//                  |         -----------       |    |
//                  |         |         |       |    |
//              --------      |  Gy-521 |       |    |
//              |      |      |         |       |    |
//              |      |      -----------       |    |
//              |      |                        |    |
//              |      |      ----------------  |    | Right Motor/Hub
//              --------      |              |  |    |
//                            |              |==|    |
//                            |              |  |    |
//                            ----------------  |    |
//                                              |    |
//                                              |    |
//                                              |    |
//                                              |    |
//                                              |    |
//                                              ------
//
//      ypr[ ] values for pitch and roll are as follows
//      Front Edge Pitch Down:  Negative Values of ypr[1]
//      Front Edge Pitch Up:    Positive Values of ypr[1]
//      Roll to the Right:      Positive Values of ypr[2]
//      Roll to the Left:       Negative Values of ypr[2]
//
//      Note that "forward" motor rotations, associated with positive voltages,
//      are alway counter-clockwise when viewed from the "shaft end", and
//      clockwise when viewed from the motor body end.
//
//      Hence,
//      Positive Right Motor rotation ==> couter-clockwise shaft spin => pitch up => positive voltage ==> positive Omega ==> positive ypr[1]
//      Positive Left Motor rotation ==> couter-clockwise shaft spin  => roll to the right => positive voltage ==> positive Theta ==> positive ypr[2]
///////////////////////////////////////////////////////////////////////////////////////////////////

void get_IMU_Values() {
	Quaternion    q;                    // [w, x, y, z]         quaternion container
	VectorFloat   gravity;              // [x, y, z]            gravity vector
	float         ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	static int     p = 0;
	static int     p_Prev = 0;
	static float           theta_Error = 0;
	static float           omega_Error = 0;
	static float           theta_Smoothed = 0;
	static float           theta_Smoothed_Speed = 0;
	static float           omega_Smoothed = 0;
	static float           omega_Smoothed_Speed = 0;
	static unsigned long   imu_Time_Now = 0;
	static float           theta_Average = 0;




	///////////////////////////////////////////////////// IMU Comms ////////////////////////////////////////////////////////////////////////
	if (!dmpReady) {
		printf("Shit failed yo\n");
		return;               // if programming failed, don't try to do anything
	}

	uint32_t time_IMU_Prev = __MICROS();

	while (!mpuInterrupt) { // && fifoCount < packetSize) {       // wait for MPU interrupt or extra packet(s) available
		// other program behavior stuff here
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		//Serial.println("Waiting in IMU function");

	}

	uint32_t time_IMU_Dif = __MICROS() - time_IMU_Prev;

	mpuInterrupt = false;                                // reset interrupt flag
	uint8_t mpuIntStatus = mpu.getIntStatus();                    // get INT_STATUS byte
	uint16_t fifoCount = mpu.getFIFOCount();                    // get current FIFO count

	if ((mpuIntStatus & 0x10) || fifoCount >= 1024) { // check for overflow (this should never happen unless our code is too inefficient)
		mpu.resetFIFO();                     // reset so we can continue cleanly
		printf("FIFO overflow!\n");
	}

	else if (mpuIntStatus & 0x02) { // otherwise, check for DMP data ready interrupt (this should happen frequently)
		while (fifoCount < packetSize) {
			HAL_Delay(1); // wait for correct available data length, should be a VERY short wait
			fifoCount = mpu.getFIFOCount();
			mpuIntStatus = mpu.getIntStatus();
		}
		mpu.getFIFOBytes(fifoBuffer, packetSize);     // read a packet from FIFO
		fifoCount -= packetSize; // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
		mpu.dmpGetQuaternion(&q, fifoBuffer);   // display YPR angles in degrees
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}

	mpu.resetFIFO();      // ADDED BY ME!!!!!!!!!!!

	if (p == 1 && p_Prev == 0) { // Resets integral when balancing mode is started to avoid wide up while holding
		theta_Integral = 0;
		omega_Integral = 0;
		theta_Error = 0;
		omega_Error = 0;
		theta_Smoothed = theta_Now;
		omega_Smoothed = omega_Now;
		theta_Smoothed_Speed = 0;
		omega_Smoothed_Speed = 0;
	}

	p_Prev = p;
	unsigned long imu_Time_Prev = imu_Time_Now;
	imu_Time_Now = __MICROS();

	////////////////////////////////////////////////////////// Theta Calcs//////////////////////////////////////////////////////////////////////////////////////////

	float theta_Prev = theta_Now;
	float theta_Now_Unfiltered = round((ypr[2] * 180 / M_PI) * angle_Rounding_Value)
			/ angle_Rounding_Value;    //undo
	float theta_Now = (1 - theta_Filter) * (theta_Now_Unfiltered)
			+ (theta_Filter * theta_Prev); //undo

	theta_Error = theta_Now - theta_Zero;

	theta_Integral += theta_Error * (imu_Time_Now - imu_Time_Prev) / 1000000.0;
	if (theta_Integral > theta_Integral_Max) {
		theta_Integral = theta_Integral_Max;
	} else if (theta_Integral < -theta_Integral_Max) {
		theta_Integral = -theta_Integral_Max;
	}

	float theta_Speed_Prev = theta_Speed_Now;
	theta_Speed_Now = ((1 - theta_Speed_Filter) * (theta_Now - theta_Prev)
			/ (imu_Time_Now - imu_Time_Prev) * 1000000.)
			+ theta_Speed_Filter * theta_Speed_Prev; // Relative to absolute zero
	float theta_Average_Prev = theta_Average;
	theta_Average = (1 - angle_Average_Filter) * (theta_Now)
			+ (angle_Average_Filter * theta_Average_Prev);

	float theta_Smoothed_Prev = theta_Smoothed;
	theta_Smoothed = (1 - angle_Smoothed_Filter) * theta_Now
			+ angle_Smoothed_Filter * theta_Smoothed_Prev;
	theta_Smoothed_Speed = (theta_Smoothed - theta_Smoothed_Prev)
			/ (imu_Time_Now - imu_Time_Prev) * 1000000.;

	float theta_Zero_Prev = theta_Zero;
	if (p == 1) {                               // Automatic setpoint adjustment
		theta_Zero_Unfiltered = theta_Zero - theta_Kt * theta_Error
				- theta_Ktd * theta_Smoothed_Speed;
		theta_Zero = (1 - theta_Zero_Filter) * theta_Zero_Unfiltered
				+ theta_Zero_Filter * theta_Zero_Prev;
	} else {
		theta_Zero = theta_Average;
	}

	//////////////////////////////////////////////////////////////// Omega Calcs//////////////////////////////////////////////////////////////////////////////////////
	omega_Prev = omega_Now;
	//  omega_Now_Unfiltered = round((-ypr[1] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value;    //undo
	omega_Now_Unfiltered = round((ypr[1] * 180 / M_PI) * angle_Rounding_Value)
			/ angle_Rounding_Value;    //undo
	omega_Now = (1 - omega_Filter) * (omega_Now_Unfiltered)
			+ omega_Filter * (omega_Prev); //undo
	omega_Error = omega_Now - omega_Zero;

	omega_Error = omega_Now - omega_Zero;

	omega_Integral += omega_Error * (imu_Time_Now - imu_Time_Prev) / 1000000.0;
	if (omega_Integral > omega_Integral_Max) {
		omega_Integral = omega_Integral_Max;
	} else if (omega_Integral < -omega_Integral_Max) {
		omega_Integral = -omega_Integral_Max;
	}

	omega_Speed_Prev = omega_Speed_Now;
	omega_Speed_Now = ((1 - omega_Speed_Filter) * (omega_Now - omega_Prev)
			/ (imu_Time_Now - imu_Time_Prev) * 1000000.)
			+ omega_Speed_Filter * omega_Speed_Prev; // Relative to absolute zero
	omega_Average_Prev = omega_Average;
	omega_Average = (1 - angle_Average_Filter) * (omega_Now)
			+ (angle_Average_Filter * omega_Average_Prev);

	omega_Smoothed_Prev = omega_Smoothed;
	omega_Smoothed = (1 - angle_Smoothed_Filter) * omega_Now
			+ angle_Smoothed_Filter * omega_Smoothed_Prev;
	omega_Smoothed_Speed = (omega_Smoothed - omega_Smoothed_Prev)
			/ (imu_Time_Now - imu_Time_Prev) * 1000000.;

	omega_Zero_Prev = omega_Zero;
	if (p == 1) {                               // Automatic setpoint adjustment
		omega_Zero_Unfiltered = omega_Zero - omega_Kt * omega_Error
				- omega_Ktd * omega_Smoothed_Speed;
		omega_Zero = (1 - omega_Zero_Filter) * omega_Zero_Unfiltered
				+ omega_Zero_Filter * omega_Zero_Prev;
	} else {
		omega_Zero = omega_Average;
	}

}

}
