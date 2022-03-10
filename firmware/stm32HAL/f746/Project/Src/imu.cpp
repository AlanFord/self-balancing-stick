/*
 * @file imu.cpp
 * @author Alan Ford
 * @date   22 feb 2022
 * @brief Class definition for an MPU6050 inertial measurement unit (IMU)
 *
 * This class definition encapulates the required functions of an MPU6050
 * gyroscope/accelerometer.  The class methods call library functions derived
 * from Jeff Rowberg's i2cdevlib library.
 *
 * The MPU6050 has a few peculiarities, chief among them is the
 * digital motion processor (dmp) buffer.  This buffer holds the periodic
 * results of yaw, pitch, and roll, updated approximately every 10 msec.
 * The buffer is a ring buffer, but the size of the ring buffer is not an
 * integral multiple of the data packet size (packet size of up to 42 bytes
 * with a FIFO buffer of 1024 bytes as shown in MPU6050_6Axis_motionApps20.cpp.
 * Thus, if the buffer overflows it will corrupt the oldest data in the buffer.
 *
 * @see https://github.com/jrowberg/i2cdevlib
 * @see https://www.fpaynter.com/2019/10/mpu6050-fifo-buffer-management-study/
 * @see https://github.com/ZHomeSlice/Simple_MPU6050
 */

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


#include "tim.h"
#include "imu.hpp"
#include <math.h>
#include <stdio.h>

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

const uint16_t angle_Rounding_Value = 1000; // Defines what value to multiple by before round theta and omega. Cuts down on noise, [10 = x.1,  100 = x.x1 , 1(Default)]
const float theta_Speed_Filter = 0.7;
const float omega_Speed_Filter = 0.7;
const float theta_Filter = 0.7;
const float omega_Filter = 0.7;


volatile uint16_t mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

/*
 * @brief external interrupt callback for gpio pins
 * @param[in] GPIO_Pin gpio pin number (0-15)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == IMU_INT_Pin) {
		mpuInterrupt = true;
	}
}

/*
 * @brief IMU constructor
 *
 * - runs the initialization function
 *   - sets the clock source
 *   - sets the full scale for accel and gyro
 *   - wakes up the imu
 * - Tests the I2C connection to the IMU
 * - initializes the dmp
 *   - resets the imu
 *   - turns on the fifo overflow and data interrupts
 * - Sets up the MPU-6050 gyro offsets
 * - gets the packetSize for this configuration
 *
 * @param[in] hi2c I2C HAL handle
 * @param[in] addrress imu I2C address
 */
IMU::IMU(I2C_HandleTypeDef *hi2c, uint8_t address) :
		mpu(hi2c, address),
		thetaFilter((float) theta_Filter),
		thetaSpeedFilter((float) theta_Speed_Filter),
//		thetaAverageFilter((float) angle_Average_Filter),
//		thetaSmoothedFilter((float) angle_Smoothed_Filter),
//		thetaZeroFilter((float) theta_Zero_Filter),
		omegaFilter((float) omega_Filter),
		omegaSpeedFilter((float) omega_Speed_Filter) {

	const int16_t xGyroOffset = 224;
	const int16_t yGyroOffset = 98;
	const int16_t zGyroOffset = 24;
	const int16_t xAccelOffset = -3405;
	const int16_t yAccelOffset = 339;
	const int16_t zAccelOffset = 1473;
/*
 *  The following aren't needed; mpu.dmpInitialize() is a substitute
 *	mpu.reset();
 *	HAL_Delay(100);
 */
	mpu.initialize();  // required to "wake up" the mpu from sleep mode
	printf(
			mpu.testConnection() ?
					"MPU6050 connection successful\n" :
					"MPU6050 connection failed\n"); // verify connection
	uint8_t devStatus = mpu.dmpInitialize();       // load and configure the DMP

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(xGyroOffset);
	mpu.setYGyroOffset(yGyroOffset);
	mpu.setZGyroOffset(zGyroOffset);
	mpu.setXAccelOffset(xAccelOffset);
	mpu.setYAccelOffset(yAccelOffset);
	mpu.setZAccelOffset(zAccelOffset);
	mpu.PrintActiveOffsets();

	if (devStatus == 0) {     // make sure it worked (devStatus returns 0 if so)
		mpu.setDMPEnabled(true);         // turn on the DMP, now that it's ready
		// mpu.getIntStatus(); // enable Arduino interrupt detection (Remove attachInterrupt function here)
		dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
		packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code ");
		printf("%u", devStatus);
		printf(")");
	}
}

/*
 * @brief Retrieves raw ypr values from the imu if data is ready
 * @param[out] ypr an array of floats representing yaw, pitch, and roll in RADIANS
 * @param[out] timestamp in microseconds
 * @return true if new data is found, false otherwise
 *
 */

bool IMU::update_ypr_values(float (&ypr)[3], uint32_t *timestamp){

	// if programming failed, don't try to do anything
	if (!dmpReady) {
		printf("Shit failed yo\n");
		return false;
	}
	// if no interrupt has been received, quit now
	if (!mpuInterrupt) {
		return false;
	}
	if (GetCurrentFIFOPacket(fifoBuffer, packetSize, timestamp)){
		mpuInterrupt = false;                                // reset interrupt flag
		Quaternion q;                   // [w, x, y, z]         quaternion container
		mpu.dmpGetQuaternion(&q, fifoBuffer);   // display YPR angles in degrees
		VectorFloat gravity;              // [x, y, z]            gravity vector
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		return true;
	} else {
		// tried to retrieve data too early or possible time-out
		return false;
	}
}


/*
 * @brief Converts pitch and roll into filtered values of omega and theta
 *
 * Theta values then calculated from the roll.  Omega value calculated from the pitch.
 * Theta is forward/backward angle (i.e pitch) in DEGREES, from -180 to +180.
 * Omega is left/right angle (i.e.roll) in DEGREES, from -180 to +180.
 *
 * @return true if updated data is available, false otherwise
 */
bool IMU::update_IMU_values(void) {
	float ypr[3]; // [yaw, pitch, roll] in radians
	static uint32_t imu_Time_Now = 0;

	// hold the last valid ypr timestamp before updating (to permit calculaton of velocities)
	unsigned long imu_Time_Prev = imu_Time_Now;

	if (!update_ypr_values(ypr, &imu_Time_Now)) {
		return false;
	}

	// time interval in microseconds;
	deltaTime = imu_Time_Now - imu_Time_Prev;

	////////////////////////////////////////////////////////// Theta Calcs//////////////////////////////////////////////////////////////////////////////////////////

	float theta_Prev = theta_Now;
	//FIXME: does the use of an angle_rounding_value make any sense when using floats?
	// convert the pitch for radians to degrees and apply a digital filter
	// FIXME : validate filtering
	float theta_Now_Unfiltered = round(
			(ypr[2] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value; //undo
	theta_Now = thetaFilter.filter(theta_Now_Unfiltered);

	// calculate the rotational velocity in degrees/sec using a digital filter
	// FIXME validate more filtering
	theta_Speed_Now = thetaSpeedFilter.filter((theta_Now - theta_Prev) / (imu_Time_Now - imu_Time_Prev) * 1000000.);

	//////////////////////////////////////////////////////////////// Omega Calcs//////////////////////////////////////////////////////////////////////////////////////
	float omega_Prev = omega_Now;
	float omega_Now_Unfiltered = round(
			(ypr[1] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value; //undo
	omega_Now = omegaFilter.filter(omega_Now_Unfiltered);

	// calculate the angular velocity
	omega_Speed_Now = omegaSpeedFilter.filter((omega_Now - omega_Prev) / (imu_Time_Now - imu_Time_Prev) * 1000000.);

	return true;
}

/*
 * @brief returns previously calculated angle (theta or omega) values
 *
 * @param[in] angle Denotes which axis to retrieve OMEGA or THETA
 * @param[out] angle_Now Current angle value in DEGREES
 * @param[out] angle_Speed_Now Angle rate of change, DEGREES/sec
 * @param[out] deltaTime elapsed time since last IMU update
 */
void IMU::get_values(imu_angle angle, float &angle_Now,
		float &angle_Speed_Now, uint32_t& deltaTime) {
	deltaTime = this->deltaTime;
	if (angle == THETA) {
		angle_Now = this->theta_Now;
		angle_Speed_Now = this->theta_Speed_Now;
	}
	else {  // must be OMEGA
		angle_Now = this->omega_Now;
		angle_Speed_Now = this->omega_Speed_Now;
	}
}

/**
 @brief Get latest byte from FIFO buffer no matter how much time has passed.
 @param[out] data Buffer for returned data
 @param[in]  length Length is bytes of data to be returned
 @param[out] timestamp Timestamp in microseconds
 @returns true if data is available, false if no valid data is availabl
*/
 bool IMU::GetCurrentFIFOPacket(uint8_t *data, uint8_t length, uint32_t *timestamp) { // overflow proof
	 int16_t fifoC;
	 // This section of code is for when we allowed more than 1 packet to be acquired
	 uint32_t BreakTimer = __MICROS();
	 do {
		 if ((fifoC = mpu.getFIFOCount())  > length) {

			 if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
				 mpu.resetFIFO(); // Fixes any overflow corruption
				 fifoC = 0;
				 //FIXME: eliminate this blocking issue
				 //printf("Blocked\n");
				 while (!(fifoC = mpu.getFIFOCount()) && ((__MICROS() - BreakTimer) <= (11000))); // Get Next New Packet
			 } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
				 uint8_t Trash[I2CDEVLIB_WIRE_BUFFER_LENGTH];
				 while ((fifoC = mpu.getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
					 fifoC = fifoC - length; // Save the last packet
					 uint16_t  RemoveBytes;
					 while (fifoC) { // fifo count will reach zero so this is safe
						 RemoveBytes = MIN((int)fifoC, I2CDEVLIB_WIRE_BUFFER_LENGTH); // Buffer Length is different than the packet length this will efficiently clear the buffer
                        mpu.getFIFOBytes(Trash, (uint8_t)RemoveBytes);
						 fifoC -= RemoveBytes;
					 }
				 }
			 }
		 }
		 if (!fifoC) return false; // Called too early no data or we timed out after FIFO Reset
		 // We have 1 packet
		 if ((__MICROS() - BreakTimer) > (11000)) return false;
	 } while (fifoC != length);
	 // keep the following two lines together so the timestamp is valid!
	 mpu.getFIFOBytes(data, length);     // read a packet from FIFO
	 *timestamp = __MICROS();
return true;
}

