#include "main.h"
#include "tim.h"
//#include <string.h>
#include <cstdio>
#include "MPU6050_6Axis_MotionApps20.h"
#include "appEntry.hpp"

float getAccelScalingFactor(MPU6050_6Axis_MotionApps20 &mpu);
float getGyroScalingFactor(MPU6050_6Axis_MotionApps20 &mpu);
void get_basic_data(MPU6050_6Axis_MotionApps20 &mpu, float gyro_scale, float accel_scale);
void get_dmp_data(MPU6050_6Axis_MotionApps20 &mpu, uint8_t mpuIntStatus);

/*
 * Firmware to test the use of the MPU6050
 * References are:
 * https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
 * http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
 * https://www.i2cdevlib.com/forums/topic/27-fifo-overflow/
 *
 * Offsets - The InvenSense Application Note "MPU Hardware Offset Registers Application Note"
 * provides information about the format of the accel and gyro offsets
 * Section 6.2 - The gyro registers at boot up will have a default value of 0.  The value of the bias
 * inputted needs to be in +-1000dps sensitivity range.  This means each 1 dps = 32.8 LSB
 * Section 7.2 - Initial values contain the OTP values of the Accel factory trim.  Therefore at
 * bootup there will be a non-zer value in these registers.  users will need to first read the register
 * and apply the biases to that value.  Format is in +-8G in which 1mg=4096 LSB.  Bit 0
 * on the low byte of each axis is a reserved bit and needs to be preserved.
 *
 */

#define _BV(n) (1 << n)
volatile uint16_t mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == IMU_INT_Pin) {
		mpuInterrupt = true;
	}
}

void app_entry(void) {
	//prepare for future "unbuffered" use of printf
	//setbuf(stdout, NULL);

	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);

	//initialize mpu6050
	MPU6050_6Axis_MotionApps20 mpu(&hi2c1, MPU6050_DEFAULT_ADDRESS);
	printf("Resetting I2C devices...\n");
	//mpu.reset();
	//HAL_Delay(100);
	printf("Initializing I2C devices...\n");
	if (!mpu.initialize()) {
		printf("mpu initialization Failed!\n");
		while (1){}
	}

	// test the connection
	printf("Testing device connections...\n");
	printf(
			mpu.testConnection() ?
					"MPU6050 connection successful\n" :
					"MPU6050 connection failed\n"); // verify connection

    // load and configure the DMP
	if (mpu.dmpInitialize()) {
		printf("dmp initialization Failed!\n");
		while (1){}
	}

	// change the gyro and accel ranges
	//mpu.setSleepEnabled(true); // thanks to Jack Elston for pointing this one out!
//	if (!mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000)) {
//		printf("setFullScaleGyroRange Failed!\n");
//		while (1){}
//	}
	/////////////////////////////////////
//	HAL_Delay(100);  // not sure this is necessary
//	if (!mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2)) {
//		printf("setFullScaleAccelRange Failed!\n");
//		while (1){}
//	}
//	HAL_Delay(100);  // not sure this is necessary

	// use the code below to change accel/gyro offset values
	printf("Updating internal sensor offsets...\n");
	mpu.PrintActiveOffsets();
	mpu.setXAccelOffset(-3405);
	mpu.setYAccelOffset(339);
	mpu.setZAccelOffset(1473);
	mpu.setXGyroOffset(224);
	mpu.setYGyroOffset(98);
	mpu.setZGyroOffset(24);
	// Calibration Time: generate offsets and calibrate our MPU6050
	//mpu.CalibrateAccel(6);
	//mpu.CalibrateGyro(6);
	mpu.PrintActiveOffsets();

	// set the gyro scaling factor
	float gyro_scale = getGyroScalingFactor(mpu);

	// set the accelerometer scaling factor
	float accel_scale = getAccelScalingFactor(mpu);
//
//	mpu.setDLPFMode(MPU6050_DLPF_BW_5);
//	HAL_Delay(100);
	printf("Filter bandwidth set to: ");
	switch (mpu.getDLPFMode()) {
	case MPU6050_DLPF_BW_256:
		printf("256 Hz\n");
		break;
	case MPU6050_DLPF_BW_188:
		printf("188 Hz\n");
		break;
	case MPU6050_DLPF_BW_98:
		printf("98 Hz\n");
		break;
	case MPU6050_DLPF_BW_42:
		printf("42 Hz\n");
		break;
	case MPU6050_DLPF_BW_20:
		printf("20 Hz\n");
		break;
	case MPU6050_DLPF_BW_10:
		printf("10 Hz\n");
		break;
	case MPU6050_DLPF_BW_5:
		printf("5 Hz\n");
		break;
	}

    // turn on the DMP, now that it's ready
    printf("Enabling DMP...\n");
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

	// let's check the status of the MPU6050
	printf(mpu.getAccelFIFOEnabled() ? "Acceleration is FIFO enabled\n" : "Acceleration is NOT FIFO enabled\n");
	printf(mpu.getIntEnabled() ? "Interrupt enabled\n" : "Interrupt NOT enabled\n");

	// get basic data
	get_basic_data(mpu, gyro_scale, accel_scale);
	//get_dmp_data(mpu);

	while (1) {
		// processing dmp data
		//if (mpuInterrupt && fifoCount < packetSize)
		if (mpuInterrupt)
		{
			mpuInterrupt = false;//reset interrupt flag
			get_dmp_data(mpu, packetSize);
		}
	}

}

void get_dmp_data(MPU6050_6Axis_MotionApps20 &mpu, uint8_t packetSize){

	// MPU control/status vars
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	uint8_t mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		//  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
		printf("FIFO overflow!\n");

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	{
		// read all available packets from FIFO
		while (fifoCount >= packetSize) // Lets catch up to NOW, in case someone is using the dreaded delay()!
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;
		}
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	    printf("ypr\t");
	    printf("%f\t",ypr[0] * 180/M_PI);
	    printf("%f\t",ypr[1] * 180/M_PI);
	    printf("%f\n",ypr[2] * 180/M_PI);
	}
}

void get_basic_data(MPU6050_6Axis_MotionApps20 &mpu, float gyro_scale, float accel_scale) {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int32_t axSum, aySum, azSum;
	int32_t gxSum, gySum, gzSum;
	axSum = aySum = azSum = 0;
	gxSum = gySum = gzSum = 0;
	const int maxcount = 1;
	for (int i=0; i<maxcount; ++i) {
		//verify data is ready to be read
		while (!mpu.getIntDataReadyStatus()){ }
		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		axSum += ax;
		aySum += ay;
		azSum += az;
		gxSum += gx;
		gySum += gy;
		gzSum += gz;
	}
	float accX, accY, accZ, gyroX, gyroY, gyroZ;
	// these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);

	// display tab-separated accel/gyro x/y/z values
	printf("a/g:\t");
	printf("%li\t",axSum/maxcount);
	printf("%li\t",aySum/maxcount);
	printf("%li\t",azSum/maxcount);
	printf("%li\t",gxSum/maxcount);
	printf("%li\t",gySum/maxcount);
	printf("%li\n",gzSum/maxcount);
	// setup range dependent scaling
	accX = ((float) axSum) /maxcount / accel_scale;
	accY = ((float) aySum) /maxcount / accel_scale;
	accZ = ((float) azSum) /maxcount / accel_scale;
	printf("Acceleration X: ");
	printf("%f", accX);
	printf(", Y: ");
	printf("%f", accY);
	printf(", Z: ");
	printf("%f", accZ);
	printf(" g's\n");
	// setup range dependent scaling
	gyroX = ((float) gxSum) /maxcount / gyro_scale;
	gyroY = ((float) gySum) /maxcount / gyro_scale;
	gyroZ = ((float) gzSum) /maxcount / gyro_scale;
	printf("Rotation X: ");
	printf("%f", gyroX);
	printf(", Y: ");
	printf("%f", gyroY);
	printf(", Z: ");
	printf("%f", gyroZ);
	printf(" deg/s\n");
}

float getAccelScalingFactor(MPU6050_6Axis_MotionApps20 &mpu) {
	float accel_scale = -1;
	printf("Accelerometer range set to: ");
	switch (mpu.getFullScaleAccelRange()) {
	case MPU6050_ACCEL_FS_2:
		accel_scale = 16384;
		printf("+-2G\n");
		break;
	case MPU6050_ACCEL_FS_4:
		accel_scale = 8192;
		printf("+-4G\n");
		break;
	case MPU6050_ACCEL_FS_8:
		accel_scale = 4096;
		printf("+-8G\n");
		break;
	case MPU6050_ACCEL_FS_16:
		accel_scale = 2048;
		printf("+-16G\n");
		break;
	default:
		printf("Failed to get accel scaling factor");
	}
	return accel_scale;
}

float getGyroScalingFactor(MPU6050_6Axis_MotionApps20 &mpu) {
	float gyro_scale = -1;
	printf("Gyro range set to: ");
	switch (mpu.getFullScaleGyroRange()) {
	case MPU6050_GYRO_FS_250:
		gyro_scale = 131;
		printf("+- 250 deg/s\n");
		break;
	case MPU6050_GYRO_FS_500:
		gyro_scale = 65.5;
		printf("+- 500 deg/s\n");
		break;
	case MPU6050_GYRO_FS_1000:
		gyro_scale = 32.8;
		printf("+- 1000 deg/s\n");
		break;
	case MPU6050_GYRO_FS_2000:
		gyro_scale = 16.4;
		printf("+- 2000 deg/s\n");
		break;
	default:
		printf("Failed to get gyro scaling factor");
	}
	return gyro_scale;
}
