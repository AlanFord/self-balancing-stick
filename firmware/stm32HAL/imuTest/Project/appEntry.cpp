#include "main.h"
#include "tim.h"
//#include <string.h>
#include <cstdio>
#include "appEntry.hpp"
#include "MPU6050.h"

/*
 * Firmware to test the use of the MPU6050
 * References are:
 * https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
 * http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
 * https://www.i2cdevlib.com/forums/topic/27-fifo-overflow/
 */
int16_t ax, ay, az;
int16_t gx, gy, gz;
int32_t axSum, aySum, azSum;
int32_t gxSum, gySum, gzSum;

void app_entry(void) {
	//prepare for future "unbuffered" use of printf
	//setbuf(stdout, NULL);

	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);

	//initialize mpu6050
	MPU6050_Base mpu(&hi2c1, MPU6050_DEFAULT_ADDRESS);
	printf("Resetting I2C devices...\n");
	mpu.reset();
	HAL_Delay(100);
	printf("Initializing I2C devices...\n");
	mpu.initialize();
	mpu.setSleepEnabled(true); // thanks to Jack Elston for pointing this one out!
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

	// test the connection
	printf("Testing device connections...\n");
	printf(
			mpu.testConnection() ?
					"MPU6050 connection successful\n" :
					"MPU6050 connection failed\n"); // verify connection

	// use the code below to change accel/gyro offset values
	printf("Updating internal sensor offsets...\n");
	printf("%i\t",mpu.getXAccelOffset());
	printf("%i\t",mpu.getYAccelOffset());
	printf("%i\t",mpu.getZAccelOffset());
	printf("%i\t",mpu.getXGyroOffset());
	printf("%i\t",mpu.getYGyroOffset());
	printf("%i\n",mpu.getZGyroOffset());
	mpu.setXAccelOffset(__REVSH(-3405));
	mpu.setYAccelOffset(__REVSH(339));
	mpu.setZAccelOffset(__REVSH(1473));
	mpu.setXGyroOffset(__REVSH(224));
	mpu.setYGyroOffset(__REVSH(98));
	mpu.setZGyroOffset(__REVSH(24));
	printf("%i\t",mpu.getXAccelOffset());
	printf("%i\t",mpu.getYAccelOffset());
	printf("%i\t",mpu.getZAccelOffset());
	printf("%i\t",mpu.getXGyroOffset());
	printf("%i\t",mpu.getYGyroOffset());
	printf("%i\n",mpu.getZGyroOffset());
//	float gyro_scale = 1;
//	printf("Gyro range set to: ");
//	switch (mpu.getFullScaleGyroRange()) {
//	case MPU6050_GYRO_FS_250:
//		gyro_scale = 131;
//		printf("+- 250 deg/s\n");
//		break;
//	case MPU6050_GYRO_FS_500:
//		gyro_scale = 65.5;
//		printf("+- 500 deg/s\n");
//		break;
//	case MPU6050_GYRO_FS_1000:
//		gyro_scale = 32.8;
//		printf("+- 1000 deg/s\n");
//		break;
//	case MPU6050_GYRO_FS_2000:
//		gyro_scale = 16.4;
//		printf("+- 2000 deg/s\n");
//		break;
//	}
//
//	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
//	HAL_Delay(100);
//	float accel_scale = 1;
//	printf("Accelerometer range set to: ");
//	switch (mpu.getFullScaleAccelRange()) {
//	case MPU6050_ACCEL_FS_2:
//		accel_scale = 16384;
//		printf("+-2G\n");
//		break;
//	case MPU6050_ACCEL_FS_4:
//		accel_scale = 8192;
//		printf("+-4G\n");
//		break;
//	case MPU6050_ACCEL_FS_8:
//		accel_scale = 4096;
//		printf("+-8G\n");
//		break;
//	case MPU6050_ACCEL_FS_16:
//		accel_scale = 2048;
//		printf("+-16G\n");
//		break;
//	}
//
//	mpu.setDLPFMode(MPU6050_DLPF_BW_5);
//	HAL_Delay(100);
//	printf("Filter bandwidth set to: ");
//	switch (mpu.getDLPFMode()) {
//	case MPU6050_DLPF_BW_256:
//		printf("256 Hz\n");
//		break;
//	case MPU6050_DLPF_BW_188:
//		printf("188 Hz\n");
//		break;
//	case MPU6050_DLPF_BW_98:
//		printf("98 Hz\n");
//		break;
//	case MPU6050_DLPF_BW_42:
//		printf("42 Hz\n");
//		break;
//	case MPU6050_DLPF_BW_20:
//		printf("20 Hz\n");
//		break;
//	case MPU6050_DLPF_BW_10:
//		printf("10 Hz\n");
//		break;
//	case MPU6050_DLPF_BW_5:
//		printf("5 Hz\n");
//		break;
//	}
//	// calibrate gyro
//	//printf("calibrating gyro\n");
//	//mpu.CalibrateGyro(6);
//	//printf("calibrating accel\n");
//	//mpu.CalibrateAccel(6);
//	//printf("printing offsets\n");
//	//mpu.PrintActiveOffsets();
//
//	int16_t ax, ay, az;
//	int16_t gx, gy, gz;
//	float   accX,          ///< Last reading's accelerometer X axis m/s^2
//			accY,          ///< Last reading's accelerometer Y axis m/s^2
//			accZ,          ///< Last reading's accelerometer Z axis m/s^2
//			gyroX,         ///< Last reading's gyro X axis in rad/s
//			gyroY,         ///< Last reading's gyro Y axis in rad/s
//			gyroZ;         ///< Last reading's gyro Z axis in rad/s
//	// read raw accel/gyro measurements from device
//	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//	/* Print out the values */
//	printf("Acceleration X: ");
//	printf("%i", ax);
//	printf(", Y: ");
//	printf("%i", ay);
//	printf(", Z: ");
//	printf("%i", az);
//	printf(" m/s^2\n");
////	// setup range dependent scaling
////	accX = ((float) ax) / accel_scale;
////	accY = ((float) ay) / accel_scale;
////	accZ = ((float) az) / accel_scale;
////	printf("Acceleration X: ");
////	printf("%f", accX);
////	printf(", Y: ");
////	printf("%f", accY);
////	printf(", Z: ");
////	printf("%f", accZ);
////	printf(" m/s^2\n");
//
//	printf("Rotation X: ");
//	printf("%i", gx);
//	printf(", Y: ");
//	printf("%i", gy);
//	printf(", Z: ");
//	printf("%i", gz);
//	printf(" rad/s\n");
////	// setup range dependent scaling
////	gyroX = ((float) gx) / gyro_scale;
////	gyroY = ((float) gy) / gyro_scale;
////	gyroZ = ((float) gz) / gyro_scale;
////	printf("Rotation X: ");
////	printf("%f", gyroX);
////	printf(", Y: ");
////	printf("%f", gyroY);
////	printf(", Z: ");
////	printf("%f", gyroZ);
////	printf(" rad/s\n");
//
//	printf("\n");
	/* let's check the status of the MPU6050
	 *
	 */
	printf(mpu.getAccelFIFOEnabled() ? "Acceleration is FIFO enabled\n" : "Acceleration is NOT FIFO enabled\n");
	printf(mpu.getIntEnabled() ? "Interrupt enabled\n" : "Interrupt NOT enabled\n");
	int myCounter = 0;
	axSum = aySum = azSum = 0;;
	gxSum = gySum = gzSum = 0;
	while (1) {
		//verify data is ready to be read
		if(mpu.getIntDataReadyStatus()){
			myCounter++;
			// read raw accel/gyro measurements from device
			mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			axSum += ax;
			aySum += ay;
			azSum += az;
			gxSum += gx;
			gySum += gy;
			gzSum += gz;
			// these methods (and a few others) are also available
			//accelgyro.getAcceleration(&ax, &ay, &az);
			//accelgyro.getRotation(&gx, &gy, &gz);
			if (myCounter >= 1000){
				// display tab-separated accel/gyro x/y/z values
				printf("a/g:\t");
				printf("%li\t",axSum/myCounter);
				printf("%li\t",aySum/myCounter);
				printf("%li\t",azSum/myCounter);
				printf("%li\t",gxSum/myCounter);
				printf("%li\t",gySum/myCounter);
				printf("%li\n",gzSum/myCounter);
				myCounter = 0;
				axSum = aySum = azSum = 0;;
				gxSum = gySum = gzSum = 0;
			}
		}
	}

}
