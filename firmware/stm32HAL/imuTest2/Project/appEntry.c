#include "main.h"
#include "tim.h"
//#include <string.h>
#include <stdio.h>
#include "appEntry.h"
#include "MPU6050.h"
#include "i2c.h"

/*
 * Firmware to test the use of the MPU6050
 *
 * The goal here is to emulate the Arduino MPU6050_raw sketch
 * References are:
 * https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
 * http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
 * https://www.i2cdevlib.com/forums/topic/27-fifo-overflow/
 */

int16_t ax, ay, az;
int16_t gx, gy, gz;
int32_t axSum, aySum, azSum;
int32_t gxSum, gySum, gzSum;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

void app_entry(void) {
	//prepare for future "unbuffered" use of printf
	//setbuf(stdout, NULL);

	//initialize microsecond timer
	HAL_TIM_Base_Start(&htim5);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    //accelgyro.initialize();
	//initialize mpu6050
    I2Cdev_init(&hi2c1); // init of i2cdevlib.
    MPU6050_setAddress(MPU6050_DEFAULT_ADDRESS);
    printf("Resetting I2C devices...\n");
    MPU6050_reset();
    HAL_Delay(100);
    printf("Initializing I2C devices...\n");
    MPU6050_initialize();
    MPU6050_setSleepEnabled(true); // thanks to Jack Elston for pointing this one out!
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    printf("Testing device connections...\n");
    printf(MPU6050_testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // use the code below to change accel/gyro offset values
    ///*
    printf("Updating internal sensor offsets...\n");
	printf("%i\t",MPU6050_getXAccelOffset());
	printf("%i\t",MPU6050_getYAccelOffset());
	printf("%i\t",MPU6050_getZAccelOffset());
	printf("%i\t",MPU6050_getXGyroOffset());
	printf("%i\t",MPU6050_getYGyroOffset());
	printf("%i\n",MPU6050_getZGyroOffset());
    MPU6050_setXAccelOffset(__REVSH(-3405));
    MPU6050_setYAccelOffset(__REVSH(339));
    MPU6050_setZAccelOffset(__REVSH(1473));
    MPU6050_setXGyroOffset(__REVSH(224));
    MPU6050_setYGyroOffset(__REVSH(98));
    MPU6050_setZGyroOffset(__REVSH(24));
	printf("%i\t",MPU6050_getXAccelOffset());
	printf("%i\t",MPU6050_getYAccelOffset());
	printf("%i\t",MPU6050_getZAccelOffset());
	printf("%i\t",MPU6050_getXGyroOffset());
	printf("%i\t",MPU6050_getYGyroOffset());
	printf("%i\n",MPU6050_getZGyroOffset());
	/*
    MPU6050_setXAccelOffset(-3426);
    MPU6050_setYAccelOffset(368);
    MPU6050_setZAccelOffset(1476);
    MPU6050_setXGyroOffset(224);
    MPU6050_setYGyroOffset(100);
    MPU6050_setZGyroOffset(28);
	*/

    /* let's check the status of the MPU6050
     *
     */
    printf(MPU6050_getAccelFIFOEnabled() ? "Acceleration is FIFO enabled\n" : "Acceleration is NOT FIFO enabled\n");
    printf(MPU6050_getIntEnabled() ? "Interrupt enabled\n" : "Interrupt NOT enabled\n");
    int myCounter = 0;
    axSum = aySum = azSum = 0;;
    gxSum = gySum = gzSum = 0;

	while (1) {
		//verify data is ready to be read
		if(MPU6050_getIntDataReadyStatus()){
			myCounter++;
			// read raw accel/gyro measurements from device
			MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
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

			// blink LED to indicate activity
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			//HAL_Delay(100);
		}
	}

}
