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
    printf("Initializing I2C devices...\n");
    MPU6050_initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    printf("Testing device connections...\n");
    printf(MPU6050_testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // use the code below to change accel/gyro offset values
    ///*
    printf("Updating internal sensor offsets...\n");
    // -76	-2359	1688	0	0	0
	printf("%i\t",MPU6050_getXAccelOffset());
	printf("%i\t",MPU6050_getYAccelOffset());
	printf("%i\t",MPU6050_getZAccelOffset());
	printf("%i\t",MPU6050_getXGyroOffset());
	printf("%i\t",MPU6050_getYGyroOffset());
	printf("%i\n",MPU6050_getZGyroOffset());
	/*
    MPU6050_setXGyroOffset(224);
    MPU6050_setYGyroOffset(100);
    MPU6050_setZGyroOffset(28);
    MPU6050_setXAccelOffset(-3426);
    MPU6050_setYAccelOffset(368);
    MPU6050_setZAccelOffset(1476);
	printf("%i\t",MPU6050_getXAccelOffset());
	printf("%i\t",MPU6050_getYAccelOffset());
	printf("%i\t",MPU6050_getZAccelOffset());
	printf("%i\t",MPU6050_getXGyroOffset());
	printf("%i\t",MPU6050_getYGyroOffset());
	printf("%i\n",MPU6050_getZGyroOffset());
    */

    /* let's check the status of the MPU6050
     *
     */
    printf(MPU6050_getAccelFIFOEnabled() ? "Acceleration is FIFO enabled\n" : "Acceleration is NOT FIFO enabled\n");
    printf(MPU6050_getIntEnabled() ? "Interrupt enabled\n" : "Interrupt NOT enabled\n");
    int myCounter = 0;
	while (1) {
		myCounter++;
		//verify data is ready to be read
		if(MPU6050_getIntDataReadyStatus()){
			// read raw accel/gyro measurements from device
			MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

			// these methods (and a few others) are also available
			//accelgyro.getAcceleration(&ax, &ay, &az);
			//accelgyro.getRotation(&gx, &gy, &gz);

			#ifdef OUTPUT_READABLE_ACCELGYRO
				printf("myCounter: %i\n",myCounter);
				myCounter = 0;
				// display tab-separated accel/gyro x/y/z values
				printf("a/g:\t");
				printf("%i\t",ax);
				printf("%i\t",ay);
				printf("%i\t",az);
				printf("%i\t",gx);
				printf("%i\t",gy);
				printf("%i\n",gz);
			#endif

			#ifdef OUTPUT_BINARY_ACCELGYRO
				Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
				Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
				Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
				Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
				Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
				Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
			#endif

			// blink LED to indicate activity
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			//HAL_Delay(100);
		}
	}

}
