
#include "nodate.h"
#include "I2Cdev.h"
#include "imu.h"
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "MPU6050.h"

const uint8_t MPU6050_ADDR = 0xD0;

volatile bool I2C_wait = false;
volatile uint8_t I2C_byte = 0;

void imuCallback(uint8_t byte) {
	I2C_byte = byte;
	I2C_wait = false;
}


// define the I2C connections
I2C_devices I2C_device = I2C_1;
const uint8_t SCL_pin = 8;
const GPIO_ports SCL_port = GPIO_PORT_B;
const uint8_t SCL_af = 4;
const uint8_t SDA_pin = 9;
const GPIO_ports SDA_port = GPIO_PORT_B;
const uint8_t SDA_af = 4;

bool initialize_imu(void) {
	// start the I2C peripheral
	I2C::startI2C(I2C_device, SCL_port,  SCL_pin, SCL_af, SDA_port,  SDA_pin, SDA_af);
	// set the I2C peripheral to Master mode
	if (false == I2C::startMaster(I2C_device, I2C_MODE_FM, imuCallback)) {
		return false;
	};
	
	// set the I2C target address
	I2C::setSlaveTarget(I2C_device, MPU6050_ADDR);
	// initialize the I2Cdev library
	I2Cdev_init(&I2C_device);
	MPU6050_initialize();
	if (!MPU6050_testConnection()) return false;
	if (MPU6050_dmpInitialize() != 0) return false;                                                // load and configure the DMP

	// supply your own gyro offsets here, scaled for min sensitivity
	MPU6050_setXGyroOffset(233);
	MPU6050_setYGyroOffset(96);
	MPU6050_setZGyroOffset(18);
	MPU6050_setZAccelOffset(1434);                  // 1688 factory default for my test chip

	MPU6050_setDMPEnabled(true);                  // turn on the DMP, now that it's ready
	mpuIntStatus = MPU6050_getIntStatus();        // enable Arduino interrupt detection (Remove attachInterrupt function here)
	dmpReady = true;                          // set our DMP Ready flag so the main loop() function knows it's okay to use it
	packetSize = dmpGetFIFOPacketSize();  // get expected DMP packet size for later comparison

	printf("IMU Setup Complete");
	//TODO: move this after terminal initialization?
	MPU6050_resetFIFO();
	
	return true;
}
