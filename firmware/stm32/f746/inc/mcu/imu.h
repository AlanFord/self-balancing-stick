////////////////////////////////////////////////////////////////////////////////
/// \file imu.h
/// Author: Alan Ford
////////////////////////////////////////////////////////////////////////////////

#ifndef __IMU_H__
#define __IMU_H__
#include "universal.h"

//For IMU initialization, see:
// https://controllerstech.com/how-to-interface-mpu6050-gy-521-with-stm32/
#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C



class Imu {
	I2C_devices I2C_device = I2C_1;
	uint8_t I2C_address;
	uint8_t SCL_pin = 8;
	uint8_t SCL_port = GPIO_PORT_B;
	Uint8_t SCL_af = 4;
	uint8_t SDA_pin = 9;
	uint8_t SDA_port = GPIO_PORT_B;
	uint8_t SDA_af 4;
	bool initialized = false;
	
	
	pin_IMU_Interrrupt;  //rising, dmpDataReady
	imu_initialize();
	imu_initialize_dmp();
public:
	Imu();
	get_values();
}






#endif /* __IMU_H__ */


