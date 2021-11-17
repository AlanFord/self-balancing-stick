The I2Cdev files were taken from the STM32HAL folder found at 
https://github.com/jrowberg/i2cdevlib/tree/master/STM32HAL

The I2Cdev files were converted from using the STM32HAL library to the Nodate library.

Conversion consisted of the following:
I2Cdev.h:
	- added an include statement for nodate.h
	- commented out the typedef for "bool, as it exists in C++
	- changed the declaration of I2Cdev_init to use the Nodate library

I2Cdev.cpp
	- changed the file suffix from ".c" to ".cpp" to use the Nodate library
	- changed the definition of I2Cdev_hi2c to use the Nodate library
	- changed the definition of I2Cdev_init to use the Nodate library
	- modified I2Cdev_readBytes to call Nodate library routines in lieu of using the HAL
	- modified I2Cdev_readWords to call Nodate library routines in lieu of using the HAL
	- modified I2Cdev_writeBytes to call Nodate library routines in lieu of using the HAL
	- modified I2Cdev_writeWords to call Nodate library routines in lieu of using the HAL


