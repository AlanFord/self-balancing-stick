The MPU6050 and I2Cdev libraries are modified versions of 
Jeff Rowberg's i2cdev library.  Specifically, modified versions
of files contained in the i2cdev library for Arduino, circa 12/1/2021.

The Arduino version of the library was chosen as a starting point so
that a C++ version of the library could be generated.  The stock STM32
and STM32HAL versions of these libraries in Jeff's original release
where written only in C, lacking the class structure available in the
Arduino version.

Modifying the Arduino versions of the libraries was not for the faint
of heart.  The Arduino language diverges from C++ in a number of ways,
all of which had to be patched.
