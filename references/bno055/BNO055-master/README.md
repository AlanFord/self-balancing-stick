BNO-055
=======

![bno055front](https://cloud.githubusercontent.com/assets/6698410/4856912/7c5917ac-60c5-11e4-9ae3-fc3aa26945e6.jpg)

Teensiduino sketch for 9-axis BNO-055 motion sensor + MS5637 pressure sensor add-on shield for Teensy 3.1 available from [Tindie.com](https://www.tindie.com/products/onehorse/bno-055-9-axis-motion-sensor-with-hardware-sensor-fusion/).

Basic sketch to configure the sensors and get scaled accelerometer, gyroscope, and magnetometer data from the BNO-055 9-axis motion sensor and MS5637 pressure sensor. 

For some reason the BNO-055 chip has software ID code 02.B0 when it should be running 03.04. Maybe these initial chips from Mouser are using a slightly outdated firmware or are engineering evaluation chips.

Added accel and gyro calibration, comparison of software and hardware sensor fusion results.

Added magnetometer calibration, system error checking, and output of linear acceleration and gravity.

To do is accel/gyro threshold interrupts for tap detection, free-fall detection, and simple gesture recognition.

The Kalman filter is rock solid on the BNO-055 compared to the open-source Madgwick sensor fusion; the heading is very stable but there is a difference between the hardware and open-source heading by about 180 degrees. This must have something to do with the orientation of the sensor axes using the default Windows orientation scheme. Maybe the result will be different using the Android scheme.

Work in progress...
