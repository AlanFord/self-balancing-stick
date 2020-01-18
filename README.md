# self-balancing-stick
Dual Axis Reaction Wheel Inverted Pendulum

This project is inspired by an earlier project by Mike Rouleau.  For details, read [Mike.md](../master/Mike.md).

My design looks similar to Mike's, but a lot of the components required substitutes
* Motors (unavailable) replaced with Pololu brushed DC motors and hubs
* Rotors modified to mount on the hubs
* Control system parameters modified to reflect new motor constants
* Motor drivers modified to match the motors
* Unique Arduino shield designed to support two motor drivers and a connector
for the gyro

My CAD work is being done in Fusion 360.  I haven't added those files to this repository yet, but in some cases drawings (in the form of PDFs) have been included.

Design of the control system is being done in Matlab and documented in LaTeX.  The
work can be found in the folders (you guessed it) matlab and latex.

The hardware is based on what I have at hand.  
* The shaft is a 1 ft length of 3003 Aluminum tube (McMaster Carr part number 7237K33)
* 4.4:1 Metal Gearmotor 25Dx48L mm MP 12V with 48 CPR Encoder (Pololu Part # 3237)
* Pololu Universal Aluminum Mounting Hub for 4mm Shaft, #4-40 Holes (2-Pack) (Pololu Part # 1081)
* Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 (Adafruit Part # 2472)
* Gy-521 MPU-6050 Module 3 Axis Analog Gyro and Accelerometer (various suppliers on Amazon)

