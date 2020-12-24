# self-balancing-stick
# UPDATE:
With apologies, this project was clearly beyond my skill set.  However, I've made a lot of progress and am closing in on completion.  Many thanks to a couple of makerspaces - [HackRVA](https://www.hackrva.org/blog/) and [BuildRVA](https://buildrva.spaces.nexudus.com/en).

The Countdown:
* 10 - Study control system theory so as to verify that replacement motors (torques, speeds, etc.) can still work. A lot has changed in this area since I last studied it 45 years ago. (**Complete**)
* 9 - Determine new control parameters using said control system theory and MATLAB (love/hate relationship right there) (**Complete**)
* 8 - Learn Arduino programming (**Complete**)
* 7 - Learn to use machine tools such that I can fabricate parts ((**Complete**)
* 6 - Pause to build a claw machine (because who doesn't want a claw machine to pick up candy and plushies?) (**Complete**)
* 5 - Make the motor clamps from ABS on an X-Carve CNC router at HackRVA (**Complete**)
* 4 - Make the balancing point from acetal on a Sherline lathe at BuildRVA (**Complete**)
* 3 - Make the gyro mount from HDPE on a Grizzly manual lathe at BuildRVA (**Complete**)
* 2 - Make the motor mount block from aluminum on a Tormach 1100 CNC mill at BuildRVA (**Complete**)
* 1 - Fabricate the reaction wheels (**In Progress!**)
* Go! (In the Near Future)

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

