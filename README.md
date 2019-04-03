# self-balancing-stick
Dual Axis Reaction Wheel Inverted Pendulum

This project is inspired by an earlier project by Mike Rouleau.  For details, read Mike.md.

My design looks similar to Mike's, but a lot of the components required substitutes
* Machined tube and end-point replaced with a knitting needle
* Motors (unavailable) replaced with Pololu brushed DC motors and hubs
* Rotors modified to mount on the hubs
* Control system parameters modified to reflect new motor constants
* Motor drivers modified to match the motors

Finally, I may substitute a different board for the Arduino.  The possibilities are
* Arduino (simple, cheap, somewhat slow)
* STM NUCLEO-L432KC Dev Board (Arduino compatibility, faster, real floating point math)
* PocketBeagle (Linux! - plus PRU controllers for realtime control)


CAD work is being done in Fusion 360.  I haven't added those files to this repository yet.

Design of the control system is being done in Matlab and documented in LaTeX.  The
work can be found in the folders (you guessed it) matlab and latex.

The hardware is based on what I have at hand.  
* 12" knitting needle
* 4.4:1 Metal Gearmotor 25Dx48L mm MP 12V with 48 CPR Encoder (Pololu Part # 3237)
* Pololu Universal Aluminum Mounting Hub for 4mm Shaft, #4-40 Holes (2-Pack) (Pololu Part # 1081)
* Pololu Dual VNH5019 Motor Driver Shield for Arduino (Pololu Part # 2507)
* STMicrolectronics NUCLEO-L432KC Dev Board (possibly?)
* Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 (Adafruit Part # 2472)
