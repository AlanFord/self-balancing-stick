# Arduino Shield for the Self-Balancing Stick

The Arduino firmware supports options for two different motor drivers - the Pololu Dual MAX14870 Motor Driver Shield (https://www.pololu.com/product/2519) and a custom shield that also uses the Maxim MAX14870 chips as motor drivers.

The Pololu motor driver shield was used in the initial testing stages to ensure the hardware and software worked together properly.  However, I found the Dupont headers to make a less-than-satisfactory connection when moving parts are involved.  Hence the move to a custom design.

The design for the Arduino shield is intended to include motor drivers for two dc motors as well as the interface for a GY-521 Gyro.

The motor drivers each use a MAX14780 motor driver IC. A barrel jack (2.1 mm, center-positive) on the shield provides power for the motors (12V, 4.2A max).  Separate pin headers are made available for connecting the motors and the GY-521 gyro.  The motors are Pololu 4.4:1 gear motors witha 48 CPR encoder, requireing a 6-pin connector.  The GY-521 interface also requires a 6-pin connector.

The Arduino pins used by each of the connectors are as follows:


GY-521 -> Arduino

* SCL -> A5/SCL
* SDA -> A4/SDA
* INT -> A3


Left Motor -> Arduino

* Vcc -> +5V
* PWM -> D7
* DIR -> D8
* Encoder_A -> D2 (Interrrupt 0)
* Encoder_B -> A2


Right Motor -> Arduino

* Vcc -> +5V
* PWM -> D12
* DIR -> D13
* Encoder_A -> D3 (Interrupt 1)
* Encoder_B -> D6

The shield contains LEDs for both motors that indicate the direction the motor is being driven.

* Red LED -> DIR=1 (Clockwise)
* Green LED -> DIR=0 (Counterclockwise)
