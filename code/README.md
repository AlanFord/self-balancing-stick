# Software

This software is derived from Mike Rouleau's work and is designed for the Arduino IDE.  The following packages are required for compilation:
* I2CDevLib - see https://www.i2cdevlib.com
* PinChangeInt - see https://github.com/GreyGnome/PinChangeInt
* digitalWriteFast - see https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

Note that the PinChangeInt library isn't being updated and should be replaced.  DigitalWriteFast may no longer
be needed (digital writes appear to have been sped up).  I2CDevLib has several components that will be needed -
install the whole whopping thing or add parts to Arduino/library as you find you need them.

Summary of Files:
Controller_3.0.ino
	This is really just a dummy file intended to contain the library include files and 
	Arduino pin assignments.  The normal Arduino functions (setup and loop) are located elsewhere.

\_1_Global_Variables.ino:
	Here we have a lot of #define statement used to define macros that will be used in the Arduino code.
	These are followed by other global variables and global volatile variables that may be changed while
	processing interrupts.

\_3_Setup.ino:
	Here we have the traditional setup() function that would normally be found in the Controller_3.0.ino file.
	setup() includes a number of actions -
	* initializes the serial output
	* sets the pin modes for the IMU interrupt input and motor controler outputs
	* sets the pull up on the IMU interrupt pin
	* attaches functions to the IMU and encoder interrupts
	* sets the PWM wavelength (?)
	* initializes the IMU

\_4_MainLoop.ino
	Here we have the traditional loop() function that would normally be found in the Controller_3.0.ino file.
	The loop() function is really just a container for a huge switch statement.  The switch variable (stat)
	is set from user input through the serial monitor.
		Code 	Character	Function
		127		Del 		Motors off, but print updates (?)
		121		y 			Charge left motor
		104		h 			Charge right motor
		110		n 			Charge both motors
		117		u 			Balance with left motor
		106		j 			Balance with right motor
		109		m			Balance with both motors

		122		z 			Change Zeros 
		Subcodes:
			122		z 		Set both zeros to current value	
			120		x		Set theta zeros to current value
			99		c       Set omega zeros to current value
			111		o 		Manually set theta zero
			108		l 		Manually set omega zero

		113		q 			set theta Kp
		119		w 			set theta Ki
		101		e 			set theta Kd
		114		r 			set theta Ks
		111		o 			set theta Kt
		97		a 			set omega Kp
		115		s 			set omega Ki
		100		d 			set omega Kd
		102		f 			set omega Ks
		108		l 			set omega Kt
		116		t 			set left Target Voltage
		103		g 			set right Target Voltage
		49		1			set the angle average filter
		50		2 			set the theta zero filter
		51		3 			set the omega zero filter
		52		4 			set the angle smoothed filter
		57		9			set the friction value
		112		p 			show extended data


\_5_Functions.ino
	Here we have most of the basic functions.
	The functions are:
	* get_IMU_values()
	* get_left_Encoder_Speeds()
	* get_right_Encoder_Speeds()
	* set_left_Motor_Voltage()
	* set_right_Motor_voltage()
	* update_Value() (prints current and changed value to the console)
	* get_Serial() (reads a new value from the console)

\_5_Interrupts.ino
	Timing is everything!  These are the functions called by the hardware interrupts.
	* ISR_left_Encoder()
	* ISR_right_Encoder()
	* dmpDataReady() (processes the IMU interrupt)


\_6_Controllers.ino
	This where the magic happens!  These implement the negative feedback for control.
	They don't really "get" values, rather they are calculating updated voltages based
	on the angles and speeds.
	* get_left_PID_Voltage_Value()
	* get_right_PID_Voltage_Value()

MPU6050_6Axis_MotionApps20_mod_1.h
	* MPU6050::dmpInitialize()
	* MPU6050::dmpPacketAvailable()
	* MPU6050::dmpGetAccel()
	* MPU6050::dmpGetQuaternion()
	* MPU6050::dmpGetGyro()
	* MPU6050::dmpGetLinearAccel()
	* MPU6050::dmpGetLinearAccelInWorld()
	* MPU6050::dmpGetGravity()
	* MPU6050::dmpGetEuler
	* MPU6050::dmpGetYawPitchRoll()
	* MPU6050::dmpProcessFIFOPacket()
	* MPU6050::dmpReadAndProcessFIFOPacket() 





