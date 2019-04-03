# Motor Testing

## Motor Constant
Using a new/different small brushed DC motor can be aggravating.  Specs for the motor can be missing
or of questionable accuracy.  The goal here is to physically test the motors and generate any missing
motor data.

Motor testing is documented in the LaTeX report Appendices.  Normally a motor torque constant would be
measured by using the motor to drive a known load and measure the current.  In this instance an attempt to
construct a Prony Brake to use in testing the motor was unsuccessful.  Instead, a basic set of data (V, I, and rotation speed)
can be used to determine the motor constants.

The Arduino program rpm.ino is used to measure the encoder output from the motor and calculate a velocity.
The program uses the Encoder library by Paul Stoffregen (see https://www.pjrc.com/teensy/td_libs_Encoder.html).
It can be installed from the Arduino IDE (Tools -> Manage Libraries ...).  Apply a known voltage to the motor,
measure the current, and read the velocity (counts/sec) from the Arduino using the Arduino IDE serial monitor.
The encoder output must be converted from counts/sec to RPM using the encoder coefficient (counts/rev).

The folder rpm contains the Arduino program.  The spreadsheet motorConstant.xlsx is used for data
averaging when determining the average encoder output.  

## Motor/Gearbox Friction Measurement

The friction in the motor and gearbox under load was measured by varying the voltage while measuring
the current and shaft speed (using rpm.ino).  The spreadsheet motorResistance.xlsx shows the data
reduction to determine both the Coulomb and viscous friction constants.
