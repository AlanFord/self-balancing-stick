# Arduino library for the Pololu Dual VNH5019 Motor Driver Shield

Version: 3.0.0<br>
Release Date: 2018-04-11<br>
[![Build Status](https://travis-ci.org/pololu/dual-vnh5019-motor-shield.svg?branch=master)](https://travis-ci.org/pololu/dual-vnh5019-motor-shield)<br>
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that interfaces with the Pololu
[Dual VNH5019 Motor Driver Shield](https://www.pololu.com/catalog/product/2507).
It makes it simple to drive two brushed, DC motors.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.6.x
or later; we have not tested it with earlier versions.  This library
should support any Arduino-compatible board, including the
[Pololu A-Star 32U4 controllers](https://www.pololu.com/category/149/a-star-programmable-controllers).

## Getting started

### Hardware

The
[Dual VNH5019 Motor Driver Shield](https://www.pololu.com/product/2507)
can be purchased from Pololu's website.  Before continuing, careful
reading of the product page as well as the
[product user's guide](https://www.pololu.com/docs/0J49) is
recommended.

### Software

If you are using version 1.6.2 or later of the
[Arduino software (IDE)](https://www.arduino.cc/en/Main/Software), you can use
the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then
   "Manage Libraries...".
2. Search for "DualVNH5019MotorShield".
3. Click the DualVNH5019MotorShield entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the
   [latest release archive from GitHub](https://github.com/pololu/dual-vnh5019-motor-shield/releases)
   and decompress it.
2. Rename the folder "dual-vnh5019-motor-shield-xxxx" to "DualVNH5019MotorShield".
3. Drag the "DualVNH5019MotorShield" folder into the "libraries"
   directory inside your Arduino sketchbook directory.  You can view
   your sketchbook location by opening the "File" menu and selecting
   "Preferences" in the Arduino IDE.  If there is not already a
   "libraries" folder in that location, you should make the folder
   yourself.
4. After installing the library, restart the Arduino IDE.


## Example

An example sketch is available that shows how to use the library.  You
can access it from the Arduino IDE by opening the "File" menu,
selecting "Examples", and then selecting "DualVNH5019MotorShield".  If
you cannot find these examples, the library was probably installed
incorrectly and you should retry the installation instructions above.

### Demo

The demo ramps motor 1 from stopped to full speed forward, ramps down
to full speed reverse, and back to stopped.  Then, it does the same
with the other motor.  Current readings for each motor are sent over
serial and can be seen with the serial monitor.  If a fault is
detected, a message is sent over serial.

## Documentation

- `DualVNH5019MotorShield()`<br> Default constructor, selects the
  default pins as connected by the motor shield.
- `DualVNH5019MotorShield(unsigned char INA1, unsigned char INB1,
  unsigned char PWM1, unsigned char EN1DIAG1, unsigned char CS1,
  unsigned char INA2, unsigned char INB2, unsigned char PWM2, unsigned
  char EN2DIAG2, unsigned char CS2)` <br> Alternate constructor for
  shield connections remapped by user. If PWM1 and PWM2 are
  remapped, it will try to use analogWrite instead of timer1.
- `void init()` <br> Initialize pinModes and timer1.
- `void setM1Speed(int speed)` <br> Set speed and direction for motor 1.
  Speed should be between -400 and 400.  400 corresponds to motor
  current flowing from M1A to M1B.  -400 corresponds to motor current
  flowing from M1B to M1A.  0 corresponds to full coast.
- `void setM2Speed(int speed)` <br> Set speed and direction for motor 2.
  Speed should be between -400 and 400.  400 corresponds to motor
  current flowing from M2A to M2B.  -400 corresponds to motor current
  flowing from M2B to M2A.  0 corresponds to full coast.
- `void setSpeeds(int m1Speed, int m2Speed)` <br> Set speed and direction
  for motor 1 and 2.
- `void setM1Brake(int brake)` <br> Set brake for motor 1.  Brake should be
  between 0 and 400.  0 corresponds to full coast, and 400 corresponds
  to full brake.
- `void setM2Brake(int brake)` <br> Set brake for motor 2.  Brake should be
  between 0 and 400.  0 corresponds to full coast, and 400 corresponds
  to full brake.
- `void setBrakes(int m1Brake, int m2Brake)` <br> Set brake for motor 1 and
  2.
- `unsigned int getM1CurrentMilliamps()` <br> Returns current reading from
  motor 1 in milliamps.  See the notes in the "Current readings" section below.
- `unsigned int getM2CurrentMilliamps()` <br> Returns current reading from
  motor 2 in milliamps.  See the notes in the "Current readings" section below.
- `unsigned char getM1Fault()` <br> Returns 1 if there is a fault on motor
  driver 1, 0 if no fault.
- `unsigned char getM2Fault()` <br> Returns 1 if there is a fault on motor
  driver 2, 0 if no fault.

### Current readings

The current readings returned by `getM1CurrentMilliamps` and
`getM2CurrentMilliamps` will be noisy and unreliable if you are using
a PWM frequency below about 5&nbsp;kHz.  We expect these readings to
work fine if you haven't remapped the PWM pins and you are using a
board based on the ATmega168, ATmega328P, ATmega328PB, or ATmega32U4,
since this library uses 20&nbsp;kHz hardware PWM on those boards.

On other boards, this library uses `analogWrite` to generate PWM
signals, which usually means that the PWM frequency will be too low to
get reliable current measurements.  If `analogWrite` uses a frequency
of 490&nbsp;Hz or more on your board, you can add a 1&nbsp;&micro;F
(or larger) capacitor between each current sense line you are using
and GND.  To make `getM1CurrentMilliamps` work well, you would add the
capacitor between M1CS and GND.  To make `getM2CurrentMilliamps` work
well, you would add the capacitor between M2CS and GND.

## Version history
* 3.0.0 (2018-04-11): Allow PWM remapping (use analogWrite if PWM pins
  remapped).
* 2.0.0 (2016-08-16): Updated library to work with the Arduino Library Manager.
* 1.2.4 (2016-08-10): Added continuous integration testing. Thanks photodude.
* 1.2.3 (2014-03-24): Added 20 kHz PWM support for ATmega32U4. Thanks blacksound.
* 1.2.2 (2014-03-18): Add keywords.txt file. Thanks eatonphil.
* 1.2.1 (2013-01-06): Fixed a bug in setM2Speed that was introduced in 1.2.0.
* 1.2.0 (2012-12-26): Changes the behavior of the library at speed 0 so that it makes the motor coast regardless of which direction the motor is spinning.
* 1.1.0 (2011-12-15): Arduino 1.0 compatibility.
* 1.0.1 (2011-11-07): Adds support for Arduinos not based on ATmega168/328.
* 1.0.0 (2011-10-28): Original release.
