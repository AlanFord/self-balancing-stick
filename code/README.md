# Software

This software is derived from Mike Rouleau's work and is designed for the Arduino IDE.  The following packages are required for compilation:
* I2CDevLib - see https://www.i2cdevlib.com
* PinChangeInt - see https://github.com/GreyGnome/PinChangeInt
* digitalWriteFast - see https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

Note that the PinChangeInt library isn't being updated and should be replaced.  DigitalWriteFast may no longer
be needed (digital writes appear to have been sped up).  I2CDevLib has several components that will be needed -
install the whole whopping thing or add parts to Arduino/library as you find you need them.
