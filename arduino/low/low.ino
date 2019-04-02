/*
 * Test program for the Pololu VNH5019 Motor Shield
 * hooked up to a Pololu 12V gear motor with a 211.2 counts/rev encoder
 */


#include "DualVNH5019MotorShield.h"
#include <Encoder.h>
/*
 * board               encoder wire      purpose
 * M1A                 red               motor power (VCC?)
 * M1B                 black             motor power (ground?)
 * GND                                   motor power supply GND
 * VIN                                   motor power (12 V)
 * arduino GND         green             encoder GND
 * arduino VDD         blue              encoder Vcc (3.5 V - 20 V)
 * D3                    yellow            encoder A output
 * D5                    white             encoder B output
 * 
 * Other pins used by the driver board
 * D2        Motor 1 direction input A
 * D4        Motor 1 direction input B
 * D6        Motor 1 enable input/fault output
 * D7        Motor 2 direction input A
 * D8        Motor 2 direction input B
 * D9        Motor 1 speed input
 * D10       Motor 2 speed input
 * D12       Motor 2 enable input/fault output
 * A0        Motor 1 current sense output
 * A1        Motor 2 current sense output
 * 
 * Note: current sense is approximately equal to 140 mV per A.  The current sense is designed 
 *       for PWM frequencies of 5 kHz or higher.  Lower PWM frequencies should have a 1uF
 *       capacitor added between M1CS (A0) and GND.
 */
DualVNH5019MotorShield md;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
}

void loop()
{
  for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = 400; i >= -400; i--)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -400; i <= 0; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }

  for (int i = 0; i <= 400; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = 400; i >= -400; i--)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -400; i <= 0; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
}
