#include "DualVNH5019MotorShield.h"
#include <Encoder.h>

// 25D motor wiring
// red - motor power
// black - motor power
// green - encoder GND
// blue - encoder VCC
// yellow - encoder A output
// white - encoder B output

/*
 * VNH5019 motor shield uses the following pins by default:
 * 2 INA1 --> Motor #1 motor direction input A
 * 4 INB1 --> Motor #1 motor direction input B
 * 6 EN1DIAG1 --> Motor #1 diagnostic and enable line
 * 7 INA2 --> Motor #2 motor direction input A
 * 8 INB2 --> Motor #2 motor direction input B
 * 9 PWM1 --> Motor #1 PWM signal
 * 10 PWM2 --> Motor #2 PWM signal
 * 12 EN2DIAG2 --> Motor #2 diagnostic and enable line
 * A0 CS1 --> Motor #1 current sense
 * A1 CS2 --> Motor #2 current sense
 */

/*
 * Encoder library
 * #include <Encoder.h>
 * 
 * Change these two numbers to the pins connected to your encoder.
 *   Best Performance: both pins have interrupt capability
 *   Good Performance: only the first pin has interrupt capability
 *   Low Performance:  neither pin has interrupt capability
 * Encoder myEnc(5, 6);
 *   avoid using pins with LEDs attached
 * Interrupt Enabled Pins:
 * Arduino Uno: 2, 3
 * Mega, Mega2560, MegaAdK: 2,3,18,19,20,21
 */


 /*
  * Motor wiring
  * red - motor power
  * black - motor power
  * green - encoder GND
  * blue - encoder Vcc (3.5 - 20V)
  * Yellow - encoder A output
  * White - encoder B output
  */

/*
 * Hence, the connections
 * motor red - M1A
 * motor black - M1B
 * motor green - gnd pin
 * motor blue - 5V
 * motor yellow - pin 3
 * motor white - pin 5
 */



DualVNH5019MotorShield md;
Encoder myEnc(3, 5);
long newposition = 0;
long oldposition = 0;
unsigned long newtime= 0;
unsigned long oldtime = 0;
long vel;

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
  md.setM1Speed(400);  // out of +/- 400
  stopIfFault();
}

void loop()
{
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  getVelocity();
  Serial.print("M1 new: ");
  Serial.println(newposition);
  Serial.print("M1 old: ");
  Serial.println(oldposition);
  Serial.print(" delta T:");
  Serial.println(newtime-oldtime);
  Serial.print("M1 speed: ");
  Serial.println(vel);
  delay(1000);
}

void getVelocity()
{
  oldposition = newposition;
  oldtime = newtime;
  newposition = myEnc.read();
  newtime = millis();
  vel = (newposition-oldposition) * 1000 /(newtime-oldtime);
}
