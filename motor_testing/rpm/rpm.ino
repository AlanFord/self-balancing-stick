#include <Encoder.h>

// 25D motor wiring
// red - motor power
// black - motor power
// green - encoder GND
// blue - encoder VCC
// yellow - encoder A output
// white - encoder B output

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
 * motor red - power supply
 * motor black - power supply
 * motor green - gnd pin
 * motor blue - 5V
 * motor white - pin 2
 * motor yellow - pin 3
 */



Encoder myEnc(2, 3);
long newposition = 0;
long oldposition = 0;
unsigned long newtime= 0;
unsigned long oldtime = 0;
long vel;

void setup()
{
  Serial.begin(115200);
  Serial.println("DC motor encoder RPM");
}

void loop()
{
  getVelocity();
  //Serial.print("M1 new: ");
  //Serial.println(newposition);
  //Serial.print("M1 old: ");
  //Serial.println(oldposition);
  //Serial.print(" delta T:");
  //Serial.println(newtime-oldtime);
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
