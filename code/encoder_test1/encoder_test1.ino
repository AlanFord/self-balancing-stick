//////////////////////////////////////////////
// file: encoder_test1.ino
// purpose: This is a test routine that will demonstrate proper functioning of
//          the motor encoders.
//          WARNING: Disconnect power to the motors.  Only leave the encoders connected.
//          Manually turn the motor shaft and verify that the encoder direction is correct.
//////////////////////////////////////////////

// Library Inclusions

#include <I2Cdev.h>                             // IMU IC2 Communication
#include <Wire.h>                               // IMU IC2 Communication
#include "MPU6050_6Axis_MotionApps20_mod_1.h"   // IMU,  changed value in line 305 to 1 to change update speed from every 15ms to every 10ms
#include <PinChangeInt.h>                       // Pin Change Interrupt
#include <digitalWriteFast.h>                   // Fast Digital Readings

// #define CUSTOM
#define POLOLU

#if defined(CUSTOM)
  // PinAssignments
  // These pin assignments correspond to the Arduino shield developed as part of this project.
  
  #define   pin_left_PMW          7    // PWM output for Left Motor
  #define   pin_left_EncoderB     A2   // Encoder_B input from Left Motor
  #define   pin_left_dir          8
  
  #define   pin_right_PMW         12   // PWM output for Left Motor
  #define   pin_right_EncoderB    6    // Encoder_B input from Right Moor
  #define   pin_right_dir         13
  
  #define   pin_IMU_Interrupt     A3
  #define   pin_IMU_SDA           A4
  #define   pin_IMU_SCL           A5
  
  // Interrupt Assignment
  
  #define   interruptPin_left_Encoder    0   // mapped to D2 - pin_left_EncoderA
  #define   interruptPin_right_Encoder   1   // mapped to D3 - pin_right_EncoderA
#else
  #if defined(POLOLU)
    // PinAssignments
    // These pin assignments correspond to the Pololu Dual MAX14870 Motor Driver Shield for Arduino.
    // https://www.pololu.com/product/2519
    
    #define   pin_left_PMW          9    // PWM output for Left Motor
    #define   pin_left_EncoderB     A2   // Encoder_B input from Left Motor
    #define   pin_left_dir          7
    
    #define   pin_right_PMW         10   // PWM output for Left Motor
    #define   pin_right_EncoderB    6    // Encoder_B input from Right Moor
    #define   pin_right_dir         8
    
    #define   pin_IMU_Interrupt     A3
    #define   pin_IMU_SDA           A4
    #define   pin_IMU_SCL           A5
    
    // Interrupt Assignment
    
    #define   interruptPin_left_Encoder    0   // mapped to D2 - pin_left_EncoderA
    #define   interruptPin_right_Encoder   1   // mapped to D3 - pin_right_EncoderA
  #endif
#endif
