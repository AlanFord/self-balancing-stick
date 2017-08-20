
// Library Inclusions

#include <I2Cdev.h>                             // IMU IC2 Communication
#include <Wire.h>                               // IMU IC2 Communication
#include <MPU6050_6Axis_MotionApps20_mod_1.h>   // IMU,  changed value in line 305 to 1 to change update speed from every 15ms to every 10ms
#include <PinChangeInt.h>                       // Pin Change Interrupt
#include <digitalWriteFast.h>                   // Fast Digital Readings



// Pin Assignments

#define   pin_left_PMW          9
#define   pin_left_EncoderA     2
#define   pin_left_EncoderB     A2

#define   pin_right_PMW         10
#define   pin_right_EncoderA    3  
#define   pin_right_EncoderB    4

#define   pin_IN1               5
#define   pin_IN2               6
#define   pin_IN3               7
#define   pin_IN4               8

#define   pin_left_Current      A0
#define   pin_right_Current     A1

#define   pin_IMU_Interrupt     12
#define   pin_IMU_SDA           A4
#define   pin_IMU_SCL           A5

#define   pin_Pot               A3
#define   pin_Pot_kp            //
#define   pin_Pot_ki            //
#define   pin_Pot_kd            //



// Interrupt Assignment

#define   interruptPin_left_Encoder    0
#define   interruptPin_right_Encoder   1
