# self-balancing-stick
Dual Axis Reaction Wheel Inverted Pendulum

## Those who go before us ...
This project is inspired by Mike Rouleau's "Self Balancing Stick", altered so that all of the components can be fabricated at local makerspaces.  Links to descriptions of Mike Rouleau's work are:
- http://hackaday.com/2016/08/11/stick-balances-itself-with-reaction-wheels/
- https://www.youtube.com/watch?v=woCdjbsjbPg
- https://drive.google.com/file/d/0B9CxKhrEOT-3Z2Y2ajhiaWl6QU0/view
- https://drive.google.com/file/d/0B9CxKhrEOT-3ekE1MnRoT3Izdm8/view
- https://grabcad.com/library/self-balancing-stick-dual-axis-reaction-wheel-inverted-pendulum-1/details
- https://www.reddit.com/r/arduino/comments/3n0g8p/self_balancing_stick_dual_axis_reaction_wheel/?st=is6s1bi9&sh=097623e7

Primary components used by Mike are:
- Mitsumi M25N-2 series Motor with a 334 PPR Encoder
- Arduino Uno Rev 3
- Seeed Studio 4A Motor Shield
- GY-521 Gyro (uses the MPU-6050 chip)﻿

Some links to parts used by Mike:
- http://www.ebay.com/itm/171395060368?_trksid=p2060353.m1438.l2649&ssPageName=STRK%3AMEBIDX%3AIT
- http://www.mitsumi.co.jp/latest/Catalog/pdf/motorav_m25n_2_e.pdf
- http://www.ebay.com/itm/231284342156?_trksid=p2060353.m1438.l2649&ssPageName=STRK%3AMEBIDX%3AIT

## The current work ...
Physical design work is being done in Fusion 360

Design of the control system is being done in Matlab and documented in Latex.

The hardware is based on what I have at hand.  
* 12" knitting needle
* 4.4:1 Metal Gearmotor 25Dx48L mm MP 12V with 48 CPR Encoder (Pololu Part # 3237)
* Pololu Universal Aluminum Mounting Hub for 4mm Shaft, #4-40 Holes (2-Pack) (Pololu Part # 1081)
* Pololu Dual VNH5019 Motor Driver Shield for Arduino (Pololu Part # 2507)
* STMicrolectronics NUCLEO-L432KC Dev Board
* Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 (Adafruit Part # 2472)
