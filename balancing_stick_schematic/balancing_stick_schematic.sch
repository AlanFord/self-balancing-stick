EESchema Schematic File Version 4
LIBS:balancing_stick_schematic-cache
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR?
U 1 1 5CF3F9AD
P 3650 3200
F 0 "#PWR?" H 3650 2950 50  0001 C CNN
F 1 "GND" H 3655 3027 50  0000 C CNN
F 2 "" H 3650 3200 50  0001 C CNN
F 3 "" H 3650 3200 50  0001 C CNN
	1    3650 3200
	1    0    0    -1  
$EndComp
NoConn ~ 4250 2950
NoConn ~ 4100 2950
NoConn ~ 4400 2950
Text Label 6000 2700 0    50   ~ 0
M1INA
Text Label 6000 2900 0    50   ~ 0
M1INB
Text Label 6000 3200 0    50   ~ 0
M2INA
Text Label 6000 3300 0    50   ~ 0
M2INB
$Comp
L balancing_stick:encoder U?
U 1 1 5DD3C2DD
P 4300 5150
F 0 "U?" H 4828 4913 50  0000 L CNN
F 1 "encoder" H 4828 4822 50  0000 L CNN
F 2 "" H 4300 5150 50  0001 C CNN
F 3 "" H 4300 5150 50  0001 C CNN
	1    4300 5150
	-1   0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_UNO_R3 A?
U 1 1 5DD43955
P 6800 3100
F 0 "A?" H 6300 4200 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 6300 4100 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 6950 2050 50  0001 L CNN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 6600 4150 50  0001 C CNN
	1    6800 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 2700 5900 2700
Wire Wire Line
	6300 2900 5900 2900
Wire Wire Line
	6300 3200 5900 3200
Wire Wire Line
	6300 3300 5900 3300
NoConn ~ 5900 2700
NoConn ~ 5900 2900
NoConn ~ 5900 3200
NoConn ~ 5900 3300
$Comp
L balancing_stick:GY-521 U?
U 1 1 5DD62D4F
P 4000 2350
F 0 "U?" H 4678 2163 50  0000 L CNN
F 1 "GY-521" H 4678 2072 50  0000 L CNN
F 2 "" H 3840 2380 50  0001 C CNN
F 3 "" H 3840 2380 50  0001 C CNN
	1    4000 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2950 3650 3200
Wire Wire Line
	7000 2100 7000 1850
$Comp
L balancing_stick:Pololu_breakout_VNH5019 U?
U 1 1 5DD7C136
P 5650 5200
F 0 "U?" H 5600 5250 50  0000 L CNN
F 1 "Pololu_breakout_VNH5019" H 5150 5100 50  0000 L CNN
F 2 "" H 5650 5200 50  0001 C CNN
F 3 "" H 5650 5200 50  0001 C CNN
	1    5650 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3100 5900 3100
NoConn ~ 5900 3100
Text Label 6000 3100 0    50   ~ 0
M1EN
Wire Wire Line
	6300 3400 5900 3400
Text Label 6000 3400 0    50   ~ 0
M1PWM
NoConn ~ 5900 3400
Wire Wire Line
	6300 3500 5900 3500
Text Label 6000 3500 0    50   ~ 0
M2PWM
NoConn ~ 5900 3500
Text Label 6000 3700 0    50   ~ 0
M2EN
NoConn ~ 5900 3700
Wire Wire Line
	5900 3700 6300 3700
Text Label 7600 3100 2    50   ~ 0
M1CS
Wire Wire Line
	7300 3100 7700 3100
NoConn ~ 7700 3100
Text Label 7600 3200 2    50   ~ 0
M2CS
Wire Wire Line
	7300 3200 7700 3200
NoConn ~ 7700 3200
Text Notes 8400 2750 0    50   ~ 0
VNH5019 pins are tied to the Arduino pins.\nDescriptions:\nM1INA: Motor 1 direction input A\nM1INB: Motor 1 direction input B\nM1EN: Motor 1 enable input\nM2INA: Motor 2 direction input A\nM2INB: Motor 2 direction input B\nM1PWM: Motor 1 speed input\nM2PWM: Motor 2 speed input\nM2EN: Motor 2 enable input\nM1CS: Motor 1 current sense output\nM2CS: Motor 2 current sense output
Wire Wire Line
	3950 2950 3950 4400
Wire Wire Line
	3950 4400 7700 4400
Wire Wire Line
	3800 2950 3800 4550
Wire Wire Line
	3800 4550 7850 4550
Text Notes 8350 3250 0    50   ~ 0
Motor Definitions for Arudino Code\nMotor 1 —> Left Motor\nMotor 2 —> Right Motor
Text Notes 5350 2150 0    50   ~ 0
Available Arduino Pins:\nD3\nD5\nD11\nD13\nA2\nA3\nA4\nA5
NoConn ~ 7300 2500
NoConn ~ 7300 2700
$Comp
L power:+5V #PWR?
U 1 1 5DD48ABD
P 7000 1850
F 0 "#PWR?" H 7000 1700 50  0001 C CNN
F 1 "+5V" H 7015 2023 50  0000 C CNN
F 2 "" H 7000 1850 50  0001 C CNN
F 3 "" H 7000 1850 50  0001 C CNN
	1    7000 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5DD4963C
P 3500 3200
F 0 "#PWR?" H 3500 3050 50  0001 C CNN
F 1 "+5V" H 3515 3373 50  0000 C CNN
F 2 "" H 3500 3200 50  0001 C CNN
F 3 "" H 3500 3200 50  0001 C CNN
	1    3500 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	3500 2950 3500 3200
$Comp
L power:+5V #PWR?
U 1 1 5DD4BA39
P 4200 6050
F 0 "#PWR?" H 4200 5900 50  0001 C CNN
F 1 "+5V" H 4215 6223 50  0000 C CNN
F 2 "" H 4200 6050 50  0001 C CNN
F 3 "" H 4200 6050 50  0001 C CNN
	1    4200 6050
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DD4D107
P 4350 5950
F 0 "#PWR?" H 4350 5700 50  0001 C CNN
F 1 "GND" H 4355 5777 50  0000 C CNN
F 2 "" H 4350 5950 50  0001 C CNN
F 3 "" H 4350 5950 50  0001 C CNN
	1    4350 5950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4350 5800 4350 5950
$Comp
L power:+5V #PWR?
U 1 1 5DD4E2CB
P 7100 5950
F 0 "#PWR?" H 7100 5800 50  0001 C CNN
F 1 "+5V" H 7115 6123 50  0000 C CNN
F 2 "" H 7100 5950 50  0001 C CNN
F 3 "" H 7100 5950 50  0001 C CNN
	1    7100 5950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DD4EB59
P 6950 5950
F 0 "#PWR?" H 6950 5700 50  0001 C CNN
F 1 "GND" H 6955 5777 50  0000 C CNN
F 2 "" H 6950 5950 50  0001 C CNN
F 3 "" H 6950 5950 50  0001 C CNN
	1    6950 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 5800 6950 5950
Wire Wire Line
	7100 5800 7100 5950
Wire Wire Line
	4550 2950 4550 3250
Text Notes 7400 4950 0    50   ~ 0
Encoder Vcc is 3.5V - 20V
$Comp
L balancing_stick:encoder U?
U 1 1 5DD3EB6A
P 7000 5150
F 0 "U?" H 7528 4913 50  0000 L CNN
F 1 "encoder" H 7528 4822 50  0000 L CNN
F 2 "" H 7000 5150 50  0001 C CNN
F 3 "" H 7000 5150 50  0001 C CNN
	1    7000 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 5800 5200 5950
Wire Wire Line
	5200 5950 4650 5950
Wire Wire Line
	4650 5950 4650 5800
Wire Wire Line
	5350 5800 5350 6050
Wire Wire Line
	5350 6050 4500 6050
Wire Wire Line
	4500 6050 4500 5800
Wire Wire Line
	6650 5800 6650 5900
Wire Wire Line
	6650 5900 6000 5900
Wire Wire Line
	6000 5900 6000 5800
Wire Wire Line
	6800 5800 6800 6000
Wire Wire Line
	6800 6000 6150 6000
Wire Wire Line
	6150 6000 6150 5800
$Comp
L power:GND #PWR?
U 1 1 5DD5F226
P 5550 5950
F 0 "#PWR?" H 5550 5700 50  0001 C CNN
F 1 "GND" H 5555 5777 50  0000 C CNN
F 2 "" H 5550 5950 50  0001 C CNN
F 3 "" H 5550 5950 50  0001 C CNN
	1    5550 5950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5550 5800 5550 5950
$Comp
L power:+12V #PWR?
U 1 1 5DD602E7
P 5700 6050
F 0 "#PWR?" H 5700 5900 50  0001 C CNN
F 1 "+12V" H 5715 6223 50  0000 C CNN
F 2 "" H 5700 6050 50  0001 C CNN
F 3 "" H 5700 6050 50  0001 C CNN
	1    5700 6050
	-1   0    0    1   
$EndComp
Wire Wire Line
	5700 5800 5700 6050
Wire Wire Line
	4200 5800 4200 6050
Wire Wire Line
	6300 2800 5600 2800
Wire Wire Line
	5600 2800 5600 3250
Wire Wire Line
	5600 3250 4550 3250
Wire Wire Line
	5900 3000 6300 3000
Text Label 5900 3000 2    50   ~ 0
M1_A
Wire Wire Line
	6300 3600 5900 3600
Text Label 5900 3600 2    50   ~ 0
M1_B
Text Label 4050 6100 2    50   ~ 0
M1_A
Wire Wire Line
	3900 5800 3900 5950
Text Label 3900 5950 2    50   ~ 0
M1_B
Wire Wire Line
	4050 5800 4050 6100
Text Label 7250 6050 0    50   ~ 0
M2_A
Wire Wire Line
	7400 5800 7400 5900
Text Label 7400 5900 0    50   ~ 0
M2_B
Wire Wire Line
	7250 5800 7250 6050
Text Label 5900 3800 2    50   ~ 0
M2_A
Wire Wire Line
	6300 3800 5900 3800
Text Label 7700 3300 0    50   ~ 0
M2_B
Wire Wire Line
	7300 3300 7700 3300
Wire Wire Line
	7700 3500 7300 3500
Wire Wire Line
	7700 3500 7700 4400
Wire Wire Line
	7300 3600 7850 3600
Wire Wire Line
	7850 3600 7850 4550
$EndSCHEMATC
