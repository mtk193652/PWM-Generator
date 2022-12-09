EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
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
L My-library:Encoder-Button U?
U 1 1 6392AAFA
P 4550 3300
F 0 "U?" H 4550 3625 50  0000 C CNN
F 1 "Encoder-Button" H 4550 3534 50  0000 C CNN
F 2 "" H 4550 3650 50  0001 C CNN
F 3 "" H 4550 3650 50  0001 C CNN
	1    4550 3300
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_Nano_Every A?
U 1 1 6392BB50
P 1800 2100
F 0 "A?" H 1800 1011 50  0000 C CNN
F 1 "Arduino_Nano_Every" H 1800 920 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 1800 2100 50  0001 C CIN
F 3 "https://content.arduino.cc/assets/NANOEveryV3.0_sch.pdf" H 1800 2100 50  0001 C CNN
	1    1800 2100
	1    0    0    -1  
$EndComp
$Comp
L My-library:Small-I2C-OLED U?
U 1 1 6392DC02
P 3700 1250
F 0 "U?" H 3878 1301 50  0000 L CNN
F 1 "Small-I2C-OLED" H 3878 1210 50  0000 L CNN
F 2 "" H 3700 1650 50  0001 C CNN
F 3 "" H 3700 1650 50  0001 C CNN
	1    3700 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 639329AD
P 5550 3550
F 0 "R?" H 5620 3596 50  0000 L CNN
F 1 "R" H 5620 3505 50  0000 L CNN
F 2 "" V 5480 3550 50  0001 C CNN
F 3 "~" H 5550 3550 50  0001 C CNN
	1    5550 3550
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 63933D30
P 3500 3350
F 0 "C?" H 3615 3396 50  0000 L CNN
F 1 "C" H 3615 3305 50  0000 L CNN
F 2 "" H 3538 3200 50  0001 C CNN
F 3 "~" H 3500 3350 50  0001 C CNN
	1    3500 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 63936D61
P 3850 3350
F 0 "C?" H 3965 3396 50  0000 L CNN
F 1 "C" H 3965 3305 50  0000 L CNN
F 2 "" H 3888 3200 50  0001 C CNN
F 3 "~" H 3850 3350 50  0001 C CNN
	1    3850 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3200 3850 3200
Wire Wire Line
	3850 3200 3500 3200
Connection ~ 3850 3200
Wire Wire Line
	3850 3500 4100 3500
Wire Wire Line
	4100 3500 4100 3300
Wire Wire Line
	4100 3300 4250 3300
Wire Wire Line
	4250 3400 4200 3400
Wire Wire Line
	4200 3700 3500 3700
Text GLabel 3350 3200 0    50   Input ~ 0
GND
Wire Wire Line
	3500 3200 3350 3200
Connection ~ 3500 3200
Text GLabel 2000 1000 1    50   Input ~ 0
Vcc
Text GLabel 3350 1200 0    50   Input ~ 0
Vcc
Text GLabel 3350 1100 0    50   Input ~ 0
GND
Text GLabel 5850 3550 2    50   Input ~ 0
GND
Wire Wire Line
	5700 3550 5850 3550
Wire Wire Line
	5400 3550 5250 3550
$Comp
L Device:C C?
U 1 1 6393E51B
P 5250 3400
F 0 "C?" H 5365 3446 50  0000 L CNN
F 1 "C" H 5365 3355 50  0000 L CNN
F 2 "" H 5288 3250 50  0001 C CNN
F 3 "~" H 5250 3400 50  0001 C CNN
	1    5250 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3250 4850 3250
Wire Wire Line
	4850 3350 5000 3350
Wire Wire Line
	5000 3350 5000 3550
Wire Wire Line
	5000 3550 5250 3550
Connection ~ 5250 3550
Wire Wire Line
	2000 1100 2000 1000
Text GLabel 5850 3250 2    50   Input ~ 0
Vcc
Wire Wire Line
	5850 3250 5250 3250
Connection ~ 5250 3250
Text GLabel 4950 3550 0    50   Input ~ 0
Enc2_Button
Wire Wire Line
	4950 3550 5000 3550
Connection ~ 5000 3550
Text GLabel 4050 3600 0    50   Input ~ 0
Enc2_A
Text GLabel 4050 3800 0    50   Input ~ 0
Enc2_B
Wire Wire Line
	4100 3500 4100 3600
Wire Wire Line
	4100 3600 4050 3600
Connection ~ 4100 3500
Wire Wire Line
	3500 3500 3500 3700
Wire Wire Line
	4200 3400 4200 3700
Wire Wire Line
	4050 3800 4200 3800
Wire Wire Line
	4200 3800 4200 3700
Connection ~ 4200 3700
Text GLabel 3350 1300 0    50   Input ~ 0
SCL
Text GLabel 3350 1400 0    50   Input ~ 0
SDA
Wire Wire Line
	3450 1400 3350 1400
Wire Wire Line
	3450 1300 3350 1300
Wire Wire Line
	3450 1200 3350 1200
Text GLabel 2400 2500 2    50   Input ~ 0
SDA
Wire Wire Line
	2300 2500 2400 2500
Text GLabel 2400 2600 2    50   Input ~ 0
SCL
Wire Wire Line
	2300 2600 2400 2600
Wire Wire Line
	3350 1100 3450 1100
Text GLabel 1900 3100 2    50   Input ~ 0
GND
Wire Wire Line
	1900 3100 1800 3100
$Comp
L My-library:Encoder-Button U?
U 1 1 63959B83
P 4550 2150
F 0 "U?" H 4550 2475 50  0000 C CNN
F 1 "Encoder-Button" H 4550 2384 50  0000 C CNN
F 2 "" H 4550 2500 50  0001 C CNN
F 3 "" H 4550 2500 50  0001 C CNN
	1    4550 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 63959B89
P 5550 2400
F 0 "R?" H 5620 2446 50  0000 L CNN
F 1 "R" H 5620 2355 50  0000 L CNN
F 2 "" V 5480 2400 50  0001 C CNN
F 3 "~" H 5550 2400 50  0001 C CNN
	1    5550 2400
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 63959B8F
P 3500 2200
F 0 "C?" H 3615 2246 50  0000 L CNN
F 1 "C" H 3615 2155 50  0000 L CNN
F 2 "" H 3538 2050 50  0001 C CNN
F 3 "~" H 3500 2200 50  0001 C CNN
	1    3500 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 63959B95
P 3850 2200
F 0 "C?" H 3965 2246 50  0000 L CNN
F 1 "C" H 3965 2155 50  0000 L CNN
F 2 "" H 3888 2050 50  0001 C CNN
F 3 "~" H 3850 2200 50  0001 C CNN
	1    3850 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 2050 3850 2050
Wire Wire Line
	3850 2050 3500 2050
Connection ~ 3850 2050
Wire Wire Line
	3850 2350 4100 2350
Wire Wire Line
	4100 2350 4100 2150
Wire Wire Line
	4100 2150 4250 2150
Wire Wire Line
	4250 2250 4200 2250
Wire Wire Line
	4200 2550 3500 2550
Text GLabel 3350 2050 0    50   Input ~ 0
GND
Wire Wire Line
	3500 2050 3350 2050
Connection ~ 3500 2050
Text GLabel 5850 2400 2    50   Input ~ 0
GND
Wire Wire Line
	5700 2400 5850 2400
Wire Wire Line
	5400 2400 5250 2400
$Comp
L Device:C C?
U 1 1 63959BA9
P 5250 2250
F 0 "C?" H 5365 2296 50  0000 L CNN
F 1 "C" H 5365 2205 50  0000 L CNN
F 2 "" H 5288 2100 50  0001 C CNN
F 3 "~" H 5250 2250 50  0001 C CNN
	1    5250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2100 4850 2100
Wire Wire Line
	4850 2200 5000 2200
Wire Wire Line
	5000 2200 5000 2400
Wire Wire Line
	5000 2400 5250 2400
Connection ~ 5250 2400
Text GLabel 5850 2100 2    50   Input ~ 0
Vcc
Wire Wire Line
	5850 2100 5750 2100
Connection ~ 5250 2100
Text GLabel 4950 2400 0    50   Input ~ 0
Enc1_Button
Wire Wire Line
	4950 2400 5000 2400
Connection ~ 5000 2400
Text GLabel 4050 2450 0    50   Input ~ 0
Enc1_A
Text GLabel 4050 2650 0    50   Input ~ 0
Enc1_B
Wire Wire Line
	4100 2350 4100 2450
Wire Wire Line
	4100 2450 4050 2450
Connection ~ 4100 2350
Wire Wire Line
	3500 2350 3500 2550
Wire Wire Line
	4200 2250 4200 2550
Wire Wire Line
	4050 2650 4200 2650
Wire Wire Line
	4200 2650 4200 2550
Connection ~ 4200 2550
Text GLabel 1200 2400 0    50   Input ~ 0
Enc1_Button
Text GLabel 1200 2200 0    50   Input ~ 0
Enc1_A
Text GLabel 1200 2300 0    50   Input ~ 0
Enc1_B
Text GLabel 1200 2600 0    50   Input ~ 0
Enc2_A
Text GLabel 1200 2700 0    50   Input ~ 0
Enc2_B
Text GLabel 1200 2800 0    50   Input ~ 0
Enc2_Button
Wire Wire Line
	1300 2200 1200 2200
Wire Wire Line
	1200 2300 1300 2300
Wire Wire Line
	1200 2400 1300 2400
Wire Wire Line
	1200 2600 1300 2600
Wire Wire Line
	1300 2700 1200 2700
Wire Wire Line
	1200 2800 1300 2800
$Comp
L Device:R R?
U 1 1 63976E49
P 4550 2650
F 0 "R?" H 4620 2696 50  0000 L CNN
F 1 "R" H 4620 2605 50  0000 L CNN
F 2 "" V 4480 2650 50  0001 C CNN
F 3 "~" H 4550 2650 50  0001 C CNN
	1    4550 2650
	0    1    -1   0   
$EndComp
Connection ~ 4100 2150
Wire Wire Line
	4200 2650 4400 2650
Connection ~ 4200 2650
Wire Wire Line
	4700 2650 5750 2650
Wire Wire Line
	5750 2650 5750 2100
Connection ~ 5750 2100
Wire Wire Line
	5750 2100 5250 2100
$Comp
L Device:R R?
U 1 1 6398E76F
P 5250 1900
F 0 "R?" H 5320 1946 50  0000 L CNN
F 1 "R" H 5320 1855 50  0000 L CNN
F 2 "" V 5180 1900 50  0001 C CNN
F 3 "~" H 5250 1900 50  0001 C CNN
	1    5250 1900
	1    0    0    1   
$EndComp
Wire Wire Line
	5250 2050 5250 2100
Wire Wire Line
	5250 1750 5250 1700
Wire Wire Line
	5250 1700 4100 1700
Wire Wire Line
	4100 1700 4100 2150
$Comp
L Device:R R?
U 1 1 63993D51
P 4550 3800
F 0 "R?" H 4620 3846 50  0000 L CNN
F 1 "R" H 4620 3755 50  0000 L CNN
F 2 "" V 4480 3800 50  0001 C CNN
F 3 "~" H 4550 3800 50  0001 C CNN
	1    4550 3800
	0    1    -1   0   
$EndComp
Wire Wire Line
	4200 3800 4400 3800
Wire Wire Line
	4700 3800 5750 3800
Wire Wire Line
	5750 3800 5750 3250
$Comp
L Device:R R?
U 1 1 63996C8C
P 5250 3050
F 0 "R?" H 5320 3096 50  0000 L CNN
F 1 "R" H 5320 3005 50  0000 L CNN
F 2 "" V 5180 3050 50  0001 C CNN
F 3 "~" H 5250 3050 50  0001 C CNN
	1    5250 3050
	1    0    0    1   
$EndComp
Wire Wire Line
	5250 2900 5250 2850
Wire Wire Line
	5250 2850 4100 2850
Wire Wire Line
	4100 2850 4100 3300
Wire Wire Line
	5250 3200 5250 3250
Text GLabel 1200 2100 0    50   Input ~ 0
PWM
Wire Wire Line
	1200 2100 1300 2100
$EndSCHEMATC
