EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 7874 5906
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
P 5050 3200
F 0 "U?" H 5050 3525 50  0000 C CNN
F 1 "Encoder-Button" H 5050 3434 50  0000 C CNN
F 2 "" H 5050 3550 50  0001 C CNN
F 3 "" H 5050 3550 50  0001 C CNN
	1    5050 3200
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_Nano_Every A?
U 1 1 6392BB50
P 2200 2400
F 0 "A?" H 2200 1311 50  0000 C CNN
F 1 "Arduino_Nano_Every" H 2200 1220 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 2200 2400 50  0001 C CIN
F 3 "https://content.arduino.cc/assets/NANOEveryV3.0_sch.pdf" H 2200 2400 50  0001 C CNN
	1    2200 2400
	1    0    0    -1  
$EndComp
$Comp
L My-library:Small-I2C-OLED U?
U 1 1 6392DC02
P 5050 1150
F 0 "U?" H 5228 1201 50  0000 L CNN
F 1 "Small-I2C-OLED" H 5228 1110 50  0000 L CNN
F 2 "" H 5050 1550 50  0001 C CNN
F 3 "" H 5050 1550 50  0001 C CNN
	1    5050 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 639329AD
P 6050 3450
F 0 "R?" H 6120 3496 50  0000 L CNN
F 1 "10K" H 6120 3405 50  0000 L CNN
F 2 "" V 5980 3450 50  0001 C CNN
F 3 "~" H 6050 3450 50  0001 C CNN
	1    6050 3450
	0    -1   1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 63933D30
P 3700 3250
F 0 "C?" H 3815 3296 50  0000 L CNN
F 1 "100nF" H 3815 3205 50  0000 L CNN
F 2 "" H 3738 3100 50  0001 C CNN
F 3 "~" H 3700 3250 50  0001 C CNN
	1    3700 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 63936D61
P 4200 3250
F 0 "C?" H 4315 3296 50  0000 L CNN
F 1 "100nF" H 4315 3205 50  0000 L CNN
F 2 "" H 4238 3100 50  0001 C CNN
F 3 "~" H 4200 3250 50  0001 C CNN
	1    4200 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3400 4600 3200
Wire Wire Line
	4600 3200 4750 3200
Wire Wire Line
	4750 3300 4700 3300
Text GLabel 3550 3100 0    50   Input ~ 0
GND
Wire Wire Line
	3700 3100 3550 3100
Connection ~ 3700 3100
Text GLabel 2400 1300 1    50   Input ~ 0
Vcc
Text GLabel 4700 1100 0    50   Input ~ 0
Vcc
Text GLabel 4700 1000 0    50   Input ~ 0
GND
Text GLabel 6350 3450 2    50   Input ~ 0
GND
Wire Wire Line
	6200 3450 6350 3450
Wire Wire Line
	5900 3450 5750 3450
$Comp
L Device:C C?
U 1 1 6393E51B
P 5750 3300
F 0 "C?" H 5865 3346 50  0000 L CNN
F 1 "100nF" H 5865 3255 50  0000 L CNN
F 2 "" H 5788 3150 50  0001 C CNN
F 3 "~" H 5750 3300 50  0001 C CNN
	1    5750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3150 5350 3150
Wire Wire Line
	5350 3250 5500 3250
Wire Wire Line
	5500 3250 5500 3450
Wire Wire Line
	5500 3450 5750 3450
Connection ~ 5750 3450
Wire Wire Line
	2400 1400 2400 1300
Text GLabel 6350 3150 2    50   Input ~ 0
Vcc
Wire Wire Line
	6350 3150 6250 3150
Connection ~ 5750 3150
Text GLabel 5450 3450 0    50   Input ~ 0
Enc2_Button
Wire Wire Line
	5450 3450 5500 3450
Connection ~ 5500 3450
Text GLabel 4550 3500 0    50   Input ~ 0
Enc2_A
Text GLabel 4550 3700 0    50   Input ~ 0
Enc2_B
Wire Wire Line
	4600 3400 4600 3500
Wire Wire Line
	4600 3500 4550 3500
Connection ~ 4600 3400
Wire Wire Line
	3700 3400 3700 3600
Wire Wire Line
	4550 3700 4700 3700
Text GLabel 4700 1200 0    50   Input ~ 0
SCL
Text GLabel 4700 1300 0    50   Input ~ 0
SDA
Wire Wire Line
	4800 1300 4700 1300
Wire Wire Line
	4800 1200 4700 1200
Wire Wire Line
	4800 1100 4700 1100
Text GLabel 2800 2800 2    50   Input ~ 0
SDA
Wire Wire Line
	2700 2800 2800 2800
Text GLabel 2800 2900 2    50   Input ~ 0
SCL
Wire Wire Line
	2700 2900 2800 2900
Wire Wire Line
	4700 1000 4800 1000
Text GLabel 2300 3400 2    50   Input ~ 0
GND
Wire Wire Line
	2300 3400 2200 3400
$Comp
L My-library:Encoder-Button U?
U 1 1 63959B83
P 5050 1950
F 0 "U?" H 5050 2275 50  0000 C CNN
F 1 "Encoder-Button" H 5050 2184 50  0000 C CNN
F 2 "" H 5050 2300 50  0001 C CNN
F 3 "" H 5050 2300 50  0001 C CNN
	1    5050 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 63959B89
P 6050 2200
F 0 "R?" H 6120 2246 50  0000 L CNN
F 1 "10K" H 6120 2155 50  0000 L CNN
F 2 "" V 5980 2200 50  0001 C CNN
F 3 "~" H 6050 2200 50  0001 C CNN
	1    6050 2200
	0    -1   1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 63959B8F
P 3700 2000
F 0 "C?" H 3815 2046 50  0000 L CNN
F 1 "100nF" H 3815 1955 50  0000 L CNN
F 2 "" H 3738 1850 50  0001 C CNN
F 3 "~" H 3700 2000 50  0001 C CNN
	1    3700 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 63959B95
P 4200 2000
F 0 "C?" H 4315 2046 50  0000 L CNN
F 1 "100nF" H 4315 1955 50  0000 L CNN
F 2 "" H 4238 1850 50  0001 C CNN
F 3 "~" H 4200 2000 50  0001 C CNN
	1    4200 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2150 4600 1950
Wire Wire Line
	4600 1950 4750 1950
Wire Wire Line
	4750 2050 4700 2050
Text GLabel 3550 1850 0    50   Input ~ 0
GND
Wire Wire Line
	3700 1850 3550 1850
Connection ~ 3700 1850
Text GLabel 6350 2200 2    50   Input ~ 0
GND
Wire Wire Line
	6200 2200 6350 2200
Wire Wire Line
	5900 2200 5750 2200
$Comp
L Device:C C?
U 1 1 63959BA9
P 5750 2050
F 0 "C?" H 5865 2096 50  0000 L CNN
F 1 "100nF" H 5865 2005 50  0000 L CNN
F 2 "" H 5788 1900 50  0001 C CNN
F 3 "~" H 5750 2050 50  0001 C CNN
	1    5750 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1900 5350 1900
Wire Wire Line
	5350 2000 5500 2000
Wire Wire Line
	5500 2000 5500 2200
Wire Wire Line
	5500 2200 5750 2200
Connection ~ 5750 2200
Text GLabel 6350 1900 2    50   Input ~ 0
Vcc
Wire Wire Line
	6350 1900 6250 1900
Connection ~ 5750 1900
Text GLabel 5450 2200 0    50   Input ~ 0
Enc1_Button
Wire Wire Line
	5450 2200 5500 2200
Connection ~ 5500 2200
Text GLabel 4550 2250 0    50   Input ~ 0
Enc1_A
Text GLabel 4550 2450 0    50   Input ~ 0
Enc1_B
Wire Wire Line
	4600 2150 4600 2250
Wire Wire Line
	4600 2250 4550 2250
Connection ~ 4600 2150
Wire Wire Line
	3700 2150 3700 2350
Wire Wire Line
	4550 2450 4700 2450
Text GLabel 1600 2700 0    50   Input ~ 0
Enc1_Button
Text GLabel 1600 2500 0    50   Input ~ 0
Enc1_A
Text GLabel 1600 2600 0    50   Input ~ 0
Enc1_B
Text GLabel 1600 2900 0    50   Input ~ 0
Enc2_A
Text GLabel 1600 3000 0    50   Input ~ 0
Enc2_B
Text GLabel 1600 3100 0    50   Input ~ 0
Enc2_Button
Wire Wire Line
	1700 2500 1600 2500
Wire Wire Line
	1600 2600 1700 2600
Wire Wire Line
	1600 2700 1700 2700
Wire Wire Line
	1600 2900 1700 2900
Wire Wire Line
	1700 3000 1600 3000
Wire Wire Line
	1600 3100 1700 3100
$Comp
L Device:R R?
U 1 1 63976E49
P 5050 2450
F 0 "R?" H 5120 2496 50  0000 L CNN
F 1 "10K" H 5120 2405 50  0000 L CNN
F 2 "" V 4980 2450 50  0001 C CNN
F 3 "~" H 5050 2450 50  0001 C CNN
	1    5050 2450
	0    1    1    0   
$EndComp
Connection ~ 4600 1950
Wire Wire Line
	4700 2450 4900 2450
Connection ~ 4700 2450
Wire Wire Line
	5200 2450 6250 2450
Wire Wire Line
	6250 2450 6250 1900
Connection ~ 6250 1900
Wire Wire Line
	6250 1900 5750 1900
$Comp
L Device:R R?
U 1 1 6398E76F
P 5750 1700
F 0 "R?" H 5820 1746 50  0000 L CNN
F 1 "10K" H 5820 1655 50  0000 L CNN
F 2 "" V 5680 1700 50  0001 C CNN
F 3 "~" H 5750 1700 50  0001 C CNN
	1    5750 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1850 5750 1900
Wire Wire Line
	5750 1550 5750 1500
Wire Wire Line
	5750 1500 4600 1500
Wire Wire Line
	4600 1500 4600 1950
$Comp
L Device:R R?
U 1 1 63993D51
P 5050 3700
F 0 "R?" H 5120 3746 50  0000 L CNN
F 1 "10K" H 5120 3655 50  0000 L CNN
F 2 "" V 4980 3700 50  0001 C CNN
F 3 "~" H 5050 3700 50  0001 C CNN
	1    5050 3700
	0    -1   1    0   
$EndComp
Wire Wire Line
	4700 3700 4900 3700
Wire Wire Line
	5200 3700 6250 3700
Wire Wire Line
	6250 3700 6250 3150
$Comp
L Device:R R?
U 1 1 63996C8C
P 5750 2950
F 0 "R?" H 5820 2996 50  0000 L CNN
F 1 "10K" H 5820 2905 50  0000 L CNN
F 2 "" V 5680 2950 50  0001 C CNN
F 3 "~" H 5750 2950 50  0001 C CNN
	1    5750 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 2800 5750 2750
Wire Wire Line
	5750 2750 4600 2750
Wire Wire Line
	4600 2750 4600 3200
Wire Wire Line
	5750 3100 5750 3150
Text GLabel 1600 2400 0    50   Input ~ 0
PWM
Wire Wire Line
	1600 2400 1700 2400
Text GLabel 1600 1300 0    50   Input ~ 0
Vin
$Comp
L Device:D D?
U 1 1 6393A3FE
P 1850 1300
F 0 "D?" H 1850 1083 50  0000 C CNN
F 1 "D" H 1850 1174 50  0000 C CNN
F 2 "" H 1850 1300 50  0001 C CNN
F 3 "~" H 1850 1300 50  0001 C CNN
	1    1850 1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 1400 2100 1300
Wire Wire Line
	2100 1300 2000 1300
Wire Wire Line
	1600 1300 1700 1300
Connection ~ 4600 3200
Connection ~ 4700 3700
Connection ~ 6250 3150
Wire Wire Line
	6250 3150 5750 3150
Wire Wire Line
	4700 3300 4700 3600
Wire Wire Line
	3700 3100 4200 3100
Connection ~ 4200 3100
Wire Wire Line
	4200 3100 4750 3100
Wire Wire Line
	4200 3400 4600 3400
Connection ~ 4700 3600
Wire Wire Line
	4700 3600 4700 3700
Wire Wire Line
	3700 3600 4700 3600
Wire Wire Line
	4700 2050 4700 2350
Wire Wire Line
	4200 2150 4600 2150
Wire Wire Line
	3700 1850 4200 1850
Connection ~ 4200 1850
Wire Wire Line
	4200 1850 4750 1850
Wire Wire Line
	3700 2350 4700 2350
Connection ~ 4700 2350
Wire Wire Line
	4700 2350 4700 2450
Text GLabel 3950 1300 2    50   Input ~ 0
GND
Wire Wire Line
	3800 1300 3950 1300
$Comp
L Device:C C?
U 1 1 6399C612
P 3800 1150
F 0 "C?" H 3915 1196 50  0000 L CNN
F 1 "100uF" H 3915 1105 50  0000 L CNN
F 2 "" H 3838 1000 50  0001 C CNN
F 3 "~" H 3800 1150 50  0001 C CNN
	1    3800 1150
	-1   0    0    -1  
$EndComp
Text GLabel 3900 1000 2    50   Input ~ 0
Vcc
Wire Wire Line
	3800 1000 3900 1000
$EndSCHEMATC
