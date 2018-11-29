EESchema Schematic File Version 4
LIBS:Shield_Arduino_MOSH-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "lun. 30 mars 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 8950 1450 1    60   ~ 0
Vin
Text Label 9350 1450 1    60   ~ 0
IOREF
Text Label 8900 2500 0    60   ~ 0
A0
Text Label 8900 2600 0    60   ~ 0
A1
Text Label 8900 2700 0    60   ~ 0
A2
Text Label 8900 2800 0    60   ~ 0
A3
Text Label 8900 2900 0    60   ~ 0
A4(SDA)
Text Label 8900 3000 0    60   ~ 0
A5(SCL)
Text Label 10550 3000 0    60   ~ 0
0(Rx)
Text Label 10550 2800 0    60   ~ 0
2
Text Label 10550 2900 0    60   ~ 0
1(Tx)
Text Label 10550 2700 0    60   ~ 0
3(**)
Text Label 10550 2600 0    60   ~ 0
4
Text Label 10550 2500 0    60   ~ 0
5(**)
Text Label 10550 2400 0    60   ~ 0
6(**)
Text Label 10550 2300 0    60   ~ 0
7
Text Label 10550 2100 0    60   ~ 0
8
Text Label 10550 2000 0    60   ~ 0
9(**)
Text Label 10550 1900 0    60   ~ 0
10(**/SS)
Text Label 10550 1800 0    60   ~ 0
11(**/MOSI)
Text Label 10550 1700 0    60   ~ 0
12(MISO)
Text Label 10550 1600 0    60   ~ 0
13(SCK)
Text Label 10550 1400 0    60   ~ 0
AREF
NoConn ~ 9400 1600
Text Label 10550 1300 0    60   ~ 0
A4(SDA)
Text Label 10550 1200 0    60   ~ 0
A5(SCL)
Text Notes 10850 1000 0    60   ~ 0
Holes
Text Notes 8550 750  0    60   ~ 0
Shield for Arduino that uses\nthe same pin disposition\nlike "Uno" board Rev 3.
$Comp
L Connector_Generic:Conn_01x08 P1
U 1 1 56D70129
P 9600 1900
F 0 "P1" H 9600 2350 50  0000 C CNN
F 1 "Power" V 9700 1900 50  0000 C CNN
F 2 "Socket_Arduino_Uno:Socket_Strip_Arduino_1x08" V 9750 1900 20  0000 C CNN
F 3 "" H 9600 1900 50  0000 C CNN
	1    9600 1900
	1    0    0    -1  
$EndComp
Text Label 8650 1800 0    60   ~ 0
Reset
$Comp
L power:+3.3V #PWR01
U 1 1 56D70538
P 9150 1450
F 0 "#PWR01" H 9150 1300 50  0001 C CNN
F 1 "+3.3V" V 9150 1700 50  0000 C CNN
F 2 "" H 9150 1450 50  0000 C CNN
F 3 "" H 9150 1450 50  0000 C CNN
	1    9150 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 56D707BB
P 9050 1350
F 0 "#PWR02" H 9050 1200 50  0001 C CNN
F 1 "+5V" V 9050 1550 50  0000 C CNN
F 2 "" H 9050 1350 50  0000 C CNN
F 3 "" H 9050 1350 50  0000 C CNN
	1    9050 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 56D70CC2
P 9300 3150
F 0 "#PWR03" H 9300 2900 50  0001 C CNN
F 1 "GND" H 9300 3000 50  0000 C CNN
F 2 "" H 9300 3150 50  0000 C CNN
F 3 "" H 9300 3150 50  0000 C CNN
	1    9300 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 56D70CFF
P 10300 3150
F 0 "#PWR04" H 10300 2900 50  0001 C CNN
F 1 "GND" H 10300 3000 50  0000 C CNN
F 2 "" H 10300 3150 50  0000 C CNN
F 3 "" H 10300 3150 50  0000 C CNN
	1    10300 3150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 P2
U 1 1 56D70DD8
P 9600 2700
F 0 "P2" H 9600 2300 50  0000 C CNN
F 1 "Analog" V 9700 2700 50  0000 C CNN
F 2 "Socket_Arduino_Uno:Socket_Strip_Arduino_1x06" V 9750 2750 20  0000 C CNN
F 3 "" H 9600 2700 50  0000 C CNN
	1    9600 2700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 P5
U 1 1 56D71177
P 10800 650
F 0 "P5" V 10900 650 50  0000 C CNN
F 1 "CONN_01X01" V 10900 650 50  0001 C CNN
F 2 "Socket_Arduino_Uno:Arduino_1pin" H 10721 724 20  0000 C CNN
F 3 "" H 10800 650 50  0000 C CNN
	1    10800 650 
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 P6
U 1 1 56D71274
P 10900 650
F 0 "P6" V 11000 650 50  0000 C CNN
F 1 "CONN_01X01" V 11000 650 50  0001 C CNN
F 2 "Socket_Arduino_Uno:Arduino_1pin" H 10900 650 20  0001 C CNN
F 3 "" H 10900 650 50  0000 C CNN
	1    10900 650 
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 P7
U 1 1 56D712A8
P 11000 650
F 0 "P7" V 11100 650 50  0000 C CNN
F 1 "CONN_01X01" V 11100 650 50  0001 C CNN
F 2 "Socket_Arduino_Uno:Arduino_1pin" V 11000 650 20  0001 C CNN
F 3 "" H 11000 650 50  0000 C CNN
	1    11000 650 
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 P8
U 1 1 56D712DB
P 11100 650
F 0 "P8" V 11200 650 50  0000 C CNN
F 1 "CONN_01X01" V 11200 650 50  0001 C CNN
F 2 "Socket_Arduino_Uno:Arduino_1pin" H 11024 572 20  0000 C CNN
F 3 "" H 11100 650 50  0000 C CNN
	1    11100 650 
	0    -1   -1   0   
$EndComp
NoConn ~ 10800 850 
NoConn ~ 10900 850 
NoConn ~ 11000 850 
NoConn ~ 11100 850 
$Comp
L Connector_Generic:Conn_01x08 P4
U 1 1 56D7164F
P 10000 2600
F 0 "P4" H 10000 2100 50  0000 C CNN
F 1 "Digital" V 10100 2600 50  0000 C CNN
F 2 "Socket_Arduino_Uno:Socket_Strip_Arduino_1x08" V 10150 2550 20  0000 C CNN
F 3 "" H 10000 2600 50  0000 C CNN
	1    10000 2600
	-1   0    0    -1  
$EndComp
Wire Notes Line
	8525 825  9925 825 
Wire Notes Line
	9925 825  9925 475 
Wire Wire Line
	9350 1450 9350 1700
Wire Wire Line
	9350 1700 9400 1700
Wire Wire Line
	9400 1900 9150 1900
Wire Wire Line
	9400 2000 9050 2000
Wire Wire Line
	9400 2300 8950 2300
Wire Wire Line
	9400 2100 9300 2100
Wire Wire Line
	9400 2200 9300 2200
Connection ~ 9300 2200
Wire Wire Line
	8950 2300 8950 1450
Wire Wire Line
	9050 2000 9050 1350
Wire Wire Line
	9150 1900 9150 1450
Wire Wire Line
	9400 2500 8900 2500
Wire Wire Line
	9400 2600 8900 2600
Wire Wire Line
	9400 2700 8900 2700
Wire Wire Line
	9400 2800 8900 2800
Wire Wire Line
	9400 2900 8900 2900
Wire Wire Line
	9400 3000 8900 3000
$Comp
L Connector_Generic:Conn_01x10 P3
U 1 1 56D721E0
P 10000 1600
F 0 "P3" H 10000 2150 50  0000 C CNN
F 1 "Digital" V 10100 1600 50  0000 C CNN
F 2 "Socket_Arduino_Uno:Socket_Strip_Arduino_1x10" V 10150 1600 20  0000 C CNN
F 3 "" H 10000 1600 50  0000 C CNN
	1    10000 1600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10200 2100 10550 2100
Wire Wire Line
	10200 2000 10550 2000
Wire Wire Line
	10200 1900 10550 1900
Wire Wire Line
	10200 1800 10550 1800
Wire Wire Line
	10200 1700 10550 1700
Wire Wire Line
	10200 1600 10550 1600
Wire Wire Line
	10200 1400 10550 1400
Wire Wire Line
	10200 1300 10550 1300
Wire Wire Line
	10200 1200 10550 1200
Wire Wire Line
	10200 3000 10550 3000
Wire Wire Line
	10200 2900 10550 2900
Wire Wire Line
	10200 2800 10550 2800
Wire Wire Line
	10200 2700 10550 2700
Wire Wire Line
	10200 2600 10550 2600
Wire Wire Line
	10200 2500 10550 2500
Wire Wire Line
	10200 2400 10550 2400
Wire Wire Line
	10200 2300 10550 2300
Wire Wire Line
	10200 1500 10300 1500
Wire Wire Line
	10300 1500 10300 3150
Wire Wire Line
	9300 2100 9300 2200
Wire Wire Line
	9300 2200 9300 3150
Wire Notes Line
	8500 500  8500 3450
Wire Notes Line
	8500 3450 11200 3450
Wire Wire Line
	9400 1800 8650 1800
Text Notes 9700 1600 0    60   ~ 0
1
Wire Notes Line
	11200 1000 10700 1000
Wire Notes Line
	10700 1000 10700 500 
$Comp
L Device:R R3
U 1 1 5BBC5B73
P 7750 1450
F 0 "R3" H 7820 1496 50  0000 L CNN
F 1 "10k" H 7820 1405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7680 1450 50  0001 C CNN
F 3 "~" H 7750 1450 50  0001 C CNN
	1    7750 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5BBC5C87
P 6250 1600
F 0 "R1" H 6320 1646 50  0000 L CNN
F 1 "660" H 6320 1555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6180 1600 50  0001 C CNN
F 3 "~" H 6250 1600 50  0001 C CNN
	1    6250 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5BBC5CFF
P 7250 1050
F 0 "D1" V 7288 933 50  0000 R CNN
F 1 "LED" V 7197 933 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_Clear" H 7250 1050 50  0001 C CNN
F 3 "~" H 7250 1050 50  0001 C CNN
	1    7250 1050
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5BBC5E64
P 7750 1050
F 0 "D2" V 7788 933 50  0000 R CNN
F 1 "LED" V 7697 933 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_Clear" H 7750 1050 50  0001 C CNN
F 3 "~" H 7750 1050 50  0001 C CNN
	1    7750 1050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5BBC5EF7
P 7250 1450
F 0 "R2" H 7320 1496 50  0000 L CNN
F 1 "10k" H 7320 1405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7180 1450 50  0001 C CNN
F 3 "~" H 7250 1450 50  0001 C CNN
	1    7250 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5BBC5FE1
P 7750 1650
F 0 "#PWR0101" H 7750 1400 50  0001 C CNN
F 1 "GND" H 7750 1500 50  0000 C CNN
F 2 "" H 7750 1650 50  0000 C CNN
F 3 "" H 7750 1650 50  0000 C CNN
	1    7750 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5BBC6006
P 7250 1650
F 0 "#PWR0102" H 7250 1400 50  0001 C CNN
F 1 "GND" H 7250 1500 50  0000 C CNN
F 2 "" H 7250 1650 50  0000 C CNN
F 3 "" H 7250 1650 50  0000 C CNN
	1    7250 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1200 7750 1300
Wire Wire Line
	7750 1600 7750 1650
Wire Wire Line
	7250 1650 7250 1600
Wire Wire Line
	7250 1200 7250 1300
Text Label 7750 750  0    50   ~ 0
9(**)
Wire Wire Line
	7750 900  7750 750 
Text Label 7250 750  0    60   ~ 0
8
Wire Wire Line
	7250 750  7250 900 
$Comp
L power:+5V #PWR0103
U 1 1 5BBCBA9B
P 6250 850
F 0 "#PWR0103" H 6250 700 50  0001 C CNN
F 1 "+5V" V 6250 1050 50  0000 C CNN
F 2 "" H 6250 850 50  0000 C CNN
F 3 "" H 6250 850 50  0000 C CNN
	1    6250 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5BBCBAC0
P 6250 1800
F 0 "#PWR0104" H 6250 1550 50  0001 C CNN
F 1 "GND" H 6250 1650 50  0000 C CNN
F 2 "" H 6250 1800 50  0000 C CNN
F 3 "" H 6250 1800 50  0000 C CNN
	1    6250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1750 6250 1800
$Comp
L Switch:SW_Push SW1
U 1 1 5BBCC577
P 6250 1100
F 0 "SW1" V 6296 1052 50  0000 R CNN
F 1 "SW_Push" V 6205 1052 50  0000 R CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H4.3mm" H 6250 1300 50  0001 C CNN
F 3 "" H 6250 1300 50  0001 C CNN
	1    6250 1100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6250 850  6250 900 
Text Label 6600 1350 0    60   ~ 0
7
Wire Wire Line
	6600 1350 6250 1350
Wire Wire Line
	6250 1300 6250 1350
Connection ~ 6250 1350
Wire Wire Line
	6250 1350 6250 1450
$Comp
L Device:Buzzer BZ1
U 1 1 5BBD2E8E
P 5450 1200
F 0 "BZ1" H 5603 1229 50  0000 L CNN
F 1 "Buzzer" H 5603 1138 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_15x7.5RM7.6" V 5425 1300 50  0001 C CNN
F 3 "~" V 5425 1300 50  0001 C CNN
	1    5450 1200
	1    0    0    -1  
$EndComp
Text Label 5150 950  0    60   ~ 0
6(**)
Wire Wire Line
	5150 950  5150 1100
Wire Wire Line
	5150 1100 5350 1100
$Comp
L power:GND #PWR0105
U 1 1 5BBD3CAD
P 5300 1400
F 0 "#PWR0105" H 5300 1150 50  0001 C CNN
F 1 "GND" H 5300 1250 50  0000 C CNN
F 2 "" H 5300 1400 50  0000 C CNN
F 3 "" H 5300 1400 50  0000 C CNN
	1    5300 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1300 5300 1300
Wire Wire Line
	5300 1300 5300 1400
$Comp
L Shield_Arduino:RN2483A U1
U 1 1 5BBD67A7
P 3400 1450
F 0 "U1" V 4015 1444 50  0000 C CNN
F 1 "RN2483A" V 3924 1444 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 3750 1400 50  0001 C CNN
F 3 "" H 3750 1400 50  0001 C CNN
	1    3400 1450
	0    -1   -1   0   
$EndComp
Text Label 3950 1800 0    60   ~ 0
10(**/SS)
Text Label 3950 1700 0    60   ~ 0
11(**/MOSI)
Text Label 3950 1400 0    60   ~ 0
12(MISO)
$Comp
L power:+3.3V #PWR0106
U 1 1 5BBD9453
P 4550 1050
F 0 "#PWR0106" H 4550 900 50  0001 C CNN
F 1 "+3.3V" V 4550 1300 50  0000 C CNN
F 2 "" H 4550 1050 50  0000 C CNN
F 3 "" H 4550 1050 50  0000 C CNN
	1    4550 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5BBD947E
P 3800 2000
F 0 "#PWR0107" H 3800 1750 50  0001 C CNN
F 1 "GND" H 3800 1850 50  0000 C CNN
F 2 "" H 3800 2000 50  0000 C CNN
F 3 "" H 3800 2000 50  0000 C CNN
	1    3800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1100 3800 1100
Wire Wire Line
	3800 1100 3800 2000
Wire Wire Line
	3600 1200 4550 1200
Wire Wire Line
	3600 1300 4550 1300
Wire Wire Line
	4550 1050 4550 1200
Connection ~ 4550 1200
Wire Wire Line
	4550 1200 4550 1300
Wire Wire Line
	3950 1400 3600 1400
Wire Wire Line
	3600 1700 3950 1700
Wire Wire Line
	3600 1800 3950 1800
NoConn ~ 3600 1500
NoConn ~ 3600 1600
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5BBE3AA0
P 1950 1500
F 0 "J1" H 1870 1075 50  0000 C CNN
F 1 "Conn_01x04" H 1870 1166 50  0000 C CNN
F 2 "MesEmpreintes:TO-5-4" H 1950 1500 50  0001 C CNN
F 3 "~" H 1950 1500 50  0001 C CNN
	1    1950 1500
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5BBE6370
P 2250 1800
F 0 "#PWR0108" H 2250 1550 50  0001 C CNN
F 1 "GND" H 2250 1650 50  0000 C CNN
F 2 "" H 2250 1800 50  0000 C CNN
F 3 "" H 2250 1800 50  0000 C CNN
	1    2250 1800
	1    0    0    -1  
$EndComp
Text Label 2500 1500 0    60   ~ 0
Capt_preamp
$Comp
L power:GND #PWR05
U 1 1 5BD806BF
P 3600 3200
F 0 "#PWR05" H 3600 2950 50  0001 C CNN
F 1 "GND" H 3600 3050 50  0000 C CNN
F 2 "" H 3600 3200 50  0000 C CNN
F 3 "" H 3600 3200 50  0000 C CNN
	1    3600 3200
	1    0    0    -1  
$EndComp
Text Label 1200 2900 2    60   ~ 0
Capt_preamp
$Comp
L Shield_Arduino_une~autre:LTC1050 U2
U 1 1 5BD9AEAB
P 3200 3550
F 0 "U2" H 3791 3596 50  0000 L CNN
F 1 "LTC1050" H 3791 3505 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 3200 3550 50  0001 C CNN
F 3 "" H 3200 3550 50  0001 C CNN
	1    3200 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3700 2700 3700
$Comp
L power:GND #PWR07
U 1 1 5BDA8B16
P 3250 3850
F 0 "#PWR07" H 3250 3600 50  0001 C CNN
F 1 "GND" H 3250 3700 50  0000 C CNN
F 2 "" H 3250 3850 50  0000 C CNN
F 3 "" H 3250 3850 50  0000 C CNN
	1    3250 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3800 3250 3850
$Comp
L power:+5V #PWR06
U 1 1 5BDAAB55
P 3250 2800
F 0 "#PWR06" H 3250 2650 50  0001 C CNN
F 1 "+5V" V 3250 3000 50  0000 C CNN
F 2 "" H 3250 2800 50  0000 C CNN
F 3 "" H 3250 2800 50  0000 C CNN
	1    3250 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5BDB7AFC
P 1600 3050
F 0 "R5" H 1670 3096 50  0000 L CNN
F 1 "10k" H 1670 3005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1530 3050 50  0001 C CNN
F 3 "~" H 1600 3050 50  0001 C CNN
	1    1600 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5BDB7B78
P 2000 3600
F 0 "R6" H 2070 3646 50  0000 L CNN
F 1 "100k" H 2070 3555 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1930 3600 50  0001 C CNN
F 3 "~" H 2000 3600 50  0001 C CNN
	1    2000 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5BDB7BFE
P 1600 3600
F 0 "C1" H 1715 3646 50  0000 L CNN
F 1 "100n" H 1715 3555 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 1638 3450 50  0001 C CNN
F 3 "~" H 1600 3600 50  0001 C CNN
	1    1600 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2900 1600 2900
Wire Wire Line
	1600 3200 1600 3400
Wire Wire Line
	1600 3400 2000 3400
Wire Wire Line
	2000 3400 2000 3450
Connection ~ 1600 3400
Wire Wire Line
	1600 3400 1600 3450
Wire Wire Line
	3750 3550 4300 3550
Wire Wire Line
	2700 3700 2700 4100
$Comp
L Device:C C3
U 1 1 5BDC98C1
P 4300 3850
F 0 "C3" H 4415 3896 50  0000 L CNN
F 1 "1u" H 4415 3805 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 4338 3700 50  0001 C CNN
F 3 "~" H 4300 3850 50  0001 C CNN
	1    4300 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5BDC9A24
P 4600 3850
F 0 "R7" H 4670 3896 50  0000 L CNN
F 1 "100k" H 4670 3805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4530 3850 50  0001 C CNN
F 3 "~" H 4600 3850 50  0001 C CNN
	1    4600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4100 4300 4100
Wire Wire Line
	4300 4100 4300 4000
Wire Wire Line
	4600 4000 4600 4100
Wire Wire Line
	4600 4100 4300 4100
Connection ~ 4300 4100
Wire Wire Line
	4300 3700 4300 3550
Connection ~ 4300 3550
Wire Wire Line
	4600 3550 4600 3700
Wire Wire Line
	4300 3550 4600 3550
Wire Wire Line
	1600 4500 1600 3750
Wire Wire Line
	2000 3750 2000 4500
$Comp
L Device:C C2
U 1 1 5BDD86CD
P 3600 3000
F 0 "C2" H 3715 3046 50  0000 L CNN
F 1 "100n" H 3715 2955 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3638 2850 50  0001 C CNN
F 3 "~" H 3600 3000 50  0001 C CNN
	1    3600 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3150 3600 3200
Wire Wire Line
	3600 2850 3250 2850
Wire Wire Line
	3250 2850 3250 2800
Wire Wire Line
	3250 2850 3250 3300
Connection ~ 3250 2850
$Comp
L Device:R R8
U 1 1 5BDE1341
P 5300 3550
F 0 "R8" V 5093 3550 50  0000 C CNN
F 1 "1k" V 5184 3550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5230 3550 50  0001 C CNN
F 3 "~" H 5300 3550 50  0001 C CNN
	1    5300 3550
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 5BDE1432
P 5600 3850
F 0 "C4" H 5715 3896 50  0000 L CNN
F 1 "100n" H 5715 3805 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 5638 3700 50  0001 C CNN
F 3 "~" H 5600 3850 50  0001 C CNN
	1    5600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3550 4850 3550
Connection ~ 4600 3550
Wire Wire Line
	5450 3550 5600 3550
Wire Wire Line
	5600 3550 5600 3700
Wire Wire Line
	5600 4000 5600 4100
Wire Wire Line
	5600 3550 5950 3550
Connection ~ 5600 3550
$Comp
L power:GND #PWR08
U 1 1 5BDEDDA6
P 2500 4950
F 0 "#PWR08" H 2500 4700 50  0001 C CNN
F 1 "GND" H 2500 4800 50  0000 C CNN
F 2 "" H 2500 4950 50  0000 C CNN
F 3 "" H 2500 4950 50  0000 C CNN
	1    2500 4950
	1    0    0    -1  
$EndComp
Text Label 5950 3550 0    50   ~ 0
A0
$Comp
L Device:R_POT RV1
U 1 1 5BDEE86B
P 2700 4650
F 0 "RV1" H 2631 4696 50  0000 R CNN
F 1 "1k" H 2631 4605 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_ACP_CA9-V10_Vertical" H 2700 4650 50  0001 C CNN
F 3 "~" H 2700 4650 50  0001 C CNN
	1    2700 4650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2500 4500 2500 4650
Wire Wire Line
	1600 4500 2000 4500
Wire Wire Line
	2700 4150 2700 4100
Connection ~ 2700 4100
Wire Wire Line
	2550 4650 2500 4650
$Comp
L Device:R R4
U 1 1 5BE19696
P 2700 4300
F 0 "R4" H 2770 4346 50  0000 L CNN
F 1 "100" H 2770 4255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2630 4300 50  0001 C CNN
F 3 "~" H 2700 4300 50  0001 C CNN
	1    2700 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4450 2700 4500
Wire Wire Line
	2500 4500 2000 4500
Connection ~ 2000 4500
Connection ~ 2500 4650
Wire Wire Line
	2500 4650 2500 4800
$Comp
L Device:R R9
U 1 1 5BE2B84C
P 2450 3400
F 0 "R9" V 2243 3400 50  0000 C CNN
F 1 "0" V 2334 3400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2380 3400 50  0001 C CNN
F 3 "~" H 2450 3400 50  0001 C CNN
	1    2450 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 3400 2000 3400
Connection ~ 2000 3400
Wire Wire Line
	2600 3400 2850 3400
$Comp
L Device:R R10
U 1 1 5BE36168
P 5000 3550
F 0 "R10" V 4793 3550 50  0000 C CNN
F 1 "0" V 4884 3550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4930 3550 50  0001 C CNN
F 3 "~" H 5000 3550 50  0001 C CNN
	1    5000 3550
	0    1    1    0   
$EndComp
Connection ~ 4600 4100
Wire Wire Line
	4600 4100 5600 4100
Wire Wire Line
	2700 4800 2500 4800
Connection ~ 2500 4800
Wire Wire Line
	2500 4800 2500 4950
$Comp
L power:+3.3V #PWR09
U 1 1 5BE435C9
P 2350 1100
F 0 "#PWR09" H 2350 950 50  0001 C CNN
F 1 "+3.3V" V 2350 1350 50  0000 C CNN
F 2 "" H 2350 1100 50  0000 C CNN
F 3 "" H 2350 1100 50  0000 C CNN
	1    2350 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1100 2350 1600
Wire Wire Line
	2350 1600 2150 1600
Wire Wire Line
	2500 1500 2150 1500
$Comp
L Switch:SW_Push_SPDT SW2
U 1 1 5BE51B6C
P 8950 5100
F 0 "SW2" H 8950 5385 50  0000 C CNN
F 1 "SW_Push_SPDT" H 8950 5294 50  0000 C CNN
F 2 "" H 8950 5100 50  0001 C CNN
F 3 "" H 8950 5100 50  0001 C CNN
	1    8950 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1400 2250 1400
Wire Wire Line
	2250 1400 2250 1800
Text Label 2500 1300 0    50   ~ 0
Recharge
Wire Wire Line
	2500 1300 2150 1300
Text Label 8250 5100 2    50   ~ 0
Recharge
Wire Wire Line
	8250 5100 8750 5100
NoConn ~ 9150 5200
$Comp
L power:+3.3V #PWR010
U 1 1 5BE6964A
P 9800 4850
F 0 "#PWR010" H 9800 4700 50  0001 C CNN
F 1 "+3.3V" V 9800 5100 50  0000 C CNN
F 2 "" H 9800 4850 50  0000 C CNN
F 3 "" H 9800 4850 50  0000 C CNN
	1    9800 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 5000 9800 5000
Wire Wire Line
	9800 4850 9800 5000
$EndSCHEMATC
