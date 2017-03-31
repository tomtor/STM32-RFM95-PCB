EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32f103c8t6-module-china
LIBS:RFM95
LIBS:STM-LORA-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "RFM95"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L GND #PWR01
U 1 1 58DE3B62
P 8650 3750
F 0 "#PWR01" H 8650 3500 50  0001 C CNN
F 1 "GND" H 8650 3600 50  0000 C CNN
F 2 "" H 8650 3750 50  0000 C CNN
F 3 "" H 8650 3750 50  0000 C CNN
	1    8650 3750
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR02
U 1 1 58DE3B7A
P 8650 2900
F 0 "#PWR02" H 8650 2750 50  0001 C CNN
F 1 "VCC" H 8650 3050 50  0000 C CNN
F 2 "" H 8650 2900 50  0000 C CNN
F 3 "" H 8650 2900 50  0000 C CNN
	1    8650 2900
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG03
U 1 1 58DE3B92
P 9200 3650
F 0 "#FLG03" H 9200 3745 50  0001 C CNN
F 1 "PWR_FLAG" H 9200 3830 50  0000 C CNN
F 2 "" H 9200 3650 50  0000 C CNN
F 3 "" H 9200 3650 50  0000 C CNN
	1    9200 3650
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG04
U 1 1 58DE3BAA
P 9200 2900
F 0 "#FLG04" H 9200 2995 50  0001 C CNN
F 1 "PWR_FLAG" H 9200 3080 50  0000 C CNN
F 2 "" H 9200 2900 50  0000 C CNN
F 3 "" H 9200 2900 50  0000 C CNN
	1    9200 2900
	1    0    0    -1  
$EndComp
$Comp
L stm32f103c8t6-module-china U1
U 1 1 58DE43B8
P 2850 3650
F 0 "U1" H 2850 5000 60  0000 C CNN
F 1 "stm32f103c8t6-module-china" V 2850 3650 60  0000 C CNN
F 2 "STM32China:stm32f103c8t6-module-china" H 2850 1950 60  0001 C CNN
F 3 "" H 2450 3900 60  0000 C CNN
	1    2850 3650
	1    0    0    -1  
$EndComp
$Comp
L RFM95HW U2
U 1 1 58DE43F7
P 5250 3300
F 0 "U2" H 4900 3550 40  0000 C CNN
F 1 "RFM95HW" H 5500 2550 40  0000 C CNN
F 2 "RFM:RFM95" H 5250 3300 30  0001 C CIN
F 3 "" H 5250 3300 60  0000 C CNN
	1    5250 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 58DE5F24
P 3900 2250
F 0 "#PWR05" H 3900 2000 50  0001 C CNN
F 1 "GND" H 3900 2100 50  0000 C CNN
F 2 "" H 3900 2250 50  0000 C CNN
F 3 "" H 3900 2250 50  0000 C CNN
	1    3900 2250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR06
U 1 1 58DE5F62
P 3900 2400
F 0 "#PWR06" H 3900 2150 50  0001 C CNN
F 1 "GND" H 3900 2250 50  0000 C CNN
F 2 "" H 3900 2400 50  0000 C CNN
F 3 "" H 3900 2400 50  0000 C CNN
	1    3900 2400
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR07
U 1 1 58DE5FA0
P 1800 4950
F 0 "#PWR07" H 1800 4700 50  0001 C CNN
F 1 "GND" H 1800 4800 50  0000 C CNN
F 2 "" H 1800 4950 50  0000 C CNN
F 3 "" H 1800 4950 50  0000 C CNN
	1    1800 4950
	0    1    1    0   
$EndComp
$Comp
L GND #PWR08
U 1 1 58DE6092
P 5150 4150
F 0 "#PWR08" H 5150 3900 50  0001 C CNN
F 1 "GND" H 5150 4000 50  0000 C CNN
F 2 "" H 5150 4150 50  0000 C CNN
F 3 "" H 5150 4150 50  0000 C CNN
	1    5150 4150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR09
U 1 1 58DE60AC
P 5250 4150
F 0 "#PWR09" H 5250 3900 50  0001 C CNN
F 1 "GND" H 5250 4000 50  0000 C CNN
F 2 "" H 5250 4150 50  0000 C CNN
F 3 "" H 5250 4150 50  0000 C CNN
	1    5250 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 58DE60C6
P 5350 4150
F 0 "#PWR010" H 5350 3900 50  0001 C CNN
F 1 "GND" H 5350 4000 50  0000 C CNN
F 2 "" H 5350 4150 50  0000 C CNN
F 3 "" H 5350 4150 50  0000 C CNN
	1    5350 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8650 3750 9200 3750
Wire Wire Line
	9200 3750 9200 3650
Wire Wire Line
	8650 2900 9200 2900
Wire Wire Line
	3900 3450 4700 3450
Wire Wire Line
	3900 3600 4700 3600
Wire Wire Line
	4700 3600 4700 3550
Wire Wire Line
	4700 3650 3900 3650
Wire Wire Line
	3900 3650 3900 3750
Wire Wire Line
	3900 2550 5250 2550
Wire Wire Line
	5250 2550 5250 2950
Wire Wire Line
	4700 3350 3900 3350
Wire Wire Line
	3900 3350 3900 3300
Wire Wire Line
	4700 3850 4350 3850
Wire Wire Line
	4350 3850 4350 3150
Wire Wire Line
	4350 3150 3900 3150
Wire Wire Line
	5800 3300 6150 3300
Wire Wire Line
	6150 3300 6150 4450
Wire Wire Line
	6150 4450 4400 4450
Wire Wire Line
	4400 4450 4400 4050
Wire Wire Line
	4400 4050 3900 4050
Wire Wire Line
	5800 3400 6000 3400
Wire Wire Line
	6000 3400 6000 5500
Wire Wire Line
	6000 5500 1350 5500
Wire Wire Line
	1350 5500 1350 4050
Wire Wire Line
	1350 4050 1800 4050
Text Label 4450 2550 0    60   ~ 0
3V3
Wire Wire Line
	1450 5100 1800 5100
Text Label 1650 5100 3    60   ~ 0
3V3
$EndSCHEMATC
