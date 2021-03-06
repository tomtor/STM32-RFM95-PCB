EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "STM32 RFM95 Proto Board"
Date "2017-06-10"
Rev "0.3"
Comp "http://v7f.eu"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:VCC #PWR01
U 1 1 58DE3B7A
P 7750 1900
F 0 "#PWR01" H 7750 1750 50  0001 C CNN
F 1 "VCC" H 7750 2050 50  0000 C CNN
F 2 "" H 7750 1900 50  0000 C CNN
F 3 "" H 7750 1900 50  0000 C CNN
	1    7750 1900
	0    -1   -1   0   
$EndComp
$Comp
L RFM95:RFM95HW U2
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
L power:GND #PWR04
U 1 1 58DE5FA0
P 1700 4950
F 0 "#PWR04" H 1700 4700 50  0001 C CNN
F 1 "GND" H 1700 4800 50  0000 C CNN
F 2 "" H 1700 4950 50  0000 C CNN
F 3 "" H 1700 4950 50  0000 C CNN
	1    1700 4950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 58DE6092
P 5150 4150
F 0 "#PWR05" H 5150 3900 50  0001 C CNN
F 1 "GND" H 5150 4000 50  0000 C CNN
F 2 "" H 5150 4150 50  0000 C CNN
F 3 "" H 5150 4150 50  0000 C CNN
	1    5150 4150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 58DE60AC
P 5250 4150
F 0 "#PWR06" H 5250 3900 50  0001 C CNN
F 1 "GND" H 5250 4000 50  0000 C CNN
F 2 "" H 5250 4150 50  0000 C CNN
F 3 "" H 5250 4150 50  0000 C CNN
	1    5250 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 58DE60C6
P 5350 4150
F 0 "#PWR07" H 5350 3900 50  0001 C CNN
F 1 "GND" H 5350 4000 50  0000 C CNN
F 2 "" H 5350 4150 50  0000 C CNN
F 3 "" H 5350 4150 50  0000 C CNN
	1    5350 4150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 58DF686D
P 7550 4500
F 0 "#PWR08" H 7550 4250 50  0001 C CNN
F 1 "GND" H 7550 4350 50  0000 C CNN
F 2 "" H 7550 4500 50  0000 C CNN
F 3 "" H 7550 4500 50  0000 C CNN
	1    7550 4500
	0    1    1    0   
$EndComp
$Comp
L STM-LORA-rescue:R R2
U 1 1 58DF68CB
P 8900 3950
F 0 "R2" V 8980 3950 50  0000 C CNN
F 1 "100k" V 8900 3950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" V 8830 3950 50  0001 C CNN
F 3 "" H 8900 3950 50  0000 C CNN
	1    8900 3950
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:R R1
U 1 1 58DF68F7
P 8900 3650
F 0 "R1" V 8980 3650 50  0000 C CNN
F 1 "100k" V 8900 3650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" V 8830 3650 50  0001 C CNN
F 3 "" H 8900 3650 50  0000 C CNN
	1    8900 3650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR09
U 1 1 58DF69FC
P 8900 3500
F 0 "#PWR09" H 8900 3350 50  0001 C CNN
F 1 "VCC" H 8900 3650 50  0000 C CNN
F 2 "" H 8900 3500 50  0000 C CNN
F 3 "" H 8900 3500 50  0000 C CNN
	1    8900 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 58DF6A18
P 8900 4100
F 0 "#PWR010" H 8900 3850 50  0001 C CNN
F 1 "GND" H 8900 3950 50  0000 C CNN
F 2 "" H 8900 4100 50  0000 C CNN
F 3 "" H 8900 4100 50  0000 C CNN
	1    8900 4100
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:D_Schottky D1
U 1 1 58DF6C86
P 8800 2400
F 0 "D1" H 8800 2500 50  0000 C CNN
F 1 "BAT85" H 8800 2300 50  0000 C CNN
F 2 "Diodes_THT:D_DO-35_SOD27_P2.54mm_Vertical_AnodeUp" H 8800 2400 50  0001 C CNN
F 3 "" H 8800 2400 50  0000 C CNN
	1    8800 2400
	0    1    1    0   
$EndComp
Text Notes 9600 2950 0    60   ~ 0
Solar Panel (4V - ..)
Text Notes 9550 2050 0    60   ~ 0
2.4V (2x AA(A) NiMH) or\n3.6V (3x AA(A) NiMH) or ...
$Comp
L power:PWR_FLAG #FLG011
U 1 1 58DF7179
P 8850 4500
F 0 "#FLG011" H 8850 4595 50  0001 C CNN
F 1 "PWR_FLAG" H 8850 4680 50  0000 C CNN
F 2 "" H 8850 4500 50  0000 C CNN
F 3 "" H 8850 4500 50  0000 C CNN
	1    8850 4500
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG012
U 1 1 58DF724A
P 8800 1900
F 0 "#FLG012" H 8800 1995 50  0001 C CNN
F 1 "PWR_FLAG" H 8800 2080 50  0000 C CNN
F 2 "" H 8800 1900 50  0000 C CNN
F 3 "" H 8800 1900 50  0000 C CNN
	1    8800 1900
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG013
U 1 1 58DF72A1
P 9000 2850
F 0 "#FLG013" H 9000 2945 50  0001 C CNN
F 1 "PWR_FLAG" H 9000 3030 50  0000 C CNN
F 2 "" H 9000 2850 50  0000 C CNN
F 3 "" H 9000 2850 50  0000 C CNN
	1    9000 2850
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR014
U 1 1 58DF7708
P 1800 4800
F 0 "#PWR014" H 1800 4650 50  0001 C CNN
F 1 "VCC" H 1800 4950 50  0000 C CNN
F 2 "" H 1800 4800 50  0000 C CNN
F 3 "" H 1800 4800 50  0000 C CNN
	1    1800 4800
	0    -1   -1   0   
$EndComp
NoConn ~ 3900 4800
NoConn ~ 5800 3500
NoConn ~ 5800 3600
NoConn ~ 5800 3700
NoConn ~ 5800 3800
Text Label 1550 5100 3    60   ~ 0
3V3
$Comp
L STM-LORA-rescue:CONN_01X02_MALE J1
U 1 1 58DFA0FA
P 9400 2000
F 0 "J1" H 9400 2175 50  0000 C CNN
F 1 "Batt" H 9400 1800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9400 2100 50  0001 C CNN
F 3 "" H 9400 2100 50  0001 C CNN
	1    9400 2000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 58DFA43D
P 9100 2100
F 0 "#PWR017" H 9100 1850 50  0001 C CNN
F 1 "GND" H 9100 1950 50  0000 C CNN
F 2 "" H 9100 2100 50  0001 C CNN
F 3 "" H 9100 2100 50  0001 C CNN
	1    9100 2100
	0    1    1    0   
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X02_MALE J2
U 1 1 58DFA467
P 9400 2950
F 0 "J2" H 9400 3125 50  0000 C CNN
F 1 "Solar" H 9400 2750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9400 3050 50  0001 C CNN
F 3 "" H 9400 3050 50  0001 C CNN
	1    9400 2950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 58DFA51A
P 9100 3050
F 0 "#PWR018" H 9100 2800 50  0001 C CNN
F 1 "GND" H 9100 2900 50  0000 C CNN
F 2 "" H 9100 3050 50  0001 C CNN
F 3 "" H 9100 3050 50  0001 C CNN
	1    9100 3050
	0    1    1    0   
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X17 J3
U 1 1 58E01D61
P 650 1650
F 0 "J3" H 650 2550 50  0000 C CNN
F 1 "CONN_01X17" V 750 1650 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x17_Pitch2.54mm" H 650 1650 50  0001 C CNN
F 3 "" H 650 1650 50  0001 C CNN
	1    650  1650
	-1   0    0    1   
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X03 J4
U 1 1 58E028DC
P 650 5000
F 0 "J4" H 650 5200 50  0000 C CNN
F 1 "CONN_01X03" V 750 5000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 650 5000 50  0001 C CNN
F 3 "" H 650 5000 50  0001 C CNN
	1    650  5000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 58E02B12
P 850 5000
F 0 "#PWR019" H 850 4750 50  0001 C CNN
F 1 "GND" H 850 4850 50  0000 C CNN
F 2 "" H 850 5000 50  0001 C CNN
F 3 "" H 850 5000 50  0001 C CNN
	1    850  5000
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR020
U 1 1 58E02B47
P 850 4900
F 0 "#PWR020" H 850 4750 50  0001 C CNN
F 1 "VCC" H 850 5050 50  0000 C CNN
F 2 "" H 850 4900 50  0001 C CNN
F 3 "" H 850 4900 50  0001 C CNN
	1    850  4900
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X03 J5
U 1 1 58E02E8B
P 6400 2050
F 0 "J5" H 6400 2250 50  0000 C CNN
F 1 "CONN_01X03" V 6500 2050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6400 2050 50  0001 C CNN
F 3 "" H 6400 2050 50  0001 C CNN
	1    6400 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 58E02F25
P 6200 1950
F 0 "#PWR021" H 6200 1700 50  0001 C CNN
F 1 "GND" H 6200 1800 50  0000 C CNN
F 2 "" H 6200 1950 50  0001 C CNN
F 3 "" H 6200 1950 50  0001 C CNN
	1    6200 1950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 58E02F54
P 6200 2050
F 0 "#PWR022" H 6200 1800 50  0001 C CNN
F 1 "GND" H 6200 1900 50  0000 C CNN
F 2 "" H 6200 2050 50  0001 C CNN
F 3 "" H 6200 2050 50  0001 C CNN
	1    6200 2050
	0    1    1    0   
$EndComp
Text Label 5850 2150 0    60   ~ 0
3V3
$Comp
L STM-LORA-rescue:CONN_01X01 J6
U 1 1 58E096FD
P 4100 5100
F 0 "J6" H 4100 5200 50  0000 C CNN
F 1 "CONN_01X01" V 4200 5100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 4100 5100 50  0001 C CNN
F 3 "" H 4100 5100 50  0001 C CNN
	1    4100 5100
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X14 J7
U 1 1 58E0990E
P 5400 5500
F 0 "J7" H 5400 6250 50  0000 C CNN
F 1 "CONN_01X14" V 5500 5500 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x14_Pitch2.54mm" H 5400 5500 50  0001 C CNN
F 3 "" H 5400 5500 50  0001 C CNN
	1    5400 5500
	1    0    0    -1  
$EndComp
Text Label 3900 3000 0    60   ~ 0
P35
Text Label 3900 3150 0    60   ~ 0
P34
Text Label 3900 3450 0    60   ~ 0
P32
Text Label 3900 3750 0    60   ~ 0
P30
Text Label 3900 3300 0    60   ~ 0
P33
Text Label 3900 3600 0    60   ~ 0
P31
Text Label 3900 3900 0    60   ~ 0
P29
Text Label 3900 4050 0    60   ~ 0
P28
Text Label 3900 4200 0    60   ~ 0
P27
Text Label 3900 4500 0    60   ~ 0
P25
Text Label 3900 4950 0    60   ~ 0
P22
NoConn ~ 3900 4650
Text Label 5200 4850 2    60   ~ 0
P37
Text Label 5200 4950 2    60   ~ 0
P36
Text Label 5200 5050 2    60   ~ 0
P35
Text Label 5200 5150 2    60   ~ 0
P34
Text Label 5200 5250 2    60   ~ 0
P33
Text Label 5200 5350 2    60   ~ 0
P32
Text Label 5200 5450 2    60   ~ 0
P31
Text Label 5200 5550 2    60   ~ 0
P30
Text Label 5200 5650 2    60   ~ 0
P29
Text Label 5200 5750 2    60   ~ 0
P28
Text Label 5200 5850 2    60   ~ 0
P27
Text Label 5200 6050 2    60   ~ 0
P25
Text Label 5200 6150 2    60   ~ 0
P22
Text Label 3900 4350 0    60   ~ 0
P26
Text Label 5200 5950 2    60   ~ 0
P26
Text Label 8400 3800 0    60   ~ 0
P26
$Comp
L power:PWR_FLAG #FLG023
U 1 1 58E0A7A9
P 3900 5100
F 0 "#FLG023" H 3900 5175 50  0001 C CNN
F 1 "PWR_FLAG" V 3900 5250 50  0000 L CNN
F 2 "" H 3900 5100 50  0001 C CNN
F 3 "" H 3900 5100 50  0001 C CNN
	1    3900 5100
	-1   0    0    1   
$EndComp
Text Label 8800 2850 2    60   ~ 0
Solar
$Comp
L STM-LORA-rescue:CONN_01X05 J8
U 1 1 58E0ADB1
P 3150 7150
F 0 "J8" H 3150 7450 50  0000 C CNN
F 1 "CONN_01X05" V 3250 7150 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x05_Pitch2.54mm" H 3150 7150 50  0001 C CNN
F 3 "" H 3150 7150 50  0001 C CNN
	1    3150 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1900 8800 1900
Wire Wire Line
	3900 3600 4700 3600
Wire Wire Line
	4700 3600 4700 3550
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
	6000 3400 6000 6500
Wire Wire Line
	6000 6500 1100 6500
Wire Wire Line
	1100 6500 1100 4050
Wire Wire Line
	1100 4050 1800 4050
Wire Wire Line
	8400 3800 8900 3800
Connection ~ 8900 3800
Connection ~ 8800 1900
Wire Wire Line
	8800 1900 8800 2250
Wire Wire Line
	8800 2850 9000 2850
Wire Wire Line
	7550 4500 8850 4500
Connection ~ 9000 2850
Wire Wire Line
	8800 2850 8800 2550
Wire Wire Line
	3900 3300 4200 3300
Wire Wire Line
	4200 3300 4200 3350
Wire Wire Line
	4200 3350 4700 3350
Wire Wire Line
	4700 3450 3900 3450
Wire Wire Line
	1700 4950 1800 4950
Wire Wire Line
	5250 2550 5250 2950
Wire Wire Line
	4700 3650 4700 3750
Wire Wire Line
	4700 3750 3900 3750
Wire Wire Line
	4800 2100 4800 2550
Wire Wire Line
	1800 4650 850  4650
Wire Wire Line
	850  4650 850  2450
Wire Wire Line
	1800 4500 900  4500
Wire Wire Line
	900  4500 900  2350
Wire Wire Line
	900  2350 850  2350
Wire Wire Line
	1800 4350 950  4350
Wire Wire Line
	950  4350 950  2250
Wire Wire Line
	950  2250 850  2250
Wire Wire Line
	1800 4200 1000 4200
Wire Wire Line
	1000 4200 1000 2150
Wire Wire Line
	1000 2150 850  2150
Connection ~ 1100 4050
Wire Wire Line
	1100 2050 850  2050
Wire Wire Line
	1800 3900 1150 3900
Wire Wire Line
	1150 3900 1150 1950
Wire Wire Line
	1150 1950 850  1950
Wire Wire Line
	1800 3750 1200 3750
Wire Wire Line
	1200 3750 1200 1850
Wire Wire Line
	1200 1850 850  1850
Wire Wire Line
	1800 3600 1250 3600
Wire Wire Line
	1250 3600 1250 1750
Wire Wire Line
	1250 1750 850  1750
Wire Wire Line
	1800 3450 1300 3450
Wire Wire Line
	1300 3450 1300 1650
Wire Wire Line
	1300 1650 850  1650
Wire Wire Line
	1800 3300 1350 3300
Wire Wire Line
	1350 3300 1350 1550
Wire Wire Line
	1350 1550 850  1550
Wire Wire Line
	1400 1450 1400 3150
Wire Wire Line
	850  1450 1400 1450
Wire Wire Line
	850  1350 1450 1350
Wire Wire Line
	1450 1350 1450 3000
Wire Wire Line
	1450 3000 1800 3000
Wire Wire Line
	1800 2850 1500 2850
Wire Wire Line
	1500 2850 1500 1250
Wire Wire Line
	1500 1250 850  1250
Wire Wire Line
	850  1150 1550 1150
Wire Wire Line
	1550 1150 1550 2700
Wire Wire Line
	1550 2700 1800 2700
Wire Wire Line
	1800 2550 1600 2550
Wire Wire Line
	1600 2550 1600 1050
Wire Wire Line
	1600 1050 850  1050
Wire Wire Line
	850  950  1650 950 
Wire Wire Line
	1650 950  1650 2400
Wire Wire Line
	1650 2400 1800 2400
Wire Wire Line
	1800 2250 1700 2250
Wire Wire Line
	1700 2250 1700 850 
Wire Wire Line
	1700 850  850  850 
Wire Wire Line
	850  5100 1800 5100
Wire Wire Line
	1400 3150 1800 3150
Wire Wire Line
	5750 2150 6200 2150
Wire Wire Line
	2950 6950 2950 7050
Connection ~ 2950 7050
Connection ~ 2950 7150
Connection ~ 2950 7250
$Comp
L power:GND #PWR024
U 1 1 58E0B20A
P 2950 7350
F 0 "#PWR024" H 2950 7100 50  0001 C CNN
F 1 "GND" H 2950 7200 50  0000 C CNN
F 2 "" H 2950 7350 50  0001 C CNN
F 3 "" H 2950 7350 50  0001 C CNN
	1    2950 7350
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X05 J9
U 1 1 58E0B40A
P 3700 7150
F 0 "J9" H 3700 7450 50  0000 C CNN
F 1 "CONN_01X05" V 3800 7150 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x05_Pitch2.54mm" H 3700 7150 50  0001 C CNN
F 3 "" H 3700 7150 50  0001 C CNN
	1    3700 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6950 3500 7050
Connection ~ 3500 7050
Connection ~ 3500 7150
Connection ~ 3500 7250
Text Label 3500 7350 3    60   ~ 0
3V3
Text Notes 6550 5000 0    60   ~ 0
Exp. Area:
$Comp
L STM-LORA-rescue:CONN_02X07 J10
U 1 1 58E22FE3
P 7400 5250
F 0 "J10" H 7400 5650 50  0000 C CNN
F 1 "CONN_02X07" V 7400 5250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x07_Pitch2.54mm" H 7400 4050 50  0001 C CNN
F 3 "" H 7400 4050 50  0001 C CNN
	1    7400 5250
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_02X09 J11
U 1 1 58E2304D
P 9150 5350
F 0 "J11" H 9150 5850 50  0000 C CNN
F 1 "CONN_02X09" V 9150 5350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x09_Pitch2.54mm" H 9150 4150 50  0001 C CNN
F 3 "" H 9150 4150 50  0001 C CNN
	1    9150 5350
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X02 J12
U 1 1 58E237FE
P 7950 5000
F 0 "J12" H 7950 5150 50  0000 C CNN
F 1 "CONN_01X02" V 8050 5000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7950 5000 50  0001 C CNN
F 3 "" H 7950 5000 50  0001 C CNN
	1    7950 5000
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X02 J13
U 1 1 58E23852
P 7950 5600
F 0 "J13" H 7950 5750 50  0000 C CNN
F 1 "CONN_01X02" V 8050 5600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7950 5600 50  0001 C CNN
F 3 "" H 7950 5600 50  0001 C CNN
	1    7950 5600
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X02 J14
U 1 1 58E238BA
P 7950 6200
F 0 "J14" H 7950 6350 50  0000 C CNN
F 1 "CONN_01X02" V 8050 6200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7950 6200 50  0001 C CNN
F 3 "" H 7950 6200 50  0001 C CNN
	1    7950 6200
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X03 J15
U 1 1 58E239DC
P 8500 5050
F 0 "J15" H 8500 5250 50  0000 C CNN
F 1 "CONN_01X03" V 8600 5050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 8500 5050 50  0001 C CNN
F 3 "" H 8500 5050 50  0001 C CNN
	1    8500 5050
	1    0    0    -1  
$EndComp
NoConn ~ 7750 4950
NoConn ~ 7750 5050
NoConn ~ 7750 5550
NoConn ~ 7750 5650
NoConn ~ 7750 6150
NoConn ~ 7750 6250
NoConn ~ 8300 4950
NoConn ~ 8300 5050
NoConn ~ 8300 5150
$Comp
L STM-LORA-rescue:CONN_01X01 J16
U 1 1 593BCF25
P 4700 2950
F 0 "J16" H 4700 3050 50  0000 C CNN
F 1 "CONN_01X01" V 4800 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 4700 2950 50  0001 C CNN
F 3 "" H 4700 2950 50  0001 C CNN
	1    4700 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8900 4950 9400 4950
Wire Wire Line
	8900 5050 9400 5050
Wire Wire Line
	9400 5150 8900 5150
Wire Wire Line
	8900 5250 9400 5250
Wire Wire Line
	9400 5350 8900 5350
Wire Wire Line
	8900 5450 9400 5450
Wire Wire Line
	9400 5550 8900 5550
Wire Wire Line
	8900 5650 9400 5650
Wire Wire Line
	8900 5750 9400 5750
Wire Wire Line
	7150 4950 7650 4950
Wire Wire Line
	7650 5050 7150 5050
Wire Wire Line
	7150 5150 7650 5150
Wire Wire Line
	7150 5250 7650 5250
Wire Wire Line
	7150 5350 7650 5350
Wire Wire Line
	7650 5450 7150 5450
Wire Wire Line
	7150 5550 7650 5550
Wire Wire Line
	8800 1900 9100 1900
Wire Wire Line
	9000 2850 9100 2850
Wire Wire Line
	4800 2550 5250 2550
Wire Wire Line
	1100 4050 1100 2050
Wire Wire Line
	2950 7050 2950 7150
Wire Wire Line
	2950 7150 2950 7250
Wire Wire Line
	2950 7250 2950 7350
Wire Wire Line
	3500 7050 3500 7150
Wire Wire Line
	3500 7150 3500 7250
Wire Wire Line
	3500 7250 3500 7350
Connection ~ 4800 2550
Wire Wire Line
	4400 2550 4800 2550
Wire Wire Line
	4400 2550 4400 2100
Connection ~ 4400 2550
Wire Wire Line
	3900 2550 4400 2550
Text Label 3900 2850 0    60   ~ 0
P36
Text Label 3900 2700 0    60   ~ 0
P37
$Comp
L power:GND #PWR016
U 1 1 58DF9BB7
P 4400 1800
F 0 "#PWR016" H 4400 1550 50  0001 C CNN
F 1 "GND" H 4400 1650 50  0000 C CNN
F 2 "" H 4400 1800 50  0000 C CNN
F 3 "" H 4400 1800 50  0000 C CNN
	1    4400 1800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 58DF9B90
P 4800 1800
F 0 "#PWR015" H 4800 1550 50  0001 C CNN
F 1 "GND" H 4800 1650 50  0000 C CNN
F 2 "" H 4800 1800 50  0000 C CNN
F 3 "" H 4800 1800 50  0000 C CNN
	1    4800 1800
	-1   0    0    1   
$EndComp
Text Label 4550 2550 0    60   ~ 0
3V3
$Comp
L STM-LORA-rescue:C C2
U 1 1 58DF7A58
P 4800 1950
F 0 "C2" H 4825 2050 50  0000 L CNN
F 1 "100n" H 4825 1850 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 4838 1800 50  0001 C CNN
F 3 "" H 4800 1950 50  0000 C CNN
	1    4800 1950
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:CP C1
U 1 1 58DF7A11
P 4400 1950
F 0 "C1" H 4425 2050 50  0000 L CNN
F 1 "100µ" H 4425 1850 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D6.3mm_P2.50mm" H 4438 1800 50  0001 C CNN
F 3 "" H 4400 1950 50  0000 C CNN
	1    4400 1950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 58DE5F62
P 3900 2400
F 0 "#PWR03" H 3900 2150 50  0001 C CNN
F 1 "GND" H 3900 2250 50  0000 C CNN
F 2 "" H 3900 2400 50  0000 C CNN
F 3 "" H 3900 2400 50  0000 C CNN
	1    3900 2400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 58DE5F24
P 3900 2250
F 0 "#PWR02" H 3900 2000 50  0001 C CNN
F 1 "GND" H 3900 2100 50  0000 C CNN
F 2 "" H 3900 2250 50  0000 C CNN
F 3 "" H 3900 2250 50  0000 C CNN
	1    3900 2250
	0    -1   -1   0   
$EndComp
$Comp
L stm32f103c8t6-module-china:stm32f103c8t6-module-china U1
U 1 1 58DE43B8
P 2850 3650
F 0 "U1" H 2850 5000 60  0000 C CNN
F 1 "stm32f103c8t6-module-china" V 2850 3650 60  0000 C CNN
F 2 "STM32China:stm32f103c8t6-module-china" H 2850 1950 60  0001 C CNN
F 3 "" H 2450 3900 60  0000 C CNN
	1    2850 3650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
