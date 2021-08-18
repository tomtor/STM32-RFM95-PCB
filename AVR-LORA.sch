EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "AVR RFM95 Board"
Date "2021-08-17"
Rev "0.1"
Comp "http://v7f.eu"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:VCC #PWR01
U 1 1 58DE3B7A
P 7650 1200
F 0 "#PWR01" H 7650 1050 50  0001 C CNN
F 1 "VCC" H 7650 1350 50  0000 C CNN
F 2 "" H 7650 1200 50  0000 C CNN
F 3 "" H 7650 1200 50  0000 C CNN
	1    7650 1200
	0    -1   -1   0   
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
P 3400 1300
F 0 "#PWR08" H 3400 1050 50  0001 C CNN
F 1 "GND" H 3400 1150 50  0000 C CNN
F 2 "" H 3400 1300 50  0000 C CNN
F 3 "" H 3400 1300 50  0000 C CNN
	1    3400 1300
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:R R2
U 1 1 58DF68CB
P 8300 3050
F 0 "R2" V 8380 3050 50  0000 C CNN
F 1 "100k" V 8300 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8230 3050 50  0001 C CNN
F 3 "" H 8300 3050 50  0000 C CNN
	1    8300 3050
	0    -1   -1   0   
$EndComp
$Comp
L STM-LORA-rescue:R R1
U 1 1 58DF68F7
P 8000 3050
F 0 "R1" V 8080 3050 50  0000 C CNN
F 1 "100k" V 8000 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 7930 3050 50  0001 C CNN
F 3 "" H 8000 3050 50  0000 C CNN
	1    8000 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR09
U 1 1 58DF69FC
P 7850 3050
F 0 "#PWR09" H 7850 2900 50  0001 C CNN
F 1 "VCC" H 7850 3200 50  0000 C CNN
F 2 "" H 7850 3050 50  0000 C CNN
F 3 "" H 7850 3050 50  0000 C CNN
	1    7850 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 58DF6A18
P 8450 3050
F 0 "#PWR010" H 8450 2800 50  0001 C CNN
F 1 "GND" H 8450 2900 50  0000 C CNN
F 2 "" H 8450 3050 50  0000 C CNN
F 3 "" H 8450 3050 50  0000 C CNN
	1    8450 3050
	0    -1   -1   0   
$EndComp
$Comp
L STM-LORA-rescue:D_Schottky D1
U 1 1 58DF6C86
P 8000 1700
F 0 "D1" H 8000 1800 50  0000 C CNN
F 1 "BAT54S" H 8000 1600 50  0000 C CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 8000 1700 50  0001 C CNN
F 3 "" H 8000 1700 50  0000 C CNN
	1    8000 1700
	0    1    1    0   
$EndComp
Text Notes 8800 2250 0    60   ~ 0
Solar Panel (4V - ..)
Text Notes 8750 1350 0    60   ~ 0
2.4V (2x AA(A) NiMH) or\n3.6V (3x AA(A) NiMH) or ...
$Comp
L power:PWR_FLAG #FLG011
U 1 1 58DF7179
P 3400 1100
F 0 "#FLG011" H 3400 1195 50  0001 C CNN
F 1 "PWR_FLAG" H 3400 1280 50  0000 C CNN
F 2 "" H 3400 1100 50  0000 C CNN
F 3 "" H 3400 1100 50  0000 C CNN
	1    3400 1100
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG012
U 1 1 58DF724A
P 8000 1000
F 0 "#FLG012" H 8000 1095 50  0001 C CNN
F 1 "PWR_FLAG" H 8000 1180 50  0000 C CNN
F 2 "" H 8000 1000 50  0000 C CNN
F 3 "" H 8000 1000 50  0000 C CNN
	1    8000 1000
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG013
U 1 1 58DF72A1
P 8250 2150
F 0 "#FLG013" H 8250 2245 50  0001 C CNN
F 1 "PWR_FLAG" H 8250 2330 50  0000 C CNN
F 2 "" H 8250 2150 50  0000 C CNN
F 3 "" H 8250 2150 50  0000 C CNN
	1    8250 2150
	1    0    0    -1  
$EndComp
NoConn ~ 5800 3600
NoConn ~ 5800 3700
NoConn ~ 5800 3800
$Comp
L STM-LORA-rescue:CONN_01X02_MALE J1
U 1 1 58DFA0FA
P 8600 1300
F 0 "J1" H 8600 1475 50  0000 C CNN
F 1 "Batt" H 8600 1100 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8600 1400 50  0001 C CNN
F 3 "" H 8600 1400 50  0001 C CNN
	1    8600 1300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 58DFA43D
P 8300 1400
F 0 "#PWR017" H 8300 1150 50  0001 C CNN
F 1 "GND" H 8300 1250 50  0000 C CNN
F 2 "" H 8300 1400 50  0001 C CNN
F 3 "" H 8300 1400 50  0001 C CNN
	1    8300 1400
	0    1    1    0   
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X02_MALE J2
U 1 1 58DFA467
P 8600 2250
F 0 "J2" H 8600 2425 50  0000 C CNN
F 1 "Solar" H 8600 2050 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8600 2350 50  0001 C CNN
F 3 "" H 8600 2350 50  0001 C CNN
	1    8600 2250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 58DFA51A
P 8300 2350
F 0 "#PWR018" H 8300 2100 50  0001 C CNN
F 1 "GND" H 8300 2200 50  0000 C CNN
F 2 "" H 8300 2350 50  0001 C CNN
F 3 "" H 8300 2350 50  0001 C CNN
	1    8300 2350
	0    1    1    0   
$EndComp
$Comp
L STM-LORA-rescue:CONN_01X14 J4
U 1 1 58E0990E
P 7450 5500
F 0 "J4" H 7450 6250 50  0000 C CNN
F 1 "CONN_01X14" V 7550 5500 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x14_P2.54mm_Vertical" H 7450 5500 50  0001 C CNN
F 3 "" H 7450 5500 50  0001 C CNN
	1    7450 5500
	1    0    0    -1  
$EndComp
Text Label 8150 3550 1    60   ~ 0
D1
Text Label 8000 2150 2    60   ~ 0
Solar
Wire Wire Line
	8150 3550 8150 3050
Connection ~ 8150 3050
Connection ~ 8000 1200
Wire Wire Line
	8000 1200 8000 1550
Wire Wire Line
	8000 2150 8250 2150
Connection ~ 8250 2150
Wire Wire Line
	8000 2150 8000 1850
$Comp
L STM-LORA-rescue:CONN_01X01 J3
U 1 1 593BCF25
P 4700 2950
F 0 "J3" H 4700 3050 50  0000 C CNN
F 1 "CONN_01X01" V 4800 2950 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4700 2950 50  0001 C CNN
F 3 "" H 4700 2950 50  0001 C CNN
	1    4700 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8000 1200 8300 1200
Wire Wire Line
	8250 2150 8300 2150
$Comp
L AVR:AVR128DB28 U2
U 1 1 611C6D25
P 3000 3950
F 0 "U2" H 3000 3850 50  0000 C CNN
F 1 "AVR128DB28" H 3500 2650 50  0000 C CNN
F 2 "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm" H 3050 3150 50  0001 C CIN
F 3 "https://www.microchip.com/content/dam/mchp/documents/MCU08/ProductDocuments/DataSheets/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf" H 3000 3700 50  0001 C CNN
	1    3000 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1300 3400 1100
Wire Wire Line
	7650 1200 8000 1200
Wire Wire Line
	8000 1200 8000 1000
$Comp
L power:GND #PWR0101
U 1 1 611D3C4C
P 2900 5150
F 0 "#PWR0101" H 2900 4900 50  0001 C CNN
F 1 "GND" H 2900 5000 50  0000 C CNN
F 2 "" H 2900 5150 50  0000 C CNN
F 3 "" H 2900 5150 50  0000 C CNN
	1    2900 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5150 2900 5150
Connection ~ 2900 5150
$Comp
L STM-LORA-rescue:C C7
U 1 1 611D5797
P 3500 2500
F 0 "C7" H 3525 2600 50  0000 L CNN
F 1 "100nF" H 3525 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3538 2350 50  0001 C CNN
F 3 "" H 3500 2500 50  0000 C CNN
	1    3500 2500
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:C C8
U 1 1 611D698A
P 3800 2500
F 0 "C8" H 3825 2600 50  0000 L CNN
F 1 "1uF" H 3825 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3838 2350 50  0001 C CNN
F 3 "" H 3800 2500 50  0000 C CNN
	1    3800 2500
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:C C5
U 1 1 611D77DD
P 2900 2500
F 0 "C5" H 2925 2600 50  0000 L CNN
F 1 "100nF" H 2925 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2938 2350 50  0001 C CNN
F 3 "" H 2900 2500 50  0000 C CNN
	1    2900 2500
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:C C6
U 1 1 611D7933
P 3200 2500
F 0 "C6" H 3225 2600 50  0000 L CNN
F 1 "1uF" H 3225 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3238 2350 50  0001 C CNN
F 3 "" H 3200 2500 50  0000 C CNN
	1    3200 2500
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:C C2
U 1 1 611D7A9B
P 2300 2500
F 0 "C2" H 2325 2600 50  0000 L CNN
F 1 "100nF" H 2325 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2338 2350 50  0001 C CNN
F 3 "" H 2300 2500 50  0000 C CNN
	1    2300 2500
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:C C3
U 1 1 611D7BD5
P 2600 2500
F 0 "C3" H 2625 2600 50  0000 L CNN
F 1 "1uF" H 2625 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2638 2350 50  0001 C CNN
F 3 "" H 2600 2500 50  0000 C CNN
	1    2600 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2350 2500 2350
Wire Wire Line
	2900 2350 3050 2350
Wire Wire Line
	3500 2350 3600 2350
Wire Wire Line
	2300 2650 2500 2650
Wire Wire Line
	2900 2650 3000 2650
Wire Wire Line
	3150 2800 3500 2800
Wire Wire Line
	3500 2800 3500 2650
Connection ~ 3500 2650
Wire Wire Line
	3500 2650 3800 2650
Wire Wire Line
	3000 2800 3000 2650
Wire Wire Line
	2900 2800 3000 2800
Connection ~ 3000 2650
Wire Wire Line
	3000 2650 3200 2650
Wire Wire Line
	2800 2800 2500 2800
Wire Wire Line
	2500 2800 2500 2650
Connection ~ 2500 2650
Wire Wire Line
	2500 2650 2600 2650
Text Label 3500 2800 0    60   ~ 0
3V3
Text Label 2550 2800 0    60   ~ 0
3V3
Text Label 3000 2750 0    60   ~ 0
3V3
$Comp
L power:GND #PWR0102
U 1 1 611E7EF5
P 2500 2350
F 0 "#PWR0102" H 2500 2100 50  0001 C CNN
F 1 "GND" H 2500 2200 50  0000 C CNN
F 2 "" H 2500 2350 50  0000 C CNN
F 3 "" H 2500 2350 50  0000 C CNN
	1    2500 2350
	-1   0    0    1   
$EndComp
Connection ~ 2500 2350
Wire Wire Line
	2500 2350 2600 2350
$Comp
L power:GND #PWR0103
U 1 1 611E8650
P 3050 2350
F 0 "#PWR0103" H 3050 2100 50  0001 C CNN
F 1 "GND" H 3050 2200 50  0000 C CNN
F 2 "" H 3050 2350 50  0000 C CNN
F 3 "" H 3050 2350 50  0000 C CNN
	1    3050 2350
	-1   0    0    1   
$EndComp
Connection ~ 3050 2350
Wire Wire Line
	3050 2350 3200 2350
$Comp
L power:GND #PWR0104
U 1 1 611E8D3C
P 3600 2350
F 0 "#PWR0104" H 3600 2100 50  0001 C CNN
F 1 "GND" H 3600 2200 50  0000 C CNN
F 2 "" H 3600 2350 50  0000 C CNN
F 3 "" H 3600 2350 50  0000 C CNN
	1    3600 2350
	-1   0    0    1   
$EndComp
Connection ~ 3600 2350
Wire Wire Line
	3600 2350 3800 2350
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 U1
U 1 1 611E964D
P 2350 1050
F 0 "U1" H 2350 1292 50  0000 C CNN
F 1 "MCP1700-3302E_SOT23" H 2350 1201 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2350 1275 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 2350 1050 50  0001 C CNN
	1    2350 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 611E9B95
P 2350 1350
F 0 "#PWR0105" H 2350 1100 50  0001 C CNN
F 1 "GND" H 2350 1200 50  0000 C CNN
F 2 "" H 2350 1350 50  0000 C CNN
F 3 "" H 2350 1350 50  0000 C CNN
	1    2350 1350
	1    0    0    -1  
$EndComp
Text Label 2800 1050 0    60   ~ 0
3V3
$Comp
L power:VCC #PWR0106
U 1 1 611EA431
P 1850 1050
F 0 "#PWR0106" H 1850 900 50  0001 C CNN
F 1 "VCC" H 1850 1200 50  0000 C CNN
F 2 "" H 1850 1050 50  0000 C CNN
F 3 "" H 1850 1050 50  0000 C CNN
	1    1850 1050
	0    -1   -1   0   
$EndComp
$Comp
L STM-LORA-rescue:C C4
U 1 1 611EB9C7
P 2700 1200
F 0 "C4" H 2725 1300 50  0000 L CNN
F 1 "1uF" H 2725 1100 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2738 1050 50  0001 C CNN
F 3 "" H 2700 1200 50  0000 C CNN
	1    2700 1200
	1    0    0    -1  
$EndComp
$Comp
L STM-LORA-rescue:C C1
U 1 1 611EC9E6
P 2000 1200
F 0 "C1" H 2025 1300 50  0000 L CNN
F 1 "1uF" H 2025 1100 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2038 1050 50  0001 C CNN
F 3 "" H 2000 1200 50  0000 C CNN
	1    2000 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1050 2700 1050
Wire Wire Line
	2700 1050 2800 1050
Connection ~ 2700 1050
Wire Wire Line
	1850 1050 2000 1050
Wire Wire Line
	2000 1050 2050 1050
Connection ~ 2000 1050
Wire Wire Line
	2000 1350 2350 1350
Connection ~ 2350 1350
Wire Wire Line
	2350 1350 2700 1350
$Comp
L RFM95:RFM95HW U3
U 1 1 58DE43F7
P 5250 3300
F 0 "U3" H 4900 3550 40  0000 C CNN
F 1 "RFM95HW" H 5750 2500 40  0000 C CNN
F 2 "RFM:RFM95" H 5250 3300 30  0001 C CIN
F 3 "" H 5250 3300 60  0000 C CNN
	1    5250 3300
	1    0    0    -1  
$EndComp
Text Label 5800 3300 0    50   ~ 0
D2
Text Label 5800 3400 0    50   ~ 0
D3
Text Label 5800 3500 0    50   ~ 0
D4
Text Label 4700 3850 2    50   ~ 0
D5
Text Label 4700 3350 2    50   ~ 0
D6
Text Label 4700 3650 2    50   ~ 0
A6
Text Label 4700 3450 2    50   ~ 0
A4
Text Label 4700 3550 2    50   ~ 0
A5
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 61210DFF
P 2350 1800
F 0 "JP1" H 2350 1950 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2350 1650 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 2350 1800 50  0001 C CNN
F 3 "~" H 2350 1800 50  0001 C CNN
	1    2350 1800
	1    0    0    -1  
$EndComp
Text Label 2500 1800 0    60   ~ 0
3V3
$Comp
L power:VCC #PWR0107
U 1 1 61211C58
P 2200 1800
F 0 "#PWR0107" H 2200 1650 50  0001 C CNN
F 1 "VCC" H 2200 1950 50  0000 C CNN
F 2 "" H 2200 1800 50  0000 C CNN
F 3 "" H 2200 1800 50  0000 C CNN
	1    2200 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 61212471
P 6900 1900
F 0 "D2" V 6939 1782 50  0000 R CNN
F 1 "LED" V 6848 1782 50  0000 R CNN
F 2 "LED_SMD:LED_1206_3216Metric" H 6900 1900 50  0001 C CNN
F 3 "~" H 6900 1900 50  0001 C CNN
	1    6900 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 612142C8
P 6900 2050
F 0 "#PWR0108" H 6900 1800 50  0001 C CNN
F 1 "GND" H 6900 1900 50  0000 C CNN
F 2 "" H 6900 2050 50  0000 C CNN
F 3 "" H 6900 2050 50  0000 C CNN
	1    6900 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 61214A67
P 6900 1600
F 0 "R4" H 6970 1646 50  0000 L CNN
F 1 "100R" H 6970 1555 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6830 1600 50  0001 C CNN
F 3 "~" H 6900 1600 50  0001 C CNN
	1    6900 1600
	1    0    0    -1  
$EndComp
Text Label 6900 1450 2    50   ~ 0
A7
Text Label 5250 2950 0    60   ~ 0
3V3
Text Label 2300 3300 2    50   ~ 0
UPDI
Text Label 2300 3500 2    50   ~ 0
D1
Text Label 2300 3600 2    50   ~ 0
D2
Text Label 2300 3700 2    50   ~ 0
D3
Text Label 2300 3800 2    50   ~ 0
D4
Text Label 2300 3900 2    50   ~ 0
D5
Text Label 2300 4000 2    50   ~ 0
D6
Text Label 2300 4100 2    50   ~ 0
D7
Text Label 2300 4300 2    50   ~ 0
F0
Text Label 2300 4400 2    50   ~ 0
F1
Text Label 2300 4500 2    50   ~ 0
F6
Text Label 3700 3300 0    50   ~ 0
A0
Text Label 3700 3400 0    50   ~ 0
A1
Text Label 3700 3500 0    50   ~ 0
A2
Text Label 3700 3600 0    50   ~ 0
A3
Text Label 3700 3700 0    50   ~ 0
A4
Text Label 3700 3800 0    50   ~ 0
A5
Text Label 3700 3900 0    50   ~ 0
A6
Text Label 3700 4000 0    50   ~ 0
A7
Text Label 3700 4300 0    50   ~ 0
C0
Text Label 3700 4400 0    50   ~ 0
C1
Text Label 3700 4500 0    50   ~ 0
C2
Text Label 3700 4600 0    50   ~ 0
C3
$Comp
L STM-LORA-rescue:CONN_01X14 J5
U 1 1 6121F14D
P 8450 5500
F 0 "J5" H 8450 6250 50  0000 C CNN
F 1 "CONN_01X14" V 8550 5500 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x14_P2.54mm_Vertical" H 8450 5500 50  0001 C CNN
F 3 "" H 8450 5500 50  0001 C CNN
	1    8450 5500
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 612210DB
P 8650 4850
F 0 "#PWR0109" H 8650 4600 50  0001 C CNN
F 1 "GND" H 8650 4700 50  0000 C CNN
F 2 "" H 8650 4850 50  0000 C CNN
F 3 "" H 8650 4850 50  0000 C CNN
	1    8650 4850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 612216D6
P 8650 6150
F 0 "#PWR0110" H 8650 5900 50  0001 C CNN
F 1 "GND" H 8650 6000 50  0000 C CNN
F 2 "" H 8650 6150 50  0000 C CNN
F 3 "" H 8650 6150 50  0000 C CNN
	1    8650 6150
	0    -1   -1   0   
$EndComp
Text Label 7250 4850 2    60   ~ 0
3V3
Text Label 7250 6150 2    60   ~ 0
3V3
Text Label 8650 4950 0    50   ~ 0
A6
Text Label 8650 5050 0    50   ~ 0
A5
Text Label 8650 5150 0    50   ~ 0
A4
Text Label 8650 5250 0    50   ~ 0
A3
Text Label 8650 5350 0    50   ~ 0
A2
Text Label 8650 5450 0    50   ~ 0
CA1
Text Label 8650 5550 0    50   ~ 0
CA0
Text Label 8650 5650 0    60   ~ 0
3V3
Text Label 8650 5750 0    50   ~ 0
UPDI
Text Label 8650 5850 0    50   ~ 0
F6
Text Label 8650 5950 0    50   ~ 0
CF1
Text Label 8650 6050 0    50   ~ 0
CF0
Text Label 7250 4950 2    50   ~ 0
A7
Text Label 7250 5050 2    50   ~ 0
C0
Text Label 7250 5150 2    50   ~ 0
C1
Text Label 7250 5250 2    50   ~ 0
C2
Text Label 7250 5350 2    50   ~ 0
C3
Text Label 7250 5450 2    50   ~ 0
D1
Text Label 7250 5550 2    50   ~ 0
D2
Text Label 7250 5650 2    50   ~ 0
D3
Text Label 7250 5750 2    50   ~ 0
D4
Text Label 7250 5850 2    50   ~ 0
D5
Text Label 7250 5950 2    50   ~ 0
D6
Text Label 7250 6050 2    50   ~ 0
D7
$Comp
L Device:R R3
U 1 1 612233E5
P 6750 3000
F 0 "R3" H 6820 3046 50  0000 L CNN
F 1 "4K7" H 6820 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6680 3000 50  0001 C CNN
F 3 "~" H 6750 3000 50  0001 C CNN
	1    6750 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 6122370A
P 7100 3000
F 0 "R5" H 7170 3046 50  0000 L CNN
F 1 "4K7" H 7170 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 7030 3000 50  0001 C CNN
F 3 "~" H 7100 3000 50  0001 C CNN
	1    7100 3000
	1    0    0    -1  
$EndComp
Text Label 6750 2850 1    60   ~ 0
3V3
Text Label 7100 2850 1    60   ~ 0
3V3
Text Label 6750 3150 3    50   ~ 0
A3
Text Label 7100 3150 3    50   ~ 0
A2
Text Notes 6700 3400 0    50   ~ 0
I2C pull ups
$Comp
L Device:Crystal Y1
U 1 1 61227560
P 4300 1400
F 0 "Y1" H 4300 1668 50  0000 C CNN
F 1 "8MHz" H 4300 1577 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_MicroCrystal_CC7V-T1A-2Pin_3.2x1.5mm" H 4300 1400 50  0001 C CNN
F 3 "~" H 4300 1400 50  0001 C CNN
	1    4300 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 61227F49
P 4500 1650
F 0 "C10" H 4615 1696 50  0000 L CNN
F 1 "22pF" H 4615 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4538 1500 50  0001 C CNN
F 3 "~" H 4500 1650 50  0001 C CNN
	1    4500 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 61228826
P 4100 1650
F 0 "C9" H 4215 1696 50  0000 L CNN
F 1 "22pF" H 4215 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4138 1500 50  0001 C CNN
F 3 "~" H 4100 1650 50  0001 C CNN
	1    4100 1650
	1    0    0    -1  
$EndComp
Text Label 4100 950  2    50   ~ 0
A0
$Comp
L power:GND #PWR0111
U 1 1 6122D62F
P 4100 1800
F 0 "#PWR0111" H 4100 1550 50  0001 C CNN
F 1 "GND" H 4100 1650 50  0000 C CNN
F 2 "" H 4100 1800 50  0000 C CNN
F 3 "" H 4100 1800 50  0000 C CNN
	1    4100 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 6122DCD0
P 4500 1800
F 0 "#PWR0112" H 4500 1550 50  0001 C CNN
F 1 "GND" H 4500 1650 50  0000 C CNN
F 2 "" H 4500 1800 50  0000 C CNN
F 3 "" H 4500 1800 50  0000 C CNN
	1    4500 1800
	1    0    0    -1  
$EndComp
Text Notes 4600 2150 0    50   ~ 0
Optional external crystals
Wire Wire Line
	4500 1400 4500 1500
$Comp
L Jumper:SolderJumper_2_Open JP2
U 1 1 6123888A
P 3950 1150
F 0 "JP2" H 3950 1017 50  0000 C CNN
F 1 "SolderJumper_2_Open" V 3995 1218 50  0001 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 3950 1150 50  0001 C CNN
F 3 "~" H 3950 1150 50  0001 C CNN
	1    3950 1150
	-1   0    0    1   
$EndComp
Connection ~ 4100 1150
Wire Wire Line
	4100 950  4100 1150
Text Label 3800 1150 2    50   ~ 0
CA0
Text Label 4500 950  2    50   ~ 0
A1
$Comp
L Jumper:SolderJumper_2_Open JP3
U 1 1 612451AD
P 4650 1150
F 0 "JP3" H 4650 1263 50  0000 C CNN
F 1 "SolderJumper_2_Open" V 4695 1218 50  0001 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 4650 1150 50  0001 C CNN
F 3 "~" H 4650 1150 50  0001 C CNN
	1    4650 1150
	1    0    0    -1  
$EndComp
Connection ~ 4500 1400
Wire Wire Line
	4500 950  4500 1150
Connection ~ 4500 1150
Wire Wire Line
	4500 1150 4500 1400
Wire Wire Line
	4100 1150 4100 1400
Wire Wire Line
	4450 1400 4500 1400
Wire Wire Line
	4150 1400 4100 1400
Connection ~ 4100 1400
Wire Wire Line
	4100 1400 4100 1500
$Comp
L Device:Crystal Y2
U 1 1 6124AB95
P 5800 1400
F 0 "Y2" H 5800 1668 50  0000 C CNN
F 1 "32768Hz" H 5800 1577 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_MicroCrystal_CC7V-T1A-2Pin_3.2x1.5mm" H 5800 1400 50  0001 C CNN
F 3 "~" H 5800 1400 50  0001 C CNN
	1    5800 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 6124AFC7
P 6000 1650
F 0 "C12" H 6115 1696 50  0000 L CNN
F 1 "10pF" H 6115 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6038 1500 50  0001 C CNN
F 3 "~" H 6000 1650 50  0001 C CNN
	1    6000 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 6124AFD1
P 5600 1650
F 0 "C11" H 5715 1696 50  0000 L CNN
F 1 "10pF" H 5715 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5638 1500 50  0001 C CNN
F 3 "~" H 5600 1650 50  0001 C CNN
	1    5600 1650
	1    0    0    -1  
$EndComp
Text Label 5600 950  2    50   ~ 0
F0
$Comp
L power:GND #PWR0113
U 1 1 6124AFDC
P 5600 1800
F 0 "#PWR0113" H 5600 1550 50  0001 C CNN
F 1 "GND" H 5600 1650 50  0000 C CNN
F 2 "" H 5600 1800 50  0000 C CNN
F 3 "" H 5600 1800 50  0000 C CNN
	1    5600 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 6124AFE6
P 6000 1800
F 0 "#PWR0114" H 6000 1550 50  0001 C CNN
F 1 "GND" H 6000 1650 50  0000 C CNN
F 2 "" H 6000 1800 50  0000 C CNN
F 3 "" H 6000 1800 50  0000 C CNN
	1    6000 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 1400 6000 1500
$Comp
L Jumper:SolderJumper_2_Open JP4
U 1 1 6124AFF1
P 5450 1150
F 0 "JP4" H 5450 1017 50  0000 C CNN
F 1 "SolderJumper_2_Open" V 5495 1218 50  0001 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 5450 1150 50  0001 C CNN
F 3 "~" H 5450 1150 50  0001 C CNN
	1    5450 1150
	-1   0    0    1   
$EndComp
Connection ~ 5600 1150
Wire Wire Line
	5600 950  5600 1150
Text Label 5300 1150 2    50   ~ 0
CF0
Text Label 6000 950  2    50   ~ 0
F1
$Comp
L Jumper:SolderJumper_2_Open JP5
U 1 1 6124AFFF
P 6150 1150
F 0 "JP5" H 6150 1263 50  0000 C CNN
F 1 "SolderJumper_2_Open" V 6195 1218 50  0001 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 6150 1150 50  0001 C CNN
F 3 "~" H 6150 1150 50  0001 C CNN
	1    6150 1150
	1    0    0    -1  
$EndComp
Connection ~ 6000 1400
Wire Wire Line
	6000 950  6000 1150
Connection ~ 6000 1150
Wire Wire Line
	6000 1150 6000 1400
Wire Wire Line
	5600 1150 5600 1400
Wire Wire Line
	5950 1400 6000 1400
Wire Wire Line
	5650 1400 5600 1400
Connection ~ 5600 1400
Wire Wire Line
	5600 1400 5600 1500
Text Label 4800 1150 0    50   ~ 0
CA1
Text Label 6300 1150 0    50   ~ 0
CF1
$EndSCHEMATC
