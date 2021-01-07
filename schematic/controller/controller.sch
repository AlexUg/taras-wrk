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
L power:GND #PWR022
U 1 1 5FF4B7C5
P 11000 750
F 0 "#PWR022" H 11000 500 50  0001 C CNN
F 1 "GND" H 11005 577 50  0000 C CNN
F 2 "" H 11000 750 50  0001 C CNN
F 3 "" H 11000 750 50  0001 C CNN
	1    11000 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR021
U 1 1 5FF4BEF5
P 10650 700
F 0 "#PWR021" H 10650 550 50  0001 C CNN
F 1 "+24V" H 10665 873 50  0000 C CNN
F 2 "" H 10650 700 50  0001 C CNN
F 3 "" H 10650 700 50  0001 C CNN
	1    10650 700 
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5FF4C6D1
P 10650 750
F 0 "#FLG02" H 10650 825 50  0001 C CNN
F 1 "PWR_FLAG" H 10650 923 50  0000 C CNN
F 2 "" H 10650 750 50  0001 C CNN
F 3 "~" H 10650 750 50  0001 C CNN
	1    10650 750 
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5FF4CC7D
P 11000 700
F 0 "#FLG03" H 11000 775 50  0001 C CNN
F 1 "PWR_FLAG" H 11000 873 50  0000 C CNN
F 2 "" H 11000 700 50  0001 C CNN
F 3 "~" H 11000 700 50  0001 C CNN
	1    11000 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 700  10650 750 
Wire Wire Line
	11000 700  11000 750 
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5FF4E0DB
P 1300 2600
F 0 "J1" H 1218 2917 50  0000 C CNN
F 1 "Sensor_Conn" H 1218 2826 50  0000 C CNN
F 2 "" H 1300 2600 50  0001 C CNN
F 3 "~" H 1300 2600 50  0001 C CNN
	1    1300 2600
	-1   0    0    -1  
$EndComp
$Comp
L power:+24V #PWR01
U 1 1 5FF4F79F
P 1750 2300
F 0 "#PWR01" H 1750 2150 50  0001 C CNN
F 1 "+24V" H 1765 2473 50  0000 C CNN
F 2 "" H 1750 2300 50  0001 C CNN
F 3 "" H 1750 2300 50  0001 C CNN
	1    1750 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5FF4FADA
P 1750 2850
F 0 "#PWR02" H 1750 2600 50  0001 C CNN
F 1 "GND" H 1755 2677 50  0000 C CNN
F 2 "" H 1750 2850 50  0001 C CNN
F 3 "" H 1750 2850 50  0001 C CNN
	1    1750 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2500 1750 2500
Wire Wire Line
	1750 2500 1750 2300
Wire Wire Line
	1500 2700 1750 2700
Wire Wire Line
	1750 2700 1750 2850
Wire Wire Line
	1500 2600 2650 2600
$Comp
L Isolator:PC817 U1
U 1 1 5FF50787
P 2950 2500
F 0 "U1" H 2950 2825 50  0000 C CNN
F 1 "PC817" H 2950 2734 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 2750 2300 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 2950 2500 50  0001 L CNN
	1    2950 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5FF52DF7
P 2550 2100
F 0 "R1" H 2620 2146 50  0000 L CNN
F 1 "2.2k" V 2550 2000 50  0000 L CNN
F 2 "" V 2480 2100 50  0001 C CNN
F 3 "~" H 2550 2100 50  0001 C CNN
	1    2550 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2250 2550 2400
Wire Wire Line
	2550 2400 2650 2400
$Comp
L power:+24V #PWR04
U 1 1 5FF5356E
P 2550 1750
F 0 "#PWR04" H 2550 1600 50  0001 C CNN
F 1 "+24V" H 2565 1923 50  0000 C CNN
F 2 "" H 2550 1750 50  0001 C CNN
F 3 "" H 2550 1750 50  0001 C CNN
	1    2550 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1750 2550 1950
$Comp
L Device:R R2
U 1 1 5FF567AB
P 3450 2100
F 0 "R2" H 3520 2146 50  0000 L CNN
F 1 "10k" V 3450 2050 50  0000 L CNN
F 2 "" V 3380 2100 50  0001 C CNN
F 3 "~" H 3450 2100 50  0001 C CNN
	1    3450 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2400 3450 2400
Wire Wire Line
	3450 2400 3450 2250
$Comp
L power:+5V #PWR05
U 1 1 5FF59AEB
P 3450 1750
F 0 "#PWR05" H 3450 1600 50  0001 C CNN
F 1 "+5V" H 3465 1923 50  0000 C CNN
F 2 "" H 3450 1750 50  0001 C CNN
F 3 "" H 3450 1750 50  0001 C CNN
	1    3450 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1750 3450 1950
Wire Wire Line
	3450 2400 3800 2400
Connection ~ 3450 2400
Text GLabel 3800 2400 2    50   Input ~ 0
Sensor
$Comp
L power:GND #PWR06
U 1 1 5FF5B9C9
P 3450 2850
F 0 "#PWR06" H 3450 2600 50  0001 C CNN
F 1 "GND" H 3455 2677 50  0000 C CNN
F 2 "" H 3450 2850 50  0001 C CNN
F 3 "" H 3450 2850 50  0001 C CNN
	1    3450 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2600 3450 2600
Wire Wire Line
	3450 2600 3450 2850
Text GLabel 5350 2050 0    50   Input ~ 0
Sensor
Wire Wire Line
	5350 2050 5750 2050
$Comp
L power:GND #PWR08
U 1 1 5FF5DC21
P 6250 3750
F 0 "#PWR08" H 6250 3500 50  0001 C CNN
F 1 "GND" H 6255 3577 50  0000 C CNN
F 2 "" H 6250 3750 50  0001 C CNN
F 3 "" H 6250 3750 50  0001 C CNN
	1    6250 3750
	1    0    0    -1  
$EndComp
NoConn ~ 5750 2950
NoConn ~ 5750 3050
NoConn ~ 6150 1450
NoConn ~ 5750 3150
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 5FF6B4C3
P 9850 5450
F 0 "J4" H 9930 5492 50  0000 L CNN
F 1 "Relay_Conn" H 9930 5401 50  0000 L CNN
F 2 "" H 9850 5450 50  0001 C CNN
F 3 "~" H 9850 5450 50  0001 C CNN
	1    9850 5450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR014
U 1 1 5FF6BD82
P 9400 5150
F 0 "#PWR014" H 9400 5000 50  0001 C CNN
F 1 "+5V" H 9415 5323 50  0000 C CNN
F 2 "" H 9400 5150 50  0001 C CNN
F 3 "" H 9400 5150 50  0001 C CNN
	1    9400 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5150 9400 5350
Wire Wire Line
	9400 5350 9650 5350
$Comp
L power:GND #PWR015
U 1 1 5FF6D749
P 9400 5800
F 0 "#PWR015" H 9400 5550 50  0001 C CNN
F 1 "GND" H 9405 5627 50  0000 C CNN
F 2 "" H 9400 5800 50  0001 C CNN
F 3 "" H 9400 5800 50  0001 C CNN
	1    9400 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5800 9400 5550
Wire Wire Line
	9400 5550 9650 5550
Wire Wire Line
	9650 5450 9050 5450
Text GLabel 9050 5450 0    50   Input ~ 0
Relay
Text GLabel 5350 2150 0    50   Input ~ 0
Relay
Wire Wire Line
	5350 2150 5750 2150
$Comp
L Display_Character:WC1602A DS1
U 1 1 5FF7EAC5
P 9850 2700
F 0 "DS1" H 9650 3450 50  0000 C CNN
F 1 "WC1602A" H 10050 3450 50  0000 C CNN
F 2 "Display:WC1602A" H 9850 1800 50  0001 C CIN
F 3 "http://www.wincomlcd.com/pdf/WC1602A-SFYLYHTC06.pdf" H 10550 2700 50  0001 C CNN
	1    9850 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR016
U 1 1 5FF9F7C3
P 9850 1550
F 0 "#PWR016" H 9850 1400 50  0001 C CNN
F 1 "+5V" H 9865 1723 50  0000 C CNN
F 2 "" H 9850 1550 50  0001 C CNN
F 3 "" H 9850 1550 50  0001 C CNN
	1    9850 1550
	1    0    0    -1  
$EndComp
Text GLabel 5350 2250 0    50   Input ~ 0
LCD_D4
Text GLabel 5350 2350 0    50   Input ~ 0
LCD_D5
Text GLabel 5350 2450 0    50   Input ~ 0
LCD_D6
Text GLabel 5350 2550 0    50   Input ~ 0
LCD_D7
Text GLabel 9200 3000 0    50   Input ~ 0
LCD_D4
Text GLabel 9200 3100 0    50   Input ~ 0
LCD_D5
Text GLabel 9200 3200 0    50   Input ~ 0
LCD_D6
Text GLabel 9200 3300 0    50   Input ~ 0
LCD_D7
Wire Wire Line
	5350 2250 5750 2250
Wire Wire Line
	5350 2350 5750 2350
Wire Wire Line
	5350 2450 5750 2450
Wire Wire Line
	5350 2550 5750 2550
Wire Wire Line
	9200 3000 9450 3000
Wire Wire Line
	9200 3100 9450 3100
Wire Wire Line
	9200 3200 9450 3200
Wire Wire Line
	9200 3300 9450 3300
Text GLabel 5350 2750 0    50   Input ~ 0
LCD_E
Text GLabel 5350 2650 0    50   Input ~ 0
LCD_RS
Wire Wire Line
	5350 2650 5750 2650
Wire Wire Line
	5350 2750 5750 2750
NoConn ~ 5750 1850
NoConn ~ 5750 1950
NoConn ~ 6750 1850
NoConn ~ 6750 1950
NoConn ~ 6750 2250
NoConn ~ 6750 3050
NoConn ~ 5750 2850
Text GLabel 7050 2650 2    50   Input ~ 0
BTN_UP
Text GLabel 7050 2550 2    50   Input ~ 0
BTN_DWN
Text GLabel 7050 2750 2    50   Input ~ 0
BTN_LEFT
Text GLabel 7050 2450 2    50   Input ~ 0
BTN_RIGHT
Wire Wire Line
	6750 2450 7050 2450
Wire Wire Line
	6750 2550 7050 2550
Wire Wire Line
	6750 2650 7050 2650
Wire Wire Line
	6750 2750 7050 2750
NoConn ~ 6750 2850
NoConn ~ 6750 2950
NoConn ~ 6750 3150
NoConn ~ 6350 1450
NoConn ~ 9450 2600
NoConn ~ 9450 2700
NoConn ~ 9450 2800
NoConn ~ 9450 2900
Text GLabel 9150 2100 0    50   Input ~ 0
LCD_E
Text GLabel 9150 2300 0    50   Input ~ 0
LCD_RS
Wire Wire Line
	9150 2100 9450 2100
Wire Wire Line
	9150 2300 9450 2300
$Comp
L power:GND #PWR017
U 1 1 5FFB756C
P 9850 3800
F 0 "#PWR017" H 9850 3550 50  0001 C CNN
F 1 "GND" H 9855 3627 50  0000 C CNN
F 2 "" H 9850 3800 50  0001 C CNN
F 3 "" H 9850 3800 50  0001 C CNN
	1    9850 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1900 9850 1550
Wire Wire Line
	9850 3500 9850 3800
$Comp
L power:GND #PWR020
U 1 1 5FFB8FDD
P 10450 2750
F 0 "#PWR020" H 10450 2500 50  0001 C CNN
F 1 "GND" H 10455 2577 50  0000 C CNN
F 2 "" H 10450 2750 50  0001 C CNN
F 3 "" H 10450 2750 50  0001 C CNN
	1    10450 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2500 10450 2500
Wire Wire Line
	10450 2500 10450 2750
Wire Wire Line
	10250 2100 10450 2100
Wire Wire Line
	10450 2100 10450 2500
Connection ~ 10450 2500
$Comp
L power:GND #PWR013
U 1 1 5FFBAE8C
P 9250 2450
F 0 "#PWR013" H 9250 2200 50  0001 C CNN
F 1 "GND" H 9255 2277 50  0000 C CNN
F 2 "" H 9250 2450 50  0001 C CNN
F 3 "" H 9250 2450 50  0001 C CNN
	1    9250 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 2450 9250 2200
Wire Wire Line
	9250 2200 9450 2200
$Comp
L power:+5V #PWR019
U 1 1 5FFBC32E
P 10350 1850
F 0 "#PWR019" H 10350 1700 50  0001 C CNN
F 1 "+5V" H 10365 2023 50  0000 C CNN
F 2 "" H 10350 1850 50  0001 C CNN
F 3 "" H 10350 1850 50  0001 C CNN
	1    10350 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2400 10350 2400
Wire Wire Line
	10350 2400 10350 1850
$Comp
L Switch:SW_Push SW1
U 1 1 5FFBE559
P 2700 3750
F 0 "SW1" H 2700 4035 50  0000 C CNN
F 1 "SW_UP" H 2700 3944 50  0000 C CNN
F 2 "" H 2700 3950 50  0001 C CNN
F 3 "~" H 2700 3950 50  0001 C CNN
	1    2700 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5FFBFA4E
P 1750 6900
F 0 "#PWR03" H 1750 6650 50  0001 C CNN
F 1 "GND" H 1755 6727 50  0000 C CNN
F 2 "" H 1750 6900 50  0001 C CNN
F 3 "" H 1750 6900 50  0001 C CNN
	1    1750 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5FFC1E1E
P 2700 4100
F 0 "C1" V 2448 4100 50  0000 C CNN
F 1 "100n" V 2539 4100 50  0000 C CNN
F 2 "" H 2738 3950 50  0001 C CNN
F 3 "~" H 2700 4100 50  0001 C CNN
	1    2700 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 4100 2400 4100
Wire Wire Line
	2400 4100 2400 3750
Wire Wire Line
	2400 3750 2500 3750
Wire Wire Line
	2900 3750 3000 3750
Wire Wire Line
	3000 3750 3000 4100
Wire Wire Line
	3000 4100 2850 4100
Connection ~ 2400 3750
Wire Wire Line
	3000 3750 3200 3750
Connection ~ 3000 3750
Text GLabel 3200 3750 2    50   Input ~ 0
BTN_UP
$Comp
L Switch:SW_Push SW2
U 1 1 5FFCA1D2
P 2700 4550
F 0 "SW2" H 2700 4835 50  0000 C CNN
F 1 "SW_DOWN" H 2700 4744 50  0000 C CNN
F 2 "" H 2700 4750 50  0001 C CNN
F 3 "~" H 2700 4750 50  0001 C CNN
	1    2700 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5FFCA1D8
P 2700 4900
F 0 "C2" V 2448 4900 50  0000 C CNN
F 1 "100n" V 2539 4900 50  0000 C CNN
F 2 "" H 2738 4750 50  0001 C CNN
F 3 "~" H 2700 4900 50  0001 C CNN
	1    2700 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 4900 2400 4900
Wire Wire Line
	2400 4900 2400 4550
Wire Wire Line
	2400 4550 2500 4550
Wire Wire Line
	2900 4550 3000 4550
Wire Wire Line
	3000 4550 3000 4900
Wire Wire Line
	3000 4900 2850 4900
Connection ~ 2400 4550
Wire Wire Line
	3000 4550 3200 4550
Connection ~ 3000 4550
Text GLabel 3200 4550 2    50   Input ~ 0
BTN_DWN
$Comp
L Switch:SW_Push SW3
U 1 1 5FFD158E
P 2700 5350
F 0 "SW3" H 2700 5635 50  0000 C CNN
F 1 "SW_LEFT" H 2700 5544 50  0000 C CNN
F 2 "" H 2700 5550 50  0001 C CNN
F 3 "~" H 2700 5550 50  0001 C CNN
	1    2700 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5FFD1594
P 2700 5700
F 0 "C3" V 2448 5700 50  0000 C CNN
F 1 "100n" V 2539 5700 50  0000 C CNN
F 2 "" H 2738 5550 50  0001 C CNN
F 3 "~" H 2700 5700 50  0001 C CNN
	1    2700 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 5700 2400 5700
Wire Wire Line
	2400 5700 2400 5350
Wire Wire Line
	2400 5350 2500 5350
Wire Wire Line
	2900 5350 3000 5350
Wire Wire Line
	3000 5350 3000 5700
Wire Wire Line
	3000 5700 2850 5700
Connection ~ 2400 5350
Wire Wire Line
	3000 5350 3200 5350
Connection ~ 3000 5350
Text GLabel 3200 5350 2    50   Input ~ 0
BTN_LEFT
$Comp
L Switch:SW_Push SW4
U 1 1 5FFD3C87
P 2700 6150
F 0 "SW4" H 2700 6435 50  0000 C CNN
F 1 "SW_RIGHT" H 2700 6344 50  0000 C CNN
F 2 "" H 2700 6350 50  0001 C CNN
F 3 "~" H 2700 6350 50  0001 C CNN
	1    2700 6150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5FFD3C8D
P 2700 6500
F 0 "C4" V 2448 6500 50  0000 C CNN
F 1 "100n" V 2539 6500 50  0000 C CNN
F 2 "" H 2738 6350 50  0001 C CNN
F 3 "~" H 2700 6500 50  0001 C CNN
	1    2700 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 6500 2400 6500
Wire Wire Line
	2400 6500 2400 6150
Wire Wire Line
	2400 6150 2500 6150
Wire Wire Line
	2900 6150 3000 6150
Wire Wire Line
	3000 6150 3000 6500
Wire Wire Line
	3000 6500 2850 6500
Connection ~ 2400 6150
Wire Wire Line
	3000 6150 3200 6150
Connection ~ 3000 6150
Text GLabel 3200 6150 2    50   Input ~ 0
BTN_RIGHT
Wire Wire Line
	1750 3750 1750 4550
Wire Wire Line
	1750 3750 2400 3750
Wire Wire Line
	1750 4550 2400 4550
Connection ~ 1750 4550
Wire Wire Line
	1750 5350 2400 5350
Wire Wire Line
	1750 4550 1750 5350
Connection ~ 1750 5350
Wire Wire Line
	1750 5350 1750 6150
Wire Wire Line
	1750 6150 2400 6150
Connection ~ 1750 6150
Wire Wire Line
	1750 6150 1750 6900
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5FFE7866
P 6050 5550
F 0 "J2" H 6130 5542 50  0000 L CNN
F 1 "+24V_Power" H 6130 5451 50  0000 L CNN
F 2 "" H 6050 5550 50  0001 C CNN
F 3 "~" H 6050 5550 50  0001 C CNN
	1    6050 5550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5FFE7DAF
P 7550 5550
F 0 "J3" H 7630 5542 50  0000 L CNN
F 1 "+5V_Power" H 7630 5451 50  0000 L CNN
F 2 "" H 7550 5550 50  0001 C CNN
F 3 "~" H 7550 5550 50  0001 C CNN
	1    7550 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR09
U 1 1 5FFE9124
P 5500 5300
F 0 "#PWR09" H 5500 5150 50  0001 C CNN
F 1 "+24V" H 5515 5473 50  0000 C CNN
F 2 "" H 5500 5300 50  0001 C CNN
F 3 "" H 5500 5300 50  0001 C CNN
	1    5500 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5FFEA838
P 5500 6050
F 0 "#PWR010" H 5500 5800 50  0001 C CNN
F 1 "GND" H 5505 5877 50  0000 C CNN
F 2 "" H 5500 6050 50  0001 C CNN
F 3 "" H 5500 6050 50  0001 C CNN
	1    5500 6050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 5FFEB2CF
P 7000 5250
F 0 "#PWR011" H 7000 5100 50  0001 C CNN
F 1 "+5V" H 7015 5423 50  0000 C CNN
F 2 "" H 7000 5250 50  0001 C CNN
F 3 "" H 7000 5250 50  0001 C CNN
	1    7000 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5FFEB7D8
P 7000 6000
F 0 "#PWR012" H 7000 5750 50  0001 C CNN
F 1 "GND" H 7005 5827 50  0000 C CNN
F 2 "" H 7000 6000 50  0001 C CNN
F 3 "" H 7000 6000 50  0001 C CNN
	1    7000 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 5550 5500 5550
Wire Wire Line
	5500 5550 5500 5300
Wire Wire Line
	5850 5650 5500 5650
Wire Wire Line
	5500 5650 5500 6050
Wire Wire Line
	7350 5550 7000 5550
Wire Wire Line
	7000 5550 7000 5250
Wire Wire Line
	7350 5650 7000 5650
Wire Wire Line
	7000 5650 7000 6000
$Comp
L MCU_Module:Arduino_Nano_v2.x A1
U 1 1 6000629A
P 6250 2450
F 0 "A1" H 5950 3400 50  0000 C CNN
F 1 "Arduino_Nano_v2.x" H 6850 3450 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6250 2450 50  0001 C CIN
F 3 "https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf" H 6250 2450 50  0001 C CNN
	1    6250 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 600486C4
P 6450 1100
F 0 "#PWR07" H 6450 950 50  0001 C CNN
F 1 "+5V" H 6465 1273 50  0000 C CNN
F 2 "" H 6450 1100 50  0001 C CNN
F 3 "" H 6450 1100 50  0001 C CNN
	1    6450 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1100 6450 1450
Wire Wire Line
	6250 3750 6250 3600
Wire Wire Line
	6250 3600 6350 3600
Wire Wire Line
	6350 3600 6350 3450
Connection ~ 6250 3600
Wire Wire Line
	6250 3600 6250 3450
$EndSCHEMATC
