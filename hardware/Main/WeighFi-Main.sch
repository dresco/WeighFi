EESchema Schematic File Version 2
LIBS:dresco
LIBS:WeighFi-Main-cache
EELAYER 27 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "WeighFi-Main"
Date "15 apr 2015"
Rev "1.0"
Comp ""
Comment1 "Released under Creative Commons Attribution-Sharealike (CC BY-SA 3.0) license"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_12 P16
U 1 1 50BC8C47
P 14750 1650
F 0 "P16" V 14710 1650 60  0000 C CNN
F 1 "CONN_12" V 14820 1650 60  0000 C CNN
F 2 "" H 14750 1650 60  0001 C CNN
F 3 "" H 14750 1650 60  0001 C CNN
	1    14750 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR28
U 1 1 50BC8C46
P 13250 2250
F 0 "#PWR28" H 13250 2250 30  0001 C CNN
F 1 "GND" H 13250 2180 30  0001 C CNN
F 2 "" H 13250 2250 60  0001 C CNN
F 3 "" H 13250 2250 60  0001 C CNN
	1    13250 2250
	-1   0    0    -1  
$EndComp
Text GLabel 14200 1050 0    60   Input ~ 0
LCD_SUPPLY
Text GLabel 14200 1150 0    60   Input ~ 0
I2C_DATA
Text GLabel 14200 1250 0    60   Input ~ 0
I2C_CLOCK
Text GLabel 14200 1350 0    60   Input ~ 0
LCD_CLOCK
$Comp
L +3V #PWR26
U 1 1 50BC8C45
P 12800 1900
F 0 "#PWR26" H 12800 1810 30  0001 C CNN
F 1 "+3V" H 12800 2000 30  0000 C CNN
F 2 "" H 12800 1900 60  0001 C CNN
F 3 "" H 12800 1900 60  0001 C CNN
	1    12800 1900
	-1   0    0    -1  
$EndComp
$Comp
L +BATT #PWR27
U 1 1 50BC8C44
P 13250 1900
F 0 "#PWR27" H 13250 1850 20  0001 C CNN
F 1 "+BATT" H 13250 2000 30  0000 C CNN
F 2 "" H 13250 1900 60  0001 C CNN
F 3 "" H 13250 1900 60  0001 C CNN
	1    13250 1900
	-1   0    0    -1  
$EndComp
Text GLabel 14200 1450 0    60   Input ~ 0
ADC_SPEED
Text GLabel 14200 1550 0    60   Input ~ 0
ADC_PWRDN
Text GLabel 14200 1650 0    60   Input ~ 0
ADC_CLOCK
Text GLabel 14200 1750 0    60   Input ~ 0
ADC_DATA
NoConn ~ 14400 1850
Text Notes 13250 700  0    60   ~ 0
Display board connector
Text GLabel 11100 4950 2    60   Input ~ 0
DEBUG
Text GLabel 1400 10750 0    60   Input ~ 0
DEBUG
$Comp
L GND #PWR5
U 1 1 4FE9FA1E
P 3100 11000
F 0 "#PWR5" H 3100 11000 30  0001 C CNN
F 1 "GND" H 3100 10930 30  0001 C CNN
F 2 "" H 3100 11000 60  0001 C CNN
F 3 "" H 3100 11000 60  0001 C CNN
	1    3100 11000
	1    0    0    -1  
$EndComp
$Comp
L JUMPER JP1
U 1 1 4FE9FA0C
P 2100 10750
F 0 "JP1" H 2100 10900 60  0000 C CNN
F 1 "JUMPER" H 2100 10670 40  0000 C CNN
F 2 "" H 2100 10750 60  0001 C CNN
F 3 "" H 2100 10750 60  0001 C CNN
	1    2100 10750
	1    0    0    -1  
$EndComp
$Comp
L SWITCH_PUSH_NO S3
U 1 1 4FE9F9E8
P 10900 10200
F 0 "S3" H 10900 10375 60  0000 C CNN
F 1 "SWITCH_PUSH_NO" H 10925 10075 60  0000 C CNN
F 2 "" H 10900 10200 60  0001 C CNN
F 3 "" H 10900 10200 60  0001 C CNN
	1    10900 10200
	1    0    0    -1  
$EndComp
Text GLabel 11050 1100 0    60   Input ~ 0
STATUS
Text GLabel 11100 4350 2    60   Input ~ 0
STATUS
$Comp
L R R9
U 1 1 4FDB64DE
P 11150 1500
F 0 "R9" V 11230 1500 50  0000 C CNN
F 1 "1K8" V 11150 1500 50  0000 C CNN
F 2 "" H 11150 1500 60  0001 C CNN
F 3 "" H 11150 1500 60  0001 C CNN
	1    11150 1500
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 4FDB64DD
P 11150 2100
F 0 "D2" H 11150 2200 50  0000 C CNN
F 1 "LED" H 11150 2000 50  0000 C CNN
F 2 "" H 11150 2100 60  0001 C CNN
F 3 "" H 11150 2100 60  0001 C CNN
	1    11150 2100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR22
U 1 1 4FDB64DC
P 11150 2450
F 0 "#PWR22" H 11150 2450 30  0001 C CNN
F 1 "GND" H 11150 2380 30  0001 C CNN
F 2 "" H 11150 2450 60  0001 C CNN
F 3 "" H 11150 2450 60  0001 C CNN
	1    11150 2450
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P15
U 1 1 4FDB64DB
P 11150 950
F 0 "P15" H 11230 950 40  0000 L CNN
F 1 "CONN_1" H 11150 1005 30  0001 C CNN
F 2 "" H 11150 950 60  0001 C CNN
F 3 "" H 11150 950 60  0001 C CNN
	1    11150 950 
	0    -1   -1   0   
$EndComp
$Comp
L SWITCH_SP3T S2
U 1 1 4FDB62FA
P 2100 10100
F 0 "S2" H 2100 10275 60  0000 C CNN
F 1 "SWITCH_SP3T" H 2125 9875 60  0000 C CNN
F 2 "" H 2100 10100 60  0001 C CNN
F 3 "" H 2100 10100 60  0001 C CNN
	1    2100 10100
	1    0    0    -1  
$EndComp
$Comp
L SWITCH_SPDT S1
U 1 1 4FDB62EB
P 2100 9350
F 0 "S1" H 2100 9525 60  0000 C CNN
F 1 "SWITCH_SPDT" H 2125 9125 60  0000 C CNN
F 2 "" H 2100 9350 60  0001 C CNN
F 3 "" H 2100 9350 60  0001 C CNN
	1    2100 9350
	1    0    0    -1  
$EndComp
$Comp
L HOLE H4
U 1 1 4FCE3FAD
P 14550 9400
F 0 "H4" H 14510 9225 40  0000 L CNN
F 1 "HOLE" H 14550 9570 30  0001 C CNN
F 2 "" H 14550 9400 60  0001 C CNN
F 3 "" H 14550 9400 60  0001 C CNN
	1    14550 9400
	1    0    0    -1  
$EndComp
$Comp
L HOLE H2
U 1 1 4FCE3F9E
P 13850 9400
F 0 "H2" H 13810 9225 40  0000 L CNN
F 1 "HOLE" H 13850 9570 30  0001 C CNN
F 2 "" H 13850 9400 60  0001 C CNN
F 3 "" H 13850 9400 60  0001 C CNN
	1    13850 9400
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P3
U 1 1 4FCCF97C
P 2150 4100
F 0 "P3" H 2230 4100 40  0000 L CNN
F 1 "CONN_1" H 2150 4155 30  0001 C CNN
F 2 "" H 2150 4100 60  0001 C CNN
F 3 "" H 2150 4100 60  0001 C CNN
	1    2150 4100
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P2
U 1 1 4FCCF965
P 2050 4100
F 0 "P2" H 2130 4100 40  0000 L CNN
F 1 "CONN_1" H 2050 4155 30  0001 C CNN
F 2 "" H 2050 4100 60  0001 C CNN
F 3 "" H 2050 4100 60  0001 C CNN
	1    2050 4100
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P1
U 1 1 4FCCF940
P 1950 4100
F 0 "P1" H 2030 4100 40  0000 L CNN
F 1 "CONN_1" H 1950 4155 30  0001 C CNN
F 2 "" H 1950 4100 60  0001 C CNN
F 3 "" H 1950 4100 60  0001 C CNN
	1    1950 4100
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P14
U 1 1 4FCCF8DB
P 10200 950
F 0 "P14" H 10280 950 40  0000 L CNN
F 1 "CONN_1" H 10200 1005 30  0001 C CNN
F 2 "" H 10200 950 60  0001 C CNN
F 3 "" H 10200 950 60  0001 C CNN
	1    10200 950 
	0    -1   -1   0   
$EndComp
$Comp
L ATMEGA16U4/32U4 U3
U 1 1 4FCCF7C0
P 9700 5650
F 0 "U3" H 9650 5750 50  0000 L BNN
F 1 "ATMEGA16U4/32U4" H 9300 5920 50  0000 L BNN
F 2 "" H 9700 5650 60  0001 C CNN
F 3 "" H 9700 5650 60  0001 C CNN
	1    9700 5650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR23
U 1 1 4FCCF6AD
P 11150 5100
F 0 "#PWR23" H 11150 5100 30  0001 C CNN
F 1 "GND" H 11150 5030 30  0001 C CNN
F 2 "" H 11150 5100 60  0001 C CNN
F 3 "" H 11150 5100 60  0001 C CNN
	1    11150 5100
	1    0    0    -1  
$EndComp
Text GLabel 10100 1100 0    60   Input ~ 0
CHG
$Comp
L GND #PWR21
U 1 1 4FCCE26F
P 10200 2450
F 0 "#PWR21" H 10200 2450 30  0001 C CNN
F 1 "GND" H 10200 2380 30  0001 C CNN
F 2 "" H 10200 2450 60  0001 C CNN
F 3 "" H 10200 2450 60  0001 C CNN
	1    10200 2450
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 4FCCE26C
P 10200 2100
F 0 "D1" H 10200 2200 50  0000 C CNN
F 1 "LED" H 10200 2000 50  0000 C CNN
F 2 "" H 10200 2100 60  0001 C CNN
F 3 "" H 10200 2100 60  0001 C CNN
	1    10200 2100
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 4FCCE264
P 10200 1500
F 0 "R8" V 10280 1500 50  0000 C CNN
F 1 "1K8" V 10200 1500 50  0000 C CNN
F 2 "" H 10200 1500 60  0001 C CNN
F 3 "" H 10200 1500 60  0001 C CNN
	1    10200 1500
	1    0    0    -1  
$EndComp
Text Notes 10200 700  0    60   ~ 0
Status LEDs
Text GLabel 11100 4450 2    60   Input ~ 0
WIFI_RESET
Text GLabel 11100 6950 2    60   Input ~ 0
LCD_CLOCK
$Comp
L CONN_1 P11
U 1 1 4FC7A464
P 7350 9300
F 0 "P11" H 7430 9300 40  0000 L CNN
F 1 "CONN_1" H 7350 9355 30  0001 C CNN
F 2 "" H 7350 9300 60  0001 C CNN
F 3 "" H 7350 9300 60  0001 C CNN
	1    7350 9300
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P13
U 1 1 4FC7A442
P 8700 9850
F 0 "P13" H 8780 9850 40  0000 L CNN
F 1 "CONN_1" H 8700 9905 30  0001 C CNN
F 2 "" H 8700 9850 60  0001 C CNN
F 3 "" H 8700 9850 60  0001 C CNN
	1    8700 9850
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P8
U 1 1 4FC7A426
P 5200 9300
F 0 "P8" H 5280 9300 40  0000 L CNN
F 1 "CONN_1" H 5200 9355 30  0001 C CNN
F 2 "" H 5200 9300 60  0001 C CNN
F 3 "" H 5200 9300 60  0001 C CNN
	1    5200 9300
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P6
U 1 1 4FC7A40D
P 3000 9800
F 0 "P6" H 3080 9800 40  0000 L CNN
F 1 "CONN_1" H 3000 9855 30  0001 C CNN
F 2 "" H 3000 9800 60  0001 C CNN
F 3 "" H 3000 9800 60  0001 C CNN
	1    3000 9800
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P5
U 1 1 4FC7A40A
P 2900 9800
F 0 "P5" H 2980 9800 40  0000 L CNN
F 1 "CONN_1" H 2900 9855 30  0001 C CNN
F 2 "" H 2900 9800 60  0001 C CNN
F 3 "" H 2900 9800 60  0001 C CNN
	1    2900 9800
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P4
U 1 1 4FC7A3FD
P 2800 9800
F 0 "P4" H 2880 9800 40  0000 L CNN
F 1 "CONN_1" H 2800 9855 30  0001 C CNN
F 2 "" H 2800 9800 60  0001 C CNN
F 3 "" H 2800 9800 60  0001 C CNN
	1    2800 9800
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG3
U 1 1 4FC509BC
P 7150 2300
F 0 "#FLG3" H 7150 2570 30  0001 C CNN
F 1 "PWR_FLAG" H 7150 2530 30  0000 C CNN
F 2 "" H 7150 2300 60  0001 C CNN
F 3 "" H 7150 2300 60  0001 C CNN
	1    7150 2300
	1    0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG4
U 1 1 4FC50659
P 8950 10000
F 0 "#FLG4" H 8950 10095 30  0001 C CNN
F 1 "PWR_FLAG" H 8950 10180 30  0000 C CNN
F 2 "" H 8950 10000 60  0001 C CNN
F 3 "" H 8950 10000 60  0001 C CNN
	1    8950 10000
	1    0    0    -1  
$EndComp
Text GLabel 11100 5650 2    60   Input ~ 0
LCD_ENABLE
$Comp
L PWR_FLAG #FLG2
U 1 1 4FC50360
P 7150 1500
F 0 "#FLG2" H 7150 1770 30  0001 C CNN
F 1 "PWR_FLAG" H 7150 1730 30  0000 C CNN
F 2 "" H 7150 1500 60  0001 C CNN
F 3 "" H 7150 1500 60  0001 C CNN
	1    7150 1500
	1    0    0    -1  
$EndComp
Text Notes 700  8900 0    60   ~ 0
Main power switch, display units switch,\ndebug mode jumper
Text Notes 4400 8900 0    60   ~ 0
Voltage divider for \nbattery measurements \nusing internal ADC
Text Notes 7250 8900 0    60   ~ 0
Power control for LCD driver\n(reduce sleep current)
Text Notes 2800 3550 0    60   ~ 0
WiFi Module
Text Notes 8900 3550 0    60   ~ 0
MCU
Text GLabel 11100 5550 2    60   Input ~ 0
WIFI_RX
Text GLabel 11100 5450 2    60   Input ~ 0
WIFI_TX
Text GLabel 1900 4950 0    60   Input ~ 0
WIFI_RESET
Text GLabel 11100 7050 2    60   Input ~ 0
SW_KG
Text GLabel 11100 6850 2    60   Input ~ 0
SW_LB
Text GLabel 11100 6450 2    60   Input ~ 0
SW_ST
Text GLabel 3100 10200 2    60   Input ~ 0
SW_ST
Text GLabel 3100 10100 2    60   Input ~ 0
SW_LB
Text GLabel 3100 10000 2    60   Input ~ 0
SW_KG
Text GLabel 6950 1600 0    60   Input ~ 0
PWR_SW_OUT
Text GLabel 5700 1600 2    60   Input ~ 0
PWR_SW_IN
$Comp
L GND #PWR1
U 1 1 4FC34993
P 1400 10350
F 0 "#PWR1" H 1400 10350 30  0001 C CNN
F 1 "GND" H 1400 10280 30  0001 C CNN
F 2 "" H 1400 10350 60  0001 C CNN
F 3 "" H 1400 10350 60  0001 C CNN
	1    1400 10350
	1    0    0    -1  
$EndComp
Text GLabel 1400 9350 0    60   Input ~ 0
PWR_SW_IN
Text GLabel 3100 9450 2    60   Input ~ 0
PWR_SW_OUT
NoConn ~ 2600 9250
Text GLabel 11100 6250 2    60   Input ~ 0
ADC_PWRDN
Text GLabel 11100 6150 2    60   Input ~ 0
ADC_SPEED
Text GLabel 11100 5950 2    60   Input ~ 0
ADC_DATA
Text GLabel 11100 5850 2    60   Input ~ 0
ADC_CLOCK
$Comp
L C C4
U 1 1 4FBF8D57
P 5200 9950
F 0 "C4" H 5250 10050 50  0000 L CNN
F 1 "10nF" H 5250 9850 50  0000 L CNN
F 2 "" H 5200 9950 60  0001 C CNN
F 3 "" H 5200 9950 60  0001 C CNN
	1    5200 9950
	1    0    0    -1  
$EndComp
Text GLabel 11100 4250 2    60   Input ~ 0
BAT_ADC
Text GLabel 4850 9600 0    60   Input ~ 0
BAT_ADC
$Comp
L GND #PWR10
U 1 1 4FBF8CF7
P 5550 10400
F 0 "#PWR10" H 5550 10400 30  0001 C CNN
F 1 "GND" H 5550 10330 30  0001 C CNN
F 2 "" H 5550 10400 60  0001 C CNN
F 3 "" H 5550 10400 60  0001 C CNN
	1    5550 10400
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 4FBF8CEF
P 5550 9950
F 0 "R2" V 5630 9950 50  0000 C CNN
F 1 "R" V 5550 9950 50  0000 C CNN
F 2 "" H 5550 9950 60  0001 C CNN
F 3 "" H 5550 9950 60  0001 C CNN
	1    5550 9950
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 4FBF8CE3
P 5550 9250
F 0 "R1" V 5630 9250 50  0000 C CNN
F 1 "R" V 5550 9250 50  0000 C CNN
F 2 "" H 5550 9250 60  0001 C CNN
F 3 "" H 5550 9250 60  0001 C CNN
	1    5550 9250
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR9
U 1 1 4FBF8CD4
P 5550 8900
F 0 "#PWR9" H 5550 8850 20  0001 C CNN
F 1 "+BATT" H 5550 9000 30  0000 C CNN
F 2 "" H 5550 8900 60  0001 C CNN
F 3 "" H 5550 8900 60  0001 C CNN
	1    5550 8900
	1    0    0    -1  
$EndComp
$Comp
L +VUSB #PWR13
U 1 1 4FBF89B4
P 6900 5600
F 0 "#PWR13" H 6900 5550 20  0001 C CNN
F 1 "+VUSB" H 6900 5700 30  0000 C CNN
F 2 "" H 6900 5600 60  0001 C CNN
F 3 "" H 6900 5600 60  0001 C CNN
	1    6900 5600
	1    0    0    -1  
$EndComp
Text GLabel 7400 5850 0    60   Input ~ 0
D-
Text GLabel 7400 5750 0    60   Input ~ 0
D+
NoConn ~ 2250 1900
Text GLabel 8500 10250 2    60   Input ~ 0
LCD_SUPPLY
$Comp
L MOSFET_P Q1
U 1 1 4FBF7AB0
P 8350 9600
F 0 "Q1" H 8350 9790 60  0000 R CNN
F 1 "MOSFET_P" H 8950 9600 60  0000 R CNN
F 2 "" H 8350 9600 60  0001 C CNN
F 3 "" H 8350 9600 60  0001 C CNN
	1    8350 9600
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 4FBEC438
P 7700 9600
F 0 "R4" V 7780 9600 50  0000 C CNN
F 1 "100" V 7700 9600 50  0000 C CNN
F 2 "" H 7700 9600 60  0001 C CNN
F 3 "" H 7700 9600 60  0001 C CNN
	1    7700 9600
	0    -1   -1   0   
$EndComp
$Comp
L R R7
U 1 1 4FBEC3ED
P 8050 9350
F 0 "R7" V 8130 9350 50  0000 C CNN
F 1 "100K" V 8050 9350 50  0000 C CNN
F 2 "" H 8050 9350 60  0001 C CNN
F 3 "" H 8050 9350 60  0001 C CNN
	1    8050 9350
	-1   0    0    1   
$EndComp
Text GLabel 7200 9600 0    60   Input ~ 0
LCD_ENABLE
$Comp
L +3V #PWR18
U 1 1 4FBEC3B3
P 8450 9050
F 0 "#PWR18" H 8450 8960 30  0001 C CNN
F 1 "+3V" H 8450 9160 30  0000 C CNN
F 2 "" H 8450 9050 60  0001 C CNN
F 3 "" H 8450 9050 60  0001 C CNN
	1    8450 9050
	1    0    0    -1  
$EndComp
Text GLabel 11100 5250 2    60   Input ~ 0
I2C_CLOCK
Text GLabel 11100 5350 2    60   Input ~ 0
I2C_DATA
Text GLabel 8800 3900 2    60   Input ~ 0
RST
Text GLabel 11100 6550 2    60   Input ~ 0
SCLK
Text GLabel 11100 6750 2    60   Input ~ 0
MISO
Text GLabel 11100 6650 2    60   Input ~ 0
MOSI
$Comp
L GND #PWR14
U 1 1 4FBE97EF
P 6900 6300
F 0 "#PWR14" H 6900 6300 30  0001 C CNN
F 1 "GND" H 6900 6230 30  0001 C CNN
F 2 "" H 6900 6300 60  0001 C CNN
F 3 "" H 6900 6300 60  0001 C CNN
	1    6900 6300
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 4FBE97BB
P 6900 5900
F 0 "C6" H 6950 6000 50  0000 L CNN
F 1 "10uF" H 6950 5800 50  0000 L CNN
F 2 "" H 6900 5900 60  0001 C CNN
F 3 "" H 6900 5900 60  0001 C CNN
	1    6900 5900
	-1   0    0    1   
$EndComp
$Comp
L USB USB1
U 1 1 4FBE9598
P 2050 1800
F 0 "USB1" H 1950 1450 60  0000 C CNN
F 1 "USB" H 2050 1450 60  0001 C CNN
F 2 "" H 2050 1800 60  0001 C CNN
F 3 "" H 2050 1800 60  0001 C CNN
	1    2050 1800
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 4FBE83F1
P 7850 5850
F 0 "R6" V 7800 5650 50  0000 C CNN
F 1 "22" V 7850 5850 50  0000 C CNN
F 2 "" H 7850 5850 60  0001 C CNN
F 3 "" H 7850 5850 60  0001 C CNN
	1    7850 5850
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 4FBE83EA
P 7850 5750
F 0 "R5" V 7900 5550 50  0000 C CNN
F 1 "22" V 7850 5750 50  0000 C CNN
F 2 "" H 7850 5750 60  0001 C CNN
F 3 "" H 7850 5750 60  0001 C CNN
	1    7850 5750
	0    -1   -1   0   
$EndComp
$Comp
L C C7
U 1 1 4FBE82BB
P 6900 6950
F 0 "C7" H 6950 7050 50  0000 L CNN
F 1 "1uF" H 6950 6850 50  0000 L CNN
F 2 "" H 6900 6950 60  0001 C CNN
F 3 "" H 6900 6950 60  0001 C CNN
	1    6900 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 4FBE8254
P 6900 7350
F 0 "#PWR16" H 6900 7350 30  0001 C CNN
F 1 "GND" H 6900 7280 30  0001 C CNN
F 2 "" H 6900 7350 60  0001 C CNN
F 3 "" H 6900 7350 60  0001 C CNN
	1    6900 7350
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR15
U 1 1 4FBE823E
P 6900 6550
F 0 "#PWR15" H 6900 6460 30  0001 C CNN
F 1 "+3V" H 6900 6660 30  0000 C CNN
F 2 "" H 6900 6550 60  0001 C CNN
F 3 "" H 6900 6550 60  0001 C CNN
	1    6900 6550
	1    0    0    -1  
$EndComp
NoConn ~ 8600 5550
$Comp
L CRYSTAL X1
U 1 1 4FBE75E9
P 7800 6650
F 0 "X1" H 7800 6800 60  0000 C CNN
F 1 "CRYSTAL" H 7800 6500 60  0000 C CNN
F 2 "" H 7800 6650 60  0001 C CNN
F 3 "" H 7800 6650 60  0001 C CNN
	1    7800 6650
	-1   0    0    1   
$EndComp
$Comp
L C C11
U 1 1 4FBE75E8
P 7500 6950
F 0 "C11" H 7550 7050 50  0000 L CNN
F 1 "22pF" H 7550 6850 50  0000 L CNN
F 2 "" H 7500 6950 60  0001 C CNN
F 3 "" H 7500 6950 60  0001 C CNN
	1    7500 6950
	-1   0    0    1   
$EndComp
$Comp
L C C13
U 1 1 4FBE75E7
P 8100 6950
F 0 "C13" H 8150 7050 50  0000 L CNN
F 1 "22pF" H 8150 6850 50  0000 L CNN
F 2 "" H 8100 6950 60  0001 C CNN
F 3 "" H 8100 6950 60  0001 C CNN
	1    8100 6950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR17
U 1 1 4FBE75E6
P 7500 7350
F 0 "#PWR17" H 7500 7350 30  0001 C CNN
F 1 "GND" H 7500 7280 30  0001 C CNN
F 2 "" H 7500 7350 60  0001 C CNN
F 3 "" H 7500 7350 60  0001 C CNN
	1    7500 7350
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 4FB6D42C
P 7000 4550
F 0 "L1" V 6950 4350 40  0000 C CNN
F 1 "INDUCTOR" V 6950 4600 40  0000 C CNN
F 2 "" H 7000 4550 60  0001 C CNN
F 3 "" H 7000 4550 60  0001 C CNN
	1    7000 4550
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 4FB6D295
P 7000 4250
F 0 "R3" V 7080 4250 50  0000 C CNN
F 1 "10K" V 7000 4250 50  0000 C CNN
F 2 "" H 7000 4250 60  0001 C CNN
F 3 "" H 7000 4250 60  0001 C CNN
	1    7000 4250
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 4FB6D239
P 7000 5200
F 0 "C8" H 7050 5300 50  0000 L CNN
F 1 "0.1uF" H 7050 5100 50  0000 L CNN
F 2 "" H 7000 5200 60  0001 C CNN
F 3 "" H 7000 5200 60  0001 C CNN
	1    7000 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR12
U 1 1 4FB6D221
P 6600 5550
F 0 "#PWR12" H 6600 5550 30  0001 C CNN
F 1 "GND" H 6600 5480 30  0001 C CNN
F 2 "" H 6600 5550 60  0001 C CNN
F 3 "" H 6600 5550 60  0001 C CNN
	1    6600 5550
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 4FB6D1D8
P 8200 5200
F 0 "C14" H 8250 5300 50  0000 L CNN
F 1 "0.1uF" H 8250 5100 50  0000 L CNN
F 2 "" H 8200 5200 60  0001 C CNN
F 3 "" H 8200 5200 60  0001 C CNN
	1    8200 5200
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 4FB6D1D6
P 6600 5200
F 0 "C5" H 6650 5300 50  0000 L CNN
F 1 "0.1uF" H 6650 5100 50  0000 L CNN
F 2 "" H 6600 5200 60  0001 C CNN
F 3 "" H 6600 5200 60  0001 C CNN
	1    6600 5200
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 4FB6D1D3
P 7400 5200
F 0 "C10" H 7450 5300 50  0000 L CNN
F 1 "0.1uF" H 7450 5100 50  0000 L CNN
F 2 "" H 7400 5200 60  0001 C CNN
F 3 "" H 7400 5200 60  0001 C CNN
	1    7400 5200
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 4FB6D1B1
P 7800 5200
F 0 "C12" H 7850 5300 50  0000 L CNN
F 1 "0.1uF" H 7850 5100 50  0000 L CNN
F 2 "" H 7800 5200 60  0001 C CNN
F 3 "" H 7800 5200 60  0001 C CNN
	1    7800 5200
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR11
U 1 1 4FB4FBCB
P 6600 3950
F 0 "#PWR11" H 6600 3860 30  0001 C CNN
F 1 "+3V" H 6600 4060 30  0000 C CNN
F 2 "" H 6600 3950 60  0001 C CNN
F 3 "" H 6600 3950 60  0001 C CNN
	1    6600 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 4FB4FBAE
P 8500 7350
F 0 "#PWR19" H 8500 7350 30  0001 C CNN
F 1 "GND" H 8500 7280 30  0001 C CNN
F 2 "" H 8500 7350 60  0001 C CNN
F 3 "" H 8500 7350 60  0001 C CNN
	1    8500 7350
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR2
U 1 1 4FB1357C
P 1600 3950
F 0 "#PWR2" H 1600 3860 30  0001 C CNN
F 1 "+3V" H 1600 4060 30  0000 C CNN
F 2 "" H 1600 3950 60  0001 C CNN
F 3 "" H 1600 3950 60  0001 C CNN
	1    1600 3950
	1    0    0    -1  
$EndComp
Text GLabel 1900 5150 0    60   Input ~ 0
WIFI_RX
Text GLabel 1900 5050 0    60   Input ~ 0
WIFI_TX
$Comp
L GND #PWR3
U 1 1 4FB0BC87
P 2250 4850
F 0 "#PWR3" H 2250 4850 30  0001 C CNN
F 1 "GND" H 2250 4780 30  0001 C CNN
F 2 "" H 2250 4850 60  0001 C CNN
F 3 "" H 2250 4850 60  0001 C CNN
	1    2250 4850
	1    0    0    -1  
$EndComp
$Comp
L MCP1700 U2
U 1 1 4F9C1A72
P 7650 1650
F 0 "U2" H 7800 1454 60  0000 C CNN
F 1 "MCP1700" H 7650 1850 60  0000 C CNN
F 2 "" H 7650 1650 60  0001 C CNN
F 3 "" H 7650 1650 60  0001 C CNN
	1    7650 1650
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG1
U 1 1 4F9C2D4F
P 4100 1850
F 0 "#FLG1" H 4100 2120 30  0001 C CNN
F 1 "PWR_FLAG" H 4100 2080 30  0000 C CNN
F 2 "" H 4100 1850 60  0001 C CNN
F 3 "" H 4100 1850 60  0001 C CNN
	1    4100 1850
	1    0    0    1   
$EndComp
$Comp
L MAX1555 U1
U 1 1 4F9C2CDB
P 3700 1700
F 0 "U1" H 3800 1450 50  0000 L BNN
F 1 "MAX1555" H 3550 1870 50  0000 L BNN
F 2 "" H 3700 1700 60  0001 C CNN
F 3 "" H 3700 1700 60  0001 C CNN
	1    3700 1700
	1    0    0    -1  
$EndComp
Text GLabel 2400 1800 2    60   Input ~ 0
D-
Text GLabel 2400 1700 2    60   Input ~ 0
D+
$Comp
L +VUSB #PWR4
U 1 1 4F9C233A
P 2800 1450
F 0 "#PWR4" H 2800 1400 20  0001 C CNN
F 1 "+VUSB" H 2800 1550 30  0000 C CNN
F 2 "" H 2800 1450 60  0001 C CNN
F 3 "" H 2800 1450 60  0001 C CNN
	1    2800 1450
	1    0    0    -1  
$EndComp
Text GLabel 3550 2050 2    60   Input ~ 0
CHG
$Comp
L C C1
U 1 1 4F9C1CF4
P 2950 1950
F 0 "C1" H 3000 2050 50  0000 L CNN
F 1 "1uF" H 3000 1850 50  0000 L CNN
F 2 "" H 2950 1950 60  0001 C CNN
F 3 "" H 2950 1950 60  0001 C CNN
	1    2950 1950
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 4F9C1C02
P 4650 1950
F 0 "C3" H 4700 2050 50  0000 L CNN
F 1 "1uF" H 4700 1850 50  0000 L CNN
F 2 "" H 4650 1950 60  0001 C CNN
F 3 "" H 4650 1950 60  0001 C CNN
	1    4650 1950
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 4F9C1BDF
P 4350 1950
F 0 "C2" H 4400 2050 50  0000 L CNN
F 1 "1uF" H 4400 1850 50  0000 L CNN
F 2 "" H 4350 1950 60  0001 C CNN
F 3 "" H 4350 1950 60  0001 C CNN
	1    4350 1950
	1    0    0    -1  
$EndComp
Text Notes 4400 700  0    60   ~ 0
USB, LiPo charging, and power supply
$Comp
L C C9
U 1 1 4F9C1A7A
P 7050 1950
F 0 "C9" H 7100 2050 50  0000 L CNN
F 1 "1uF" H 7100 1850 50  0000 L CNN
F 2 "" H 7050 1950 60  0001 C CNN
F 3 "" H 7050 1950 60  0001 C CNN
	1    7050 1950
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 4F9C1A79
P 8250 1950
F 0 "C15" H 8300 2050 50  0000 L CNN
F 1 "1uF" H 8300 1850 50  0000 L CNN
F 2 "" H 8250 1950 60  0001 C CNN
F 3 "" H 8250 1950 60  0001 C CNN
	1    8250 1950
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR20
U 1 1 4F9C1A78
P 8650 1450
F 0 "#PWR20" H 8650 1360 30  0001 C CNN
F 1 "+3V" H 8650 1560 30  0000 C CNN
F 2 "" H 8650 1450 60  0001 C CNN
F 3 "" H 8650 1450 60  0001 C CNN
	1    8650 1450
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR7
U 1 1 4F9C1A77
P 5400 1450
F 0 "#PWR7" H 5400 1400 20  0001 C CNN
F 1 "+BATT" H 5400 1550 30  0000 C CNN
F 2 "" H 5400 1450 60  0001 C CNN
F 3 "" H 5400 1450 60  0001 C CNN
	1    5400 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR8
U 1 1 4F9C1A76
P 5400 2400
F 0 "#PWR8" H 5400 2400 30  0001 C CNN
F 1 "GND" H 5400 2330 30  0001 C CNN
F 2 "" H 5400 2400 60  0001 C CNN
F 3 "" H 5400 2400 60  0001 C CNN
	1    5400 2400
	1    0    0    -1  
$EndComp
$Comp
L BATTERY BT1
U 1 1 4F9C1A75
P 5150 1900
F 0 "BT1" H 5150 2100 50  0000 C CNN
F 1 "BATTERY" H 5150 1710 50  0000 C CNN
F 2 "" H 5150 1900 60  0001 C CNN
F 3 "" H 5150 1900 60  0001 C CNN
	1    5150 1900
	0    1    1    0   
$EndComp
Text GLabel 10350 9500 0    60   Input ~ 0
MISO
Text GLabel 10350 9600 0    60   Input ~ 0
SCLK
Text GLabel 10350 9700 0    60   Input ~ 0
RST
Text GLabel 11500 9600 2    60   Input ~ 0
MOSI
$Comp
L +3V #PWR24
U 1 1 4F9C1A71
P 11500 9250
F 0 "#PWR24" H 11500 9160 30  0001 C CNN
F 1 "+3V" H 11500 9360 30  0000 C CNN
F 2 "" H 11500 9250 60  0001 C CNN
F 3 "" H 11500 9250 60  0001 C CNN
	1    11500 9250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR25
U 1 1 4F9C1A70
P 11500 10500
F 0 "#PWR25" H 11500 10500 30  0001 C CNN
F 1 "GND" H 11500 10430 30  0001 C CNN
F 2 "" H 11500 10500 60  0001 C CNN
F 3 "" H 11500 10500 60  0001 C CNN
	1    11500 10500
	1    0    0    -1  
$EndComp
$Comp
L HOLE H1
U 1 1 4F9C1A6E
P 13500 9400
F 0 "H1" H 13460 9225 40  0000 L CNN
F 1 "HOLE" H 13500 9570 30  0001 C CNN
F 2 "" H 13500 9400 60  0001 C CNN
F 3 "" H 13500 9400 60  0001 C CNN
	1    13500 9400
	1    0    0    -1  
$EndComp
$Comp
L HOLE H3
U 1 1 4F9C1A6D
P 14200 9400
F 0 "H3" H 14160 9225 40  0000 L CNN
F 1 "HOLE" H 14200 9570 30  0001 C CNN
F 2 "" H 14200 9400 60  0001 C CNN
F 3 "" H 14200 9400 60  0001 C CNN
	1    14200 9400
	1    0    0    -1  
$EndComp
Text Notes 13650 8900 0    60   ~ 0
Mounting holes
Text Notes 10200 8900 0    60   ~ 0
In System Programming header\nand reset pushbutton
$Comp
L CONN_1 P9
U 1 1 4F9C1A6A
P 5600 1450
F 0 "P9" H 5680 1450 40  0000 L CNN
F 1 "CONN_1" H 5600 1505 30  0001 C CNN
F 2 "" H 5600 1450 60  0001 C CNN
F 3 "" H 5600 1450 60  0001 C CNN
	1    5600 1450
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P12
U 1 1 4F9C1A69
P 8450 1450
F 0 "P12" H 8530 1450 40  0000 L CNN
F 1 "CONN_1" H 8450 1505 30  0001 C CNN
F 2 "" H 8450 1450 60  0001 C CNN
F 3 "" H 8450 1450 60  0001 C CNN
	1    8450 1450
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P10
U 1 1 4F9C1A68
P 5600 2350
F 0 "P10" H 5680 2350 40  0000 L CNN
F 1 "CONN_1" H 5600 2405 30  0001 C CNN
F 2 "" H 5600 2350 60  0001 C CNN
F 3 "" H 5600 2350 60  0001 C CNN
	1    5600 2350
	0    -1   1    0   
$EndComp
$Comp
L AVR-ISP-6 ISP1
U 1 1 4F9C1A67
P 10950 9600
F 0 "ISP1" H 11125 9345 50  0000 C CNN
F 1 "AVR-ISP-6" H 10740 9815 50  0000 L BNN
F 2 "AVR-ISP-6" V 10450 9600 50  0001 C CNN
F 3 "" H 10950 9600 60  0001 C CNN
	1    10950 9600
	1    0    0    -1  
$EndComp
$Comp
L ESP8266-ESP-03 MOD1
U 1 1 552EE147
P 3050 5300
F 0 "MOD1" H 3050 6025 60  0000 C CNN
F 1 "ESP8266-ESP-03" H 3050 5175 60  0000 C CNN
F 2 "" H 3050 5300 60  0000 C CNN
F 3 "" H 3050 5300 60  0000 C CNN
	1    3050 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	13250 1900 13250 1950
Wire Wire Line
	13250 1950 14400 1950
Wire Wire Line
	14400 1650 14200 1650
Wire Wire Line
	14400 1450 14200 1450
Wire Wire Line
	14200 1350 14400 1350
Wire Wire Line
	14200 1150 14400 1150
Wire Wire Line
	14400 1050 14200 1050
Wire Wire Line
	14400 1250 14200 1250
Wire Wire Line
	14400 1550 14200 1550
Wire Wire Line
	14200 1750 14400 1750
Wire Wire Line
	14400 2050 12800 2050
Wire Wire Line
	12800 2050 12800 1900
Wire Wire Line
	14400 2150 13250 2150
Wire Wire Line
	13250 2150 13250 2250
Wire Wire Line
	10800 4950 11100 4950
Wire Wire Line
	1800 10750 1400 10750
Connection ~ 11500 10200
Wire Wire Line
	11400 10200 11500 10200
Wire Wire Line
	11150 2300 11150 2450
Wire Wire Line
	11150 1250 11150 1100
Wire Wire Line
	11150 1100 11050 1100
Wire Wire Line
	11150 1750 11150 1900
Wire Wire Line
	11500 9700 11500 10500
Wire Wire Line
	11150 5100 11150 5050
Wire Wire Line
	11150 5050 10800 5050
Wire Wire Line
	8800 3900 8500 3900
Wire Wire Line
	10200 1750 10200 1900
Wire Notes Line
	9350 900  9350 2500
Wire Wire Line
	11100 6950 10800 6950
Connection ~ 7350 9600
Wire Wire Line
	7350 9450 7350 9600
Connection ~ 3000 10200
Wire Wire Line
	3000 9950 3000 10200
Connection ~ 2800 10000
Wire Wire Line
	2800 9950 2800 10000
Wire Wire Line
	6900 6150 8400 6150
Connection ~ 8450 10100
Wire Wire Line
	8450 10100 8950 10100
Wire Wire Line
	8950 10100 8950 10000
Connection ~ 7150 1600
Wire Wire Line
	7150 1600 7150 1500
Wire Wire Line
	6950 1600 7250 1600
Wire Notes Line
	6250 8750 6250 11050
Wire Notes Line
	9700 8700 9700 11000
Wire Notes Line
	5950 3350 5950 8200
Wire Wire Line
	10800 5550 11100 5550
Wire Wire Line
	10800 5250 11100 5250
Wire Wire Line
	1900 4950 2450 4950
Wire Notes Line
	12050 3350 12050 8200
Wire Notes Line
	15750 8450 750  8450
Wire Wire Line
	10800 6850 11100 6850
Wire Wire Line
	2600 10200 3100 10200
Wire Wire Line
	2600 10000 3100 10000
Wire Wire Line
	2600 9450 3100 9450
Wire Wire Line
	10800 6250 11100 6250
Wire Wire Line
	10800 5950 11100 5950
Connection ~ 5550 10300
Wire Wire Line
	5550 10300 5200 10300
Wire Wire Line
	5200 10300 5200 10150
Wire Wire Line
	10800 4250 11100 4250
Wire Wire Line
	5550 10200 5550 10400
Wire Wire Line
	5550 8900 5550 9000
Wire Wire Line
	7400 5850 7600 5850
Wire Wire Line
	8600 5750 8100 5750
Wire Wire Line
	6900 6100 6900 6300
Wire Wire Line
	8600 5650 6900 5650
Wire Wire Line
	7950 9600 8150 9600
Connection ~ 8450 9100
Wire Wire Line
	8050 9100 8450 9100
Wire Wire Line
	10800 6750 11100 6750
Wire Wire Line
	10800 6550 11100 6550
Wire Wire Line
	6600 5000 6600 4950
Connection ~ 7400 4750
Wire Wire Line
	7400 5000 7400 4750
Wire Wire Line
	6600 5400 8200 5400
Connection ~ 6900 6650
Wire Wire Line
	8500 6050 8500 6250
Wire Wire Line
	8500 6050 8600 6050
Wire Wire Line
	8100 7150 7500 7150
Connection ~ 8100 6650
Wire Wire Line
	8100 6450 8100 6750
Wire Wire Line
	8100 6450 8600 6450
Wire Wire Line
	8600 6350 7500 6350
Wire Wire Line
	7500 6350 7500 6750
Connection ~ 7500 6650
Wire Wire Line
	7500 7150 7500 7350
Connection ~ 7400 4550
Wire Wire Line
	7400 4550 7400 4650
Wire Wire Line
	7400 4650 8600 4650
Connection ~ 7400 5400
Wire Wire Line
	7300 4550 8600 4550
Wire Wire Line
	6600 5400 6600 5550
Wire Wire Line
	7250 4250 8600 4250
Wire Wire Line
	6600 3950 6600 4850
Wire Wire Line
	6600 4750 8600 4750
Connection ~ 8500 7050
Wire Wire Line
	8500 7050 8600 7050
Wire Wire Line
	8500 6850 8500 7350
Wire Wire Line
	8500 6850 8600 6850
Wire Wire Line
	1600 3950 1600 4700
Wire Wire Line
	1600 4700 2450 4700
Wire Wire Line
	4100 1850 4100 1700
Connection ~ 5150 2200
Connection ~ 4650 2200
Wire Wire Line
	2400 2200 8250 2200
Wire Notes Line
	12050 8700 12050 10300
Connection ~ 2800 1600
Wire Wire Line
	2250 1600 3300 1600
Wire Wire Line
	2250 1700 2400 1700
Wire Wire Line
	3550 2050 3250 2050
Wire Wire Line
	3250 2050 3250 1800
Wire Wire Line
	3250 1800 3300 1800
Connection ~ 2950 1600
Wire Wire Line
	2950 2200 2950 2150
Wire Wire Line
	4650 2200 4650 2150
Wire Wire Line
	4350 1750 4350 1700
Wire Wire Line
	4350 1700 4100 1700
Connection ~ 3200 2200
Wire Wire Line
	7050 1750 7050 1600
Wire Wire Line
	11500 9250 11500 9500
Wire Wire Line
	11500 9500 11250 9500
Wire Wire Line
	11250 9600 11500 9600
Wire Wire Line
	10650 9600 10350 9600
Wire Wire Line
	8250 2200 8250 2150
Connection ~ 5400 1600
Wire Wire Line
	5400 1600 5400 1450
Connection ~ 7650 2200
Wire Wire Line
	7650 1900 7650 2200
Wire Wire Line
	8650 1600 8650 1450
Wire Wire Line
	5400 2200 5400 2400
Connection ~ 5400 2200
Wire Wire Line
	7050 2200 7050 2150
Connection ~ 7050 2200
Wire Wire Line
	8250 1600 8250 1750
Connection ~ 8250 1600
Connection ~ 7050 1600
Wire Wire Line
	10650 9500 10350 9500
Wire Wire Line
	10350 9700 10650 9700
Wire Wire Line
	11500 9700 11250 9700
Connection ~ 8450 1600
Connection ~ 5600 1600
Connection ~ 5600 2200
Wire Wire Line
	2950 1600 2950 1750
Wire Wire Line
	4650 1750 4650 1600
Connection ~ 4650 1600
Wire Wire Line
	4350 2200 4350 2150
Connection ~ 4350 2200
Connection ~ 2950 2200
Wire Wire Line
	3300 1700 3200 1700
Wire Wire Line
	3200 1700 3200 2200
Wire Wire Line
	2800 1450 2800 1600
Wire Wire Line
	2250 1800 2400 1800
Wire Wire Line
	2250 2000 2400 2000
Wire Wire Line
	2400 2000 2400 2200
Connection ~ 5150 1600
Wire Notes Line
	12050 900  12050 2500
Wire Wire Line
	8600 6950 8500 6950
Connection ~ 8500 6950
Wire Wire Line
	8500 7150 8600 7150
Connection ~ 8500 7150
Wire Wire Line
	6600 4850 8600 4850
Connection ~ 6600 4750
Connection ~ 6600 4550
Wire Wire Line
	8200 5000 8200 4550
Connection ~ 8200 4550
Wire Wire Line
	6600 4250 6750 4250
Connection ~ 6600 4250
Connection ~ 6600 5400
Wire Wire Line
	6600 4550 6700 4550
Connection ~ 7800 5400
Connection ~ 7000 5400
Wire Wire Line
	6900 7150 6900 7350
Wire Wire Line
	6900 6550 6900 6750
Wire Wire Line
	8600 5150 8500 5150
Wire Wire Line
	8500 5150 8500 4950
Wire Wire Line
	8500 4950 6600 4950
Wire Wire Line
	7800 5000 7800 4650
Connection ~ 7800 4650
Wire Wire Line
	7000 5000 7000 4850
Connection ~ 7000 4850
Wire Wire Line
	8500 6250 7200 6250
Wire Wire Line
	7200 6250 7200 6650
Wire Wire Line
	7200 6650 6900 6650
Wire Wire Line
	8500 3900 8500 4250
Connection ~ 8500 4250
Wire Wire Line
	10800 6650 11100 6650
Wire Wire Line
	11100 5350 10800 5350
Wire Wire Line
	8450 9050 8450 9400
Connection ~ 8050 9600
Wire Wire Line
	7200 9600 7450 9600
Wire Wire Line
	8450 9800 8450 10250
Wire Wire Line
	8450 10250 8500 10250
Wire Wire Line
	6900 5600 6900 5700
Connection ~ 6900 5650
Wire Wire Line
	8600 5950 8400 5950
Wire Wire Line
	8400 5950 8400 6150
Connection ~ 6900 6150
Wire Wire Line
	7600 5750 7400 5750
Wire Wire Line
	8100 5850 8600 5850
Wire Wire Line
	5550 9500 5550 9700
Wire Wire Line
	4850 9600 5550 9600
Connection ~ 5550 9600
Connection ~ 5200 9600
Wire Wire Line
	10800 5850 11100 5850
Wire Wire Line
	10800 6150 11100 6150
Wire Wire Line
	1400 9350 1650 9350
Wire Wire Line
	1650 10100 1400 10100
Wire Wire Line
	1400 10100 1400 10350
Wire Wire Line
	4100 1600 5700 1600
Wire Wire Line
	2600 10100 3100 10100
Wire Wire Line
	10800 7050 11100 7050
Wire Wire Line
	10800 6450 11100 6450
Wire Notes Line
	15750 3050 750  3050
Wire Wire Line
	10800 5450 11100 5450
Wire Wire Line
	10800 4550 11100 4550
Wire Notes Line
	4100 8700 4100 11000
Wire Wire Line
	8050 1600 8650 1600
Wire Wire Line
	11100 5650 10800 5650
Wire Wire Line
	7150 2200 7150 2300
Connection ~ 7150 2200
Wire Wire Line
	2900 9950 2900 10100
Connection ~ 2900 10100
Wire Wire Line
	5200 9450 5200 9750
Wire Wire Line
	8700 10000 8700 10100
Connection ~ 8700 10100
Wire Wire Line
	11100 4450 10800 4450
Wire Wire Line
	10100 1100 10200 1100
Wire Wire Line
	10200 1100 10200 1250
Wire Wire Line
	10200 2300 10200 2450
Wire Wire Line
	1950 4250 1950 4950
Connection ~ 1950 4950
Wire Wire Line
	10800 4350 11100 4350
Wire Wire Line
	10450 10200 10400 10200
Wire Wire Line
	10400 10200 10400 9700
Connection ~ 10400 9700
Wire Wire Line
	2400 10750 3100 10750
Wire Wire Line
	3100 10750 3100 11000
Wire Wire Line
	1900 5050 2450 5050
Wire Wire Line
	1900 5150 2450 5150
Text GLabel 3850 4700 2    60   Input ~ 0
WIFI_BOOTLDR
Wire Wire Line
	3650 4700 3850 4700
Wire Wire Line
	2050 4250 2050 5050
Connection ~ 2050 5050
Wire Wire Line
	2150 4250 2150 5150
Connection ~ 2150 5150
NoConn ~ 3650 4800
NoConn ~ 3650 5000
NoConn ~ 3650 5100
NoConn ~ 3650 5200
NoConn ~ 3650 5300
$Comp
L GND #PWR6
U 1 1 552EF380
P 3850 4950
F 0 "#PWR6" H 3850 4950 30  0001 C CNN
F 1 "GND" H 3850 4880 30  0001 C CNN
F 2 "" H 3850 4950 60  0001 C CNN
F 3 "" H 3850 4950 60  0001 C CNN
	1    3850 4950
	1    0    0    -1  
$EndComp
NoConn ~ 2450 5300
Wire Wire Line
	3650 4900 3850 4900
Wire Wire Line
	3850 4900 3850 4950
Wire Wire Line
	2450 4800 2250 4800
Wire Wire Line
	2250 4800 2250 4850
Text GLabel 11100 4550 2    60   Input ~ 0
WIFI_BOOTLDR
NoConn ~ 10800 4650
NoConn ~ 10800 4750
NoConn ~ 10800 5750
NoConn ~ 10800 7150
$Comp
L CONN_1 P7
U 1 1 552EF81F
P 3800 4100
F 0 "P7" H 3880 4100 40  0000 L CNN
F 1 "CONN_1" H 3800 4155 30  0001 C CNN
F 2 "" H 3800 4100 60  0001 C CNN
F 3 "" H 3800 4100 60  0001 C CNN
	1    3800 4100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3800 4250 3800 4700
Connection ~ 3800 4700
$EndSCHEMATC
