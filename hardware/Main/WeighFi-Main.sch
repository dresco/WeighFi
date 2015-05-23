EESchema Schematic File Version 2
LIBS:dresco
LIBS:WeighFi-Main-cache
EELAYER 24 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "WeighFi-Main"
Date "22 apr 2015"
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
L GND #PWR01
U 1 1 50BC8C46
P 13250 2250
F 0 "#PWR01" H 13250 2250 30  0001 C CNN
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
L +3V #PWR02
U 1 1 50BC8C45
P 12800 1900
F 0 "#PWR02" H 12800 1810 30  0001 C CNN
F 1 "+3V" H 12800 2000 30  0000 C CNN
F 2 "" H 12800 1900 60  0001 C CNN
F 3 "" H 12800 1900 60  0001 C CNN
	1    12800 1900
	-1   0    0    -1  
$EndComp
$Comp
L +BATT #PWR03
U 1 1 50BC8C44
P 13250 1900
F 0 "#PWR03" H 13250 1850 20  0001 C CNN
F 1 "+BATT" H 13250 2000 30  0000 C CNN
F 2 "" H 13250 1900 60  0001 C CNN
F 3 "" H 13250 1900 60  0001 C CNN
	1    13250 1900
	-1   0    0    -1  
$EndComp
Text GLabel 14200 1450 0    60   Input ~ 0
ADC_SPEED
Text GLabel 14200 1550 0    60   Input ~ 0
ADC_ENABLE
Text GLabel 14200 1650 0    60   Input ~ 0
ADC_CLOCK
Text GLabel 14200 1750 0    60   Input ~ 0
ADC_DATA
NoConn ~ 14400 1850
Text Notes 13250 700  0    60   ~ 0
Display board connector
Text GLabel 10350 4950 2    60   Input ~ 0
DEBUG
Text GLabel 1400 10750 0    60   Input ~ 0
DEBUG
$Comp
L GND #PWR04
U 1 1 4FE9FA1E
P 3100 11000
F 0 "#PWR04" H 3100 11000 30  0001 C CNN
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
Text GLabel 7200 1100 0    60   Input ~ 0
STATUS
Text GLabel 10350 6450 2    60   Input ~ 0
STATUS
$Comp
L R R6
U 1 1 4FDB64DE
P 7300 1500
F 0 "R6" V 7380 1500 50  0000 C CNN
F 1 "1K8" V 7300 1500 50  0000 C CNN
F 2 "" H 7300 1500 60  0001 C CNN
F 3 "" H 7300 1500 60  0001 C CNN
	1    7300 1500
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 4FDB64DD
P 7300 2100
F 0 "D1" H 7300 2200 50  0000 C CNN
F 1 "LED" H 7300 2000 50  0000 C CNN
F 2 "" H 7300 2100 60  0001 C CNN
F 3 "" H 7300 2100 60  0001 C CNN
	1    7300 2100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR05
U 1 1 4FDB64DC
P 7300 2450
F 0 "#PWR05" H 7300 2450 30  0001 C CNN
F 1 "GND" H 7300 2380 30  0001 C CNN
F 2 "" H 7300 2450 60  0001 C CNN
F 3 "" H 7300 2450 60  0001 C CNN
	1    7300 2450
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P12
U 1 1 4FDB64DB
P 7300 950
F 0 "P12" H 7380 950 40  0000 L CNN
F 1 "CONN_1" H 7300 1005 30  0001 C CNN
F 2 "" H 7300 950 60  0001 C CNN
F 3 "" H 7300 950 60  0001 C CNN
	1    7300 950 
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
L CONN_1 P5
U 1 1 4FCCF97C
P 2150 4100
F 0 "P5" H 2230 4100 40  0000 L CNN
F 1 "CONN_1" H 2150 4155 30  0001 C CNN
F 2 "" H 2150 4100 60  0001 C CNN
F 3 "" H 2150 4100 60  0001 C CNN
	1    2150 4100
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P4
U 1 1 4FCCF965
P 2050 4100
F 0 "P4" H 2130 4100 40  0000 L CNN
F 1 "CONN_1" H 2050 4155 30  0001 C CNN
F 2 "" H 2050 4100 60  0001 C CNN
F 3 "" H 2050 4100 60  0001 C CNN
	1    2050 4100
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P3
U 1 1 4FCCF940
P 1950 4100
F 0 "P3" H 2030 4100 40  0000 L CNN
F 1 "CONN_1" H 1950 4155 30  0001 C CNN
F 2 "" H 1950 4100 60  0001 C CNN
F 3 "" H 1950 4100 60  0001 C CNN
	1    1950 4100
	0    -1   -1   0   
$EndComp
$Comp
L ATMEGA16U4/32U4 U2
U 1 1 4FCCF7C0
P 8950 5650
F 0 "U2" H 8900 5750 50  0000 L BNN
F 1 "ATMEGA16U4/32U4" H 8550 5920 50  0000 L BNN
F 2 "" H 8950 5650 60  0001 C CNN
F 3 "" H 8950 5650 60  0001 C CNN
	1    8950 5650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 4FCCF6AD
P 10400 5100
F 0 "#PWR06" H 10400 5100 30  0001 C CNN
F 1 "GND" H 10400 5030 30  0001 C CNN
F 2 "" H 10400 5100 60  0001 C CNN
F 3 "" H 10400 5100 60  0001 C CNN
	1    10400 5100
	1    0    0    -1  
$EndComp
Text Notes 7000 700  0    60   ~ 0
Status LED
Text GLabel 11350 5750 2    60   Input ~ 0
WIFI_ENABLE
Text GLabel 10350 7050 2    60   Input ~ 0
LCD_CLOCK
$Comp
L CONN_1 P13
U 1 1 4FC7A464
P 7350 9300
F 0 "P13" H 7430 9300 40  0000 L CNN
F 1 "CONN_1" H 7350 9355 30  0001 C CNN
F 2 "" H 7350 9300 60  0001 C CNN
F 3 "" H 7350 9300 60  0001 C CNN
	1    7350 9300
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P14
U 1 1 4FC7A442
P 8700 9850
F 0 "P14" H 8780 9850 40  0000 L CNN
F 1 "CONN_1" H 8700 9905 30  0001 C CNN
F 2 "" H 8700 9850 60  0001 C CNN
F 3 "" H 8700 9850 60  0001 C CNN
	1    8700 9850
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P11
U 1 1 4FC7A426
P 5200 9300
F 0 "P11" H 5280 9300 40  0000 L CNN
F 1 "CONN_1" H 5200 9355 30  0001 C CNN
F 2 "" H 5200 9300 60  0001 C CNN
F 3 "" H 5200 9300 60  0001 C CNN
	1    5200 9300
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P8
U 1 1 4FC7A40D
P 3000 9800
F 0 "P8" H 3080 9800 40  0000 L CNN
F 1 "CONN_1" H 3000 9855 30  0001 C CNN
F 2 "" H 3000 9800 60  0001 C CNN
F 3 "" H 3000 9800 60  0001 C CNN
	1    3000 9800
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P7
U 1 1 4FC7A40A
P 2900 9800
F 0 "P7" H 2980 9800 40  0000 L CNN
F 1 "CONN_1" H 2900 9855 30  0001 C CNN
F 2 "" H 2900 9800 60  0001 C CNN
F 3 "" H 2900 9800 60  0001 C CNN
	1    2900 9800
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P6
U 1 1 4FC7A3FD
P 2800 9800
F 0 "P6" H 2880 9800 40  0000 L CNN
F 1 "CONN_1" H 2800 9855 30  0001 C CNN
F 2 "" H 2800 9800 60  0001 C CNN
F 3 "" H 2800 9800 60  0001 C CNN
	1    2800 9800
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG07
U 1 1 4FC509BC
P 3450 2300
F 0 "#FLG07" H 3450 2570 30  0001 C CNN
F 1 "PWR_FLAG" H 3450 2530 30  0000 C CNN
F 2 "" H 3450 2300 60  0001 C CNN
F 3 "" H 3450 2300 60  0001 C CNN
	1    3450 2300
	1    0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG08
U 1 1 4FC50659
P 8950 10000
F 0 "#FLG08" H 8950 10095 30  0001 C CNN
F 1 "PWR_FLAG" H 8950 10180 30  0000 C CNN
F 2 "" H 8950 10000 60  0001 C CNN
F 3 "" H 8950 10000 60  0001 C CNN
	1    8950 10000
	1    0    0    -1  
$EndComp
Text GLabel 10350 6150 2    60   Input ~ 0
LCD_ENABLE
$Comp
L PWR_FLAG #FLG09
U 1 1 4FC50360
P 3450 1500
F 0 "#FLG09" H 3450 1770 30  0001 C CNN
F 1 "PWR_FLAG" H 3450 1730 30  0000 C CNN
F 2 "" H 3450 1500 60  0001 C CNN
F 3 "" H 3450 1500 60  0001 C CNN
	1    3450 1500
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
Text Notes 8150 3550 0    60   ~ 0
MCU
Text GLabel 11350 5550 2    60   Input ~ 0
WIFI_RX
Text GLabel 11350 5450 2    60   Input ~ 0
WIFI_TX
Text GLabel 1900 4950 0    60   Input ~ 0
WIFI_ENABLE
Text GLabel 10350 4750 2    60   Input ~ 0
SW_KG
Text GLabel 10350 4650 2    60   Input ~ 0
SW_LB
Text GLabel 10350 4550 2    60   Input ~ 0
SW_ST
Text GLabel 3100 10200 2    60   Input ~ 0
SW_ST
Text GLabel 3100 10100 2    60   Input ~ 0
SW_LB
Text GLabel 3100 10000 2    60   Input ~ 0
SW_KG
Text GLabel 3250 1600 0    60   Input ~ 0
PWR_SW_OUT
Text GLabel 2000 1600 2    60   Input ~ 0
PWR_SW_IN
$Comp
L GND #PWR010
U 1 1 4FC34993
P 1400 10350
F 0 "#PWR010" H 1400 10350 30  0001 C CNN
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
Text GLabel 10350 6850 2    60   Input ~ 0
ADC_ENABLE
Text GLabel 10350 6950 2    60   Input ~ 0
ADC_SPEED
Text GLabel 10350 5850 2    60   Input ~ 0
ADC_DATA
Text GLabel 10350 5950 2    60   Input ~ 0
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
Text GLabel 10350 4250 2    60   Input ~ 0
BAT_ADC
Text GLabel 4850 9600 0    60   Input ~ 0
BAT_ADC
$Comp
L GND #PWR011
U 1 1 4FBF8CF7
P 5550 10400
F 0 "#PWR011" H 5550 10400 30  0001 C CNN
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
F 1 "470K" V 5550 9950 50  0000 C CNN
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
F 1 "670K" V 5550 9250 50  0000 C CNN
F 2 "" H 5550 9250 60  0001 C CNN
F 3 "" H 5550 9250 60  0001 C CNN
	1    5550 9250
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR012
U 1 1 4FBF8CD4
P 5550 8900
F 0 "#PWR012" H 5550 8850 20  0001 C CNN
F 1 "+BATT" H 5550 9000 30  0000 C CNN
F 2 "" H 5550 8900 60  0001 C CNN
F 3 "" H 5550 8900 60  0001 C CNN
	1    5550 8900
	1    0    0    -1  
$EndComp
$Comp
L +VUSB #PWR013
U 1 1 4FBF89B4
P 6150 5600
F 0 "#PWR013" H 6150 5550 20  0001 C CNN
F 1 "+VUSB" H 6150 5700 30  0000 C CNN
F 2 "" H 6150 5600 60  0001 C CNN
F 3 "" H 6150 5600 60  0001 C CNN
	1    6150 5600
	1    0    0    -1  
$EndComp
Text GLabel 6650 5850 0    60   Input ~ 0
D-
Text GLabel 6650 5750 0    60   Input ~ 0
D+
NoConn ~ 10200 1900
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
L R R7
U 1 1 4FBEC438
P 7700 9600
F 0 "R7" V 7780 9600 50  0000 C CNN
F 1 "100" V 7700 9600 50  0000 C CNN
F 2 "" H 7700 9600 60  0001 C CNN
F 3 "" H 7700 9600 60  0001 C CNN
	1    7700 9600
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 4FBEC3ED
P 8050 9350
F 0 "R8" V 8130 9350 50  0000 C CNN
F 1 "100K" V 8050 9350 50  0000 C CNN
F 2 "" H 8050 9350 60  0001 C CNN
F 3 "" H 8050 9350 60  0001 C CNN
	1    8050 9350
	-1   0    0    1   
$EndComp
Text GLabel 7200 9600 0    60   Input ~ 0
LCD_ENABLE
$Comp
L +3V #PWR014
U 1 1 4FBEC3B3
P 8450 9050
F 0 "#PWR014" H 8450 8960 30  0001 C CNN
F 1 "+3V" H 8450 9160 30  0000 C CNN
F 2 "" H 8450 9050 60  0001 C CNN
F 3 "" H 8450 9050 60  0001 C CNN
	1    8450 9050
	1    0    0    -1  
$EndComp
Text GLabel 10350 5250 2    60   Input ~ 0
I2C_CLOCK
Text GLabel 10350 5350 2    60   Input ~ 0
I2C_DATA
Text GLabel 8050 3900 2    60   Input ~ 0
RST
Text GLabel 10350 6550 2    60   Input ~ 0
SCLK
Text GLabel 10350 6750 2    60   Input ~ 0
MISO
Text GLabel 10350 6650 2    60   Input ~ 0
MOSI
$Comp
L GND #PWR015
U 1 1 4FBE97EF
P 6150 6300
F 0 "#PWR015" H 6150 6300 30  0001 C CNN
F 1 "GND" H 6150 6230 30  0001 C CNN
F 2 "" H 6150 6300 60  0001 C CNN
F 3 "" H 6150 6300 60  0001 C CNN
	1    6150 6300
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 4FBE97BB
P 6150 5900
F 0 "C6" H 6200 6000 50  0000 L CNN
F 1 "10uF" H 6200 5800 50  0000 L CNN
F 2 "" H 6150 5900 60  0001 C CNN
F 3 "" H 6150 5900 60  0001 C CNN
	1    6150 5900
	1    0    0    -1  
$EndComp
$Comp
L USB USB1
U 1 1 4FBE9598
P 10000 1800
F 0 "USB1" H 9900 1450 60  0000 C CNN
F 1 "USB" H 10000 1450 60  0001 C CNN
F 2 "" H 10000 1800 60  0001 C CNN
F 3 "" H 10000 1800 60  0001 C CNN
	1    10000 1800
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 4FBE83F1
P 7100 5850
F 0 "R5" V 7050 5650 50  0000 C CNN
F 1 "22" V 7100 5850 50  0000 C CNN
F 2 "" H 7100 5850 60  0001 C CNN
F 3 "" H 7100 5850 60  0001 C CNN
	1    7100 5850
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 4FBE83EA
P 7100 5750
F 0 "R4" V 7150 5550 50  0000 C CNN
F 1 "22" V 7100 5750 50  0000 C CNN
F 2 "" H 7100 5750 60  0001 C CNN
F 3 "" H 7100 5750 60  0001 C CNN
	1    7100 5750
	0    -1   -1   0   
$EndComp
$Comp
L C C7
U 1 1 4FBE82BB
P 6150 6950
F 0 "C7" H 6200 7050 50  0000 L CNN
F 1 "1uF" H 6200 6850 50  0000 L CNN
F 2 "" H 6150 6950 60  0001 C CNN
F 3 "" H 6150 6950 60  0001 C CNN
	1    6150 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 4FBE8254
P 6150 7350
F 0 "#PWR016" H 6150 7350 30  0001 C CNN
F 1 "GND" H 6150 7280 30  0001 C CNN
F 2 "" H 6150 7350 60  0001 C CNN
F 3 "" H 6150 7350 60  0001 C CNN
	1    6150 7350
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR017
U 1 1 4FBE823E
P 6150 6550
F 0 "#PWR017" H 6150 6460 30  0001 C CNN
F 1 "+3V" H 6150 6660 30  0000 C CNN
F 2 "" H 6150 6550 60  0001 C CNN
F 3 "" H 6150 6550 60  0001 C CNN
	1    6150 6550
	1    0    0    -1  
$EndComp
NoConn ~ 7850 5550
$Comp
L CRYSTAL X1
U 1 1 4FBE75E9
P 7050 6650
F 0 "X1" H 7050 6800 60  0000 C CNN
F 1 "CRYSTAL" H 7050 6500 60  0000 C CNN
F 2 "" H 7050 6650 60  0001 C CNN
F 3 "" H 7050 6650 60  0001 C CNN
	1    7050 6650
	-1   0    0    1   
$EndComp
$Comp
L C C10
U 1 1 4FBE75E8
P 6750 6950
F 0 "C10" H 6800 7050 50  0000 L CNN
F 1 "22pF" H 6800 6850 50  0000 L CNN
F 2 "" H 6750 6950 60  0001 C CNN
F 3 "" H 6750 6950 60  0001 C CNN
	1    6750 6950
	-1   0    0    1   
$EndComp
$Comp
L C C12
U 1 1 4FBE75E7
P 7350 6950
F 0 "C12" H 7400 7050 50  0000 L CNN
F 1 "22pF" H 7400 6850 50  0000 L CNN
F 2 "" H 7350 6950 60  0001 C CNN
F 3 "" H 7350 6950 60  0001 C CNN
	1    7350 6950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR018
U 1 1 4FBE75E6
P 6750 7350
F 0 "#PWR018" H 6750 7350 30  0001 C CNN
F 1 "GND" H 6750 7280 30  0001 C CNN
F 2 "" H 6750 7350 60  0001 C CNN
F 3 "" H 6750 7350 60  0001 C CNN
	1    6750 7350
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 4FB6D42C
P 6250 4550
F 0 "L1" V 6200 4350 40  0000 C CNN
F 1 "INDUCTOR" V 6200 4600 40  0000 C CNN
F 2 "" H 6250 4550 60  0001 C CNN
F 3 "" H 6250 4550 60  0001 C CNN
	1    6250 4550
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 4FB6D295
P 6250 4250
F 0 "R3" V 6330 4250 50  0000 C CNN
F 1 "10K" V 6250 4250 50  0000 C CNN
F 2 "" H 6250 4250 60  0001 C CNN
F 3 "" H 6250 4250 60  0001 C CNN
	1    6250 4250
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 4FB6D239
P 6250 5200
F 0 "C8" H 6300 5300 50  0000 L CNN
F 1 "0.1uF" H 6300 5100 50  0000 L CNN
F 2 "" H 6250 5200 60  0001 C CNN
F 3 "" H 6250 5200 60  0001 C CNN
	1    6250 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 4FB6D221
P 5850 5550
F 0 "#PWR019" H 5850 5550 30  0001 C CNN
F 1 "GND" H 5850 5480 30  0001 C CNN
F 2 "" H 5850 5550 60  0001 C CNN
F 3 "" H 5850 5550 60  0001 C CNN
	1    5850 5550
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 4FB6D1D8
P 7450 5200
F 0 "C13" H 7500 5300 50  0000 L CNN
F 1 "0.1uF" H 7500 5100 50  0000 L CNN
F 2 "" H 7450 5200 60  0001 C CNN
F 3 "" H 7450 5200 60  0001 C CNN
	1    7450 5200
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 4FB6D1D6
P 5850 5200
F 0 "C5" H 5900 5300 50  0000 L CNN
F 1 "0.1uF" H 5900 5100 50  0000 L CNN
F 2 "" H 5850 5200 60  0001 C CNN
F 3 "" H 5850 5200 60  0001 C CNN
	1    5850 5200
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 4FB6D1D3
P 6650 5200
F 0 "C9" H 6700 5300 50  0000 L CNN
F 1 "0.1uF" H 6700 5100 50  0000 L CNN
F 2 "" H 6650 5200 60  0001 C CNN
F 3 "" H 6650 5200 60  0001 C CNN
	1    6650 5200
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 4FB6D1B1
P 7050 5200
F 0 "C11" H 7100 5300 50  0000 L CNN
F 1 "0.1uF" H 7100 5100 50  0000 L CNN
F 2 "" H 7050 5200 60  0001 C CNN
F 3 "" H 7050 5200 60  0001 C CNN
	1    7050 5200
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR020
U 1 1 4FB4FBCB
P 5850 3950
F 0 "#PWR020" H 5850 3860 30  0001 C CNN
F 1 "+3V" H 5850 4060 30  0000 C CNN
F 2 "" H 5850 3950 60  0001 C CNN
F 3 "" H 5850 3950 60  0001 C CNN
	1    5850 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 4FB4FBAE
P 7750 7350
F 0 "#PWR021" H 7750 7350 30  0001 C CNN
F 1 "GND" H 7750 7280 30  0001 C CNN
F 2 "" H 7750 7350 60  0001 C CNN
F 3 "" H 7750 7350 60  0001 C CNN
	1    7750 7350
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR022
U 1 1 4FB1357C
P 1600 3950
F 0 "#PWR022" H 1600 3860 30  0001 C CNN
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
L GND #PWR023
U 1 1 4FB0BC87
P 2250 4850
F 0 "#PWR023" H 2250 4850 30  0001 C CNN
F 1 "GND" H 2250 4780 30  0001 C CNN
F 2 "" H 2250 4850 60  0001 C CNN
F 3 "" H 2250 4850 60  0001 C CNN
	1    2250 4850
	1    0    0    -1  
$EndComp
$Comp
L MCP1700 U1
U 1 1 4F9C1A72
P 3950 1650
F 0 "U1" H 4100 1454 60  0000 C CNN
F 1 "MCP1700" H 3950 1850 60  0000 C CNN
F 2 "" H 3950 1650 60  0001 C CNN
F 3 "" H 3950 1650 60  0001 C CNN
	1    3950 1650
	1    0    0    -1  
$EndComp
Text GLabel 10350 1800 2    60   Input ~ 0
D-
Text GLabel 10350 1700 2    60   Input ~ 0
D+
$Comp
L +VUSB #PWR024
U 1 1 4F9C233A
P 10750 1450
F 0 "#PWR024" H 10750 1400 20  0001 C CNN
F 1 "+VUSB" H 10750 1550 30  0000 C CNN
F 2 "" H 10750 1450 60  0001 C CNN
F 3 "" H 10750 1450 60  0001 C CNN
	1    10750 1450
	1    0    0    -1  
$EndComp
Text Notes 2300 700  0    60   ~ 0
Power supply
$Comp
L C C2
U 1 1 4F9C1A7A
P 3350 1950
F 0 "C2" H 3400 2050 50  0000 L CNN
F 1 "1uF" H 3400 1850 50  0000 L CNN
F 2 "" H 3350 1950 60  0001 C CNN
F 3 "" H 3350 1950 60  0001 C CNN
	1    3350 1950
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 4F9C1A79
P 4550 1950
F 0 "C3" H 4600 2050 50  0000 L CNN
F 1 "1uF" H 4600 1850 50  0000 L CNN
F 2 "" H 4550 1950 60  0001 C CNN
F 3 "" H 4550 1950 60  0001 C CNN
	1    4550 1950
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR025
U 1 1 4F9C1A78
P 5350 1450
F 0 "#PWR025" H 5350 1360 30  0001 C CNN
F 1 "+3V" H 5350 1560 30  0000 C CNN
F 2 "" H 5350 1450 60  0001 C CNN
F 3 "" H 5350 1450 60  0001 C CNN
	1    5350 1450
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR026
U 1 1 4F9C1A77
P 1700 1450
F 0 "#PWR026" H 1700 1400 20  0001 C CNN
F 1 "+BATT" H 1700 1550 30  0000 C CNN
F 2 "" H 1700 1450 60  0001 C CNN
F 3 "" H 1700 1450 60  0001 C CNN
	1    1700 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 4F9C1A76
P 1700 2400
F 0 "#PWR027" H 1700 2400 30  0001 C CNN
F 1 "GND" H 1700 2330 30  0001 C CNN
F 2 "" H 1700 2400 60  0001 C CNN
F 3 "" H 1700 2400 60  0001 C CNN
	1    1700 2400
	1    0    0    -1  
$EndComp
$Comp
L BATTERY BT1
U 1 1 4F9C1A75
P 950 1900
F 0 "BT1" H 950 2100 50  0000 C CNN
F 1 "BATTERY" H 950 1710 50  0000 C CNN
F 2 "" H 950 1900 60  0001 C CNN
F 3 "" H 950 1900 60  0001 C CNN
	1    950  1900
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
L +3V #PWR028
U 1 1 4F9C1A71
P 11500 9250
F 0 "#PWR028" H 11500 9160 30  0001 C CNN
F 1 "+3V" H 11500 9360 30  0000 C CNN
F 2 "" H 11500 9250 60  0001 C CNN
F 3 "" H 11500 9250 60  0001 C CNN
	1    11500 9250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 4F9C1A70
P 11500 10500
F 0 "#PWR029" H 11500 10500 30  0001 C CNN
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
L CONN_1 P1
U 1 1 4F9C1A6A
P 1900 1450
F 0 "P1" H 1980 1450 40  0000 L CNN
F 1 "CONN_1" H 1900 1505 30  0001 C CNN
F 2 "" H 1900 1450 60  0001 C CNN
F 3 "" H 1900 1450 60  0001 C CNN
	1    1900 1450
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P10
U 1 1 4F9C1A69
P 5200 1450
F 0 "P10" H 5280 1450 40  0000 L CNN
F 1 "CONN_1" H 5200 1505 30  0001 C CNN
F 2 "" H 5200 1450 60  0001 C CNN
F 3 "" H 5200 1450 60  0001 C CNN
	1    5200 1450
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P2
U 1 1 4F9C1A68
P 1900 2350
F 0 "P2" H 1980 2350 40  0000 L CNN
F 1 "CONN_1" H 1900 2405 30  0001 C CNN
F 2 "" H 1900 2350 60  0001 C CNN
F 3 "" H 1900 2350 60  0001 C CNN
	1    1900 2350
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
Text GLabel 3850 4700 2    60   Input ~ 0
WIFI_BOOTLDR
NoConn ~ 3650 4800
NoConn ~ 3650 5000
NoConn ~ 3650 5100
NoConn ~ 3650 5200
NoConn ~ 3650 5300
$Comp
L GND #PWR030
U 1 1 552EF380
P 3850 4950
F 0 "#PWR030" H 3850 4950 30  0001 C CNN
F 1 "GND" H 3850 4880 30  0001 C CNN
F 2 "" H 3850 4950 60  0001 C CNN
F 3 "" H 3850 4950 60  0001 C CNN
	1    3850 4950
	1    0    0    -1  
$EndComp
NoConn ~ 2450 5300
Text GLabel 11350 5650 2    60   Input ~ 0
WIFI_BOOTLDR
$Comp
L CONN_1 P9
U 1 1 552EF81F
P 3800 4100
F 0 "P9" H 3880 4100 40  0000 L CNN
F 1 "CONN_1" H 3800 4155 30  0001 C CNN
F 2 "" H 3800 4100 60  0001 C CNN
F 3 "" H 3800 4100 60  0001 C CNN
	1    3800 4100
	0    -1   -1   0   
$EndComp
Text GLabel 10350 7150 2    60   Input ~ 0
MCU_WAKE
NoConn ~ 10050 4350
NoConn ~ 10050 4450
$Comp
L R R13
U 1 1 552FE189
P 13650 4650
F 0 "R13" V 13730 4650 50  0000 C CNN
F 1 "1M" V 13650 4650 50  0000 C CNN
F 2 "~" H 13650 4650 60  0000 C CNN
F 3 "~" H 13650 4650 60  0000 C CNN
	1    13650 4650
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR031
U 1 1 552FE1A5
P 13650 6150
F 0 "#PWR031" H 13650 6150 30  0001 C CNN
F 1 "GND" H 13650 6080 30  0001 C CNN
F 2 "" H 13650 6150 60  0001 C CNN
F 3 "" H 13650 6150 60  0001 C CNN
	1    13650 6150
	1    0    0    -1  
$EndComp
$Comp
L +3V #PWR032
U 1 1 552FE1AB
P 13650 4250
F 0 "#PWR032" H 13650 4160 30  0001 C CNN
F 1 "+3V" H 13650 4360 30  0000 C CNN
F 2 "" H 13650 4250 60  0001 C CNN
F 3 "" H 13650 4250 60  0001 C CNN
	1    13650 4250
	1    0    0    -1  
$EndComp
Text GLabel 14250 5050 2    60   Input ~ 0
MCU_WAKE
Text Notes 14150 3550 2    60   ~ 0
Vibration Sensor
$Comp
L CONN_1 P15
U 1 1 552FECF6
P 14000 4400
F 0 "P15" H 14080 4400 40  0000 L CNN
F 1 "CONN_1" H 14000 4455 30  0001 C CNN
F 2 "" H 14000 4400 60  0001 C CNN
F 3 "" H 14000 4400 60  0001 C CNN
	1    14000 4400
	0    -1   -1   0   
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
	1800 10750 1400 10750
Connection ~ 11500 10200
Wire Wire Line
	11400 10200 11500 10200
Wire Wire Line
	7300 2300 7300 2450
Wire Wire Line
	7300 1250 7300 1100
Wire Wire Line
	7300 1100 7200 1100
Wire Wire Line
	7300 1750 7300 1900
Wire Wire Line
	11500 9700 11500 10500
Wire Wire Line
	10400 5100 10400 5050
Wire Wire Line
	10400 5050 10050 5050
Wire Wire Line
	8050 3900 7750 3900
Wire Notes Line
	9050 900  9050 2500
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
	6150 6150 7650 6150
Connection ~ 8450 10100
Wire Wire Line
	8450 10100 8950 10100
Wire Wire Line
	8950 10100 8950 10000
Connection ~ 3450 1600
Wire Wire Line
	3450 1600 3450 1500
Wire Wire Line
	3250 1600 3550 1600
Wire Notes Line
	6250 8750 6250 11050
Wire Notes Line
	9700 8700 9700 11000
Wire Notes Line
	5200 3350 5200 8200
Wire Wire Line
	10050 5250 10350 5250
Wire Wire Line
	1900 4950 2450 4950
Wire Notes Line
	12050 3350 12050 8200
Wire Notes Line
	15750 8450 750  8450
Wire Wire Line
	2600 10200 3100 10200
Wire Wire Line
	2600 10000 3100 10000
Wire Wire Line
	2600 9450 3100 9450
Connection ~ 5550 10300
Wire Wire Line
	5550 10300 5200 10300
Wire Wire Line
	5200 10300 5200 10150
Wire Wire Line
	10050 4250 10350 4250
Wire Wire Line
	5550 10200 5550 10400
Wire Wire Line
	5550 8900 5550 9000
Wire Wire Line
	6650 5850 6850 5850
Wire Wire Line
	7850 5750 7350 5750
Wire Wire Line
	6150 6100 6150 6300
Wire Wire Line
	7850 5650 6150 5650
Wire Wire Line
	7950 9600 8150 9600
Connection ~ 8450 9100
Wire Wire Line
	8050 9100 8450 9100
Wire Wire Line
	10050 6750 10350 6750
Wire Wire Line
	10050 6550 10350 6550
Wire Wire Line
	5850 5000 5850 4950
Connection ~ 6650 4750
Wire Wire Line
	6650 5000 6650 4750
Wire Wire Line
	5850 5400 7450 5400
Connection ~ 6150 6650
Wire Wire Line
	7750 6050 7750 6250
Wire Wire Line
	7750 6050 7850 6050
Wire Wire Line
	7350 7150 6750 7150
Connection ~ 7350 6650
Wire Wire Line
	7350 6450 7350 6750
Wire Wire Line
	7350 6450 7850 6450
Wire Wire Line
	7850 6350 6750 6350
Wire Wire Line
	6750 6350 6750 6750
Connection ~ 6750 6650
Wire Wire Line
	6750 7150 6750 7350
Connection ~ 6650 4550
Wire Wire Line
	6650 4550 6650 4650
Wire Wire Line
	6650 4650 7850 4650
Connection ~ 6650 5400
Wire Wire Line
	6550 4550 7850 4550
Wire Wire Line
	5850 5400 5850 5550
Wire Wire Line
	6500 4250 7850 4250
Wire Wire Line
	5850 3950 5850 4850
Wire Wire Line
	5850 4750 7850 4750
Connection ~ 7750 7050
Wire Wire Line
	7750 7050 7850 7050
Wire Wire Line
	7750 6850 7750 7350
Wire Wire Line
	7750 6850 7850 6850
Wire Wire Line
	1600 3950 1600 4700
Wire Wire Line
	1600 4700 2450 4700
Connection ~ 950  2200
Wire Notes Line
	12050 8700 12050 10300
Wire Wire Line
	10200 1600 10750 1600
Wire Wire Line
	10200 1700 10350 1700
Wire Wire Line
	3350 1750 3350 1600
Wire Wire Line
	11500 9250 11500 9500
Wire Wire Line
	11500 9500 11250 9500
Wire Wire Line
	11250 9600 11500 9600
Wire Wire Line
	10650 9600 10350 9600
Wire Wire Line
	4550 2200 4550 2150
Connection ~ 1700 1600
Wire Wire Line
	1700 1600 1700 1450
Connection ~ 3950 2200
Wire Wire Line
	3950 2200 3950 1900
Wire Wire Line
	5350 1600 5350 1450
Wire Wire Line
	1700 2200 1700 2400
Connection ~ 1700 2200
Wire Wire Line
	3350 2200 3350 2150
Connection ~ 3350 2200
Wire Wire Line
	4550 1600 4550 1750
Connection ~ 4550 1600
Connection ~ 3350 1600
Wire Wire Line
	10650 9500 10350 9500
Wire Wire Line
	10350 9700 10650 9700
Wire Wire Line
	11500 9700 11250 9700
Connection ~ 5200 1600
Connection ~ 1900 1600
Connection ~ 1900 2200
Wire Wire Line
	10750 1600 10750 1450
Wire Wire Line
	10200 1800 10350 1800
Connection ~ 950  1600
Wire Notes Line
	12050 900  12050 2500
Wire Wire Line
	7850 6950 7750 6950
Connection ~ 7750 6950
Wire Wire Line
	7750 7150 7850 7150
Connection ~ 7750 7150
Wire Wire Line
	5850 4850 7850 4850
Connection ~ 5850 4750
Connection ~ 5850 4550
Wire Wire Line
	7450 5000 7450 4550
Connection ~ 7450 4550
Wire Wire Line
	5850 4250 6000 4250
Connection ~ 5850 4250
Connection ~ 5850 5400
Wire Wire Line
	5850 4550 5950 4550
Connection ~ 7050 5400
Connection ~ 6250 5400
Wire Wire Line
	6150 7150 6150 7350
Wire Wire Line
	6150 6550 6150 6750
Wire Wire Line
	7850 5150 7750 5150
Wire Wire Line
	7750 5150 7750 4950
Wire Wire Line
	7750 4950 5850 4950
Wire Wire Line
	7050 5000 7050 4650
Connection ~ 7050 4650
Wire Wire Line
	6250 5000 6250 4850
Connection ~ 6250 4850
Wire Wire Line
	7750 6250 6450 6250
Wire Wire Line
	6450 6250 6450 6650
Wire Wire Line
	6450 6650 6150 6650
Wire Wire Line
	7750 3900 7750 4250
Connection ~ 7750 4250
Wire Wire Line
	10050 6650 10350 6650
Wire Wire Line
	10350 5350 10050 5350
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
	6150 5600 6150 5700
Connection ~ 6150 5650
Wire Wire Line
	7850 5950 7650 5950
Wire Wire Line
	7650 5950 7650 6150
Connection ~ 6150 6150
Wire Wire Line
	6850 5750 6650 5750
Wire Wire Line
	7350 5850 7850 5850
Wire Wire Line
	5550 9500 5550 9700
Wire Wire Line
	4850 9600 5550 9600
Connection ~ 5550 9600
Connection ~ 5200 9600
Wire Wire Line
	1400 9350 1650 9350
Wire Wire Line
	1650 10100 1400 10100
Wire Wire Line
	1400 10100 1400 10350
Wire Wire Line
	950  1600 2000 1600
Wire Wire Line
	2600 10100 3100 10100
Wire Notes Line
	15750 3050 750  3050
Wire Notes Line
	4100 8700 4100 11000
Wire Wire Line
	4350 1600 5350 1600
Wire Wire Line
	3450 2200 3450 2300
Connection ~ 3450 2200
Wire Wire Line
	2900 9950 2900 10100
Connection ~ 2900 10100
Wire Wire Line
	5200 9450 5200 9750
Wire Wire Line
	8700 10000 8700 10100
Connection ~ 8700 10100
Wire Wire Line
	1950 4250 1950 4950
Connection ~ 1950 4950
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
Wire Wire Line
	3650 4700 3850 4700
Wire Wire Line
	2050 4250 2050 5050
Connection ~ 2050 5050
Wire Wire Line
	2150 4250 2150 5150
Connection ~ 2150 5150
Wire Wire Line
	3650 4900 3850 4900
Wire Wire Line
	3850 4900 3850 4950
Wire Wire Line
	2450 4800 2250 4800
Wire Wire Line
	2250 4800 2250 4850
Wire Wire Line
	3800 4250 3800 4700
Connection ~ 3800 4700
Wire Wire Line
	13650 4250 13650 4400
Wire Wire Line
	13650 4900 13650 5200
Wire Wire Line
	13650 5900 13650 6150
Wire Wire Line
	13650 5050 14250 5050
Connection ~ 13650 5050
Wire Wire Line
	14000 4550 14000 5050
Connection ~ 14000 5050
$Comp
L SWITCH_VIBRATION S4
U 1 1 552FF525
P 13800 5550
F 0 "S4" H 13800 5850 60  0000 C CNN
F 1 "SWITCH_VIBRATION" H 13800 5550 60  0000 C CNN
F 2 "" H 13750 5550 60  0000 C CNN
F 3 "" H 13750 5550 60  0000 C CNN
	1    13800 5550
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR033
U 1 1 552FF729
P 10750 2400
F 0 "#PWR033" H 10750 2400 30  0001 C CNN
F 1 "GND" H 10750 2330 30  0001 C CNN
F 2 "" H 10750 2400 60  0001 C CNN
F 3 "" H 10750 2400 60  0001 C CNN
	1    10750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 2000 10750 2000
Wire Wire Line
	10750 2000 10750 2400
Wire Notes Line
	5900 900  5900 2500
Text Notes 10550 700  2    60   ~ 0
USB connector
Wire Wire Line
	950  2200 4950 2200
Wire Wire Line
	1450 1700 1450 1600
Connection ~ 1450 1600
Wire Wire Line
	1450 2100 1450 2200
Connection ~ 1450 2200
$Comp
L PWR_FLAG #FLG034
U 1 1 552FFAD1
P 1250 1500
F 0 "#FLG034" H 1250 1770 30  0001 C CNN
F 1 "PWR_FLAG" H 1250 1730 30  0000 C CNN
F 2 "" H 1250 1500 60  0001 C CNN
F 3 "" H 1250 1500 60  0001 C CNN
	1    1250 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1500 1250 1600
Connection ~ 1250 1600
$Comp
L CP C1
U 1 1 55322967
P 1450 1900
F 0 "C1" H 1500 2000 50  0000 L CNN
F 1 "330uF" H 1500 1780 50  0000 L CNN
F 2 "~" H 1450 1900 60  0000 C CNN
F 3 "~" H 1450 1900 60  0000 C CNN
	1    1450 1900
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 554FA8A3
P 11000 5750
F 0 "R12" V 11050 5550 50  0000 C CNN
F 1 "100" V 11000 5750 50  0000 C CNN
F 2 "" H 11000 5750 60  0001 C CNN
F 3 "" H 11000 5750 60  0001 C CNN
	1    11000 5750
	0    -1   -1   0   
$EndComp
$Comp
L R R11
U 1 1 554FA8F4
P 11000 5650
F 0 "R11" V 11050 5450 50  0000 C CNN
F 1 "100" V 11000 5650 50  0000 C CNN
F 2 "" H 11000 5650 60  0001 C CNN
F 3 "" H 11000 5650 60  0001 C CNN
	1    11000 5650
	0    -1   -1   0   
$EndComp
$Comp
L R R9
U 1 1 554FA8FF
P 11000 5450
F 0 "R9" V 11050 5250 50  0000 C CNN
F 1 "100" V 11000 5450 50  0000 C CNN
F 2 "" H 11000 5450 60  0001 C CNN
F 3 "" H 11000 5450 60  0001 C CNN
	1    11000 5450
	0    -1   -1   0   
$EndComp
$Comp
L R R10
U 1 1 554FA90A
P 11000 5550
F 0 "R10" V 11050 5350 50  0000 C CNN
F 1 "100" V 11000 5550 50  0000 C CNN
F 2 "" H 11000 5550 60  0001 C CNN
F 3 "" H 11000 5550 60  0001 C CNN
	1    11000 5550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11250 5750 11350 5750
Wire Wire Line
	11250 5650 11350 5650
Wire Wire Line
	10050 5450 10750 5450
Wire Wire Line
	10050 5550 10750 5550
Wire Wire Line
	11250 5450 11350 5450
Wire Wire Line
	11250 5550 11350 5550
Wire Wire Line
	10350 4550 10050 4550
Wire Wire Line
	10350 4650 10050 4650
Wire Wire Line
	10350 4750 10050 4750
Wire Wire Line
	10750 5650 10050 5650
Wire Wire Line
	10750 5750 10050 5750
Wire Wire Line
	10350 5850 10050 5850
Wire Wire Line
	10350 5950 10050 5950
Wire Wire Line
	10350 6150 10050 6150
Wire Wire Line
	10350 6850 10050 6850
Wire Wire Line
	10350 6950 10050 6950
Wire Wire Line
	10350 7050 10050 7050
Wire Wire Line
	10350 7150 10050 7150
Wire Wire Line
	10350 6450 10050 6450
NoConn ~ 10050 6250
Wire Wire Line
	10350 4950 10050 4950
$Comp
L CP C14
U 1 1 55608D98
P 4950 1950
F 0 "C14" H 5000 2050 50  0000 L CNN
F 1 "330uF" H 5000 1830 50  0000 L CNN
F 2 "~" H 4950 1950 60  0000 C CNN
F 3 "~" H 4950 1950 60  0000 C CNN
	1    4950 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1750 4950 1600
Connection ~ 4950 1600
Wire Wire Line
	4950 2200 4950 2150
Connection ~ 4550 2200
$EndSCHEMATC
