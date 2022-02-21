EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "RC Interface for MR-8"
Date "2021-09-30"
Rev "1"
Comp "@tokuhira"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L mr8-iso-mux-sbus-4ws:AdafruitQTPy U5
U 1 1 614B4C46
P 6750 3400
F 0 "U5" H 6725 4481 50  0000 C CNN
F 1 "AdafruitQTPy" H 6725 4390 50  0000 C CNN
F 2 "mr8-iso-mux-sbus-4ws:AdafruitQTPy-MOUDLE14P-2.54-21X17.8MM" H 6400 3600 50  0001 C CNN
F 3 "" H 6400 3600 50  0001 C CNN
	1    6750 3400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR09
U 1 1 614BA2E6
P 8500 2950
F 0 "#PWR09" H 8500 2800 50  0001 C CNN
F 1 "+3V3" H 8515 3123 50  0000 C CNN
F 2 "" H 8500 2950 50  0001 C CNN
F 3 "" H 8500 2950 50  0001 C CNN
	1    8500 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 614BCC2D
P 750 750
F 0 "#PWR01" H 750 500 50  0001 C CNN
F 1 "GND" H 755 577 50  0000 C CNN
F 2 "" H 750 750 50  0001 C CNN
F 3 "" H 750 750 50  0001 C CNN
	1    750  750 
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:GND_EXT #PWR02
U 1 1 614BD646
P 750 2500
F 0 "#PWR02" H 750 2250 50  0001 C CNN
F 1 "GND_EXT" H 755 2327 50  0000 C CNN
F 2 "" H 750 2500 50  0001 C CNN
F 3 "" H 750 2500 50  0001 C CNN
	1    750  2500
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:V_EXT #PWR04
U 1 1 614BD9C0
P 1250 2500
F 0 "#PWR04" H 1250 2350 50  0001 C CNN
F 1 "V_EXT" H 1265 2673 50  0000 C CNN
F 2 "" H 1250 2500 50  0001 C CNN
F 3 "" H 1250 2500 50  0001 C CNN
	1    1250 2500
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 614BE653
P 750 750
F 0 "#FLG01" H 750 825 50  0001 C CNN
F 1 "PWR_FLAG" H 750 923 50  0000 C CNN
F 2 "" H 750 750 50  0001 C CNN
F 3 "~" H 750 750 50  0001 C CNN
	1    750  750 
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 614BEA90
P 1250 750
F 0 "#FLG03" H 1250 825 50  0001 C CNN
F 1 "PWR_FLAG" H 1250 923 50  0000 C CNN
F 2 "" H 1250 750 50  0001 C CNN
F 3 "~" H 1250 750 50  0001 C CNN
	1    1250 750 
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG05
U 1 1 614BF0A5
P 1750 750
F 0 "#FLG05" H 1750 825 50  0001 C CNN
F 1 "PWR_FLAG" H 1750 923 50  0000 C CNN
F 2 "" H 1750 750 50  0001 C CNN
F 3 "~" H 1750 750 50  0001 C CNN
	1    1750 750 
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG06
U 1 1 614BF419
P 2250 750
F 0 "#FLG06" H 2250 825 50  0001 C CNN
F 1 "PWR_FLAG" H 2250 923 50  0000 C CNN
F 2 "" H 2250 750 50  0001 C CNN
F 3 "~" H 2250 750 50  0001 C CNN
	1    2250 750 
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG04
U 1 1 614BF7C8
P 1250 2500
F 0 "#FLG04" H 1250 2575 50  0001 C CNN
F 1 "PWR_FLAG" H 1250 2673 50  0000 C CNN
F 2 "" H 1250 2500 50  0001 C CNN
F 3 "~" H 1250 2500 50  0001 C CNN
	1    1250 2500
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 614BFD16
P 750 2500
F 0 "#FLG02" H 750 2575 50  0001 C CNN
F 1 "PWR_FLAG" H 750 2673 50  0000 C CNN
F 2 "" H 750 2500 50  0001 C CNN
F 3 "~" H 750 2500 50  0001 C CNN
	1    750  2500
	1    0    0    -1  
$EndComp
NoConn ~ 6650 4250
NoConn ~ 6850 4250
$Comp
L power:GND #PWR011
U 1 1 614C11F6
P 9000 3100
F 0 "#PWR011" H 9000 2850 50  0001 C CNN
F 1 "GND" H 9005 2927 50  0000 C CNN
F 2 "" H 9000 3100 50  0001 C CNN
F 3 "" H 9000 3100 50  0001 C CNN
	1    9000 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3100 7550 3100
NoConn ~ 6850 2500
NoConn ~ 6650 2500
Wire Wire Line
	7550 3250 8500 3250
Wire Wire Line
	8500 3250 8500 2950
$Comp
L power:+5V #PWR07
U 1 1 614C3ADB
P 8000 2600
F 0 "#PWR07" H 8000 2450 50  0001 C CNN
F 1 "+5V" H 8015 2773 50  0000 C CNN
F 2 "" H 8000 2600 50  0001 C CNN
F 3 "" H 8000 2600 50  0001 C CNN
	1    8000 2600
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:V_VEH(+6V) #PWR010
U 1 1 614C3BF1
P 9000 2600
F 0 "#PWR010" H 9000 2450 50  0001 C CNN
F 1 "V_VEH(+6V)" H 9015 2773 50  0000 C CNN
F 2 "" H 9000 2600 50  0001 C CNN
F 3 "" H 9000 2600 50  0001 C CNN
	1    9000 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:D D1
U 1 1 614C7AF0
P 8500 2600
F 0 "D1" H 8500 2817 50  0000 C CNN
F 1 "NSR0320(1A)" H 8500 2726 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 8500 2600 50  0001 C CNN
F 3 "~" H 8500 2600 50  0001 C CNN
	1    8500 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2600 8650 2600
Wire Wire Line
	8350 2600 8000 2600
Text Label 5000 2950 0    50   ~ 0
PWM_A+E
Text Label 5000 3100 0    50   ~ 0
PWM_B+F
Text Label 5000 3250 0    50   ~ 0
PWM_C+G
Text Label 5000 3400 0    50   ~ 0
PWM_D+H
Text Label 5000 3550 0    50   ~ 0
CON_LOST
Text Label 5000 3700 0    50   ~ 0
CON_EXT
Text Label 5000 3850 0    50   ~ 0
S.BUS_OUT
Text Label 8000 3850 0    50   ~ 0
COM_IN
Text Label 8000 3700 0    50   ~ 0
PWM_STR_INT_INV
Text Label 8000 3550 0    50   ~ 0
PWM_STR_EXT_INV
Text Label 8000 3400 0    50   ~ 0
PWM_STR_EXT
Wire Wire Line
	5000 2950 5900 2950
Wire Wire Line
	5000 3100 5900 3100
Wire Wire Line
	5000 3250 5900 3250
Wire Wire Line
	5000 3400 5900 3400
Wire Wire Line
	5000 3550 5900 3550
Wire Wire Line
	5000 3700 5900 3700
Wire Wire Line
	5000 3850 5900 3850
Wire Wire Line
	7550 3850 8000 3850
Wire Wire Line
	7550 3700 8000 3700
Wire Wire Line
	7550 3550 8000 3550
Wire Wire Line
	7550 3400 8000 3400
Text Label 4250 3700 0    50   ~ 0
S.BUS_OUT
Wire Wire Line
	2000 3900 2750 3900
Text Label 4250 3900 0    50   ~ 0
COM_IN
Wire Wire Line
	2000 3700 2750 3700
$Comp
L mr8-iso-mux-sbus-4ws:V_VEH(+6V) #PWR0101
U 1 1 614E2E62
P 2250 750
F 0 "#PWR0101" H 2250 600 50  0001 C CNN
F 1 "V_VEH(+6V)" H 2265 923 50  0000 C CNN
F 2 "" H 2250 750 50  0001 C CNN
F 3 "" H 2250 750 50  0001 C CNN
	1    2250 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0102
U 1 1 614E2FC4
P 1250 750
F 0 "#PWR0102" H 1250 600 50  0001 C CNN
F 1 "+3V3" H 1265 923 50  0000 C CNN
F 2 "" H 1250 750 50  0001 C CNN
F 3 "" H 1250 750 50  0001 C CNN
	1    1250 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 614E4CC2
P 1750 750
F 0 "#PWR0103" H 1750 600 50  0001 C CNN
F 1 "+5V" H 1765 923 50  0000 C CNN
F 2 "" H 1750 750 50  0001 C CNN
F 3 "" H 1750 750 50  0001 C CNN
	1    1750 750 
	1    0    0    -1  
$EndComp
Text Label 5800 1350 0    50   ~ 0
PWM_A+E
Text Label 5800 1650 0    50   ~ 0
PWM_B+F
Text Label 8050 1750 0    50   ~ 0
PWM_C+G
Text Label 8050 1450 0    50   ~ 0
PWM_D+H
Wire Wire Line
	5800 1350 6600 1350
Wire Wire Line
	5800 1650 6600 1650
Wire Wire Line
	8050 1750 7500 1750
Wire Wire Line
	8050 1450 7500 1450
Wire Wire Line
	7800 1150 7500 1150
Wire Wire Line
	6300 1750 6600 1750
Text Label 5800 1150 0    50   ~ 0
PWM_A
Wire Wire Line
	5800 1150 6600 1150
Text Label 5800 1250 0    50   ~ 0
PWM_E
Wire Wire Line
	5800 1250 6600 1250
Text Label 5800 1550 0    50   ~ 0
PWM_F
Wire Wire Line
	5800 1550 6600 1550
Text Label 5800 1450 0    50   ~ 0
PWM_B
Wire Wire Line
	5800 1450 6600 1450
Text Label 8050 1350 0    50   ~ 0
PWM_D
Wire Wire Line
	8050 1350 7500 1350
Text Label 8050 1250 0    50   ~ 0
PWM_H
Wire Wire Line
	8050 1250 7500 1250
Text Label 8050 1650 0    50   ~ 0
PWM_C
Wire Wire Line
	8050 1650 7500 1650
Text Label 8050 1550 0    50   ~ 0
PWM_G
Wire Wire Line
	8050 1550 7500 1550
$Comp
L mr8-iso-mux-sbus-4ws:74HC4053 U2
U 1 1 6151BA1C
P 7050 5600
F 0 "U2" H 7100 6503 60  0000 C CNN
F 1 "74HC4053" H 7100 6397 60  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 7100 6291 60  0000 C CNN
F 3 "" H 7050 5600 60  0000 C CNN
	1    7050 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 6151CD8A
P 6350 6000
F 0 "#PWR014" H 6350 5750 50  0001 C CNN
F 1 "GND" H 6355 5827 50  0000 C CNN
F 2 "" H 6350 6000 50  0001 C CNN
F 3 "" H 6350 6000 50  0001 C CNN
	1    6350 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 5500 6350 5500
Wire Wire Line
	6350 5500 6350 5600
$Comp
L power:+3V3 #PWR013
U 1 1 615206A8
P 6350 4950
F 0 "#PWR013" H 6350 4800 50  0001 C CNN
F 1 "+3V3" H 6365 5123 50  0000 C CNN
F 2 "" H 6350 4950 50  0001 C CNN
F 3 "" H 6350 4950 50  0001 C CNN
	1    6350 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 5400 6350 5400
Wire Wire Line
	6350 5400 6350 4950
Text Label 5100 5050 0    50   ~ 0
CON_EXT
Wire Wire Line
	5100 5050 5950 5050
Wire Wire Line
	6500 5250 6450 5250
Wire Wire Line
	6450 5250 6450 5150
Connection ~ 6450 5050
Wire Wire Line
	6450 5050 6500 5050
Wire Wire Line
	6500 5150 6450 5150
Connection ~ 6450 5150
Wire Wire Line
	6450 5150 6450 5050
$Comp
L Device:R R2
U 1 1 6152C4B9
P 5950 5450
F 0 "R2" H 6020 5496 50  0000 L CNN
F 1 "10K" H 6020 5405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5880 5450 50  0001 C CNN
F 3 "~" H 5950 5450 50  0001 C CNN
	1    5950 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6152CFBB
P 5500 5850
F 0 "R1" H 5570 5896 50  0000 L CNN
F 1 "10K" H 5570 5805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5430 5850 50  0001 C CNN
F 3 "~" H 5500 5850 50  0001 C CNN
	1    5500 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 6152D9B1
P 5950 6000
F 0 "#PWR012" H 5950 5750 50  0001 C CNN
F 1 "GND" H 5955 5827 50  0000 C CNN
F 2 "" H 5950 6000 50  0001 C CNN
F 3 "" H 5950 6000 50  0001 C CNN
	1    5950 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 5300 5950 5050
Wire Wire Line
	5950 5600 5950 6000
Wire Wire Line
	6500 5600 6350 5600
Connection ~ 6350 5600
Wire Wire Line
	6350 5600 6350 6000
Text Label 8350 5650 0    50   ~ 0
PWM_STR_INT_INV
Text Label 8350 5750 0    50   ~ 0
PWM_STR_EXT_INV
Text Label 8350 5450 0    50   ~ 0
PWM_STR
Wire Wire Line
	7700 5650 8350 5650
Wire Wire Line
	7700 5750 8350 5750
Wire Wire Line
	7700 5450 8350 5450
Text Label 8350 5350 0    50   ~ 0
PWM_D
Wire Wire Line
	8350 5350 7700 5350
Text Label 8350 5050 0    50   ~ 0
PWM_A
Text Label 8350 5150 0    50   ~ 0
PWM_THR
Text Label 8350 5250 0    50   ~ 0
PWM_THR_VEH
Text Label 8350 5550 0    50   ~ 0
PWM_STR_VEH
Text Label 8350 5850 0    50   ~ 0
PWM_STR_VEH_INV
Wire Wire Line
	8350 5050 7700 5050
Wire Wire Line
	8350 5150 7700 5150
Wire Wire Line
	8350 5250 7700 5250
Wire Wire Line
	8350 5550 7700 5550
Wire Wire Line
	8350 5850 7700 5850
Text Label 4250 5350 0    50   ~ 0
PWM_THR
Wire Wire Line
	7550 2950 8000 2950
Wire Wire Line
	8000 2950 8000 2600
Connection ~ 8000 2600
$Comp
L power:+3V3 #PWR08
U 1 1 615800AC
P 3000 1000
F 0 "#PWR08" H 3000 850 50  0001 C CNN
F 1 "+3V3" H 3015 1173 50  0000 C CNN
F 2 "" H 3000 1000 50  0001 C CNN
F 3 "" H 3000 1000 50  0001 C CNN
	1    3000 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 61593942
P 3000 1500
F 0 "#PWR015" H 3000 1250 50  0001 C CNN
F 1 "GND" H 3005 1327 50  0000 C CNN
F 2 "" H 3000 1500 50  0001 C CNN
F 3 "" H 3000 1500 50  0001 C CNN
	1    3000 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 61594169
P 3000 1250
F 0 "C1" H 3115 1296 50  0000 L CNN
F 1 "0.1u" H 3115 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3038 1100 50  0001 C CNN
F 3 "~" H 3000 1250 50  0001 C CNN
	1    3000 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 615948AC
P 3500 1250
F 0 "C2" H 3615 1296 50  0000 L CNN
F 1 "0.1u" H 3615 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3538 1100 50  0001 C CNN
F 3 "~" H 3500 1250 50  0001 C CNN
	1    3500 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 61594C5E
P 4000 1250
F 0 "C3" H 4115 1296 50  0000 L CNN
F 1 "0.1u" H 4115 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4038 1100 50  0001 C CNN
F 3 "~" H 4000 1250 50  0001 C CNN
	1    4000 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 61594F70
P 4500 1250
F 0 "C4" H 4615 1296 50  0000 L CNN
F 1 "0.1u" H 4615 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4538 1100 50  0001 C CNN
F 3 "~" H 4500 1250 50  0001 C CNN
	1    4500 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1000 3000 1100
Wire Wire Line
	3000 1400 3000 1500
Wire Wire Line
	3000 1000 3500 1000
Connection ~ 3000 1000
Wire Wire Line
	3000 1500 3500 1500
Connection ~ 3000 1500
Wire Wire Line
	3500 1100 3500 1000
Connection ~ 3500 1000
Wire Wire Line
	3500 1000 4000 1000
Wire Wire Line
	4000 1100 4000 1000
Connection ~ 4000 1000
Wire Wire Line
	4000 1000 4500 1000
Wire Wire Line
	4500 1100 4500 1000
Wire Wire Line
	3500 1400 3500 1500
Connection ~ 3500 1500
Wire Wire Line
	3500 1500 4000 1500
Wire Wire Line
	4000 1400 4000 1500
Connection ~ 4000 1500
Wire Wire Line
	4000 1500 4500 1500
Wire Wire Line
	4500 1400 4500 1500
$Comp
L mr8-iso-mux-sbus-4ws:74HC32 U1
U 1 1 6150391B
P 7050 1500
F 0 "U1" H 7050 2137 60  0000 C CNN
F 1 "74HC32" H 7050 2031 60  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 7050 1500 60  0001 C CNN
F 3 "" H 7050 1500 60  0001 C CNN
	1    7050 1500
	1    0    0    -1  
$EndComp
Text Label 8850 1400 0    50   ~ 0
PWM_F
Text Label 8850 1600 0    50   ~ 0
PWM_B
Text Label 8850 1500 0    50   ~ 0
PWM_D
Text Label 8850 1300 0    50   ~ 0
PWM_H
Text Label 10600 1500 0    50   ~ 0
PWM_C
Text Label 10600 1300 0    50   ~ 0
PWM_G
$Comp
L mr8-iso-mux-sbus-4ws:V_VEH(+6V) #PWR016
U 1 1 615B67A5
P 10350 1100
F 0 "#PWR016" H 10350 950 50  0001 C CNN
F 1 "V_VEH(+6V)" H 10365 1273 50  0000 C CNN
F 2 "" H 10350 1100 50  0001 C CNN
F 3 "" H 10350 1100 50  0001 C CNN
	1    10350 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 615B6BB5
P 9350 1700
F 0 "#PWR017" H 9350 1450 50  0001 C CNN
F 1 "GND" H 9355 1527 50  0000 C CNN
F 2 "" H 9350 1700 50  0001 C CNN
F 3 "" H 9350 1700 50  0001 C CNN
	1    9350 1700
	1    0    0    -1  
$EndComp
Text Label 10600 1400 0    50   ~ 0
PWM_E
Text Label 10600 1600 0    50   ~ 0
PWM_A
Wire Wire Line
	10100 1300 10600 1300
Wire Wire Line
	10100 1400 10600 1400
Wire Wire Line
	10100 1500 10600 1500
Wire Wire Line
	10100 1600 10600 1600
Wire Wire Line
	8850 1400 9600 1400
Wire Wire Line
	9600 1500 8850 1500
Wire Wire Line
	9600 1600 8850 1600
Wire Wire Line
	9600 1300 8850 1300
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J1
U 1 1 615AD5F5
P 9800 1400
F 0 "J1" H 9850 1817 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 9850 1726 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 9800 1400 50  0001 C CNN
F 3 "~" H 9800 1400 50  0001 C CNN
	1    9800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1200 10350 1200
Wire Wire Line
	10350 1200 10350 1100
Wire Wire Line
	9600 1200 9350 1200
Wire Wire Line
	9350 1200 9350 1700
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 614EAC01
P 10400 3850
F 0 "J2" H 10480 3892 50  0000 L CNN
F 1 "Conn_01x03" H 10480 3801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10400 3850 50  0001 C CNN
F 3 "~" H 10400 3850 50  0001 C CNN
	1    10400 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 614ED3F7
P 9900 4050
F 0 "#PWR019" H 9900 3800 50  0001 C CNN
F 1 "GND" H 9905 3877 50  0000 C CNN
F 2 "" H 9900 4050 50  0001 C CNN
F 3 "" H 9900 4050 50  0001 C CNN
	1    9900 4050
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:V_VEH(+6V) #PWR018
U 1 1 614EDFAD
P 9900 3650
F 0 "#PWR018" H 9900 3500 50  0001 C CNN
F 1 "V_VEH(+6V)" H 9915 3823 50  0000 C CNN
F 2 "" H 9900 3650 50  0001 C CNN
F 3 "" H 9900 3650 50  0001 C CNN
	1    9900 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 3950 9900 3950
Wire Wire Line
	9900 3950 9900 4050
Wire Wire Line
	10200 3850 9900 3850
Wire Wire Line
	9900 3850 9900 3650
Text Label 9400 3750 0    50   ~ 0
PWM_THR_VEH
Text Label 9400 4750 0    50   ~ 0
PWM_STR_VEH
Text Label 9400 5750 0    50   ~ 0
PWM_STR_VEH_INV
Wire Wire Line
	10200 3750 9400 3750
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 614FE98A
P 10400 4850
F 0 "J3" H 10480 4892 50  0000 L CNN
F 1 "Conn_01x03" H 10480 4801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10400 4850 50  0001 C CNN
F 3 "~" H 10400 4850 50  0001 C CNN
	1    10400 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 614FEC96
P 9900 5050
F 0 "#PWR021" H 9900 4800 50  0001 C CNN
F 1 "GND" H 9905 4877 50  0000 C CNN
F 2 "" H 9900 5050 50  0001 C CNN
F 3 "" H 9900 5050 50  0001 C CNN
	1    9900 5050
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:V_VEH(+6V) #PWR020
U 1 1 614FECA0
P 9900 4650
F 0 "#PWR020" H 9900 4500 50  0001 C CNN
F 1 "V_VEH(+6V)" H 9915 4823 50  0000 C CNN
F 2 "" H 9900 4650 50  0001 C CNN
F 3 "" H 9900 4650 50  0001 C CNN
	1    9900 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 4950 9900 4950
Wire Wire Line
	9900 4950 9900 5050
Wire Wire Line
	10200 4850 9900 4850
Wire Wire Line
	9900 4850 9900 4650
Wire Wire Line
	10200 4750 9400 4750
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 61502FC9
P 10400 5850
F 0 "J4" H 10480 5892 50  0000 L CNN
F 1 "Conn_01x03" H 10480 5801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 10400 5850 50  0001 C CNN
F 3 "~" H 10400 5850 50  0001 C CNN
	1    10400 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 61503309
P 9900 6050
F 0 "#PWR023" H 9900 5800 50  0001 C CNN
F 1 "GND" H 9905 5877 50  0000 C CNN
F 2 "" H 9900 6050 50  0001 C CNN
F 3 "" H 9900 6050 50  0001 C CNN
	1    9900 6050
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:V_VEH(+6V) #PWR022
U 1 1 61503313
P 9900 5650
F 0 "#PWR022" H 9900 5500 50  0001 C CNN
F 1 "V_VEH(+6V)" H 9915 5823 50  0000 C CNN
F 2 "" H 9900 5650 50  0001 C CNN
F 3 "" H 9900 5650 50  0001 C CNN
	1    9900 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 5950 9900 5950
Wire Wire Line
	9900 5950 9900 6050
Wire Wire Line
	10200 5850 9900 5850
Wire Wire Line
	9900 5850 9900 5650
Wire Wire Line
	10200 5750 9400 5750
$Comp
L Isolator:ADuM120N U4
U 1 1 61522EA0
P 3250 5450
F 0 "U4" H 3250 5917 50  0000 C CNN
F 1 "ADuM120N" H 3250 5826 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3250 5050 50  0001 C CIN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADuM120N_121N.pdf" H 2800 5850 50  0001 C CNN
	1    3250 5450
	1    0    0    -1  
$EndComp
$Comp
L Isolator:ADuM121N U3
U 1 1 61523B82
P 3250 3800
F 0 "U3" H 3794 3846 50  0000 L CNN
F 1 "ADuM121N" H 3794 3755 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3250 3100 50  0001 C CIN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADuM120N_121N.pdf" H 2800 4200 50  0001 C CNN
	1    3250 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR024
U 1 1 6152B6E1
P 3950 3400
F 0 "#PWR024" H 3950 3250 50  0001 C CNN
F 1 "+3V3" H 3965 3573 50  0000 C CNN
F 2 "" H 3950 3400 50  0001 C CNN
F 3 "" H 3950 3400 50  0001 C CNN
	1    3950 3400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR026
U 1 1 6152BEA3
P 4000 5050
F 0 "#PWR026" H 4000 4900 50  0001 C CNN
F 1 "+3V3" H 4015 5223 50  0000 C CNN
F 2 "" H 4000 5050 50  0001 C CNN
F 3 "" H 4000 5050 50  0001 C CNN
	1    4000 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3400 2500 3400
Wire Wire Line
	2500 5050 2500 5250
Wire Wire Line
	2500 5250 2750 5250
$Comp
L power:GND #PWR025
U 1 1 6153527B
P 3950 4200
F 0 "#PWR025" H 3950 3950 50  0001 C CNN
F 1 "GND" H 3955 4027 50  0000 C CNN
F 2 "" H 3950 4200 50  0001 C CNN
F 3 "" H 3950 4200 50  0001 C CNN
	1    3950 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 615357F4
P 4000 5850
F 0 "#PWR027" H 4000 5600 50  0001 C CNN
F 1 "GND" H 4005 5677 50  0000 C CNN
F 2 "" H 4000 5850 50  0001 C CNN
F 3 "" H 4000 5850 50  0001 C CNN
	1    4000 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 4200 2500 4200
Wire Wire Line
	2750 5650 2500 5650
Wire Wire Line
	2500 5650 2500 5850
$Comp
L mr8-iso-mux-sbus-4ws:V_EXT #PWR028
U 1 1 6153E480
P 2500 3400
F 0 "#PWR028" H 2500 3250 50  0001 C CNN
F 1 "V_EXT" H 2515 3573 50  0000 C CNN
F 2 "" H 2500 3400 50  0001 C CNN
F 3 "" H 2500 3400 50  0001 C CNN
	1    2500 3400
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:V_EXT #PWR030
U 1 1 6154788B
P 2500 5050
F 0 "#PWR030" H 2500 4900 50  0001 C CNN
F 1 "V_EXT" H 2515 5223 50  0000 C CNN
F 2 "" H 2500 5050 50  0001 C CNN
F 3 "" H 2500 5050 50  0001 C CNN
	1    2500 5050
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:GND_EXT #PWR029
U 1 1 61547F30
P 2500 4200
F 0 "#PWR029" H 2500 3950 50  0001 C CNN
F 1 "GND_EXT" H 2505 4027 50  0000 C CNN
F 2 "" H 2500 4200 50  0001 C CNN
F 3 "" H 2500 4200 50  0001 C CNN
	1    2500 4200
	1    0    0    -1  
$EndComp
$Comp
L mr8-iso-mux-sbus-4ws:GND_EXT #PWR031
U 1 1 61548458
P 2500 5850
F 0 "#PWR031" H 2500 5600 50  0001 C CNN
F 1 "GND_EXT" H 2505 5677 50  0000 C CNN
F 2 "" H 2500 5850 50  0001 C CNN
F 3 "" H 2500 5850 50  0001 C CNN
	1    2500 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5650 4000 5650
Wire Wire Line
	4000 5650 4000 5850
Wire Wire Line
	3750 5250 4000 5250
Wire Wire Line
	4000 5250 4000 5050
Wire Wire Line
	3350 3400 3950 3400
Wire Wire Line
	3350 4200 3950 4200
Wire Wire Line
	4250 3700 3750 3700
Wire Wire Line
	4250 3900 3750 3900
Text Label 4250 5550 0    50   ~ 0
PWM_STR
Wire Wire Line
	4250 5350 3750 5350
Wire Wire Line
	4250 5550 3750 5550
Text Label 2000 5350 0    50   ~ 0
PWM_THR_EXT
Text Label 2000 5550 0    50   ~ 0
PWM_STR_EXT
Wire Wire Line
	2750 5350 2000 5350
Wire Wire Line
	2750 5550 2000 5550
Text Label 2000 3700 0    50   ~ 0
S.BUS_OUT_EXT
Text Label 2000 3900 0    50   ~ 0
COM_IN_EXT
Wire Wire Line
	1600 7000 2400 7000
Wire Wire Line
	1600 6900 2400 6900
Text Label 1600 7100 0    50   ~ 0
PWM_THR_EXT
Wire Wire Line
	2400 7100 1600 7100
Text Label 1600 6900 0    50   ~ 0
S.BUS_OUT_EXT
Text Label 1600 7000 0    50   ~ 0
COM_IN_EXT
$Comp
L mr8-iso-mux-sbus-4ws:GND_EXT #PWR033
U 1 1 615B3388
P 2300 7350
F 0 "#PWR033" H 2300 7100 50  0001 C CNN
F 1 "GND_EXT" H 2305 7177 50  0000 C CNN
F 2 "" H 2300 7350 50  0001 C CNN
F 3 "" H 2300 7350 50  0001 C CNN
	1    2300 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6700 2300 6700
Wire Wire Line
	2300 6700 2300 7350
$Comp
L mr8-iso-mux-sbus-4ws:V_EXT #PWR032
U 1 1 615CC141
P 2200 6550
F 0 "#PWR032" H 2200 6400 50  0001 C CNN
F 1 "V_EXT" H 2215 6723 50  0000 C CNN
F 2 "" H 2200 6550 50  0001 C CNN
F 3 "" H 2200 6550 50  0001 C CNN
	1    2200 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6800 2200 6800
Wire Wire Line
	2200 6800 2200 6550
$Comp
L power:+3V3 #PWR05
U 1 1 6150CA54
P 7800 1150
F 0 "#PWR05" H 7800 1000 50  0001 C CNN
F 1 "+3V3" H 7815 1323 50  0000 C CNN
F 2 "" H 7800 1150 50  0001 C CNN
F 3 "" H 7800 1150 50  0001 C CNN
	1    7800 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 6150BE0E
P 6300 1750
F 0 "#PWR03" H 6300 1500 50  0001 C CNN
F 1 "GND" H 6305 1577 50  0000 C CNN
F 2 "" H 6300 1750 50  0001 C CNN
F 3 "" H 6300 1750 50  0001 C CNN
	1    6300 1750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J6
U 1 1 615FA9DB
P 2600 6900
F 0 "J6" H 2680 6892 50  0000 L CNN
F 1 "Conn_01x06" H 2680 6801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2600 6900 50  0001 C CNN
F 3 "~" H 2600 6900 50  0001 C CNN
	1    2600 6900
	1    0    0    -1  
$EndComp
Text Label 1600 7200 0    50   ~ 0
PWM_STR_EXT
Wire Wire Line
	2400 7200 1600 7200
$Comp
L Connector_Generic:Conn_01x04 J5
U 1 1 61667BC5
P 6100 6900
F 0 "J5" H 6180 6892 50  0000 L CNN
F 1 "Conn_01x04" H 6180 6801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6100 6900 50  0001 C CNN
F 3 "~" H 6100 6900 50  0001 C CNN
	1    6100 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 6800 5800 6800
Wire Wire Line
	5800 6800 5800 7250
Wire Wire Line
	5900 6900 5700 6900
Wire Wire Line
	5700 6900 5700 6650
Text Label 5100 7000 0    50   ~ 0
CON_LOST
Text Label 5100 7100 0    50   ~ 0
CON_EXT
Wire Wire Line
	5100 7000 5900 7000
Wire Wire Line
	5100 7100 5900 7100
$Comp
L power:GND #PWR0104
U 1 1 61692FDA
P 5800 7250
F 0 "#PWR0104" H 5800 7000 50  0001 C CNN
F 1 "GND" H 5805 7077 50  0000 C CNN
F 2 "" H 5800 7250 50  0001 C CNN
F 3 "" H 5800 7250 50  0001 C CNN
	1    5800 7250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0105
U 1 1 6169349B
P 5700 6650
F 0 "#PWR0105" H 5700 6500 50  0001 C CNN
F 1 "+3V3" H 5715 6823 50  0000 C CNN
F 2 "" H 5700 6650 50  0001 C CNN
F 3 "" H 5700 6650 50  0001 C CNN
	1    5700 6650
	1    0    0    -1  
$EndComp
Wire Notes Line
	550  2000 3250 2000
Wire Notes Line
	3250 2000 3250 3300
Wire Notes Line
	3250 4300 3250 4900
Wire Notes Line
	3250 5950 3250 7750
Text Notes 1500 2200 0    100  ~ 0
External Interface
Text Notes 1500 1900 0    100  ~ 0
Vehicle Control
$Comp
L Device:C C5
U 1 1 617B7B82
P 750 4750
F 0 "C5" H 865 4796 50  0000 L CNN
F 1 "0.1u" H 865 4705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 788 4600 50  0001 C CNN
F 3 "~" H 750 4750 50  0001 C CNN
	1    750  4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 617B7B8C
P 1250 4750
F 0 "C6" H 1365 4796 50  0000 L CNN
F 1 "0.1u" H 1365 4705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 1288 4600 50  0001 C CNN
F 3 "~" H 1250 4750 50  0001 C CNN
	1    1250 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  4500 750  4600
Wire Wire Line
	750  4900 750  5000
Wire Wire Line
	750  4500 1250 4500
Wire Wire Line
	750  5000 1250 5000
Wire Wire Line
	1250 4600 1250 4500
Wire Wire Line
	1250 4900 1250 5000
$Comp
L mr8-iso-mux-sbus-4ws:V_EXT #PWR034
U 1 1 617CF676
P 750 4500
F 0 "#PWR034" H 750 4350 50  0001 C CNN
F 1 "V_EXT" H 765 4673 50  0000 C CNN
F 2 "" H 750 4500 50  0001 C CNN
F 3 "" H 750 4500 50  0001 C CNN
	1    750  4500
	1    0    0    -1  
$EndComp
Connection ~ 750  4500
$Comp
L mr8-iso-mux-sbus-4ws:GND_EXT #PWR035
U 1 1 617CF99C
P 750 5000
F 0 "#PWR035" H 750 4750 50  0001 C CNN
F 1 "GND_EXT" H 755 4827 50  0000 C CNN
F 2 "" H 750 5000 50  0001 C CNN
F 3 "" H 750 5000 50  0001 C CNN
	1    750  5000
	1    0    0    -1  
$EndComp
Connection ~ 750  5000
$Comp
L Mechanical:MountingHole H1
U 1 1 6182D66A
P 750 1250
F 0 "H1" H 850 1296 50  0000 L CNN
F 1 "MountingHole" H 850 1205 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 750 1250 50  0001 C CNN
F 3 "~" H 750 1250 50  0001 C CNN
	1    750  1250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 6182DB76
P 750 1500
F 0 "H2" H 850 1546 50  0000 L CNN
F 1 "MountingHole" H 850 1455 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 750 1500 50  0001 C CNN
F 3 "~" H 750 1500 50  0001 C CNN
	1    750  1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 6157E106
P 5100 6000
F 0 "#PWR0106" H 5100 5750 50  0001 C CNN
F 1 "GND" H 5105 5827 50  0000 C CNN
F 2 "" H 5100 6000 50  0001 C CNN
F 3 "" H 5100 6000 50  0001 C CNN
	1    5100 6000
	1    0    0    -1  
$EndComp
Connection ~ 5950 5050
Wire Wire Line
	5950 5050 6450 5050
Wire Wire Line
	6500 5850 5650 5850
Wire Wire Line
	5350 5850 5100 5850
Wire Wire Line
	5100 5850 5100 6000
$EndSCHEMATC
