EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "MR-8 Gender Changer"
Date "2021-09-28"
Rev "0"
Comp "@tokuhira"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Notes Line
	3250 2000 3250 7750
Text Notes 1500 1900 0    100  ~ 0
Vehicle Control
Text Notes 1500 2200 0    100  ~ 0
External Interface
Wire Notes Line
	550  2000 3250 2000
Wire Wire Line
	9350 1200 9350 1700
Wire Wire Line
	9600 1200 9350 1200
Wire Wire Line
	10350 1200 10350 1100
Wire Wire Line
	10100 1200 10350 1200
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J1
U 1 1 615AD5F5
P 9800 1400
F 0 "J1" H 9850 1817 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 9850 1726 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x10_P2.54mm_Vertical" H 9800 1400 50  0001 C CNN
F 3 "~" H 9800 1400 50  0001 C CNN
	1    9800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 1300 8850 1300
Wire Wire Line
	9600 1600 8850 1600
Wire Wire Line
	9600 1500 8850 1500
Wire Wire Line
	8850 1400 9600 1400
Wire Wire Line
	10100 1600 10600 1600
Wire Wire Line
	10100 1500 10600 1500
Wire Wire Line
	10100 1400 10600 1400
Wire Wire Line
	10100 1300 10600 1300
Text Label 10600 1600 0    50   ~ 0
PWM_A
Text Label 10600 1400 0    50   ~ 0
PWM_E
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
$Comp
L mr8-gender-changer:V_VEH(+6V) #PWR016
U 1 1 615B67A5
P 10350 1100
F 0 "#PWR016" H 10350 950 50  0001 C CNN
F 1 "V_VEH(+6V)" H 10365 1273 50  0000 C CNN
F 2 "" H 10350 1100 50  0001 C CNN
F 3 "" H 10350 1100 50  0001 C CNN
	1    10350 1100
	1    0    0    -1  
$EndComp
Text Label 10600 1300 0    50   ~ 0
PWM_G
Text Label 10600 1500 0    50   ~ 0
PWM_C
Text Label 8850 1300 0    50   ~ 0
PWM_H
Text Label 8850 1500 0    50   ~ 0
PWM_D
Text Label 8850 1600 0    50   ~ 0
PWM_B
Text Label 8850 1400 0    50   ~ 0
PWM_F
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
L mr8-gender-changer:V_VEH(+6V) #PWR0101
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
L mr8-gender-changer:V_EXT #PWR04
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
L mr8-gender-changer:GND_EXT #PWR02
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
L Connector_Generic:Conn_01x02 J2
U 1 1 61533248
P 7300 1200
F 0 "J2" H 7380 1192 50  0000 L CNN
F 1 "Conn_01x02" H 7380 1101 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 7300 1200 50  0001 C CNN
F 3 "~" H 7300 1200 50  0001 C CNN
	1    7300 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x08 J3
U 1 1 61534ECF
P 7300 2550
F 0 "J3" H 7380 2542 50  0000 L CNN
F 1 "Conn_01x08" H 7380 2451 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 7300 2550 50  0001 C CNN
F 3 "~" H 7300 2550 50  0001 C CNN
	1    7300 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 61538A62
P 6800 1700
F 0 "#PWR03" H 6800 1450 50  0001 C CNN
F 1 "GND" H 6805 1527 50  0000 C CNN
F 2 "" H 6800 1700 50  0001 C CNN
F 3 "" H 6800 1700 50  0001 C CNN
	1    6800 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1200 6800 1200
Wire Wire Line
	6800 1200 6800 1700
$Comp
L mr8-gender-changer:V_VEH(+6V) #PWR05
U 1 1 6153A5F8
P 7000 1100
F 0 "#PWR05" H 7000 950 50  0001 C CNN
F 1 "V_VEH(+6V)" H 7015 1273 50  0000 C CNN
F 2 "" H 7000 1100 50  0001 C CNN
F 3 "" H 7000 1100 50  0001 C CNN
	1    7000 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1300 7000 1300
Wire Wire Line
	7000 1300 7000 1100
Text Label 4150 2250 0    50   ~ 0
PWM_A
Text Label 4150 2650 0    50   ~ 0
PWM_E
Text Label 4150 2850 0    50   ~ 0
PWM_G
Text Label 4150 2450 0    50   ~ 0
PWM_C
Wire Wire Line
	7100 2950 7000 2950
Wire Wire Line
	7100 2350 4500 2350
Wire Wire Line
	7100 2550 4150 2550
Wire Wire Line
	4150 2750 6000 2750
Text Label 4150 2950 0    50   ~ 0
PWM_H
Text Label 4150 2550 0    50   ~ 0
PWM_D
Text Label 4150 2350 0    50   ~ 0
PWM_B
Text Label 4150 2750 0    50   ~ 0
PWM_F
Wire Wire Line
	7100 2250 4150 2250
Wire Wire Line
	7100 2450 5000 2450
Wire Wire Line
	7100 2850 6500 2850
$Comp
L Device:R R1
U 1 1 615C34DF
P 4500 3250
F 0 "R1" H 4570 3296 50  0000 L CNN
F 1 "10k DNF" H 4570 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4430 3250 50  0001 C CNN
F 3 "~" H 4500 3250 50  0001 C CNN
	1    4500 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 615C3B0A
P 5000 3250
F 0 "R2" H 5070 3296 50  0000 L CNN
F 1 "10k DNF" H 5070 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4930 3250 50  0001 C CNN
F 3 "~" H 5000 3250 50  0001 C CNN
	1    5000 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 615C3E11
P 5500 3250
F 0 "R3" H 5570 3296 50  0000 L CNN
F 1 "10k DNF" H 5570 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5430 3250 50  0001 C CNN
F 3 "~" H 5500 3250 50  0001 C CNN
	1    5500 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 615C3FA9
P 6000 3250
F 0 "R4" H 6070 3296 50  0000 L CNN
F 1 "10k DNF" H 6070 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5930 3250 50  0001 C CNN
F 3 "~" H 6000 3250 50  0001 C CNN
	1    6000 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 615C47F9
P 6500 3250
F 0 "R5" H 6570 3296 50  0000 L CNN
F 1 "10k DNF" H 6570 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6430 3250 50  0001 C CNN
F 3 "~" H 6500 3250 50  0001 C CNN
	1    6500 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 615C4A9F
P 7000 3250
F 0 "R6" H 7070 3296 50  0000 L CNN
F 1 "10k DNF" H 7070 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6930 3250 50  0001 C CNN
F 3 "~" H 7000 3250 50  0001 C CNN
	1    7000 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 615C4E3B
P 4500 3500
F 0 "#PWR06" H 4500 3250 50  0001 C CNN
F 1 "GND" H 4505 3327 50  0000 C CNN
F 2 "" H 4500 3500 50  0001 C CNN
F 3 "" H 4500 3500 50  0001 C CNN
	1    4500 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 615C5367
P 5000 3500
F 0 "#PWR07" H 5000 3250 50  0001 C CNN
F 1 "GND" H 5005 3327 50  0000 C CNN
F 2 "" H 5000 3500 50  0001 C CNN
F 3 "" H 5000 3500 50  0001 C CNN
	1    5000 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 615C5686
P 5500 3500
F 0 "#PWR08" H 5500 3250 50  0001 C CNN
F 1 "GND" H 5505 3327 50  0000 C CNN
F 2 "" H 5500 3500 50  0001 C CNN
F 3 "" H 5500 3500 50  0001 C CNN
	1    5500 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 615C5B15
P 6000 3500
F 0 "#PWR09" H 6000 3250 50  0001 C CNN
F 1 "GND" H 6005 3327 50  0000 C CNN
F 2 "" H 6000 3500 50  0001 C CNN
F 3 "" H 6000 3500 50  0001 C CNN
	1    6000 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 615C5EC2
P 6500 3500
F 0 "#PWR010" H 6500 3250 50  0001 C CNN
F 1 "GND" H 6505 3327 50  0000 C CNN
F 2 "" H 6500 3500 50  0001 C CNN
F 3 "" H 6500 3500 50  0001 C CNN
	1    6500 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 615C6187
P 7000 3500
F 0 "#PWR011" H 7000 3250 50  0001 C CNN
F 1 "GND" H 7005 3327 50  0000 C CNN
F 2 "" H 7000 3500 50  0001 C CNN
F 3 "" H 7000 3500 50  0001 C CNN
	1    7000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3400 4500 3500
Wire Wire Line
	5000 3400 5000 3500
Wire Wire Line
	5500 3400 5500 3500
Wire Wire Line
	6000 3400 6000 3500
Wire Wire Line
	6500 3400 6500 3500
Wire Wire Line
	7000 3400 7000 3500
Wire Wire Line
	4500 3100 4500 2350
Connection ~ 4500 2350
Wire Wire Line
	4500 2350 4150 2350
Wire Wire Line
	5000 3100 5000 2450
Connection ~ 5000 2450
Wire Wire Line
	5000 2450 4150 2450
Wire Wire Line
	5500 3100 5500 2650
Connection ~ 5500 2650
Wire Wire Line
	5500 2650 4150 2650
Wire Wire Line
	7100 2650 5500 2650
Wire Wire Line
	6000 3100 6000 2750
Connection ~ 6000 2750
Wire Wire Line
	6000 2750 7100 2750
Wire Wire Line
	6500 3100 6500 2850
Connection ~ 6500 2850
Wire Wire Line
	6500 2850 4150 2850
Wire Wire Line
	7000 3100 7000 2950
Connection ~ 7000 2950
Wire Wire Line
	7000 2950 4150 2950
$EndSCHEMATC