EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "otto_shield"
Date "2020-03-06"
Rev "A"
Comp "iralab"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R1
U 1 1 5E67BB10
P 2030 2490
F 0 "R1" V 1823 2490 50  0000 C CNN
F 1 "2.7K" V 1914 2490 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 1960 2490 50  0001 C CNN
F 3 "~" H 2030 2490 50  0001 C CNN
	1    2030 2490
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E67CB6D
P 2030 2790
F 0 "R2" V 1823 2790 50  0000 C CNN
F 1 "2.7K" V 1914 2790 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 1960 2790 50  0001 C CNN
F 3 "~" H 2030 2790 50  0001 C CNN
	1    2030 2790
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E67DA7F
P 8110 3270
F 0 "R3" V 7903 3270 50  0000 C CNN
F 1 "2.7K" V 7994 3270 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 8040 3270 50  0001 C CNN
F 3 "~" H 8110 3270 50  0001 C CNN
	1    8110 3270
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5E67E80F
P 7940 2170
F 0 "R4" V 7733 2170 50  0000 C CNN
F 1 "2.7K" V 7824 2170 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 7870 2170 50  0001 C CNN
F 3 "~" H 7940 2170 50  0001 C CNN
	1    7940 2170
	0    1    1    0   
$EndComp
Wire Wire Line
	6590 4260 6590 5980
Wire Wire Line
	6590 5980 10480 5980
Wire Wire Line
	10480 5980 10480 5280
Wire Wire Line
	10480 5280 10380 5280
Wire Wire Line
	8070 4880 8580 4880
Wire Wire Line
	8070 4760 8070 4880
Wire Wire Line
	8070 4360 8180 4360
Wire Wire Line
	8180 4360 8180 4780
Wire Wire Line
	8180 4780 8580 4780
Wire Wire Line
	7120 4260 6590 4260
$Comp
L az_serial_module:az_serial_module U1
U 1 1 5E60DDAF
P 7670 3860
F 0 "U1" H 7595 4125 50  0000 C CNN
F 1 "az_serial_module" H 7595 4034 50  0000 C CNN
F 2 "az_serial_module:az_serial_module" H 7620 3760 50  0001 C CNN
F 3 "" H 7620 3760 50  0001 C CNN
	1    7670 3860
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5E6AF27D
P 10660 5750
F 0 "#PWR0101" H 10660 5500 50  0001 C CNN
F 1 "GND" H 10665 5577 50  0000 C CNN
F 2 "" H 10660 5750 50  0001 C CNN
F 3 "" H 10660 5750 50  0001 C CNN
	1    10660 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10380 5680 10660 5680
Wire Wire Line
	10660 5680 10660 5750
$Comp
L power:GND #PWR0102
U 1 1 5E6B5821
P 8270 4170
F 0 "#PWR0102" H 8270 3920 50  0001 C CNN
F 1 "GND" H 8275 3997 50  0000 C CNN
F 2 "" H 8270 4170 50  0001 C CNN
F 3 "" H 8270 4170 50  0001 C CNN
	1    8270 4170
	1    0    0    -1  
$EndComp
Wire Wire Line
	8070 4160 8270 4160
Wire Wire Line
	8270 4160 8270 4170
$Comp
L power:GND #PWR0103
U 1 1 5E6B947D
P 7000 4760
F 0 "#PWR0103" H 7000 4510 50  0001 C CNN
F 1 "GND" H 7005 4587 50  0000 C CNN
F 2 "" H 7000 4760 50  0001 C CNN
F 3 "" H 7000 4760 50  0001 C CNN
	1    7000 4760
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4760 7120 4760
$Comp
L NUCLEO-F767ZI:NUCLEO-F767ZI A1
U 1 1 5E60E56C
P 3380 3550
F 0 "A1" H 3089 5927 50  0000 C CNN
F 1 "NUCLEO-F767ZI" H 3089 5836 50  0000 C CNN
F 2 "NUCLEO-F767ZI:ST_NUCLEO-F767ZI" H 3380 3550 50  0001 L BNN
F 3 "7" H 3380 3550 50  0001 L BNN
F 4 "N/A" H 3380 3550 50  0001 L BNN "Field4"
F 5 "Manufacturer Recommendations" H 3380 3550 50  0001 L BNN "Field5"
F 6 "STMicroelectronics" H 3380 3550 50  0001 L BNN "Field6"
	1    3380 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	610  810  2340 810 
Wire Wire Line
	4160 810  4160 1650
Wire Wire Line
	4160 1650 4080 1650
Wire Wire Line
	4220 1670 4220 1650
Wire Wire Line
	4220 1650 4160 1650
Connection ~ 4160 1650
$Comp
L power:GND #PWR0104
U 1 1 5E72878C
P 630 1140
F 0 "#PWR0104" H 630 890 50  0001 C CNN
F 1 "GND" H 635 967 50  0000 C CNN
F 2 "" H 630 1140 50  0001 C CNN
F 3 "" H 630 1140 50  0001 C CNN
	1    630  1140
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5E72C0C6
P 5320 1130
F 0 "#PWR0105" H 5320 880 50  0001 C CNN
F 1 "GND" H 5325 957 50  0000 C CNN
F 2 "" H 5320 1130 50  0001 C CNN
F 3 "" H 5320 1130 50  0001 C CNN
	1    5320 1130
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1670 5200 2170
Wire Wire Line
	5200 2170 7790 2170
Wire Wire Line
	5200 1670 4220 1670
Wire Wire Line
	5200 2170 5200 3270
Wire Wire Line
	5200 3270 7960 3270
Connection ~ 5200 2170
Wire Wire Line
	1880 2490 1880 2100
Wire Wire Line
	1880 1500 2340 1500
Wire Wire Line
	2340 1500 2340 810 
Connection ~ 2340 810 
Wire Wire Line
	2340 810  4160 810 
Wire Wire Line
	1880 2100 1760 2100
Wire Wire Line
	1760 2100 1760 2790
Wire Wire Line
	1760 2790 1880 2790
Connection ~ 1880 2100
Wire Wire Line
	1880 2100 1880 1500
Wire Wire Line
	2180 2490 2220 2490
Wire Wire Line
	2220 2490 2220 2550
Connection ~ 2220 2550
Wire Wire Line
	2220 2550 2680 2550
Wire Wire Line
	2180 2790 2350 2790
Wire Wire Line
	2350 2790 2350 2650
Connection ~ 2350 2650
Wire Wire Line
	2350 2650 2680 2650
Wire Wire Line
	5870 1670 5870 3370
Wire Wire Line
	5870 3370 8380 3370
Wire Wire Line
	8380 3370 8380 3280
Wire Wire Line
	8380 3280 8580 3280
Wire Wire Line
	8260 3270 8380 3270
Wire Wire Line
	8380 3270 8380 3280
Connection ~ 8380 3280
Wire Wire Line
	5970 2280 8380 2280
Wire Wire Line
	8380 2180 8580 2180
Wire Wire Line
	8090 2170 8380 2170
Wire Wire Line
	8380 2170 8380 2180
Connection ~ 8380 2180
Wire Wire Line
	8380 2180 8380 2280
NoConn ~ 2680 2250
NoConn ~ 2680 2350
NoConn ~ 2680 2750
NoConn ~ 2680 2850
NoConn ~ 2680 2950
NoConn ~ 2680 3050
NoConn ~ 2680 3250
NoConn ~ 2680 3350
NoConn ~ 2680 3550
NoConn ~ 2680 3650
NoConn ~ 2680 3750
NoConn ~ 2680 3850
NoConn ~ 2680 3950
NoConn ~ 2680 4050
NoConn ~ 2680 4150
NoConn ~ 2680 4250
NoConn ~ 2680 4350
NoConn ~ 2680 4450
NoConn ~ 2680 4650
NoConn ~ 2680 4750
NoConn ~ 2680 4850
NoConn ~ 2680 4950
NoConn ~ 2680 5050
NoConn ~ 2680 5150
NoConn ~ 2680 5250
NoConn ~ 2680 5350
NoConn ~ 2680 5450
NoConn ~ 4080 5250
NoConn ~ 4080 5150
NoConn ~ 4080 4950
NoConn ~ 4080 4850
NoConn ~ 4080 4750
NoConn ~ 4080 4650
NoConn ~ 4080 4550
NoConn ~ 4080 4450
NoConn ~ 4080 4350
NoConn ~ 4080 4250
NoConn ~ 4080 4150
NoConn ~ 4080 4050
NoConn ~ 4080 3850
NoConn ~ 4080 3750
NoConn ~ 4080 3650
NoConn ~ 4080 3550
NoConn ~ 4080 3450
NoConn ~ 4080 3350
NoConn ~ 4080 3250
NoConn ~ 4080 3050
NoConn ~ 4080 2950
NoConn ~ 4080 2850
NoConn ~ 4080 2750
NoConn ~ 4080 2650
NoConn ~ 4080 2550
NoConn ~ 4080 2350
NoConn ~ 4080 2050
NoConn ~ 4080 1950
NoConn ~ 4080 1850
NoConn ~ 4080 1750
NoConn ~ 4080 1550
NoConn ~ 10380 5580
NoConn ~ 10380 5380
NoConn ~ 10380 5180
NoConn ~ 10380 5080
NoConn ~ 10380 4980
NoConn ~ 10380 4880
NoConn ~ 10380 4680
NoConn ~ 10380 4580
NoConn ~ 10380 4480
NoConn ~ 10380 4380
NoConn ~ 10380 4280
NoConn ~ 10380 4180
NoConn ~ 10380 4080
NoConn ~ 10380 3980
NoConn ~ 10380 3880
NoConn ~ 10380 3680
NoConn ~ 10380 3580
NoConn ~ 10380 3480
NoConn ~ 10380 3380
NoConn ~ 10380 3280
NoConn ~ 10380 3180
NoConn ~ 10380 3080
NoConn ~ 10380 2980
NoConn ~ 10380 2880
NoConn ~ 10380 2780
NoConn ~ 10380 2580
NoConn ~ 10380 2480
NoConn ~ 10380 2380
NoConn ~ 10380 2280
NoConn ~ 10380 2180
NoConn ~ 10380 2080
NoConn ~ 10380 1980
NoConn ~ 10380 1780
NoConn ~ 10380 1680
NoConn ~ 8580 1980
NoConn ~ 8580 2080
NoConn ~ 8580 2280
NoConn ~ 8580 2380
NoConn ~ 8580 2480
NoConn ~ 8580 2580
NoConn ~ 8580 2680
NoConn ~ 8580 2780
NoConn ~ 8580 2880
NoConn ~ 8580 3080
NoConn ~ 8580 3180
NoConn ~ 8580 3380
NoConn ~ 8580 3480
NoConn ~ 8580 3580
NoConn ~ 8580 3680
NoConn ~ 8580 3780
NoConn ~ 8580 3880
NoConn ~ 8580 3980
NoConn ~ 8580 4080
NoConn ~ 8580 4180
NoConn ~ 8580 4280
NoConn ~ 8580 4380
NoConn ~ 8580 4580
NoConn ~ 8580 4680
NoConn ~ 8580 4980
NoConn ~ 8580 5080
$Comp
L power:GND #PWR0106
U 1 1 5E840035
P 4190 5660
F 0 "#PWR0106" H 4190 5410 50  0001 C CNN
F 1 "GND" H 4195 5487 50  0000 C CNN
F 2 "" H 4190 5660 50  0001 C CNN
F 3 "" H 4190 5660 50  0001 C CNN
	1    4190 5660
	1    0    0    -1  
$EndComp
Wire Wire Line
	4190 5650 4190 5660
Wire Wire Line
	4080 5650 4190 5650
$Comp
L 3-5338556-1:3-5338556-1 J2
U 1 1 5E66670D
P 5480 1030
F 0 "J2" H 5930 1295 50  0000 C CNN
F 1 "left_encoder" H 5930 1204 50  0000 C CNN
F 2 "rj45-353385561:353385561" H 6230 1130 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/3-5338556-1.pdf" H 6230 1030 50  0001 L CNN
F 4 "TE Connectivity Female Cat3 RJ45 Modular Jack, STP, Right Angle, PCB Mount Mount, 35338556" H 6230 930 50  0001 L CNN "Description"
F 5 "13.15" H 6230 830 50  0001 L CNN "Height"
F 6 "571-3-5338556-1" H 6230 730 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=571-3-5338556-1" H 6230 630 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 6230 530 50  0001 L CNN "Manufacturer_Name"
F 9 "3-5338556-1" H 6230 430 50  0001 L CNN "Manufacturer_Part_Number"
	1    5480 1030
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1670 5200 1030
Wire Wire Line
	5200 1030 5480 1030
Connection ~ 5200 1670
Wire Wire Line
	5320 1130 5480 1130
Wire Wire Line
	5870 1670 5420 1670
Wire Wire Line
	5420 1670 5420 1230
Wire Wire Line
	5420 1230 5480 1230
Wire Wire Line
	5970 1580 5460 1580
Wire Wire Line
	5460 1580 5460 1330
Wire Wire Line
	5460 1330 5480 1330
Wire Wire Line
	5970 1580 5970 2280
$Comp
L NUCLEO-F767ZI:NUCLEO-F767ZI A1
U 2 1 5E62138F
P 9480 3680
F 0 "A1" H 9289 6057 50  0000 C CNN
F 1 "NUCLEO-F767ZI" H 9289 5966 50  0000 C CNN
F 2 "NUCLEO-F767ZI:ST_NUCLEO-F767ZI" H 9480 3680 50  0001 L BNN
F 3 "7" H 9480 3680 50  0001 L BNN
F 4 "N/A" H 9480 3680 50  0001 L BNN "Field4"
F 5 "Manufacturer Recommendations" H 9480 3680 50  0001 L BNN "Field5"
F 6 "STMicroelectronics" H 9480 3680 50  0001 L BNN "Field6"
	2    9480 3680
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5E66D988
P 6610 1300
F 0 "#PWR0107" H 6610 1050 50  0001 C CNN
F 1 "GND" H 6615 1127 50  0000 C CNN
F 2 "" H 6610 1300 50  0001 C CNN
F 3 "" H 6610 1300 50  0001 C CNN
	1    6610 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6380 1330 6480 1330
Wire Wire Line
	6500 1330 6500 1300
Wire Wire Line
	6500 1300 6610 1300
Wire Wire Line
	6380 1430 6480 1430
Wire Wire Line
	6480 1430 6480 1330
Connection ~ 6480 1330
Wire Wire Line
	6480 1330 6500 1330
$Comp
L 3-5338556-1:3-5338556-1 J1
U 1 1 5E6DB436
P 860 1040
F 0 "J1" H 1310 1305 50  0000 C CNN
F 1 "3-5338556-1" H 1310 1214 50  0000 C CNN
F 2 "353385561" H 1610 1140 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/3-5338556-1.pdf" H 1610 1040 50  0001 L CNN
F 4 "TE Connectivity Female Cat3 RJ45 Modular Jack, STP, Right Angle, PCB Mount Mount, 35338556" H 1610 940 50  0001 L CNN "Description"
F 5 "13.15" H 1610 840 50  0001 L CNN "Height"
F 6 "571-3-5338556-1" H 1610 740 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=571-3-5338556-1" H 1610 640 50  0001 L CNN "Mouser Price/Stock"
F 8 "TE Connectivity" H 1610 540 50  0001 L CNN "Manufacturer_Name"
F 9 "3-5338556-1" H 1610 440 50  0001 L CNN "Manufacturer_Part_Number"
	1    860  1040
	1    0    0    -1  
$EndComp
Wire Wire Line
	610  810  610  1020
Wire Wire Line
	610  1020 860  1020
Wire Wire Line
	860  1020 860  1040
Wire Wire Line
	630  1140 860  1140
Wire Wire Line
	720  1240 860  1240
Wire Wire Line
	720  1240 720  2550
Wire Wire Line
	720  2550 2220 2550
Wire Wire Line
	810  1340 860  1340
Wire Wire Line
	810  1340 810  2650
Wire Wire Line
	810  2650 2350 2650
$Comp
L power:GND #PWR0108
U 1 1 5E7285F3
P 1910 1300
F 0 "#PWR0108" H 1910 1050 50  0001 C CNN
F 1 "GND" H 1915 1127 50  0000 C CNN
F 2 "" H 1910 1300 50  0001 C CNN
F 3 "" H 1910 1300 50  0001 C CNN
	1    1910 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1760 1340 1790 1340
Wire Wire Line
	1810 1340 1810 1300
Wire Wire Line
	1810 1300 1910 1300
Wire Wire Line
	1760 1440 1790 1440
Wire Wire Line
	1790 1440 1790 1340
Connection ~ 1790 1340
Wire Wire Line
	1790 1340 1810 1340
$EndSCHEMATC
