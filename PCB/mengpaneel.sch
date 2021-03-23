EESchema Schematic File Version 4
LIBS:mengpaneel-cache
EELAYER 30 0
EELAYER END
$Descr A1 33110 23386
encoding utf-8
Sheet 1 1
Title ""
Date "2021-02-20"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 19160 9780 0    50   Input ~ 0
RX_UART
Text GLabel 19160 9880 0    50   Input ~ 0
TX_UART
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 9538DC76
P 19360 9780
F 0 "J2" H 19440 9772 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 19440 9681 50  0000 L CNN
F 2 "" H 19360 9780 50  0001 C CNN
F 3 "~" H 19360 9780 50  0001 C CNN
	1    19360 9780
	1    0    0    -1  
$EndComp
Wire Wire Line
	14620 11810 14620 11630
Wire Wire Line
	14950 11810 14950 11680
Wire Wire Line
	15290 11810 15290 11720
Connection ~ 14620 12410
Wire Wire Line
	14490 12410 14620 12410
Wire Wire Line
	14950 12410 14620 12410
Connection ~ 14950 12410
Wire Wire Line
	15290 12410 14950 12410
$Comp
L Switch:SW_DPST_x2 SW4
U 1 1 8F841806
P 14620 12010
F 0 "SW4" H 14620 12245 50  0000 C CNN
F 1 "SW_DPST_x2" H 14620 12154 50  0000 C CNN
F 2 "" H 14620 12010 50  0001 C CNN
F 3 "~" H 14620 12010 50  0001 C CNN
	1    14620 12010
	0    1    -1   0   
$EndComp
Wire Wire Line
	14620 12410 14620 12210
$Comp
L Switch:SW_DPST_x2 SW3
U 2 1 8F2CC16C
P 14950 12010
F 0 "SW3" H 14950 12245 50  0000 C CNN
F 1 "SW_DPST_x2" H 14950 12154 50  0000 C CNN
F 2 "" H 14950 12010 50  0001 C CNN
F 3 "~" H 14950 12010 50  0001 C CNN
	2    14950 12010
	0    1    -1   0   
$EndComp
Wire Wire Line
	14950 12410 14950 12210
$Comp
L Switch:SW_DPST_x2 SW3
U 1 1 603E27EC
P 15290 12010
F 0 "SW3" H 15290 12245 50  0000 C CNN
F 1 "SW_DPST_x2" H 15290 12154 50  0000 C CNN
F 2 "" H 15290 12010 50  0001 C CNN
F 3 "~" H 15290 12010 50  0001 C CNN
	1    15290 12010
	0    1    -1   0   
$EndComp
Text GLabel 14880 10280 0    50   Input ~ 0
CS_POT
Text GLabel 14880 10180 0    50   Input ~ 0
cs_leds
Text GLabel 16680 9980 2    50   Input ~ 0
CLK
Text GLabel 15280 11080 3    50   Input ~ 0
GPIO_INT
Text GLabel 16080 8580 1    50   Input ~ 0
L1.1
Text GLabel 16380 8580 1    50   Input ~ 0
Din_Leds
Text GLabel 15780 8580 1    50   Input ~ 0
SDA
Text GLabel 15880 8580 1    50   Input ~ 0
SCL
Text GLabel 16680 9780 2    50   Input ~ 0
RX_UART
Text GLabel 16680 9880 2    50   Input ~ 0
TX_UART
Text GLabel 11450 16790 0    50   Input ~ 0
Din_Leds
Text GLabel 10800 17890 0    50   Input ~ 0
cs_leds
Connection ~ 10880 17890
Wire Wire Line
	10880 17890 10800 17890
Wire Wire Line
	10880 17890 11450 17890
Wire Wire Line
	10880 19530 10880 17890
Wire Wire Line
	11450 19530 10880 19530
Wire Wire Line
	11350 18430 11450 18430
Wire Wire Line
	11350 18030 11350 18430
Wire Wire Line
	14000 18030 11350 18030
Wire Wire Line
	14000 16790 14000 18030
Wire Wire Line
	12950 16790 14000 16790
NoConn ~ 12950 18430
Text GLabel 12950 17890 2    50   Input ~ 0
CLK
Text GLabel 12950 19530 2    50   Input ~ 0
CLK
Wire Wire Line
	11450 17590 11070 17590
Wire Wire Line
	11070 17090 11450 17090
$Comp
L power:GND #PWR0217
U 1 1 72309BBA
P 11070 17090
F 0 "#PWR0217" H 11070 16840 50  0001 C CNN
F 1 "GND" H 11075 16917 50  0000 C CNN
F 2 "" H 11070 17090 50  0001 C CNN
F 3 "" H 11070 17090 50  0001 C CNN
	1    11070 17090
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0216
U 1 1 7212D723
P 11070 17590
F 0 "#PWR0216" H 11070 17340 50  0001 C CNN
F 1 "GND" H 11075 17417 50  0000 C CNN
F 2 "" H 11070 17590 50  0001 C CNN
F 3 "" H 11070 17590 50  0001 C CNN
	1    11070 17590
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0215
U 1 1 71F50EE2
P 11450 18730
F 0 "#PWR0215" H 11450 18480 50  0001 C CNN
F 1 "GND" H 11455 18557 50  0000 C CNN
F 2 "" H 11450 18730 50  0001 C CNN
F 3 "" H 11450 18730 50  0001 C CNN
	1    11450 18730
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0214
U 1 1 71D74F9F
P 11450 19230
F 0 "#PWR0214" H 11450 18980 50  0001 C CNN
F 1 "GND" H 11455 19057 50  0000 C CNN
F 2 "" H 11450 19230 50  0001 C CNN
F 3 "" H 11450 19230 50  0001 C CNN
	1    11450 19230
	0    1    1    0   
$EndComp
NoConn ~ 11450 18630
NoConn ~ 11450 18830
NoConn ~ 11450 19030
NoConn ~ 11450 19130
NoConn ~ 11450 19330
Text GLabel 11450 18930 0    50   Input ~ 0
LEDslider_11
Text GLabel 11450 19430 0    50   Input ~ 0
LEDslider_10
Text GLabel 11450 18530 0    50   Input ~ 0
LEDslider_9
Text GLabel 11450 17790 0    50   Input ~ 0
LEDslider_2
Text GLabel 11450 17690 0    50   Input ~ 0
LEDslider_6
Text GLabel 11450 17490 0    50   Input ~ 0
LEDslider_8
Text GLabel 11450 17390 0    50   Input ~ 0
LEDslider_4
Text GLabel 11450 17290 0    50   Input ~ 0
LEDslider_3
Text GLabel 11450 17190 0    50   Input ~ 0
LEDslider_7
Text GLabel 11450 16990 0    50   Input ~ 0
LEDslider_5
Text GLabel 11450 16890 0    50   Input ~ 0
LEDslider_1
NoConn ~ 12950 19330
NoConn ~ 12950 19130
NoConn ~ 12950 18730
NoConn ~ 12950 18630
Text GLabel 12950 18530 2    50   Input ~ 0
LEDlaag_12
Text GLabel 12950 18830 2    50   Input ~ 0
LEDlaag_11
Text GLabel 12950 19230 2    50   Input ~ 0
LEDlaag_10
Text GLabel 12950 19430 2    50   Input ~ 0
LEDlaag_9
Text GLabel 13680 18670 1    50   Input ~ 0
5V
Connection ~ 13680 18730
Wire Wire Line
	13680 18730 13680 18670
Wire Wire Line
	13550 18730 13680 18730
Wire Wire Line
	13550 18930 13550 18730
Wire Wire Line
	12950 18930 13550 18930
Wire Wire Line
	12950 19030 13680 19030
$Comp
L Device:R R20
U 1 1 6EB52893
P 13680 18880
F 0 "R20" H 13611 18834 50  0000 R CNN
F 1 "9,53k" H 13611 18925 50  0000 R CNN
F 2 "" V 13610 18880 50  0001 C CNN
F 3 "~" H 13680 18880 50  0001 C CNN
	1    13680 18880
	-1   0    0    1   
$EndComp
$Comp
L mengpaneel-rescue:MAX7219CNG+-MAX7219CNG+ IC2
U 1 1 6EB5288D
P 11450 18430
F 0 "IC2" H 12200 18695 50  0000 C CNN
F 1 "MAX7219CNG+" H 12200 18604 50  0000 C CNN
F 2 "" H 12800 18530 50  0001 L CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf" H 12800 18430 50  0001 L CNN
F 4 "8-digit LED display driver,MAX7219CNG,BP MAX7219CNG+, LED Driver, 8-Digits 64-Segments, 5 V, 24-Pin PDIP N" H 12800 18330 50  0001 L CNN "Description"
F 5 "4.572" H 12800 18230 50  0001 L CNN "Height"
F 6 "700-MAX7219CNG" H 12800 18130 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Maxim-Integrated/MAX7219CNG%2b?qs=1THa7WoU59Gme2Z0GeVXUQ%3D%3D" H 12800 18030 50  0001 L CNN "Mouser Price/Stock"
F 8 "Maxim Integrated" H 12800 17930 50  0001 L CNN "Manufacturer_Name"
F 9 "MAX7219CNG+" H 12800 17830 50  0001 L CNN "Manufacturer_Part_Number"
	1    11450 18430
	1    0    0    -1  
$EndComp
Text GLabel 12950 17490 2    50   Input ~ 0
LEDlaag_7
Text GLabel 12950 16890 2    50   Input ~ 0
LEDlaag_4
Text GLabel 12950 16990 2    50   Input ~ 0
LEDlaag_8
Text GLabel 12950 17090 2    50   Input ~ 0
LEDlaag_5
Text GLabel 12950 17190 2    50   Input ~ 0
LEDlaag_3
Text GLabel 12950 17590 2    50   Input ~ 0
LEDlaag_2
Text GLabel 12950 17690 2    50   Input ~ 0
LEDlaag_6
Text GLabel 12950 17790 2    50   Input ~ 0
LEDlaag_1
Text GLabel 13680 17030 1    50   Input ~ 0
5V
Connection ~ 13680 17090
Wire Wire Line
	13680 17090 13680 17030
Wire Wire Line
	13550 17090 13680 17090
Wire Wire Line
	13550 17290 13550 17090
Wire Wire Line
	12950 17290 13550 17290
Wire Wire Line
	12950 17390 13680 17390
$Comp
L Device:R R19
U 1 1 6C47B9E8
P 13680 17240
F 0 "R19" H 13611 17194 50  0000 R CNN
F 1 "9,53k" H 13611 17285 50  0000 R CNN
F 2 "" V 13610 17240 50  0001 C CNN
F 3 "~" H 13680 17240 50  0001 C CNN
	1    13680 17240
	-1   0    0    1   
$EndComp
$Comp
L mengpaneel-rescue:MAX7219CNG+-MAX7219CNG+ IC1
U 1 1 6C477F60
P 11450 16790
F 0 "IC1" H 12200 17055 50  0000 C CNN
F 1 "MAX7219CNG+" H 12200 16964 50  0000 C CNN
F 2 "" H 12800 16890 50  0001 L CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf" H 12800 16790 50  0001 L CNN
F 4 "8-digit LED display driver,MAX7219CNG,BP MAX7219CNG+, LED Driver, 8-Digits 64-Segments, 5 V, 24-Pin PDIP N" H 12800 16690 50  0001 L CNN "Description"
F 5 "4.572" H 12800 16590 50  0001 L CNN "Height"
F 6 "700-MAX7219CNG" H 12800 16490 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Maxim-Integrated/MAX7219CNG%2b?qs=1THa7WoU59Gme2Z0GeVXUQ%3D%3D" H 12800 16390 50  0001 L CNN "Mouser Price/Stock"
F 8 "Maxim Integrated" H 12800 16290 50  0001 L CNN "Manufacturer_Name"
F 9 "MAX7219CNG+" H 12800 16190 50  0001 L CNN "Manufacturer_Part_Number"
	1    11450 16790
	1    0    0    -1  
$EndComp
Connection ~ 8960 16800
Wire Wire Line
	9140 16800 8960 16800
Connection ~ 8960 16300
Wire Wire Line
	9140 16300 8960 16300
Connection ~ 8960 15780
Wire Wire Line
	9140 15780 8960 15780
Connection ~ 8960 15280
Wire Wire Line
	9140 15280 8960 15280
Connection ~ 8960 14780
Wire Wire Line
	9140 14780 8960 14780
Connection ~ 8960 14280
Wire Wire Line
	9140 14280 8960 14280
Connection ~ 8960 13780
Wire Wire Line
	9140 13780 8960 13780
Connection ~ 8960 13280
Wire Wire Line
	9140 13280 8960 13280
Connection ~ 8960 12780
Wire Wire Line
	9140 12780 8960 12780
Wire Wire Line
	8760 10340 8830 10340
Wire Wire Line
	8760 10200 8760 10340
Connection ~ 8960 17300
Wire Wire Line
	8960 17300 9140 17300
Connection ~ 8960 17800
Wire Wire Line
	8960 17800 9140 17800
Connection ~ 8960 18300
Wire Wire Line
	8960 18300 9140 18300
NoConn ~ 13390 5900
NoConn ~ 13390 5800
Wire Wire Line
	10600 5460 10670 5460
Wire Wire Line
	10600 5330 10600 5460
Wire Wire Line
	7580 10860 7730 10860
Wire Wire Line
	8740 5470 8800 5470
Wire Wire Line
	8740 5340 8740 5470
Wire Wire Line
	5230 10330 5290 10330
Wire Wire Line
	5230 10200 5230 10330
Wire Wire Line
	5200 8740 5290 8740
Wire Wire Line
	5200 8630 5200 8740
Wire Wire Line
	8740 8750 8780 8750
Wire Wire Line
	8740 8640 8740 8750
Wire Wire Line
	8700 7140 8750 7140
Wire Wire Line
	8700 7040 8700 7140
Wire Wire Line
	14290 7100 14420 7100
Connection ~ 14290 7100
Wire Wire Line
	14190 7100 14290 7100
$Comp
L Device:R R77
U 1 1 7DE5E2ED
P 14290 6950
F 0 "R77" H 14360 6917 50  0000 L CNN
F 1 "20k" H 14360 6846 50  0000 L CNN
F 2 "" V 14220 6950 50  0001 C CNN
F 3 "~" H 14290 6950 50  0001 C CNN
	1    14290 6950
	1    0    0    -1  
$EndComp
Connection ~ 13180 5300
Wire Wire Line
	13270 5300 13180 5300
Wire Wire Line
	13270 5600 13270 5300
Wire Wire Line
	13390 5600 13270 5600
Wire Wire Line
	13020 5300 13180 5300
Wire Wire Line
	12800 5000 13180 5000
Text GLabel 12800 5000 0    50   Input ~ 0
3.3V
$Comp
L Device:R R70
U 1 1 7C849D6C
P 13180 5150
F 0 "R70" H 13250 5196 50  0000 L CNN
F 1 "1k" H 13250 5105 50  0000 L CNN
F 2 "" V 13110 5150 50  0001 C CNN
F 3 "~" H 13180 5150 50  0001 C CNN
	1    13180 5150
	1    0    0    -1  
$EndComp
Text GLabel 13020 5300 0    50   Input ~ 0
GPIO_INT
Wire Wire Line
	13180 3800 13180 3700
Wire Wire Line
	13180 4600 13180 4500
Text GLabel 13180 4500 1    50   Input ~ 0
3.3V
Text GLabel 13180 3700 1    50   Input ~ 0
3.3V
Wire Wire Line
	13180 4100 13020 4100
Connection ~ 13180 4100
Wire Wire Line
	13790 4100 13180 4100
Wire Wire Line
	13790 4200 13790 4100
Wire Wire Line
	13180 4900 13020 4900
Connection ~ 13180 4900
Wire Wire Line
	13390 4900 13180 4900
Text GLabel 13020 4900 0    50   Input ~ 0
SDA
Text GLabel 13020 4100 0    50   Input ~ 0
SCL
$Comp
L Device:R R69
U 1 1 7ADF1028
P 13180 4750
F 0 "R69" H 13250 4796 50  0000 L CNN
F 1 "1k" H 13250 4705 50  0000 L CNN
F 2 "" V 13110 4750 50  0001 C CNN
F 3 "~" H 13180 4750 50  0001 C CNN
	1    13180 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R68
U 1 1 7ADE96CE
P 13180 3950
F 0 "R68" H 13250 3996 50  0000 L CNN
F 1 "1k" H 13250 3905 50  0000 L CNN
F 2 "" V 13110 3950 50  0001 C CNN
F 3 "~" H 13180 3950 50  0001 C CNN
	1    13180 3950
	1    0    0    -1  
$EndComp
Text GLabel 9140 14780 2    50   Input ~ 0
LEDlaag_8
Text GLabel 9140 18300 2    50   Input ~ 0
LEDlaag_1
Text GLabel 9140 17800 2    50   Input ~ 0
LEDlaag_2
Text GLabel 9140 17300 2    50   Input ~ 0
LEDlaag_3
Text GLabel 9140 16800 2    50   Input ~ 0
LEDlaag_4
Text GLabel 9140 16300 2    50   Input ~ 0
LEDlaag_5
Text GLabel 9140 15780 2    50   Input ~ 0
LEDlaag_6
Text GLabel 9140 15280 2    50   Input ~ 0
LEDlaag_7
Text GLabel 9140 14280 2    50   Input ~ 0
LEDlaag_9
Text GLabel 9140 13780 2    50   Input ~ 0
LEDlaag_10
Text GLabel 9140 13280 2    50   Input ~ 0
LEDlaag_11
Text GLabel 9140 12780 2    50   Input ~ 0
LEDlaag_12
Wire Notes Line
	7850 8680 8850 8680
Wire Notes Line
	8850 7190 7850 7190
Wire Notes Line
	5470 11410 7670 11410
Wire Notes Line
	5470 10400 5470 11400
Text GLabel 5760 11310 1    50   Input ~ 0
Slider_7.2
Text GLabel 5870 11310 1    50   Input ~ 0
Slider_7.3
Text GLabel 5990 11310 1    50   Input ~ 0
Slider_7.4
Text GLabel 6100 11310 1    50   Input ~ 0
Slider_7.5
Text GLabel 6220 11310 1    50   Input ~ 0
Slider_7.6
Text GLabel 6330 11310 1    50   Input ~ 0
Slider_7.7
Text GLabel 6440 11310 1    50   Input ~ 0
Slider_7.8
Text GLabel 7010 11290 1    50   Input ~ 0
GND
Text GLabel 5650 11310 1    50   Input ~ 0
Slider_7.1
Wire Notes Line
	7670 10400 7670 11400
Wire Notes Line
	8850 7070 8850 6070
Wire Notes Line
	7850 7070 8850 7070
Wire Notes Line
	5470 10400 7670 10400
Wire Notes Line
	4320 8690 5320 8690
Text Notes 5390 6300 0    50   ~ 0
BASS
Text Notes 8960 6290 0    50   ~ 0
BASS
Text Notes 8960 8070 0    50   ~ 0
MEDIUM
Text Notes 8960 9580 0    50   ~ 0
TREBLE
Text Notes 5380 7840 0    50   ~ 0
MEDIUM
Text Notes 5390 9560 0    50   ~ 0
TREBLE
Text Notes 4640 3840 0    50   ~ 0
Line 1 IN
Text Notes 8060 3860 0    50   ~ 0
Line 2 IN
Text Notes 10160 3850 0    50   ~ 0
OUT 
Text GLabel 8700 7040 1    50   Input ~ 0
Slider_8x
Text GLabel 8740 8640 1    50   Input ~ 0
Slider_9x
Text GLabel 5200 8630 1    50   Input ~ 0
Slider_4x
Text GLabel 5230 10200 1    50   Input ~ 0
Slider_5x
Text GLabel 10600 5330 1    50   Input ~ 0
Slider_11x
Text GLabel 8760 10200 1    50   Input ~ 0
Slider_10x
Text GLabel 7580 10860 0    50   Input ~ 0
Slider_7x
Text Notes 6350 10360 0    50   ~ 0
L-R Line 1
Wire Wire Line
	5650 11470 5650 11310
Wire Wire Line
	5760 11470 5760 11310
Wire Wire Line
	6440 11470 6440 11310
Wire Wire Line
	5870 11470 5870 11310
Wire Wire Line
	5990 11310 5990 11470
Wire Wire Line
	6100 11470 6100 11310
Wire Wire Line
	6220 11310 6220 11470
Wire Wire Line
	6330 11470 6330 11310
Text GLabel 6560 11310 1    50   Input ~ 0
Slider_7.9
Text GLabel 6680 11310 1    50   Input ~ 0
Slider_7.10
Text GLabel 6790 11310 1    50   Input ~ 0
Slider_7.11
Text GLabel 6900 11310 1    50   Input ~ 0
Slider_7.12
Wire Wire Line
	6900 11470 6900 11310
Wire Wire Line
	6560 11470 6560 11310
Wire Wire Line
	6680 11310 6680 11470
Wire Wire Line
	6790 11470 6790 11310
Text GLabel 3040 3380 0    50   Input ~ 0
laag_1
Text GLabel 3040 3280 0    50   Input ~ 0
laag_2
Text GLabel 3040 3180 0    50   Input ~ 0
laag_3
Text GLabel 3040 3080 0    50   Input ~ 0
laag_4
Text GLabel 3040 2980 0    50   Input ~ 0
laag_5
Text GLabel 3040 2880 0    50   Input ~ 0
laag_6
Text GLabel 3040 2780 0    50   Input ~ 0
laag_7
Text GLabel 3040 2680 0    50   Input ~ 0
laag_8
Text GLabel 3040 2580 0    50   Input ~ 0
laag_9
Text GLabel 3040 2480 0    50   Input ~ 0
laag_10
Text GLabel 3040 2380 0    50   Input ~ 0
laag_11
Text GLabel 3040 2280 0    50   Input ~ 0
laag_12
Wire Wire Line
	7010 11760 7010 11290
Text GLabel 7950 6800 2    50   Input ~ 0
Slider_8.2
Text GLabel 7950 6690 2    50   Input ~ 0
Slider_8.3
Text GLabel 7950 6570 2    50   Input ~ 0
Slider_8.4
Text GLabel 7950 6460 2    50   Input ~ 0
Slider_8.5
Text GLabel 7950 6340 2    50   Input ~ 0
Slider_8.6
Text GLabel 7950 6230 2    50   Input ~ 0
Slider_8.7
Text GLabel 7950 6120 2    50   Input ~ 0
Slider_8.8
Text GLabel 7950 6910 2    50   Input ~ 0
Slider_8.1
Text GLabel 7790 6910 0    50   Input ~ 0
laag_1
Text GLabel 7790 6800 0    50   Input ~ 0
laag_2
Text GLabel 7790 6690 0    50   Input ~ 0
laag_3
Text GLabel 7790 6570 0    50   Input ~ 0
laag_4
Text GLabel 7790 6460 0    50   Input ~ 0
laag_5
Text GLabel 7790 6340 0    50   Input ~ 0
laag_6
Text GLabel 7790 6230 0    50   Input ~ 0
laag_7
Text GLabel 7790 6120 0    50   Input ~ 0
laag_8
Wire Wire Line
	7790 6910 7950 6910
Wire Wire Line
	7790 6800 7950 6800
Wire Wire Line
	7790 6120 7950 6120
Wire Wire Line
	7790 6690 7950 6690
Wire Wire Line
	7950 6570 7790 6570
Wire Wire Line
	7790 6460 7950 6460
Wire Wire Line
	7950 6340 7790 6340
Wire Wire Line
	7790 6230 7950 6230
Text GLabel 7950 6000 2    50   Input ~ 0
Slider_8.9
Text GLabel 7950 5880 2    50   Input ~ 0
Slider_8.10
Text GLabel 7950 5770 2    50   Input ~ 0
Slider_8.11
Text GLabel 7950 5660 2    50   Input ~ 0
Slider_8.12
Text GLabel 7790 6000 0    50   Input ~ 0
laag_9
Text GLabel 7790 5880 0    50   Input ~ 0
laag_10
Text GLabel 7790 5770 0    50   Input ~ 0
laag_11
Text GLabel 7790 5660 0    50   Input ~ 0
laag_12
Wire Wire Line
	7790 5660 7950 5660
Wire Wire Line
	7790 6000 7950 6000
Wire Wire Line
	7950 5880 7790 5880
Wire Wire Line
	7790 5770 7950 5770
Text GLabel 7980 8410 2    50   Input ~ 0
Slider_9.2
Text GLabel 7980 8300 2    50   Input ~ 0
Slider_9.3
Text GLabel 7980 8180 2    50   Input ~ 0
Slider_9.4
Text GLabel 7980 8070 2    50   Input ~ 0
Slider_9.5
Text GLabel 7980 7950 2    50   Input ~ 0
Slider_9.6
Text GLabel 7980 7840 2    50   Input ~ 0
Slider_9.7
Text GLabel 7980 7730 2    50   Input ~ 0
Slider_9.8
Text GLabel 7980 8520 2    50   Input ~ 0
Slider_9.1
Text GLabel 7820 8520 0    50   Input ~ 0
laag_1
Text GLabel 7820 8410 0    50   Input ~ 0
laag_2
Text GLabel 7820 8300 0    50   Input ~ 0
laag_3
Text GLabel 7820 8180 0    50   Input ~ 0
laag_4
Text GLabel 7820 8070 0    50   Input ~ 0
laag_5
Text GLabel 7820 7950 0    50   Input ~ 0
laag_6
Text GLabel 7820 7840 0    50   Input ~ 0
laag_7
Text GLabel 7820 7730 0    50   Input ~ 0
laag_8
Wire Wire Line
	7820 8520 7980 8520
Wire Wire Line
	7820 8410 7980 8410
Wire Wire Line
	7820 7730 7980 7730
Wire Wire Line
	7820 8300 7980 8300
Wire Wire Line
	7980 8180 7820 8180
Wire Wire Line
	7820 8070 7980 8070
Wire Wire Line
	7980 7950 7820 7950
Wire Wire Line
	7820 7840 7980 7840
Text GLabel 7980 7610 2    50   Input ~ 0
Slider_9.9
Text GLabel 7980 7490 2    50   Input ~ 0
Slider_9.10
Text GLabel 7980 7380 2    50   Input ~ 0
Slider_9.11
Text GLabel 7980 7270 2    50   Input ~ 0
Slider_9.12
Text GLabel 7820 7610 0    50   Input ~ 0
laag_9
Text GLabel 7820 7490 0    50   Input ~ 0
laag_10
Text GLabel 7820 7380 0    50   Input ~ 0
laag_11
Text GLabel 7820 7270 0    50   Input ~ 0
laag_12
Wire Wire Line
	7820 7270 7980 7270
Wire Wire Line
	7820 7610 7980 7610
Wire Wire Line
	7980 7490 7820 7490
Wire Wire Line
	7820 7380 7980 7380
Wire Notes Line
	8850 7190 8850 8680
Wire Notes Line
	7850 7190 7850 8680
Wire Notes Line
	8850 6060 8850 5570
Wire Notes Line
	7850 5570 7850 7070
Wire Notes Line
	7850 5570 8850 5570
Text GLabel 4420 8420 2    50   Input ~ 0
Slider_4.2
Text GLabel 4420 8310 2    50   Input ~ 0
Slider_4.3
Text GLabel 4420 8190 2    50   Input ~ 0
Slider_4.4
Text GLabel 4420 8080 2    50   Input ~ 0
Slider_4.5
Text GLabel 4420 7960 2    50   Input ~ 0
Slider_4.6
Text GLabel 4420 7850 2    50   Input ~ 0
Slider_4.7
Text GLabel 4420 7740 2    50   Input ~ 0
Slider_4.8
Text GLabel 4420 8530 2    50   Input ~ 0
Slider_4.1
Text GLabel 4260 8530 0    50   Input ~ 0
laag_1
Text GLabel 4260 8420 0    50   Input ~ 0
laag_2
Text GLabel 4260 8310 0    50   Input ~ 0
laag_3
Text GLabel 4260 8190 0    50   Input ~ 0
laag_4
Text GLabel 4260 8080 0    50   Input ~ 0
laag_5
Text GLabel 4260 7960 0    50   Input ~ 0
laag_6
Text GLabel 4260 7850 0    50   Input ~ 0
laag_7
Text GLabel 4260 7740 0    50   Input ~ 0
laag_8
Wire Wire Line
	4260 8530 4420 8530
Wire Wire Line
	4260 8420 4420 8420
Wire Wire Line
	4260 7740 4420 7740
Wire Wire Line
	4260 8310 4420 8310
Wire Wire Line
	4420 8190 4260 8190
Wire Wire Line
	4260 8080 4420 8080
Wire Wire Line
	4420 7960 4260 7960
Wire Wire Line
	4260 7850 4420 7850
Text GLabel 4420 7620 2    50   Input ~ 0
Slider_4.9
Text GLabel 4420 7500 2    50   Input ~ 0
Slider_4.10
Text GLabel 4420 7390 2    50   Input ~ 0
Slider_4.11
Text GLabel 4420 7280 2    50   Input ~ 0
Slider_4.12
Text GLabel 4260 7620 0    50   Input ~ 0
laag_9
Text GLabel 4260 7500 0    50   Input ~ 0
laag_10
Text GLabel 4260 7390 0    50   Input ~ 0
laag_11
Text GLabel 4260 7280 0    50   Input ~ 0
laag_12
Wire Wire Line
	4260 7280 4420 7280
Wire Wire Line
	4260 7620 4420 7620
Wire Wire Line
	4420 7500 4260 7500
Wire Wire Line
	4260 7390 4420 7390
Wire Notes Line
	4320 8690 4320 7200
Wire Notes Line
	4320 7200 5320 7200
Wire Notes Line
	5320 7200 5320 8690
Text GLabel 8740 5340 1    50   Input ~ 0
Slider_6x
Wire Notes Line
	4320 10280 5320 10280
Text GLabel 4420 10010 2    50   Input ~ 0
Slider_5.2
Text GLabel 4420 9900 2    50   Input ~ 0
Slider_5.3
Text GLabel 4420 9780 2    50   Input ~ 0
Slider_5.4
Text GLabel 4420 9670 2    50   Input ~ 0
Slider_5.5
Text GLabel 4420 9550 2    50   Input ~ 0
Slider_5.6
Text GLabel 4420 9440 2    50   Input ~ 0
Slider_5.7
Text GLabel 4420 9330 2    50   Input ~ 0
Slider_5.8
Text GLabel 4420 10120 2    50   Input ~ 0
Slider_5.1
Text GLabel 4260 10120 0    50   Input ~ 0
laag_1
Text GLabel 4260 10010 0    50   Input ~ 0
laag_2
Text GLabel 4260 9900 0    50   Input ~ 0
laag_3
Text GLabel 4260 9780 0    50   Input ~ 0
laag_4
Text GLabel 4260 9670 0    50   Input ~ 0
laag_5
Text GLabel 4260 9550 0    50   Input ~ 0
laag_6
Text GLabel 4260 9440 0    50   Input ~ 0
laag_7
Text GLabel 4260 9330 0    50   Input ~ 0
laag_8
Wire Wire Line
	4260 10120 4420 10120
Wire Wire Line
	4260 10010 4420 10010
Wire Wire Line
	4260 9330 4420 9330
Wire Wire Line
	4260 9900 4420 9900
Wire Wire Line
	4420 9780 4260 9780
Wire Wire Line
	4260 9670 4420 9670
Wire Wire Line
	4420 9550 4260 9550
Wire Wire Line
	4260 9440 4420 9440
Text GLabel 4420 9210 2    50   Input ~ 0
Slider_5.9
Text GLabel 4420 9090 2    50   Input ~ 0
Slider_5.10
Text GLabel 4420 8980 2    50   Input ~ 0
Slider_5.11
Text GLabel 4420 8870 2    50   Input ~ 0
Slider_5.12
Text GLabel 4260 9210 0    50   Input ~ 0
laag_9
Text GLabel 4260 9090 0    50   Input ~ 0
laag_10
Text GLabel 4260 8980 0    50   Input ~ 0
laag_11
Text GLabel 4260 8870 0    50   Input ~ 0
laag_12
Wire Wire Line
	4260 8870 4420 8870
Wire Wire Line
	4260 9210 4420 9210
Wire Wire Line
	4420 9090 4260 9090
Wire Wire Line
	4260 8980 4420 8980
Wire Notes Line
	4320 8790 5320 8790
Wire Notes Line
	5320 8790 5320 10280
Wire Notes Line
	7830 5420 8830 5420
Text GLabel 7930 5150 2    50   Input ~ 0
Slider_6.2
Text GLabel 7930 5040 2    50   Input ~ 0
Slider_6.3
Text GLabel 7930 4920 2    50   Input ~ 0
Slider_6.4
Text GLabel 7930 4810 2    50   Input ~ 0
Slider_6.5
Text GLabel 7930 4690 2    50   Input ~ 0
Slider_6.6
Text GLabel 7930 4580 2    50   Input ~ 0
Slider_6.7
Text GLabel 7930 4470 2    50   Input ~ 0
Slider_6.8
Text GLabel 7930 5260 2    50   Input ~ 0
Slider_6.1
Text GLabel 7770 5260 0    50   Input ~ 0
laag_1
Text GLabel 7770 5150 0    50   Input ~ 0
laag_2
Text GLabel 7770 5040 0    50   Input ~ 0
laag_3
Text GLabel 7770 4920 0    50   Input ~ 0
laag_4
Text GLabel 7770 4810 0    50   Input ~ 0
laag_5
Text GLabel 7770 4690 0    50   Input ~ 0
laag_6
Text GLabel 7770 4580 0    50   Input ~ 0
laag_7
Text GLabel 7770 4470 0    50   Input ~ 0
laag_8
Wire Wire Line
	7770 5260 7930 5260
Wire Wire Line
	7770 5150 7930 5150
Wire Wire Line
	7770 4470 7930 4470
Wire Wire Line
	7770 5040 7930 5040
Wire Wire Line
	7930 4920 7770 4920
Wire Wire Line
	7770 4810 7930 4810
Wire Wire Line
	7930 4690 7770 4690
Wire Wire Line
	7770 4580 7930 4580
Text GLabel 7930 4350 2    50   Input ~ 0
Slider_6.9
Text GLabel 7930 4230 2    50   Input ~ 0
Slider_6.10
Text GLabel 7930 4120 2    50   Input ~ 0
Slider_6.11
Text GLabel 7930 4010 2    50   Input ~ 0
Slider_6.12
Text GLabel 7770 4350 0    50   Input ~ 0
laag_9
Text GLabel 7770 4230 0    50   Input ~ 0
laag_10
Text GLabel 7770 4120 0    50   Input ~ 0
laag_11
Text GLabel 7770 4010 0    50   Input ~ 0
laag_12
Wire Wire Line
	7770 4010 7930 4010
Wire Wire Line
	7770 4350 7930 4350
Wire Wire Line
	7930 4230 7770 4230
Wire Wire Line
	7770 4120 7930 4120
Wire Notes Line
	7830 5420 7830 3930
Wire Notes Line
	7830 3930 8830 3930
Wire Notes Line
	8830 3930 8830 5420
Wire Notes Line
	7860 10290 8860 10290
Text GLabel 7960 10020 2    50   Input ~ 0
Slider_10.2
Text GLabel 7960 9910 2    50   Input ~ 0
Slider_10.3
Text GLabel 7960 9790 2    50   Input ~ 0
Slider_10.4
Text GLabel 7960 9680 2    50   Input ~ 0
Slider_10.5
Text GLabel 7960 9560 2    50   Input ~ 0
Slider_10.6
Text GLabel 7960 9450 2    50   Input ~ 0
Slider_10.7
Text GLabel 7960 9340 2    50   Input ~ 0
Slider_10.8
Text GLabel 7960 10130 2    50   Input ~ 0
Slider_10.1
Text GLabel 7800 10130 0    50   Input ~ 0
laag_1
Text GLabel 7800 10020 0    50   Input ~ 0
laag_2
Text GLabel 7800 9910 0    50   Input ~ 0
laag_3
Text GLabel 7800 9790 0    50   Input ~ 0
laag_4
Text GLabel 7800 9680 0    50   Input ~ 0
laag_5
Text GLabel 7800 9560 0    50   Input ~ 0
laag_6
Text GLabel 7800 9450 0    50   Input ~ 0
laag_7
Text GLabel 7800 9340 0    50   Input ~ 0
laag_8
Wire Wire Line
	7800 10130 7960 10130
Wire Wire Line
	7800 10020 7960 10020
Wire Wire Line
	7800 9340 7960 9340
Wire Wire Line
	7800 9910 7960 9910
Wire Wire Line
	7960 9790 7800 9790
Wire Wire Line
	7800 9680 7960 9680
Wire Wire Line
	7960 9560 7800 9560
Wire Wire Line
	7800 9450 7960 9450
Text GLabel 7960 9220 2    50   Input ~ 0
Slider_10.9
Text GLabel 7960 9100 2    50   Input ~ 0
Slider_10.10
Text GLabel 7960 8990 2    50   Input ~ 0
Slider_10.11
Text GLabel 7960 8880 2    50   Input ~ 0
Slider_10.12
Text GLabel 7800 9220 0    50   Input ~ 0
laag_9
Text GLabel 7800 9100 0    50   Input ~ 0
laag_10
Text GLabel 7800 8990 0    50   Input ~ 0
laag_11
Text GLabel 7800 8880 0    50   Input ~ 0
laag_12
Wire Wire Line
	7800 8880 7960 8880
Wire Wire Line
	7800 9220 7960 9220
Wire Wire Line
	7960 9100 7800 9100
Wire Wire Line
	7800 8990 7960 8990
Wire Notes Line
	7860 10290 7860 8800
Wire Notes Line
	7860 8800 8860 8800
Wire Notes Line
	8860 8800 8860 10290
Wire Notes Line
	9700 5410 10700 5410
Text GLabel 9800 5140 2    50   Input ~ 0
Slider_11.2
Text GLabel 9800 5030 2    50   Input ~ 0
Slider_11.3
Text GLabel 9800 4910 2    50   Input ~ 0
Slider_11.4
Text GLabel 9800 4800 2    50   Input ~ 0
Slider_11.5
Text GLabel 9800 4680 2    50   Input ~ 0
Slider_11.6
Text GLabel 9800 4570 2    50   Input ~ 0
Slider_11.7
Text GLabel 9800 4460 2    50   Input ~ 0
Slider_11.8
Text GLabel 9800 5250 2    50   Input ~ 0
Slider_11.1
Text GLabel 9640 5250 0    50   Input ~ 0
laag_1
Text GLabel 9640 5140 0    50   Input ~ 0
laag_2
Text GLabel 9640 5030 0    50   Input ~ 0
laag_3
Text GLabel 9640 4910 0    50   Input ~ 0
laag_4
Text GLabel 9640 4800 0    50   Input ~ 0
laag_5
Text GLabel 9640 4680 0    50   Input ~ 0
laag_6
Text GLabel 9640 4570 0    50   Input ~ 0
laag_7
Text GLabel 9640 4460 0    50   Input ~ 0
laag_8
Wire Wire Line
	9640 5250 9800 5250
Wire Wire Line
	9640 5140 9800 5140
Wire Wire Line
	9640 4460 9800 4460
Wire Wire Line
	9640 5030 9800 5030
Wire Wire Line
	9800 4910 9640 4910
Wire Wire Line
	9640 4800 9800 4800
Wire Wire Line
	9800 4680 9640 4680
Wire Wire Line
	9640 4570 9800 4570
Text GLabel 9800 4340 2    50   Input ~ 0
Slider_11.9
Text GLabel 9800 4220 2    50   Input ~ 0
Slider_11.10
Text GLabel 9800 4110 2    50   Input ~ 0
Slider_11.11
Text GLabel 9800 4000 2    50   Input ~ 0
Slider_11.12
Text GLabel 9640 4340 0    50   Input ~ 0
laag_9
Text GLabel 9640 4220 0    50   Input ~ 0
laag_10
Text GLabel 9640 4110 0    50   Input ~ 0
laag_11
Text GLabel 9640 4000 0    50   Input ~ 0
laag_12
Wire Wire Line
	9640 4000 9800 4000
Wire Wire Line
	9640 4340 9800 4340
Wire Wire Line
	9800 4220 9640 4220
Wire Wire Line
	9640 4110 9800 4110
Wire Notes Line
	9700 5410 9700 3920
Wire Notes Line
	9700 3920 10700 3920
Wire Notes Line
	10700 3920 10700 5410
Wire Wire Line
	4010 10230 4420 10230
$Comp
L power:GND #PWR0213
U 1 1 68186611
P 4010 10230
F 0 "#PWR0213" H 4010 9980 50  0001 C CNN
F 1 "GND" H 4015 10057 50  0000 C CNN
F 2 "" H 4010 10230 50  0001 C CNN
F 3 "" H 4010 10230 50  0001 C CNN
	1    4010 10230
	0    1    1    0   
$EndComp
Wire Notes Line
	4320 10280 4320 8790
Text GLabel 4420 10230 2    50   Input ~ 0
GND
Wire Wire Line
	7520 5370 7930 5370
$Comp
L power:GND #PWR0212
U 1 1 6842B1A6
P 7520 5370
F 0 "#PWR0212" H 7520 5120 50  0001 C CNN
F 1 "GND" H 7525 5197 50  0000 C CNN
F 2 "" H 7520 5370 50  0001 C CNN
F 3 "" H 7520 5370 50  0001 C CNN
	1    7520 5370
	0    1    1    0   
$EndComp
Text GLabel 7930 5370 2    50   Input ~ 0
GND
Wire Wire Line
	7550 10240 7960 10240
$Comp
L power:GND #PWR0211
U 1 1 68AAF4E2
P 7550 10240
F 0 "#PWR0211" H 7550 9990 50  0001 C CNN
F 1 "GND" H 7555 10067 50  0000 C CNN
F 2 "" H 7550 10240 50  0001 C CNN
F 3 "" H 7550 10240 50  0001 C CNN
	1    7550 10240
	0    1    1    0   
$EndComp
Text GLabel 7960 10240 2    50   Input ~ 0
GND
Wire Wire Line
	9390 5360 9800 5360
$Comp
L power:GND #PWR0210
U 1 1 68B58A20
P 9390 5360
F 0 "#PWR0210" H 9390 5110 50  0001 C CNN
F 1 "GND" H 9395 5187 50  0000 C CNN
F 2 "" H 9390 5360 50  0001 C CNN
F 3 "" H 9390 5360 50  0001 C CNN
	1    9390 5360
	0    1    1    0   
$EndComp
Text GLabel 9800 5360 2    50   Input ~ 0
GND
Wire Wire Line
	4010 8640 4420 8640
$Comp
L power:GND #PWR0209
U 1 1 68D559EE
P 4010 8640
F 0 "#PWR0209" H 4010 8390 50  0001 C CNN
F 1 "GND" H 4015 8467 50  0000 C CNN
F 2 "" H 4010 8640 50  0001 C CNN
F 3 "" H 4010 8640 50  0001 C CNN
	1    4010 8640
	0    1    1    0   
$EndComp
Text GLabel 4420 8640 2    50   Input ~ 0
GND
$Comp
L power:GND #PWR0208
U 1 1 69150775
P 7670 8630
F 0 "#PWR0208" H 7670 8380 50  0001 C CNN
F 1 "GND" H 7675 8457 50  0000 C CNN
F 2 "" H 7670 8630 50  0001 C CNN
F 3 "" H 7670 8630 50  0001 C CNN
	1    7670 8630
	0    1    1    0   
$EndComp
Text GLabel 7980 8630 2    50   Input ~ 0
GND
Wire Wire Line
	7540 7020 7950 7020
$Comp
L power:GND #PWR0207
U 1 1 691FC018
P 7540 7020
F 0 "#PWR0207" H 7540 6770 50  0001 C CNN
F 1 "GND" H 7545 6847 50  0000 C CNN
F 2 "" H 7540 7020 50  0001 C CNN
F 3 "" H 7540 7020 50  0001 C CNN
	1    7540 7020
	0    1    1    0   
$EndComp
Text GLabel 7950 7020 2    50   Input ~ 0
GND
Wire Wire Line
	7670 8630 7980 8630
Text GLabel 5210 5470 2    50   Input ~ 0
slider_2
Text GLabel 5260 7150 2    50   Input ~ 0
slider_3
Text GLabel 8750 7140 2    50   Input ~ 0
slider_8
Text GLabel 8780 8750 2    50   Input ~ 0
slider_9
Text GLabel 8690 11950 3    50   Input ~ 0
slider_7
Text GLabel 10670 5460 2    50   Input ~ 0
slider_11
Text GLabel 8830 10340 2    50   Input ~ 0
slider_10
Text GLabel 5290 8740 2    50   Input ~ 0
slider_4
Text GLabel 5290 10330 2    50   Input ~ 0
slider_5
Text GLabel 8800 5470 2    50   Input ~ 0
slider_6
Text GLabel 8960 18700 3    50   Input ~ 0
LEDslider_11
Text GLabel 8360 18700 3    50   Input ~ 0
LEDslider_10
Text GLabel 7650 18700 3    50   Input ~ 0
LEDslider_9
Text GLabel 7080 18700 3    50   Input ~ 0
LEDslider_8
Text GLabel 6370 18700 3    50   Input ~ 0
LEDslider_7
Text GLabel 5770 18700 3    50   Input ~ 0
LEDslider_6
Text GLabel 5060 18700 3    50   Input ~ 0
LEDslider_5
Text GLabel 4470 18700 3    50   Input ~ 0
LEDslider_4
Text GLabel 3760 18700 3    50   Input ~ 0
LEDslider_3
Text GLabel 3160 18700 3    50   Input ~ 0
LEDslider_2
Text GLabel 2450 18700 3    50   Input ~ 0
LEDslider_1
Connection ~ 8360 15780
Wire Wire Line
	8360 15780 8960 15780
Connection ~ 3760 15780
Connection ~ 3160 15780
Wire Wire Line
	3160 15780 3760 15780
Connection ~ 6370 15780
Connection ~ 5770 15780
Wire Wire Line
	5770 15780 6370 15780
Connection ~ 7650 18300
Connection ~ 7080 18300
Wire Wire Line
	7080 18300 7650 18300
Connection ~ 7650 17800
Connection ~ 7080 17800
Wire Wire Line
	7080 17800 7650 17800
Connection ~ 7650 17300
Connection ~ 7080 17300
Wire Wire Line
	7080 17300 7650 17300
Connection ~ 7650 16800
Connection ~ 7080 16800
Wire Wire Line
	7080 16800 7650 16800
Connection ~ 5060 12780
Connection ~ 4470 12780
Wire Wire Line
	4470 12780 5060 12780
Connection ~ 5060 13780
Connection ~ 4470 13780
Wire Wire Line
	4470 13780 5060 13780
Connection ~ 5060 14280
Connection ~ 4470 14280
Wire Wire Line
	4470 14280 5060 14280
Connection ~ 5060 14780
Connection ~ 4470 14780
Wire Wire Line
	4470 14780 5060 14780
Connection ~ 5060 15280
Connection ~ 4470 15280
Wire Wire Line
	4470 15280 5060 15280
Connection ~ 5060 15780
Connection ~ 4470 15780
Wire Wire Line
	4470 15780 5060 15780
Connection ~ 5060 16300
Connection ~ 4470 16300
Wire Wire Line
	4470 16300 5060 16300
Connection ~ 5060 16800
Connection ~ 4470 16800
Wire Wire Line
	4470 16800 5060 16800
Connection ~ 5060 17300
Connection ~ 4470 17300
Wire Wire Line
	4470 17300 5060 17300
Connection ~ 5060 17800
Connection ~ 4470 17800
Wire Wire Line
	4470 17800 5060 17800
Connection ~ 5060 18300
Connection ~ 4470 18300
Wire Wire Line
	4470 18300 5060 18300
Connection ~ 7650 16300
Connection ~ 7080 16300
Wire Wire Line
	7080 16300 7650 16300
Connection ~ 7650 15780
Connection ~ 7080 15780
Wire Wire Line
	7080 15780 7650 15780
Connection ~ 7650 15280
Connection ~ 7080 15280
Wire Wire Line
	7080 15280 7650 15280
Connection ~ 7650 14780
Connection ~ 7080 14780
Wire Wire Line
	7080 14780 7650 14780
Connection ~ 7650 14280
Connection ~ 7080 14280
Wire Wire Line
	7080 14280 7650 14280
Connection ~ 7650 13780
Connection ~ 7080 13780
Wire Wire Line
	7080 13780 7650 13780
Connection ~ 7650 13280
Connection ~ 7080 13280
Wire Wire Line
	7080 13280 7650 13280
Connection ~ 7650 12780
Connection ~ 7080 12780
Wire Wire Line
	7080 12780 7650 12780
Wire Wire Line
	5770 13140 5770 13080
Wire Wire Line
	5770 13630 5770 13580
Wire Wire Line
	5770 14150 5770 14080
Wire Wire Line
	5770 14650 5770 14580
Wire Wire Line
	5770 15080 5770 15140
Wire Wire Line
	5770 12780 5060 12780
Wire Wire Line
	5770 13780 5060 13780
Wire Wire Line
	5770 14780 5060 14780
Wire Wire Line
	5770 15780 5060 15780
Wire Wire Line
	5060 15280 5770 15280
Wire Wire Line
	5060 14280 5770 14280
Wire Wire Line
	5060 13280 5770 13280
Wire Wire Line
	5570 15650 5570 15140
Connection ~ 5570 15650
Wire Wire Line
	5570 15650 5770 15650
Wire Wire Line
	5570 15140 5570 14650
Connection ~ 5570 15140
Wire Wire Line
	5570 15140 5770 15140
Wire Wire Line
	5570 14650 5570 14150
Connection ~ 5570 14650
Wire Wire Line
	5570 14650 5770 14650
Wire Wire Line
	5570 14150 5570 13630
Connection ~ 5570 14150
Wire Wire Line
	5570 14150 5770 14150
Wire Wire Line
	5570 13630 5570 13140
Connection ~ 5570 13630
Wire Wire Line
	5570 13630 5770 13630
Wire Wire Line
	5570 13140 5770 13140
Wire Wire Line
	5770 16180 5570 16180
Wire Wire Line
	4870 13640 4870 13140
Connection ~ 4870 13640
Wire Wire Line
	5060 13640 5060 13580
Wire Wire Line
	4870 13640 5060 13640
Wire Wire Line
	4870 14140 4870 13640
Connection ~ 4870 14140
Wire Wire Line
	5060 14140 5060 14080
Wire Wire Line
	4870 14140 5060 14140
Wire Wire Line
	5060 13140 5060 13080
Wire Wire Line
	4870 13140 5060 13140
Connection ~ 4870 14650
Wire Wire Line
	4870 14650 4870 14140
Connection ~ 4870 15140
Wire Wire Line
	5060 14650 5060 14580
Wire Wire Line
	4870 14650 5060 14650
Wire Wire Line
	4870 15140 4870 14650
Connection ~ 4870 15650
Wire Wire Line
	5060 15140 5060 15080
Wire Wire Line
	4870 15140 5060 15140
Wire Wire Line
	4870 15650 4870 15140
Wire Wire Line
	5060 15650 5060 15580
Wire Wire Line
	4870 15650 5060 15650
Wire Wire Line
	4870 16180 4870 15650
Wire Wire Line
	5060 16180 4870 16180
Wire Wire Line
	5060 16080 5060 16180
$Comp
L Device:LED D67
U 1 1 6D247108
P 5770 15930
F 0 "D67" H 5763 16146 50  0000 C CNN
F 1 "LED" H 5763 16055 50  0000 C CNN
F 2 "" H 5770 15930 50  0001 C CNN
F 3 "~" H 5770 15930 50  0001 C CNN
	1    5770 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D55
U 1 1 6D24710E
P 5060 15930
F 0 "D55" H 5053 16146 50  0000 C CNN
F 1 "LED" H 5053 16055 50  0000 C CNN
F 2 "" H 5060 15930 50  0001 C CNN
F 3 "~" H 5060 15930 50  0001 C CNN
	1    5060 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D66
U 1 1 6D247114
P 5770 15430
F 0 "D66" H 5763 15646 50  0000 C CNN
F 1 "LED" H 5763 15555 50  0000 C CNN
F 2 "" H 5770 15430 50  0001 C CNN
F 3 "~" H 5770 15430 50  0001 C CNN
	1    5770 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D54
U 1 1 6D24711A
P 5060 15430
F 0 "D54" H 5053 15646 50  0000 C CNN
F 1 "LED" H 5053 15555 50  0000 C CNN
F 2 "" H 5060 15430 50  0001 C CNN
F 3 "~" H 5060 15430 50  0001 C CNN
	1    5060 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D65
U 1 1 6D247120
P 5770 14930
F 0 "D65" H 5763 15146 50  0000 C CNN
F 1 "LED" H 5763 15055 50  0000 C CNN
F 2 "" H 5770 14930 50  0001 C CNN
F 3 "~" H 5770 14930 50  0001 C CNN
	1    5770 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D53
U 1 1 6D247126
P 5060 14930
F 0 "D53" H 5053 15146 50  0000 C CNN
F 1 "LED" H 5053 15055 50  0000 C CNN
F 2 "" H 5060 14930 50  0001 C CNN
F 3 "~" H 5060 14930 50  0001 C CNN
	1    5060 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D64
U 1 1 6D24712C
P 5770 14430
F 0 "D64" H 5763 14646 50  0000 C CNN
F 1 "LED" H 5763 14555 50  0000 C CNN
F 2 "" H 5770 14430 50  0001 C CNN
F 3 "~" H 5770 14430 50  0001 C CNN
	1    5770 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D52
U 1 1 6D247132
P 5060 14430
F 0 "D52" H 5053 14646 50  0000 C CNN
F 1 "LED" H 5053 14555 50  0000 C CNN
F 2 "" H 5060 14430 50  0001 C CNN
F 3 "~" H 5060 14430 50  0001 C CNN
	1    5060 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D63
U 1 1 6D247138
P 5770 13930
F 0 "D63" H 5763 14146 50  0000 C CNN
F 1 "LED" H 5763 14055 50  0000 C CNN
F 2 "" H 5770 13930 50  0001 C CNN
F 3 "~" H 5770 13930 50  0001 C CNN
	1    5770 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D51
U 1 1 6D24713E
P 5060 13930
F 0 "D51" H 5053 14146 50  0000 C CNN
F 1 "LED" H 5053 14055 50  0000 C CNN
F 2 "" H 5060 13930 50  0001 C CNN
F 3 "~" H 5060 13930 50  0001 C CNN
	1    5060 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D62
U 1 1 6D247144
P 5770 13430
F 0 "D62" H 5763 13646 50  0000 C CNN
F 1 "LED" H 5763 13555 50  0000 C CNN
F 2 "" H 5770 13430 50  0001 C CNN
F 3 "~" H 5770 13430 50  0001 C CNN
	1    5770 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D50
U 1 1 6D24714A
P 5060 13430
F 0 "D50" H 5053 13646 50  0000 C CNN
F 1 "LED" H 5053 13555 50  0000 C CNN
F 2 "" H 5060 13430 50  0001 C CNN
F 3 "~" H 5060 13430 50  0001 C CNN
	1    5060 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D61
U 1 1 6D247150
P 5770 12930
F 0 "D61" H 5763 13146 50  0000 C CNN
F 1 "LED" H 5763 13055 50  0000 C CNN
F 2 "" H 5770 12930 50  0001 C CNN
F 3 "~" H 5770 12930 50  0001 C CNN
	1    5770 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D49
U 1 1 6D247156
P 5060 12930
F 0 "D49" H 5053 13146 50  0000 C CNN
F 1 "LED" H 5053 13055 50  0000 C CNN
F 2 "" H 5060 12930 50  0001 C CNN
F 3 "~" H 5060 12930 50  0001 C CNN
	1    5060 12930
	0    1    -1   0   
$EndComp
Wire Wire Line
	5770 16180 5770 16080
Wire Wire Line
	5770 15650 5770 15580
Wire Wire Line
	5570 16180 5570 15650
$Comp
L Device:LED D56
U 1 1 6D24716B
P 5060 16450
F 0 "D56" H 5053 16666 50  0000 C CNN
F 1 "LED" H 5053 16575 50  0000 C CNN
F 2 "" H 5060 16450 50  0001 C CNN
F 3 "~" H 5060 16450 50  0001 C CNN
	1    5060 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D68
U 1 1 6D247171
P 5770 16450
F 0 "D68" H 5763 16666 50  0000 C CNN
F 1 "LED" H 5763 16575 50  0000 C CNN
F 2 "" H 5770 16450 50  0001 C CNN
F 3 "~" H 5770 16450 50  0001 C CNN
	1    5770 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D57
U 1 1 6D247177
P 5060 16950
F 0 "D57" H 5053 17166 50  0000 C CNN
F 1 "LED" H 5053 17075 50  0000 C CNN
F 2 "" H 5060 16950 50  0001 C CNN
F 3 "~" H 5060 16950 50  0001 C CNN
	1    5060 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D69
U 1 1 6D24717D
P 5770 16950
F 0 "D69" H 5763 17166 50  0000 C CNN
F 1 "LED" H 5763 17075 50  0000 C CNN
F 2 "" H 5770 16950 50  0001 C CNN
F 3 "~" H 5770 16950 50  0001 C CNN
	1    5770 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D58
U 1 1 6D247183
P 5060 17450
F 0 "D58" H 5053 17666 50  0000 C CNN
F 1 "LED" H 5053 17575 50  0000 C CNN
F 2 "" H 5060 17450 50  0001 C CNN
F 3 "~" H 5060 17450 50  0001 C CNN
	1    5060 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D70
U 1 1 6D247189
P 5770 17450
F 0 "D70" H 5763 17666 50  0000 C CNN
F 1 "LED" H 5763 17575 50  0000 C CNN
F 2 "" H 5770 17450 50  0001 C CNN
F 3 "~" H 5770 17450 50  0001 C CNN
	1    5770 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D59
U 1 1 6D24718F
P 5060 17950
F 0 "D59" H 5053 18166 50  0000 C CNN
F 1 "LED" H 5053 18075 50  0000 C CNN
F 2 "" H 5060 17950 50  0001 C CNN
F 3 "~" H 5060 17950 50  0001 C CNN
	1    5060 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D71
U 1 1 6D247195
P 5770 17950
F 0 "D71" H 5763 18166 50  0000 C CNN
F 1 "LED" H 5763 18075 50  0000 C CNN
F 2 "" H 5770 17950 50  0001 C CNN
F 3 "~" H 5770 17950 50  0001 C CNN
	1    5770 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D72
U 1 1 6D24719B
P 5770 18450
F 0 "D72" H 5763 18666 50  0000 C CNN
F 1 "LED" H 5763 18575 50  0000 C CNN
F 2 "" H 5770 18450 50  0001 C CNN
F 3 "~" H 5770 18450 50  0001 C CNN
	1    5770 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	5060 18600 5060 18700
Wire Wire Line
	5060 18700 4870 18700
Wire Wire Line
	4870 18700 4870 18170
Wire Wire Line
	4870 18170 5060 18170
Wire Wire Line
	5060 18170 5060 18100
Wire Wire Line
	4870 18170 4870 17660
Wire Wire Line
	4870 17660 5060 17660
Wire Wire Line
	5060 17660 5060 17600
Connection ~ 4870 18170
Wire Wire Line
	4870 17660 4870 17170
Wire Wire Line
	4870 17170 5060 17170
Wire Wire Line
	5060 17170 5060 17100
Connection ~ 4870 17660
Wire Wire Line
	4870 17170 4870 16660
Connection ~ 4870 17170
Wire Wire Line
	4870 16660 5060 16660
Wire Wire Line
	5060 16660 5060 16600
Wire Wire Line
	5770 18700 5570 18700
Wire Wire Line
	5570 18700 5570 18170
Wire Wire Line
	5570 16670 5770 16670
Wire Wire Line
	5570 17170 5770 17170
Connection ~ 5570 17170
Wire Wire Line
	5570 17170 5570 16670
Wire Wire Line
	5570 17660 5770 17660
Connection ~ 5570 17660
Wire Wire Line
	5570 17660 5570 17170
Wire Wire Line
	5570 18170 5770 18170
Connection ~ 5570 18170
Wire Wire Line
	5570 18170 5570 17660
Wire Wire Line
	5060 16800 5770 16800
Wire Wire Line
	5060 17800 5770 17800
Wire Wire Line
	5770 17300 5060 17300
Wire Wire Line
	5770 17600 5770 17660
Wire Wire Line
	5770 17170 5770 17100
Wire Wire Line
	5770 16670 5770 16600
Wire Wire Line
	5770 18700 5770 18600
Wire Wire Line
	5770 18170 5770 18100
Wire Wire Line
	5060 16300 5770 16300
Connection ~ 4870 16660
Connection ~ 5570 16670
Wire Wire Line
	5570 16180 5570 16670
Connection ~ 5570 16180
Wire Wire Line
	4870 16180 4870 16660
Connection ~ 4870 16180
Wire Wire Line
	5770 18300 5060 18300
$Comp
L Device:LED D60
U 1 1 6D2471CE
P 5060 18450
F 0 "D60" H 5053 18666 50  0000 C CNN
F 1 "LED" H 5053 18575 50  0000 C CNN
F 2 "" H 5060 18450 50  0001 C CNN
F 3 "~" H 5060 18450 50  0001 C CNN
	1    5060 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	7080 13140 7080 13080
Wire Wire Line
	7080 13630 7080 13580
Wire Wire Line
	7080 14150 7080 14080
Wire Wire Line
	7080 14650 7080 14580
Wire Wire Line
	7080 15080 7080 15140
Wire Wire Line
	7080 12780 6370 12780
Wire Wire Line
	7080 13780 6370 13780
Wire Wire Line
	7080 14780 6370 14780
Wire Wire Line
	7080 15780 6370 15780
Wire Wire Line
	6370 15280 7080 15280
Wire Wire Line
	6370 14280 7080 14280
Wire Wire Line
	6370 13280 7080 13280
Wire Wire Line
	6880 15650 6880 15140
Connection ~ 6880 15650
Wire Wire Line
	6880 15650 7080 15650
Wire Wire Line
	6880 15140 6880 14650
Connection ~ 6880 15140
Wire Wire Line
	6880 15140 7080 15140
Wire Wire Line
	6880 14650 6880 14150
Connection ~ 6880 14650
Wire Wire Line
	6880 14650 7080 14650
Wire Wire Line
	6880 14150 6880 13630
Connection ~ 6880 14150
Wire Wire Line
	6880 14150 7080 14150
Wire Wire Line
	6880 13630 6880 13140
Connection ~ 6880 13630
Wire Wire Line
	6880 13630 7080 13630
Wire Wire Line
	6880 13140 7080 13140
Wire Wire Line
	7080 16180 6880 16180
Wire Wire Line
	6180 13640 6180 13140
Connection ~ 6180 13640
Wire Wire Line
	6370 13640 6370 13580
Wire Wire Line
	6180 13640 6370 13640
Wire Wire Line
	6180 14140 6180 13640
Connection ~ 6180 14140
Wire Wire Line
	6370 14140 6370 14080
Wire Wire Line
	6180 14140 6370 14140
Wire Wire Line
	6370 13140 6370 13080
Wire Wire Line
	6180 13140 6370 13140
Connection ~ 6180 14650
Wire Wire Line
	6180 14650 6180 14140
Connection ~ 6180 15140
Wire Wire Line
	6370 14650 6370 14580
Wire Wire Line
	6180 14650 6370 14650
Wire Wire Line
	6180 15140 6180 14650
Connection ~ 6180 15650
Wire Wire Line
	6370 15140 6370 15080
Wire Wire Line
	6180 15140 6370 15140
Wire Wire Line
	6180 15650 6180 15140
Wire Wire Line
	6370 15650 6370 15580
Wire Wire Line
	6180 15650 6370 15650
Wire Wire Line
	6180 16180 6180 15650
Wire Wire Line
	6370 16180 6180 16180
Wire Wire Line
	6370 16080 6370 16180
$Comp
L Device:LED D91
U 1 1 6D247213
P 7080 15930
F 0 "D91" H 7073 16146 50  0000 C CNN
F 1 "LED" H 7073 16055 50  0000 C CNN
F 2 "" H 7080 15930 50  0001 C CNN
F 3 "~" H 7080 15930 50  0001 C CNN
	1    7080 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D79
U 1 1 6D247219
P 6370 15930
F 0 "D79" H 6363 16146 50  0000 C CNN
F 1 "LED" H 6363 16055 50  0000 C CNN
F 2 "" H 6370 15930 50  0001 C CNN
F 3 "~" H 6370 15930 50  0001 C CNN
	1    6370 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D90
U 1 1 6D24721F
P 7080 15430
F 0 "D90" H 7073 15646 50  0000 C CNN
F 1 "LED" H 7073 15555 50  0000 C CNN
F 2 "" H 7080 15430 50  0001 C CNN
F 3 "~" H 7080 15430 50  0001 C CNN
	1    7080 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D78
U 1 1 6D247225
P 6370 15430
F 0 "D78" H 6363 15646 50  0000 C CNN
F 1 "LED" H 6363 15555 50  0000 C CNN
F 2 "" H 6370 15430 50  0001 C CNN
F 3 "~" H 6370 15430 50  0001 C CNN
	1    6370 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D89
U 1 1 6D24722B
P 7080 14930
F 0 "D89" H 7073 15146 50  0000 C CNN
F 1 "LED" H 7073 15055 50  0000 C CNN
F 2 "" H 7080 14930 50  0001 C CNN
F 3 "~" H 7080 14930 50  0001 C CNN
	1    7080 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D77
U 1 1 6D247231
P 6370 14930
F 0 "D77" H 6363 15146 50  0000 C CNN
F 1 "LED" H 6363 15055 50  0000 C CNN
F 2 "" H 6370 14930 50  0001 C CNN
F 3 "~" H 6370 14930 50  0001 C CNN
	1    6370 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D88
U 1 1 6D247237
P 7080 14430
F 0 "D88" H 7073 14646 50  0000 C CNN
F 1 "LED" H 7073 14555 50  0000 C CNN
F 2 "" H 7080 14430 50  0001 C CNN
F 3 "~" H 7080 14430 50  0001 C CNN
	1    7080 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D76
U 1 1 6D24723D
P 6370 14430
F 0 "D76" H 6363 14646 50  0000 C CNN
F 1 "LED" H 6363 14555 50  0000 C CNN
F 2 "" H 6370 14430 50  0001 C CNN
F 3 "~" H 6370 14430 50  0001 C CNN
	1    6370 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D87
U 1 1 6D247243
P 7080 13930
F 0 "D87" H 7073 14146 50  0000 C CNN
F 1 "LED" H 7073 14055 50  0000 C CNN
F 2 "" H 7080 13930 50  0001 C CNN
F 3 "~" H 7080 13930 50  0001 C CNN
	1    7080 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D75
U 1 1 6D247249
P 6370 13930
F 0 "D75" H 6363 14146 50  0000 C CNN
F 1 "LED" H 6363 14055 50  0000 C CNN
F 2 "" H 6370 13930 50  0001 C CNN
F 3 "~" H 6370 13930 50  0001 C CNN
	1    6370 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D86
U 1 1 6D24724F
P 7080 13430
F 0 "D86" H 7073 13646 50  0000 C CNN
F 1 "LED" H 7073 13555 50  0000 C CNN
F 2 "" H 7080 13430 50  0001 C CNN
F 3 "~" H 7080 13430 50  0001 C CNN
	1    7080 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D74
U 1 1 6D247255
P 6370 13430
F 0 "D74" H 6363 13646 50  0000 C CNN
F 1 "LED" H 6363 13555 50  0000 C CNN
F 2 "" H 6370 13430 50  0001 C CNN
F 3 "~" H 6370 13430 50  0001 C CNN
	1    6370 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D85
U 1 1 6D24725B
P 7080 12930
F 0 "D85" H 7073 13146 50  0000 C CNN
F 1 "LED" H 7073 13055 50  0000 C CNN
F 2 "" H 7080 12930 50  0001 C CNN
F 3 "~" H 7080 12930 50  0001 C CNN
	1    7080 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D73
U 1 1 6D247261
P 6370 12930
F 0 "D73" H 6363 13146 50  0000 C CNN
F 1 "LED" H 6363 13055 50  0000 C CNN
F 2 "" H 6370 12930 50  0001 C CNN
F 3 "~" H 6370 12930 50  0001 C CNN
	1    6370 12930
	0    1    -1   0   
$EndComp
Wire Wire Line
	7080 16180 7080 16080
Wire Wire Line
	7080 15650 7080 15580
Wire Wire Line
	6880 16180 6880 15650
$Comp
L Device:LED D80
U 1 1 6D247276
P 6370 16450
F 0 "D80" H 6363 16666 50  0000 C CNN
F 1 "LED" H 6363 16575 50  0000 C CNN
F 2 "" H 6370 16450 50  0001 C CNN
F 3 "~" H 6370 16450 50  0001 C CNN
	1    6370 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D92
U 1 1 6D24727C
P 7080 16450
F 0 "D92" H 7073 16666 50  0000 C CNN
F 1 "LED" H 7073 16575 50  0000 C CNN
F 2 "" H 7080 16450 50  0001 C CNN
F 3 "~" H 7080 16450 50  0001 C CNN
	1    7080 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D81
U 1 1 6D247282
P 6370 16950
F 0 "D81" H 6363 17166 50  0000 C CNN
F 1 "LED" H 6363 17075 50  0000 C CNN
F 2 "" H 6370 16950 50  0001 C CNN
F 3 "~" H 6370 16950 50  0001 C CNN
	1    6370 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D93
U 1 1 6D247288
P 7080 16950
F 0 "D93" H 7073 17166 50  0000 C CNN
F 1 "LED" H 7073 17075 50  0000 C CNN
F 2 "" H 7080 16950 50  0001 C CNN
F 3 "~" H 7080 16950 50  0001 C CNN
	1    7080 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D82
U 1 1 6D24728E
P 6370 17450
F 0 "D82" H 6363 17666 50  0000 C CNN
F 1 "LED" H 6363 17575 50  0000 C CNN
F 2 "" H 6370 17450 50  0001 C CNN
F 3 "~" H 6370 17450 50  0001 C CNN
	1    6370 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D94
U 1 1 6D247294
P 7080 17450
F 0 "D94" H 7073 17666 50  0000 C CNN
F 1 "LED" H 7073 17575 50  0000 C CNN
F 2 "" H 7080 17450 50  0001 C CNN
F 3 "~" H 7080 17450 50  0001 C CNN
	1    7080 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D83
U 1 1 6D24729A
P 6370 17950
F 0 "D83" H 6363 18166 50  0000 C CNN
F 1 "LED" H 6363 18075 50  0000 C CNN
F 2 "" H 6370 17950 50  0001 C CNN
F 3 "~" H 6370 17950 50  0001 C CNN
	1    6370 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D95
U 1 1 6D2472A0
P 7080 17950
F 0 "D95" H 7073 18166 50  0000 C CNN
F 1 "LED" H 7073 18075 50  0000 C CNN
F 2 "" H 7080 17950 50  0001 C CNN
F 3 "~" H 7080 17950 50  0001 C CNN
	1    7080 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D96
U 1 1 6D2472A6
P 7080 18450
F 0 "D96" H 7073 18666 50  0000 C CNN
F 1 "LED" H 7073 18575 50  0000 C CNN
F 2 "" H 7080 18450 50  0001 C CNN
F 3 "~" H 7080 18450 50  0001 C CNN
	1    7080 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	6370 18600 6370 18700
Wire Wire Line
	6370 18700 6180 18700
Wire Wire Line
	6180 18700 6180 18170
Wire Wire Line
	6180 18170 6370 18170
Wire Wire Line
	6370 18170 6370 18100
Wire Wire Line
	6180 18170 6180 17660
Wire Wire Line
	6180 17660 6370 17660
Wire Wire Line
	6370 17660 6370 17600
Connection ~ 6180 18170
Wire Wire Line
	6180 17660 6180 17170
Wire Wire Line
	6180 17170 6370 17170
Wire Wire Line
	6370 17170 6370 17100
Connection ~ 6180 17660
Wire Wire Line
	6180 17170 6180 16660
Connection ~ 6180 17170
Wire Wire Line
	6180 16660 6370 16660
Wire Wire Line
	6370 16660 6370 16600
Wire Wire Line
	7080 18700 6880 18700
Wire Wire Line
	6880 18700 6880 18170
Wire Wire Line
	6880 16670 7080 16670
Wire Wire Line
	6880 17170 7080 17170
Connection ~ 6880 17170
Wire Wire Line
	6880 17170 6880 16670
Wire Wire Line
	6880 17660 7080 17660
Connection ~ 6880 17660
Wire Wire Line
	6880 17660 6880 17170
Wire Wire Line
	6880 18170 7080 18170
Connection ~ 6880 18170
Wire Wire Line
	6880 18170 6880 17660
Wire Wire Line
	6370 16800 7080 16800
Wire Wire Line
	6370 17800 7080 17800
Wire Wire Line
	7080 17300 6370 17300
Wire Wire Line
	7080 17600 7080 17660
Wire Wire Line
	7080 17170 7080 17100
Wire Wire Line
	7080 16670 7080 16600
Wire Wire Line
	7080 18700 7080 18600
Wire Wire Line
	7080 18170 7080 18100
Wire Wire Line
	6370 16300 7080 16300
Connection ~ 6180 16660
Connection ~ 6880 16670
Wire Wire Line
	6880 16180 6880 16670
Connection ~ 6880 16180
Wire Wire Line
	6180 16180 6180 16660
Connection ~ 6180 16180
Wire Wire Line
	7080 18300 6370 18300
$Comp
L Device:LED D84
U 1 1 6D2472D9
P 6370 18450
F 0 "D84" H 6363 18666 50  0000 C CNN
F 1 "LED" H 6363 18575 50  0000 C CNN
F 2 "" H 6370 18450 50  0001 C CNN
F 3 "~" H 6370 18450 50  0001 C CNN
	1    6370 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	5770 12780 6370 12780
Connection ~ 5770 12780
Connection ~ 6370 12780
Wire Wire Line
	5770 13280 6370 13280
Connection ~ 5770 13280
Connection ~ 6370 13280
Wire Wire Line
	5770 13780 6370 13780
Connection ~ 5770 13780
Connection ~ 6370 13780
Wire Wire Line
	5770 14280 6370 14280
Connection ~ 5770 14280
Connection ~ 6370 14280
Wire Wire Line
	5770 14780 6370 14780
Connection ~ 5770 14780
Connection ~ 6370 14780
Wire Wire Line
	5770 18300 6370 18300
Connection ~ 5770 18300
Connection ~ 6370 18300
Wire Wire Line
	5770 17800 6370 17800
Connection ~ 5770 17800
Connection ~ 6370 17800
Wire Wire Line
	5770 17300 6370 17300
Connection ~ 5770 17300
Connection ~ 6370 17300
Wire Wire Line
	5770 16800 6370 16800
Connection ~ 5770 16800
Connection ~ 6370 16800
Wire Wire Line
	5770 16300 6370 16300
Connection ~ 5770 16300
Connection ~ 6370 16300
Wire Wire Line
	5770 15280 6370 15280
Connection ~ 5770 15280
Connection ~ 6370 15280
Wire Wire Line
	8360 13140 8360 13080
Wire Wire Line
	8360 13630 8360 13580
Wire Wire Line
	8360 14150 8360 14080
Wire Wire Line
	8360 14650 8360 14580
Wire Wire Line
	8360 15080 8360 15140
Wire Wire Line
	8360 12780 7650 12780
Wire Wire Line
	8360 13780 7650 13780
Wire Wire Line
	8360 14780 7650 14780
Wire Wire Line
	8360 15780 7650 15780
Wire Wire Line
	7650 15280 8360 15280
Wire Wire Line
	7650 14280 8360 14280
Wire Wire Line
	7650 13280 8360 13280
Wire Wire Line
	8160 15650 8160 15140
Connection ~ 8160 15650
Wire Wire Line
	8160 15650 8360 15650
Wire Wire Line
	8160 15140 8160 14650
Connection ~ 8160 15140
Wire Wire Line
	8160 15140 8360 15140
Wire Wire Line
	8160 14650 8160 14150
Connection ~ 8160 14650
Wire Wire Line
	8160 14650 8360 14650
Wire Wire Line
	8160 14150 8160 13630
Connection ~ 8160 14150
Wire Wire Line
	8160 14150 8360 14150
Wire Wire Line
	8160 13630 8160 13140
Connection ~ 8160 13630
Wire Wire Line
	8160 13630 8360 13630
Wire Wire Line
	8160 13140 8360 13140
Wire Wire Line
	8360 16180 8160 16180
Wire Wire Line
	7460 13640 7460 13140
Connection ~ 7460 13640
Wire Wire Line
	7650 13640 7650 13580
Wire Wire Line
	7460 13640 7650 13640
Wire Wire Line
	7460 14140 7460 13640
Connection ~ 7460 14140
Wire Wire Line
	7650 14140 7650 14080
Wire Wire Line
	7460 14140 7650 14140
Wire Wire Line
	7650 13140 7650 13080
Wire Wire Line
	7460 13140 7650 13140
Connection ~ 7460 14650
Wire Wire Line
	7460 14650 7460 14140
Connection ~ 7460 15140
Wire Wire Line
	7650 14650 7650 14580
Wire Wire Line
	7460 14650 7650 14650
Wire Wire Line
	7460 15140 7460 14650
Connection ~ 7460 15650
Wire Wire Line
	7650 15140 7650 15080
Wire Wire Line
	7460 15140 7650 15140
Wire Wire Line
	7460 15650 7460 15140
Wire Wire Line
	7650 15650 7650 15580
Wire Wire Line
	7460 15650 7650 15650
Wire Wire Line
	7460 16180 7460 15650
Wire Wire Line
	7650 16180 7460 16180
Wire Wire Line
	7650 16080 7650 16180
$Comp
L Device:LED D115
U 1 1 6D395DBB
P 8360 15930
F 0 "D115" H 8353 16146 50  0000 C CNN
F 1 "LED" H 8353 16055 50  0000 C CNN
F 2 "" H 8360 15930 50  0001 C CNN
F 3 "~" H 8360 15930 50  0001 C CNN
	1    8360 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D103
U 1 1 6D395DC1
P 7650 15930
F 0 "D103" H 7643 16146 50  0000 C CNN
F 1 "LED" H 7643 16055 50  0000 C CNN
F 2 "" H 7650 15930 50  0001 C CNN
F 3 "~" H 7650 15930 50  0001 C CNN
	1    7650 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D114
U 1 1 6D395DC7
P 8360 15430
F 0 "D114" H 8353 15646 50  0000 C CNN
F 1 "LED" H 8353 15555 50  0000 C CNN
F 2 "" H 8360 15430 50  0001 C CNN
F 3 "~" H 8360 15430 50  0001 C CNN
	1    8360 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D102
U 1 1 6D395DCD
P 7650 15430
F 0 "D102" H 7643 15646 50  0000 C CNN
F 1 "LED" H 7643 15555 50  0000 C CNN
F 2 "" H 7650 15430 50  0001 C CNN
F 3 "~" H 7650 15430 50  0001 C CNN
	1    7650 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D113
U 1 1 6D395DD3
P 8360 14930
F 0 "D113" H 8353 15146 50  0000 C CNN
F 1 "LED" H 8353 15055 50  0000 C CNN
F 2 "" H 8360 14930 50  0001 C CNN
F 3 "~" H 8360 14930 50  0001 C CNN
	1    8360 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D101
U 1 1 6D395DD9
P 7650 14930
F 0 "D101" H 7643 15146 50  0000 C CNN
F 1 "LED" H 7643 15055 50  0000 C CNN
F 2 "" H 7650 14930 50  0001 C CNN
F 3 "~" H 7650 14930 50  0001 C CNN
	1    7650 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D112
U 1 1 6D395DDF
P 8360 14430
F 0 "D112" H 8353 14646 50  0000 C CNN
F 1 "LED" H 8353 14555 50  0000 C CNN
F 2 "" H 8360 14430 50  0001 C CNN
F 3 "~" H 8360 14430 50  0001 C CNN
	1    8360 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D100
U 1 1 6D395DE5
P 7650 14430
F 0 "D100" H 7643 14646 50  0000 C CNN
F 1 "LED" H 7643 14555 50  0000 C CNN
F 2 "" H 7650 14430 50  0001 C CNN
F 3 "~" H 7650 14430 50  0001 C CNN
	1    7650 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D111
U 1 1 6D395DEB
P 8360 13930
F 0 "D111" H 8353 14146 50  0000 C CNN
F 1 "LED" H 8353 14055 50  0000 C CNN
F 2 "" H 8360 13930 50  0001 C CNN
F 3 "~" H 8360 13930 50  0001 C CNN
	1    8360 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D99
U 1 1 6D395DF1
P 7650 13930
F 0 "D99" H 7643 14146 50  0000 C CNN
F 1 "LED" H 7643 14055 50  0000 C CNN
F 2 "" H 7650 13930 50  0001 C CNN
F 3 "~" H 7650 13930 50  0001 C CNN
	1    7650 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D110
U 1 1 6D395DF7
P 8360 13430
F 0 "D110" H 8353 13646 50  0000 C CNN
F 1 "LED" H 8353 13555 50  0000 C CNN
F 2 "" H 8360 13430 50  0001 C CNN
F 3 "~" H 8360 13430 50  0001 C CNN
	1    8360 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D98
U 1 1 6D395DFD
P 7650 13430
F 0 "D98" H 7643 13646 50  0000 C CNN
F 1 "LED" H 7643 13555 50  0000 C CNN
F 2 "" H 7650 13430 50  0001 C CNN
F 3 "~" H 7650 13430 50  0001 C CNN
	1    7650 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D109
U 1 1 6D395E03
P 8360 12930
F 0 "D109" H 8353 13146 50  0000 C CNN
F 1 "LED" H 8353 13055 50  0000 C CNN
F 2 "" H 8360 12930 50  0001 C CNN
F 3 "~" H 8360 12930 50  0001 C CNN
	1    8360 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D97
U 1 1 6D395E09
P 7650 12930
F 0 "D97" H 7643 13146 50  0000 C CNN
F 1 "LED" H 7643 13055 50  0000 C CNN
F 2 "" H 7650 12930 50  0001 C CNN
F 3 "~" H 7650 12930 50  0001 C CNN
	1    7650 12930
	0    1    -1   0   
$EndComp
Wire Wire Line
	8360 16180 8360 16080
Wire Wire Line
	8360 15650 8360 15580
Wire Wire Line
	8160 16180 8160 15650
$Comp
L Device:LED D104
U 1 1 6D395E1E
P 7650 16450
F 0 "D104" H 7643 16666 50  0000 C CNN
F 1 "LED" H 7643 16575 50  0000 C CNN
F 2 "" H 7650 16450 50  0001 C CNN
F 3 "~" H 7650 16450 50  0001 C CNN
	1    7650 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D116
U 1 1 6D395E24
P 8360 16450
F 0 "D116" H 8353 16666 50  0000 C CNN
F 1 "LED" H 8353 16575 50  0000 C CNN
F 2 "" H 8360 16450 50  0001 C CNN
F 3 "~" H 8360 16450 50  0001 C CNN
	1    8360 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D105
U 1 1 6D395E2A
P 7650 16950
F 0 "D105" H 7643 17166 50  0000 C CNN
F 1 "LED" H 7643 17075 50  0000 C CNN
F 2 "" H 7650 16950 50  0001 C CNN
F 3 "~" H 7650 16950 50  0001 C CNN
	1    7650 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D117
U 1 1 6D395E30
P 8360 16950
F 0 "D117" H 8353 17166 50  0000 C CNN
F 1 "LED" H 8353 17075 50  0000 C CNN
F 2 "" H 8360 16950 50  0001 C CNN
F 3 "~" H 8360 16950 50  0001 C CNN
	1    8360 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D106
U 1 1 6D395E36
P 7650 17450
F 0 "D106" H 7643 17666 50  0000 C CNN
F 1 "LED" H 7643 17575 50  0000 C CNN
F 2 "" H 7650 17450 50  0001 C CNN
F 3 "~" H 7650 17450 50  0001 C CNN
	1    7650 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D118
U 1 1 6D395E3C
P 8360 17450
F 0 "D118" H 8353 17666 50  0000 C CNN
F 1 "LED" H 8353 17575 50  0000 C CNN
F 2 "" H 8360 17450 50  0001 C CNN
F 3 "~" H 8360 17450 50  0001 C CNN
	1    8360 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D107
U 1 1 6D395E42
P 7650 17950
F 0 "D107" H 7643 18166 50  0000 C CNN
F 1 "LED" H 7643 18075 50  0000 C CNN
F 2 "" H 7650 17950 50  0001 C CNN
F 3 "~" H 7650 17950 50  0001 C CNN
	1    7650 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D119
U 1 1 6D395E48
P 8360 17950
F 0 "D119" H 8353 18166 50  0000 C CNN
F 1 "LED" H 8353 18075 50  0000 C CNN
F 2 "" H 8360 17950 50  0001 C CNN
F 3 "~" H 8360 17950 50  0001 C CNN
	1    8360 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D120
U 1 1 6D395E4E
P 8360 18450
F 0 "D120" H 8353 18666 50  0000 C CNN
F 1 "LED" H 8353 18575 50  0000 C CNN
F 2 "" H 8360 18450 50  0001 C CNN
F 3 "~" H 8360 18450 50  0001 C CNN
	1    8360 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	7650 18600 7650 18700
Wire Wire Line
	7650 18700 7460 18700
Wire Wire Line
	7460 18700 7460 18170
Wire Wire Line
	7460 18170 7650 18170
Wire Wire Line
	7650 18170 7650 18100
Wire Wire Line
	7460 18170 7460 17660
Wire Wire Line
	7460 17660 7650 17660
Wire Wire Line
	7650 17660 7650 17600
Connection ~ 7460 18170
Wire Wire Line
	7460 17660 7460 17170
Wire Wire Line
	7460 17170 7650 17170
Wire Wire Line
	7650 17170 7650 17100
Connection ~ 7460 17660
Wire Wire Line
	7460 17170 7460 16660
Connection ~ 7460 17170
Wire Wire Line
	7460 16660 7650 16660
Wire Wire Line
	7650 16660 7650 16600
Wire Wire Line
	8360 18700 8160 18700
Wire Wire Line
	8160 18700 8160 18170
Wire Wire Line
	8160 16670 8360 16670
Wire Wire Line
	8160 17170 8360 17170
Connection ~ 8160 17170
Wire Wire Line
	8160 17170 8160 16670
Wire Wire Line
	8160 17660 8360 17660
Connection ~ 8160 17660
Wire Wire Line
	8160 17660 8160 17170
Wire Wire Line
	8160 18170 8360 18170
Connection ~ 8160 18170
Wire Wire Line
	8160 18170 8160 17660
Wire Wire Line
	7650 16800 8360 16800
Wire Wire Line
	7650 17800 8360 17800
Wire Wire Line
	8360 17300 7650 17300
Wire Wire Line
	8360 17600 8360 17660
Wire Wire Line
	8360 17170 8360 17100
Wire Wire Line
	8360 16670 8360 16600
Wire Wire Line
	8360 18700 8360 18600
Wire Wire Line
	8360 18170 8360 18100
Wire Wire Line
	7650 16300 8360 16300
Connection ~ 7460 16660
Connection ~ 8160 16670
Wire Wire Line
	8160 16180 8160 16670
Connection ~ 8160 16180
Wire Wire Line
	7460 16180 7460 16660
Connection ~ 7460 16180
Wire Wire Line
	8360 18300 7650 18300
$Comp
L Device:LED D108
U 1 1 6D395E81
P 7650 18450
F 0 "D108" H 7643 18666 50  0000 C CNN
F 1 "LED" H 7643 18575 50  0000 C CNN
F 2 "" H 7650 18450 50  0001 C CNN
F 3 "~" H 7650 18450 50  0001 C CNN
	1    7650 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	8770 13640 8770 13140
Connection ~ 8770 13640
Wire Wire Line
	8960 13640 8960 13580
Wire Wire Line
	8770 13640 8960 13640
Wire Wire Line
	8770 14140 8770 13640
Connection ~ 8770 14140
Wire Wire Line
	8960 14140 8960 14080
Wire Wire Line
	8770 14140 8960 14140
Wire Wire Line
	8960 13140 8960 13080
Wire Wire Line
	8770 13140 8960 13140
Connection ~ 8770 14650
Wire Wire Line
	8770 14650 8770 14140
Connection ~ 8770 15140
Wire Wire Line
	8960 14650 8960 14580
Wire Wire Line
	8770 14650 8960 14650
Wire Wire Line
	8770 15140 8770 14650
Connection ~ 8770 15650
Wire Wire Line
	8960 15140 8960 15080
Wire Wire Line
	8770 15140 8960 15140
Wire Wire Line
	8770 15650 8770 15140
Wire Wire Line
	8960 15650 8960 15580
Wire Wire Line
	8770 15650 8960 15650
Wire Wire Line
	8770 16180 8770 15650
Wire Wire Line
	8960 16180 8770 16180
Wire Wire Line
	8960 16080 8960 16180
$Comp
L Device:LED D127
U 1 1 6D395ECC
P 8960 15930
F 0 "D127" H 8953 16146 50  0000 C CNN
F 1 "LED" H 8953 16055 50  0000 C CNN
F 2 "" H 8960 15930 50  0001 C CNN
F 3 "~" H 8960 15930 50  0001 C CNN
	1    8960 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D126
U 1 1 6D395ED8
P 8960 15430
F 0 "D126" H 8953 15646 50  0000 C CNN
F 1 "LED" H 8953 15555 50  0000 C CNN
F 2 "" H 8960 15430 50  0001 C CNN
F 3 "~" H 8960 15430 50  0001 C CNN
	1    8960 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D125
U 1 1 6D395EE4
P 8960 14930
F 0 "D125" H 8953 15146 50  0000 C CNN
F 1 "LED" H 8953 15055 50  0000 C CNN
F 2 "" H 8960 14930 50  0001 C CNN
F 3 "~" H 8960 14930 50  0001 C CNN
	1    8960 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D124
U 1 1 6D395EF0
P 8960 14430
F 0 "D124" H 8953 14646 50  0000 C CNN
F 1 "LED" H 8953 14555 50  0000 C CNN
F 2 "" H 8960 14430 50  0001 C CNN
F 3 "~" H 8960 14430 50  0001 C CNN
	1    8960 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D123
U 1 1 6D395EFC
P 8960 13930
F 0 "D123" H 8953 14146 50  0000 C CNN
F 1 "LED" H 8953 14055 50  0000 C CNN
F 2 "" H 8960 13930 50  0001 C CNN
F 3 "~" H 8960 13930 50  0001 C CNN
	1    8960 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D122
U 1 1 6D395F08
P 8960 13430
F 0 "D122" H 8953 13646 50  0000 C CNN
F 1 "LED" H 8953 13555 50  0000 C CNN
F 2 "" H 8960 13430 50  0001 C CNN
F 3 "~" H 8960 13430 50  0001 C CNN
	1    8960 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D121
U 1 1 6D395F14
P 8960 12930
F 0 "D121" H 8953 13146 50  0000 C CNN
F 1 "LED" H 8953 13055 50  0000 C CNN
F 2 "" H 8960 12930 50  0001 C CNN
F 3 "~" H 8960 12930 50  0001 C CNN
	1    8960 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D128
U 1 1 6D395F29
P 8960 16450
F 0 "D128" H 8953 16666 50  0000 C CNN
F 1 "LED" H 8953 16575 50  0000 C CNN
F 2 "" H 8960 16450 50  0001 C CNN
F 3 "~" H 8960 16450 50  0001 C CNN
	1    8960 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D129
U 1 1 6D395F35
P 8960 16950
F 0 "D129" H 8953 17166 50  0000 C CNN
F 1 "LED" H 8953 17075 50  0000 C CNN
F 2 "" H 8960 16950 50  0001 C CNN
F 3 "~" H 8960 16950 50  0001 C CNN
	1    8960 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D130
U 1 1 6D395F41
P 8960 17450
F 0 "D130" H 8953 17666 50  0000 C CNN
F 1 "LED" H 8953 17575 50  0000 C CNN
F 2 "" H 8960 17450 50  0001 C CNN
F 3 "~" H 8960 17450 50  0001 C CNN
	1    8960 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D131
U 1 1 6D395F4D
P 8960 17950
F 0 "D131" H 8953 18166 50  0000 C CNN
F 1 "LED" H 8953 18075 50  0000 C CNN
F 2 "" H 8960 17950 50  0001 C CNN
F 3 "~" H 8960 17950 50  0001 C CNN
	1    8960 17950
	0    1    -1   0   
$EndComp
Wire Wire Line
	8960 18600 8960 18700
Wire Wire Line
	8960 18700 8770 18700
Wire Wire Line
	8770 18700 8770 18170
Wire Wire Line
	8770 18170 8960 18170
Wire Wire Line
	8960 18170 8960 18100
Wire Wire Line
	8770 18170 8770 17660
Wire Wire Line
	8770 17660 8960 17660
Wire Wire Line
	8960 17660 8960 17600
Connection ~ 8770 18170
Wire Wire Line
	8770 17660 8770 17170
Wire Wire Line
	8770 17170 8960 17170
Wire Wire Line
	8960 17170 8960 17100
Connection ~ 8770 17660
Wire Wire Line
	8770 17170 8770 16660
Connection ~ 8770 17170
Wire Wire Line
	8770 16660 8960 16660
Wire Wire Line
	8960 16660 8960 16600
Connection ~ 8770 16660
Wire Wire Line
	8770 16180 8770 16660
Connection ~ 8770 16180
$Comp
L Device:LED D132
U 1 1 6D395F8C
P 8960 18450
F 0 "D132" H 8953 18666 50  0000 C CNN
F 1 "LED" H 8953 18575 50  0000 C CNN
F 2 "" H 8960 18450 50  0001 C CNN
F 3 "~" H 8960 18450 50  0001 C CNN
	1    8960 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	8360 12780 8960 12780
Connection ~ 8360 12780
Wire Wire Line
	8360 13280 8960 13280
Connection ~ 8360 13280
Wire Wire Line
	8360 13780 8960 13780
Connection ~ 8360 13780
Wire Wire Line
	8360 14280 8960 14280
Connection ~ 8360 14280
Wire Wire Line
	8360 14780 8960 14780
Connection ~ 8360 14780
Wire Wire Line
	8360 18300 8960 18300
Connection ~ 8360 18300
Wire Wire Line
	8360 17800 8960 17800
Connection ~ 8360 17800
Wire Wire Line
	8360 17300 8960 17300
Connection ~ 8360 17300
Wire Wire Line
	8360 16800 8960 16800
Connection ~ 8360 16800
Wire Wire Line
	8360 16300 8960 16300
Connection ~ 8360 16300
Wire Wire Line
	8360 15280 8960 15280
Connection ~ 8360 15280
Connection ~ 3760 15280
Connection ~ 3160 15280
Wire Wire Line
	3160 15280 3760 15280
Connection ~ 3760 16300
Connection ~ 3160 16300
Wire Wire Line
	3160 16300 3760 16300
Connection ~ 3760 16800
Connection ~ 3160 16800
Wire Wire Line
	3160 16800 3760 16800
Connection ~ 3760 17300
Connection ~ 3160 17300
Wire Wire Line
	3160 17300 3760 17300
Connection ~ 3760 17800
Connection ~ 3160 17800
Wire Wire Line
	3160 17800 3760 17800
Connection ~ 3760 18300
Connection ~ 3160 18300
Wire Wire Line
	3160 18300 3760 18300
Connection ~ 3760 14780
Connection ~ 3160 14780
Wire Wire Line
	3160 14780 3760 14780
Connection ~ 3760 14280
Connection ~ 3160 14280
Wire Wire Line
	3160 14280 3760 14280
Connection ~ 3760 13780
Connection ~ 3160 13780
Wire Wire Line
	3160 13780 3760 13780
Connection ~ 3760 13280
Connection ~ 3160 13280
Wire Wire Line
	3160 13280 3760 13280
Connection ~ 3760 12780
Connection ~ 3160 12780
Wire Wire Line
	3160 12780 3760 12780
$Comp
L Device:LED D36
U 1 1 6C517EA7
P 3760 18450
F 0 "D36" H 3753 18666 50  0000 C CNN
F 1 "LED" H 3753 18575 50  0000 C CNN
F 2 "" H 3760 18450 50  0001 C CNN
F 3 "~" H 3760 18450 50  0001 C CNN
	1    3760 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	4470 18300 3760 18300
Connection ~ 3570 16180
Wire Wire Line
	3570 16180 3570 16660
Connection ~ 4270 16180
Wire Wire Line
	4270 16180 4270 16670
Connection ~ 4270 16670
Connection ~ 3570 16660
Wire Wire Line
	3760 16300 4470 16300
Wire Wire Line
	4470 18170 4470 18100
Wire Wire Line
	4470 18700 4470 18600
Wire Wire Line
	4470 16670 4470 16600
Wire Wire Line
	4470 17170 4470 17100
Wire Wire Line
	4470 17600 4470 17660
Wire Wire Line
	4470 17300 3760 17300
Wire Wire Line
	3760 17800 4470 17800
Wire Wire Line
	3760 16800 4470 16800
Wire Wire Line
	4270 18170 4270 17660
Connection ~ 4270 18170
Wire Wire Line
	4270 18170 4470 18170
Wire Wire Line
	4270 17660 4270 17170
Connection ~ 4270 17660
Wire Wire Line
	4270 17660 4470 17660
Wire Wire Line
	4270 17170 4270 16670
Connection ~ 4270 17170
Wire Wire Line
	4270 17170 4470 17170
Wire Wire Line
	4270 16670 4470 16670
Wire Wire Line
	4270 18700 4270 18170
Wire Wire Line
	4470 18700 4270 18700
Wire Wire Line
	3760 16660 3760 16600
Wire Wire Line
	3570 16660 3760 16660
Connection ~ 3570 17170
Wire Wire Line
	3570 17170 3570 16660
Connection ~ 3570 17660
Wire Wire Line
	3760 17170 3760 17100
Wire Wire Line
	3570 17170 3760 17170
Wire Wire Line
	3570 17660 3570 17170
Connection ~ 3570 18170
Wire Wire Line
	3760 17660 3760 17600
Wire Wire Line
	3570 17660 3760 17660
Wire Wire Line
	3570 18170 3570 17660
Wire Wire Line
	3760 18170 3760 18100
Wire Wire Line
	3570 18170 3760 18170
Wire Wire Line
	3570 18700 3570 18170
Wire Wire Line
	3760 18700 3570 18700
Wire Wire Line
	3760 18600 3760 18700
$Comp
L Device:LED D48
U 1 1 6C517E74
P 4470 18450
F 0 "D48" H 4463 18666 50  0000 C CNN
F 1 "LED" H 4463 18575 50  0000 C CNN
F 2 "" H 4470 18450 50  0001 C CNN
F 3 "~" H 4470 18450 50  0001 C CNN
	1    4470 18450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D47
U 1 1 6C517E6E
P 4470 17950
F 0 "D47" H 4463 18166 50  0000 C CNN
F 1 "LED" H 4463 18075 50  0000 C CNN
F 2 "" H 4470 17950 50  0001 C CNN
F 3 "~" H 4470 17950 50  0001 C CNN
	1    4470 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D35
U 1 1 6C517E68
P 3760 17950
F 0 "D35" H 3753 18166 50  0000 C CNN
F 1 "LED" H 3753 18075 50  0000 C CNN
F 2 "" H 3760 17950 50  0001 C CNN
F 3 "~" H 3760 17950 50  0001 C CNN
	1    3760 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D46
U 1 1 6C517E62
P 4470 17450
F 0 "D46" H 4463 17666 50  0000 C CNN
F 1 "LED" H 4463 17575 50  0000 C CNN
F 2 "" H 4470 17450 50  0001 C CNN
F 3 "~" H 4470 17450 50  0001 C CNN
	1    4470 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D34
U 1 1 6C517E5C
P 3760 17450
F 0 "D34" H 3753 17666 50  0000 C CNN
F 1 "LED" H 3753 17575 50  0000 C CNN
F 2 "" H 3760 17450 50  0001 C CNN
F 3 "~" H 3760 17450 50  0001 C CNN
	1    3760 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D45
U 1 1 6C517E56
P 4470 16950
F 0 "D45" H 4463 17166 50  0000 C CNN
F 1 "LED" H 4463 17075 50  0000 C CNN
F 2 "" H 4470 16950 50  0001 C CNN
F 3 "~" H 4470 16950 50  0001 C CNN
	1    4470 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D33
U 1 1 6C517E50
P 3760 16950
F 0 "D33" H 3753 17166 50  0000 C CNN
F 1 "LED" H 3753 17075 50  0000 C CNN
F 2 "" H 3760 16950 50  0001 C CNN
F 3 "~" H 3760 16950 50  0001 C CNN
	1    3760 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D44
U 1 1 6C517E4A
P 4470 16450
F 0 "D44" H 4463 16666 50  0000 C CNN
F 1 "LED" H 4463 16575 50  0000 C CNN
F 2 "" H 4470 16450 50  0001 C CNN
F 3 "~" H 4470 16450 50  0001 C CNN
	1    4470 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D32
U 1 1 6C517E44
P 3760 16450
F 0 "D32" H 3753 16666 50  0000 C CNN
F 1 "LED" H 3753 16575 50  0000 C CNN
F 2 "" H 3760 16450 50  0001 C CNN
F 3 "~" H 3760 16450 50  0001 C CNN
	1    3760 16450
	0    1    -1   0   
$EndComp
Wire Wire Line
	4270 16180 4270 15650
Wire Wire Line
	4470 15650 4470 15580
Wire Wire Line
	4470 16180 4470 16080
$Comp
L Device:LED D25
U 1 1 6C517E2F
P 3760 12930
F 0 "D25" H 3753 13146 50  0000 C CNN
F 1 "LED" H 3753 13055 50  0000 C CNN
F 2 "" H 3760 12930 50  0001 C CNN
F 3 "~" H 3760 12930 50  0001 C CNN
	1    3760 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D37
U 1 1 6C517E29
P 4470 12930
F 0 "D37" H 4463 13146 50  0000 C CNN
F 1 "LED" H 4463 13055 50  0000 C CNN
F 2 "" H 4470 12930 50  0001 C CNN
F 3 "~" H 4470 12930 50  0001 C CNN
	1    4470 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D26
U 1 1 6C517E23
P 3760 13430
F 0 "D26" H 3753 13646 50  0000 C CNN
F 1 "LED" H 3753 13555 50  0000 C CNN
F 2 "" H 3760 13430 50  0001 C CNN
F 3 "~" H 3760 13430 50  0001 C CNN
	1    3760 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D38
U 1 1 6C517E1D
P 4470 13430
F 0 "D38" H 4463 13646 50  0000 C CNN
F 1 "LED" H 4463 13555 50  0000 C CNN
F 2 "" H 4470 13430 50  0001 C CNN
F 3 "~" H 4470 13430 50  0001 C CNN
	1    4470 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D27
U 1 1 6C517E17
P 3760 13930
F 0 "D27" H 3753 14146 50  0000 C CNN
F 1 "LED" H 3753 14055 50  0000 C CNN
F 2 "" H 3760 13930 50  0001 C CNN
F 3 "~" H 3760 13930 50  0001 C CNN
	1    3760 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D39
U 1 1 6C517E11
P 4470 13930
F 0 "D39" H 4463 14146 50  0000 C CNN
F 1 "LED" H 4463 14055 50  0000 C CNN
F 2 "" H 4470 13930 50  0001 C CNN
F 3 "~" H 4470 13930 50  0001 C CNN
	1    4470 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D28
U 1 1 6C517E0B
P 3760 14430
F 0 "D28" H 3753 14646 50  0000 C CNN
F 1 "LED" H 3753 14555 50  0000 C CNN
F 2 "" H 3760 14430 50  0001 C CNN
F 3 "~" H 3760 14430 50  0001 C CNN
	1    3760 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D40
U 1 1 6C517E05
P 4470 14430
F 0 "D40" H 4463 14646 50  0000 C CNN
F 1 "LED" H 4463 14555 50  0000 C CNN
F 2 "" H 4470 14430 50  0001 C CNN
F 3 "~" H 4470 14430 50  0001 C CNN
	1    4470 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D29
U 1 1 6C517DFF
P 3760 14930
F 0 "D29" H 3753 15146 50  0000 C CNN
F 1 "LED" H 3753 15055 50  0000 C CNN
F 2 "" H 3760 14930 50  0001 C CNN
F 3 "~" H 3760 14930 50  0001 C CNN
	1    3760 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D41
U 1 1 6C517DF9
P 4470 14930
F 0 "D41" H 4463 15146 50  0000 C CNN
F 1 "LED" H 4463 15055 50  0000 C CNN
F 2 "" H 4470 14930 50  0001 C CNN
F 3 "~" H 4470 14930 50  0001 C CNN
	1    4470 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D30
U 1 1 6C517DF3
P 3760 15430
F 0 "D30" H 3753 15646 50  0000 C CNN
F 1 "LED" H 3753 15555 50  0000 C CNN
F 2 "" H 3760 15430 50  0001 C CNN
F 3 "~" H 3760 15430 50  0001 C CNN
	1    3760 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D42
U 1 1 6C517DED
P 4470 15430
F 0 "D42" H 4463 15646 50  0000 C CNN
F 1 "LED" H 4463 15555 50  0000 C CNN
F 2 "" H 4470 15430 50  0001 C CNN
F 3 "~" H 4470 15430 50  0001 C CNN
	1    4470 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D31
U 1 1 6C517DE7
P 3760 15930
F 0 "D31" H 3753 16146 50  0000 C CNN
F 1 "LED" H 3753 16055 50  0000 C CNN
F 2 "" H 3760 15930 50  0001 C CNN
F 3 "~" H 3760 15930 50  0001 C CNN
	1    3760 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D43
U 1 1 6C517DE1
P 4470 15930
F 0 "D43" H 4463 16146 50  0000 C CNN
F 1 "LED" H 4463 16055 50  0000 C CNN
F 2 "" H 4470 15930 50  0001 C CNN
F 3 "~" H 4470 15930 50  0001 C CNN
	1    4470 15930
	0    1    -1   0   
$EndComp
Wire Wire Line
	3760 16080 3760 16180
Wire Wire Line
	3760 16180 3570 16180
Wire Wire Line
	3570 16180 3570 15650
Wire Wire Line
	3570 15650 3760 15650
Wire Wire Line
	3760 15650 3760 15580
Wire Wire Line
	3570 15650 3570 15140
Wire Wire Line
	3570 15140 3760 15140
Wire Wire Line
	3760 15140 3760 15080
Connection ~ 3570 15650
Wire Wire Line
	3570 15140 3570 14650
Wire Wire Line
	3570 14650 3760 14650
Wire Wire Line
	3760 14650 3760 14580
Connection ~ 3570 15140
Wire Wire Line
	3570 14650 3570 14140
Connection ~ 3570 14650
Wire Wire Line
	3570 13140 3760 13140
Wire Wire Line
	3760 13140 3760 13080
Wire Wire Line
	3570 14140 3760 14140
Wire Wire Line
	3760 14140 3760 14080
Connection ~ 3570 14140
Wire Wire Line
	3570 14140 3570 13640
Wire Wire Line
	3570 13640 3760 13640
Wire Wire Line
	3760 13640 3760 13580
Connection ~ 3570 13640
Wire Wire Line
	3570 13640 3570 13140
Wire Wire Line
	4470 16180 4270 16180
Wire Wire Line
	4270 13140 4470 13140
Wire Wire Line
	4270 13630 4470 13630
Connection ~ 4270 13630
Wire Wire Line
	4270 13630 4270 13140
Wire Wire Line
	4270 14150 4470 14150
Connection ~ 4270 14150
Wire Wire Line
	4270 14150 4270 13630
Wire Wire Line
	4270 14650 4470 14650
Connection ~ 4270 14650
Wire Wire Line
	4270 14650 4270 14150
Wire Wire Line
	4270 15140 4470 15140
Connection ~ 4270 15140
Wire Wire Line
	4270 15140 4270 14650
Wire Wire Line
	4270 15650 4470 15650
Connection ~ 4270 15650
Wire Wire Line
	4270 15650 4270 15140
Wire Wire Line
	3760 13280 4470 13280
Wire Wire Line
	3760 14280 4470 14280
Wire Wire Line
	3760 15280 4470 15280
Wire Wire Line
	4470 15780 3760 15780
Wire Wire Line
	4470 14780 3760 14780
Wire Wire Line
	4470 13780 3760 13780
Wire Wire Line
	4470 12780 3760 12780
Wire Wire Line
	4470 15080 4470 15140
Wire Wire Line
	4470 14650 4470 14580
Wire Wire Line
	4470 14150 4470 14080
Wire Wire Line
	4470 13630 4470 13580
Wire Wire Line
	4470 13140 4470 13080
$Comp
L Device:LED D12
U 1 1 6B7F05B7
P 2450 18450
F 0 "D12" H 2443 18666 50  0000 C CNN
F 1 "LED" H 2443 18575 50  0000 C CNN
F 2 "" H 2450 18450 50  0001 C CNN
F 3 "~" H 2450 18450 50  0001 C CNN
	1    2450 18450
	0    1    -1   0   
$EndComp
Wire Wire Line
	3160 18300 2450 18300
Connection ~ 2260 16180
Wire Wire Line
	2260 16180 2260 16660
Connection ~ 2960 16180
Wire Wire Line
	2960 16180 2960 16670
Connection ~ 2960 16670
Connection ~ 2260 16660
Wire Wire Line
	2450 16300 3160 16300
Wire Wire Line
	3160 18170 3160 18100
Wire Wire Line
	3160 18700 3160 18600
Wire Wire Line
	3160 16670 3160 16600
Wire Wire Line
	3160 17170 3160 17100
Wire Wire Line
	3160 17600 3160 17660
Wire Wire Line
	3160 17300 2450 17300
Wire Wire Line
	2450 17800 3160 17800
Wire Wire Line
	2450 16800 3160 16800
Wire Wire Line
	2960 18170 2960 17660
Connection ~ 2960 18170
Wire Wire Line
	2960 18170 3160 18170
Wire Wire Line
	2960 17660 2960 17170
Connection ~ 2960 17660
Wire Wire Line
	2960 17660 3160 17660
Wire Wire Line
	2960 17170 2960 16670
Connection ~ 2960 17170
Wire Wire Line
	2960 17170 3160 17170
Wire Wire Line
	2960 16670 3160 16670
Wire Wire Line
	2960 18700 2960 18170
Wire Wire Line
	3160 18700 2960 18700
Wire Wire Line
	2450 16660 2450 16600
Wire Wire Line
	2260 16660 2450 16660
Connection ~ 2260 17170
Wire Wire Line
	2260 17170 2260 16660
Connection ~ 2260 17660
Wire Wire Line
	2450 17170 2450 17100
Wire Wire Line
	2260 17170 2450 17170
Wire Wire Line
	2260 17660 2260 17170
Connection ~ 2260 18170
Wire Wire Line
	2450 17660 2450 17600
Wire Wire Line
	2260 17660 2450 17660
Wire Wire Line
	2260 18170 2260 17660
Wire Wire Line
	2450 18170 2450 18100
Wire Wire Line
	2260 18170 2450 18170
Wire Wire Line
	2260 18700 2260 18170
Wire Wire Line
	2450 18700 2260 18700
Wire Wire Line
	2450 18600 2450 18700
$Comp
L Device:LED D24
U 1 1 6B7F05BD
P 3160 18450
F 0 "D24" H 3153 18666 50  0000 C CNN
F 1 "LED" H 3153 18575 50  0000 C CNN
F 2 "" H 3160 18450 50  0001 C CNN
F 3 "~" H 3160 18450 50  0001 C CNN
	1    3160 18450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D23
U 1 1 6B7F05B1
P 3160 17950
F 0 "D23" H 3153 18166 50  0000 C CNN
F 1 "LED" H 3153 18075 50  0000 C CNN
F 2 "" H 3160 17950 50  0001 C CNN
F 3 "~" H 3160 17950 50  0001 C CNN
	1    3160 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D11
U 1 1 6B7F05AB
P 2450 17950
F 0 "D11" H 2443 18166 50  0000 C CNN
F 1 "LED" H 2443 18075 50  0000 C CNN
F 2 "" H 2450 17950 50  0001 C CNN
F 3 "~" H 2450 17950 50  0001 C CNN
	1    2450 17950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D22
U 1 1 6B7F05A5
P 3160 17450
F 0 "D22" H 3153 17666 50  0000 C CNN
F 1 "LED" H 3153 17575 50  0000 C CNN
F 2 "" H 3160 17450 50  0001 C CNN
F 3 "~" H 3160 17450 50  0001 C CNN
	1    3160 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D10
U 1 1 6B7F059F
P 2450 17450
F 0 "D10" H 2443 17666 50  0000 C CNN
F 1 "LED" H 2443 17575 50  0000 C CNN
F 2 "" H 2450 17450 50  0001 C CNN
F 3 "~" H 2450 17450 50  0001 C CNN
	1    2450 17450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D21
U 1 1 6B7F0599
P 3160 16950
F 0 "D21" H 3153 17166 50  0000 C CNN
F 1 "LED" H 3153 17075 50  0000 C CNN
F 2 "" H 3160 16950 50  0001 C CNN
F 3 "~" H 3160 16950 50  0001 C CNN
	1    3160 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D9
U 1 1 6B7F0593
P 2450 16950
F 0 "D9" H 2443 17166 50  0000 C CNN
F 1 "LED" H 2443 17075 50  0000 C CNN
F 2 "" H 2450 16950 50  0001 C CNN
F 3 "~" H 2450 16950 50  0001 C CNN
	1    2450 16950
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D20
U 1 1 6B7F058D
P 3160 16450
F 0 "D20" H 3153 16666 50  0000 C CNN
F 1 "LED" H 3153 16575 50  0000 C CNN
F 2 "" H 3160 16450 50  0001 C CNN
F 3 "~" H 3160 16450 50  0001 C CNN
	1    3160 16450
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D8
U 1 1 6B7F0587
P 2450 16450
F 0 "D8" H 2443 16666 50  0000 C CNN
F 1 "LED" H 2443 16575 50  0000 C CNN
F 2 "" H 2450 16450 50  0001 C CNN
F 3 "~" H 2450 16450 50  0001 C CNN
	1    2450 16450
	0    1    -1   0   
$EndComp
Wire Wire Line
	2960 16180 2960 15650
Wire Wire Line
	3160 15650 3160 15580
Wire Wire Line
	3160 16180 3160 16080
$Comp
L Device:LED D1
U 1 1 6A26F903
P 2450 12930
F 0 "D1" H 2443 13146 50  0000 C CNN
F 1 "LED" H 2443 13055 50  0000 C CNN
F 2 "" H 2450 12930 50  0001 C CNN
F 3 "~" H 2450 12930 50  0001 C CNN
	1    2450 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D13
U 1 1 6A26F909
P 3160 12930
F 0 "D13" H 3153 13146 50  0000 C CNN
F 1 "LED" H 3153 13055 50  0000 C CNN
F 2 "" H 3160 12930 50  0001 C CNN
F 3 "~" H 3160 12930 50  0001 C CNN
	1    3160 12930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 6A26F90F
P 2450 13430
F 0 "D2" H 2443 13646 50  0000 C CNN
F 1 "LED" H 2443 13555 50  0000 C CNN
F 2 "" H 2450 13430 50  0001 C CNN
F 3 "~" H 2450 13430 50  0001 C CNN
	1    2450 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D14
U 1 1 6A26F915
P 3160 13430
F 0 "D14" H 3153 13646 50  0000 C CNN
F 1 "LED" H 3153 13555 50  0000 C CNN
F 2 "" H 3160 13430 50  0001 C CNN
F 3 "~" H 3160 13430 50  0001 C CNN
	1    3160 13430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D3
U 1 1 6A26F91B
P 2450 13930
F 0 "D3" H 2443 14146 50  0000 C CNN
F 1 "LED" H 2443 14055 50  0000 C CNN
F 2 "" H 2450 13930 50  0001 C CNN
F 3 "~" H 2450 13930 50  0001 C CNN
	1    2450 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D15
U 1 1 6A26F921
P 3160 13930
F 0 "D15" H 3153 14146 50  0000 C CNN
F 1 "LED" H 3153 14055 50  0000 C CNN
F 2 "" H 3160 13930 50  0001 C CNN
F 3 "~" H 3160 13930 50  0001 C CNN
	1    3160 13930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D4
U 1 1 6A26F927
P 2450 14430
F 0 "D4" H 2443 14646 50  0000 C CNN
F 1 "LED" H 2443 14555 50  0000 C CNN
F 2 "" H 2450 14430 50  0001 C CNN
F 3 "~" H 2450 14430 50  0001 C CNN
	1    2450 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D16
U 1 1 6A26F92D
P 3160 14430
F 0 "D16" H 3153 14646 50  0000 C CNN
F 1 "LED" H 3153 14555 50  0000 C CNN
F 2 "" H 3160 14430 50  0001 C CNN
F 3 "~" H 3160 14430 50  0001 C CNN
	1    3160 14430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 6A26F933
P 2450 14930
F 0 "D5" H 2443 15146 50  0000 C CNN
F 1 "LED" H 2443 15055 50  0000 C CNN
F 2 "" H 2450 14930 50  0001 C CNN
F 3 "~" H 2450 14930 50  0001 C CNN
	1    2450 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D17
U 1 1 6A26F939
P 3160 14930
F 0 "D17" H 3153 15146 50  0000 C CNN
F 1 "LED" H 3153 15055 50  0000 C CNN
F 2 "" H 3160 14930 50  0001 C CNN
F 3 "~" H 3160 14930 50  0001 C CNN
	1    3160 14930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D6
U 1 1 6A26F93F
P 2450 15430
F 0 "D6" H 2443 15646 50  0000 C CNN
F 1 "LED" H 2443 15555 50  0000 C CNN
F 2 "" H 2450 15430 50  0001 C CNN
F 3 "~" H 2450 15430 50  0001 C CNN
	1    2450 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D18
U 1 1 6A26F945
P 3160 15430
F 0 "D18" H 3153 15646 50  0000 C CNN
F 1 "LED" H 3153 15555 50  0000 C CNN
F 2 "" H 3160 15430 50  0001 C CNN
F 3 "~" H 3160 15430 50  0001 C CNN
	1    3160 15430
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D7
U 1 1 6A26F94B
P 2450 15930
F 0 "D7" H 2443 16146 50  0000 C CNN
F 1 "LED" H 2443 16055 50  0000 C CNN
F 2 "" H 2450 15930 50  0001 C CNN
F 3 "~" H 2450 15930 50  0001 C CNN
	1    2450 15930
	0    1    -1   0   
$EndComp
$Comp
L Device:LED D19
U 1 1 6A26F951
P 3160 15930
F 0 "D19" H 3153 16146 50  0000 C CNN
F 1 "LED" H 3153 16055 50  0000 C CNN
F 2 "" H 3160 15930 50  0001 C CNN
F 3 "~" H 3160 15930 50  0001 C CNN
	1    3160 15930
	0    1    -1   0   
$EndComp
Wire Wire Line
	2450 16080 2450 16180
Wire Wire Line
	2450 16180 2260 16180
Wire Wire Line
	2260 16180 2260 15650
Wire Wire Line
	2260 15650 2450 15650
Wire Wire Line
	2450 15650 2450 15580
Wire Wire Line
	2260 15650 2260 15140
Wire Wire Line
	2260 15140 2450 15140
Wire Wire Line
	2450 15140 2450 15080
Connection ~ 2260 15650
Wire Wire Line
	2260 15140 2260 14650
Wire Wire Line
	2260 14650 2450 14650
Wire Wire Line
	2450 14650 2450 14580
Connection ~ 2260 15140
Wire Wire Line
	2260 14650 2260 14140
Connection ~ 2260 14650
Wire Wire Line
	2260 13140 2450 13140
Wire Wire Line
	2450 13140 2450 13080
Wire Wire Line
	2260 14140 2450 14140
Wire Wire Line
	2450 14140 2450 14080
Connection ~ 2260 14140
Wire Wire Line
	2260 14140 2260 13640
Wire Wire Line
	2260 13640 2450 13640
Wire Wire Line
	2450 13640 2450 13580
Connection ~ 2260 13640
Wire Wire Line
	2260 13640 2260 13140
Wire Wire Line
	3160 16180 2960 16180
Wire Wire Line
	2960 13140 3160 13140
Wire Wire Line
	2960 13630 3160 13630
Connection ~ 2960 13630
Wire Wire Line
	2960 13630 2960 13140
Wire Wire Line
	2960 14150 3160 14150
Connection ~ 2960 14150
Wire Wire Line
	2960 14150 2960 13630
Wire Wire Line
	2960 14650 3160 14650
Connection ~ 2960 14650
Wire Wire Line
	2960 14650 2960 14150
Wire Wire Line
	2960 15140 3160 15140
Connection ~ 2960 15140
Wire Wire Line
	2960 15140 2960 14650
Wire Wire Line
	2960 15650 3160 15650
Connection ~ 2960 15650
Wire Wire Line
	2960 15650 2960 15140
Wire Wire Line
	2450 13280 3160 13280
Wire Wire Line
	2450 14280 3160 14280
Wire Wire Line
	2450 15280 3160 15280
Wire Wire Line
	3160 15780 2450 15780
Wire Wire Line
	3160 14780 2450 14780
Wire Wire Line
	3160 13780 2450 13780
Wire Wire Line
	3160 12780 2450 12780
Wire Wire Line
	3160 15080 3160 15140
Wire Wire Line
	3160 14650 3160 14580
Wire Wire Line
	3160 14150 3160 14080
Wire Wire Line
	3160 13630 3160 13580
Wire Wire Line
	3160 13140 3160 13080
$Comp
L power:GND #PWR0203
U 1 1 65073B47
P 7010 11760
F 0 "#PWR0203" H 7010 11510 50  0001 C CNN
F 1 "GND" H 7015 11587 50  0000 C CNN
F 2 "" H 7010 11760 50  0001 C CNN
F 3 "" H 7010 11760 50  0001 C CNN
	1    7010 11760
	-1   0    0    -1  
$EndComp
Connection ~ 12810 5850
Wire Wire Line
	12810 5890 12810 5850
$Comp
L power:GND #PWR0202
U 1 1 6376C511
P 12810 5890
F 0 "#PWR0202" H 12810 5640 50  0001 C CNN
F 1 "GND" H 12815 5717 50  0000 C CNN
F 2 "" H 12810 5890 50  0001 C CNN
F 3 "" H 12810 5890 50  0001 C CNN
	1    12810 5890
	1    0    0    -1  
$EndComp
Wire Wire Line
	12810 5500 13390 5500
Wire Wire Line
	12810 5510 12810 5500
Wire Wire Line
	12810 5850 12810 5810
Wire Wire Line
	12660 5850 12810 5850
Wire Wire Line
	12660 5400 12660 5850
Wire Wire Line
	13390 5400 12660 5400
Wire Wire Line
	14440 3300 14440 3750
Wire Wire Line
	14290 3750 14440 3750
Wire Wire Line
	14290 4200 14290 3750
Wire Wire Line
	14040 3300 14040 3750
Wire Wire Line
	14190 3750 14040 3750
Wire Wire Line
	14190 4200 14190 3750
Wire Wire Line
	14290 6800 14290 6600
Wire Wire Line
	14420 7100 14420 7510
Wire Wire Line
	14190 6600 14190 7100
Wire Wire Line
	14020 7100 14020 7510
Wire Wire Line
	14090 7100 14090 6600
Wire Wire Line
	14020 7100 14090 7100
Connection ~ 14020 7510
Wire Wire Line
	13980 7510 14020 7510
Connection ~ 14420 7510
Wire Wire Line
	14420 7510 14380 7510
Wire Wire Line
	14420 7510 14470 7510
Wire Wire Line
	14020 7510 14080 7510
$Comp
L power:GND #PWR0201
U 1 1 632B6E3E
P 13980 7510
F 0 "#PWR0201" H 13980 7260 50  0001 C CNN
F 1 "GND" H 13985 7337 50  0000 C CNN
F 2 "" H 13980 7510 50  0001 C CNN
F 3 "" H 13980 7510 50  0001 C CNN
	1    13980 7510
	0    1    -1   0   
$EndComp
Text GLabel 14470 7510 2    50   Input ~ 0
3.3V
$Comp
L Device:C C60
U 1 1 632B6E37
P 14230 7510
F 0 "C60" V 14482 7510 50  0000 C CNN
F 1 "0,1µ" V 14391 7510 50  0000 C CNN
F 2 "" H 14268 7360 50  0001 C CNN
F 3 "~" H 14230 7510 50  0001 C CNN
	1    14230 7510
	0    1    -1   0   
$EndComp
Connection ~ 14440 3300
Wire Wire Line
	14480 3300 14440 3300
Connection ~ 14040 3300
Wire Wire Line
	14040 3300 14080 3300
Wire Wire Line
	14040 3300 13990 3300
Wire Wire Line
	14440 3300 14380 3300
$Comp
L power:GND #PWR0200
U 1 1 6313E08F
P 14480 3300
F 0 "#PWR0200" H 14480 3050 50  0001 C CNN
F 1 "GND" H 14485 3127 50  0000 C CNN
F 2 "" H 14480 3300 50  0001 C CNN
F 3 "" H 14480 3300 50  0001 C CNN
	1    14480 3300
	0    -1   1    0   
$EndComp
Text GLabel 13990 3300 0    50   Input ~ 0
3.3V
$Comp
L Device:C C59
U 1 1 6313E088
P 14230 3300
F 0 "C59" V 14482 3300 50  0000 C CNN
F 1 "0,1µ" V 14391 3300 50  0000 C CNN
F 2 "" H 14268 3150 50  0001 C CNN
F 3 "~" H 14230 3300 50  0001 C CNN
	1    14230 3300
	0    -1   1    0   
$EndComp
Connection ~ 16140 5160
Wire Wire Line
	16140 5120 16140 5160
Connection ~ 16140 5560
Wire Wire Line
	16140 5560 16140 5520
Wire Wire Line
	16140 5560 16140 5610
Wire Wire Line
	15740 5560 16140 5560
Wire Wire Line
	15740 5400 15740 5560
Wire Wire Line
	15090 5400 15740 5400
$Comp
L mengpaneel-rescue:MTCH6301-I_ML-MTCH6301-I_ML IC3
U 1 1 6064E9CE
P 13390 4900
F 0 "IC3" H 14937 3698 50  0000 L CNN
F 1 "MTCH6301-I_ML" H 14937 3568 50  0000 L BNN
F 2 "" H 14940 5400 50  0001 L CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en560085" H 14940 5300 50  0001 L CNN
F 4 "Capacitive Touch Sensors 32KB Flash 8KB RAM, 40 MHz" H 14940 5200 50  0001 L CNN "Description"
F 5 "1" H 14940 5100 50  0001 L CNN "Height"
F 6 "579-MTCH6301-I/ML" H 14940 5000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Microchip-Technology/MTCH6301-I-ML?qs=VteoEApvSICWu1DtjlRtOA%3D%3D" H 14940 4900 50  0001 L CNN "Mouser Price/Stock"
F 8 "Microchip" H 14940 4800 50  0001 L CNN "Manufacturer_Name"
F 9 "MTCH6301-I/ML" H 14940 4700 50  0001 L CNN "Manufacturer_Part_Number"
	1    13390 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	16140 5160 16140 5220
Wire Wire Line
	15740 5160 16140 5160
Wire Wire Line
	15740 5300 15740 5160
Wire Wire Line
	15090 5300 15740 5300
$Comp
L power:GND #PWR0199
U 1 1 62BF0416
P 16140 5120
F 0 "#PWR0199" H 16140 4870 50  0001 C CNN
F 1 "GND" H 16145 4947 50  0000 C CNN
F 2 "" H 16140 5120 50  0001 C CNN
F 3 "" H 16140 5120 50  0001 C CNN
	1    16140 5120
	-1   0    0    1   
$EndComp
Text GLabel 16140 5610 3    50   Input ~ 0
3.3V
$Comp
L Device:C C63
U 1 1 6271A567
P 16140 5370
F 0 "C63" V 16392 5370 50  0000 C CNN
F 1 "0,1µ" V 16301 5370 50  0000 C CNN
F 2 "" H 16178 5220 50  0001 C CNN
F 3 "~" H 16140 5370 50  0001 C CNN
	1    16140 5370
	-1   0    0    1   
$EndComp
$Comp
L Device:C C50
U 1 1 627155A5
P 12810 5660
F 0 "C50" H 12925 5706 50  0000 L CNN
F 1 "10µ" H 12925 5615 50  0000 L CNN
F 2 "" H 12848 5510 50  0001 C CNN
F 3 "~" H 12810 5660 50  0001 C CNN
	1    12810 5660
	1    0    0    -1  
$EndComp
NoConn ~ 13690 4200
NoConn ~ 13390 5000
NoConn ~ 13390 5100
NoConn ~ 13390 5200
NoConn ~ 13390 5300
Text GLabel 13990 4200 1    50   Input ~ 0
laag_11
Text GLabel 13890 4200 1    50   Input ~ 0
laag_12
Text GLabel 15090 5500 2    50   Input ~ 0
slider_1
Text GLabel 13890 6600 3    50   Input ~ 0
slider_11
Text GLabel 13990 6600 3    50   Input ~ 0
slider_10
Text GLabel 14390 6600 3    50   Input ~ 0
slider_9
Text GLabel 14490 6600 3    50   Input ~ 0
slider_8
Text GLabel 14590 6600 3    50   Input ~ 0
slider_7
Text GLabel 14690 6600 3    50   Input ~ 0
slider_6
Text GLabel 15090 5900 2    50   Input ~ 0
slider_5
Text GLabel 15090 5800 2    50   Input ~ 0
slider_4
Text GLabel 15090 5700 2    50   Input ~ 0
slider_3
Text GLabel 15090 5600 2    50   Input ~ 0
slider_2
Text GLabel 14090 4200 1    50   Input ~ 0
laag_10
Text GLabel 14390 4200 1    50   Input ~ 0
laag_6
Text GLabel 14490 4200 1    50   Input ~ 0
laag_7
Text GLabel 14590 4200 1    50   Input ~ 0
laag_8
Text GLabel 14690 4200 1    50   Input ~ 0
laag_9
Text GLabel 14790 4200 1    50   Input ~ 0
laag_5
Text GLabel 15090 5200 2    50   Input ~ 0
laag_4
Text GLabel 15090 5100 2    50   Input ~ 0
laag_3
Text GLabel 15090 5000 2    50   Input ~ 0
laag_2
Text GLabel 15090 4900 2    50   Input ~ 0
laag_1
NoConn ~ 13790 6600
NoConn ~ 13690 6600
Connection ~ 15290 12410
$Comp
L power:GND #PWR0198
U 1 1 6059BB6E
P 14490 12410
F 0 "#PWR0198" H 14490 12160 50  0001 C CNN
F 1 "GND" H 14495 12237 50  0000 C CNN
F 2 "" H 14490 12410 50  0001 C CNN
F 3 "" H 14490 12410 50  0001 C CNN
	1    14490 12410
	0    1    1    0   
$EndComp
Wire Wire Line
	15290 12410 15290 12210
Wire Wire Line
	15710 12410 15290 12410
Wire Wire Line
	15710 12210 15710 12410
$Comp
L Switch:SW_DPDT_x2 SW2
U 1 1 603E0750
P 15710 12010
F 0 "SW2" H 15710 12295 50  0000 C CNN
F 1 "SW_DPDT_x2" H 15710 12204 50  0000 C CNN
F 2 "" H 15710 12010 50  0001 C CNN
F 3 "~" H 15710 12010 50  0001 C CNN
	1    15710 12010
	0    1    -1   0   
$EndComp
$Comp
L Device:R R58
U 1 1 611EAB78
P 15680 8250
F 0 "R58" V 15473 8250 50  0000 C CNN
F 1 "100K" V 15564 8250 50  0000 C CNN
F 2 "" V 15610 8250 50  0001 C CNN
F 3 "~" H 15680 8250 50  0001 C CNN
	1    15680 8250
	-1   0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 60D3110D
P 13120 9980
F 0 "SW1" H 13120 10265 50  0000 C CNN
F 1 "SW_Push" H 13120 10174 50  0000 C CNN
F 2 "" H 13120 10180 50  0001 C CNN
F 3 "~" H 13120 10180 50  0001 C CNN
	1    13120 9980
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 60D0209E
P 14250 10240
F 0 "#PWR0126" H 14250 9990 50  0001 C CNN
F 1 "GND" H 14255 10067 50  0000 C CNN
F 2 "" H 14250 10240 50  0001 C CNN
F 3 "" H 14250 10240 50  0001 C CNN
	1    14250 10240
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C42
U 1 1 60A7CF24
P 14250 10070
F 0 "C42" H 14365 10116 50  0000 L CNN
F 1 "100nF" H 14365 10025 50  0000 L CNN
F 2 "" H 14288 9920 50  0001 C CNN
F 3 "~" H 14250 10070 50  0001 C CNN
	1    14250 10070
	1    0    0    -1  
$EndComp
Wire Wire Line
	14030 9780 14170 9780
Wire Wire Line
	14170 9740 14170 9780
Wire Wire Line
	14030 9410 14170 9410
Wire Wire Line
	14170 9440 14170 9410
Text GLabel 17790 15890 2    50   Input ~ 0
-12V
Text GLabel 17770 14850 2    50   Input ~ 0
+12V
Wire Wire Line
	25500 8990 25500 6720
Wire Wire Line
	22320 7060 22320 6720
$Comp
L Device:CP C16
U 1 1 5FD51A53
P 22320 7210
F 0 "C16" H 22438 7256 50  0000 L CNN
F 1 "4,7µ" H 22438 7165 50  0000 L CNN
F 2 "" H 22358 7060 50  0001 C CNN
F 3 "~" H 22320 7210 50  0001 C CNN
	1    22320 7210
	1    0    0    1   
$EndComp
$Comp
L pspice:OPAMP U24
U 1 1 5FD51B21
P 25800 9090
F 0 "U24" H 26144 9136 50  0000 L BNN
F 1 "LM324N" H 26144 9045 50  0000 L TNN
F 2 "" H 25800 9090 50  0001 C CNN
F 3 "~" H 25800 9090 50  0001 C CNN
	1    25800 9090
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR0115
U 1 1 5FD51B1B
P 25700 9390
F 0 "#PWR0115" H 25700 9490 50  0001 C CNN
F 1 "-12V" H 25715 9563 50  0000 C CNN
F 2 "" H 25700 9390 50  0001 C CNN
F 3 "" H 25700 9390 50  0001 C CNN
	1    25700 9390
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR0116
U 1 1 5FD51B15
P 25700 8790
F 0 "#PWR0116" H 25700 8640 50  0001 C CNN
F 1 "+12V" H 25715 8963 50  0000 C CNN
F 2 "" H 25700 8790 50  0001 C CNN
F 3 "" H 25700 8790 50  0001 C CNN
	1    25700 8790
	1    0    0    -1  
$EndComp
Wire Wire Line
	25400 9190 25500 9190
Connection ~ 26100 9090
Wire Wire Line
	24840 9110 24840 9840
Connection ~ 24840 9110
Wire Wire Line
	24710 9110 24840 9110
Wire Wire Line
	24840 9840 26100 9840
Connection ~ 24840 9840
Wire Wire Line
	24840 7880 24840 9110
Wire Wire Line
	24710 7880 24840 7880
Wire Wire Line
	22320 7360 22320 7460
Connection ~ 22320 7880
Wire Wire Line
	22320 7760 22320 7880
$Comp
L Device:R R18
U 1 1 5FD51AD7
P 22320 7610
F 0 "R18" H 22250 7564 50  0000 R CNN
F 1 "10k" H 22250 7655 50  0000 R CNN
F 2 "" V 22250 7610 50  0001 C CNN
F 3 "~" H 22320 7610 50  0001 C CNN
	1    22320 7610
	-1   0    0    -1  
$EndComp
Wire Wire Line
	26100 9090 26390 9090
Wire Wire Line
	26100 9840 26100 9090
Wire Wire Line
	24710 9840 24840 9840
Wire Wire Line
	25400 9780 25400 9190
Wire Wire Line
	25280 9780 25400 9780
Wire Wire Line
	24010 9780 24010 9840
Wire Wire Line
	24980 9780 24010 9780
$Comp
L Device:R R48
U 1 1 5FD51AC4
P 25130 9780
F 0 "R48" V 25245 9780 50  0000 C CNN
F 1 "22k" V 25336 9780 50  0000 C CNN
F 2 "" V 25060 9780 50  0001 C CNN
F 3 "~" H 25130 9780 50  0001 C CNN
	1    25130 9780
	0    -1   -1   0   
$EndComp
Wire Wire Line
	23180 9110 23180 8440
Connection ~ 23180 9110
Wire Wire Line
	23180 9840 23180 9110
Wire Wire Line
	23370 9840 23180 9840
Wire Wire Line
	24210 9680 24210 9840
Wire Wire Line
	24140 9680 24210 9680
Wire Wire Line
	24210 9840 24410 9840
Wire Wire Line
	23810 9680 23840 9680
Wire Wire Line
	23810 9840 23810 9680
Wire Wire Line
	23670 9840 23810 9840
Wire Wire Line
	23810 9290 23860 9290
Wire Wire Line
	23810 9110 23810 9290
Wire Wire Line
	24210 9290 24160 9290
Wire Wire Line
	24210 9110 24210 9290
$Comp
L Device:C C21
U 1 1 5FD51AAC
P 23990 9680
F 0 "C21" V 24242 9680 50  0000 L BNN
F 1 "2,2µ" V 24151 9680 50  0000 L BNN
F 2 "" H 24028 9530 50  0001 C CNN
F 3 "~" H 23990 9680 50  0001 C CNN
	1    23990 9680
	0    -1   -1   0   
$EndComp
Connection ~ 25400 9190
Wire Wire Line
	25400 7940 25400 9190
Wire Wire Line
	25290 7940 25400 7940
Wire Wire Line
	25290 9190 25400 9190
Wire Wire Line
	24020 7940 24990 7940
Wire Wire Line
	24010 9190 24990 9190
Wire Wire Line
	24010 9110 24010 9190
$Comp
L Device:C C22
U 1 1 5FD51A9E
P 24010 9290
F 0 "C22" V 24262 9290 50  0000 R BNN
F 1 "2,2µ" V 24171 9290 50  0000 R BNN
F 2 "" H 24048 9140 50  0001 C CNN
F 3 "~" H 24010 9290 50  0001 C CNN
	1    24010 9290
	0    -1   1    0   
$EndComp
Wire Wire Line
	24210 9110 24410 9110
Wire Wire Line
	23670 9110 23810 9110
Connection ~ 23180 8440
Wire Wire Line
	23180 9110 23370 9110
$Comp
L Device:R R40
U 1 1 5FD51A94
P 24560 9110
F 0 "R40" V 24767 9110 50  0000 C CNN
F 1 "22k" V 24676 9110 50  0000 C CNN
F 2 "" V 24490 9110 50  0001 C CNN
F 3 "~" H 24560 9110 50  0001 C CNN
	1    24560 9110
	0    -1   1    0   
$EndComp
Wire Wire Line
	24220 7880 24410 7880
Wire Wire Line
	23670 7880 23820 7880
Wire Wire Line
	23180 7880 22870 7880
Connection ~ 23180 7880
Wire Wire Line
	23180 8440 23180 7880
Wire Wire Line
	23110 8440 23180 8440
Wire Wire Line
	23370 7880 23180 7880
Wire Wire Line
	22320 7880 22570 7880
Wire Wire Line
	22320 8340 22320 7880
Wire Wire Line
	22510 8340 22320 8340
Wire Wire Line
	22370 8540 22510 8540
$Comp
L power:-12V #PWR0123
U 1 1 5FD51A6B
P 22710 8140
F 0 "#PWR0123" H 22710 8240 50  0001 C CNN
F 1 "-12V" H 22725 8313 50  0000 C CNN
F 2 "" H 22710 8140 50  0001 C CNN
F 3 "" H 22710 8140 50  0001 C CNN
	1    22710 8140
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0124
U 1 1 5FD51A65
P 22710 8740
F 0 "#PWR0124" H 22710 8590 50  0001 C CNN
F 1 "+12V" H 22725 8913 50  0000 C CNN
F 2 "" H 22710 8740 50  0001 C CNN
F 3 "" H 22710 8740 50  0001 C CNN
	1    22710 8740
	1    0    0    1   
$EndComp
$Comp
L pspice:OPAMP U7
U 1 1 5FD51A5F
P 22810 8440
F 0 "U7" H 22810 8680 50  0000 C BNN
F 1 "LM324N" H 22760 8640 50  0000 L CNN
F 2 "" H 22810 8440 50  0001 C CNN
F 3 "~" H 22810 8440 50  0001 C CNN
	1    22810 8440
	1    0    0    1   
$EndComp
$Comp
L Device:CP C39
U 1 1 5FD51A59
P 31140 11300
F 0 "C39" V 30885 11300 50  0000 C CNN
F 1 "2,2µ" V 30976 11300 50  0000 C CNN
F 2 "" H 31178 11150 50  0001 C CNN
F 3 "~" H 31140 11300 50  0001 C CNN
	1    31140 11300
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP C11
U 1 1 5FD51A4D
P 22220 8540
F 0 "C11" V 21965 8540 50  0000 C CNN
F 1 "2,2µ" V 22056 8540 50  0000 C CNN
F 2 "" H 22258 8390 50  0001 C CNN
F 3 "~" H 22220 8540 50  0001 C CNN
	1    22220 8540
	0    1    -1   0   
$EndComp
$Comp
L Device:R R26
U 1 1 5FD51A3B
P 22720 7880
F 0 "R26" V 22927 7880 50  0000 C CNN
F 1 "100k" V 22836 7880 50  0000 C CNN
F 2 "" V 22650 7880 50  0001 C CNN
F 3 "~" H 22720 7880 50  0001 C CNN
	1    22720 7880
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R39
U 1 1 5FD51A35
P 24560 7880
F 0 "R39" V 24767 7880 50  0000 C CNN
F 1 "10k" V 24676 7880 50  0000 C CNN
F 2 "" V 24490 7880 50  0001 C CNN
F 3 "~" H 24560 7880 50  0001 C CNN
	1    24560 7880
	0    -1   1    0   
$EndComp
$Comp
L Device:R R30
U 1 1 5FD51A2F
P 23520 7880
F 0 "R30" V 23727 7880 50  0000 C CNN
F 1 "10k" V 23636 7880 50  0000 C CNN
F 2 "" V 23450 7880 50  0001 C CNN
F 3 "~" H 23520 7880 50  0001 C CNN
	1    23520 7880
	0    -1   1    0   
$EndComp
$Comp
L Device:R R31
U 1 1 5FD51A29
P 23520 9110
F 0 "R31" V 23727 9110 50  0000 C CNN
F 1 "22k" V 23636 9110 50  0000 C CNN
F 2 "" V 23450 9110 50  0001 C CNN
F 3 "~" H 23520 9110 50  0001 C CNN
	1    23520 9110
	0    -1   1    0   
$EndComp
$Comp
L Device:R R41
U 1 1 5FD51A23
P 24560 9840
F 0 "R41" V 24767 9840 50  0000 C CNN
F 1 "10k" V 24676 9840 50  0000 C CNN
F 2 "" V 24490 9840 50  0001 C CNN
F 3 "~" H 24560 9840 50  0001 C CNN
	1    24560 9840
	0    -1   1    0   
$EndComp
$Comp
L Device:R R32
U 1 1 5FD51A1D
P 23520 9840
F 0 "R32" V 23405 9840 50  0000 C CNN
F 1 "10k" V 23314 9840 50  0000 C CNN
F 2 "" V 23450 9840 50  0001 C CNN
F 3 "~" H 23520 9840 50  0001 C CNN
	1    23520 9840
	0    -1   1    0   
$EndComp
$Comp
L Device:C C33
U 1 1 5FD51A17
P 26540 9090
F 0 "C33" V 26792 9090 50  0000 C CNN
F 1 "1µ" V 26701 9090 50  0000 C CNN
F 2 "" H 26578 8940 50  0001 C CNN
F 3 "~" H 26540 9090 50  0001 C CNN
	1    26540 9090
	0    -1   1    0   
$EndComp
$Comp
L Device:C C27
U 1 1 5FD51A11
P 25140 7940
F 0 "C27" V 25392 7940 50  0000 C CNN
F 1 "1,5n" V 25301 7940 50  0000 C CNN
F 2 "" H 25178 7790 50  0001 C CNN
F 3 "~" H 25140 7940 50  0001 C CNN
	1    25140 7940
	0    -1   1    0   
$EndComp
$Comp
L Device:C C28
U 1 1 5FD51A0B
P 25140 9190
F 0 "C28" V 25392 9190 50  0000 C CNN
F 1 "6,8n" V 25301 9190 50  0000 C CNN
F 2 "" H 25178 9040 50  0001 C CNN
F 3 "~" H 25140 9190 50  0001 C CNN
	1    25140 9190
	0    -1   1    0   
$EndComp
Connection ~ 25500 6720
Connection ~ 22320 6720
Wire Wire Line
	25500 6720 22320 6720
Wire Wire Line
	25500 4420 25500 6720
$Comp
L pspice:OPAMP U12
U 1 1 5FAC9461
P 25800 4320
F 0 "U12" H 26144 4366 50  0000 L BNN
F 1 "LM324N" H 26144 4275 50  0000 L TNN
F 2 "" H 25800 4320 50  0001 C CNN
F 3 "~" H 25800 4320 50  0001 C CNN
	1    25800 4320
	1    0    0    1   
$EndComp
$Comp
L power:-12V #PWR0108
U 1 1 5FAC946D
P 25700 4020
F 0 "#PWR0108" H 25700 4120 50  0001 C CNN
F 1 "-12V" H 25715 4193 50  0000 C CNN
F 2 "" H 25700 4020 50  0001 C CNN
F 3 "" H 25700 4020 50  0001 C CNN
	1    25700 4020
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0107
U 1 1 5FAC9467
P 25700 4620
F 0 "#PWR0107" H 25700 4470 50  0001 C CNN
F 1 "+12V" H 25715 4793 50  0000 C CNN
F 2 "" H 25700 4620 50  0001 C CNN
F 3 "" H 25700 4620 50  0001 C CNN
	1    25700 4620
	1    0    0    1   
$EndComp
Wire Wire Line
	25400 4220 25500 4220
Connection ~ 26100 4320
Wire Wire Line
	22320 6720 22250 6720
Wire Wire Line
	22320 6350 22320 6720
$Comp
L power:GND #PWR0131
U 1 1 5FC8D668
P 22250 6720
F 0 "#PWR0131" H 22250 6470 50  0001 C CNN
F 1 "GND" H 22255 6547 50  0000 C CNN
F 2 "" H 22250 6720 50  0001 C CNN
F 3 "" H 22250 6720 50  0001 C CNN
	1    22250 6720
	0    1    -1   0   
$EndComp
Wire Wire Line
	24840 4300 24840 3570
Connection ~ 24840 4300
Wire Wire Line
	24710 4300 24840 4300
Wire Wire Line
	24840 3570 26100 3570
Connection ~ 24840 3570
Wire Wire Line
	24840 5530 24840 4300
Wire Wire Line
	24710 5530 24840 5530
Wire Wire Line
	22320 6050 22320 5950
Connection ~ 22320 5530
Wire Wire Line
	22320 5650 22320 5530
$Comp
L Device:R R10
U 1 1 5FAC50A6
P 22320 5800
F 0 "R10" H 22250 5754 50  0000 R CNN
F 1 "10k" H 22250 5845 50  0000 R CNN
F 2 "" V 22250 5800 50  0001 C CNN
F 3 "~" H 22320 5800 50  0001 C CNN
	1    22320 5800
	-1   0    0    1   
$EndComp
Wire Wire Line
	26100 4320 26390 4320
Wire Wire Line
	26100 3570 26100 4320
Wire Wire Line
	24710 3570 24840 3570
Wire Wire Line
	25400 3630 25400 4220
Wire Wire Line
	25280 3630 25400 3630
Wire Wire Line
	24010 3630 24010 3570
Wire Wire Line
	24980 3630 24010 3630
$Comp
L Device:R R27
U 1 1 5FABD77E
P 25130 3630
F 0 "R27" V 25245 3630 50  0000 C CNN
F 1 "22k" V 25336 3630 50  0000 C CNN
F 2 "" V 25060 3630 50  0001 C CNN
F 3 "~" H 25130 3630 50  0001 C CNN
	1    25130 3630
	0    -1   1    0   
$EndComp
Wire Wire Line
	23180 4300 23180 4970
Connection ~ 23180 4300
Wire Wire Line
	23180 3570 23180 4300
Wire Wire Line
	23370 3570 23180 3570
Wire Wire Line
	24210 3730 24210 3570
Wire Wire Line
	24140 3730 24210 3730
Wire Wire Line
	24210 3570 24410 3570
Wire Wire Line
	23810 3730 23840 3730
Wire Wire Line
	23810 3570 23810 3730
Wire Wire Line
	23670 3570 23810 3570
Wire Wire Line
	23810 4120 23860 4120
Wire Wire Line
	23810 4300 23810 4120
Wire Wire Line
	24210 4120 24160 4120
Wire Wire Line
	24210 4300 24210 4120
$Comp
L Device:C C9
U 1 1 5FAB0B54
P 23990 3730
F 0 "C9" V 24242 3730 50  0000 L BNN
F 1 "2,2µ" V 24151 3730 50  0000 L BNN
F 2 "" H 24028 3580 50  0001 C CNN
F 3 "~" H 23990 3730 50  0001 C CNN
	1    23990 3730
	0    -1   1    0   
$EndComp
Connection ~ 25400 4220
Wire Wire Line
	25400 5470 25400 4220
Wire Wire Line
	25290 5470 25400 5470
Wire Wire Line
	25290 4220 25400 4220
Wire Wire Line
	24020 5470 24990 5470
Wire Wire Line
	24020 5530 24020 5470
Wire Wire Line
	24010 4220 24990 4220
Wire Wire Line
	24010 4300 24010 4220
$Comp
L Device:C C10
U 1 1 5FAB0F6A
P 24010 4120
F 0 "C10" V 24262 4120 50  0000 R BNN
F 1 "2,2µ" V 24171 4120 50  0000 R BNN
F 2 "" H 24048 3970 50  0001 C CNN
F 3 "~" H 24010 4120 50  0001 C CNN
	1    24010 4120
	0    -1   -1   0   
$EndComp
Wire Wire Line
	24210 4300 24410 4300
Wire Wire Line
	23670 4300 23810 4300
Connection ~ 23180 4970
Wire Wire Line
	23180 4300 23370 4300
$Comp
L Device:R R22
U 1 1 5FABD307
P 24560 4300
F 0 "R22" V 24767 4300 50  0000 C CNN
F 1 "22k" V 24676 4300 50  0000 C CNN
F 2 "" V 24490 4300 50  0001 C CNN
F 3 "~" H 24560 4300 50  0001 C CNN
	1    24560 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	24220 5530 24410 5530
Wire Wire Line
	23670 5530 23820 5530
Wire Wire Line
	23180 5530 22870 5530
Connection ~ 23180 5530
Wire Wire Line
	23180 4970 23180 5530
Wire Wire Line
	23110 4970 23180 4970
Wire Wire Line
	23370 5530 23180 5530
Wire Wire Line
	22320 5530 22570 5530
Wire Wire Line
	22320 5070 22320 5530
Wire Wire Line
	22510 5070 22320 5070
Wire Wire Line
	22370 4870 22510 4870
Connection ~ 21900 4870
Wire Wire Line
	22070 4870 21900 4870
Wire Wire Line
	22020 8540 21810 8540
Wire Wire Line
	21420 5180 22020 5180
Wire Wire Line
	19760 7580 19800 7580
Wire Wire Line
	19760 5970 19760 7580
Wire Wire Line
	19610 5970 19760 5970
Wire Wire Line
	19800 5870 19610 5870
Wire Wire Line
	21110 6520 21030 6520
Wire Wire Line
	21030 6720 21130 6720
Wire Wire Line
	21430 8540 21510 8540
Wire Wire Line
	21030 8540 21130 8540
Wire Wire Line
	21110 8340 21030 8340
Wire Wire Line
	20100 7580 21110 7580
$Comp
L Device:R R12
U 1 1 5FB75601
P 21660 8540
F 0 "R12" V 21867 8540 50  0000 C CNN
F 1 "22k" V 21776 8540 50  0000 C CNN
F 2 "" V 21590 8540 50  0001 C CNN
F 3 "~" H 21660 8540 50  0001 C CNN
	1    21660 8540
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 5FB755FB
P 19950 7580
F 0 "R8" V 20157 7580 50  0000 C CNN
F 1 "100k" V 20066 7580 50  0000 C CNN
F 2 "" V 19880 7580 50  0001 C CNN
F 3 "~" H 19950 7580 50  0001 C CNN
	1    19950 7580
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C4
U 1 1 5FB755F5
P 21280 8540
F 0 "C4" V 21532 8540 50  0000 C CNN
F 1 "1µ" V 21441 8540 50  0000 C CNN
F 2 "" H 21318 8390 50  0001 C CNN
F 3 "~" H 21280 8540 50  0001 C CNN
	1    21280 8540
	0    -1   -1   0   
$EndComp
Wire Wire Line
	21900 4870 21420 4870
Wire Wire Line
	21900 6720 21900 4870
Wire Wire Line
	21810 6720 21900 6720
Wire Wire Line
	21430 6720 21510 6720
Wire Wire Line
	21110 5870 21110 6520
Wire Wire Line
	20100 5870 21110 5870
$Comp
L pspice:OPAMP U1
U 1 1 5FA990D4
P 21160 3470
F 0 "U1" H 21504 3516 50  0000 L CNN
F 1 "LMV321AUIDBVR" H 21504 3425 50  0000 L CNN
F 2 "" H 21160 3470 50  0001 C CNN
F 3 "~" H 21160 3470 50  0001 C CNN
	1    21160 3470
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR0103
U 1 1 5FAA5456
P 21060 3770
F 0 "#PWR0103" H 21060 3870 50  0001 C CNN
F 1 "-12V" H 21075 3943 50  0000 C CNN
F 2 "" H 21060 3770 50  0001 C CNN
F 3 "" H 21060 3770 50  0001 C CNN
	1    21060 3770
	-1   0    0    1   
$EndComp
Wire Wire Line
	20960 4870 21120 4870
Connection ~ 20960 4870
Wire Wire Line
	20960 5180 20960 4870
Wire Wire Line
	21120 5180 20960 5180
Wire Wire Line
	20820 4870 20960 4870
Wire Wire Line
	20520 4670 21460 4670
Connection ~ 21460 4270
Wire Wire Line
	21460 4270 21460 4670
$Comp
L power:-12V #PWR0106
U 1 1 5FAC8ED7
P 22710 5270
F 0 "#PWR0106" H 22710 5370 50  0001 C CNN
F 1 "-12V" H 22725 5443 50  0000 C CNN
F 2 "" H 22710 5270 50  0001 C CNN
F 3 "" H 22710 5270 50  0001 C CNN
	1    22710 5270
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR0105
U 1 1 5FAC8ED1
P 22710 4670
F 0 "#PWR0105" H 22710 4520 50  0001 C CNN
F 1 "+12V" H 22725 4843 50  0000 C CNN
F 2 "" H 22710 4670 50  0001 C CNN
F 3 "" H 22710 4670 50  0001 C CNN
	1    22710 4670
	1    0    0    -1  
$EndComp
$Comp
L pspice:OPAMP U5
U 1 1 5FAC8ECB
P 22810 4970
F 0 "U5" H 22810 5210 50  0000 C BNN
F 1 "LM324N" H 22760 5170 50  0000 L CNN
F 2 "" H 22810 4970 50  0001 C CNN
F 3 "~" H 22810 4970 50  0001 C CNN
	1    22810 4970
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C19
U 1 1 5FAC7C1A
P 31140 11010
F 0 "C19" V 30885 11010 50  0000 C CNN
F 1 "2,2µ" V 30976 11010 50  0000 C CNN
F 2 "" H 31178 10860 50  0001 C CNN
F 3 "~" H 31140 11010 50  0001 C CNN
	1    31140 11010
	0    -1   1    0   
$EndComp
$Comp
L Device:CP C8
U 1 1 5FAC7702
P 22320 6200
F 0 "C8" H 22438 6246 50  0000 L CNN
F 1 "4,7µ" H 22438 6155 50  0000 L CNN
F 2 "" H 22358 6050 50  0001 C CNN
F 3 "~" H 22320 6200 50  0001 C CNN
	1    22320 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C6
U 1 1 5FAC70EF
P 22220 4870
F 0 "C6" V 21965 4870 50  0000 C CNN
F 1 "2,2µ" V 22056 4870 50  0000 C CNN
F 2 "" H 22258 4720 50  0001 C CNN
F 3 "~" H 22220 4870 50  0001 C CNN
	1    22220 4870
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5FAC4B34
P 22720 5530
F 0 "R11" V 22927 5530 50  0000 C CNN
F 1 "100k" V 22836 5530 50  0000 C CNN
F 2 "" V 22650 5530 50  0001 C CNN
F 3 "~" H 22720 5530 50  0001 C CNN
	1    22720 5530
	0    -1   1    0   
$EndComp
$Comp
L Device:R R23
U 1 1 5FAC46C9
P 24560 5530
F 0 "R23" V 24767 5530 50  0000 C CNN
F 1 "10k" V 24676 5530 50  0000 C CNN
F 2 "" V 24490 5530 50  0001 C CNN
F 3 "~" H 24560 5530 50  0001 C CNN
	1    24560 5530
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R13
U 1 1 5FAC36AB
P 23520 5530
F 0 "R13" V 23727 5530 50  0000 C CNN
F 1 "10k" V 23636 5530 50  0000 C CNN
F 2 "" V 23450 5530 50  0001 C CNN
F 3 "~" H 23520 5530 50  0001 C CNN
	1    23520 5530
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R14
U 1 1 5FABCCC1
P 23520 4300
F 0 "R14" V 23727 4300 50  0000 C CNN
F 1 "22k" V 23636 4300 50  0000 C CNN
F 2 "" V 23450 4300 50  0001 C CNN
F 3 "~" H 23520 4300 50  0001 C CNN
	1    23520 4300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R21
U 1 1 5FABC768
P 24560 3570
F 0 "R21" V 24767 3570 50  0000 C CNN
F 1 "10k" V 24676 3570 50  0000 C CNN
F 2 "" V 24490 3570 50  0001 C CNN
F 3 "~" H 24560 3570 50  0001 C CNN
	1    24560 3570
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R15
U 1 1 5FABC217
P 23520 3570
F 0 "R15" V 23405 3570 50  0000 C CNN
F 1 "10k" V 23314 3570 50  0000 C CNN
F 2 "" V 23450 3570 50  0001 C CNN
F 3 "~" H 23520 3570 50  0001 C CNN
	1    23520 3570
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R7
U 1 1 5FABB1A6
P 21660 6720
F 0 "R7" V 21867 6720 50  0000 C CNN
F 1 "22k" V 21776 6720 50  0000 C CNN
F 2 "" V 21590 6720 50  0001 C CNN
F 3 "~" H 21660 6720 50  0001 C CNN
	1    21660 6720
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 5FABA4E3
P 21270 5180
F 0 "R6" V 21477 5180 50  0000 C CNN
F 1 "100k" V 21386 5180 50  0000 C CNN
F 2 "" V 21200 5180 50  0001 C CNN
F 3 "~" H 21270 5180 50  0001 C CNN
	1    21270 5180
	0    -1   1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5FAB9FFE
P 21270 4870
F 0 "R5" V 21477 4870 50  0000 C CNN
F 1 "100k" V 21386 4870 50  0000 C CNN
F 2 "" V 21200 4870 50  0001 C CNN
F 3 "~" H 21270 4870 50  0001 C CNN
	1    21270 4870
	0    -1   1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5FAB9128
P 19950 5870
F 0 "R3" V 20157 5870 50  0000 C CNN
F 1 "100k" V 20066 5870 50  0000 C CNN
F 2 "" V 19880 5870 50  0001 C CNN
F 3 "~" H 19950 5870 50  0001 C CNN
	1    19950 5870
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP C1
U 1 1 5FAB3E5C
P 19910 3570
F 0 "C1" V 19655 3570 50  0000 C CNN
F 1 "2,2µ" V 19746 3570 50  0000 C CNN
F 2 "" H 19948 3420 50  0001 C CNN
F 3 "~" H 19910 3570 50  0001 C CNN
	1    19910 3570
	0    1    1    0   
$EndComp
$Comp
L Device:C C17
U 1 1 5FAB1BA8
P 26540 4320
F 0 "C17" V 26792 4320 50  0000 C CNN
F 1 "1µ" V 26701 4320 50  0000 C CNN
F 2 "" H 26578 4170 50  0001 C CNN
F 3 "~" H 26540 4320 50  0001 C CNN
	1    26540 4320
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C14
U 1 1 5FAB1751
P 25140 5470
F 0 "C14" V 25392 5470 50  0000 C CNN
F 1 "1,5n" V 25301 5470 50  0000 C CNN
F 2 "" H 25178 5320 50  0001 C CNN
F 3 "~" H 25140 5470 50  0001 C CNN
	1    25140 5470
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C13
U 1 1 5FAB133F
P 25140 4220
F 0 "C13" V 25392 4220 50  0000 C CNN
F 1 "6,8n" V 25301 4220 50  0000 C CNN
F 2 "" H 25178 4070 50  0001 C CNN
F 3 "~" H 25140 4220 50  0001 C CNN
	1    25140 4220
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C3
U 1 1 5FAB05EC
P 21280 6720
F 0 "C3" V 21532 6720 50  0000 C CNN
F 1 "1µ" V 21441 6720 50  0000 C CNN
F 2 "" H 21318 6570 50  0001 C CNN
F 3 "~" H 21280 6720 50  0001 C CNN
	1    21280 6720
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5FAB00C5
P 20670 4870
F 0 "C2" V 20922 4870 50  0000 C CNN
F 1 "1µ" V 20831 4870 50  0000 C CNN
F 2 "" H 20708 4720 50  0001 C CNN
F 3 "~" H 20670 4870 50  0001 C CNN
	1    20670 4870
	0    -1   -1   0   
$EndComp
Wire Wire Line
	20060 3570 20270 3570
Wire Wire Line
	21460 4270 21460 3470
Wire Wire Line
	21210 4270 21460 4270
Wire Wire Line
	20720 3370 20860 3370
Wire Wire Line
	20720 3280 20720 3370
Wire Wire Line
	20720 3570 20860 3570
Connection ~ 20720 3570
Wire Wire Line
	20720 4270 20720 3570
Wire Wire Line
	20910 4270 20720 4270
Wire Wire Line
	20570 3570 20720 3570
$Comp
L power:GND #PWR0104
U 1 1 5FAA7B85
P 20720 3280
F 0 "#PWR0104" H 20720 3030 50  0001 C CNN
F 1 "GND" H 20725 3107 50  0000 C CNN
F 2 "" H 20720 3280 50  0001 C CNN
F 3 "" H 20720 3280 50  0001 C CNN
	1    20720 3280
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5FAA6D79
P 21060 4270
F 0 "R2" V 21267 4270 50  0000 C CNN
F 1 "100k" V 21176 4270 50  0000 C CNN
F 2 "" V 20990 4270 50  0001 C CNN
F 3 "~" H 21060 4270 50  0001 C CNN
	1    21060 4270
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0102
U 1 1 5FAA4930
P 21060 3170
F 0 "#PWR0102" H 21060 3020 50  0001 C CNN
F 1 "+12V" H 21075 3343 50  0000 C CNN
F 2 "" H 21060 3170 50  0001 C CNN
F 3 "" H 21060 3170 50  0001 C CNN
	1    21060 3170
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5FA7471D
P 20420 3570
F 0 "R1" V 20627 3570 50  0000 C CNN
F 1 "1k" V 20536 3570 50  0000 C CNN
F 2 "" V 20350 3570 50  0001 C CNN
F 3 "~" H 20420 3570 50  0001 C CNN
	1    20420 3570
	0    -1   -1   0   
$EndComp
Wire Wire Line
	25540 18120 25540 15850
Wire Wire Line
	22360 16190 22360 15850
$Comp
L Device:CP C20
U 1 1 5FB3C080
P 22360 16340
F 0 "C20" H 22478 16386 50  0000 L CNN
F 1 "4,7µ" H 22478 16295 50  0000 L CNN
F 2 "" H 22398 16190 50  0001 C CNN
F 3 "~" H 22360 16340 50  0001 C CNN
	1    22360 16340
	1    0    0    1   
$EndComp
$Comp
L pspice:OPAMP U26
U 1 1 5FB3C0AB
P 25840 18220
F 0 "U26" H 26184 18266 50  0000 L BNN
F 1 "LM324N" H 26184 18175 50  0000 L TNN
F 2 "" H 25840 18220 50  0001 C CNN
F 3 "~" H 25840 18220 50  0001 C CNN
	1    25840 18220
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR0156
U 1 1 5FB3C0B1
P 25740 18520
F 0 "#PWR0156" H 25740 18620 50  0001 C CNN
F 1 "-12V" H 25755 18693 50  0000 C CNN
F 2 "" H 25740 18520 50  0001 C CNN
F 3 "" H 25740 18520 50  0001 C CNN
	1    25740 18520
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR0157
U 1 1 5FB3C0B7
P 25740 17920
F 0 "#PWR0157" H 25740 17770 50  0001 C CNN
F 1 "+12V" H 25755 18093 50  0000 C CNN
F 2 "" H 25740 17920 50  0001 C CNN
F 3 "" H 25740 17920 50  0001 C CNN
	1    25740 17920
	1    0    0    -1  
$EndComp
Wire Wire Line
	25440 18320 25540 18320
Connection ~ 26140 18220
Wire Wire Line
	24880 18240 24880 18970
Connection ~ 24880 18240
Wire Wire Line
	24750 18240 24880 18240
Wire Wire Line
	24880 18970 26140 18970
Connection ~ 24880 18970
Wire Wire Line
	24880 17010 24880 18240
Wire Wire Line
	24750 17010 24880 17010
Wire Wire Line
	22360 16490 22360 16590
Connection ~ 22360 17010
Wire Wire Line
	22360 16890 22360 17010
$Comp
L Device:R R25
U 1 1 5FB3C0ED
P 22360 16740
F 0 "R25" H 22290 16694 50  0000 R CNN
F 1 "10k" H 22290 16785 50  0000 R CNN
F 2 "" V 22290 16740 50  0001 C CNN
F 3 "~" H 22360 16740 50  0001 C CNN
	1    22360 16740
	-1   0    0    -1  
$EndComp
Wire Wire Line
	26140 18220 26430 18220
Wire Wire Line
	26140 18970 26140 18220
Wire Wire Line
	24750 18970 24880 18970
Wire Wire Line
	25440 18910 25440 18320
Wire Wire Line
	25320 18910 25440 18910
Wire Wire Line
	24050 18910 24050 18970
Wire Wire Line
	25020 18910 24050 18910
$Comp
L Device:R R50
U 1 1 5FB3C100
P 25170 18910
F 0 "R50" V 25285 18910 50  0000 C CNN
F 1 "22k" V 25376 18910 50  0000 C CNN
F 2 "" V 25100 18910 50  0001 C CNN
F 3 "~" H 25170 18910 50  0001 C CNN
	1    25170 18910
	0    -1   -1   0   
$EndComp
Wire Wire Line
	23220 18240 23220 17570
Connection ~ 23220 18240
Wire Wire Line
	23220 18970 23220 18240
Wire Wire Line
	23410 18970 23220 18970
Wire Wire Line
	24250 18810 24250 18970
Wire Wire Line
	24180 18810 24250 18810
Wire Wire Line
	24250 18970 24450 18970
Wire Wire Line
	23850 18810 23880 18810
Wire Wire Line
	23850 18970 23850 18810
Wire Wire Line
	23710 18970 23850 18970
Wire Wire Line
	23850 18420 23900 18420
Wire Wire Line
	23850 18240 23850 18420
Wire Wire Line
	24250 18420 24200 18420
Wire Wire Line
	24250 18240 24250 18420
$Comp
L Device:C C24
U 1 1 5FB3C118
P 24030 18810
F 0 "C24" V 24282 18810 50  0000 L BNN
F 1 "2,2µ" V 24191 18810 50  0000 L BNN
F 2 "" H 24068 18660 50  0001 C CNN
F 3 "~" H 24030 18810 50  0001 C CNN
	1    24030 18810
	0    -1   -1   0   
$EndComp
Connection ~ 25440 18320
Wire Wire Line
	25440 17070 25440 18320
Wire Wire Line
	25330 17070 25440 17070
Wire Wire Line
	25330 18320 25440 18320
Wire Wire Line
	24060 17070 25030 17070
Wire Wire Line
	24060 17010 24060 17070
Wire Wire Line
	24050 18320 25030 18320
Wire Wire Line
	24050 18240 24050 18320
$Comp
L Device:C C26
U 1 1 5FB3C126
P 24050 18420
F 0 "C26" V 24302 18420 50  0000 R BNN
F 1 "2,2µ" V 24211 18420 50  0000 R BNN
F 2 "" H 24088 18270 50  0001 C CNN
F 3 "~" H 24050 18420 50  0001 C CNN
	1    24050 18420
	0    -1   1    0   
$EndComp
Wire Wire Line
	24250 18240 24450 18240
Wire Wire Line
	23710 18240 23850 18240
Connection ~ 23220 17570
Wire Wire Line
	23220 18240 23410 18240
$Comp
L Device:R R46
U 1 1 5FB3C130
P 24600 18240
F 0 "R46" V 24807 18240 50  0000 C CNN
F 1 "22k" V 24716 18240 50  0000 C CNN
F 2 "" V 24530 18240 50  0001 C CNN
F 3 "~" H 24600 18240 50  0001 C CNN
	1    24600 18240
	0    -1   1    0   
$EndComp
Wire Wire Line
	24260 17010 24450 17010
Wire Wire Line
	23710 17010 23860 17010
Wire Wire Line
	23220 17010 22910 17010
Connection ~ 23220 17010
Wire Wire Line
	23220 17570 23220 17010
Wire Wire Line
	23150 17570 23220 17570
Wire Wire Line
	23410 17010 23220 17010
Wire Wire Line
	22360 17010 22610 17010
Wire Wire Line
	22360 17470 22360 17010
Wire Wire Line
	22550 17470 22360 17470
Wire Wire Line
	22410 17670 22550 17670
$Comp
L power:-12V #PWR0164
U 1 1 5FB3C153
P 22750 17270
F 0 "#PWR0164" H 22750 17370 50  0001 C CNN
F 1 "-12V" H 22765 17443 50  0000 C CNN
F 2 "" H 22750 17270 50  0001 C CNN
F 3 "" H 22750 17270 50  0001 C CNN
	1    22750 17270
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0165
U 1 1 5FB3C159
P 22750 17870
F 0 "#PWR0165" H 22750 17720 50  0001 C CNN
F 1 "+12V" H 22765 18043 50  0000 C CNN
F 2 "" H 22750 17870 50  0001 C CNN
F 3 "" H 22750 17870 50  0001 C CNN
	1    22750 17870
	1    0    0    1   
$EndComp
$Comp
L pspice:OPAMP U10
U 1 1 5FB3C15F
P 22850 17570
F 0 "U10" H 22850 17810 50  0000 C BNN
F 1 "LM324N" H 22800 17770 50  0000 L CNN
F 2 "" H 22850 17570 50  0001 C CNN
F 3 "~" H 22850 17570 50  0001 C CNN
	1    22850 17570
	1    0    0    1   
$EndComp
$Comp
L Device:CP C15
U 1 1 5FB3C16B
P 22260 17670
F 0 "C15" V 22005 17670 50  0000 C CNN
F 1 "2,2µ" V 22096 17670 50  0000 C CNN
F 2 "" H 22298 17520 50  0001 C CNN
F 3 "~" H 22260 17670 50  0001 C CNN
	1    22260 17670
	0    1    -1   0   
$EndComp
$Comp
L Device:R R29
U 1 1 5FB3C17D
P 22760 17010
F 0 "R29" V 22967 17010 50  0000 C CNN
F 1 "100k" V 22876 17010 50  0000 C CNN
F 2 "" V 22690 17010 50  0001 C CNN
F 3 "~" H 22760 17010 50  0001 C CNN
	1    22760 17010
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R45
U 1 1 5FB3C183
P 24600 17010
F 0 "R45" V 24807 17010 50  0000 C CNN
F 1 "10k" V 24716 17010 50  0000 C CNN
F 2 "" V 24530 17010 50  0001 C CNN
F 3 "~" H 24600 17010 50  0001 C CNN
	1    24600 17010
	0    -1   1    0   
$EndComp
$Comp
L Device:R R36
U 1 1 5FB3C189
P 23560 17010
F 0 "R36" V 23767 17010 50  0000 C CNN
F 1 "10k" V 23676 17010 50  0000 C CNN
F 2 "" V 23490 17010 50  0001 C CNN
F 3 "~" H 23560 17010 50  0001 C CNN
	1    23560 17010
	0    -1   1    0   
$EndComp
$Comp
L Device:R R37
U 1 1 5FB3C18F
P 23560 18240
F 0 "R37" V 23767 18240 50  0000 C CNN
F 1 "22k" V 23676 18240 50  0000 C CNN
F 2 "" V 23490 18240 50  0001 C CNN
F 3 "~" H 23560 18240 50  0001 C CNN
	1    23560 18240
	0    -1   1    0   
$EndComp
$Comp
L Device:R R47
U 1 1 5FB3C195
P 24600 18970
F 0 "R47" V 24807 18970 50  0000 C CNN
F 1 "10k" V 24716 18970 50  0000 C CNN
F 2 "" V 24530 18970 50  0001 C CNN
F 3 "~" H 24600 18970 50  0001 C CNN
	1    24600 18970
	0    -1   1    0   
$EndComp
$Comp
L Device:R R38
U 1 1 5FB3C19B
P 23560 18970
F 0 "R38" V 23445 18970 50  0000 C CNN
F 1 "10k" V 23354 18970 50  0000 C CNN
F 2 "" V 23490 18970 50  0001 C CNN
F 3 "~" H 23560 18970 50  0001 C CNN
	1    23560 18970
	0    -1   1    0   
$EndComp
$Comp
L Device:C C35
U 1 1 5FB3C1A1
P 26580 18220
F 0 "C35" V 26832 18220 50  0000 C CNN
F 1 "1µ" V 26741 18220 50  0000 C CNN
F 2 "" H 26618 18070 50  0001 C CNN
F 3 "~" H 26580 18220 50  0001 C CNN
	1    26580 18220
	0    -1   1    0   
$EndComp
$Comp
L Device:C C31
U 1 1 5FB3C1A7
P 25180 17070
F 0 "C31" V 25432 17070 50  0000 C CNN
F 1 "1,5n" V 25341 17070 50  0000 C CNN
F 2 "" H 25218 16920 50  0001 C CNN
F 3 "~" H 25180 17070 50  0001 C CNN
	1    25180 17070
	0    -1   1    0   
$EndComp
$Comp
L Device:C C32
U 1 1 5FB3C1AD
P 25180 18320
F 0 "C32" V 25432 18320 50  0000 C CNN
F 1 "6,8n" V 25341 18320 50  0000 C CNN
F 2 "" H 25218 18170 50  0001 C CNN
F 3 "~" H 25180 18320 50  0001 C CNN
	1    25180 18320
	0    -1   1    0   
$EndComp
Connection ~ 25540 15850
Connection ~ 22360 15850
Wire Wire Line
	25540 15850 22360 15850
Wire Wire Line
	25540 13550 25540 15850
$Comp
L pspice:OPAMP U25
U 1 1 5FB3C1F3
P 25840 13450
F 0 "U25" H 26184 13496 50  0000 L BNN
F 1 "LM324N" H 26184 13405 50  0000 L TNN
F 2 "" H 25840 13450 50  0001 C CNN
F 3 "~" H 25840 13450 50  0001 C CNN
	1    25840 13450
	1    0    0    1   
$EndComp
$Comp
L power:-12V #PWR0171
U 1 1 5FB3C1F9
P 25740 13150
F 0 "#PWR0171" H 25740 13250 50  0001 C CNN
F 1 "-12V" H 25755 13323 50  0000 C CNN
F 2 "" H 25740 13150 50  0001 C CNN
F 3 "" H 25740 13150 50  0001 C CNN
	1    25740 13150
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0172
U 1 1 5FB3C1FF
P 25740 13750
F 0 "#PWR0172" H 25740 13600 50  0001 C CNN
F 1 "+12V" H 25755 13923 50  0000 C CNN
F 2 "" H 25740 13750 50  0001 C CNN
F 3 "" H 25740 13750 50  0001 C CNN
	1    25740 13750
	1    0    0    1   
$EndComp
Wire Wire Line
	25440 13350 25540 13350
Connection ~ 26140 13450
Wire Wire Line
	22360 15850 22290 15850
Wire Wire Line
	22360 15480 22360 15850
$Comp
L power:GND #PWR0173
U 1 1 5FB3C209
P 22290 15850
F 0 "#PWR0173" H 22290 15600 50  0001 C CNN
F 1 "GND" H 22295 15677 50  0000 C CNN
F 2 "" H 22290 15850 50  0001 C CNN
F 3 "" H 22290 15850 50  0001 C CNN
	1    22290 15850
	0    1    -1   0   
$EndComp
Wire Wire Line
	24880 13430 24880 12700
Connection ~ 24880 13430
Wire Wire Line
	24750 13430 24880 13430
Wire Wire Line
	24880 12700 26140 12700
Connection ~ 24880 12700
Wire Wire Line
	24880 14660 24880 13430
Wire Wire Line
	24750 14660 24880 14660
Wire Wire Line
	22360 15180 22360 15080
Connection ~ 22360 14660
Wire Wire Line
	22360 14780 22360 14660
$Comp
L Device:R R24
U 1 1 5FB3C23D
P 22360 14930
F 0 "R24" H 22290 14884 50  0000 R CNN
F 1 "10k" H 22290 14975 50  0000 R CNN
F 2 "" V 22290 14930 50  0001 C CNN
F 3 "~" H 22360 14930 50  0001 C CNN
	1    22360 14930
	-1   0    0    1   
$EndComp
Wire Wire Line
	26140 13450 26430 13450
Wire Wire Line
	26140 12700 26140 13450
Wire Wire Line
	24750 12700 24880 12700
Wire Wire Line
	25440 12760 25440 13350
Wire Wire Line
	25320 12760 25440 12760
Wire Wire Line
	24050 12760 24050 12700
Wire Wire Line
	25020 12760 24050 12760
$Comp
L Device:R R49
U 1 1 5FB3C250
P 25170 12760
F 0 "R49" V 25285 12760 50  0000 C CNN
F 1 "22k" V 25376 12760 50  0000 C CNN
F 2 "" V 25100 12760 50  0001 C CNN
F 3 "~" H 25170 12760 50  0001 C CNN
	1    25170 12760
	0    -1   1    0   
$EndComp
Wire Wire Line
	23220 13430 23220 14100
Connection ~ 23220 13430
Wire Wire Line
	23220 12700 23220 13430
Wire Wire Line
	23410 12700 23220 12700
Wire Wire Line
	24250 12860 24250 12700
Wire Wire Line
	24180 12860 24250 12860
Wire Wire Line
	24250 12700 24450 12700
Wire Wire Line
	23850 12860 23880 12860
Wire Wire Line
	23850 12700 23850 12860
Wire Wire Line
	23710 12700 23850 12700
Wire Wire Line
	23850 13250 23900 13250
Wire Wire Line
	23850 13430 23850 13250
Wire Wire Line
	24250 13250 24200 13250
Wire Wire Line
	24250 13430 24250 13250
$Comp
L Device:C C23
U 1 1 5FB3C268
P 24030 12860
F 0 "C23" V 24282 12860 50  0000 L BNN
F 1 "2,2µ" V 24191 12860 50  0000 L BNN
F 2 "" H 24068 12710 50  0001 C CNN
F 3 "~" H 24030 12860 50  0001 C CNN
	1    24030 12860
	0    -1   1    0   
$EndComp
Connection ~ 25440 13350
Wire Wire Line
	25440 14600 25440 13350
Wire Wire Line
	25330 14600 25440 14600
Wire Wire Line
	25330 13350 25440 13350
Wire Wire Line
	24060 14600 25030 14600
Wire Wire Line
	24060 14660 24060 14600
Wire Wire Line
	24050 13350 25030 13350
Wire Wire Line
	24050 13430 24050 13350
$Comp
L Device:C C25
U 1 1 5FB3C276
P 24050 13250
F 0 "C25" V 24302 13250 50  0000 R BNN
F 1 "2,2µ" V 24211 13250 50  0000 R BNN
F 2 "" H 24088 13100 50  0001 C CNN
F 3 "~" H 24050 13250 50  0001 C CNN
	1    24050 13250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	24250 13430 24450 13430
Wire Wire Line
	23710 13430 23850 13430
Connection ~ 23220 14100
Wire Wire Line
	23220 13430 23410 13430
$Comp
L Device:R R43
U 1 1 5FB3C280
P 24600 13430
F 0 "R43" V 24807 13430 50  0000 C CNN
F 1 "22k" V 24716 13430 50  0000 C CNN
F 2 "" V 24530 13430 50  0001 C CNN
F 3 "~" H 24600 13430 50  0001 C CNN
	1    24600 13430
	0    -1   -1   0   
$EndComp
Wire Wire Line
	24260 14660 24450 14660
Wire Wire Line
	23710 14660 23860 14660
Wire Wire Line
	23220 14660 22910 14660
Connection ~ 23220 14660
Wire Wire Line
	23220 14100 23220 14660
Wire Wire Line
	23150 14100 23220 14100
Wire Wire Line
	23410 14660 23220 14660
Wire Wire Line
	22360 14660 22610 14660
Wire Wire Line
	22360 14200 22360 14660
Wire Wire Line
	22550 14200 22360 14200
Wire Wire Line
	22410 14000 22550 14000
Wire Wire Line
	22110 14000 21940 14000
Wire Wire Line
	19800 16710 19840 16710
Wire Wire Line
	19800 15100 19800 16710
Wire Wire Line
	19650 15100 19800 15100
Wire Wire Line
	19840 15000 19650 15000
Wire Wire Line
	21150 15650 21070 15650
Wire Wire Line
	21070 15850 21170 15850
Wire Wire Line
	21070 17670 21170 17670
Wire Wire Line
	21150 17470 21070 17470
Wire Wire Line
	20140 16710 21150 16710
$Comp
L Device:R R17
U 1 1 5FB3C2D2
P 21800 17670
F 0 "R17" V 22007 17670 50  0000 C CNN
F 1 "22k" V 21916 17670 50  0000 C CNN
F 2 "" V 21730 17670 50  0001 C CNN
F 3 "~" H 21800 17670 50  0001 C CNN
	1    21800 17670
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R9
U 1 1 5FB3C2D8
P 19990 16710
F 0 "R9" V 20197 16710 50  0000 C CNN
F 1 "100k" V 20106 16710 50  0000 C CNN
F 2 "" V 19920 16710 50  0001 C CNN
F 3 "~" H 19990 16710 50  0001 C CNN
	1    19990 16710
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C7
U 1 1 5FB3C2DE
P 21320 17670
F 0 "C7" V 21572 17670 50  0000 C CNN
F 1 "1µ" V 21481 17670 50  0000 C CNN
F 2 "" H 21358 17520 50  0001 C CNN
F 3 "~" H 21320 17670 50  0001 C CNN
	1    21320 17670
	0    -1   -1   0   
$EndComp
Wire Wire Line
	21940 15850 21940 14000
Wire Wire Line
	21850 15850 21940 15850
Wire Wire Line
	21470 15850 21550 15850
Wire Wire Line
	21150 15000 21150 15650
Wire Wire Line
	20140 15000 21150 15000
$Comp
L power:-12V #PWR0188
U 1 1 5FB3C350
P 22750 14400
F 0 "#PWR0188" H 22750 14500 50  0001 C CNN
F 1 "-12V" H 22765 14573 50  0000 C CNN
F 2 "" H 22750 14400 50  0001 C CNN
F 3 "" H 22750 14400 50  0001 C CNN
	1    22750 14400
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR0189
U 1 1 5FB3C356
P 22750 13800
F 0 "#PWR0189" H 22750 13650 50  0001 C CNN
F 1 "+12V" H 22765 13973 50  0000 C CNN
F 2 "" H 22750 13800 50  0001 C CNN
F 3 "" H 22750 13800 50  0001 C CNN
	1    22750 13800
	1    0    0    -1  
$EndComp
$Comp
L pspice:OPAMP U9
U 1 1 5FB3C35C
P 22850 14100
F 0 "U9" H 22850 14340 50  0000 C BNN
F 1 "LM324N" H 22800 14300 50  0000 L CNN
F 2 "" H 22850 14100 50  0001 C CNN
F 3 "~" H 22850 14100 50  0001 C CNN
	1    22850 14100
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C18
U 1 1 5FB3C368
P 22360 15330
F 0 "C18" H 22478 15376 50  0000 L CNN
F 1 "4,7µ" H 22478 15285 50  0000 L CNN
F 2 "" H 22398 15180 50  0001 C CNN
F 3 "~" H 22360 15330 50  0001 C CNN
	1    22360 15330
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C12
U 1 1 5FB3C36E
P 22260 14000
F 0 "C12" V 22005 14000 50  0000 C CNN
F 1 "2,2µ" V 22096 14000 50  0000 C CNN
F 2 "" H 22298 13850 50  0001 C CNN
F 3 "~" H 22260 14000 50  0001 C CNN
	1    22260 14000
	0    1    1    0   
$EndComp
$Comp
L Device:R R55
U 1 1 5FB3C374
P 29300 10580
F 0 "R55" H 29230 10534 50  0000 R CNN
F 1 "100" H 29230 10625 50  0000 R CNN
F 2 "" V 29230 10580 50  0001 C CNN
F 3 "~" H 29300 10580 50  0001 C CNN
	1    29300 10580
	0    1    1    0   
$EndComp
$Comp
L Device:R R28
U 1 1 5FB3C380
P 22760 14660
F 0 "R28" V 22967 14660 50  0000 C CNN
F 1 "100k" V 22876 14660 50  0000 C CNN
F 2 "" V 22690 14660 50  0001 C CNN
F 3 "~" H 22760 14660 50  0001 C CNN
	1    22760 14660
	0    -1   1    0   
$EndComp
$Comp
L Device:R R44
U 1 1 5FB3C386
P 24600 14660
F 0 "R44" V 24807 14660 50  0000 C CNN
F 1 "10k" V 24716 14660 50  0000 C CNN
F 2 "" V 24530 14660 50  0001 C CNN
F 3 "~" H 24600 14660 50  0001 C CNN
	1    24600 14660
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R35
U 1 1 5FB3C38C
P 23560 14660
F 0 "R35" V 23767 14660 50  0000 C CNN
F 1 "10k" V 23676 14660 50  0000 C CNN
F 2 "" V 23490 14660 50  0001 C CNN
F 3 "~" H 23560 14660 50  0001 C CNN
	1    23560 14660
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R34
U 1 1 5FB3C392
P 23560 13430
F 0 "R34" V 23767 13430 50  0000 C CNN
F 1 "22k" V 23676 13430 50  0000 C CNN
F 2 "" V 23490 13430 50  0001 C CNN
F 3 "~" H 23560 13430 50  0001 C CNN
	1    23560 13430
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R42
U 1 1 5FB3C398
P 24600 12700
F 0 "R42" V 24807 12700 50  0000 C CNN
F 1 "10k" V 24716 12700 50  0000 C CNN
F 2 "" V 24530 12700 50  0001 C CNN
F 3 "~" H 24600 12700 50  0001 C CNN
	1    24600 12700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R33
U 1 1 5FB3C39E
P 23560 12700
F 0 "R33" V 23445 12700 50  0000 C CNN
F 1 "10k" V 23354 12700 50  0000 C CNN
F 2 "" V 23490 12700 50  0001 C CNN
F 3 "~" H 23560 12700 50  0001 C CNN
	1    23560 12700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R16
U 1 1 5FB3C3A4
P 21700 15850
F 0 "R16" V 21907 15850 50  0000 C CNN
F 1 "22k" V 21816 15850 50  0000 C CNN
F 2 "" V 21630 15850 50  0001 C CNN
F 3 "~" H 21700 15850 50  0001 C CNN
	1    21700 15850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 5FB3C3B6
P 19990 15000
F 0 "R4" V 20197 15000 50  0000 C CNN
F 1 "100k" V 20106 15000 50  0000 C CNN
F 2 "" V 19920 15000 50  0001 C CNN
F 3 "~" H 19990 15000 50  0001 C CNN
	1    19990 15000
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C34
U 1 1 5FB3C3C2
P 26580 13450
F 0 "C34" V 26832 13450 50  0000 C CNN
F 1 "1µ" V 26741 13450 50  0000 C CNN
F 2 "" H 26618 13300 50  0001 C CNN
F 3 "~" H 26580 13450 50  0001 C CNN
	1    26580 13450
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C30
U 1 1 5FB3C3C8
P 25180 14600
F 0 "C30" V 25432 14600 50  0000 C CNN
F 1 "1,5n" V 25341 14600 50  0000 C CNN
F 2 "" H 25218 14450 50  0001 C CNN
F 3 "~" H 25180 14600 50  0001 C CNN
	1    25180 14600
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C29
U 1 1 5FB3C3CE
P 25180 13350
F 0 "C29" V 25432 13350 50  0000 C CNN
F 1 "6,8n" V 25341 13350 50  0000 C CNN
F 2 "" H 25218 13200 50  0001 C CNN
F 3 "~" H 25180 13350 50  0001 C CNN
	1    25180 13350
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C5
U 1 1 5FB3C3D4
P 21320 15850
F 0 "C5" V 21572 15850 50  0000 C CNN
F 1 "1µ" V 21481 15850 50  0000 C CNN
F 2 "" H 21358 15700 50  0001 C CNN
F 3 "~" H 21320 15850 50  0001 C CNN
	1    21320 15850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	31550 11200 31550 11010
Wire Wire Line
	31550 11010 31290 11010
Wire Wire Line
	22070 8540 22020 8540
$Comp
L Device:R R52
U 1 1 5FB3C37A
P 26730 12380
F 0 "R52" H 26660 12334 50  0000 R CNN
F 1 "200" H 26660 12425 50  0000 R CNN
F 2 "" V 26660 12380 50  0001 C CNN
F 3 "~" H 26730 12380 50  0001 C CNN
	1    26730 12380
	-1   0    0    1   
$EndComp
$Comp
L Device:R R56
U 1 1 5FB3C171
P 29300 12030
F 0 "R56" H 29230 11984 50  0000 R CNN
F 1 "100" H 29230 12075 50  0000 R CNN
F 2 "" V 29230 12030 50  0001 C CNN
F 3 "~" H 29300 12030 50  0001 C CNN
	1    29300 12030
	0    -1   1    0   
$EndComp
$Comp
L Device:R R54
U 1 1 60F9667D
P 27390 12380
F 0 "R54" H 27320 12334 50  0000 R CNN
F 1 "200" H 27320 12425 50  0000 R CNN
F 2 "" V 27320 12380 50  0001 C CNN
F 3 "~" H 27390 12380 50  0001 C CNN
	1    27390 12380
	-1   0    0    1   
$EndComp
$Comp
L Device:R R51
U 1 1 610AA565
P 26730 10180
F 0 "R51" H 26660 10134 50  0000 R CNN
F 1 "200" H 26660 10225 50  0000 R CNN
F 2 "" V 26660 10180 50  0001 C CNN
F 3 "~" H 26730 10180 50  0001 C CNN
	1    26730 10180
	-1   0    0    1   
$EndComp
$Comp
L Device:R R53
U 1 1 611070A8
P 27390 10180
F 0 "R53" H 27320 10134 50  0000 R CNN
F 1 "200" H 27320 10225 50  0000 R CNN
F 2 "" V 27320 10180 50  0001 C CNN
F 3 "~" H 27390 10180 50  0001 C CNN
	1    27390 10180
	-1   0    0    1   
$EndComp
Wire Wire Line
	26730 13450 26730 12530
Wire Wire Line
	26690 9090 27390 9090
Wire Wire Line
	27390 9090 27390 10030
Wire Wire Line
	26730 10030 26730 4320
Wire Wire Line
	26730 4320 26690 4320
Wire Wire Line
	26730 10330 26730 10580
Wire Wire Line
	26730 18220 27390 18220
Wire Wire Line
	27390 18220 27390 12530
Wire Wire Line
	29150 12030 27390 12030
Connection ~ 27390 12030
Wire Wire Line
	27390 12030 27390 12230
Wire Wire Line
	29150 10580 26730 10580
Connection ~ 26730 10580
Wire Wire Line
	30780 11010 30990 11010
Wire Wire Line
	30990 11300 30780 11300
Connection ~ 22020 8540
Wire Wire Line
	22020 5180 22020 8540
Wire Wire Line
	21110 7580 21110 8340
Wire Wire Line
	21150 16710 21150 17470
Wire Wire Line
	22110 17670 21950 17670
Wire Wire Line
	21650 17670 21470 17670
Wire Wire Line
	24020 7880 24020 7940
$Comp
L Device:Crystal Y1
U 1 1 6374A2D5
P 14170 9590
F 0 "Y1" V 14216 9459 50  0000 R CNN
F 1 "8MHz" V 14125 9459 50  0000 R CNN
F 2 "" H 14170 9590 50  0001 C CNN
F 3 "~" H 14170 9590 50  0001 C CNN
	1    14170 9590
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C47
U 1 1 63A9AD4D
P 13880 9410
F 0 "C47" V 14132 9410 50  0000 C CNN
F 1 "18p" V 14041 9410 50  0000 C CNN
F 2 "" H 13918 9260 50  0001 C CNN
F 3 "~" H 13880 9410 50  0001 C CNN
	1    13880 9410
	0    1    -1   0   
$EndComp
$Comp
L Device:C C64
U 1 1 63A9D928
P 13880 9780
F 0 "C64" V 14132 9780 50  0000 C CNN
F 1 "18p" V 14041 9780 50  0000 C CNN
F 2 "" H 13918 9630 50  0001 C CNN
F 3 "~" H 13880 9780 50  0001 C CNN
	1    13880 9780
	0    -1   -1   0   
$EndComp
Connection ~ 14170 9410
Wire Wire Line
	14170 9410 14520 9410
Connection ~ 14170 9780
Text GLabel 13040 14850 0    50   Input ~ 0
5V
$Comp
L Device:L L1
U 1 1 66E2ECA6
P 13500 14850
F 0 "L1" V 13690 14850 50  0000 C CNN
F 1 "6.8µH" V 13599 14850 50  0000 C CNN
F 2 "" H 13500 14850 50  0001 C CNN
F 3 "~" H 13500 14850 50  0001 C CNN
	1    13500 14850
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C38
U 1 1 695833DE
P 13150 15200
F 0 "C38" H 13265 15246 50  0000 L CNN
F 1 "4.7µ" H 13265 15155 50  0000 L CNN
F 2 "" H 13188 15050 50  0001 C CNN
F 3 "~" H 13150 15200 50  0001 C CNN
	1    13150 15200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C44
U 1 1 69588931
P 15960 15720
F 0 "C44" H 16075 15766 50  0000 L CNN
F 1 "10p" H 16075 15675 50  0000 L CNN
F 2 "" H 15998 15570 50  0001 C CNN
F 3 "~" H 15960 15720 50  0001 C CNN
	1    15960 15720
	1    0    0    -1  
$EndComp
$Comp
L Device:C C40
U 1 1 69588937
P 13440 15510
F 0 "C40" H 13555 15556 50  0000 L CNN
F 1 "4.7µ" H 13555 15465 50  0000 L CNN
F 2 "" H 13478 15360 50  0001 C CNN
F 3 "~" H 13440 15510 50  0001 C CNN
	1    13440 15510
	1    0    0    -1  
$EndComp
$Comp
L Device:C C43
U 1 1 6973675C
P 15540 14850
F 0 "C43" H 15655 14896 50  0000 L CNN
F 1 "100n" H 15655 14805 50  0000 L CNN
F 2 "" H 15578 14700 50  0001 C CNN
F 3 "~" H 15540 14850 50  0001 C CNN
	1    15540 14850
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C41
U 1 1 69736762
P 15590 15940
F 0 "C41" H 15705 15986 50  0000 L CNN
F 1 "150p" H 15705 15895 50  0000 L CNN
F 2 "" H 15628 15790 50  0001 C CNN
F 3 "~" H 15590 15940 50  0001 C CNN
	1    15590 15940
	1    0    0    -1  
$EndComp
$Comp
L Device:R R85
U 1 1 69A9A23C
P 13780 15520
F 0 "R85" H 13850 15566 50  0000 L CNN
F 1 "57.6k" H 13850 15475 50  0000 L CNN
F 2 "" V 13710 15520 50  0001 C CNN
F 3 "~" H 13780 15520 50  0001 C CNN
	1    13780 15520
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 6A2F59B0
P 16860 16170
F 0 "#PWR0109" H 16860 15920 50  0001 C CNN
F 1 "GND" H 16865 15997 50  0000 C CNN
F 2 "" H 16860 16170 50  0001 C CNN
F 3 "" H 16860 16170 50  0001 C CNN
	1    16860 16170
	-1   0    0    -1  
$EndComp
Wire Wire Line
	15590 15720 15590 15790
Wire Wire Line
	15960 15870 15960 16090
Text GLabel 16700 18040 0    50   Input ~ 0
5V
$Comp
L Device:C C72
U 1 1 732690A9
P 17420 15170
F 0 "C72" H 17535 15216 50  0000 L CNN
F 1 "0,1µ" H 17535 15125 50  0000 L CNN
F 2 "" H 17458 15020 50  0001 C CNN
F 3 "~" H 17420 15170 50  0001 C CNN
	1    17420 15170
	1    0    0    -1  
$EndComp
$Comp
L Device:C C73
U 1 1 7326B98D
P 17420 15630
F 0 "C73" H 17535 15676 50  0000 L CNN
F 1 "0.1µ" H 17535 15585 50  0000 L CNN
F 2 "" H 17458 15480 50  0001 C CNN
F 3 "~" H 17420 15630 50  0001 C CNN
	1    17420 15630
	1    0    0    -1  
$EndComp
$Comp
L Device:R R87
U 1 1 69A8FD73
P 16320 15050
F 0 "R87" H 16390 15096 50  0000 L CNN
F 1 "1.02M" H 16390 15005 50  0000 L CNN
F 2 "" V 16250 15050 50  0001 C CNN
F 3 "~" H 16320 15050 50  0001 C CNN
	1    16320 15050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R88
U 1 1 69A94C83
P 16320 15450
F 0 "R88" H 16390 15496 50  0000 L CNN
F 1 "113k" H 16390 15405 50  0000 L CNN
F 2 "" V 16250 15450 50  0001 C CNN
F 3 "~" H 16320 15450 50  0001 C CNN
	1    16320 15450
	1    0    0    -1  
$EndComp
Wire Wire Line
	16320 15200 16320 15250
$Comp
L Device:C C71
U 1 1 698E47AF
P 16860 15170
F 0 "C71" H 16975 15216 50  0000 L CNN
F 1 "22µ" H 16975 15125 50  0000 L CNN
F 2 "" H 16898 15020 50  0001 C CNN
F 3 "~" H 16860 15170 50  0001 C CNN
	1    16860 15170
	1    0    0    -1  
$EndComp
$Comp
L Device:R R86
U 1 1 69A9A242
P 15590 15570
F 0 "R86" H 15660 15616 50  0000 L CNN
F 1 "210k" H 15660 15525 50  0000 L CNN
F 2 "" V 15520 15570 50  0001 C CNN
F 3 "~" H 15590 15570 50  0001 C CNN
	1    15590 15570
	1    0    0    -1  
$EndComp
Wire Wire Line
	17420 15020 17420 14850
Wire Wire Line
	17420 14850 17770 14850
Wire Wire Line
	17420 15780 17420 15890
Wire Wire Line
	17420 15890 17790 15890
Text GLabel 14880 10380 0    50   Input ~ 0
CS_POT_mic
$Comp
L Connector:Screw_Terminal_01x09 J3
U 1 1 77CF8DF8
P 19360 10790
F 0 "J3" H 19440 10832 50  0000 L CNN
F 1 "Screw_Terminal_01x09" H 19440 10741 50  0000 L CNN
F 2 "" H 19360 10790 50  0001 C CNN
F 3 "~" H 19360 10790 50  0001 C CNN
	1    19360 10790
	1    0    0    -1  
$EndComp
Text GLabel 19160 10790 0    50   Input ~ 0
GPIO_INT
Text GLabel 19160 10990 0    50   Input ~ 0
SDA
Text GLabel 19160 10890 0    50   Input ~ 0
SCL
Text GLabel 19160 11090 0    50   Input ~ 0
Din1_POT
Text GLabel 19160 11190 0    50   Input ~ 0
Din_Leds
Text GLabel 19160 10590 0    50   Input ~ 0
CS_POT
Text GLabel 19160 10690 0    50   Input ~ 0
cs_leds
Text GLabel 19160 10490 0    50   Input ~ 0
CS_POT_mic
Text GLabel 19160 10390 0    50   Input ~ 0
CLK
$Comp
L mengpaneel-rescue:LTC3121EDE#PBF-LTC3121EDE#PBF IC4
U 1 1 6094A552
P 13880 14850
F 0 "IC4" H 14580 15115 50  0000 C CNN
F 1 "LTC3121EDE#PBF" H 14580 15024 50  0000 C CNN
F 2 "LTC3121EDE#PBF:SON50P300X400X80-13N-D" H 15130 14950 50  0001 L CNN
F 3 "http://www.linear.com/docs/46926" H 15130 14850 50  0001 L CNN
F 4 "Switching Voltage Regulators 15V, 1.5A Synchronous Step-Up DC/DC Converter with Output Disconnect" H 15130 14750 50  0001 L CNN "Description"
F 5 "0.8" H 15130 14650 50  0001 L CNN "Height"
F 6 "584-LTC3121EDE#PBF" H 15130 14550 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Analog-Devices/LTC3121EDEPBF?qs=oahfZPh6IAKQaqRaQFQxpg%3D%3D" H 15130 14450 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 15130 14350 50  0001 L CNN "Manufacturer_Name"
F 9 "LTC3121EDE#PBF" H 15130 14250 50  0001 L CNN "Manufacturer_Part_Number"
	1    13880 14850
	1    0    0    -1  
$EndComp
$Comp
L mengpaneel-rescue:SJ-63053A-SJ-63053A J4
U 1 1 60ABE1CD
P 18810 5970
F 0 "J4" H 19210 6235 50  0000 C CNN
F 1 "SJ-63053A" H 19210 6144 50  0000 C CNN
F 2 "SJ-63053A:SJ63053A" H 19460 6070 50  0001 L CNN
F 3 "https://www.mouser.mx/datasheet/2/670/sj_63053a-1890674.pdf" H 19460 5970 50  0001 L CNN
F 4 "Phone Connectors audio jack, 6.35 mm, horizontal, 3 conductor, through hole, 0 switches, w/ nut" H 19460 5870 50  0001 L CNN "Description"
F 5 "12.8" H 19460 5770 50  0001 L CNN "Height"
F 6 "490-SJ-63053A" H 19460 5670 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/CUI-Devices/SJ-63053A?qs=hWgE7mdIu5Q5ZZx8mXk5JQ%3D%3D" H 19460 5570 50  0001 L CNN "Mouser Price/Stock"
F 8 "CUI Inc." H 19460 5470 50  0001 L CNN "Manufacturer_Name"
F 9 "SJ-63053A" H 19460 5370 50  0001 L CNN "Manufacturer_Part_Number"
	1    18810 5970
	1    0    0    1   
$EndComp
$Comp
L mengpaneel-rescue:UJ2-MIBH-G-SMT-TR-UJ2-MIBH-G-SMT-TR J1
U 1 1 60AC3437
P 11380 12980
F 0 "J1" H 11830 13245 50  0000 C CNN
F 1 "UJ2-MIBH-G-SMT-TR" H 11830 13154 50  0000 C CNN
F 2 "UJ2-MIBH-G-SMT-TR:UJ2MIBHGSMTTR" H 12130 13080 50  0001 L CNN
F 3 "" H 12130 12980 50  0001 L CNN
F 4 "USB Connectors" H 12130 12880 50  0001 L CNN "Description"
F 5 "3.15" H 12130 12780 50  0001 L CNN "Height"
F 6 "490-UJ2-MIBH-G-SMTTR" H 12130 12680 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/CUI-Devices/UJ2-MIBH-G-SMT-TR/?qs=IS%252B4QmGtzzpvS1XQusp0iA%3D%3D" H 12130 12580 50  0001 L CNN "Mouser Price/Stock"
F 8 "CUI Inc." H 12130 12480 50  0001 L CNN "Manufacturer_Name"
F 9 "UJ2-MIBH-G-SMT-TR" H 12130 12380 50  0001 L CNN "Manufacturer_Part_Number"
	1    11380 12980
	1    0    0    -1  
$EndComp
$Comp
L mengpaneel-rescue:AD5204BRUZ10-REEL7-AD5204BRUZ10-REEL7 IC5
U 1 1 60AD2C44
P 15550 19860
F 0 "IC5" H 16100 20125 50  0000 C CNN
F 1 "AD5204BRUZ10-REEL7" H 16100 20034 50  0000 C CNN
F 2 "SOP65P640X120-24N" H 16500 19960 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD5204_5206.pdf" H 16500 19860 50  0001 L CNN
F 4 "Digital Potentiometer ICs 4-CHANNEL DIGITAL POTENTIOMETER" H 16500 19760 50  0001 L CNN "Description"
F 5 "1.2" H 16500 19660 50  0001 L CNN "Height"
F 6 "584-AD5204BRUZ10-R7" H 16500 19560 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Analog-Devices/AD5204BRUZ10-REEL7?qs=NmRFExCfTkHt7KNHfwPhSw%3D%3D" H 16500 19460 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 16500 19360 50  0001 L CNN "Manufacturer_Name"
F 9 "AD5204BRUZ10-REEL7" H 16500 19260 50  0001 L CNN "Manufacturer_Part_Number"
	1    15550 19860
	1    0    0    -1  
$EndComp
Wire Wire Line
	13040 14850 13150 14850
Wire Wire Line
	13650 14850 13880 14850
Connection ~ 13150 14850
Wire Wire Line
	13150 14850 13350 14850
Wire Wire Line
	13880 15050 13150 15050
Wire Wire Line
	13150 14850 13150 15050
$Comp
L power:GND #PWR0101
U 1 1 62784563
P 13880 14950
F 0 "#PWR0101" H 13880 14700 50  0001 C CNN
F 1 "GND" H 13885 14777 50  0000 C CNN
F 2 "" H 13880 14950 50  0001 C CNN
F 3 "" H 13880 14950 50  0001 C CNN
	1    13880 14950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 62BDCB8F
P 13880 15150
F 0 "#PWR0110" H 13880 14900 50  0001 C CNN
F 1 "GND" H 13885 14977 50  0000 C CNN
F 2 "" H 13880 15150 50  0001 C CNN
F 3 "" H 13880 15150 50  0001 C CNN
	1    13880 15150
	0    1    1    0   
$EndComp
Text GLabel 15280 15150 2    50   Input ~ 0
SD_Step-Up
Text GLabel 15180 11080 3    50   Input ~ 0
SD_Step-Up
Wire Wire Line
	17420 15320 17420 15400
Wire Wire Line
	15280 15250 16320 15250
Connection ~ 16320 15250
Wire Wire Line
	16320 15250 16320 15300
Connection ~ 15590 16090
Wire Wire Line
	15590 16090 15960 16090
Wire Wire Line
	15280 15350 15590 15350
Wire Wire Line
	15960 15350 15960 15570
Wire Wire Line
	15590 15420 15590 15350
Connection ~ 15590 15350
Wire Wire Line
	15590 15350 15960 15350
Wire Wire Line
	16320 14900 16320 14850
Wire Wire Line
	16320 14850 15840 14850
Wire Wire Line
	15840 14850 15840 14950
Connection ~ 16320 14850
Wire Wire Line
	14580 16050 14580 16090
Connection ~ 14580 16090
Wire Wire Line
	14580 16090 15590 16090
Connection ~ 13150 15050
Wire Wire Line
	13780 15370 13780 15350
Wire Wire Line
	13780 15350 13880 15350
Wire Wire Line
	13880 15250 13440 15250
Wire Wire Line
	13440 15250 13440 15360
Wire Wire Line
	13780 15670 13780 16090
Wire Wire Line
	13780 16090 14580 16090
Wire Wire Line
	13150 15350 13150 16090
Wire Wire Line
	13150 16090 13440 16090
Connection ~ 13780 16090
Wire Wire Line
	13440 15660 13440 16090
Connection ~ 13440 16090
Wire Wire Line
	13440 16090 13780 16090
Wire Wire Line
	15960 16090 16320 16090
Wire Wire Line
	16860 16090 16860 15400
Connection ~ 15960 16090
Wire Wire Line
	16320 15600 16320 16090
Connection ~ 16320 16090
Wire Wire Line
	16320 16090 16860 16090
Wire Wire Line
	16860 16090 16860 16170
Connection ~ 16860 16090
Connection ~ 17420 14850
Wire Wire Line
	17420 15400 16860 15400
Connection ~ 17420 15400
Wire Wire Line
	17420 15400 17420 15480
Connection ~ 16860 15400
Wire Wire Line
	16320 14850 16860 14850
Wire Wire Line
	16860 15320 16860 15400
Wire Wire Line
	16860 15020 16860 14850
Connection ~ 16860 14850
Wire Wire Line
	16860 14850 17420 14850
Wire Wire Line
	15280 14950 15840 14950
Wire Wire Line
	15280 14850 15390 14850
Wire Wire Line
	15690 14850 15840 14850
Connection ~ 15840 14850
$Comp
L power:GND #PWR0111
U 1 1 68C5E648
P 15680 15050
F 0 "#PWR0111" H 15680 14800 50  0001 C CNN
F 1 "GND" H 15685 14877 50  0000 C CNN
F 2 "" H 15680 15050 50  0001 C CNN
F 3 "" H 15680 15050 50  0001 C CNN
	1    15680 15050
	0    -1   1    0   
$EndComp
Wire Wire Line
	15280 15050 15680 15050
Connection ~ 16840 17500
Wire Wire Line
	16840 17500 16840 17400
Wire Wire Line
	18010 17500 18010 17940
Wire Wire Line
	17580 17500 18010 17500
Wire Wire Line
	16840 17500 17280 17500
Wire Wire Line
	16840 17940 16840 17500
Wire Wire Line
	16940 17940 16840 17940
Wire Wire Line
	18010 17940 18080 17940
Connection ~ 18010 17940
Wire Wire Line
	18010 18300 18010 17940
Wire Wire Line
	17590 18300 18010 18300
Wire Wire Line
	16840 18040 16940 18040
Wire Wire Line
	16840 18300 17290 18300
Wire Wire Line
	16840 18040 16840 18300
Wire Wire Line
	17940 17940 18010 17940
$Comp
L mengpaneel-rescue:LM3480IM3-3.3_NOPB-LM3480IM3-3.3_NOPB IC7
U 1 1 60ABB371
P 16940 17940
F 0 "IC7" H 17440 18205 50  0000 C CNN
F 1 "LM3480IM3-3.3_NOPB" H 17440 18114 50  0000 C CNN
F 2 "LM3480IM3-3.3_NOPB:SOT95P237X112-3N" H 17790 18040 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm3480.pdf" H 17790 17940 50  0001 L CNN
F 4 "100 mA, SOT-23, Quasi Low-Dropout Linear Voltage Regulator" H 17790 17840 50  0001 L CNN "Description"
F 5 "1.12" H 17790 17740 50  0001 L CNN "Height"
F 6 "926-LM3480IM33.3NOPB" H 17790 17640 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Texas-Instruments/LM3480IM3-33-NOPB/?qs=X1J7HmVL2ZHWj012nPURIw%3D%3D" H 17790 17540 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 17790 17440 50  0001 L CNN "Manufacturer_Name"
F 9 "LM3480IM3-3.3/NOPB" H 17790 17340 50  0001 L CNN "Manufacturer_Part_Number"
	1    16940 17940
	1    0    0    -1  
$EndComp
$Comp
L Device:C C36
U 1 1 6062C633
P 17440 18300
F 0 "C36" H 17555 18346 50  0000 L CNN
F 1 "0.1µF" H 17555 18255 50  0000 L CNN
F 2 "" H 17478 18150 50  0001 C CNN
F 3 "~" H 17440 18300 50  0001 C CNN
	1    17440 18300
	0    -1   1    0   
$EndComp
$Comp
L Device:C C37
U 1 1 6062EFEB
P 17430 17500
F 0 "C37" H 17545 17546 50  0000 L CNN
F 1 "0.1µF" H 17545 17455 50  0000 L CNN
F 2 "" H 17468 17350 50  0001 C CNN
F 3 "~" H 17430 17500 50  0001 C CNN
	1    17430 17500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 60835B51
P 18080 17940
F 0 "#PWR0125" H 18080 17690 50  0001 C CNN
F 1 "GND" H 18085 17767 50  0000 C CNN
F 2 "" H 18080 17940 50  0001 C CNN
F 3 "" H 18080 17940 50  0001 C CNN
	1    18080 17940
	0    -1   1    0   
$EndComp
Text GLabel 16840 17400 1    50   Input ~ 0
3.3V
$Comp
L power:GND #PWR0149
U 1 1 6C8B9A98
P 18810 5970
F 0 "#PWR0149" H 18810 5720 50  0001 C CNN
F 1 "GND" H 18815 5797 50  0000 C CNN
F 2 "" H 18810 5970 50  0001 C CNN
F 3 "" H 18810 5970 50  0001 C CNN
	1    18810 5970
	-1   0    0    -1  
$EndComp
$Comp
L mengpaneel-rescue:SJ-63053A-SJ-63053A J5
U 1 1 6D78DD70
P 18850 15100
F 0 "J5" H 19250 15365 50  0000 C CNN
F 1 "SJ-63053A" H 19250 15274 50  0000 C CNN
F 2 "SJ-63053A:SJ63053A" H 19500 15200 50  0001 L CNN
F 3 "https://www.mouser.mx/datasheet/2/670/sj_63053a-1890674.pdf" H 19500 15100 50  0001 L CNN
F 4 "Phone Connectors audio jack, 6.35 mm, horizontal, 3 conductor, through hole, 0 switches, w/ nut" H 19500 15000 50  0001 L CNN "Description"
F 5 "12.8" H 19500 14900 50  0001 L CNN "Height"
F 6 "490-SJ-63053A" H 19500 14800 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/CUI-Devices/SJ-63053A?qs=hWgE7mdIu5Q5ZZx8mXk5JQ%3D%3D" H 19500 14700 50  0001 L CNN "Mouser Price/Stock"
F 8 "CUI Inc." H 19500 14600 50  0001 L CNN "Manufacturer_Name"
F 9 "SJ-63053A" H 19500 14500 50  0001 L CNN "Manufacturer_Part_Number"
	1    18850 15100
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0192
U 1 1 6D78DD76
P 18850 15100
F 0 "#PWR0192" H 18850 14850 50  0001 C CNN
F 1 "GND" H 18855 14927 50  0000 C CNN
F 2 "" H 18850 15100 50  0001 C CNN
F 3 "" H 18850 15100 50  0001 C CNN
	1    18850 15100
	-1   0    0    -1  
$EndComp
$Comp
L mengpaneel-rescue:SJ-63053A-SJ-63053A J6
U 1 1 6D90AE19
P 18960 3570
F 0 "J6" H 19360 3835 50  0000 C CNN
F 1 "SJ-63053A" H 19360 3744 50  0000 C CNN
F 2 "SJ-63053A:SJ63053A" H 19610 3670 50  0001 L CNN
F 3 "https://www.mouser.mx/datasheet/2/670/sj_63053a-1890674.pdf" H 19610 3570 50  0001 L CNN
F 4 "Phone Connectors audio jack, 6.35 mm, horizontal, 3 conductor, through hole, 0 switches, w/ nut" H 19610 3470 50  0001 L CNN "Description"
F 5 "12.8" H 19610 3370 50  0001 L CNN "Height"
F 6 "490-SJ-63053A" H 19610 3270 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/CUI-Devices/SJ-63053A?qs=hWgE7mdIu5Q5ZZx8mXk5JQ%3D%3D" H 19610 3170 50  0001 L CNN "Mouser Price/Stock"
F 8 "CUI Inc." H 19610 3070 50  0001 L CNN "Manufacturer_Name"
F 9 "SJ-63053A" H 19610 2970 50  0001 L CNN "Manufacturer_Part_Number"
	1    18960 3570
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0194
U 1 1 6D90AE1F
P 18960 3570
F 0 "#PWR0194" H 18960 3320 50  0001 C CNN
F 1 "GND" H 18965 3397 50  0000 C CNN
F 2 "" H 18960 3570 50  0001 C CNN
F 3 "" H 18960 3570 50  0001 C CNN
	1    18960 3570
	-1   0    0    -1  
$EndComp
Wire Wire Line
	19760 3470 19760 3570
Connection ~ 19760 3570
$Comp
L mengpaneel-rescue:SJ-63053A-SJ-63053A J7
U 1 1 6E077880
P 32500 11300
F 0 "J7" H 32900 11565 50  0000 C CNN
F 1 "SJ-63053A" H 32900 11474 50  0000 C CNN
F 2 "SJ-63053A:SJ63053A" H 33150 11400 50  0001 L CNN
F 3 "https://www.mouser.mx/datasheet/2/670/sj_63053a-1890674.pdf" H 33150 11300 50  0001 L CNN
F 4 "Phone Connectors audio jack, 6.35 mm, horizontal, 3 conductor, through hole, 0 switches, w/ nut" H 33150 11200 50  0001 L CNN "Description"
F 5 "12.8" H 33150 11100 50  0001 L CNN "Height"
F 6 "490-SJ-63053A" H 33150 11000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/CUI-Devices/SJ-63053A?qs=hWgE7mdIu5Q5ZZx8mXk5JQ%3D%3D" H 33150 10900 50  0001 L CNN "Mouser Price/Stock"
F 8 "CUI Inc." H 33150 10800 50  0001 L CNN "Manufacturer_Name"
F 9 "SJ-63053A" H 33150 10700 50  0001 L CNN "Manufacturer_Part_Number"
	1    32500 11300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0195
U 1 1 6E077886
P 32500 11300
F 0 "#PWR0195" H 32500 11050 50  0001 C CNN
F 1 "GND" H 32505 11127 50  0000 C CNN
F 2 "" H 32500 11300 50  0001 C CNN
F 3 "" H 32500 11300 50  0001 C CNN
	1    32500 11300
	1    0    0    -1  
$EndComp
Wire Wire Line
	31550 11200 31700 11200
Wire Wire Line
	31290 11300 31700 11300
$Comp
L power:GND #PWR0196
U 1 1 7027D138
P 12420 13280
F 0 "#PWR0196" H 12420 13030 50  0001 C CNN
F 1 "GND" H 12425 13107 50  0000 C CNN
F 2 "" H 12420 13280 50  0001 C CNN
F 3 "" H 12420 13280 50  0001 C CNN
	1    12420 13280
	0    -1   1    0   
$EndComp
NoConn ~ 12280 13180
NoConn ~ 12280 13080
NoConn ~ 12280 13380
Text GLabel 12280 12980 2    50   Input ~ 0
5V
Wire Wire Line
	11380 12980 11320 12980
Wire Wire Line
	11320 12980 11320 13080
Wire Wire Line
	11320 13280 11380 13280
Wire Wire Line
	11380 13180 11320 13180
Connection ~ 11320 13180
Wire Wire Line
	11320 13180 11320 13280
Wire Wire Line
	11380 13080 11320 13080
Connection ~ 11320 13080
Wire Wire Line
	11320 13080 11320 13180
Wire Wire Line
	12280 13280 12350 13280
Wire Wire Line
	12350 13280 12350 13570
Wire Wire Line
	12350 13570 11320 13570
Wire Wire Line
	11320 13570 11320 13280
Connection ~ 12350 13280
Wire Wire Line
	12350 13280 12420 13280
Connection ~ 11320 13280
NoConn ~ 15550 19860
NoConn ~ 15550 19960
$Comp
L power:GND #PWR0112
U 1 1 72076B9F
P 15550 20060
F 0 "#PWR0112" H 15550 19810 50  0001 C CNN
F 1 "GND" H 15555 19887 50  0000 C CNN
F 2 "" H 15550 20060 50  0001 C CNN
F 3 "" H 15550 20060 50  0001 C CNN
	1    15550 20060
	0    1    1    0   
$EndComp
Text GLabel 15550 20160 0    35   Input ~ 0
CS_POT
NoConn ~ 15550 20960
Text GLabel 15550 20660 0    35   Input ~ 0
CLK
Text GLabel 15550 20360 0    35   Input ~ 0
5V
Text GLabel 15550 20460 0    35   Input ~ 0
SD_POT
Text GLabel 15550 20260 0    35   Input ~ 0
PReset_POT
Text GLabel 15550 20560 0    35   Input ~ 0
L1.1
Text GLabel 15550 20760 0    35   Input ~ 0
L1.2
$Comp
L mengpaneel-rescue:AD5204BRUZ10-REEL7-AD5204BRUZ10-REEL7 IC8
U 1 1 737456F2
P 17970 19840
F 0 "IC8" H 18520 20105 50  0000 C CNN
F 1 "AD5204BRUZ10-REEL7" H 18520 20014 50  0000 C CNN
F 2 "SOP65P640X120-24N" H 18920 19940 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD5204_5206.pdf" H 18920 19840 50  0001 L CNN
F 4 "Digital Potentiometer ICs 4-CHANNEL DIGITAL POTENTIOMETER" H 18920 19740 50  0001 L CNN "Description"
F 5 "1.2" H 18920 19640 50  0001 L CNN "Height"
F 6 "584-AD5204BRUZ10-R7" H 18920 19540 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Analog-Devices/AD5204BRUZ10-REEL7?qs=NmRFExCfTkHt7KNHfwPhSw%3D%3D" H 18920 19440 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 18920 19340 50  0001 L CNN "Manufacturer_Name"
F 9 "AD5204BRUZ10-REEL7" H 18920 19240 50  0001 L CNN "Manufacturer_Part_Number"
	1    17970 19840
	1    0    0    -1  
$EndComp
NoConn ~ 17970 19840
NoConn ~ 17970 19940
$Comp
L power:GND #PWR0113
U 1 1 737456FA
P 17970 20040
F 0 "#PWR0113" H 17970 19790 50  0001 C CNN
F 1 "GND" H 17975 19867 50  0000 C CNN
F 2 "" H 17970 20040 50  0001 C CNN
F 3 "" H 17970 20040 50  0001 C CNN
	1    17970 20040
	0    1    1    0   
$EndComp
Text GLabel 17970 20140 0    35   Input ~ 0
CS_POT
NoConn ~ 17970 20940
Text GLabel 17970 20640 0    35   Input ~ 0
CLK
Text GLabel 17970 20340 0    35   Input ~ 0
5V
Text GLabel 17970 20440 0    35   Input ~ 0
SD_POT
Text GLabel 17970 20240 0    35   Input ~ 0
PReset_POT
Text GLabel 17970 20540 0    35   Input ~ 0
L1.2
Text GLabel 17970 20740 0    35   Input ~ 0
L1.3
$Comp
L mengpaneel-rescue:AD5204BRUZ10-REEL7-AD5204BRUZ10-REEL7 IC11
U 1 1 738C903E
P 20400 19860
F 0 "IC11" H 20950 20125 50  0000 C CNN
F 1 "AD5204BRUZ10-REEL7" H 20950 20034 50  0000 C CNN
F 2 "SOP65P640X120-24N" H 21350 19960 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD5204_5206.pdf" H 21350 19860 50  0001 L CNN
F 4 "Digital Potentiometer ICs 4-CHANNEL DIGITAL POTENTIOMETER" H 21350 19760 50  0001 L CNN "Description"
F 5 "1.2" H 21350 19660 50  0001 L CNN "Height"
F 6 "584-AD5204BRUZ10-R7" H 21350 19560 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Analog-Devices/AD5204BRUZ10-REEL7?qs=NmRFExCfTkHt7KNHfwPhSw%3D%3D" H 21350 19460 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 21350 19360 50  0001 L CNN "Manufacturer_Name"
F 9 "AD5204BRUZ10-REEL7" H 21350 19260 50  0001 L CNN "Manufacturer_Part_Number"
	1    20400 19860
	1    0    0    -1  
$EndComp
NoConn ~ 20400 19860
NoConn ~ 20400 19960
$Comp
L power:GND #PWR0114
U 1 1 738C9046
P 20400 20060
F 0 "#PWR0114" H 20400 19810 50  0001 C CNN
F 1 "GND" H 20405 19887 50  0000 C CNN
F 2 "" H 20400 20060 50  0001 C CNN
F 3 "" H 20400 20060 50  0001 C CNN
	1    20400 20060
	0    1    1    0   
$EndComp
Text GLabel 20400 20160 0    35   Input ~ 0
CS_POT
NoConn ~ 20400 20960
Text GLabel 20400 20660 0    35   Input ~ 0
CLK
Text GLabel 20400 20360 0    35   Input ~ 0
5V
Text GLabel 20400 20460 0    35   Input ~ 0
SD_POT
Text GLabel 20400 20260 0    35   Input ~ 0
PReset_POT
Text GLabel 20400 20560 0    35   Input ~ 0
L1.3
Text GLabel 20400 20760 0    35   Input ~ 0
L1.4
$Comp
L mengpaneel-rescue:AD5204BRUZ10-REEL7-AD5204BRUZ10-REEL7 IC6
U 1 1 73A4C8F7
P 15580 21500
F 0 "IC6" H 16130 21765 50  0000 C CNN
F 1 "AD5204BRUZ10-REEL7" H 16130 21674 50  0000 C CNN
F 2 "SOP65P640X120-24N" H 16530 21600 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD5204_5206.pdf" H 16530 21500 50  0001 L CNN
F 4 "Digital Potentiometer ICs 4-CHANNEL DIGITAL POTENTIOMETER" H 16530 21400 50  0001 L CNN "Description"
F 5 "1.2" H 16530 21300 50  0001 L CNN "Height"
F 6 "584-AD5204BRUZ10-R7" H 16530 21200 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Analog-Devices/AD5204BRUZ10-REEL7?qs=NmRFExCfTkHt7KNHfwPhSw%3D%3D" H 16530 21100 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 16530 21000 50  0001 L CNN "Manufacturer_Name"
F 9 "AD5204BRUZ10-REEL7" H 16530 20900 50  0001 L CNN "Manufacturer_Part_Number"
	1    15580 21500
	1    0    0    -1  
$EndComp
NoConn ~ 15580 21500
NoConn ~ 15580 21600
$Comp
L power:GND #PWR0117
U 1 1 73A4C8FF
P 15580 21700
F 0 "#PWR0117" H 15580 21450 50  0001 C CNN
F 1 "GND" H 15585 21527 50  0000 C CNN
F 2 "" H 15580 21700 50  0001 C CNN
F 3 "" H 15580 21700 50  0001 C CNN
	1    15580 21700
	0    1    1    0   
$EndComp
Text GLabel 15580 21800 0    35   Input ~ 0
CS_POT
NoConn ~ 15580 22600
Text GLabel 15580 22300 0    35   Input ~ 0
CLK
Text GLabel 15580 22000 0    35   Input ~ 0
5V
Text GLabel 15580 22100 0    35   Input ~ 0
SD_POT
Text GLabel 15580 21900 0    35   Input ~ 0
PReset_POT
Text GLabel 15580 22200 0    35   Input ~ 0
L1.4
Text GLabel 15580 22400 0    35   Input ~ 0
L1.5
$Comp
L mengpaneel-rescue:AD5204BRUZ10-REEL7-AD5204BRUZ10-REEL7 IC9
U 1 1 73BD0676
P 17970 21470
F 0 "IC9" H 18520 21735 50  0000 C CNN
F 1 "AD5204BRUZ10-REEL7" H 18520 21644 50  0000 C CNN
F 2 "SOP65P640X120-24N" H 18920 21570 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/AD5204_5206.pdf" H 18920 21470 50  0001 L CNN
F 4 "Digital Potentiometer ICs 4-CHANNEL DIGITAL POTENTIOMETER" H 18920 21370 50  0001 L CNN "Description"
F 5 "1.2" H 18920 21270 50  0001 L CNN "Height"
F 6 "584-AD5204BRUZ10-R7" H 18920 21170 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Analog-Devices/AD5204BRUZ10-REEL7?qs=NmRFExCfTkHt7KNHfwPhSw%3D%3D" H 18920 21070 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 18920 20970 50  0001 L CNN "Manufacturer_Name"
F 9 "AD5204BRUZ10-REEL7" H 18920 20870 50  0001 L CNN "Manufacturer_Part_Number"
	1    17970 21470
	1    0    0    -1  
$EndComp
NoConn ~ 17970 21470
NoConn ~ 17970 21570
$Comp
L power:GND #PWR0118
U 1 1 73BD067E
P 17970 21670
F 0 "#PWR0118" H 17970 21420 50  0001 C CNN
F 1 "GND" H 17975 21497 50  0000 C CNN
F 2 "" H 17970 21670 50  0001 C CNN
F 3 "" H 17970 21670 50  0001 C CNN
	1    17970 21670
	0    1    1    0   
$EndComp
Text GLabel 17970 21770 0    35   Input ~ 0
CS_POT
NoConn ~ 17970 22570
Text GLabel 17970 22270 0    35   Input ~ 0
CLK
Text GLabel 17970 21970 0    35   Input ~ 0
5V
Text GLabel 17970 22070 0    35   Input ~ 0
SD_POT
Text GLabel 17970 21870 0    35   Input ~ 0
PReset_POT
Text GLabel 17970 22170 0    35   Input ~ 0
L1.5
NoConn ~ 17970 22370
$Comp
L power:GND #PWR0119
U 1 1 743656B8
P 16650 19860
F 0 "#PWR0119" H 16650 19610 50  0001 C CNN
F 1 "GND" H 16655 19687 50  0000 C CNN
F 2 "" H 16650 19860 50  0001 C CNN
F 3 "" H 16650 19860 50  0001 C CNN
	1    16650 19860
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 747EEE53
P 16650 20160
F 0 "#PWR0120" H 16650 19910 50  0001 C CNN
F 1 "GND" H 16655 19987 50  0000 C CNN
F 2 "" H 16650 20160 50  0001 C CNN
F 3 "" H 16650 20160 50  0001 C CNN
	1    16650 20160
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 74972238
P 16650 20660
F 0 "#PWR0121" H 16650 20410 50  0001 C CNN
F 1 "GND" H 16655 20487 50  0000 C CNN
F 2 "" H 16650 20660 50  0001 C CNN
F 3 "" H 16650 20660 50  0001 C CNN
	1    16650 20660
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 74AF57F9
P 16650 20960
F 0 "#PWR0122" H 16650 20710 50  0001 C CNN
F 1 "GND" H 16655 20787 50  0000 C CNN
F 2 "" H 16650 20960 50  0001 C CNN
F 3 "" H 16650 20960 50  0001 C CNN
	1    16650 20960
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 74C78A4D
P 19070 20640
F 0 "#PWR0127" H 19070 20390 50  0001 C CNN
F 1 "GND" H 19075 20467 50  0000 C CNN
F 2 "" H 19070 20640 50  0001 C CNN
F 3 "" H 19070 20640 50  0001 C CNN
	1    19070 20640
	0    -1   -1   0   
$EndComp
Text GLabel 16650 20460 2    35   Input ~ 0
POT1_A
Text GLabel 16650 20760 2    35   Input ~ 0
POT3_A
Text GLabel 16650 20360 2    35   Input ~ 0
POT2_A
Text GLabel 16650 20060 2    35   Input ~ 0
POT4_A
Text GLabel 19070 20440 2    35   Input ~ 0
POT5_A
Text GLabel 20520 4670 0    35   Input ~ 0
POT1_A
Text GLabel 20520 4870 0    35   Input ~ 0
POT1_W
Text GLabel 21030 6520 0    35   Input ~ 0
POT2_A
Text GLabel 21030 6720 0    35   Input ~ 0
POT2_W
Text GLabel 21030 8340 0    35   Input ~ 0
POT3_A
Text GLabel 21030 8540 0    35   Input ~ 0
POT3_W
Text GLabel 21070 15650 0    35   Input ~ 0
POT4_A
Text GLabel 21070 15850 0    35   Input ~ 0
POT4_W
Text GLabel 21070 17470 0    35   Input ~ 0
POT5_A
Text GLabel 21070 17670 0    35   Input ~ 0
POT5_W
Text GLabel 16650 20860 2    35   Input ~ 0
POT3_W
Text GLabel 16650 20560 2    35   Input ~ 0
POT1_W
Text GLabel 16650 20260 2    35   Input ~ 0
POT2_W
Text GLabel 16650 19960 2    35   Input ~ 0
POT4_W
$Comp
L power:GND #PWR0128
U 1 1 606E6E40
P 15550 20860
F 0 "#PWR0128" H 15550 20610 50  0001 C CNN
F 1 "GND" H 15555 20687 50  0000 C CNN
F 2 "" H 15550 20860 50  0001 C CNN
F 3 "" H 15550 20860 50  0001 C CNN
	1    15550 20860
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 6086836B
P 17970 20840
F 0 "#PWR0129" H 17970 20590 50  0001 C CNN
F 1 "GND" H 17975 20667 50  0000 C CNN
F 2 "" H 17970 20840 50  0001 C CNN
F 3 "" H 17970 20840 50  0001 C CNN
	1    17970 20840
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 609E9916
P 17970 22470
F 0 "#PWR0130" H 17970 22220 50  0001 C CNN
F 1 "GND" H 17975 22297 50  0000 C CNN
F 2 "" H 17970 22470 50  0001 C CNN
F 3 "" H 17970 22470 50  0001 C CNN
	1    17970 22470
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 60B6AF55
P 15580 22500
F 0 "#PWR0132" H 15580 22250 50  0001 C CNN
F 1 "GND" H 15585 22327 50  0000 C CNN
F 2 "" H 15580 22500 50  0001 C CNN
F 3 "" H 15580 22500 50  0001 C CNN
	1    15580 22500
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 60CEC420
P 20400 20860
F 0 "#PWR0133" H 20400 20610 50  0001 C CNN
F 1 "GND" H 20405 20687 50  0000 C CNN
F 2 "" H 20400 20860 50  0001 C CNN
F 3 "" H 20400 20860 50  0001 C CNN
	1    20400 20860
	0    1    -1   0   
$EndComp
Text GLabel 24210 3570 1    35   Input ~ 0
POT6_B
Text GLabel 24010 3570 1    35   Input ~ 0
POT6_W
Text GLabel 23810 3570 1    35   Input ~ 0
POT6_A
Text GLabel 19070 20140 2    35   Input ~ 0
POT6_B
Text GLabel 19070 20240 2    35   Input ~ 0
POT6_W
Text GLabel 19070 20340 2    35   Input ~ 0
POT6_A
Text GLabel 19070 20540 2    35   Input ~ 0
POT5_W
Text GLabel 19070 19840 2    35   Input ~ 0
POT8_B
Text GLabel 19070 19940 2    35   Input ~ 0
POT8_W
Text GLabel 19070 20040 2    35   Input ~ 0
POT8_A
Text GLabel 19070 20940 2    35   Input ~ 0
POT7_B
Text GLabel 19070 20840 2    35   Input ~ 0
POT7_W
Text GLabel 19070 20740 2    35   Input ~ 0
POT7_A
Text GLabel 24210 4300 3    35   Input ~ 0
POT7_B
Text GLabel 24010 4300 3    35   Input ~ 0
POT7_W
Text GLabel 23810 4300 3    35   Input ~ 0
POT7_A
Text GLabel 24220 5530 3    35   Input ~ 0
POT8_B
Text GLabel 24020 5530 3    35   Input ~ 0
POT8_W
Text GLabel 23820 5530 3    35   Input ~ 0
POT8_A
Text GLabel 24220 7880 1    35   Input ~ 0
POT9_B
Text GLabel 24020 7880 1    35   Input ~ 0
POT9_W
Text GLabel 23820 7880 1    35   Input ~ 0
POT9_A
Text GLabel 24210 9110 1    35   Input ~ 0
POT10_B
Text GLabel 24010 9110 1    35   Input ~ 0
POT10_W
Text GLabel 23810 9110 1    35   Input ~ 0
POT10_A
Text GLabel 24210 9840 3    35   Input ~ 0
POT11_B
Text GLabel 24010 9840 3    35   Input ~ 0
POT11_W
Text GLabel 23810 9840 3    35   Input ~ 0
POT11_A
Text GLabel 24250 12700 1    35   Input ~ 0
POT12_B
Text GLabel 24050 12700 1    35   Input ~ 0
POT12_W
Text GLabel 23850 12700 1    35   Input ~ 0
POT12_A
Text GLabel 24250 13430 3    35   Input ~ 0
POT13_B
Text GLabel 24050 13430 3    35   Input ~ 0
POT13_W
Text GLabel 23850 13430 3    35   Input ~ 0
POT13_A
Text GLabel 24260 14660 3    35   Input ~ 0
POT14_B
Text GLabel 24060 14660 3    35   Input ~ 0
POT14_W
Text GLabel 23860 14660 3    35   Input ~ 0
POT14_A
Text GLabel 24260 17010 1    35   Input ~ 0
POT15_B
Text GLabel 24060 17010 1    35   Input ~ 0
POT15_W
Text GLabel 23860 17010 1    35   Input ~ 0
POT15_A
Text GLabel 24250 18240 1    35   Input ~ 0
POT16_B
Text GLabel 24050 18240 1    35   Input ~ 0
POT16_W
Text GLabel 23850 18240 1    35   Input ~ 0
POT16_A
Text GLabel 24250 18970 3    35   Input ~ 0
POT17_B
Text GLabel 24050 18970 3    35   Input ~ 0
POT17_W
Text GLabel 23850 18970 3    35   Input ~ 0
POT17_A
Wire Wire Line
	16840 18040 16700 18040
Connection ~ 16840 18040
Text GLabel 30230 10580 3    35   Input ~ 0
POT17_W
Text GLabel 30030 10580 3    35   Input ~ 0
POT17_A
Text GLabel 30230 12030 1    35   Input ~ 0
POT17_W
Text GLabel 30030 12030 1    35   Input ~ 0
POT17_A
Text GLabel 27470 11560 2    35   Input ~ 0
POT17_B
Text GLabel 26810 11160 2    35   Input ~ 0
POT17_A
$Comp
L power:GND #PWR0134
U 1 1 69421721
P 19070 21470
F 0 "#PWR0134" H 19070 21220 50  0001 C CNN
F 1 "GND" H 19075 21297 50  0000 C CNN
F 2 "" H 19070 21470 50  0001 C CNN
F 3 "" H 19070 21470 50  0001 C CNN
	1    19070 21470
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 696FEC2C
P 19070 21770
F 0 "#PWR0135" H 19070 21520 50  0001 C CNN
F 1 "GND" H 19075 21597 50  0000 C CNN
F 2 "" H 19070 21770 50  0001 C CNN
F 3 "" H 19070 21770 50  0001 C CNN
	1    19070 21770
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR0136
U 1 1 6986E622
P 19070 22470
F 0 "#PWR0136" H 19070 22220 50  0001 C CNN
F 1 "GND" H 19075 22297 50  0000 C CNN
F 2 "" H 19070 22470 50  0001 C CNN
F 3 "" H 19070 22470 50  0001 C CNN
	1    19070 22470
	0    -1   1    0   
$EndComp
Wire Wire Line
	30780 11300 30780 12030
Wire Wire Line
	29450 12030 30030 12030
Wire Wire Line
	30780 12030 30230 12030
Wire Wire Line
	26730 10580 26730 11160
Wire Wire Line
	27390 10330 27390 11560
Wire Wire Line
	26810 11160 26730 11160
Connection ~ 26730 11160
Wire Wire Line
	26730 11160 26730 12230
Wire Wire Line
	27470 11560 27390 11560
Connection ~ 27390 11560
Wire Wire Line
	27390 11560 27390 12030
Wire Wire Line
	29450 10580 30030 10580
Wire Wire Line
	30230 10580 30780 10580
Wire Wire Line
	30780 10580 30780 11010
Text GLabel 21500 20460 2    35   Input ~ 0
POT9_A
Text GLabel 21500 20160 2    35   Input ~ 0
POT10_B
Text GLabel 21500 20260 2    35   Input ~ 0
POT10_W
Text GLabel 21500 20360 2    35   Input ~ 0
POT10_A
Text GLabel 21500 20560 2    35   Input ~ 0
POT9_W
Text GLabel 21500 19860 2    35   Input ~ 0
POT12_B
Text GLabel 21500 19960 2    35   Input ~ 0
POT12_W
Text GLabel 21500 20060 2    35   Input ~ 0
POT12_A
Text GLabel 21500 20960 2    35   Input ~ 0
POT11_B
Text GLabel 21500 20860 2    35   Input ~ 0
POT11_W
Text GLabel 21500 20760 2    35   Input ~ 0
POT11_A
Text GLabel 21500 20660 2    35   Input ~ 0
POT9_B
Text GLabel 16680 22100 2    35   Input ~ 0
POT13_A
Text GLabel 16680 21800 2    35   Input ~ 0
POT14_B
Text GLabel 16680 21900 2    35   Input ~ 0
POT14_W
Text GLabel 16680 22000 2    35   Input ~ 0
POT14_A
Text GLabel 16680 22200 2    35   Input ~ 0
POT13_W
Text GLabel 16680 21500 2    35   Input ~ 0
POT16_B
Text GLabel 16680 21600 2    35   Input ~ 0
POT16_W
Text GLabel 16680 21700 2    35   Input ~ 0
POT16_A
Text GLabel 16680 22600 2    35   Input ~ 0
POT15_B
Text GLabel 16680 22500 2    35   Input ~ 0
POT15_W
Text GLabel 16680 22400 2    35   Input ~ 0
POT15_A
Text GLabel 16680 22300 2    35   Input ~ 0
POT13_B
Text GLabel 19070 22070 2    35   Input ~ 0
POT17_A
Text GLabel 19070 21870 2    35   Input ~ 0
POT18_W
Text GLabel 19070 21970 2    35   Input ~ 0
POT18_A
Text GLabel 19070 22170 2    35   Input ~ 0
POT17_W
Text GLabel 19070 21570 2    35   Input ~ 0
POT20_W
Text GLabel 19070 21670 2    35   Input ~ 0
POT20_A
Text GLabel 19070 22570 2    35   Input ~ 0
POT19_B
Text GLabel 19070 22370 2    35   Input ~ 0
POT19_A
Text GLabel 19070 22270 2    35   Input ~ 0
POT17_B
Wire Wire Line
	18570 12510 20260 12510
Text GLabel 14210 9910 0    50   Input ~ 0
3.3V
Wire Wire Line
	14250 10220 14250 10230
Wire Wire Line
	14250 9910 14590 9910
Wire Wire Line
	14590 9910 14590 9980
Wire Wire Line
	14590 9980 14880 9980
Wire Wire Line
	14250 10230 14600 10230
Wire Wire Line
	14600 10230 14600 10080
Wire Wire Line
	14600 10080 14880 10080
Connection ~ 14250 10230
Wire Wire Line
	14250 10230 14250 10240
Wire Wire Line
	14250 9920 14250 9910
Wire Wire Line
	14250 9910 14210 9910
Connection ~ 14250 9910
Wire Wire Line
	14170 9780 14880 9780
Wire Wire Line
	14880 9680 14520 9680
Wire Wire Line
	14520 9680 14520 9410
Wire Wire Line
	13730 9410 13730 9780
Wire Wire Line
	13730 9780 13730 10230
Wire Wire Line
	13730 10230 14250 10230
Connection ~ 13730 9780
$Comp
L Device:C C48
U 1 1 7473720C
P 13340 9980
F 0 "C48" H 13455 10026 50  0000 L CNN
F 1 "0.1µF" H 13455 9935 50  0000 L CNN
F 2 "" H 13378 9830 50  0001 C CNN
F 3 "~" H 13340 9980 50  0001 C CNN
	1    13340 9980
	1    0    0    -1  
$EndComp
Text GLabel 14880 9880 0    50   Input ~ 0
NRST
Text GLabel 13400 9760 2    50   Input ~ 0
NRST
Wire Wire Line
	13120 9780 13120 9760
Wire Wire Line
	13120 9760 13340 9760
Wire Wire Line
	13340 9830 13340 9760
Connection ~ 13340 9760
Wire Wire Line
	13340 9760 13400 9760
Wire Wire Line
	13120 10180 13120 10230
Wire Wire Line
	13120 10230 13340 10230
Connection ~ 13730 10230
Wire Wire Line
	13340 10130 13340 10230
Connection ~ 13340 10230
Wire Wire Line
	13340 10230 13730 10230
Text GLabel 15280 8580 1    50   Input ~ 0
3.3V
$Comp
L Device:C C49
U 1 1 7567756F
P 16080 11230
F 0 "C49" H 16195 11276 50  0000 L CNN
F 1 "0.1µF" H 16195 11185 50  0000 L CNN
F 2 "" H 16118 11080 50  0001 C CNN
F 3 "~" H 16080 11230 50  0001 C CNN
	1    16080 11230
	1    0    0    -1  
$EndComp
Wire Wire Line
	16180 11080 16180 11380
Wire Wire Line
	16180 11380 16080 11380
Wire Wire Line
	16080 11380 16080 11500
Connection ~ 16080 11380
Text GLabel 14880 9280 0    50   Input ~ 0
3.3V
$Comp
L power:GND #PWR0139
U 1 1 762C615B
P 15380 8070
F 0 "#PWR0139" H 15380 7820 50  0001 C CNN
F 1 "GND" H 15385 7897 50  0000 C CNN
F 2 "" H 15380 8070 50  0001 C CNN
F 3 "" H 15380 8070 50  0001 C CNN
	1    15380 8070
	1    0    0    1   
$EndComp
Wire Wire Line
	15380 8580 15380 8070
Wire Wire Line
	15680 8100 15680 8070
Wire Wire Line
	15380 8070 15680 8070
Wire Wire Line
	15680 8580 15680 8400
Connection ~ 15380 8070
$Comp
L power:GND #PWR0140
U 1 1 76D8E73A
P 16080 11500
F 0 "#PWR0140" H 16080 11250 50  0001 C CNN
F 1 "GND" H 16085 11327 50  0000 C CNN
F 2 "" H 16080 11500 50  0001 C CNN
F 3 "" H 16080 11500 50  0001 C CNN
	1    16080 11500
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0141
U 1 1 770A7940
P 16860 9380
F 0 "#PWR0141" H 16860 9130 50  0001 C CNN
F 1 "GND" H 16865 9207 50  0000 C CNN
F 2 "" H 16860 9380 50  0001 C CNN
F 3 "" H 16860 9380 50  0001 C CNN
	1    16860 9380
	0    -1   1    0   
$EndComp
Text GLabel 16680 9280 2    50   Input ~ 0
3.3V
Text GLabel 16280 11080 2    50   Input ~ 0
3.3V
Wire Wire Line
	16680 9380 16860 9380
Wire Wire Line
	18110 13530 18110 13450
Wire Wire Line
	18110 13030 17960 13030
Connection ~ 18110 13030
Wire Wire Line
	18110 13150 18110 13030
Connection ~ 20770 13530
Wire Wire Line
	20850 13530 20770 13530
Connection ~ 20770 12710
Wire Wire Line
	20850 12710 20770 12710
$Comp
L power:GND #PWR0138
U 1 1 70EB3CF8
P 18110 13530
F 0 "#PWR0138" H 18110 13280 50  0001 C CNN
F 1 "GND" H 18115 13357 50  0000 C CNN
F 2 "" H 18110 13530 50  0001 C CNN
F 3 "" H 18110 13530 50  0001 C CNN
	1    18110 13530
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 70BC4252
P 20850 12710
F 0 "#PWR0137" H 20850 12460 50  0001 C CNN
F 1 "GND" H 20855 12537 50  0000 C CNN
F 2 "" H 20850 12710 50  0001 C CNN
F 3 "" H 20850 12710 50  0001 C CNN
	1    20850 12710
	0    -1   1    0   
$EndComp
Text GLabel 20850 13530 2    50   Input ~ 0
-12V
Wire Wire Line
	20770 13530 20770 13280
Wire Wire Line
	20260 13530 20770 13530
Wire Wire Line
	20260 12710 20260 12750
Connection ~ 20260 12710
Wire Wire Line
	20770 12710 20770 12980
Wire Wire Line
	20260 12710 20770 12710
Wire Wire Line
	20260 12510 20260 12710
Wire Wire Line
	18570 12930 18570 12810
Wire Wire Line
	20260 13530 20260 13480
Connection ~ 20260 13530
Wire Wire Line
	20110 12930 19970 12930
Wire Wire Line
	20110 13530 20110 12930
Wire Wire Line
	20260 13530 20110 13530
Wire Wire Line
	18450 13030 18110 13030
Connection ~ 18450 13030
Wire Wire Line
	19970 13870 19970 13230
Wire Wire Line
	18450 13870 19970 13870
Wire Wire Line
	18450 13030 18450 13870
Wire Wire Line
	20260 13130 20260 13180
Connection ~ 20260 13130
Wire Wire Line
	19970 13130 20260 13130
Wire Wire Line
	20260 13050 20260 13130
Wire Wire Line
	20260 13990 20260 13530
Wire Wire Line
	19270 13990 20260 13990
Wire Wire Line
	19270 13930 19270 13990
Text GLabel 17960 13030 0    50   Input ~ 0
5V
Wire Wire Line
	18570 13030 18450 13030
NoConn ~ 19970 13030
NoConn ~ 18570 13230
NoConn ~ 18570 13130
$Comp
L mengpaneel-rescue:LTC3631EMS8E#TRPBF-LTC3631EMS8E#TRPBF IC12
U 1 1 6EDA0DBB
P 18570 12930
F 0 "IC12" H 19270 13195 50  0000 C CNN
F 1 "LTC3631EMS8E#TRPBF" H 19270 13104 50  0000 C CNN
F 2 "LTC3631EMS8E#TRPBF:SOP65P490X110-9N" H 19820 13030 50  0001 L CNN
F 3 "https://www.arrow.com/en/products/ltc3631ems8etrpbf/analog-devices" H 19820 12930 50  0001 L CNN
F 4 "Conv DC-DC 4.5V to 45V Synchronous Step Down Single-Out 0.8V to 45V 0.1A Automotive 8-Pin MSOP EP T/" H 19820 12830 50  0001 L CNN "Description"
F 5 "1.1" H 19820 12730 50  0001 L CNN "Height"
F 6 "" H 19820 12630 50  0001 L CNN "Mouser Part Number"
F 7 "" H 19820 12530 50  0001 L CNN "Mouser Price/Stock"
F 8 "Analog Devices" H 19820 12430 50  0001 L CNN "Manufacturer_Name"
F 9 "LTC3631EMS8E#TRPBF" H 19820 12330 50  0001 L CNN "Manufacturer_Part_Number"
	1    18570 12930
	1    0    0    -1  
$EndComp
$Comp
L Device:L L2
U 1 1 6EC2F47B
P 18570 12660
F 0 "L2" H 18623 12706 50  0000 L CNN
F 1 "100µ" H 18623 12615 50  0000 L CNN
F 2 "" H 18570 12660 50  0001 C CNN
F 3 "~" H 18570 12660 50  0001 C CNN
	1    18570 12660
	1    0    0    -1  
$EndComp
$Comp
L Device:R R60
U 1 1 6EC2D09A
P 20260 13330
F 0 "R60" H 20330 13376 50  0000 L CNN
F 1 "71,5k" H 20330 13285 50  0000 L CNN
F 2 "" V 20190 13330 50  0001 C CNN
F 3 "~" H 20260 13330 50  0001 C CNN
	1    20260 13330
	1    0    0    -1  
$EndComp
$Comp
L Device:R R59
U 1 1 6EC2ADD0
P 20260 12900
F 0 "R59" H 20330 12946 50  0000 L CNN
F 1 "1M" H 20330 12855 50  0000 L CNN
F 2 "" V 20190 12900 50  0001 C CNN
F 3 "~" H 20260 12900 50  0001 C CNN
	1    20260 12900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C45
U 1 1 6EC26BBD
P 18110 13300
F 0 "C45" H 18225 13346 50  0000 L CNN
F 1 "1µ" H 18225 13255 50  0000 L CNN
F 2 "" H 18148 13150 50  0001 C CNN
F 3 "~" H 18110 13300 50  0001 C CNN
	1    18110 13300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C46
U 1 1 6EC228C7
P 20770 13130
F 0 "C46" H 20885 13176 50  0000 L CNN
F 1 "10µ" H 20885 13085 50  0000 L CNN
F 2 "" H 20808 12980 50  0001 C CNN
F 3 "~" H 20770 13130 50  0001 C CNN
	1    20770 13130
	1    0    0    -1  
$EndComp
$Comp
L mengpaneel-rescue:STM32F411CEU7-STM32F411CEU7 IC10
U 1 1 60AC800D
P 14880 9280
F 0 "IC10" H 14660 9760 50  0000 L CNN
F 1 "STM32F411CEU7" H 14430 9700 50  0000 L TNN
F 2 "STM32F411CEU7:QFN50P700X700X65-49N-D" H 16530 9780 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/2/STM32F411CEU7.pdf" H 16530 9680 50  0001 L CNN
F 4 "STMICROELECTRONICS - STM32F411CEU7 - MCU, 32BIT, CORTEX-M4, 100MHZ, UFQFPN-48" H 16530 9580 50  0001 L CNN "Description"
F 5 "0.65" H 16530 9480 50  0001 L CNN "Height"
F 6 "511-STM32F411CEU7" H 16530 9380 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/STMicroelectronics/STM32F411CEU7/?qs=k5OWtXsTJaprF2FuhgNRlw%3D%3D" H 16530 9280 50  0001 L CNN "Mouser Price/Stock"
F 8 "STMicroelectronics" H 16530 9180 50  0001 L CNN "Manufacturer_Name"
F 9 "STM32F411CEU7" H 16530 9080 50  0001 L CNN "Manufacturer_Part_Number"
	1    14880 9280
	1    0    0    -1  
$EndComp
NoConn ~ 15880 11080
NoConn ~ 15980 11080
NoConn ~ 16680 10080
NoConn ~ 16680 10180
NoConn ~ 16680 10280
NoConn ~ 16680 10380
NoConn ~ 15480 8580
NoConn ~ 15580 8580
NoConn ~ 15980 8580
NoConn ~ 16180 8580
NoConn ~ 16280 8580
NoConn ~ 16680 9480
NoConn ~ 16680 9580
NoConn ~ 16680 9680
NoConn ~ 14880 9480
NoConn ~ 14880 9580
NoConn ~ 14880 9380
Wire Wire Line
	15780 11080 15780 11180
Wire Wire Line
	15780 11180 15810 11180
Wire Wire Line
	15810 11180 15810 11810
Wire Wire Line
	15610 11180 15680 11180
Wire Wire Line
	15680 11180 15680 11080
Wire Wire Line
	15610 11180 15610 11810
Wire Wire Line
	15580 11080 15580 11720
Wire Wire Line
	15580 11720 15290 11720
Wire Wire Line
	15480 11080 15480 11680
Wire Wire Line
	15480 11680 14950 11680
Wire Wire Line
	15380 11080 15380 11630
Wire Wire Line
	15380 11630 14620 11630
$Comp
L touchpad_mengpaneel_library:Touchpad_mp_LR U13
U 1 1 609ED08E
P 8640 10800
F 0 "U13" H 8715 11165 50  0000 C CNN
F 1 "Touchpad_mp_LR" H 8715 11074 50  0000 C CNN
F 2 "schema_tp:touchpad_footprint_LR" H 8640 11300 50  0001 C CNN
F 3 "" H 8640 11300 50  0001 C CNN
	1    8640 10800
	1    0    0    -1  
$EndComp
Text GLabel 6900 11470 3    50   Input ~ 0
laag_12
Text GLabel 6790 11470 3    50   Input ~ 0
laag_11
Text GLabel 6680 11470 3    50   Input ~ 0
laag_10
Text GLabel 6560 11470 3    50   Input ~ 0
laag_9
Text GLabel 6440 11470 3    50   Input ~ 0
laag_8
Text GLabel 6330 11470 3    50   Input ~ 0
laag_7
Text GLabel 6220 11470 3    50   Input ~ 0
laag_6
Text GLabel 6100 11470 3    50   Input ~ 0
laag_5
Text GLabel 5990 11470 3    50   Input ~ 0
laag_4
Text GLabel 5870 11470 3    50   Input ~ 0
laag_3
Text GLabel 5760 11470 3    50   Input ~ 0
laag_2
Text GLabel 5650 11470 3    50   Input ~ 0
laag_1
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U8
U 1 1 6111DA03
P 4680 2400
F 0 "U8" H 4755 2765 50  0000 C CNN
F 1 "Touchpad_mp" H 4755 2674 50  0000 C CNN
F 2 "schema_tp:touchpad_footprint_normaal" H 4680 2900 50  0001 C CNN
F 3 "" H 4680 2900 50  0001 C CNN
	1    4680 2400
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U11
U 1 1 61122764
P 7970 2400
F 0 "U11" H 8045 2765 50  0000 C CNN
F 1 "Touchpad_mp" H 8045 2674 50  0000 C CNN
F 2 "" H 7970 2900 50  0001 C CNN
F 3 "" H 7970 2900 50  0001 C CNN
	1    7970 2400
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U17
U 1 1 611253C2
P 9870 2430
F 0 "U17" H 9945 2795 50  0000 C CNN
F 1 "Touchpad_mp" H 9945 2704 50  0000 C CNN
F 2 "" H 9870 2930 50  0001 C CNN
F 3 "" H 9870 2930 50  0001 C CNN
	1    9870 2430
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U4
U 1 1 611282A8
P 3300 5890
F 0 "U4" H 3375 6255 50  0000 C CNN
F 1 "Touchpad_mp" H 3375 6164 50  0000 C CNN
F 2 "" H 3300 6390 50  0001 C CNN
F 3 "" H 3300 6390 50  0001 C CNN
	1    3300 5890
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U16
U 1 1 6112CC43
P 9810 6030
F 0 "U16" H 9885 6395 50  0000 C CNN
F 1 "Touchpad_mp" H 9885 6304 50  0000 C CNN
F 2 "" H 9810 6530 50  0001 C CNN
F 3 "" H 9810 6530 50  0001 C CNN
	1    9810 6030
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U2
U 1 1 6113144E
P 3200 7610
F 0 "U2" H 3275 7975 50  0000 C CNN
F 1 "Touchpad_mp" H 3275 7884 50  0000 C CNN
F 2 "" H 3200 8110 50  0001 C CNN
F 3 "" H 3200 8110 50  0001 C CNN
	1    3200 7610
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U14
U 1 1 611340E2
P 9730 7730
F 0 "U14" H 9805 8095 50  0000 C CNN
F 1 "Touchpad_mp" H 9805 8004 50  0000 C CNN
F 2 "" H 9730 8230 50  0001 C CNN
F 3 "" H 9730 8230 50  0001 C CNN
	1    9730 7730
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U3
U 1 1 61134CC0
P 3200 9290
F 0 "U3" H 3275 9655 50  0000 C CNN
F 1 "Touchpad_mp" H 3275 9564 50  0000 C CNN
F 2 "" H 3200 9790 50  0001 C CNN
F 3 "" H 3200 9790 50  0001 C CNN
	1    3200 9290
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U15
U 1 1 6113700C
P 9730 9430
F 0 "U15" H 9805 9795 50  0000 C CNN
F 1 "Touchpad_mp" H 9805 9704 50  0000 C CNN
F 2 "" H 9730 9930 50  0001 C CNN
F 3 "" H 9730 9930 50  0001 C CNN
	1    9730 9430
	1    0    0    -1  
$EndComp
$Comp
L touchpad_mengpaneel_library:Touchpad_mp U6
U 1 1 6111A5AB
P 3340 2380
F 0 "U6" H 3415 2745 50  0000 C CNN
F 1 "Touchpad_mp" H 3415 2654 50  0000 C CNN
F 2 "" H 3340 2880 50  0001 C CNN
F 3 "" H 3340 2880 50  0001 C CNN
	1    3340 2380
	1    0    0    -1  
$EndComp
Text GLabel 3990 3450 3    50   Input ~ 0
slider_1
Text Notes 3160 3820 0    50   ~ 0
MICROFOON
$EndSCHEMATC
