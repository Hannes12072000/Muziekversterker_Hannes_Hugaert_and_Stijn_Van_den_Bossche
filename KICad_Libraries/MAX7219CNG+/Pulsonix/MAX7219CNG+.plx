PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//1408/429985/2.49/24/3/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c116_h76"
		(holeDiam 0.76)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.16) (shapeHeight 1.16))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.16) (shapeHeight 1.16))
	)
	(padStyleDef "s116_h76"
		(holeDiam 0.76)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.16) (shapeHeight 1.16))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 1.16) (shapeHeight 1.16))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "DIP794W56P254L3054H457Q24N" (originalName "DIP794W56P254L3054H457Q24N")
		(multiLayer
			(pad (padNum 1) (padStyleRef s116_h76) (pt -3.97, 13.97) (rotation 90))
			(pad (padNum 2) (padStyleRef c116_h76) (pt -3.97, 11.43) (rotation 90))
			(pad (padNum 3) (padStyleRef c116_h76) (pt -3.97, 8.89) (rotation 90))
			(pad (padNum 4) (padStyleRef c116_h76) (pt -3.97, 6.35) (rotation 90))
			(pad (padNum 5) (padStyleRef c116_h76) (pt -3.97, 3.81) (rotation 90))
			(pad (padNum 6) (padStyleRef c116_h76) (pt -3.97, 1.27) (rotation 90))
			(pad (padNum 7) (padStyleRef c116_h76) (pt -3.97, -1.27) (rotation 90))
			(pad (padNum 8) (padStyleRef c116_h76) (pt -3.97, -3.81) (rotation 90))
			(pad (padNum 9) (padStyleRef c116_h76) (pt -3.97, -6.35) (rotation 90))
			(pad (padNum 10) (padStyleRef c116_h76) (pt -3.97, -8.89) (rotation 90))
			(pad (padNum 11) (padStyleRef c116_h76) (pt -3.97, -11.43) (rotation 90))
			(pad (padNum 12) (padStyleRef c116_h76) (pt -3.97, -13.97) (rotation 90))
			(pad (padNum 13) (padStyleRef c116_h76) (pt 3.97, -13.97) (rotation 90))
			(pad (padNum 14) (padStyleRef c116_h76) (pt 3.97, -11.43) (rotation 90))
			(pad (padNum 15) (padStyleRef c116_h76) (pt 3.97, -8.89) (rotation 90))
			(pad (padNum 16) (padStyleRef c116_h76) (pt 3.97, -6.35) (rotation 90))
			(pad (padNum 17) (padStyleRef c116_h76) (pt 3.97, -3.81) (rotation 90))
			(pad (padNum 18) (padStyleRef c116_h76) (pt 3.97, -1.27) (rotation 90))
			(pad (padNum 19) (padStyleRef c116_h76) (pt 3.97, 1.27) (rotation 90))
			(pad (padNum 20) (padStyleRef c116_h76) (pt 3.97, 3.81) (rotation 90))
			(pad (padNum 21) (padStyleRef c116_h76) (pt 3.97, 6.35) (rotation 90))
			(pad (padNum 22) (padStyleRef c116_h76) (pt 3.97, 8.89) (rotation 90))
			(pad (padNum 23) (padStyleRef c116_h76) (pt 3.97, 11.43) (rotation 90))
			(pad (padNum 24) (padStyleRef c116_h76) (pt 3.97, 13.97) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -4.96 16.315) (pt 4.96 16.315) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 4.96 16.315) (pt 4.96 -16.315) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 4.96 -16.315) (pt -4.96 -16.315) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -4.96 -16.315) (pt -4.96 16.315) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.935 16.065) (pt 3.935 16.065) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.935 16.065) (pt 3.935 -16.065) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.935 -16.065) (pt -3.935 -16.065) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.935 -16.065) (pt -3.935 16.065) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.935 14.795) (pt -2.665 16.065) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -4.55 16.065) (pt 3.935 16.065) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.935 -16.065) (pt 3.935 -16.065) (width 0.2))
		)
	)
	(symbolDef "MAX7219CNG+" (originalName "MAX7219CNG+")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 0 mils -600 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -625 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 0 mils -700 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -725 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 0 mils -800 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -825 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 0 mils -900 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -925 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 11) (pt 0 mils -1000 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -1025 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 12) (pt 0 mils -1100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -1125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 13) (pt 1500 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 14) (pt 1500 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 15) (pt 1500 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 16) (pt 1500 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 17) (pt 1500 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 18) (pt 1500 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 19) (pt 1500 mils -600 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -625 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 20) (pt 1500 mils -700 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -725 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 21) (pt 1500 mils -800 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -825 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 22) (pt 1500 mils -900 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -925 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 23) (pt 1500 mils -1000 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -1025 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 24) (pt 1500 mils -1100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1270 mils -1125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 1300 mils 100 mils) (width 6 mils))
		(line (pt 1300 mils 100 mils) (pt 1300 mils -1200 mils) (width 6 mils))
		(line (pt 1300 mils -1200 mils) (pt 200 mils -1200 mils) (width 6 mils))
		(line (pt 200 mils -1200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1350 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 1350 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "MAX7219CNG+" (originalName "MAX7219CNG+") (compHeader (numPins 24) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "DIN") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "DIG 0") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "DIG 4") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "GND_1") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "DIG 6") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "6" (pinName "DIG 2") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "7" (pinName "DIG 3") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "8" (pinName "DIG 7") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "9" (pinName "GND_2") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "10" (pinName "DIG 5") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "11" (pinName "DIG 1") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "12" (pinName "LOAD (__CS)") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "24" (pinName "DOUT") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "23" (pinName "SEG D") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "22" (pinName "SEG DP") (partNum 1) (symPinNum 15) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "21" (pinName "SEG E") (partNum 1) (symPinNum 16) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "20" (pinName "SEG C") (partNum 1) (symPinNum 17) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "19" (pinName "V+") (partNum 1) (symPinNum 18) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "18" (pinName "ISET") (partNum 1) (symPinNum 19) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "17" (pinName "SEG G") (partNum 1) (symPinNum 20) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "16" (pinName "SEG B") (partNum 1) (symPinNum 21) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "15" (pinName "SEG F") (partNum 1) (symPinNum 22) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "14" (pinName "SEG A") (partNum 1) (symPinNum 23) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "13" (pinName "CLK") (partNum 1) (symPinNum 24) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "MAX7219CNG+"))
		(attachedPattern (patternNum 1) (patternName "DIP794W56P254L3054H457Q24N")
			(numPads 24)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
				(padNum 13) (compPinRef "13")
				(padNum 14) (compPinRef "14")
				(padNum 15) (compPinRef "15")
				(padNum 16) (compPinRef "16")
				(padNum 17) (compPinRef "17")
				(padNum 18) (compPinRef "18")
				(padNum 19) (compPinRef "19")
				(padNum 20) (compPinRef "20")
				(padNum 21) (compPinRef "21")
				(padNum 22) (compPinRef "22")
				(padNum 23) (compPinRef "23")
				(padNum 24) (compPinRef "24")
			)
		)
		(attr "Mouser Part Number" "700-MAX7219CNG")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Maxim-Integrated/MAX7219CNG%2b?qs=1THa7WoU59Gme2Z0GeVXUQ%3D%3D")
		(attr "Manufacturer_Name" "Maxim Integrated")
		(attr "Manufacturer_Part_Number" "MAX7219CNG+")
		(attr "Description" "8-digit LED display driver,MAX7219CNG,BP MAX7219CNG+, LED Driver, 8-Digits 64-Segments, 5 V, 24-Pin PDIP N")
		(attr "<Hyperlink>" "http://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf")
		(attr "<Component Height>" "4.572")
		(attr "<STEP Filename>" "MAX7219CNG+.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
