PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//717544/234443/2.47/10/3/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r145_30"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.3) (shapeHeight 1.45))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "SOP50P490X110-10N" (originalName "SOP50P490X110-10N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r145_30) (pt -2.225, 1) (rotation 90))
			(pad (padNum 2) (padStyleRef r145_30) (pt -2.225, 0.5) (rotation 90))
			(pad (padNum 3) (padStyleRef r145_30) (pt -2.225, 0) (rotation 90))
			(pad (padNum 4) (padStyleRef r145_30) (pt -2.225, -0.5) (rotation 90))
			(pad (padNum 5) (padStyleRef r145_30) (pt -2.225, -1) (rotation 90))
			(pad (padNum 6) (padStyleRef r145_30) (pt 2.225, -1) (rotation 90))
			(pad (padNum 7) (padStyleRef r145_30) (pt 2.225, -0.5) (rotation 90))
			(pad (padNum 8) (padStyleRef r145_30) (pt 2.225, 0) (rotation 90))
			(pad (padNum 9) (padStyleRef r145_30) (pt 2.225, 0.5) (rotation 90))
			(pad (padNum 10) (padStyleRef r145_30) (pt 2.225, 1) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.2 1.8) (pt 3.2 1.8) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 3.2 1.8) (pt 3.2 -1.8) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 3.2 -1.8) (pt -3.2 -1.8) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.2 -1.8) (pt -3.2 1.8) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 1.5) (pt 1.5 1.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.5 1.5) (pt 1.5 -1.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.5 -1.5) (pt -1.5 -1.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 -1.5) (pt -1.5 1.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 1) (pt -1 1.5) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.15 1.5) (pt 1.15 1.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 1.15 1.5) (pt 1.15 -1.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 1.15 -1.5) (pt -1.15 -1.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.15 -1.5) (pt -1.15 1.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.95 1.5) (pt -1.5 1.5) (width 0.2))
		)
	)
	(symbolDef "AD5290YRMZ10-R7" (originalName "AD5290YRMZ10-R7")

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
		(pin (pinNum 6) (pt 1000 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 1000 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 1000 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 1000 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 1000 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 800 mils 100 mils) (width 6 mils))
		(line (pt 800 mils 100 mils) (pt 800 mils -500 mils) (width 6 mils))
		(line (pt 800 mils -500 mils) (pt 200 mils -500 mils) (width 6 mils))
		(line (pt 200 mils -500 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 850 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 850 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "AD5290YRMZ10-R7" (originalName "AD5290YRMZ10-R7") (compHeader (numPins 10) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "A") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "B") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "VSS") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "GND") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "__CS") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "10" (pinName "W") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "9" (pinName "VDD") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "8" (pinName "SDO") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "7" (pinName "SDI") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "6" (pinName "CLK") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "AD5290YRMZ10-R7"))
		(attachedPattern (patternNum 1) (patternName "SOP50P490X110-10N")
			(numPads 10)
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
			)
		)
		(attr "Mouser Part Number" "584-AD5290YRMZ10-R7")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Analog-Devices/AD5290YRMZ10-R7?qs=NmRFExCfTkERE1iuOsYtFg%3D%3D")
		(attr "Manufacturer_Name" "Analog Devices")
		(attr "Manufacturer_Part_Number" "AD5290YRMZ10-R7")
		(attr "Description" "Digital Potentiometer ICs +/-15V 8-Bit DigiPOT")
		(attr "<Hyperlink>" "")
		(attr "<Component Height>" "1.1")
		(attr "<STEP Filename>" "AD5290YRMZ10-R7.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
