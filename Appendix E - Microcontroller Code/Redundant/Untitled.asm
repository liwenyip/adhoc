	movf	instruction, w
	addwf	PCL
	bra		INSTRUCTION_00
	bra		INSTRUCTION_01
	bra		INSTRUCTION_02
	bra		INSTRUCTION_03
	bra		INSTRUCTION_04
	bra		INSTRUCTION_05
	bra		INSTRUCTION_06
	bra		INSTRUCTION_07
	bra		INSTRUCTION_08
	bra		INSTRUCTION_09
	bra		INSTRUCTION_0A
	bra		INSTRUCTION_0B
	bra		INSTRUCTION_0C
	bra		INSTRUCTION_0D
	bra		INSTRUCTION_0E
	bra		INSTRUCTION_0F



	movf	instruction, w
	sublw	0x00
	bz		INSTRUCTION_00
	
	movf	instruction, w
	sublw	0x01
	bz		INSTRUCTION_01
	
	movf	instruction, w
	sublw	0x02
	bz		INSTRUCTION_02
	
	movf	instruction, w
	sublw	0x03
	bz		INSTRUCTION_03
	
	movf	instruction, w
	sublw	0x04
	bz		INSTRUCTION_04
	
