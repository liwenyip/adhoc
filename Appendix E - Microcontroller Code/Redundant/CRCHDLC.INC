; --------------------------------------------------------------------------
; The following code was sourced from the internet by Steven Sloots (2004).
;

;**********************************************************************
;                                                                     *
; Sub_Routine:     CrcHDLC                                            *
;                                                                     *
;            Macro for ^16+^12+^5+1, "CITT CRC16" - HDLC, X.25        *
;                                                                     *
; Arguments: newData    - data byte (destroyed)                       *
;            crcLow     - CRC accumulator low byte                    *
;            crcHigh    - 	CRC accumulator high byte                 *
;                                                                     *
;**********************************************************************
;                                                                     *
; Description: Macros to implement byte at a time ^16+^12+^5+1,       *
;              "CITT CRC16" - HDLC, X.25 16 bit CRC generation        *
;                                                                     *
;**********************************************************************
;    File Version:  1                                                 *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
CRCHDLC macro
;       oldCRC ^= newData
;       Modified to  - newData ^= oldCRC
	movf    crclow,W
	xorwf   newdata,F

;       oldCRC  = (oldCRC >> 8) | (oldCRC << 8)
;       Modified to - oldCRC  = (oldCRC >> 8) | newdata << 8)
	movf    crchigh,W
	movwf   crclow
	movf    newdata,W
	movwf   crchigh

;       oldCRC ^= (oldCRC & 0xFF00) << 4
	swapf   crchigh,W
	andlw   0xF0
	xorwf   crchigh,F

;       oldCRC ^= oldCRC >> 12
	swapf   crchigh,W
	andlw   0x0F
	xorwf   crclow,F

;       oldCRC ^= (oldCRC & 0xFF00) >> 5
	swapf   crchigh,F
	bcf     STATUS,C
	btfsc   crchigh,0
	bsf     STATUS,C
	rrcf     crchigh,W
	andlw   0xF8
	xorwf   crclow,F
	rrcf     crchigh,W
	andlw   0x07
	swapf   crchigh,F
	xorwf   crchigh,F

;	return

endm	
