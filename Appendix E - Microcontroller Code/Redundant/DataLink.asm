;*************************************************************************
; DATA LINK LAYER MODULE
; 
; Version 0.10
; 17/07/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
; Resources Requiring Exclusive Use:
; 
;
; Things you must do to use this module:
;
; Notes:
;
; What you need to understand to work with this code:
; 
; Dependencies:
; 
;
;*************************************************************************
; INCLUDE FILES
 LIST P=18C452, F=INHX32	;directive to define processor and file format
 #include <P18C452.INC>		;processor specific variable definitions
 #include "macrolib.inc" 	; common macros
 #include "buffers.inc"


;*************************************************************************
; CONFIGURATION SECTION


;******************************************************************************
; VARIABLE DEFINITIONS

 udata
 
; Variables used by CRC Subroutine
crclow		RES 1 	; CRC accumulator low byte.
crchigh		RES 1 	; CRC accumulator high byte.
newdata		RES 1 	; New data to feed into CRC.

; 



 code

;***************************************************************
; SUBROUTINE: 	CALC_CRC
; 	
; Description:	Calculate the CRC on a buffer
; Precond'ns:	FSR1 has been loaded with the address of a buffer.
;				Use the BUFSEL_xxx subroutine in <buffers.asm>.
; Postcond'ns:	crclow and crchigh contain the calculated crc.
; Regs Used:	FSR1
;***************************************************************
CALC_CRC:
	call	BUFRST			; Reset the getter <BUFRST in buffers.asm>
	clrf	crclow			; Clear the CRC register.
	clrf	crchigh			; ...

_CALC_CRC_LOOP:
	call	BUFGET			; Get a data byte. <BUFGET in buffers.asm>
	movwf	newdata			; data byte -> newdata.
	CRCHDLC					; Process the data byte.

	; Loop until we have processed all the data in the buffer.
	btfss	buf_eof			; Is there any more data? <buf_eof in buffers.asm>
	bra		_CALC_CRC_LOOP	; YES - process the next data byte.
	return					; NO - all done.
	


;***************************************************************
; SUBROUTINE: 	CALC_CRC_OUTGOING
; 	
; Description:	Calculate and append the CRC on an outgoing packet.
; Precond'ns:	TXB Buffer contains an outgoing packet.
; Postcond'ns:	CRC is appended to the TXB Buffer.
; Regs Used:	FSR1
;***************************************************************
CALC_CRC_OUTGOING:
	call	BUFSEL_TXB		; Select the Tx "B" Buffer. <BUFSEL_xxx in buffers.asm>

	call	CALC_CRC		; Calculate the CRC on the buffer.

	movf	crchigh			; Load the crc high byte...
	call	BUFPUT			; And append it to the buffer. <BUFPUT in buffers.asm>
	movf	crclow			; Load the crc low byte...
	call	BUFPUT			; And append it to the buffer. <BUFPUT in buffers.asm>

	return

;***************************************************************
; SUBROUTINE: 	CALC_CRC_INCOMING
; 	
; Description:	Calculate and verify the CRC on an incoming packet.
; Precond'ns:	RXB Buffer contains an incoming packet.
; Postcond'ns:	
; Regs Used:	FSR1
;***************************************************************
CALC_CRC_INCOMING:
	call	BUFSEL_RXB		; Select the Rx "B" Buffer. <BUFSEL_xxx in buffers.asm>

	call	CALC_CRC		; Calculate the CRC on the buffer.

	; Verify that the CRC for the packet is zero.
	tstfsz	crchigh			; Is the high byte zero?
	bra		_CRC_BAD		; NO - CRC is bad.
	tstfsz	crclow			; Is the low byte zero?
	bra		_CRC_BAD		; NO - CRC is bad.
		
; Do here whatever needs to be done if the CRC is good.
_CRC_GOOD:
	return

; Do here whatever needs to be done if the CRC is bad.
_CRC_BAD:
	return

	









; --------------------------------------------------------------------------
; The following code was sourced from the internet by Steven Sloots (2004).
;


;***********************************************************************
;**********************************************************************
;                                                                     *
;Sub_Routine:     CrcHDLC                                             *
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
MACRO CRCHDLC
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
	
 end
