;*************************************************************************
; CRC Checking Module
; 
; Version 1.00
; 16/09/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************
;
; Description: 
;
;
; Dependencies:
;
; Resources Used:
;
; Things you must do to use this module:
;
; What you need to understand to work with this code:
;
; Notes:
;

;*************************************************************************
; MASTER HEADER FILE
 #include "MasterHeader.inc"

;*************************************************************************
; CONFIGURATION CONSTANTS

;*************************************************************************
; CONSTANTS


;******************************************************************************
; IMPORTED VARIABLES

;******************************************************************************
; GLOBAL VARIABLES
.crc_globals udata_acs

crclow	res 1
crchigh	res 1

 global crclow, crchigh

;******************************************************************************
; LOCAL VARIABLES
.crc_locals udata_acs

newdata res 1
 
;******************************************************************************
; IMPORTED SUBROUTINES

 #include "Buffers.inc"

;******************************************************************************
; EXPORTED SUBROUTINES

 global CALC_CRC

;******************************************************************************
; START OF CODE

 CODE

; --------------------------------------------------------------------------
; The following mcaro was sourced from the internet by Steven Sloots (2004).
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

 endm


;***************************************************************
; SUBROUTINE: 	CALC_CRC
; 	
; Description:	Calculate the CRC on a buffer
; Precond'ns:	A buffer has been selected using BUFSELx in <buffers.asm>.
; Postcond'ns:	crclow and crchigh contain the calculated crc.
; Regs Used:	
;***************************************************************
CALC_CRC:
	movlw	0x00			; Reset the buffer cursor
	BUF_SETCURSOR			; 

	clrf	crclow			; Clear the CRC registers.
	clrf	crchigh			; ...

_CALC_CRC_LOOP:
	BUF_GET					; Get a data byte. 
	movwf	newdata			; data byte -> newdata.
	CRCHDLC					; Process the data byte.

	; Loop until we have processed all the data in the buffer.
	BUF_SNEOF				; Was that the last byte in the buffer?
	return					; NO - all done.
	bra		_CALC_CRC_LOOP	; YES - process the next data byte.
	
 end
