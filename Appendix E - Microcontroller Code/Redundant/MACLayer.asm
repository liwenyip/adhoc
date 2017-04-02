;*************************************************************************
; MEDIA ACCESS CONTROL LAYER MODULE
; 
; Version 1.00
; 24/08/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************
;
; Description: CRC -> CSMA -> TX.... RX -> SYNC -> CRC
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


; ---------------------------------------------------------------
; From <PhysicalLayer.asm>
 extern phy_flags						; Status Flags.
 #define clock_detect	phy_flags, 0	; Set when a valid clock is detected.
 #define bit_received	phy_flags, 1	; Set when a bit has been decoded.
 #define txbuf_empty	phy_flags, 2	; Set when the transmit buffer is empty.

 extern shiftreg_l, shiftreg_h			; Physical Layer shift register.

; Fom <SoftwareTimers.asm>
 extern	TMR4L, TMR4H, TMR4F

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES


;******************************************************************************
; LOCAL VARIABLES

; Local flags
local_flags	res 1
 #define got_sof		local_flags, 0	; Set when we have detected the start of frame.
 #define got_framelen	local_flags, 1	; Set when we have received frame length.
 #define sent_sof		local_flags, 2	; Set when we have sent the start of frame.
 #define sent_framelen	local_flags, 3	; Set when we have sent the frame length.
 #define sent_data		local_flags, 4	; Set when we have sent all the data bytes.
 #define csma_cts		local_flags, 5	; Set when we are clear to transmit a packet.


; Receiver Variables
bitcount	res 1
bytecount	res 1

; CRC Variables
crclow		RES 1 	; CRC accumulator low byte.
crchigh		RES 1 	; CRC accumulator high byte.
newdata		RES 1 	; New data to feed into CRC.

;******************************************************************************
; IMPORTED SUBROUTINES

; From <PhysicalLayer.asm>
 extern		TX_BYTE					; Put a byte in the transmit buffer.

;******************************************************************************
; EXPORTED SUBROUTINES

 global ENCODE_FRAMES, DECODE_FRAMES

;******************************************************************************
; START OF CODE

 CODE


;***************************************************************
; SUBROUTINE: 	MAC_INIT
; 
; Description:	Initialisation for MAC layer.
; Precond'ns:	-
; Postcond'ns:	
; Regs Used:	
;***************************************************************
MAC_INIT:

	return









;***************************************************************
; SUBROUTINE: 	DECODE_FRAMES
; 	
; Description:
; Precond'ns:	
; Postcond'ns:	
; Externals:
; Regs Used:	
;***************************************************************
DECODE_FRAMES:

; ---------------------------------------------------------------
; Check what we have to do.




;***************************************************************
; SUBROUTINE: 	ENCODE_FRAMES
; 	
; Description:
; Precond'ns:	
; Postcond'ns:	
; Externals:
; Regs Used:	
;***************************************************************
ENCODE_FRAMES:

; ---------------------------------------------------------------
; Check what we have to do.
	
	btfss	csma_cts			; Are we clear to transmit?
	return						; NO - get outta here.	

	btfss	txbuf_empty			; Is the bit encoder ready for another byte?
	return						; NO - get outta here.

	tstfsz	bytecount			; Have we sent all our preamble bytes?
	bra		_SEND_PREAMBLE		; NO - send a preamble byte.
	
	btfss	sent_sof			; Have we sent the start of frame byte?
	bra		_SEND_SOF			; NO - send a SOF byte.

	btfss	sent_framelen		; Have we sent the frame length?
	bra		_SEND_FRAMELEN		; NO - send the frame length.

	; If we get to here then we are transmitting data bytes.

; ---------------------------------------------------------------
; Transmit a DATA byte.
_SEND_DATABYTE:
	call	BUFSEL1				; Select Buffer1 <BUFSEL1 in buffers.asm>
	call	BUFGET				; Get a byte from the buffer <BUFGET in buffers.asm>
	call	TX_BYTE				; Put it in the transmit buffer.

	; If we have transmitted all the data, clean up.
	btfss	buf_eof				; Have we reached the end of the buffer?
	return						; NO
	bsf		sent_data			; YES - we are done transmitting.
	bcf		csma_cts			; Prevent this subroutine running again.
	return

; ---------------------------------------------------------------
; Transmit a preamble byte.
_SEND_PREAMBLE:
	; We will use bytecount to keep track of how many preamble bytes we have to go.
	decf	bytecount			; Clear the flag that got us here.
	movlw	PREAMBLE_BYTE		; Get the preamble byte.
	call	TX_BYTE				; Put it in the transmit buffer.
	return
	
; ---------------------------------------------------------------
; Transmit a SOF byte.
_SEND_SOF:
	bsf		sent_sof			; Cross the SOF off the to do list.
	movlw	SOF_BYTE			; Get the SOF byte.
	call	TX_BYTE				; Put it in the transmit buffer.
	return
	
; ---------------------------------------------------------------
; Transmit a frame length byte.
_SEND_FRAMELEN:
	bsf		sent_framelen		; Cross the len byte off the to do list.
	call	BUFSEL1				; Select Buffer 1 <BUFSEL1 in buffers.asm>
	call	BUFLEN				; Get the buffer data length. <BUFLEN in buffers.asm>
	call	TX_BYTE				; Put it in the transmit buffer.
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
	call	BUFCSR			; ...

	clrf	crclow			; Clear the CRC registers.
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
; Precond'ns:	Buffer 1 contains an outgoing packet.
; Postcond'ns:	CRC is appended to Buffer 1.
; Regs Used:	
;***************************************************************
CALC_CRC_OUTGOING:
	call	BUFSEL1			; Select Buffer 1 <BUFSELx in buffers.asm>

	call	CALC_CRC		; Calculate the CRC on the buffer.

	movf	crchigh			; Load the crc high byte...
	call	BUFPUT			; And append it to the buffer. <BUFPUT in buffers.asm>
	movf	crclow			; Load the crc low byte...
	call	BUFPUT			; And append it to the buffer. <BUFPUT in buffers.asm>

	call	BUFEND			; Mark the new end of the buffer.
	
	return

;***************************************************************
; SUBROUTINE: 	CALC_CRC_INCOMING
; 	
; Description:	Calculate and verify the CRC on an incoming packet.
; Precond'ns:	Buffer 0 contains an incoming packet.
; Postcond'ns:	
; Regs Used:	FSR1
;***************************************************************
CALC_CRC_INCOMING:
	call	BUFSEL0			; Select Buffer 0. <BUFSELx in buffers.asm>
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










	
 end
