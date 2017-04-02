;*************************************************************************
; SOFTWARE BUFFERS MODULE
; 
; Version 0.10
; 17/07/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;
; The RAM allocated for each buffer must not overlap two RAM banks. This allows
; faster loading and saving of the pointer values.
;
; Resources Requiring Exclusive Use:
; RAM Bank 4, FSR0
;
; Notes:
; 
;
; What you need to understand to work with this code:
; 
; 
; 
;
;*************************************************************************
; INCLUDE FILES
 LIST P=18C452, F=INHX32	;directive to define processor and file format
 #include <P18C452.INC>		;processor specific variable definitions
 #include "macrolib.inc" 	; common macros
 #include "swstack.inc"		; software stack

;*************************************************************************
; CONFIGURATION SECTION
 
;*************************************************************************
; CONSTANTS

getter equ 0
putter equ 1
bdata equ 2

;*************************************************************************
; VARIABLES

; Access RAM
 udata_acs
BUF_FLAGS	res 1
 #define	BUF_EOF		BUF_FLAGS, 0
 #define	BUF_FULL	BUF_FLAGS, 1
 #define	RXBUF_SW	BUF_FLAGS, 2
 #define	TXBUF_SW	BUF_FLAGS, 3
 #define	RXBUF_LOCK	BUF_FLAGS, 4
 #define	TXBUF_LOCK	BUF_FLAGS, 5

; Put all buffer storage in a high bank, as it will never be addressed directly
; Byte 0 = Read Pointer (Getter)
; Byte 1 = Write Pointer (Putter)
; Bytes 2->63 = Buffer Storage
.BUFFERS udata 0x400 
BUF0		res 0x40	; Buffer 0
BUF1		res 0x40	; Buffer 1
BUF2		res 0x40	; Buffer 2
BUF3		res 0x40	; Buffer 3

;*************************************************************************
; START OF CODE

 CODE



;***************************************************************
; MACRO: 		BUFPUT
; 
; Description:	Push WREG into the buffer.
; Arguments:	buf - The buffer to push to.
; Precond'ns:	-
; Postcond'ns:	WREG -> BUFn + PUTTERn++.
; Regs Used:	FSRO (Stacked).
;***************************************************************
BUFPUT macro buf
	
	; Context Saving
	SWPUSH	FSR0L, 2			; Stack FSR0.
	SWPUSH	BSR, 1				; Stack BSR.

	; Save WREG, Set BSR
	SWPUSH	WREG, 1				; WREG -> TOS.
	movlb	high buf			; Select bank containing buffer.
	
	; Load the address of the buffer & offset, write to the buffer.
	lfsr	0, buf + bdata		; Address of buffer data -> FSR0.
	movf	buf + putter, w		; Putter -> WREG.
	SWPOP	PLUSW0, 1			; TOS -> Buffer Addr + Putter.

	; Increment the putter and check if the buffer is full.
	incf	buf + putter		; Putter + 1 -> Putter.
	movlw	scnsz_low buf - 2	; Calculate buffer length.
	bcf		BUF_FULL			; Clear the FULL flag.
	cpfslt	buf + putter		; Is putter < buffer length?
	bsf		BUF_FULL			; NO - set the FULL flag.

	; Context Restoring
	SWPOP	BSR, 1				; Unstack BSR.
	SWPOP	FSR0L, 2			; Unstack FSR0.

	return

 endm

;***************************************************************
; MACRO: 		BUFGET
; 
; Description:	Pop to WREG from buffer.
; Arguments:	buf - The buffer to pop from.
; Precond'ns:	-
; Postcond'ns:	BUFn + PUTTERn-- -> WREG.
; Regs Used:	FSRO (Stacked).
;***************************************************************
BUFGET macro buf

	; Context Saving
	SWPUSH	FSR0L, 2			; Stack FSR0.
	SWPUSH	BSR, 1				; Stack BSR.
	movlb	high buf			; Select bank containing buffer.

	; Load the address of the buffer & offset, read from the buffer.
	lfsr	0, buf + bdata		; Address of buffer data -> FSR0.
	movf	buf + getter, w		; Getter -> WREG.
	SWPUSH	PLUSW0, 1			; Buffer Addr + Getter -> TOS.

	; Increment the getter and check for EOF.
	incf	buf + getter		; Increment the getter.
	movf	buf + putter, w		; Load up the putter.
	bcf		BUF_EOF				; Clear the EOF flag.
	cpfslt	buf + getter		; Is getter < putter?
	bsf		BUF_EOF				; NO - Set the EOF flag.

	; Put result in WREG
	SWPOP	WREG, 1				; TOS -> WREG.

	; Context Restoring
	SWPOP	BSR, 1				; Unstack BSR.
	SWPOP	FSR0L, 2			; Unstack FSR0.

	return
 endm

;***************************************************************
; MACRO: 		BUFCLR
; 
; Description:	Clear buffer.
; Arguments:	buf - The buffer to clear.
; Precond'ns:	-
; Postcond'ns:	putter = getter = 0.
; Regs Used:
;***************************************************************
BUFCLR macro buf
	clrf	buf + getter		; 0 -> Getter.
	clrf	buf + putter		; 0 -> Putter.
	return
 endm

;***************************************************************
; MACRO: 		BUFRST
; 
; Description:	Reset buffer.
; Arguments:	buf - The buffer to clear.
; Precond'ns:	-
; Postcond'ns:	getter = 0.
; Regs Used:
;***************************************************************
BUFRST macro buf
	clrf	buf + getter		; 0 -> Getter.
	return
 endm


; Subroutines for each buffer
PUT0:	BUFPUT 0
GET0:	BUFGET 0
CLR0:	BUFCLR 0

PUT1:	BUFPUT 1
GET1:	BUFGET 1
CLR1:	BUFCLR 1

PUT2:	BUFPUT 2
GET2:	BUFGET 2
CLR2:	BUFCLR 2

PUT3:	BUFPUT 3
GET3:	BUFGET 3
CLR3:	BUFCLR 3

RXBUFA_PUT:
	btfss	RXBUF_SW
	bra		PUT0
	bra		PUT1
RXBUFA_GET:
	btfss	RXBUF_SW
	bra		GET0
	bra		GET1
RXBUFA_CLR:
	btfss	RXBUF_SW
	bra		CLR0
	bra		CLR1
RXBUFA_RST:
	btfss	RXBUF_SW
	bra		RST0
	bra		RST1

RXBUFB_PUT:
	btfss	RXBUF_SW
	bra		PUT1
	bra		PUT0
RXBUFB_GET:
	btfss	RXBUF_SW
	bra		GET1
	bra		GET0
RXBUFB_CLR:
	btfss	RXBUF_SW
	bra		CLR1
	bra		CLR0
RXBUFB_RST:
	btfss	RXBUF_SW
	bra		RST1
	bra		RST0

TXBUFA_PUT:
	btfss	TXBUF_SW
	bra		PUT2
	bra		PUT3
TXBUFA_GET:
	btfss	TXBUF_SW
	bra		GET2
	bra		GET3
TXBUFA_CLR:
	btfss	TXBUF_SW
	bra		CLR2
	bra		CLR3
TXBUFA_RST:
	btfss	TXBUF_SW
	bra		RST2
	bra		RST3

TXBUFB_PUT:
	btfss	TXBUF_SW
	bra		PUT3
	bra		PUT2
TXBUFB_GET:
	btfss	TXBUF_SW
	bra		GET3
	bra		GET2
TXBUFB_CLR:
	btfss	TXBUF_SW
	bra		CLR3
	bra		CLR2
TXBUFB_RST:
	btfss	TXBUF_SW
	bra		RST3
	bra		RST2


 end
