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
buflen		res 1
buf_flags	res 1
 #define	buf_eof		buf_flags, 0
 #define	buf_full	buf_flags, 1
 #define	rxbuf_sw	buf_flags, 2
 #define	txbuf_sw	buf_flags, 3
 #define	rxbuf_lock	buf_flags, 4
 #define	txbuf_lock	buf_flags, 5

; Put all buffer storage in a high bank, as it will never be addressed directly
; Byte 0 = Read Pointer (Getter)
; Byte 1 = Write Pointer (Putter)
; Bytes 2->63 = Buffer Storage
.BUFFERS udata 0x400 
buf0		res 0x40	; Buffer 0
buf1		res 0x40	; Buffer 1
buf2		res 0x40	; Buffer 2
buf3		res 0x40	; Buffer 3



;*************************************************************************
; GLOBALLY AVAILABLE LABELS

 global		buf_flags
 global		BUFPUT, BUFGET, BUFCLR, BUFRST
 global		BUFSEL_RXA, BUFSEL_RXB, BUFSEL_TXA, BUFSEL_TXB


;*************************************************************************
; START OF CODE

 CODE

;***************************************************************
; SUBROUTINE: 	BUFPUT
; 
; Description:	Push WREG into the buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR1.
;					i.e. *FSR1 = Buffer[0]
;				- Length of buffer has been loaded to buflen.
; Postcond'ns:	WREG -> Buffer[Putter++]
; Regs Used:	FSR1
;***************************************************************
BUFPUT:

	; Getter = buf + 0
	; Putter = buf + 1
	; Data = buf + 2 -> buf + 63

	; Stack the W Register
	SWPUSH	WREG				; WREG -> TOS.

	; Increment the putter.
	incf	PREINC1				; *FSR1 = Buffer[1] = Putter, Increment Putter.
	movf	POSTDEC1, w			; Putter -> WREG, *FSR1 = Buffer[0].

	; Check if the buffer is full.
	bcf		buf_full			; Clear the FULL flag.
	cpfsgt	buflen				; Is Putter >= buflen?
	bsf		buf_full			; YES - set the FULL flag.
	
	; Write to the buffer.
	SWPOP	PLUSW1				; TOS -> Buffer Data + Putter.

	return

;***************************************************************
; SUBROUTINE:	BUFGET
; 
; Description:	Pop to WREG from buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR1.
;					i.e. *FSR1 = Buffer[0]
;				- Length of buffer has been loaded to buflen.
; Postcond'ns:	buffer[getter++] -> WREG, FSR1 = Buffer[0]
; Regs Used:	FSR1
;***************************************************************
BUFGET:

	; Getter = buffer[0]
	; Putter = buffer[1]
	; Data = buffer[2] -> buffer[63]

	; Increment the getter.
	incf	INDF1				; Increment Getter.
	movf	POSTINC1, w			; Getter -> WREG, *FSR1 = Buffer[1] = Putter.

	; Check for EOF.
	bcf		buf_eof				; Clear the EOF flag.
	cpfsgt	POSTDEC1			; Is Getter >= Putter?, *FSR1 = Buffer[0].
	bsf		buf_eof				; YES - Set the EOF flag.

	; Read from the buffer
	movf	PLUSW1, W			; Buffer Addr + Getter -> WREG.

	return

;***************************************************************
; SUBROUTINE: 	BUFCLR
; 
; Description:	(Re)initialise buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR1.
; Postcond'ns:	- Putter = 0.
;				- Getter = 0.
;				- Length of buffer has been loaded to buflen.
; Regs Used:	FSR1
;***************************************************************
BUFCLR:
	movlw	0x02		; Buffer[2] is where the data is at.
	movwf	POSTINC1	; 2 -> Getter, FSR1 points to Putter.
	movwf	POSTDEC1	; 2 -> Putter, FSR1 points to Getter.
	movlw	scnsz_low buf 		; Load buffer length.
	movwf	buflen
	return

;***************************************************************
; SUBROUTINE: 	BUFRST
; 
; Description:	Reset getter of a buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR1.
; Postcond'ns:	- Getter = 0.
; Regs Used:	FSR1
;***************************************************************
BUFRST:
	clrf	INDF1		; 0 -> Getter, FSR1 Points to Getter.
	return

;***************************************************************
; SUBROUTINE: 	BUFLEN
; 
; Description:	Get the length of the data in the buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR1.
; Postcond'ns:	- Length of buffer data -> WREG.
; Regs Used:	FSR1
;***************************************************************
BUFLEN:
	movlw	0x01		; Putter is at Buffer[1].
	movf	PLUSW1, W	; Putter -> WREG
	sublw	0x02		; Subtract 2 to get the size of the data
	return				; (The putter and getter don't count towards size.)


;***************************************************************
; SUBROUTINES: 	BUFSEL_xxx
; 
; Description:	Select buffer xxx
; Precond'ns:	- 
; Postcond'ns:	- Address of buffer has been loaded to FSR1.
; Regs Used: 	FSR1
;***************************************************************
BUFSEL_RXA:	; Select Receive Buffer "A"
	lfsr	1, buf0		; Select buffer 0.
	btfsc	rxbuf_sw	; Have buffers A and B been switched?
	lfsr	1, buf1		; YES - Select buffer 1 instead.
	return

BUFSEL_RXB:	; Select Receive Buffer "B"
	lfsr	1, buf1		; Select buffer 1.
	btfsc	rxbuf_sw	; Have buffers A and B been switched?
	lfsr	1, buf0		; YES - Select buffer 0 instead.
	return

BUFSEL_TXA:	; Select Transmit Buffer "A"
	lfsr	1, buf2		; Select buffer 2.
	btfsc	txbuf_sw	; Have buffers A and B been switched?
	lfsr	1, buf3		; YES - Select buffer 3 instead.
	return

BUFSEL_TXB:	; Select Transmit Buffer "B"
	lfsr	1, buf3		; Select buffer 3.
	btfsc	txbuf_sw	; Have buffers A and B been switched?
	lfsr	1, buf2		; YES - Select buffer 2 instead.
	return

 end
