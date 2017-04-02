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

;*************************************************************************
; CONFIGURATION SECTION

; How many buffers do you want?
 #define NUM_BUFS	d'02'
 

; Lengths of the buffers
BUF0_LEN	equ d'64'
BUF1_LEN	equ d'64'

 
;*************************************************************************
; CONSTANTS

; Status Flag Positions
BUF_FULL	equ 0	; Buffer Full
BUF_NE		equ 1	; Buffer Not Empty
BUF_RDEOF	equ 2	; Buffer Read End Of Data
BUF_GTEOF	equ 3	; Buffer Get End Of Data
BUF_LOCK	equ 4	; Buffer Locked

;*************************************************************************
; VARIABLES
 udata

; temp storage 
temp		res 1

; Buffer status/control registers
; These two pointers indicate which part of the buffer is locked for procesing.
BUFLKS0		res 1	; Buffer 0 Locked Start Position
BUFLKE0		res 1	; Buffer 0 Locked End Position
BUFPUT0		res 1	; Buffer 0 Putting Pointer - writes to unlocked section.
BUFGET0		res 1	; Buffer 0 Getting Pointer - reads from locked section.
BUFREAD0	res 1	; Buffer 0 Reading Pointer - reads from unlocked section.
BUFSTAT0	res 1	; Buffer 0 Status Register
 global BUFSTAT0

BUFPUT1		res 1	; Buffer 1 Putting Pointer
BUFGET1		res 1	; Buffer 1 Getting Pointer
BUFSTAT1	res 1	; Buffer 1 Status Register
 global BUFSTAT1
 
; Put all buffer storage in a high bank, as it will never be addressed directly
.BUFFERS UDATA 0x400 
BUF0		res BUF0_LEN	; Buffer 0 - Receive Buffer
BUF1		res BUF1_LEN	; Buffer 1 - Transmit Buffer

;*************************************************************************
; START OF CODE

 CODE


;***************************************************************
; MACRO: 		INCP <f>, <base>, <ptr>, <len>
; 
; Description:	Load and increment a circular pointer.
;				base + ptr -> FSRf, (ptr + 1) % len -> ptr
;
; Arguments:	f - FSR register to use.
;				base - base address (12 bit literal)
;				ptr - pointer offset (8 bit file register)
;				len - length of buffer (8 bit literal)
; Precond'ns:	-
; Postcond'ns:	-
; Regs Used:	WREG, FSRf
;***************************************************************
ldincp macro f, base, ptr, len

	; Load the address and pointer offset.
	; k + p -> FSRf
	lfsr	f, base		; Load the base address.
	movf	ptr, w		; Add the offset.
	addwf	FSR#v(f)L	; ...
	;clrf	0x00		; ...
	;addwfc	FSR#v(f)H	; ...

	; Increment the circular pointer.
	; (p + 1) % length -> p
	incf	ptr		; ptr + 1 -> p
	movlw	len		; len -> WREG
	cpfslt	ptr		; Is the ptr < len ?
	clrf	ptr		; NO: 0 -> ptr

 endm

;***************************************************************
; Macro:		PUT <n>
; 
; Description:	Puts the contents of WREG into BUFFER n.
; Precond'ns:	Data to be PUT must be loaded to WREG.
;				User must check BUF_FULL before calling.
; Postcond'ns:	BUF_FULL will be set if the buffer is full.
; Regs Used:	WREG, FSR0, BUFFER n registers.
;***************************************************************
PUT MACRO n
	
	; Save the data byte.
	movwf	temp						

	; Load and increment the circular pointer.
	ldincp	0, BUF#v(n), BUFPUT#v(n), BUF#v(n)_LEN

	; Put the data, and set the BUF_NE flag.
	movff	temp, INDF0				; temp -> *FSR0
	bsf		BUFSTAT#v(n), BUF_NE	; Buffer is not empty.

	; Check if the buffer is full, then return.
	movlw	BUFLKS1#v(n)			; Load the start pos of the locked section.
	cpfseq	BUFPUT#v(n)				; Is BUFPUTn = BUFLKSn?
	return							; NO - return.
	bsf		BUFSTAT#v(n), BUF_FULL	; YES - Set BUF_FULL flag, return.
	return
	
 endm
	
;***************************************************************
; Macro:		GET <n>
; 
; Description:	Gets a byte from the locked section of BUFFER n
;				and puts it into WREG.
; Precond'ns:	Must check that BUF_NE is set and BUF_EOF is not
; 				set before calling.
; Postcond'ns:	BUF_GTEOF will be set if the end of the buffer is reached.
; Regs Used:	WREG, FSR0, BUFFER n registers.
;***************************************************************
GET MACRO n
	
	; Load and increment the circular pointer.
	ldincp	0, BUF#v(n), BUFGET#v(n), BUF#v(n)_LEN

	; Check if we have reached the end of the locked section
	movf	BUFLKE#v(n), w			; Load the end pos of the locked section.
	cpfseq	BUFGET#v(n)				; Is BUFGETn == BUFLKEn ?
	bra		_GETDATA#v(n)			; NO - just get the data.
	bsf		BUFSTAT#v(n), BUF_GTEOF	; YES - set EOF Flag.

_GETDATA#v(n):
	; Get the data and return.
	movf	INDF0, W				; *FSR0 -> WREG
	return

 endm

;***************************************************************
; Macro:		READ <n>
; 
; Description:	Gets a byte from the unlocked section of BUFFER n
;				and puts it into WREG.
; Precond'ns:	Must check that BUF_NE is set and BUF_EOF is not
; 				set before calling.
; Postcond'ns:	BUF_EOF will be set if the end of the buffer is reached.
; Regs Used:	WREG, FSR0, BUFFER n registers.
;***************************************************************
GET MACRO n
	
	; Load and increment the circular pointer.
	ldincp	0, BUF#v(n), BUFGET#v(n), BUF#v(n)_LEN

	; Check if we have reached the end of the locked section
	movf	BUFLKE#v(n), w			; Load the end pos of the locked section.
	cpfseq	BUFGET#v(n)				; Is BUFGETn == BUFLKEn ?
	bra		_GETDATA#v(n)			; NO - just get the data.
	bsf		BUFSTAT#v(n), BUF_GTEOF	; YES - set EOF Flag.

_GETDATA#v(n):
	; Get the data and return.
	movf	INDF0, W				; *FSR0 -> WREG
	return

 endm

;***************************************************************
; Macro:		RST <n>
; 
; Description:	Resets the getting pointer of BUFFER n.
;				Clears the BUF_EOF flag.
;
; Regs Used:	WREG, FSR0, BUFFER n registers.
;***************************************************************
RST	MACRO n
	clrf	BUFGET#v(n)				; Reset get pointer.
	bcf		BUFSTAT#v(n), BUF_EOF	; Clear EOF flag.
	return

 endm

;***************************************************************
; Macro:		CLR <n>
; 
; Description:	Clears BUFFER n.
;
; Regs Used:	WREG, FSR0, BUFFER n registers.
;***************************************************************
CLR MACRO n
	; Clear pointers and status flags
	clrf	BUFPUT#v(n)				; Reset put pointer.
	clrf	BUFGET#v(n)				; Reset get pointer.
	clrf	BUFSTAT#v(n)			; Reset flags.
	return

 endm

;****************************************************************
; Here we define subroutines to call the macros for each buffer.

PUT0: PUT 0
GET0: GET 0
CLR0: CLR 0
RST0: RST 0
 global PUT0, GET0, CLR0, RST0

PUT1: PUT 1
GET1: GET 1
CLR1: CLR 1
RST1: RST 1
 global PUT1, GET1, CLR1, RST1



 end
