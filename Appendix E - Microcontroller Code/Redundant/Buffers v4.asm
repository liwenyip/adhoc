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
 #define	buf_empty	buf_flags, 2
 #define	rxbuf_sw	buf_flags, 4
 #define	txbuf_sw	buf_flags, 5
 #define	rxbuf_lock	buf_flags, 6
 #define	txbuf_lock	buf_flags, 7

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

global buf_flags

;*************************************************************************
; START OF CODE

 CODE

;***************************************************************
; SUBROUTINE: 	APPEND_HEAD
; 
; Description:	Append a byte to the head (start) of the buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR0.
;				- Address of head & tail pointers has been loaded to FSR1.
;				- Length of buffer has been loaded to buflen.
; Postcond'ns:	- Contents of WREG are appended to head of buffer.
; Regs Affected: WREG
;***************************************************************
; FSR1 + 0 = Head Pointer, FSR1 + 1 - Tail Pointer
APPEND_HEAD:
	
	; Stack the W Register.
	SWPUSH	WREG	

	; Expand the buffer on the head end.
	tstfsz	INDF1			; Are we at index 0?
	movff	buflen, INDF1	; YES - wrap round to Buffer[buflen]
	decf	INDF1			; Decrement the head pointer.

	; Check if the buffer has reached its maximum size.
	bsf		buf_full		; Set the buffer full flag.
	movf	POSTINC1, w		; Load the head pointer.
	cpfseq	POSTDEC1		; head pointer == tail pointer?
	bcf		buf_full		; NO - Clear the buffer full flag.
		
	; Place the byte at the head of the buffer.
	; Head pointer is already loaded to WREG.
	SWPOP	PLUSW0			; Pop a byte from the stack to the buffer.

	; Done.
	return

;***************************************************************
; SUBROUTINE: 	APPEND_TAIL
; 
; Description:	Append a byte to the tail (end) of the buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR0.
;				- Address of head & tail pointers has been loaded to FSR1.
;				- Length of buffer has been loaded to buflen.
; Postcond'ns:	- Contents of WREG are appended to tail of buffer.
; Regs Affected: WREG
;***************************************************************
; FSR1 + 0 = Head Pointer, FSR1 + 1 - Tail Pointer
APPEND_TAIL:

	; Stack the W Register.
	SWPUSH	WREG	

	; Place byte at the tail of the buffer
	movf	PREINC1, W		; Load the tail pointer.
	SWPOP	PLUSW0			; Pop a byte from the stack to the buffer.
		
	; Expand the buffer on the tail end.
	; Tail pointer is already loaded to WREG.
	incf	WREG			; Increment the tail pointer.
	cpfslt	buflen			; Is tail pointer >= buflen?
	clrf	WREG			; YES - set tail pointer = 0
	movwf	POSTDEC1		; Save the tail pointer.

	; Check if the buffer has reached its maximum size.
	; Tail pointer is already loaded to WREG.
	bsf		buf_full		; YES - Set the buffer full flag.
	cpfseq	INDF1			; head pointer == tail pointer?
	bcf		buf_full		; NO - Clear the buffer full flag.

	; Done.
	return

;***************************************************************
; SUBROUTINE: 	REMOVE_HEAD
; 
; Description:	Remove a byte from the head of the buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR0.
;				- Address of head & tail pointers has been loaded to FSR1.
;				- Length of buffer has been loaded to buflen.
; Postcond'ns:	- A byte has been removed from the head of buffer and placed in WREG.
; Regs Affected: WREG
;***************************************************************
; FSR1 + 0 = Head Pointer, FSR1 + 1 - Tail Pointer
REMOVE_HEAD:
	
	; Read a byte from the head and stack it.
	movf	INDF1, W		; Load the head pointer (Where's your head at?)
	SWPUSH	PLUSW0			; Read the head and stack it.

	; Shrink the buffer on the head end.
	incf	INDF1, W		; Load the head pointer and increment it.
	cpfslt	buflen			; Is head pointer >= buflen?
	clrf	WREG			; YES - set head pointer = 0
	movwf	POSTINC1		; Save the head pointer.

	; Check if the buffer is empty.
	; Head pointer is already loaded to WREG.
	bsf		buf_empty		; Set the buffer empty flag.
	cpfseq	POSTDEC1		; head pointer == tail pointer?
	bcf		buf_empty		; NO - Clear the buffer empty flag.
		
	; Put the result into the WREG.
	SWPOP	WREG			; Pop a byte from the stack to WREG.

	; Done.
	return

;***************************************************************
; SUBROUTINE: 	REMOVE_TAIL
; 
; Description:	Remove a byte from the tail of the buffer.
; Precond'ns:	- Address of buffer has been loaded to FSR0.
;				- Address of head & tail pointers has been loaded to FSR1.
;				- Length of buffer has been loaded to buflen.
; Postcond'ns:	- A byte has been removed from the tail of buffer and placed in WREG.
; Regs Affected: WREG
;***************************************************************
; FSR1 + 0 = Head Pointer, FSR1 + 1 - Tail Pointer
REMOVE_TAIL:
	
	; Shrink the buffer on the tail end.
	tstfsz	PREINC1			; Are we at index 0?
	movff	buflen, INDF1	; YES - wrap round to Buffer[buflen]
	decf	INDF1			; Decrement the tail pointer.

	; Read a byte from the tail and stack it.
	movf	POSTDEC1, W		; Load the tail pointer.
	SWPUSH	PLUSW0			; Read the tail and stack it.

	; Check if the buffer is empty.
	; Tail pointer is already loaded to WREG.
	bsf		buf_empty		; Set the buffer empty flag.
	cpfseq	INDF1			; tail pointer == head pointer?
	bcf		buf_empty		; NO - Clear the buffer empty flag.
		
	; Put the result into the WREG.
	SWPOP	WREG			; Pop a byte from the stack to WREG.

	; Done.
	return
