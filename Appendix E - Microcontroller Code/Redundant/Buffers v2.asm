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
BUF_EOF		equ 2	; Buffer End Of Data
BUF_LOCK	equ 3	; Buffer Locked

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
.BUFFERS UDATA 0x500 

; Buffer 0 - Receive Buffer
; 0x500 - 0x57F (128 bytes)
; = 101 0xxx xxxx in binary
; Use FSR0 for exclusive access to locked section
BUF0		res	0x80

; Buffer 1 - Transmit Buffer
; 0x580 - 0x5FF (128 bytes)
; = 101 1xxx xxxx in binary
; Use FSR1 for exclusive access to locked section
BUF1		res 0x80

;*************************************************************************
; START OF CODE

 CODE

;***************************************************************
; MACRO: 		ldaddr <f>, <baseaddr>, <offset>
; 
; Description:	Load an address and offset to FSRf
;				baseaddr + offset -> FSRf
;
; Arguments:	f - FSR register to use.
;				baseaddr - base address (literal)
;				offset - pointer offset (file)
; Precond'ns:	-
; Postcond'ns:	-
; Regs Used:	WREG, FSRf
;***************************************************************
LDADDR macro f, baseaddr, offset

	; Load the address and pointer offset.
	; k + p -> FSRf
	lfsr	f, baseaddr	; Load the base address.

 if offset != 0
	movf	offset, w	; Add the offset.
	addwf	FSR#v(f)L	; ...
	clrf	0x00		; ...
	addwfc	FSR#v(f)H	; ...
 endif

 endm


;***************************************************************
; MACRO: 		GET <n>, <dest>, <f>, <endaddr>
; 
; Description:	Get a byte from BUFn, put it in dest.
; Arguments:	n - the buffer to use.
;				dest - where to put the byte.
;				f - the FSR register to use.
;				endaddr - the address we want to test if we are at.
; Precond'ns:	
; Postcond'ns:	STATUS<Z> is set if we have reached the end.
; Regs Used:	WREG, FSRf
;***************************************************************
GET macro n, dest, f, endaddr
	
	; Get the byte.
	movff	POSTINC0, dest	; *FSR0++ -> dest

	; Mask the low byte of FSR0 to make sure we stay 
	; in the address range of BUFn.
 if n = 0
	bcf		FSR0L, 7
 else
	bsf		FSR0L, 7
 endif

	; Check if we have reached the end of the section
	movf	FSR0L, w		; Get the current address.
	subwf	endaddr, w		; Does current address = end address?

	; Test STATUS<z> after the macro is called.
	; If Z = 1, we have reached the end.

 endm





 END
