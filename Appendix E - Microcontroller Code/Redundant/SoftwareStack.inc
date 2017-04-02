
SWPUSH macro file,len
	variable i
i = len - 1
	while i >= 0
	movff	file + i, POSTINC2
i -= 1
	endw
	endm

;***************************************************************
; MACRO: 	SWPOP
; 
; Description:	Pop n values from the software stack.
; Arguments:	f - The first file to be popped to.
;				n - the number of values to pop.
; Regs Used:	FSR2 (Exclusive)
;***************************************************************
;SWPOP macro file, len
; 	variable i
;i = 0
;	while i < len
;	movff PREDEC2, file + i
;i += 1
;	endw
;	endm

;***************************************************************
; MACRO: 	STACKINIT
; 
; Description:	Initialise the software stack
; Arguments:	The address of the stack
; Regs Used:	FSR2 (Exclusive)
;***************************************************************
STACKINIT macro addr
	lfsr 2, addr
 	endm
