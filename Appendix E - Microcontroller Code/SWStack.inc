;*************************************************************************
; Software Stack
; 
; Version 0.10
; 11/08/2005
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

;***************************************************************
; MACRO:    SWPUSH
; 
; Description:  Push a file to the software stack.
; Arguments:    file - The file to be pushed to the stack.
; Regs Used:    FSR2 (Exclusive)
;***************************************************************
SWPUSH macro file
;   if file == WREG
;       movwf   PREINC2
;   else
        movff   file, PREINC2
;   endif
    endm

;***************************************************************
; MACRO:    SWPOP
; 
; Description:  Pop a file from the software stack.
; Arguments:    f - The file to be popped to.
; Regs Used:    FSR2 (Exclusive)
;***************************************************************
SWPOP macro file
;   if file == WREG
;       movf    POSTDEC2, W
;   else
        movff   POSTDEC2, file
;   endif
    endm

;***************************************************************
; MACRO:    STACKINIT
; 
; Description:  Initialise the software stack
; Arguments:    The address of the stack
; Regs Used:    FSR2 (Exclusive)
;***************************************************************
STACKINIT macro addr
    lfsr 2, addr
    endm
