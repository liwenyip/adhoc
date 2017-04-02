;*************************************************************************
; Macro Library
; Version 1.00
; 16/09/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************

;*********************************************************************************
; Macro:        SERVICE_IRP
; 
; Description:  Service an interrupt.
;               MUST BE CALLED FROM WITHIN A SUBROUTINE.
;
; If the flag was set, it clears it.
; If the flag wasn't set, or the interrupt wasn't enabled, 
; it returns from the subroutine.
;
; Regs Used:    
;*********************************************************************************
SERVICE_IRP macro int_flag_reg, int_flag, int_en_reg, int_en_flag

; Check the interrupt enable flag.
    btfss   int_en_reg, int_en_flag     ; Was the interrupt enabled?
    return                              ; NO - exit the routine.

; Check the interrupt flag.
    btfss   int_flag_reg, int_flag      ; Did this device's interrupt event occur?
    return                              ; NO - return.
    bcf     int_flag_reg, int_flag      ; YES - clear the interrupt flag.


 endm

;*********************************************************************************
; Macro:        ADDLF16
; 
; Description:  Add a 16 bit literal to a 16 bit field
;
; Regs Used:    WREG
;*********************************************************************************
ADDLF16 macro   k, d
    movlw   low k       ; load up the low byte of the literal
    addwf   d           ; add it to the low byte of the destination
    movlw   high k      ; load up the high byte of the literal
    addwfc  d + 1       ; add it and the carry bit to the high byte of the destination
 endm

;*********************************************************************************
; Macro:        SUBLF16
; 
; Description:  Subtract a 16 bit literal from a 16 bit field
;
; Regs Used:    WREG
;*********************************************************************************
SUBLF16 macro   k, d
    bsf     STATUS, C   ; Make sure the low byte subtraction doesnt use the borrow  bit
    movlw   low k       ; load up the low byte of the literal
    subwf   d, f        ; subtract it from the low byte of the destination
    movlw   high k      ; load up the high byte of the literal
    subwfb  d + 1, f    ; subtract it and the borrow bit from the high byte of the dest
 endm

