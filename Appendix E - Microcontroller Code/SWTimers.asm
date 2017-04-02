;*************************************************************************
; SOFTWARE TIMERS MODULE
; 
; Version x.xx
; dd/mm/yyyy
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************
;
; Description:
; - This module provides one-shot timers which may be used for timeouts.
; - Each timer consists of a 8/16-bit counter, and a boolean flag indicating
;   whether the timer is running.
; - To start the timer, load the timeout into the counter, and set the timer flag.
; - Each time a TIMER2 interrupt occurs, the counter will be decremented if the
;   timer is running. When the counter reaches zero, the flag will be cleared and the
;   timer will stop.
;
; Dependencies:
;
; Resources Used:
;  - TIMER2 (Exclusively)
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

; The timebase for all software timers, as a multiple of 4us.
TIMEBASE equ d'250'             ; Timebase = 250 x 4us = 1ms

;*************************************************************************
; CONSTANTS

;******************************************************************************
; IMPORTED VARIABLES

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES

TMRL4   res 1                   ; Timer4 Low Register.
TMRH4   res 1                   ; Timer4 High Register.
TMRF4   res 1                   ; Timer4 Running.
 global TMRL4, TMRH4, TMRF4

TMRL5   res 1                   ; Timer5 Low Register.
TMRH5   res 1                   ; Timer5 High Register.
TMRF5   res 1                   ; Timer5 Running.
 global TMRL5, TMRH5, TMRF5
 

;******************************************************************************
; LOCAL VARIABLES

;******************************************************************************
; IMPORTED SUBROUTINES

;******************************************************************************
; EXPORTED SUBROUTINES

 global SWTIMERS_INIT, SWTIMERS_ISR

;******************************************************************************
; START OF CODE

 CODE
 

;***************************************************************
; SUBROUTINE:   SOFTWARE_TIMERS_INIT
; 
; Description:  Initialises TIMER2 as the timebase for the software timers.
; Precond'ns:   None.
; Postcond'ns:  TIMER2 is configured.
; Regs Used:    TIMER2 regs, WREG.
;***************************************************************

SWTIMERS_INIT:
    
    bcf     PIR1, TMR2IE        ; Disable the interrupt.
    movlw   TIMEBASE            ; Set TMR2 period
    movwf   PR2                 ; ...
    movlw   b'00000101'         ; TIMER2 Config byte.
    ;        '-0000---'         ; 0:0 Postscale
    ;        '-----1--'         ; Timer2 ON
    ;        '------01'         ; 1:4 Prescale
    movwf   T2CON               ; ...
    bcf     IPR1, TMR2IP        ; Set TIMER2 to low priority interrupt.
    bcf     PIR1, TMR2IF        ; Clear the flag so it doesn't irp immediately.
    bsf     PIE1, TMR2IE        ; Enable the interrupt.
    return



;***************************************************************
; MACRO:        TIMER_ISR
; 
; Description:  Decrements the timer counter. If the timer and period 
;               registers are equal, the interrupt flag is set and the
;               timer registers are cleared.
; Arguments:    n - the timer number.
; Precond'ns:   
; Postcond'ns:  Counter is decremented if the timer is running.
; Regs Used:    WREG
;***************************************************************
TIMER_ISR macro n

    btfss   TMRF#v(n), 0        ; Is The timer Running??
    bra     _END_#v(n)          ; NO - nothing to do.

    ; Decrement the counter.
    ; Note: after doing decf, STATUS<C> is set unless the register underflowed.
    decf    TMRL#v(n)           ; Decrement the counter.
    btfss   STATUS, C           ; Did the register underflow? (0x00 -> 0xFF)
    decf    TMRH#v(n)           ; YES - decrement the high byte.

    ; Test if the counter is zero.
    tstfsz  TMRL#v(n)           ; Is the low byte zero?
    bra     _END_#v(n)          ; NO - timer not expired yet.
    tstfsz  TMRH#v(n)           ; Is the high byte zero?
    bra     _END_#v(n)          ; NO - timer not expired yet.
    
    ; Clear the timer flag to say the timer has stopped.
    clrf    TMRF#v(n)
;   bra     _END_#v(n)

_END_#v(n):

 endm

;***************************************************************
; SUBROUTINE:   SWTIMERS_ISR
; 
; Description:  Runs the ISR's for all the timers.
; Precond'ns:   None.
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SWTIMERS_ISR:

    ; Check if TIMER2 caused the interrupt.
    SERVICE_IRP     PIR1, TMR2IF, PIE1, TMR2IE
    
    TIMER_ISR 4                 ; Run the TIMER4 ISR.
    TIMER_ISR 5                 ; Run the TIMER5 ISR.
    return

 end
