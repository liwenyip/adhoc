;*************************************************************************
; RTC MODULE
; 
; Version 0.10
; 17/07/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************
;
; Description: 
;   Provides routines for accessing the Maxim-Dallas DS1305 Real Time Clock Chip..
;
; Functions:

;
; Dependencies:
;   SPI.asm
;   Buffers.inc
;
; Resources Used:
;   - MSSP Module.
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

; Internal Addresses
CLOCK       equ 0x00    ; Address of the Clock Registers
ALARM0      equ 0x07    ; Address of Alarm 0 Registers
ALARM1      equ 0x0B    ; Address of Alarm 1 Registers
CONTROL     equ 0x0F    ; Control Register
RTCSTATUS   equ 0x10    ; Status Register
CHARGER     equ 0x11    ; Trickle Charger Register
USERRAM     equ 0x20    ; User RAM 0x20 - 0x7F

; Offsets
SECONDS     equ 0x00    ; Seconds Offset
MINUTES     equ 0x01    ; Clock Minutes Offset
HOURS       equ 0x02    ; Clock Hours Offset
DAY         equ 0x03    ; Clock Day Offset
DATE        equ 0x04    ; Clock Date Offset
MONTH       equ 0x05    ; Clock Month Offset
YEAR        equ 0x06    ; Clock Year Offset

WRITE       equ 0x80    ; Write offset



;******************************************************************************
; IMPORTED VARIABLES

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES

;******************************************************************************
; LOCAL VARIABLES

;******************************************************************************
; IMPORTED SUBROUTINES
 
 #include "Buffers.inc"

; From <SPI.asm>
 extern SPI_INIT, SPIRW

;******************************************************************************
; EXPORTED SUBROUTINES

 global RTC_INIT
 global RTC_CLEAR_IRQS, RTC_GET_STATUS, RTC_GET_CLOCK, RTC_SET_CLOCK

;******************************************************************************
; START OF CODE

 CODE

;***************************************************************
; Subroutine:   RTC_INIT
; 
; Description:  Initialises the Real Time Clock.
; Precond'ns:   SPI bus has been initialised.
; Postcond'ns:  
; Regs Used:    WREG
;***************************************************************
RTC_INIT:
    
    ; Set up RTC Pins
    CONFIG_RTC_PINS

    ; Set up INT1
    bcf     INTCON2, INTEDG1    ; Trigger on falling edge.
    bcf     INTCON3, INT1IP     ; Low Priority
    bcf     INTCON3, INT1IF     ; Clear the IRQ flag
    bsf     INTCON3, INT1IE     ; Enable the Interrupt

    ; Enable Writes
    bsf     RTC_CS              ; Start an operation.
    movlw   CONTROL + WRITE     ; Load the CONTROL register write address
    call    SPIRW               ; ...
    movlw   b'00000000'         ; Load a byte to enable writes.
    ;        '-0------'         ; Disable Write Protect.
    call    SPIRW               ; ...
    bcf     RTC_CS              ; End the operation.
    nop
    

    ; Set up the control register
    bsf     RTC_CS              ; Start an operation.
    movlw   CONTROL + WRITE     ; Load the CONTROL register write address
    call    SPIRW               ; ...
    movlw   b'00000011'
    ;        '0-------'         ; Enable the oscillator.
    ;        '-0------'         ; Disable Write Protect.
    ;        '-----0--'         ; Both Alarm 0 and Alarm 1 activate /INT0.
    ;        '------1-'         ; Enable Alarm 1 Interrupt.
    ;        '-------1'         ; Enable Alarm 0 Interrupt.
    call    SPIRW               ; ...
    bcf     RTC_CS              ; End the operation.
    nop

    ; Set up Alarm 0 to interrupt when seconds = 00
    bsf     RTC_CS              ; Start an operation.
    movlw   ALARM0 + WRITE      ; Load the Alarm 0 write address
    call    SPIRW               ; ...
    movlw   0x00                ; Load 00 seconds (BCD)
    call    SPIRW               ; Write to the seconds register
    movlw   b'10000000'         ; Set the bit mask
    call    SPIRW               ; Write to the minutes register
    movlw   b'10000000'         ; Set the bit mask
    call    SPIRW               ; Write to the hours register
    movlw   b'10000000'         ; Set the bit mask
    call    SPIRW               ; Write to the days register
    bcf     RTC_CS              ; End the operation.
    nop

    ; Set up Alarm 1 to interrupt when seconds = 30
    bsf     RTC_CS              ; Start an operation.
    movlw   ALARM1 + WRITE      ; Load the Alarm 1 write address
    call    SPIRW               ; ...
    movlw   0x30                ; Load 30 seconds (BCD)
    call    SPIRW               ; Write to the seconds register
    movlw   b'10000000'         ; Set the bit mask
    call    SPIRW               ; Write to the minutes register
    movlw   b'10000000'         ; Set the bit mask
    call    SPIRW               ; Write to the hours register
    movlw   b'10000000'         ; Set the bit mask
    call    SPIRW               ; Write to the days register
    bcf     RTC_CS              ; End the operation.
    nop

;   return

; DEBUGGING CODE...
;
;_READCLOCK:
;    ; Do some reads
;    bsf     RTC_CS              ; Start an operation.
;    movlw   CLOCK               ; Load the clock read address
;    call    SPIRW               ; ...
;    call    SPIRW               ; Read Seconds. (CONTROL)
;    call    SPIRW               ; Read Minutes. (STATUS)
;    call    SPIRW               ; Read Hours.
;    call    SPIRW               ; Read Day.
;    call    SPIRW               ; Read Date.
;    call    SPIRW               ; Read Month.
;    call    SPIRW               ; Read Year.
;    bcf     RTC_CS              ; End the operation.
;    nop
;
;    return


;***************************************************************
; Subroutine:   CRTC_CLEAR_IRQS
; 
; Description:  Clears both IRQ flags in the STATUS register by reading
;               from the DAY register of ALARM0 and the SECONDS register of ALARM1
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    WREG
;***************************************************************
RTC_CLEAR_IRQS:

    ; Read 0x0A (ALARM0-DAY) and 0x0B (ALARM1-SECONDS).
    bsf     RTC_CS              ; Start an operation.
    movlw   ALARM0 + DAY        ; Load the Alarm 0 DAY read address
    call    SPIRW               ; ...
    call    SPIRW               ; Read ALARM0-DAY
    call    SPIRW               ; Read ALARM1-SECONDS
    bcf     RTC_CS              ; End the operation.
    nop
    
    return


;***************************************************************
; Subroutine:   RTC_GET_STATUS
; 
; Description:  Get the status register.
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    WREG
;***************************************************************
RTC_GET_STATUS:
    bsf     RTC_CS              ; Start an operation.
    movlw   RTCSTATUS           ; Load the STATUS register read address
    call    SPIRW               ; ...
    call    SPIRW               ; Read one byte.
    bcf     RTC_CS              ; End the operation.
    return

;***************************************************************
; Subroutine:   RTC_GET_CLOCK
; 
; Description:  Read the clock into the currently selected buffer.
; Precond'ns:   There are at least 7 free bytes after the current cursor location.
; Postcond'ns:  
; Regs Used:    WREG
;***************************************************************
RTC_GET_CLOCK:
    bsf     RTC_CS              ; Start an operation.
    movlw   CLOCK               ; Load the CLOCK register read address
    call    SPIRW               ; ...

    call    SPIRW               ; Read Seconds and place it in the buffer.
    addlw   d'01'               ; Compensate for the 1 second delay

        BUF_PUT                     
    call    SPIRW               ; Read Minutes and place it in the buffer.
    BUF_PUT
    call    SPIRW               ; Read Hours and place it in the buffer.
    BUF_PUT
    call    SPIRW               ; Read Day and place it in the buffer.
    BUF_PUT
    call    SPIRW               ; Read Date and place it in the buffer.
    BUF_PUT
    call    SPIRW               ; Read Month and place it in the buffer.
    BUF_PUT
    call    SPIRW               ; Read Year and place it in the buffer.
    BUF_PUT
    BUF_MARKEND                 ; Mark the end of the buffer.

    bcf     RTC_CS              ; End the operation.
    return

;***************************************************************
; Subroutine:   RTC_SET_CLOCK
; 
; Description:  Set the clock from the currently selected buffer.
; Precond'ns:   The buffer contains 7 clock bytes at the current cursor location.
; Postcond'ns:  
; Regs Used:    WREG
;***************************************************************
RTC_SET_CLOCK:
    bsf     RTC_CS              ; Start an operation.
    movlw   CLOCK + WRITE       ; Load the CLOCK register write address
    call    SPIRW               ; ...

    BUF_GET                     ; Get the Seconds byte and write it to the RTC chip.
    call    SPIRW               
    BUF_GET                     ; Get the Minutes byte and write it to the RTC chip.
    call    SPIRW               
    BUF_GET                     ; Get the Hours byte and write it to the RTC chip.
    call    SPIRW               
    BUF_GET                     ; Get the Day byte and write it to the RTC chip.
    call    SPIRW               
    BUF_GET                     ; Get the Date byte and write it to the RTC chip.
    call    SPIRW               
    BUF_GET                     ; Get the Month byte and write it to the RTC chip.
    call    SPIRW               
    BUF_GET                     ; Get the Year byte and write it to the RTC chip.
    call    SPIRW               

    bcf     RTC_CS              ; End the operation.
    return

 END
