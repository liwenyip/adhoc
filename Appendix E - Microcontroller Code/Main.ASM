;******************************************************************************
;   This file is a basic template for creating relocatable assembly code for  *
;   a PIC18C452. Copy this file into your project directory and modify or     *
;   add to it as needed. Create a project with MPLINK as the language tool    *
;   for the hex file. Add this file and the 18C452.LKR file to the project.   *
;                                                                             *
;   The PIC18CXXX architecture allows two interrupt configurations. This      *
;   template code is written for priority interrupt levels and the IPEN bit   *
;   in the RCON register must be set to enable priority levels. If IPEN is    *
;   left in its default zero state, only the interrupt vector at 0x008 will   *
;   be used and the WREG_TEMP, BSR_TEMP and STATUS_TEMP variables will not    *
;   be needed.                                                                *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on the         *
;   features of the assembler and linker.                                     *
;                                                                             *
;   Refer to the PIC18CXX2 Data Sheet for additional information on the       *
;   architecture and instruction set.                                         *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:      Main.asm                                                  *
;    Date:          25/09/2005                                                *
;    File Version:  1.00                                                      *
;                                                                             *
;    Author:        Li-Wen Yip                                                *
;    Company:       James Cook University                                     *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files required:         P18C452.INC                                      *
;                            18C452.LKR                                       *
;                                                                             *
;******************************************************************************
; MASTER HEADER FILE
 #include "MasterHeader.inc"


;********************************************************************************
; EXTERNAL LABELS

 #include "PHY.inc"
 #include "MAC.inc"

 #include "RTC.inc"
 #include "buffers.inc"     ; Labels for buffers.asm

; From <SoftwareTimers.asm>
 extern SWTIMERS_INIT, SWTIMERS_ISR

; From <SPI.asm>
 extern SPI_INIT

; From <RouteCache.asm>
 extern FRAM_INIT

;******************************************************************************
;Configuration bits
; The __CONFIG directive defines configuration data within the .ASM file.
; The labels following the directive are defined in the P18C452.INC file.
; The PIC18CXX2 Data Sheet explains the functions of the configuration bits.
; Change the following lines to suit your application.

;   __CONFIG    _CONFIG0, _CP_OFF_0
;   __CONFIG    _CONFIG1, _OSCS_OFF_1 & _RCIO_OSC_1
;   __CONFIG    _CONFIG2, _BOR_ON_2 & _BORV_25_2 & _PWRT_OFF_2
;   __CONFIG    _CONFIG3, _WDT_ON_3 & _WDTPS_128_3
;   __CONFIG    _CONFIG5, _CCP2MX_ON_5
;   __CONFIG    _CONFIG6, _STVR_ON_6

;******************************************************************************
;Variable definitions
; These variables are only needed if low priority interrupts are used. 
; More variables may be needed to store other special function registers used
; in the interrupt routines.

MAIN_UDATA  UDATA

WREG_TEMP   RES 1   ;variable in RAM for context saving 
STATUS_TEMP RES 1   ;variable in RAM for context saving
BSR_TEMP    RES 1   ;variable in RAM for context saving


 UDATA 0x500
swstack     RES 64

;******************************************************************************
;Reset vector
; This code will start executing when a reset occurs.

RESET_VECTOR    CODE 0x0000

    goto    Main        ;go to start of main code

;******************************************************************************
;High priority interrupt vector
; This code will start executing when a high priority interrupt occurs or
; when any interrupt occurs if interrupt priorities are not enabled.

HI_INT_VECTOR   CODE 0x0008

    bra     HighInt     ;go to high priority interrupt routine

;******************************************************************************
;Low priority interrupt vector
; This code will start executing when a low priority interrupt occurs.
; This code can be removed if low priority interrupts are not used.

LOW_INT_VECTOR  CODE 0x0018

    bra     LowInt      ;go to low priority interrupt routine



 CODE

;******************************************************************************
;High priority interrupt routine
; The high priority interrupt code is placed here.

HighInt:
    
; Check if CCP1 (Receive Clock) caused an interrupt.
_CCP1_ISR:
    btfss   PIR1, CCP1IF        ; Is the interrupt flag set?
    bra     _CCP2_ISR           ; NO - skip to checking CCP2.
    bcf     PIR1, CCP1IF        ; YES - clear it.

    btfss   PIE1, CCP1IE        ; Was the interrupt enabled?
    bra     _CCP2_ISR           ; NO - skip to checking CCP2.

    ; Let's do some decoding. Yeah Baby!
    call    MANCHESTER_DECODER  ; Call the manchester decoding routine. <PHY.asm>
    call    FRAME_DECODER       ; Call the Frame decoding routine. <MAC.asm>
    retfie  FAST                ; Done.

; Check if CCP2 (Transmit Clock / Receive Clock Watchdog) caused an interrupt.
_CCP2_ISR:
    btfss   PIR2, CCP2IF        ; Is the interrupt flag set?
    retfie  FAST                ; NO - return.
    bcf     PIR2, CCP2IF        ; YES - clear it.
    btfss   PIE2, CCP2IE        ; Was the interrupt enabled?
    retfie  FAST                ; NO - return.

    ; Check if we should run the encoder routine or the watchdog routine.
    btfss   NTXEN               ; Are we in transmit mode?
    bra     _RUN_ENCODER        ; YES

    call    CLOCK_WATCHDOG      ; NO - run the watchdog routine. <PHY.asm>
    call    RESET_DECODER       ; Notify the MAC layer clock has been lost. <MAC.asm>
    retfie  FAST                ; Done.

_RUN_ENCODER:
    ; Let's do some encoding.   
    call    MANCHESTER_ENCODER  ; Call the manchester encoding routine. <PHY.asm>
    call    FRAME_ENCODER       ; Call the frame encoding routine. <MAC.asm>
    retfie  FAST                ; Done.

;******************************************************************************
;Low priority interrupt routine
; The low priority interrupt code is placed here.
; This code can be removed if low priority interrupts are not used.

LowInt:
    movff   STATUS, PREINC2     ; Save STATUS register
    movwf   PREINC2             ; Save WREG register
    movff   BSR, PREINC2        ; Save BSR register

    call    SWTIMERS_ISR        ; <SWTimers.asm>
    call    TMR0_ISR            ; <MAC.asm>
    call    TRAFFIC_CONTROL     ; <MAC.asm>
    call    INT1_ISR            ; <MAC.asm>

    movff   POSTDEC2, BSR       ; Restore BSR register
    movf    POSTDEC2, W         ; Restore WREG register
    movff   POSTDEC2, STATUS    ; Restore STATUS register

    retfie

;******************************************************************************
;Start of main program
; The main program code is placed here.

Main:

    ; Disable interrupts during initialisation
    bcf     INTCON, GIEL        ; disable low priority interrupt.
    bcf     INTCON, GIEH        ; disable high priority interrupt.

    ; Initialisation Routines
    lfsr    2, 0x500            ; Initialise the stack
    call    SWTIMERS_INIT       ; Initialise software timers
    call    PHY_INIT            ; Initialise physical layer
    call    MAC_INIT            ; Initialise mac layer
    call    SPI_INIT            ; Initialise SPI Bus
    call    FRAM_INIT           ; Initialise FRAM chip
    call    RTC_INIT            ; Initialise RTC chip

    ; Reenable Interrupts
    bsf     RCON, IPEN          ; enable interrupt priorities
    bsf     INTCON, GIEL        ; enable low priority interrupt.
    bsf     INTCON, GIEH        ; enable high priority interrupt.


;   call    RX_ENABLE
    
;   bcf     INTCON3, INT1IE

    extern  MAC_TEST
    bra     MAC_TEST


Loopy:
    clrwdt
    bra Loopy

;******************************************************************************
;End of program

    reset   ; If we ever get here do a reset.

 END
