;*************************************************************************
; SPI MODULE
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
;   Provides routines for initialization and transfer on SPI.
;
; Functions:

;
; Dependencies:
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

;******************************************************************************
; IMPORTED VARIABLES

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES

;******************************************************************************
; LOCAL VARIABLES

;******************************************************************************
; IMPORTED SUBROUTINES

;******************************************************************************
; EXPORTED SUBROUTINES

 global SPI_INIT, SPIRW

;******************************************************************************
; START OF CODE

 CODE

;***************************************************************
; Subroutine:   SPI_INIT
; 
; Description:  Initialises the MSSP in SPI mode 3.
; Regs Used:    WREG, TRISC, SSP Registers.
;***************************************************************
SPI_INIT:
    ; Initialise the MSSP in SPI mode

    movlw   B'00100000'     
    ;        '--1-----'     ; Enables SPI and configure SCK, SDO, SDI, and SS
    ;        '---0----'     ; Clock idle low
    ;        '----0000'     ; SPI Master mode, clock = FOSC/4
    movwf   SSPCON1         ; ...

    clrf    SSPCON2         ; This register only used for I2C mode.

    movlw   b'11000000'
    ;        '1-------'     ; Input data sampled at end of data output time
    ;        '-1------'     ; Data transmitted on rising edge of SCK
    movwf   SSPSTAT 

    ; Configure SPI pins
    CONFIG_SPI_PINS

    return                  ; Done.

;***************************************************************
; Subroutine:   SPIRW
;
; Description:  Transfers 1 byte on the SPI bus.
; Precond'n:    WREG contains the data to be transmitted.
; Postcond'n:   WREG contains the data that was received.
; Regs Used:    WREG, SSPBUF
;***************************************************************
SPIRW:
    movwf   SSPBUF              ; Move data to be transmitted into SSPBUF.
_SPIRW_LOOP:
    btfss   SSPSTAT, BF         ; Check if transfer is complete.
    bra     _SPIRW_LOOP         ; NO - Loop.
    movf    SSPBUF, W           ; YES - Move the received data into WREG.
    return                      ; Done.


 end
