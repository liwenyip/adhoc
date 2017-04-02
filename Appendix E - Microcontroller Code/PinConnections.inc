;*************************************************************************
; Pin Connections Header File
; 
; Version 1.00
; 14/08/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************

; To get to the TRIS Register, add 0x12.
 #define    TRIS        0x12
; To get to the LAT Register, add 0x09
 #define    LAT         0x09


; -----------------------------------------------------
; Transceiver Connections
; -----------------------------------------------------
; Inputs
 #define    RSSI        PORTA, AN1      ; Pin 3
 #define    NCD         PORTB, RB0      ; Pin 36
 #define    NCD2        PORTA, RA0      ; Pin 4
 #define    RXD         PORTC, CCP1     ; Pin 19
; Outputs
 #define    TXD         PORTC, CCP2     ; Pin 18
 #define    NTXEN       PORTB, RB5      ; Pin 41
 #define    NRXEN       PORTB, RB4      ; Pin 42
 #define    TXLED       PORTB, RB7      ; Pin 44
 #define    RXLED       PORTB, RB6      ; Pin 43

CONFIG_TRANSCIEVER_PINS macro
    ; Inputs
    bsf     TRIS + RSSI     
    bsf     TRIS + NCD      
    bsf     TRIS + NCD2     
    bsf     TRIS + RXD      
    bcf     TRIS + TXD      
    ; Set Outputs' Initial State
    bsf     LAT + NTXEN     ; High
    bsf     LAT + NRXEN     ; High
    bcf     LAT + TXLED     ; Low
    bcf     LAT + RXLED     ; Low
    ; Outputs
    bcf     TRIS + NTXEN    
    bcf     TRIS + NRXEN    
    bcf     TRIS + TXLED    
    bcf     TRIS + RXLED    
 endm   


; -----------------------------------------------------
; VGA Connections
; -----------------------------------------------------
; Inputs
 #define    B0          PORTD, RD0      ; Pin 21
 #define    B1          PORTD, RD1      ; Pin 22
 #define    B2          PORTD, RD2      ; Pin 23
 #define    B3          PORTD, RD3      ; Pin 24
 #define    PWUP        PORTD, RD4      ; Pin 30
 #define    DENB        PORTD, RD5      ; Pin 31

CONFIG_VGA_PINS macro
    ; Inputs
    bsf     TRIS + B0
    bsf     TRIS + B1
    bsf     TRIS + B2
    bsf     TRIS + B3
    bsf     TRIS + PWUP
    bsf     TRIS + DENB
 endm

; -----------------------------------------------------
; SPI Connections
; -----------------------------------------------------
CONFIG_SPI_PINS macro
    ; Inputs
    bsf     TRISC, SDI
    ; Outputs
    bcf     TRISC, SDO
    bcf     TRISC, SCK
 endm


; -----------------------------------------------------
; FRAM Connections
; -----------------------------------------------------
; Outputs
 #define    FRAM_NCS    PORTB, RB2      ; Pin 18
 #define    NWP         PORTD, RD6      ; Pin 32
 #define    NHOLD       PORTD, RD7      ; Pin 33

CONFIG_FRAM_PINS macro
    ; Set Outputs' Initial State
    bsf     LAT + FRAM_NCS  ; High
    bcf     LAT + NWP       ; Low
    bsf     LAT + NHOLD     ; High
    ; Outputs
    bcf     TRIS + FRAM_NCS
    bcf     TRIS + NWP  
    bcf     TRIS + NHOLD    
 endm   

; -----------------------------------------------------
; RTC Connections
; -----------------------------------------------------
; Outputs
 #define    RTC_CS      PORTB, RB3      ; Pin 16    

CONFIG_RTC_PINS macro
    ; Inputs
    bsf     TRISB, INT1     ; ALARM IRQ
    ; Set Outputs' Initial State
    bcf     LAT + RTC_CS    ; Low
    ; Outputs
    bcf     TRIS + RTC_CS
 endm
