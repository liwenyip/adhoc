;*************************************************************************
; Physical Layer Header File
; Version 1.00
; 16/09/2005

; Initialisation Routine
 extern     PHY_INIT

; Manchester Encoding
 extern     MANCHESTER_ENCODER

; Manchester Decoding
 extern     MANCHESTER_DECODER, CLOCK_WATCHDOG

; Hardware Control
 extern     RX_DISABLE, TX_DISABLE, RX_ENABLE, TX_ENABLE

; Variables
 extern phy_status, txbyte, rxbyte, bitcount, shiftreg_l, shiftreg_h

 #define txbyte_empty   phy_status, 0 ; Set when a byte has been transmitted.
 #define rxbyte_full    phy_status, 1 ; Set when a byte has arrived.
 #define clock_detect   phy_status, 2 ; Set when there is a valid clock.
