;*************************************************************************
; PHYSICAL LAYER MODULE
; 
; Version 1.00
; 16/09/2005
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
; Functions:
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
;*************************************************************************
; MASTER HEADER FILE
 #include "MasterHeader.inc"

;*************************************************************************
; CONFIGURATION CONSTANTS

; Chip Length: The number of periods of TMR3 that equals one chip.
 #define    CHIPTIME    d'100'      ; 100 us

; Power on Delay: The amount of time to wait between powering up the transmitter
; hardware and starting to transmit data. Defined in periods of TMR3.
 #define    TX_DELAY    d'5000'     ; 5 ms

;*************************************************************************
; CONSTANTS

; CCP Compare Modes:
CCP_IRP         equ b'1010'     ; Just generate an interrupt on match
CCP_RAISE       equ b'1000'     ; Raise the output on match
CCP_CLEAR       equ b'1001'     ; Clear the output on match
CCP_TOGGLE      equ b'0010'     ; Toggle the output on match - doesn't seem to work

; CCP Capture Modes:
CCP_FALLING     equ b'0100'     ; Capture every falling edge
CCP_RISING      equ b'0101'     ; Capture every rising edge
CCP_RISING4     equ b'0110'     ; Capture every 4th rising edge
CCP_RISING16    equ b'0111'     ; Capture every 16th rising edge

 #define rx_get_sample  CCP1CON, 3      ; Have we been waiting to get a sample?

;******************************************************************************
; GLOBAL VARIABLES
.phy_global_vars udata_acs

; Status register - used to communicate state information to and from the MAC layer.
phy_status      res 1
 #define txbyte_empty   phy_status, 0 ; Set when a byte has been transmitted.
 #define rxbyte_full    phy_status, 1 ; Set when a byte has arrived.
 #define clock_detect   phy_status, 2 ; Set when there is a valid clock.

; Tx/Rx Registers - data is exchanged with the physical layer via these registers.
txbyte          res 1
rxbyte          res 1

; Bit counter - used to synchronise to the start of a byte.
bitcount        res 1

; 16 bit shift register - directly accessed by the MAC layer to detect the SOF.
shiftreg_l      res 1
shiftreg_h      res 1

 global phy_status, txbyte, rxbyte, bitcount, shiftreg_l, shiftreg_h


;******************************************************************************
; LOCAL VARIABLES
.phy_local_vars udata_acs

phy_flags           res 1
 #define    tx_clock    phy_flags, 0    ; Set when we are encoding a clock chip.


;******************************************************************************
; IMPORTED SUBROUTINES


;******************************************************************************
; EXPORTED SUBROUTINES

; Initialisation Routine
 global     PHY_INIT

; Manchester Encoding
 global     MANCHESTER_ENCODER

; Manchester Decoding
 global     MANCHESTER_DECODER, CLOCK_WATCHDOG

; Harware Control
 global     RX_DISABLE, TX_DISABLE, RX_ENABLE, TX_ENABLE


;******************************************************************************
; START OF CODE

 CODE

;***************************************************************
; SUBROUTINE:   PHY_INIT
; 
; Description:  Initialisation for physical layer.
; Precond'ns:   -
; Postcond'ns:  
; Regs Used:    WREG, CCP1 Registers, CCP2 Registers, TMR3 Registers
;***************************************************************
PHY_INIT:

    ; Configure Transceiver pins.
    CONFIG_TRANSCIEVER_PINS

    ; Configure TIMER3 (Chiprate Generator)
    movlw   b'11000001'         ; Load config byte for TIMER3.
    ;        '1-------'         ; Enable 16-bit Read/Write.
    ;        '-1--X---'         ; Use TIMER3 for both CCP modules.
    ;        '--00----'         ; 1:1 Prescale.
    ;        '------0-'         ; Use internal clock (Fosc/4)
    ;        '-------1'         ; Turn Timer3 on
    movwf   T3CON               

    ; Set up interrupts.
    bcf     PIE1, CCP1IE        ; Disable the CCP1 interrupt.
    bcf     PIE2, CCP2IE        ; Disable the CCP2 interrupt.
    bcf     PIE2, TMR3IE        ; Disable the TMR3 interrupt.
    bsf     IPR1, CCP1IP        ; CCP1 (Receive) is high priority.
    bsf     IPR2, CCP2IP        ; CCP2 (Transmit) is high priority.

    return

;***************************************************************
; SUBROUTINE:   RX_ENABLE
; 
; Description:  Enable the receiver.
; Precond'ns:   
; Postcond'ns:  - Transmitter is disabled.
;               - /RXEN is activated (set low).
;               - CCP1 is set to interrupt in 5ms.
;               - CCP1 Interrupt is enabled (PIE1<CCP1IE> is set).
;               - TMR1 Interrupt is enabled (PIE1<TMR1IE> is set).
; Regs Used:    
;***************************************************************
RX_ENABLE:
    call    TX_DISABLE          ; Disable the Transmitter.
    bcf     NRXEN               ; Activate /RXEN.       

    ; Reset the state of the manchester decoder.
    clrf    bitcount            ; Reset the bit counter.
    bcf     clock_detect

    ; Set CCP1 to match in about 5ms.
    ; This gives the transmitter enough time to power up.
    bcf     PIE1, CCP1IE        ; Disable the interrupt to stop it going off.

    movlw   low TX_DELAY        ; Load the low byte of 5ms.
    addwf   TMR3L, W            ; Add low byte of TMR3 value.
    movwf   CCPR1L              ; Store it in low byte of CCPR1

    movlw   high TX_DELAY       ; Load the high byte of 5ms.
    addwfc  TMR3H, W            ; Add the high byte of TMR3 value with carry.
    movwf   CCPR1H              ; Store it in high byte of CCPR1

    ; Enable CCP1 Interrupt.
    movlw   CCP_IRP             ; Set CCP1 to generate software interrupt on match.
    movwf   CCP1CON
    bcf     PIR1, CCP1IF        ; Clear the flag so we don't get false interrupt.
    bsf     PIE1, CCP1IE        ; Enable the interrupt.

    ; Enable CCP2 Interrupt.
    movlw   CCP_IRP             ; Set CCP2 to generate software interrupt on match.
    movwf   CCP2CON             ; ...
    bcf     PIR2, CCP2IF        ; Clear the flag so we don't get false interrupt.
    bsf     PIE2, CCP2IE        ; Enable the CCP2 interrupt.
    
    return

;***************************************************************
; SUBROUTINES:  TX_ENABLE
; 
; Description:  Enable the transmitter.
; Precond'ns:   
; Postcond'ns:  - Receiver is disabled.
;               - /TXEN is activated (set low).
;               - VGA is powered up.
;               - CCP2 is set to interrupt in 5ms.
;               - CCP2 Interrupt is enabled (PIE2<CCP2IE> is set).
; Regs Used:    
;***************************************************************
TX_ENABLE:
    ; Enable the transmitter hardware.
    call    RX_DISABLE          ; Disable the Receiver.
    bsf     PWUP                ; Power up the VGA.
    bcf     NTXEN               ; Activate /TXEN.

    ; Reset the state of the encoder.
    clrf    phy_flags
    clrf    bitcount

    ; Set CCP2 to match in about 5ms.
    ; This gives the transmitter enough time to power up.
    bcf     PIE2, CCP2IE        ; Disable the interrupt to stop it going off.

    movlw   low TX_DELAY        ; Load the low byte of 5ms.
    addwf   TMR3L, W            ; Add low byte of TMR3 value.
    movwf   CCPR2L              ; Store it in low byte of CCPR2.

    movlw   high TX_DELAY       ; Load the high byte of 5ms.
    addwfc  TMR3H, W            ; Add the high byte of TMR3 value with carry.
    movwf   CCPR2H              ; Store it in high byte of CCPR2.
    
    ; Enable CCP2 Interrupt.
    movlw   CCP_TOGGLE          ; Set CCP2 to toggle TxD on a match
    movwf   CCP2CON             ; ...
    bcf     PIR2, CCP2IF        ; Clear the flag so we don't get false interrupt.
    bsf     PIE2, CCP2IE        ; Enable the CCP2 interrupt.
    
    return

;***************************************************************
; SUBROUTINES:  RX_DISABLE
; 
; Description:  Disable the receiver.
; Precond'ns:   
; Postcond'ns:  - TMR1 Interrupt is disabled.
;               - CCP1 Interrupt is disabled.
;               - /TXEN is deactivated (set high).
; Regs Used:    
;***************************************************************
RX_DISABLE:
    bcf     PIE1, CCP1IE        ; Disable CCP1 Interrupt.
    bsf     NRXEN               ; Deactivate /RXEN.
    return

;***************************************************************
; SUBROUTINES:  TX_DISABLE
; 
; Description:  Disable the transmitter.
; Precond'ns:   
; Postcond'ns:  - CCP2 Interrupt is disabled.
;               - VGA is powered down.
;               - /TXEN is deactivated (set high).
; Regs Used:    
;***************************************************************
TX_DISABLE:
    bcf     PIE2, CCP2IE        ; Disable CCP2 Interrupt.
    movlw   CCP_IRP             ; ...
    movwf   CCP2CON             ; ...

    bcf     PWUP                ; Power down the VGA.
    bsf     NTXEN               ; Deactivate /TXEN.
    return




;***************************************************************
; SUBROUTINE:   MANCHESTER_ENCODER (ISR, CCP2)
; 
; Description:  - Manchester encodes bytes from txbyte.
; Precond'ns:   - CCP2 Interrupt must be serviced.
; Postcond'ns:  
; Regs Used:    
;***************************************************************
MANCHESTER_ENCODER:

; ---------------------------------------------------------------
; DECISION: Check if we are encoding a clock chip or a data chip.
    btfsc   tx_clock            ; Are we encoding a clock chip?
    bra     _TX_CLOCKCHIP       ; Yes - encode a clock chip.

; ---------------------------------------------------------------
; DECISION: Check if we need to get another data byte.
    tstfsz  bitcount            ; Does bitcount = 0?
    bra     _TX_DATACHIP        ; NO - don't get another byte.

; ---------------------------------------------------------------
; DECISION: Check if we have another data byte to transmit.
    btfsc   txbyte_empty        ; Is there a transmit byte?
    bra     TX_DISABLE          ; NO - disable the transmitter.
    
; ---------------------------------------------------------------
; PROCESS: Fetch the next data byte.
    movff   txbyte, shiftreg_l  ; Fetch the next byte from the buffer.
    movlw   d'08'               ; Reset the bit counter.
    movwf   bitcount            ; ...
    bsf     txbyte_empty        ; Set a flag to say we are ready for the next byte.

; ---------------------------------------------------------------
; PROCESS: Encode a data chip on the next cycle.
_TX_DATACHIP:
    decf    bitcount            ; Decrement the bit counter.
    rrcf    shiftreg_l          ; Pop a the LSB off the end of the data byte.
    bc      _DATACHIP_HIGH      ; Data bit was high.
;   bnc     _DATACHIP_LOW       ; Data bit was low.

_DATACHIP_LOW:
    btfss   PORTC, CCP2         ; Is TxD also low?
    bra     _TX_TOGGLE_IN_TWO   ; Yes - we can skip the next cycle.
    bsf     tx_clock            ; NO - set the clock flag ...
    bra     _TX_TOGGLE_IN_ONE   ; ... and Toggle TxD in one chip period.

_DATACHIP_HIGH:
    btfsc   PORTC, CCP2         ; Is TxD also high?
    bra     _TX_TOGGLE_IN_TWO   ; Yes - we can skip the next cycle.
    bsf     tx_clock            ; NO - set the clock flag ...
    bra     _TX_TOGGLE_IN_ONE   ; ... and Toggle TxD in one chip period.

; ---------------------------------------------------------------
; PROCESS: Set CCP2 (Chiprate Generator) to toggle TxD in one chip period.
_TX_TOGGLE_IN_ONE:
    movlw   low CHIPTIME        ; Load up the low byte of the chip time.
    addwf   CCPR2L              ; Add it to the low byte of CCPR2
    movlw   high CHIPTIME       ; Load up the high byte of the chip time.
    addwfc  CCPR2H              ; Add it to the high byte of CCPR2 with carry.
    return

; ---------------------------------------------------------------
; PROCESS: Set CCP2 (Chiprate Generator) to toggle TxD in two chip periods.
_TX_TOGGLE_IN_TWO:
    movlw   low CHIPTIME*2      ; Load up the low byte of the chip time * 2.
    addwf   CCPR2L              ; Add it to the low byte of CCPR2
    movlw   high CHIPTIME*2     ; Load up the high byte of the chip time * 2.
    addwfc  CCPR2H              ; Add it to the high byte of CCPR2 with carry.
    return

; ---------------------------------------------------------------
; PROCESS: Encode a clock chip.
_TX_CLOCKCHIP:

    ; Clear the clock flag.
    bcf     tx_clock            

    ; Toggle TxD in one chip period.
    movlw   low CHIPTIME        ; Load up the low byte of the chip time.
    addwf   CCPR2L              ; Add it to the low byte of CCPR2
    movlw   high CHIPTIME       ; Load up the high byte of the chip time.
    addwfc  CCPR2H              ; Add it to the high byte of CCPR2 with carry.

    return


;***************************************************************
; SUBROUTINE:   MANCHESTER_DECODER (ISR, CCP1)
; 
; Description:  - Decodes manchester encoded bits and shifts them
;                   into shiftreg_h and shiftreg_l.
; Precond'ns:   - CCP1 interrupt must be serviced.
; Postcond'ns:  
; Regs Used:    
;***************************************************************
MANCHESTER_DECODER:

; -------------------------------------------------------------------------------
; DECISION: Check if CCP1 was set to capture or compare mode.
; Compare mode (CCP1CON<3> is set) - Go to the sampling routine.
; Capture mode (CCP1CON<3> is clear) - Go to the clock detect routine.
    btfsc   rx_get_sample       ; Were we waiting to get a sample?
    bra     _SAMPLE             ; YES - Take a sample.
    ;bra    _CLOCK_DETECT       ; NO - Synchronise to the clock edge.

; -------------------------------------------------------------------------------
; PROCESS: Sync to the manchester clock (the edges in the middle of each data bit)
; Even if the edge we just captured is not the correct clock edge, it will sort itself
; out as soon as the data bit changes.
; a) Set CCP2 (Clock Watchdog) to match in 2.2 chip periods
; b) Set CCP1 (Chiprate Generator) to match in 1.5 chip periods
_CLOCK_DETECT:

    ; a) Set CCP2 (Clock Watchdog) to match in 2.2 chip periods.
    ; If don't detect a clock edge and run this routine ever two chip periods,
    ; CCP2 will generate an interrupt, indicating we have lost the clock.
    movlw   low CHIPTIME*22/10  ; Load the low byte of 2.2 chiptimes.
    addwf   CCPR1L, W           ; Add the low byte of the current time.
    movwf   CCPR2L              ; Store it to CCPR2.

    movlw   high CHIPTIME*22/10 ; Load the high byte of 2.2 chiptimes.
    addwfc  CCPR1H, W           ; Add the high byte of the current time with carry.
    movwf   CCPR2H              ; Store it to CCPR2.

    ; This is done in RX_ENABLE
    movlw   CCP_IRP             ; Set CCP2 to generate a software interrupt on match.
    movwf   CCP2CON             ; ...
    bcf     PIR2, CCP2IF        ; Prevent a false interrupt after changing modes.

    ; b) Set CCP1 (Chiprate Generator) to match in 1.5 chip periods.
    ; CCP1 should generate an interrupt in the middle of the first chip of the
    ; next bit... the perfect place to sample.
    movlw   low CHIPTIME*3/2    ; Load low byte of 1.5 chiptimes.
    addwf   CCPR1L              ; Add it to the low byte of CCPR1.

    movlw   high CHIPTIME*3/2   ; Load the high byte of the chiptime.
    addwfc  CCPR1H              ; Add it to the high byte of CCPR1 with carry.

    movlw   CCP_IRP             ; Set CCP1 to generate a software interrupt on match.
    movwf   CCP1CON             ; ...
    bcf     PIR1, CCP1IF        ; Prevent a false interrupt after changing modes.

    return                      ; Done.

; -------------------------------------------------------------------------------
; PROCESS: Take a sample and place it into the shift register.
; a) Test the value of RxD.
;       High: set STATUS<C>, set CCP1 to capture the next falling edge.
;       Low: clear STATUS<C>, set CCP1 to capture the next rising edge.
; b) Rotate shiftreg_h and shiftreg_l right through the carry bit.
; c) Increment the bit counter.
;
; NB - RRCF shifts the carry bit into the register -
; this is why we are writing to STATUS<C>.
; Make sure any instructions do not change STATUS<C> until the RRCF is done.
_SAMPLE:
    btfss   PORTC, CCP1         ; Is the TxD high or low?
    bra     _SAMPLE_LOW
_SAMPLE_HIGH:
    bsf     STATUS, C           ; Set the carry bit.
    movlw   CCP_FALLING         ; Next edge will be a falling edge.
    bra     _SAMPLE_1           ; Continue.
_SAMPLE_LOW:
    bcf     STATUS, C           ; Clear the carry bit.
    movlw   CCP_RISING          ; Next edge will be a rising edge.
_SAMPLE_1:
    movwf   CCP1CON             ; Configure CCP1.
    bcf     PIR2, CCP1IF        ; Prevent a false interrupt after changing modes.
    rrcf    shiftreg_h          ; Right shift the carry bit into the shiftreg.
    rrcf    shiftreg_l          ; ...
    incf    bitcount            ; Increment the bit counter to say we received a bit.

; -------------------------------------------------------------------------------
; DECISION: Check if we have received 8 bits.
; YES - Reset the bit counter, copy the byte into rxbyte, and set rxbyte_full.
; NO - increment the bit counter and return.
    movlw   d'07'               ;
    cpfsgt  bitcount            ; Is bitcount >= 8?
    return                      ; NO - return.
    
    ; Process the byte we just received.
    clrf    bitcount            ; Clear the bit counter.
    movff   shiftreg_h, rxbyte  ; Copy the received byte into rxbyte.
    bsf     rxbyte_full         ; Set a flag to say we have received a byte.
    bsf     clock_detect        ; Set the clock detect flag.
    bsf     RXLED
    return                      ; Bugga orf.




;***************************************************************
; Subroutine:   CLOCK_WATCHDOG (ISR, CCP2)
; 
; Description:  Called when CCP2 matches in receive mode, indicating that
;               clock synchronisation has been lost in the receiver.
; Precond'ns:   CCP2 Interrupt must be serviced.
; Postcond'ns:  
; Regs Used:    
;***************************************************************
CLOCK_WATCHDOG:
    ; We have lost the clock, so reset the decoder.
    clrf    bitcount            ; Reset the bit counter.
    clrf    shiftreg_l          ; Clear the shift registers
    clrf    shiftreg_h          ; ...
    bcf     clock_detect        ; Reset the clock detect flag.
    bcf     RXLED
    return

 end
