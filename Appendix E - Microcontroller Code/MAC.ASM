;*************************************************************************
; MEDIA ACCESS CONTROL LAYER MODULE
; 
; Version 1.00
; 16/9/2005
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

; Preamble (Synchronization Header)
; These variables define the preamble that will be sent at the start of 
; every packet.
; The encoder will sent (PREAMBLE_LEN) bytes of PREAMBLE_BYTE,
; followed by one byte of SOF_BYTE.
; The decoder will detect one byte of PREAMBLE_BYTE immediately
; followed by one byte of SOF_BYTE.
 #define    PREAMBLE_BYTE   0x55    ; 10101010 LSB first
 #define    SOF_BYTE        0x00    ; 00000000 LSB first

; CSMA: number of times to retry before giving up.
 #define    RETRY_LIMIT     d'10'   ; retry 10 times.

; Transmit/Recieve buffer sizes
RXBUFLEN        equ 0x40            ; 64 bytes
TXBUFLEN        equ 0x40            ; 64 bytes

; MAC Constants - in TMR0 periods. TMR0 is running on a 1:256 prescaler.
; Amount of time to stay awake for.
WAKETIME        equ .39             ; 10 milliseconds
; Amount of time to sleep for during low traffic.
SLEEPTIME_L     equ .3906           ; 1 second
;SLEEPTIME_L        equ .10838      ; 2.7746 seconds
; Amount of time to sleep for during high traffic.
SLEEPTIME_H     equ .217            ; 55.6 milliseconds

; Preamble length during low traffic.
PREAMBLELEN     equ .125            ; 125 bytes = 200ms @ 5000 baud
; Preamble length during high traffic.
PREAMBLELEN2    equ .32             ; 32 bytes = 50ms @ 5000 baud

; Define this cosnstant to use BAPSA (BASIC ADAPTIVE PREAMBLE SAMPLING ALGORITHM)
#define BAPSA

;*************************************************************************
; CONSTANTS

; Frame types
NORMAL          equ 0x00
CLOCKSYNC       equ 0x01

;******************************************************************************
; IMPORTED VARIABLES

 ; Physical Layer
 #include   "PHY.inc"

 ; CRC Checking
 #include   "CRC.inc"

 ; Software Timer 4 - See <SWTimers.asm>
 extern     TMRF4, TMRL4, TMRH4

;******************************************************************************
; GLOBAL VARIABLES

; Transmit/Receive Buffers
.mac_buffers udata
rxbuf           res RXBUFLEN    ; Receive Buffer    
txbuf           res TXBUFLEN    ; Transmit Buffer
rxbuf_b         res RXBUFLEN    ; Receive Buffer    
txbuf_b         res TXBUFLEN    ; Transmit Buffer

.mac_globals udata_acs
; Status flags.
mac_status          res 1
 #define    packet_received     mac_status, 0  ;Indicates we have received a packet.
 #define    packet_transmitted  mac_status, 1  ;Indicates we are ready to send a packet.
 #define    retry_limit_reached mac_status, 2  ;Indicates we reached the retry limit.

 global rxbuf, txbuf, mac_status

;******************************************************************************
; LOCAL VARIABLES
.mac_locals udata_acs

; Internal Flags
mac_flags       res 1
 #define mac_csma_active   mac_flags, 0    ; The CSMA algorithm is running.
 #define mac_sampling      mac_flags, 1    ; We are in a sampling period.
 #define mac_schedule_htp  mac_flags, 2    ; We are in a scheduled high traffic period.
 #define mac_dynamic_htp   mac_flags, 3    ; We are in a dynamic high traffic period.


tx_flags        res 1
 #define tx_clock       tx_flags, 0 ; Do we have to do a toggle?
 #define preamble_sent  tx_flags, 1 ; Set when we have sent the start of frame.
 #define framelen_sent  tx_flags, 2 ; Set when we have sent the frame length.
 #define frame_sent     tx_flags, 3 ; Set when we have sent all the data bytes.
rx_flags        res 1
 #define got_sof        rx_flags, 0 ; Set when we have detected the start of frame.
 #define got_framelen   rx_flags, 1 ; Set when we have received frame length.
 #define got_frame      rx_flags, 2 ; Set when we have received an entire frame.

; Frame Encoding
preamble_len_l  res 1       ; Preamble length
preamble_len_h  res 1       ; ...
bytecount_l     res 1       ; Byte Counter
bytecount_h     res 1       ; ...

; CSMA
retry_counter   res 1       ; Count how many times we have tried to transmit.
timeout_l       res 1       ; Retry timeout.
timeout_h       res 1
timeout_mask_l  res 1       ; Retry timeout mask.
timeout_mask_h  res 1
    

; Preamble sampling
sleeptime_l     res 1       ; Sleep value in use.
sleeptime_h     res 1       ; ...
hitraf_timer    res 1       ; High traffic mode timer.

schedule_timer  res 1       ; Timer for scheduled high traffic periods
dynamic_timer   res 1       ; Timer for dynamic high traffic periods

sleeping_con    res 1       ; Control register for sleep interval.
preamble_con    res 1       ; Control register for preamble length.

scheduled equ .0
dynamic equ .1



; Popcorn buffering
popcorn_flags   res 1
 #define switch_rxbuf   popcorn_flags, 0  ; rxbuf_a and rxbuf_b are switched.
 #define switch_txbuf   popcorn_flags, 1  ; rxbuf_b and rxbuf_a are switched.

;******************************************************************************
; IMPORTED SUBROUTINES

 #include "Buffers.inc"         ; Software Buffers Header File

 #include "RTC.inc"

;******************************************************************************
; EXPORTED SUBROUTINES

 global     MAC_INIT, MAC_TEST                             ; Initialisation
 global     FRAME_ENCODER, FRAME_DECODER, RESET_DECODER    ; High Prioirity ISR's
 global     CSMA, TMR0_ISR, INT1_ISR, TRAFFIC_CONTROL      ; Low Priority ISR's

;******************************************************************************
; START OF CODE

 CODE

MAC_TEST:
    
    BUF_SEL txbuf, TXBUFLEN
    BUF_CLEAR
    movlw   41
    BUF_PUT 
    movlw   42
    BUF_PUT 
    movlw   43
    BUF_PUT 
    BUF_MARKEND

    call    TX_PACKET

Loopy:
    clrwdt
    bra Loopy




;***************************************************************
; SUBROUTINE:   MAC_INIT
; 
; Description:  Initialisation for mac layer.
; Precond'ns:   -
; Postcond'ns:  
; Regs Used:    WREG, CCP1 Registers, CCP2 Registers, TMR3 Registers
;***************************************************************
MAC_INIT:

    ; Clear all variables.
    clrf    mac_flags
    clrf    retry_counter
    clrf    schedule_timer
    clrf    dynamic_timer
    clrf    sleeping_con
    clrf    preamble_con

    ; Set Sleep interval and preamble length for low traffic
    call    SET_LTP_SLEEP
    call    SET_LTP_PREAMBLE
            
    ; Configure TIMER0 (Sampling Timer)
    bcf     INTCON, TMR0IE      ; Disable the TMR0 interrupt.
    bcf     INTCON2, TMR0IP     ; TMR0 (Sampling Timer) is low priority.
    movlw   b'10000111'         ; Load config byte for TIMER0.
    ;        '1-------'         ; Enable Timer0.
    ;        '-0------'         ; Configure as 16-bit counter.
    ;        '--0-----'         ; Use internal clock (Fosc/4).
    ;        '----0---'         ; Use Prescaler.
    ;        '-----111'         ; 1:256 Prescale.
    movwf   T0CON

    ; Configure TIMER1 (High Traffic Mode Timer)
    bcf     PIE1, TMR1IE        ; Disable the TMR1 interrupt.
    bcf     IPR1, TMR1IP        ; TMR0 (High Traffic Timer) is low priority.
    movlw   b'10110000'         ; Load config byte for TIMER1.
    ;        '1-------'         ; Enable 16-bit read/writes.
    ;        '--11----'         ; 1:8 Prescale.
    ;        '----0---'         ; Disable Timer1 RC Oscillator.
    ;        '------0-'         ; Use internal clock (Fosc/4).
    ;        '-------0'         ; Disable TIMER1
    movwf   T1CON
    bsf     PIE1, TMR1IE        ; Enable the TMR1 interrupt.


    ; Make sure the TIMER0 Interrupt runs straight away to initialise TMR0
    bsf     INTCON, TMR0IF      ; SET!!! the TIMER0 Interrupt flag.
    bsf     INTCON, TMR0IE      ; Enable the TIMER0 Interrupt.

    return








; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                        MEDIA ACCESS CONTROL
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################

;***************************************************************
; SUBROUTINE:   TX_PACKET
; 
; Description:  Transmit a packet.
; Precond'ns:   An outgoing packet has been placed in Buffer 1.
; Postcond'ns:  CRC is appended to buffer and packet transmission is initiated.
; Regs Used:    
;***************************************************************
TX_PACKET:

    ; Calculate the CRC on the packet.
    BUF_SEL txbuf, TXBUFLEN     ; Select the transmit buffer
    call    CALC_CRC            ; Calculate the CRC on the buffer.

    movf    crclow, W           ; Load the crc low byte...
    BUF_PUT                     ; And append it to the buffer. <BUFPUT in buffers.asm>
    movf    crchigh, W          ; Load the crc high byte...
    BUF_PUT                     ; And append it to the buffer. <BUFPUT in buffers.asm>
    BUF_MARKEND

_TX_PACKET_TEST:
    ; Reset the retry count and timeout mask and run the CSMA routine.
    movlw   RETRY_LIMIT         ; Set the retry counter.
    movwf   retry_counter       ; ...
    movlw   b'00111111'         ; Set the retry timeout mask.
    movwf   timeout_mask_l      ; ...
    clrf    timeout_mask_h      ; ...
    bsf     mac_csma_active     ; Set the flag that lets the CSMA subroutine run.

    btfss   mac_sampling        ; Are we currently sampling?
    call    _START_SAMPLING     ; NO - start sampling now.
    return


;***************************************************************
; SUBROUTINE:   TMR0_ISR (ISR)
; 
; Description:  
;               
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
TMR0_ISR:

    ; Service the TMR0 Interrupt.
    SERVICE_IRP INTCON, TMR0IF, INTCON, TMR0IE

    ; Check whether CSMA or Preamble Sampling has control
    btfsc   mac_csma_active     ; Is CSMA active?
    bra     CSMA                ; YES - run the CSMA routine.
    bra     PREAMBLE_SAMPLING   ; NO - run the Preamble Sampling routine.


;***************************************************************
; SUBROUTINE:   PREAMBLE_SAMPLING (ISR)
; 
; Description:  Use TIMER0 to control preamble sampling.
;               Should be configured as a low priority interrupt.
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
PREAMBLE_SAMPLING:
    
    ; Check if we are currently transmitting.
    btfss   NTXEN               ; Are we transmitting?
    bra     _DONTTOUCH          ; Don't touch anything.

    ; Check if we are currently sampling.
    btfss   mac_sampling        ; Are we sampling?
    bra     _START_SAMPLING     ; NO - start sampling.

; --------------------------------------------------------------
; Perform a clear channel assesment.
_PREAMBLE_SAMPLING_CCA:

    bcf     mac_sampling        ; Clear the sampling flag.
    btfss   clock_detect        ; Do we have a valid signal?
    call    RX_DISABLE          ; NO - Stop sampling.
    
    ; Set TMR0 to overflow in SLEEPTIME.
    ; When TMR0 overflows, we will start sampling again.
_DONTTOUCH:
    bcf     T0CON, TMR0ON       ; Stop the timer
    nop                         ; Wait for it to stop.
    movff   sleeptime_h, TMR0H
    movff   sleeptime_l, TMR0L
    bsf     T0CON, TMR0ON       ; Start the timer

    return

; -------------------------------------------------------------
; Start sampling.
_START_SAMPLING:

    bsf     mac_sampling        ; Set the sampling flag.
    btfsc   NRXEN               ; Is the receiver already enabled?
    call    RX_ENABLE           ; NO - Enable the receiver.

    ; Set TMR0 to overflow in WAKETIME.
    ; When TMR0 overflows we will stop sampling.
    bcf     T0CON, TMR0ON       ; Stop the timer
    nop                         ; Wait for it to stop.
    movlw   high (0xFFFF - WAKETIME)
    movwf   TMR0H
    movlw   low (0xFFFF - WAKETIME)
    movwf   TMR0L
    bsf     T0CON, TMR0ON       ; Start the timer

    return


    
;***************************************************************
; SUBROUTINE:   CSMA
;   
; Description:  - Checks if the medium is free.
;               - If the medium is free, transmission is initiated.
;               - If it is not free, a random timeout is set.
;               - If after the timeout the medium is still not free, 
;                   the timeout is doubled and reset.
;
;               - This routine is designed to be called once from normal
;                 code, and subsequently from interrupt on TIMER2.
;
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
CSMA:

; --------------------------------------------------------------
; Perform a clear channel assesment.
_CSMA_CCA:
;   btfss   NCD                 ; Not used becase of noise problems.
    btfsc   clock_detect        ; Is there a valid clock?
    bra     _CSMA_WAIT          ; YES - set a timeout.
    
; ---------------------------------------------------------
; We are Clear To Send, initiate transmission.
_CSMA_CTS:


    ; Initialise the frame encoder
    call    RX_DISABLE          ; Turn off the decoder so it can't 
                                ; change things while we are initialising.     
    movlw   low preamble_len_l  ; Load up the preamble counter
    movwf   bytecount_l         ; ...
    movlw   low preamble_len_h  ; ...
    movwf   bytecount_h         ; ...
    clrf    tx_flags            ; Clear all flags.
    call    FRAME_ENCODER       ; Run the frame encoder once so the first byte is loaded.
    call    TX_ENABLE           ; Enable the transmitter.

    bcf     mac_csma_active     ; Hand control back to the preamble sampler.

    ; Set a new timeout, so it won't take 16.777 seconds for the preamble
    ; sampler routine to run again.
    bcf     T0CON, TMR0ON       ; Stop the timer
    nop                         ; Wait for it to stop.
    movff   sleeptime_h, TMR0H
    movff   sleeptime_l, TMR0L
    bsf     T0CON, TMR0ON       ; Start the timer
    
    return  
    
; ---------------------------------------------------------
; Media is busy, set a random timeout with exponential backoff.
; Max wait Time = 2^retry_count milliseconds
_CSMA_WAIT:
    
    btg     TXLED

    ; Check how many times we have tried to transmit.
    movf    retry_counter,f     ; Has the retry counter reached zero?
    bz      _CSMA_GIVEUP        ; YES - give up.

    ; Decrement the retry counter, and multiply the timeout mask by two.
    decf    retry_counter       ; We are about to burn another retry.
    bsf     STATUS, C           ; Set the carry bit
    rlcf    timeout_mask_l      ; Left shift the mask.
    rlcf    timeout_mask_h      ; ...

    ; Set a random timeout
    movf    crclow, w           ; Get the crc low byte
    xorwf   TMR3H, w            ; Multiply it with the high byte of TMR3
    andwf   timeout_mask_l, w   ; Mask it.
    comf    WREG                ; Compliment it.
    movwf   timeout_l           ; Save it
    movf    crchigh, w          ; Get the crc high byte.
    xorwf   TMR3L, w            ; Multiply it with the low byte of TMR3.
    andwf   timeout_mask_h, w   ; Mask it.
    comf    WREG                ; Negate it.
    movwf   timeout_h           ; Save it.

    ; #######################################################################
    ; TO DO - set interim samples if the timeout is greater than the sleep
    ; interval required to maintain connectivity.
    ; #######################################################################

    bcf     T0CON, TMR0ON       ; Stop the timer
    nop                         ; Wait for it to stop.
    movff   timeout_h, TMR0H
    movff   timeout_l, TMR0L
    bsf     T0CON, TMR0ON       ; Start the timer

    return

; ---------------------------------------------------------
; We have reached the retry limit; Give up already.
_CSMA_GIVEUP:

    return

; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                       MAC LAYER RECONFIGURATION ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################

;***************************************************************
; SUBROUTINE:   SET_HTP_SLEEP
; 
; Description:  Sets the sleep interval for high traffic (more frequent sampling).
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_HTP_SLEEP:
    ; Set the sleep interval for high traffic.
    movlw   low (0xFFFF - SLEEPTIME_H)
    movwf   sleeptime_l
    movlw   high (0xFFFF - SLEEPTIME_H)
    movwf   sleeptime_h
    return
    
;***************************************************************
; SUBROUTINE:   SET_HTP_PREAMBLE
; 
; Description:  Sets the preamble length for high traffic (shorter preamble).
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_HTP_PREAMBLE:
    ; Set the preamble length for high traffic.
    movlw   high PREAMBLELEN2
    movwf   preamble_len_h
    movlw   low PREAMBLELEN2
    movwf   preamble_len_l
    return
    
;***************************************************************
; SUBROUTINE:   SET_LTP_SLEEP
; 
; Description:  Sets the sleep interval for low traffic (less frequent sampling).
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_LTP_SLEEP:
    ; Set the sleep interval for low traffic.
    movlw   low (0xFFFF - SLEEPTIME_L)
    movwf   sleeptime_l
    movlw   high (0xFFFF - SLEEPTIME_L)
    movwf   sleeptime_h
    return
    
;***************************************************************
; SUBROUTINE:   SET_LTP_PREAMBLE
; 
; Description:  Sets the preamble length for low traffic (longer preamble).
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_LTP_PREAMBLE:
    ; Set the preamble length for low traffic.
    movlw   high PREAMBLELEN
    movwf   preamble_len_h
    movlw   low PREAMBLELEN
    movwf   preamble_len_l
    return

;***************************************************************
; SUBROUTINE:   TRAFFIC_CONTROL (ISR)
; 
; Description:  Runs every 0.524288 seconds under TIMER1 Low Priority Interrupt.
;               Counts down any high traffic periods which are running, and
;               configures the preamble length and sleep interval accordingly.
;               
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
TRAFFIC_CONTROL:

    ; Service the TMR1 Interrupt.
    SERVICE_IRP PIR1, TMR1IF, PIE1, TMR1IE

    ; Count down the scheduled high traffic period.
    call    _SCHEDULED_HTP

    ; Count down the dynamic high traffic period.
    call    _DYNAMIC_HTP

    ; Check if anything is still forcing the high traffic (shorter) sleep interval.
    call    _CHECK_SLEEP_INTERVAL

    ; Check if anything is still forcing the high traffic (shorter) preamble length.
    call    _CHECK_PREAMBLE_LENGTH

    ; return
    return

; ------------------------------------------------------------------------------
; Count down the scheduled high traffic period.
_SCHEDULED_HTP:

    btfss   mac_schedule_htp        ; Are we in a scheduled high traffic period?
    return                          ; NO - return.
    
    decfsz  schedule_timer          ; Has the scheduled high traffic timer reached zero?
    return                          ; NO - return.

    ; End the scheduled high traffic period.
    bcf     mac_schedule_htp        ; We are no longer in a scheduled HTP.
    bcf     sleeping_con, scheduled ; Scheduled HTP is not forcing shorter sleeping.
    bcf     preamble_con, scheduled ; Scheduled HTP is not forcing shorter preamble.

    return

; ------------------------------------------------------------------------------
; Count down the dynamic high traffic period.
_DYNAMIC_HTP:

    btfss   mac_dynamic_htp         ; Are we in a dynamic high traffic period?
    return                          ; NO - return.
    
    decfsz  dynamic_timer           ; Has the dynamic high traffic timer reached zero?
    return                          ; NO - return.

    ; End the dynamic high traffic period.
    bcf     mac_dynamic_htp         ; We are no longer in a dynamic HTP.
    bcf     sleeping_con, dynamic   ; Dynamic HTP is not forcing shorter sleeping.
    bcf     preamble_con, dynamic   ; Dynamic HTP is not forcing shorter preamble.

    return

; ------------------------------------------------------------------------------
; Check if anything is still forcing the high traffic (shorter) sleep interval.
_CHECK_SLEEP_INTERVAL:

    tstfsz  sleeping_con        ; Is sleeping_con zero?
    return                      ; NO - return.

    ; YES - revert back to the low traffic (longer) sleeping interval.
    call    SET_LTP_SLEEP
    return

; ------------------------------------------------------------------------------
; Check if anything is still forcing the high traffic (shorter) preamble length.
_CHECK_PREAMBLE_LENGTH:
    ; Check if anything is still forcing a shorter preamble.
    tstfsz  preamble_con        ; Is preamble_con zero?
    return                      ; NO - return.

    ; YES - revert back to the low traffic (longer) preamble.
    call    SET_LTP_PREAMBLE
    return


;***************************************************************
; SUBROUTINE:   SET_SCHEDULED_HTP
; 
; Description:  Starts a scheduled high traffic period for 9.961 seconds.
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_SCHEDULED_HTP:

    ; Clear TIMER1 (High Traffic Timer) and start it
    clrf        TMR1H
    clrf        TMR1L
    bsf         T1CON, TMR1ON

    ; Load the timer preset (19 * 524288us = 9.96s)
    movlw   .19     
    movwf   schedule_timer

    ; Set the flags for a scheduled high traffic period.
    bsf     mac_schedule_htp        ; Clear the flag to say we are in a scheduled HTP.
    bsf     sleeping_con, scheduled ; The scheduled HTP is forcing shorter sleeping.
    bsf     preamble_con, scheduled ; The scheduled HTP is forcing shorter preamble.

    ; Set the sleep interval for high traffic.
    call    SET_HTP_SLEEP
    
    ; Set the preamble length for high traffic.
    call    SET_HTP_PREAMBLE

    return

;***************************************************************
; SUBROUTINE:   SET_DYNAMIC_HTP
; 
; Description:  Starts a scheduled high traffic period for 5.24 seconds.
; Precond'ns:   Set sleeping_con<dynamic> to force faster sampling.
;               Set preamble_con<dynamic> to force shorter preamble.
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_DYNAMIC_HTP:

    ; Clear TIMER1 (High Traffic Timer) and start it
    clrf        TMR1H
    clrf        TMR1L
    bsf         T1CON, TMR1ON

    ; Load the timer preset (10 * 524288us = 5.24s)
    movlw   .10 
    movwf   dynamic_timer

    ; Set the flags for a dynamic high traffic period.
    bsf     mac_dynamic_htp         ; Clear the flag to say we are in a dynamic HTP.

    ; Check if the dynamic HTP needs to force shorter sleeping.
    btfsc   sleeping_con, dynamic   ; Well does it?
    call    SET_HTP_SLEEP           ; YES - set shorter sleeping.

    ; Check if the dynamic HTP needs to force shorter sleeping.
    btfsc   preamble_con, dynamic   ; Well does it?
    call    SET_HTP_PREAMBLE        ; YES - set shorter sleeping.

    ; If we aren't in a sampling period, start one right now.
    btfss   mac_sampling
    bsf     INTCON, TMR0IF          ; SET!!! the TIMER0 Interrupt flag.

    return

;***************************************************************
; SUBROUTINE:   INT1_ISR (ISR)
; 
; Description:  Starts a scheduled high traffic period for 9.961 seconds.
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
INT1_ISR:
    
    ; Service the INT1 Interrupt.
    SERVICE_IRP INTCON3, INT1IF, INTCON3, INT1IE

    ; Clear the RTC IRQ flags
    call    RTC_CLEAR_IRQS

    ; Start 
    call    SET_SCHEDULED_HTP

    ; return
    return
    
;***************************************************************
; SUBROUTINE:   TX_SYNC_FRAME:
; 
; Description:  Transmit a clock sync frame.
; Precond'ns:
; Postcond'ns:  
; Regs Used:    
;***************************************************************
TX_SYNC_FRAME:
    
    ; Select the TX Buffer and clear it.
    BUF_SEL txbuf, TXBUFLEN     ; Select the transmit buffer.
    BUF_CLEAR                   ; Clear it.

    ; Put the frame type into the buffer
    movlw   CLOCKSYNC
    BUF_PUT

    ; Load the current clock value into the buffer
    call    RTC_GET_CLOCK

    ; Transmit the packet
    call    TX_PACKET

    ; Set low traffic mode (make sure our preamble is long enough for all nodes)
;   call    SET_LOW_TRAFFIC

    return

    

; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                           FRAME ENCODING / DECODING ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################



;***************************************************************
; Subroutine:   FRAME_ENCODER
; 
; Description:  Encodes Frames. Should be called each time another
;               bit is encoded.
;
; Precond'ns:   
;
; Postcond'ns:  
;
; Regs Used:    WREG, FSR0
;***************************************************************
FRAME_ENCODER:

; ---------------------------------------------------------------
; DECISION: Check if the manchester encoder is ready for the next byte,
; and if we have a byte to give.
    btfss   txbyte_empty        ; Is the transmit buffer empty?
    return                      ; NO - return.
    btfsc   frame_sent          ; Do we have any data to transmit?
    bra     _FRAME_SENT         ; NOOOOOOO!
    bcf     txbyte_empty        ; YES - clear the flag and continue.

; ---------------------------------------------------------------
; DECISION: Check if we have transmitted the preamble and SOF yet.
    btfsc   framelen_sent       ; Have we sent the frame length yet?
    bra     _TX_DATABYTE        ; YES - transmit a data byte.
                                ; NO - continue.

; ---------------------------------------------------------------
; DECISION: Check if we have transmitted the preamble and SOF yet.
    btfsc   preamble_sent       ; Have we sent the SOF yet?
    bra     _TX_FRAMELEN        ; YES - transmit the frame length.
                                ; NO - continue.

; ---------------------------------------------------------------
; DECISION: Check if we have transmitted enough preamble bytes yet.
    tstfsz  bytecount_h         ; Is the bytecounter zero?
    bra     _TX_PREAMBLE        ; NO - send a preamble byte.
    tstfsz  bytecount_l         ; Is the bytecounter zero?
    bra     _TX_PREAMBLE        ; NO - send a preamble byte.
    bra     _TX_SOF             ; YES - send a SOF byte.

; ---------------------------------------------------------------
; PROCESS: Send a preamble byte.
_TX_PREAMBLE:
    movlw   PREAMBLE_BYTE       ; Copy the preamble byte ...
    movwf   txbyte              ; ... to the transmit reigster.
    decf    bytecount_l         ; Deccrement the counter.
    btfss   STATUS, C           ; ...
    decf    bytecount_h         ; ...
    return

; ---------------------------------------------------------------
; PROCESS: Send a SOF byte.
_TX_SOF:
    movlw   SOF_BYTE            ; Copy the SOF byte ...
    movwf   txbyte              ; ... to the transmit reigster.
    bsf     preamble_sent       ; Set the flag to say we have sent the preamble.
    return

; ---------------------------------------------------------------
; PROCESS: Send a Frame Length byte.
_TX_FRAMELEN:
    BUF_SEL txbuf, TXBUFLEN     ; Select the transmit buffer.
    BUF_GETEND                  ; Get the size of the buffer.
    movwf   txbyte              ; Place it in the transmit reigster.
    bsf     framelen_sent       ; Set flag to say that we have sent the frame length.
    movlw   0x00                ; Reset the buffer cursor.
    BUF_SETCURSOR               ; ...
    return

; ---------------------------------------------------------------
; PROCESS: Send a DATA byte.
_TX_DATABYTE:
    BUF_SEL txbuf, TXBUFLEN     ; Select the transmit buffer.
    BUF_GET                     ; Get a byte from the buffer.
    movwf   txbyte              ; Place it in the transmit reigster.
    BUF_SNEOF                   ; Was that the last byte in the buffer?
    bsf     frame_sent          ; YES - set the frame sent flag.
    return                      ; NO - return.

_FRAME_SENT:

 #ifdef BAPSA
    ; BASIC ADAPTIVE PREAMBLE SAMPLING ALGORITHM TEST:
    ; Switch to high traffic rate for 5 seconds.
    btfss   NTXEN                   ; Has the receiver been disabled yet?
    return                          ; NO - return.
    bsf     sleeping_con, dynamic   ; change the sleeping interval.
    bsf     preamble_con, dynamic   ; change the preamble length.
    call    SET_DYNAMIC_HTP         ; Just do it.
 #endif
    return


;***************************************************************
; SUBROUTINE:   FRAME_DECODER:
; 
; Description:  - Decodes frames. 
;               - Should be called every time a new bit is decoded.
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
FRAME_DECODER:

; -------------------------------------------------------------------------------
; DECISION: Check if we are currently in the process of receiving a frame.
; got_sof is clear: Run the SOF detect routine.
; got_sof is set: Run the frame receive routine.
    btfss   got_sof             ; Have we already detected the SOF pattern?
    bra     _GET_SOF            ; NO - look for the SOF pattern.

; -------------------------------------------------------------------------------
; DECISION: Check if we have received a byte.
; YES - process the byte.
; NO - return.
    btfss   rxbyte_full         ; Have we decoded a full byte?
    return                      ; NO - wait till we have.
    bcf     rxbyte_full         ; YES - Clear the flag ...

; -------------------------------------------------------------------------------
; DECISION: Check if we have received the frame length byte yet.
; got_framelen is clear: Run the Frame Length byte receive routine.
; got_framelen is set: Run the data byte receive routine.
    btfss   got_framelen        ; Have we received the frame length yet?
    bra     _GET_FRAMELEN       ; NO - receive the frame length byte.
    ;bra    _GET_DATABYTE       ; YES - receive a data byte.

; -------------------------------------------------------------------------------
; PROCESS: Read the data byte into the buffer.
_GET_DATABYTE:
    BUF_SEL rxbuf, RXBUFLEN     ; Select the receive buffer.
    movf    rxbyte, w           ; Load up the byte we just received.
    BUF_PUT                     ; Put it into the buffer. <BUFPUT in buffers.asm>

    BUF_SNFULL                  ; Is the buffer full?
    bra     _GOT_FRAME          ; YES: we have received as much of the frame as possible.

    dcfsnz  bytecount_l         ; Have we received all the bytes in the frame?
    bra     _GOT_FRAME          ; YES - we have received the entire frame.

    return                      ; NO - keep receiving bytes.

; -------------------------------------------------------------------------------
; PROCESS: Check if we have received the start of frame pattern.
_GET_SOF:

    ; Don't try and detect the SOF until we have received a new bit.
    movlw   0x00                ;
    cpfsgt  bitcount            ; Is bitcount > 0?
    return                      ; NO - return.
    clrf    bitcount            ; YES - clear bitcount and try to detect SOF.

    ; We clear the bitcount to do two things:
    ; 1) So we can determine if a new bit has arrived by testing if bitcount != zero.
    ; 2) It stops the bit counter getting to 8, therefore the physical layer
    ; cannot assert the clock_detect flag. If we have not yet detected the SOF,
    ; we only want to assert the clock detect if we detect a valid preamble.

    ; Try and detect the preamble byte.
    movlw   PREAMBLE_BYTE       ; Load up the preamble byte.
    cpfseq  shiftreg_l          ; Compare with the low byte of the shiftreg.
    return                      ; It wasn't equal, so just try again next time.
    bsf     clock_detect        ; We have preamble - set the clock detect flag.
    bsf     RXLED

    ; We have the preamble byte in the low byte of the shiftreg, 
    ; so try and detect the SOF (start of frame) byte.
    movlw   SOF_BYTE            ; Load up the start byte.
    cpfseq  shiftreg_h          ; Compare with the high byte of the shiftreg.
    return                      ; It wasn't equal, so just try again next time.
    
    ; We have a winner!
    bsf     got_sof             ; Set the Got Start Of Frame flag.
    clrf    bitcount            ; Reset the bit counter.
    bsf     TXLED               ; Turn on the RED LED to say we got the SOF.
    return


; -------------------------------------------------------------------------------
; PROCESS: Read the frame length, and store it so we know how many bytes to expect.
; Set a flag to say we know how many bytes to receive, and reset the buffer.
_GET_FRAMELEN:
    movff   rxbyte, bytecount_l ; Save the frame length
    bsf     got_framelen        ; Set the flag to say we know the frame length

    BUF_SEL     rxbuf, RXBUFLEN ; Select the Receive Buffer.
    BUF_CLEAR                   ; Reset the buffer.

    return
    
; -------------------------------------------------------------------------------
; We have received the entire frame - Verify the CRC.
_GOT_FRAME:

    bcf     TXLED

    ; Mark the end of the receive buffer
    ; (Buffer should still be selected).
    BUF_MARKEND

    ; Calculate the CRC on the receive buffer.
    call    CALC_CRC            ; Calculate the CRC on the buffer.

    ; Verify that the CRC for the packet is zero.
    tstfsz  crchigh             ; Is the high byte zero?
    bra     RESET_DECODER       ; NO - CRC is bad, reset the decoder.
    tstfsz  crclow              ; Is the low byte zero?
    bra     RESET_DECODER       ; NO - CRC is bad, reset the decoder.
        
; The CRC was good.
_CRC_GOOD:
    
 #ifdef BAPSA
    ; BASIC ADAPTIVE PREAMBLE SAMPLING ALGORITHM:
    ; Sample at the high traffic rate for 5 seconds, but don't change
    ; the preamble.
    bsf     sleeping_con, dynamic   ; Only change the sleeping interval
    call    SET_DYNAMIC_HTP         ; Just do it.
 #endif

    ; Remove the last two bytes (the CRC) from the buffer.
    BUF_GETEND                  ; Get the end pointer.
    sublw   d'2'                ; Shorten the buffer by 2
    BUF_MARKEND                 ; Set the end pointer.

    ; Go back to the beginning of the buffer
    movlw   0x00
    BUF_SETCURSOR
    
    ; Get the first byte (frame type) and check it
    BUF_GET
    sublw   CLOCKSYNC           ; Is it a clocksync packet?
    bz      _UPDATE_CLOCK       ; YES - sync the clock.
            
    ; NO - its just a normal packet.
    bsf     packet_received     ; Set the packet received flag.
    return                      ; Go pikachu!

_UPDATE_CLOCK:
    call    RTC_SET_CLOCK
    call    RESET_DECODER
    return

; Reset the decoder - start looking for a SOF again.
RESET_DECODER:
    clrf    rx_flags            ; Clear all flags.
    clrf    bytecount_l         ; Clear byte counter.
    clrf    bytecount_h         ; Clear byte counter.

    ; If we are aren't in a sampling window, turn off the receiver.
    btfss   mac_sampling        ; Are we sampling?
    call    RX_DISABLE          ; NO - disable the receiver.

    return                      ; Turns the node into a toadstool.
    
;*********************************************************************************
; END OF CODE

 END
