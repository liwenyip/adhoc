;*************************************************************************
; MANCHESTER ENCODING/DECODING MODULE
; 
; Version 0.10
; 12/07/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
; Resources Requiring Exclusive Use:
; TMR3, FSR0, CCP2
;
; Things you must do to use this module:
; 1) Connect your data transmitter to the CCP2 pin.
; 2) Set variables in CONFIGURATION SECTION.
; 3) Call MANCH_INIT when initialising the device.
; 4) Call MANCHESTER_ENCODE during your interrupt service routine.
;
; To transmit a data byte:
; 1) Put some byte(s) you want to transmit into a buffer
; 2) Load the number of bytes you want to transmit into databytes_todo
; 3) Load the address of the buffer into FSR0
; 4) Call START_TX
;
; Notes:
;
; 1 packet = many bytes
; 1 byte = 8 bits
; 1 bit = 2 chips
;
; A High bit encodes to two chips: 10
; A Low bit encodes to two chips:  01
; 
; The manchester encoding and decoding routines share some variables and resources,
; as it is impossible to be transmitting and receiving at the same time (on my hardware).
; Shared Resources are TMR3, FSR0.
;
; What you need to understand to work with this code:
; CCP modules, in particular Capture and Compare modes.
; Timer modules.
; The interrupt system.
; 
; 
;
;*************************************************************************
; INCLUDE FILES
 LIST P=18C452, F=INHX32	;directive to define processor and file format
 #include <P18C452.INC>		;processor specific variable definitions
 #include "macrolib.inc" 	; common macros

;*************************************************************************
; CONFIGURATION SECTION

; ---------------------------------------------------------------
; Asynchronous Mode (Start and Stop Bits)
; Comment out this line if you do not want to use start/stop bits
 #define	ASYNC

; ---------------------------------------------------------------
; Chip Length: The number of periods of TMR3 that equals one chip.
 #define	CHIPTIME	0x0064			; = 100 d

; ---------------------------------------------------------------
; Preamble (Synchronization Header)
; These variables define the preamble that will be sent at the start of 
; every packet.
; The encoder will sent (PREAMBLE_LEN - 1) bytes of PREAMBLE_BYTE,
; followed by one byte of START_BYTE.
; The decoder will detect one byte of PREAMBLE_BYTE immediately
; followed by one byte of START_BYTE.
 #define	PREAMBLE_BYTE	0x55	; 10101010 LSB first
 #define	START_BYTE		0xFF	; 11111111 LSB first
 #define	PREAMBLE_LEN	d'08'	; 64 bits long

; ---------------------------------------------------------------
; MUX CCP2 to PORTC<RC1> or PORTB<RB3> :
 #define	CCP2PIN		PORTB, RB3
 #define	CCP2TRIS	TRISB, RB3
; Set the line in your main file:
; for PORTC<RC1>:	__CONFIG _CONFIG5, _CCP2MX_ON_5
; for PORTB<RC3>:	__CONFIG _CONFIG5, _CCP2MX_OFF_5

;******************************************************************************
; GLOBALLY AVAILABLE LABELS

 global MANCH_INIT			; Subroutine to initiate for manchester encoding

 global MANCHESTER_DECODE	

 global	START_TX			; Subroutine to start transmitting bytes
 global	MANCHESTER_ENCODE
 global tx_databytes_todo	; Variable to set the number of bytes to transmit

;*************************************************************************
; CONSTANTS

; CCP Compare Modes:
CCP_IRP			equ b'1010'		; Just generate an interrupt on match
CCP_RAISE		equ	b'1000'		; Raise the output on match
CCP_CLEAR		equ	b'1001'		; Clear the output on match
CCP_TOGGLE		equ	b'0010'		; Toggle the output on match - doesn't seem to work

; CCP Capture Modes:
CCP_FALLING		equ	b'0100'		; Capture every falling edge
CCP_RISING		equ b'0101'		; Capture every rising edge
CCP_RISING4		equ b'0110'		; Capture every 4th rising edge
CCP_RISING16	equ b'0111'		; Capture every 16th rising edge

;******************************************************************************
; VARIABLE DEFINITIONS

 UDATA

; ----------------------------
; Shared Variables:

buffer				RES 2	; A 16 bit buffer, just a place to put stuff.

; ------------------------------
; Manchester Decoding:

; Edge Synchronisation
rx_lastedge			RES 2	; CCPR1 value when the last edge was captured
rx_period			RES 2

; Status Flags
rx_flags			RES 1
 #define rx_sampledbit	rx_flags, 0

; List of things to get
 #define rx_get_preamble	rx_flags, 1	; Do we have to get the preamble?
 #define rx_get_startbit	rx_flags, 2	; Do we have to get a start bit?
 #define rx_get_stopbit		rx_flags, 3	; Do we have to get a stop bit?
 #define rx_get_startbyte	rx_flags, 4	; Do we have to get a start byte?
rx_databits_toget	RES 1	; How many data bits do we have left to get?
rx_databytes_toget	RES 1	; How many data bytes do we have left to get?

; --------------------------------
; Manchester Encoding:

; Status Flags
tx_flags			RES 1
 #define tx_done	tx_flags, 0		; When set, all bytes have been transmitted.

; List of things to do.
 #define tx_do_toggle		tx_flags, 1	; Do we have to do a toggle?
 #define tx_do_startbit		tx_flags, 2	; Do we have to do a start bit?
 #define tx_do_stopbit		tx_flags, 3	; Do we have to do a stop bit?
 #define tx_do_startbyte	tx_flags, 4	; Do we have to do a start byte?
tx_databits_todo		RES 1	; How many data bits do we have left to do?
tx_preamblebytes_todo	RES 1	; How many preamble bytes do we have left to do?
tx_databytes_todo		RES 1	; How many data bytes do we have left to do?









 CODE

;*************************************************************************
;*************************************************************************
; SHARED SUBROUTINES
;

;*************************************************************************
; Subroutine: 	MANCH_INIT - initialisation macro for the module
;
; Description: 	You know what this does
;
; Regs Used:	
;*************************************************************************
MANCH_INIT:
	; Configure CCP1 for Manchester Decoding
	bsf		TRISC, CCP1			; Set CCP1 as an input
	bsf		PIE1, CCP1IE 		; Enable the CCP1 interrupt
	movlw	CCP_RISING			; Set CCP1 to capture every rising edge
	movwf	CCP1CON				; ...

	; Configure CCP2 for Manchester Encoding
	bcf		CCP2TRIS			; Set the CCP2 pin to be an output
	bcf		PIE2, CCP2IE 		; Disable the CCP2 interrupt

	; Configure TIMER3
	movlw	b'01000101'			; Use TMR3 for all CCP, Use internal clock with 1:1 prescale,
	movwf	T3CON				; Enable 16-bit read/writes, start the timer.
	return






;***************************************************************
; Subroutine: 	MANCHESTER_DECODE
; 
; Description:	Called once every chip period under interrupt from CCP1.
;				Receives manchester encoded data
;
; Precond'ns:	
;
; Postcond'ns:	
;
; Regs Used:	WREG, FSR0
;***************************************************************
MANCHESTER_DECODE:

; Check if a CCP1 capture actually caused the interrupt
	SERVICE_IRP	PIR1, CCP1IF, PIE1, CCP1IE		; <SERVICE_IRP in macrolib.inc>

; ----------------------------------------------------------------------------
; See what we are meant to be doing.
; Assume here that CCP1CON will only ever be one of:
; CCP_IRP 		00001010	- waiting to sample
; CCP_RISING 	00000100	- waiting for edge (rising)
; CCP_FALLING	00000101	- waiting for edge (falling)
; So we can just test bit 3 to see if we were waiting 
; for an edge or waiting to sample. 
	btfsc	CCP1CON, 3
	bra		_SAMPLE		; Take a sample.
	bra		_EDGESYNC	; Synchronise to the edge.

; ----------------------------------------------------------------------------
; Section: _EDGESYNC
; We have just recieved an edge, which (we hope) is the middle edge i.e the 
; one between the first and second chips of a bit. Therefore, if we wait 
; 1.5 chip times, we should end up in the middle of the first chip of the
; next bit - the perfect place to sample. 
; If we are synchronised onto the middle edges, we should see an edge 
; every two chiptimes. We won't see any edge which occurs before 1.5 chip times,
; as we will be waiting to sample. Therefore, if the next edge takes more than 
; 2 chip times to occur, we know we are not synchronised onto the middle edges.
; (This will occur when the data bit changes from 0->1 or 1->0).
; We will check to make sure that the edges are captured no more than
; 2.1 chip times apart.
; CCPR1 - rx_lastedge = rx_period < 2.1 * CHIPTIME
_EDGESYNC:

; `````````````````````````````````````````````
; NOTES ON THE CODE WHICH IS COMMENTED OUT:
; I think we should be alright leaving out the chunk of code which resets the 
; decoder if we are not in sync, for two reasons:
; 1) If the decoder is out of sync, it will be in sync after the data bit changes,
; regardless of whether or not we detect this happening.
; 2) We don't really need to detect if we are out of sync during the data frame,
; as anything which would cause us to lose synchronisation would screw up the
; data anyway. 

	; Calculate CCPR1 - rx_lastedge -> rx_period
;	movf	rx_lastedge, w		; Sutract the low bytes.
;	subwf	CCPR1L, w			; ...
;	movwf	rx_period			; Store it.
;	movf	rx_lastedge + 1, w	; Subtract the high bytes with borrow.
;	subwfb	CCPR1H, w			; ...
;	movwf	rx_period + 1		; Store it.

	; Save the value of CCPR1
;	movff	CCPR1L,	rx_lastedge
;	movff	CCPR1H, rx_lastedge + 1

	; Compare high bytes
;	movlw	high (CHIPTIME*21/10)
;	cpfslt	rx_period + 1		; Is rx_period < 2.1*CHIPTIME?
;	bra		_R1					; NO - next question.
;	bra		_EDGESYNC_OK		; YES - sync was ok.
;_R1:
;	cpfsgt  rx_period + 1		; Is rx_period > 2.1*CHIPTIME?
;	bra 	_R2					; NO - next question.
;	bra		_EDGESYNC_BAD		; YES - sync was bad.
;_R2:	
	; High bytes are equal, so compare low bytes.
;	movlw	low (CHIPTIME*21/10)
;	cpfsgt  rx_period			; Is rx_period > 2.1*CHIPTIME?
;	bra 	_EDGESYNC_OK		; NO - sync was ok.
;	bra		_EDGESYNC_BAD		; YES - sync was bad.

; If we just detected that the previous edge was out of sync, then we must
; be in sync now. So reset the decoder, then carry on as normal.
;_EDGESYNC_BAD:
;	call	RX_RESET			; Reset sync flags

	
; Set CCP1 to match in 1.5 chip times <ADDLF16 in macrolib.inc>
_EDGESYNC_OK:
	ADDLF16	(CHIPTIME*3/2), CCPR1L
	movlw	CCP_IRP				; Set CCP1 to generate IRP on match
	movwf	CCP1CON				; ...
	return						; done.

; -------------------------------------------------------------------------------
; Section: _SAMPLE
; We should be in the middle of the first chip of a bit, the perfect place to sample.
; It's kinda like taking the boat out for a fishing trip, except it's much less fun
; and only takes a couple of microseconds.
_SAMPLE:
; Sample the bit and save it, in case it changes again before we finish processing.
; Determine what the polarity of the next edge will be.
	btfss	PORTC, CCP1			; Is the sample high?
	bra		_SAMPLE_LOW
_SAMPLE_HIGH:
	bsf		rx_sampledbit		; Set the sampledbit flag
	movlw	CCP_FALLING			; Next edge will be a falling edge.
	bra		_SAMPLE_1			; Continue.
_SAMPLE_LOW:
	bcf		rx_sampledbit		; Clear the sampledbit flag
	movlw	CCP_RISING			; Next edge will be a rising edge.
_SAMPLE_1:
	movwf	CCP1CON				; Lock it in Eddy...

; -------------------------------------------------------------------------------
; CHECK THE LIST OF THINGS TO GET
; This is like a shopping list, to keep track of what we have received,
; what we haven't received, and what we should be receiving next.
_RX_CHECK_TOGET:
	btfsc	rx_get_preamble		; Do we have to receive a preamble?
	bra		_RX_PREAMBLE		; YES - go and get it.
 #ifdef ASYNC					
	btfsc	rx_get_startbit		; Do we have to receive a start bit?
	bra		_RX_STARTBIT		; YES - go and get it.
 #endif
	tstfsz	rx_databits_toget	; Do we have to receive any data bits?
	bra		_RX_DATABIT			; YES - go and get it.
 #ifdef ASYNC					
	btfsc	rx_get_stopbit		; Do we have to receive a stop bit?
	bra		_RX_STOPBIT			; YES - go and get it.
 #endif

; -------------------------------------------------------------------------------
; Check if we gave received the preamble.
_RX_PREAMBLE;
	; Shift the bit in from the left, use all 16 bits of the buffer.
	rrcf	buffer + 1			; Right shift the high byte.
	rrcf	buffer				; Right shift the low byte  (with carry).
	btfsc	rx_sampledbit		; Is the bit high?
	bsf		buffer + 1, 7		; YES - Set the leftmost bit of the buffer.
	btfss	rx_sampledbit		; Is the bit low?
	bsf		buffer + 1, 7		; YES - Clear the leftmost bit of the buffer.

	; Try and detect the preamble byte.
	movlw	PREAMBLE_BYTE		; Load up the preamble byte.
	cpfseq	buffer				; Compare with the low (older) byte of the buffer.
	return						; It wasn't equal, so just try again next time.

	; We have the preamble byte in the high byte of the buffer, 
	; so try and detect the start byte
	movlw	START_BYTE			; Load up the start byte.
	cpfseq	buffer + 1			; Compare with the high (more recent) byte of the buffer.
	return						; It wasn't equal, so just try again next time.
	
	; We have a winner!
	bcf		rx_get_preamble		; Cross the preamble off the list of things to get.
	return						; All done.

; -------------------------------------------------------------------------------
; Check if we have received a start bit (low).
 #ifdef ASYNC
_RX_STARTBIT:
	bcf		rx_get_startbit		; Cross the start bit off the list of things to get.
	btfsc	rx_sampledbit		; Was the bit low?
	call	RX_RESET			; NO: we have a framing error. Reset the receiver.
	return						; All done.
 #endif


; -------------------------------------------------------------------------------
; Receive a data bit and shift it into the buffer from the left.
_RX_DATABIT:
	decf	rx_databits_toget	; Cross one data bit off the list of things to get.
	rrncf	buffer				; Right shift the register.
	btfsc	rx_sampledbit		; Is the bit high?
	bsf		buffer, 7			; YES - Set the leftmost bit of the buffer.
	btfss	rx_sampledbit		; Is the bit low?
	bcf		buffer, 7			; YES - Clear the leftmost bit of the buffer.
	return						; All done.

; -------------------------------------------------------------------------------
; Check if we have recieved a stop bit (high).
 #ifdef ASYNC
_RX_STOPBIT:
	bcf		rx_get_stopbit		; Cross the stop bit off the list of things to get.
	btfss	rx_sampledbit		; Was the bit high?
	call	RX_RESET			; NO: we have a framing error. Reset the receiver.
	return						; All done.
 #endif





;***************************************************************
; Subroutine: 	RX_RESET
; 
; Description:	Makes the receiver resynchronise.
;
; Regs Used:
;***************************************************************
RX_RESET:


















;***************************************************************
; Subroutine: 	START_TX
; 
; Description:	Transmit data byte(s). Initialises CCP2 to interrupt once 
;				every chip perdiod, which calls MANCHESTER_ENCODE.
;
; Precond'ns:	a) FSR0 must contain the address of the data buffer
;				b) bytes_todo must contain the number of bytes to transmit
; 
; Postcond'ns:	a) Data bytes are transmitted on CCP2 output pin.
;
; Regs Used:	WREG, CCP2R
;***************************************************************
START_TX:
; Stop TMR3, copy the value across to CCPR2 and add one so it 
; won't match until AFTER the timer is restarted.
	bcf		T3CON, TMR3ON		; Stop TMR3
	movf	TMR3L, W			; Load the low byte of TMR3
	addlw	0x01 + 0x02			; Add 1, plus it takes 2 cycles for TMR3 to stop
	movwf	CCPR2L				; Put it in CCPR2's low byte
	movlw	0x00				; Clear W
	addwfc	TMR3H, W			; Load the high byte of TMR3, and add the carry bit
	movwf	CCPR2H				; Put it in CCPR2's high byte

; Reset the state of the manchester encoder
	clrf	tx_databits_todo	; Clear all flags.
	movlw	PREAMBLE_LEN - 1	; Do PREAMBLE_LEN - 1 preamble bytes.
	movwf	tx_preamblebytes_todo
	bsf		tx_do_startbyte		; Do one start byte.
	
; Set CCP2 to generate a software interrupt on every match.
	movlw	CCP_IRP				; Set CCP2 to interrupt on match but not change the CCP2 pin
	movwf	CCP2CON				; ...

; Enable CCP2 Interrupt and restart the timer.
	bcf		PIR2, CCP2IF		; Clear the int flag so it won't interrupt immediately
	bsf		PIE2, CCP2IE		; Enable the CCP2 interrupt.
	bsf		T3CON, TMR3ON		; Start up TMR3 (takes 2 cycles to start)

; Once TMR3 = CCPR2, this will cause an interrupt,  
; and kick-start the transmit / manchester encoding process.

_TXLOOP:
; Wait for it to finish.
	btfss	tx_done				; Check if byte is done.
	goto	_TXLOOP				; Loop if it's not done.
	return						; Take an RDO

	
;***************************************************************
; Subroutine: 	MANCHESTER_ENCODE
; 
; Description:	Called once every chip period under interrupt from CCP2.
;				Transmits data using manchester encoding.
;
; Precond'ns:	a) See preconditions for START_TX.
;				b) todo list must be reset must be cleared before running for the first time.
;
; Postcond'ns:	a) One bit of manchester encoded data has been transmitted.
;
; Regs Used:	WREG, FSR0
;***************************************************************
MANCHESTER_ENCODE:

; Check if a CCP2 match actually caused the interrupt
	SERVICE_IRP	PIR2, CCP2IF, PIE2, CCP2IE		; <SERVICE_IRP in macrolib.inc>

; Set CCP2 to match again in exactly one chip period
	ADDLF16	CHIPTIME, CCPR2L				; <ADDLF16 in macrolib.inc>

; ---------------------------------------------------------------
; CHECK THE TO DO LIST
; This is sort of like a to do list, and we can only do one thing each chip period.
; Toggling the output takes priority over everything else.
; Then, start bits, data bits, stop bits.
_TX_CHECK_TODO:
	btfsc	tx_do_toggle		; Do we have to toggle the output?
	bra		_TX_TOGGLE			; YES - go and do it.
 #ifdef ASYNC					
	btfsc	tx_do_startbit		; Do we have to transmit a start bit?
	bra		_TX_STARTBIT		; YES - go and do it.
 #endif
	tstfsz	tx_databits_todo	; Do we have to transmit any data bits?
	bra		_TX_DATABIT			; YES - go and do it.
 #ifdef ASYNC					
	btfsc	tx_do_stopbit		; Do we have to transmit a stop bit?
	bra		_TX_STOPBIT			; YES - go and do it.
 #endif

; If we get to here it means we have finished transmitting the byte.
	tstfsz	tx_preamblebytes_todo	; Do we have to transmit any preamble byte?
	bra		_TX_PREAMBLEBYTE	; YES - go and do it.
	btfsc	tx_do_startbyte		; Do we have to transmit a start byte?
	bra		_TX_STARTBYTE		; YES - go and do it.
	tstfsz	tx_databytes_todo	; Do we have to transmit any data bytes?
	bra		_TX_DATABYTE		; YES - go and do it.

; If we get to here it means we have finished transmitting all the bytes.
; Disable the CCP2 interrupts and set the done flag.
_TX_DONE:
	bcf		PIE2, CCP2IE		; Disable CCP2 interrupts.
	bsf		tx_done				; Set the done flag.
	return						; All done.

; ---------------------------------------------------------------
; Toggle the output.
_TX_TOGGLE:
	call	TOGGLE_CCP2			; Toggle the output.
	bcf		tx_do_toggle		; Cross the toggle off the to do list.
	return						; All done.

; ---------------------------------------------------------------
; Transmit a start bit (low).
 #ifdef ASYNC
_TX_STARTBIT:
	call	CLEAR_CCP2			; Clear the output.
	bcf		tx_do_startbit		; Cross the start bit off the to do list.
	bsf		tx_do_toggle		; Put a toggle on the to do list.
	return						; All done.
 #endif

; ---------------------------------------------------------------
; Transmit a data bit.
_TX_DATABIT:
	btfsc	buffer, 0			; Is the data bit (bit 0) high?
	call	RAISE_CCP2			; YES - Raise the output.
	btfss	buffer, 0			; Is the data bit (bit 0) low?
	call	CLEAR_CCP2			; YES - Clear the output.
	rrncf	INDF0				; Rotate the bits so the next data bit is now bit 0.
	decf	tx_databits_todo	; Cross one data bit off the todo list.
	bsf		tx_do_toggle		; Put a toggle on the to do list.
	return						; All done.
		
; ---------------------------------------------------------------
; Transmit a stop bit (high).
 #ifdef ASYNC
_TX_STOPBIT:
	call	RAISE_CCP2			; Raise the output.
	bcf		tx_do_stopbit		; Cross the stop bit off the todo list.
	bsf		tx_do_toggle		; Put a toggle on the todo list.
	return						; All done.
 #endif

; ---------------------------------------------------------------
; Transmit a preamble/start byte.
; NOTE: We don't want to use start or stop bits in the preamble so
; they were not added to the to do list.
_TX_PREAMBLEBYTE:
	decf	tx_preamblebytes_todo; Cross one preamble byte off the to do list.
	movlw	PREAMBLE_BYTE		; Put the preamble byte in the buffer.
	movwf	buffer				; ...
	bra		_TX_PREAMBLE_1
_TX_STARTBYTE:
	bcf		tx_do_startbyte		; Cross the start byte off the to do list.
	movlw	START_BYTE			; Put the start byte in the buffer.
	movwf	buffer				; ...
_TX_PREAMBLE_1:
	movlw	0x08				; Put 8 data bits on the to do list.
	movwf	tx_databits_todo	; ...
	bra		_TX_DATABIT			; Go and start transmitting right away.

; ---------------------------------------------------------------
; Transmit a data byte.
_TX_DATABYTE:
	movff	POSTINC0, buffer	; Copy the data byte to the buffer, and increment the pointer.
	decf	tx_databytes_todo	; Cross one data byte off the to do list.
 #ifdef ASYNC
	bsf		tx_do_startbit		; Put a start bit on the to do list.
	bsf		tx_do_stopbit		; Put a stop bit on the to do list.
 #endif
	movlw	0x08				; Put 8 data bits on the to do list.
	movwf	tx_databits_todo	; ...
 	bra		_TX_CHECK_TODO		; Go and start transmitting right away.
	
;*********************************************************************************
; Subroutines:	RAISE_CCP2, CLEAR_CCP2, TOGGLE_CCP2
;
; Description: 	Raise, Clear, or Toggle CCP2 upon the next CCP2 match, without changing
; 				the CCP2 pin if it is already in the desired state. i.e. if CCP2 is 
;				high, calling RAISE_CCP2 will not lower CCP2 when it is called.
;
; Registers used:	WREG, PORTC<CCP2>, CCP2CON
;*********************************************************************************
RAISE_CCP2:
	movlw	CCP_IRP				; Default action: Don't change CCP2 pin.
	btfss	CCP2PIN				; Is the CCP2 pin raised already?
	movlw	CCP_RAISE			; NO - raise it on next CCP2 match (override default).
	movwf	CCP2CON				; ...
	return
CLEAR_CCP2:
	movlw	CCP_IRP				; Default action: Don't change CCP2 pin.
	btfsc	CCP2PIN				; Is the CCP2 pin cleared already?
	movlw	CCP_CLEAR			; NO - clear it on next CCP2 match (override default).
	movwf	CCP2CON				; ...
	return
TOGGLE_CCP2:
	btfsc	CCP2PIN				; Is the CCP2 pin set?
	movlw	CCP_CLEAR			; YES - clear it on the next CCP2 match
	btfss	CCP2PIN				; Is the CCP2 pin cleared?
	movlw	CCP_RAISE			; YES - raise it on the next CCP2 match
	movwf	CCP2CON				;
	return

;*********************************************************************************
; END OF CODE

 END
