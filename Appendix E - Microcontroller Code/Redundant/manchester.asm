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
;*************************************************************************
;
; Description: 
;	- Subroutines to handle manchester coded data.
;	- Can work with or without start/stop bits.
;	- Receives on CCP1, Transmits on CCP2.
;
; Dependencies:
;	- buffers.asm
;	- macrolib.inc
;
; Resources Used:
;	- TMR3, CCP1, CCP2 (Exclusively)
;	- BUF0, BUF1 (Shared)
;
; Things you must do to use this module:
; 	- Connect your data receiver to the CCP1 pin.
; 	- Connect your data transmitter to the CCP2 pin.
; 	- Set variables in CONFIGURATION SECTION.
; 	- Call MANCH_INIT when initialising the device.
; 	- Call MANCHESTER_DECODE in your high priority ISR.
; 	- Call MANCHESTER_ENCODE in your high priority ISR.
;
; What you need to understand to work with this code:
; 	- CCP modules, in particular Capture and Compare modes.
; 	- Timer modules.
; 	- The interrupt system.
; 
; Notes:
;
; To transmit a data byte:
; 	- Put some byte(s) you want to transmit into BUF1
;	- Call START_TX
;
; Data will be received into BUF0 automatically.
;

;*************************************************************************
; INCLUDE FILES
 LIST P=18C452, F=INHX32	; directive to define processor and file format
 #include <P18C452.INC>		; processor specific variable definitions
 #include "macrolib.inc" 	; common macros

;*************************************************************************
; CONFIGURATION SECTION

; ---------------------------------------------------------------
; Chip Length: The number of periods of TMR3 that equals one chip.
 #define	CHIPTIME	0x0064			; = 100 d

; ---------------------------------------------------------------
; Preamble (Synchronization Header)
; These variables define the preamble that will be sent at the start of 
; every packet.
; The encoder will sent (PREAMBLE_LEN - 1) bytes of PREAMBLE_BYTE,
; followed by one byte of SOF_BYTE.
; The decoder will detect one byte of PREAMBLE_BYTE immediately
; followed by one byte of SOF_BYTE.
 #define	PREAMBLE_BYTE	0x55	; 10101010 LSB first
 #define	SOF_BYTE		0xFF	; 11111111 LSB first
 #define	PREAMBLE_LEN	d'08'	; 64 bits long

; ---------------------------------------------------------------
; MUX CCP2 to PORTC<RC1> or PORTB<RB3> :
 #define	CCP2PIN		PORTC, RC1
 #define	CCP2TRIS	TRISC, RC1
; Set the line in your main file:
; for PORTC<RC1>:	__CONFIG _CONFIG5, _CCP2MX_ON_5
; for PORTB<RC3>:	__CONFIG _CONFIG5, _CCP2MX_OFF_5

; ---------------------------------------------------------------
; /CD Pin - Where is the not carrier detect from the transciever connected?
 #define	NCD			PORTB, INT0

;******************************************************************************
; GLOBALLY AVAILABLE LABELS

 global MANCH_INIT			; Call this during initialisation.
 global MANCHESTER_DECODE	; Call this in your hi priority ISR.
 global	MANCHESTER_ENCODE	; Call this in your hi priority ISR.
 global	TX_PACKET			; Call this to start transmitting from BUF1.

;******************************************************************************
; IMPORTED LABELS

 ; From buffers.asm
 #include "buffers.inc"

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
; RAM ALLOCATION

 UDATA
; ----------------------------
; Shared Variables:
shiftreg_l			RES 1	; A 16 bit shift register.
shiftreg_h			RES 1	; ...
phy_flags			RES 1	; Status Flags.

; ------------------------------
; Manchester Decoding:
rx_flags				RES 1
 #define rx_done		rx_flags, 0		; Flag indicating we have received a whole packet.	
 #define rx_got_sof		rx_flags, 1		; Have we got the SOF pattern?
 #define rx_got_len		rx_flags, 2		; Have we got the packet length?

 #define rx_get_sample	CCP1CON, 3		; Have we been waiting to get a sample?
rx_bitcount				RES 1			; How many data bits do we have left to get?
rx_bytecount			RES 1			; How many data bytes do we have left to get?
rx_lastsamp_l			RES 1
rx_lastsamp_h			RES 1

; --------------------------------
; Manchester Encoding:
tx_flags				RES 1
 #define tx_done		tx_flags, 0		; When set, all bytes have been transmitted.
 #define tx_toggle		tx_flags, 1		; Do we have to do a toggle?
 #define tx_done_sof	tx_flags, 2		; Have we done a sof byte?
 #define tx_done_len	tx_flags, 3		; Have we transmitted the packet length?
 #define tx_done_databytes tx_flags, 4	; Have we done all the data bytes?
tx_bitcount				RES 1			; How many data bits do we have left to do?
tx_preamblecount		RES 1			; How many preamble bytes do we have left to do?

;******************************************************************************
; START OF CODE

 CODE


;***************************************************************
; SUBROUTINE: 	MANCH_INIT
; 
; Description:	Initalisation for all the Manchester encoding stuff.
; Precond'ns:	-
; Postcond'ns:	
; Regs Used:	WREG, CCP1 Registers, CCP2 Registers, TMR3 Registers
;***************************************************************
MANCH_INIT:
	; Configure Manchester Decoding
	clrf	rx_flags			; Clear all receiver flags.
	bsf		TRISC, CCP1			; Set CCP1 as an input.
	movlw	CCP_RISING			; Set CCP1 to capture every rising edge.
	movwf	CCP1CON				; ...
	bsf		PIE1, CCP1IE 		; Enable the CCP1 interrupt.
	bsf		IPR1, CCP1IP		; Set CCP1 interrupt to high priority.

	; Configure Manchester Encoding
	clrf	tx_flags			; Clear all encoder flags.
	bcf		CCP2TRIS			; Set the CCP2 pin to be an output.
	bcf		PIE2, CCP2IE 		; Disable the CCP2 interrupt.

	; Configure TIMER3
	movlw	b'01000101'			; Use TMR3 for all CCP, Use internal clock with 1:1 prescale,
	movwf	T3CON				; Enable 16-bit read/writes, start the timer.
	return

;***************************************************************
; SUBROUTINES: 	RX_ENABLE, RX_DISABLE, TX_ENABLE, TX_DISABLE
; 
; Description:	Enable/Disable the transmitter/receiver
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************

;---------------------------------------------------------------
; Enable the Receiver
RX_ENABLE:
	call	TX_DISABLE				; Disable the Transmitter.
	bcf		NRXEN					; Activate /RXEN.

	; TO DO 5ms delay

	return

;---------------------------------------------------------------
; Disable the Receiver
RX_ENABLE:
	bsf		NRXEN					; Deactivate /RXEN.
	return

;---------------------------------------------------------------
; Enable the Transmitter
TX_ENABLE:
	call	RX_DISABLE				; Disable the Receiver.
	bsf		PWUP					; Power up the VGA.
	bcf		NTXEN					; Activate /TXEN.

	; TO DO 5ms delay

	return

;---------------------------------------------------------------
; Disable the Receiver
RX_ENABLE:
	bsf		NTXEN					; Deactivate /TXEN.
	return

;***************************************************************
; SUBROUTINE: 	MANCHESTER_DECODE
; 
; Description:	Called under interrupt from CCP1.
;				Should be called in you high priority ISR.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
MANCHESTER_DECODE:

; Check if CCP1 actually caused the interrupt
	SERVICE_IRP	PIR1, CCP1IF, PIE1, CCP1IE		; <SERVICE_IRP in macrolib.inc>

; Check if we were waiting to sample, or waiting for an edge.
	btfsc	rx_get_sample	; Were we waiting to get a sample?
	bra		_SAMPLE			; YES - Take a sample.

; -------------------------------------------------------------------------------
; Section: _EDGESYNC
; Synchronise to the edge and detect clock error.
; The edge should occur within 0.5 chip times of the last sample. If the edge occurs in 1.5 chip times,
; Then we are synchronised onto the wrong edge. If the next edge occurs later than 1.5 chip
; times, then the clock has been lost. Let us say the next edge must occur within 1 chip time
; to allow for clock drift.
; Wait 1.5 chiptimes, which should put us in the perfect place to sample.
_EDGESYNC:
	; Add 1 chiptime to the last sample time
	movlw	low CHIPTIME			; lastsamp += CHIPTIME
	addwf	lastsamp_l				; ...
	movlw	high CHIPTIME			; ...
	addwfc	lastsamp_h				; ...
	; Compare to the current time
	movf	CCPR1L, W				; lastsamp -= CCPR1
	subwf	lastsamp_l				; ...
	movf	CCPR1H, W				; ...
	subwfb	lastsamp_h				; ...					
	; Check if CCPR1 > lastsamp + chiptime
	; lastsamp + chiptime - CCPR1 < 0
	bn		_CLK_ERR				; Was there a clock error?
	
	movlw	low CHIPTIME*3/2		; CCPR1 += CHIPTIME*3/2 
	addwf	CCPR1L					; ...
	movlw	high CHIPTIME*3/2		; ...
	addwfc	CCPR1H					; ...
	movlw	CCP_IRP					; Set CCP1 to generate IRP on match
	movwf	CCP1CON					; ...
	return							; Done.

; We have lost syncrhonisation, so we need to start looking for a preamble again.
_CLK_ERR:
	clrf	rx_flags				; Reset the decoder.
	return

; -------------------------------------------------------------------------------
; Section: _SAMPLE
; We should be in the middle of the first chip of a bit, the perfect place to sample.
; - Take a sample and shift it into the shift register.
; - Set CCP1 to capture the next edge.
; - Save the current CCP1 value so we can see if the clock is drifting.
_SAMPLE:
	btfss	PORTC, CCP1			; Is the sample high?
	bra		_SAMPLE_LOW
_SAMPLE_HIGH:
	bsf		STATUS, C			; Set the carry bit.
	movlw	CCP_FALLING			; Next edge will be a falling edge.
	bra		_SAMPLE_1			; Continue.
_SAMPLE_LOW:
	bcf		STATUS, C			; Clear the carry bit.
	movlw	CCP_RISING			; Next edge will be a rising edge.
_SAMPLE_1:
	movwf	CCP1CON				; Configure CCP1.
	rrcf	shiftreg_h			; Right shift the carry bit into the shiftreg.
	rrcf	shiftreg_l			; ...
	
	movff	CCPR1L, lastsamp_l	; Save the low byte.
	movff	CCPR1H, lastsamp_h	; Save the high byte.

; -------------------------------------------------------------------------------
; Check if we have received the start of frame pattern.
_RX_SOF:
	
	; Check if we need to be here.
	btfsc	rx_got_sof			; Have we already detected the SOF?
	bra		_RX_DATABYTE		; YES - check if we have received a byte.

	; Try and detect the preamble byte.
	movlw	PREAMBLE_BYTE		; Load up the preamble byte.
	cpfseq	shiftreg_l			; Compare with the low byte of the shiftreg.
	return						; It wasn't equal, so just try again next time.

	; We have the preamble byte in the low byte of the shiftreg, 
	; so try and detect the SOF (start of frame) byte.
	movlw	SOF_BYTE			; Load up the start byte.
	cpfseq	shiftreg_h			; Compare with the high byte of the shiftreg.
	return						; It wasn't equal, so just try again next time.
	
	; We have a winner!
	bsf		rx_got_sof			; Set the Got Start Of Frame flag.
	call	BUFSEL0				; Select Buffer 0. <BUFSEL0 in buffers.asm>
	call	BUFCLR				; Reset the buffer. <BUFCLR in buffers.asm>
	bra		_RX_RST_BITCOUNT	; Reset the bit counter and return.

; -------------------------------------------------------------------------------
; Check if we have received a data byte.
; If we have received 8 data bits, put the byte in rx buffer a.
_RX_BYTE:
	decfsz	rx_bitcount			; Do we have any data bits left to get?
	return						; Yes - byte not complete yet.

_RX_LENBYTE:
	btfsc	rx_got_len			; Have we received the packet length byte yet?
	bra		_RX_DATABYTE		; YES - This is a data byte.
	
	; We have received the packet length byte - store it so we know how many bytes
	; we are meant to be receiving.
	movff	shiftreg_h, rx_bytecount
	bra		_RX_RST_BITCOUNT	; Get ready to recieve the next byte.
	
	; We have received 8 bits - put the byte in the buffer.
	movf	shiftreg_h, w		; Load up the byte we just received.
	call	BUFSEL0				; Select Buffer 0 <BUFSEL0 in buffers.asm>
	call	BUFPUT				; Put it into the buffer. <BUFPUT in buffers.asm>

	; Check if we have received all the bytes.
	dcfsnz	rx_bytecount		; Have we received all the bytes?
	bra		_RX_DONE			; YES - we are done.	

_RX_RST_BITCOUNT:
	movlw	d'08'				; Reset the bit counter.
	movwf	rx_bitcount			; ...
	return						; Done.
	
; -------------------------------------------------------------------------------
; We have received all our bytes.
; Set a flag, disable the receiver until we are ready for reception nedt packet
_RX_BYTE:



;***************************************************************
; SUBROUTINE: 	CARRIER_DETECT
; 
; Description:	- Detects rising and falling edges on the CD pin.
;				- Falling edge:
;					- Enable the decoder interrupt.
;					- Start taking RSSI Measurements
;				- Rising edge:
;					- Disable the decoder interrupt.
;					- Terminate the packet being received.
;					- Pass control to the data link layer.
;				- Should be called under interrupt.
; Precond'ns:	-
; Postcond'ns:	-
; Regs Used:	-
;***************************************************************
CARRIER_DETECT:
; Check if an INT0 edge actually caused the interrupt.
	SERVICE_IRP	INTCON, INT0IF, INTCON, INT0IE	; <SERVICE_IRP in macrolib.inc>
	
; Check the polarity of the edge
	btfsc	INTCON2, INTEDG0
	bra		_CD_RISING_EDGE

;---------------------------------------------------------------
; Falling Edge.
; - Enable the decoder interrupt.
; - Start taking RSSI Measurements. ### TO DO ###
_CD_FALLING_EDGE:
	; Enable the decoder (CCP1) interrupt.
	movlw	CCP_RISING			; Set CCP1 to capture every rising edge.
	movwf	CCP1CON				; ...
	bsf		PIE1, CCP1IE		; Enable the interrupt.

;---------------------------------------------------------------
; Rising Edge.
; - Disable the decoder interrupt.
; - Terminate the packet being received. ### TO DO ###
; - Pass control to the data link layer. ### TO DO ###
_CD_RISING_EDGE:
	bcf		PIE1, CCP1IE		; Disable the interrupt.

;***************************************************************
; Subroutine: 	TX_PACKET
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
TX_PACKET:
; Stop TMR3, copy the value across to CCPR2 and add one so it 
; won't match until AFTER the timer is restarted.
	movf	TMR3L, W			; Load the low byte of TMR3
	addlw	(_TX2-TX_PACKET)/2	; Add enough so it won't IRP till we get to _TXLOOP
	movwf	CCPR2L				; Put it in CCPR2's low byte
	movlw	0x00				; Clear W
	addwfc	TMR3H, W			; Load the high byte of TMR3, and add the carry bit
	movwf	CCPR2H				; Put it in CCPR2's high byte

; Set up the list of things we need to do.
	clrf	tx_flags			; Clear flags.
	clrf	tx_bitcount			; Clear the bit counter.
	movlw	PREAMBLE_LEN - 1	; Do PREAMBLE_LEN - 1 preamble bytes.
	movwf	tx_preamblecount	; ...
	
; Set CCP2 to generate a software interrupt on every match.
	movlw	CCP_IRP				; Set CCP2 to interrupt on match but not change the CCP2 pin
	movwf	CCP2CON				; ...

; Enable CCP2 Interrupt and restart the timer.
	bcf		PIR2, CCP2IF		; Clear the int flag so it won't interrupt immediately
	bsf		PIE2, CCP2IE		; Enable the CCP2 interrupt.

; CCP2 = TMR3 when we get to here.
_TX2:

; Move the cursor to the beginning of Buffer1
	call	BUFSEL1				; Select Buffer1 <BUFSEL1 in buffers.asm>
	movlw	0x00				; 0 -> Cursor
	call	BUFCUR				; <BUFCUR in buffers.asm>

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
	SERVICE_IRP	PIR2, CCP2IF, PIE2, CCP2IE	; <SERVICE_IRP in macrolib.inc>

; Set CCP2 to match again in exactly one chip period
	ADDLF16	CHIPTIME, CCPR2L	; <ADDLF16 in macrolib.inc>

; ---------------------------------------------------------------
; Check what we have to do.
_TX_TODOLIST:
	btfsc	tx_toggle			; Do we have to toggle the output?
	bra		_TX_TOGGLE			; YES - toggle the output.

	tstfsz	tx_bitcount			; Do we have data bits left to transmit?
	bnz		_TX_NEXTBYTE		; YES - transmit the next data bit.

	tstfsz	tx_preamblecount	; Do we have preamble bytes left to transmit?
	bnz		_TX_PREAMBLEBYTE	; YES - transmit a preamble byte.

	btfss	tx_done_sof			; Do we have to transmit a SOF byte?
	bra		_TX_SOFBYTE			; YES - transmit a sof byte.

	btfss	tx_done_len			; Do we have to transmit the packet length byte?
	bra		_TX_LENBYTE			; YES - transmit the packet length byte.

	btfss	tx_done_databytes	; Do we have data bytes left to transmit?
	bra		_TX_DATABYTE		; YES - tranmit the next data byte.

	bra		tx_done				; All done.

; ---------------------------------------------------------------
; Toggle the output.
_TX_TOGGLE:
	bcf		tx_toggle			; Cross the toggle off the to do list.
;	movlw	CCP_TOGGLE			; Doesn't seem to work ???????
	btfsc	CCP2PIN				; Is the CCP2 pin set?
	movlw	CCP_CLEAR			; YES - clear it on the next CCP2 match
	btfss	CCP2PIN				; Is the CCP2 pin cleared?
	movlw	CCP_RAISE			; YES - raise it on the next CCP2 match
	movwf	CCP2CON				; ...
	return						; All done.

; ---------------------------------------------------------------
; Transmit a data bit.
_TX_DATABIT:
	decf	tx_bitcount			; Cross one data bit off the to do list.
	bsf		tx_toggle			; Put a toggle on the to do list.
	rrcf	shiftreg_l			; Pop the LSB off the end of the data byte.
	bc		_RAISE_CCP2			; The bit was high - raise the output.

_CLEAR_CCP2:					; The bit was low - clear the output.
	movlw	CCP_IRP				; Default action: Don't change CCP2 pin.
	btfsc	CCP2PIN				; Is the CCP2 pin cleared already?
	movlw	CCP_CLEAR			; NO - clear it on next CCP2 match (override default).
	movwf	CCP2CON				; ...
	return						; All done.

_RAISE_CCP2:					; The bit was high - raise the output.
	movlw	CCP_IRP				; Default action: Don't change CCP2 pin.
	btfss	CCP2PIN				; Is the CCP2 pin raised already?
	movlw	CCP_RAISE			; NO - raise it on next CCP2 match (override default).
	movwf	CCP2CON				; ...
	return						; All done.

; ---------------------------------------------------------------
; Transmit a preamble byte.
_TX_PREAMBLEBYTE:
	decf	tx_preamblecount	; Cross one preamble byte off the to do list.
	movlw	PREAMBLE_BYTE		; Get the preamble byte.
	bra		_TX_LOADBYTE		; Load the byte into the shift register.
	
; ---------------------------------------------------------------
; Transmit a SOF byte.
_TX_SOFBYTE:
	bsf		tx_done_sof			; Cross the SOF off the to do list.
	movlw	SOF_BYTE			; Get the SOF byte.
	bra		_TX_LOADBYTE		; Load the byte into the shift register.
	
; ---------------------------------------------------------------
; Transmit a packet length byte.
_TX_SOFBYTE:
	bsf		tx_done_len			; Cross the len byte off the to do list.
	call	BUFSEL_TXA			; Select the Tx "A" buffer <BUFSEL_TXA in buffers.asm>
	call	BUFLEN				; Get the buffer data length. <BUFLEN in buffers.asm>
	bra		_TX_LOADBYTE		; Load the byte into the shift register.

; ---------------------------------------------------------------
; Transmit a DATA byte.
_TX_DATABYTE:
	call	BUFSEL1				; Select Buffer1 <BUFSEL1 in buffers.asm>
	call	BUFGET				; Get a byte from the buffer <BUFGET in buffers.asm>
	btfsc	buf_eof				; Have we reached the end of the buffer?
	bsf		tx_done_databytes	; YES - all the data bytes have been processed.
	;bra	_TX_LOADBYTE		; Load the byte into the shift register.

; ---------------------------------------------------------------
; Load a byte into the shift register and start transmitting it.
_TX_LOADBYTE:
	movwf	shiftreg_l			; Put the byte in the shift register.
	movlw	0x08				; Put 8 data bits on the to do list.
	movwf	tx_bitcount			; ...
 	bra		_TX_DATABIT			; Go and start transmitting right away.

; ---------------------------------------------------------------
; Disable the transmitter and set the done flag.
_TX_DONE:
	bcf		PIE2, CCP2IE		; Disable CCP2 interrupts.
	bsf		tx_done				; Set the done flag.
	return						; All done.

;*********************************************************************************
; END OF CODE

 END
