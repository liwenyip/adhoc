;*************************************************************************
; MEDIA ACCESS LAYER
; 
; Version 1.00
; 24/08/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************
;
; Description: 
;	This code module implements the physical and data link layers.
; 	It provides transparent media access to the network layer.
;
; Functions:
;	- Controlling the transciever hardware.
;	- Manchester encoding / decoding / synchronisation.
;	- Delimiting and transmitting frames.
;	- Extracting frames from the received bit stream.
;	- Media contention (CSMA).
;	- Error checking (CRC).
;
; Dependencies:
;	- Buffers.asm
;
; Resources Used:
;	- CCP1 Used for decoding.
;	- CCP2 Used for encoding.
;	- TIMER3 Used as timebase for CCP1 and CCP2.
;	- TIMER1 Used for decoder watchdog timer.
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
 #define	CHIPTIME	d'100'		

; Power on Delay: The amount of time to wait between powering up the transmitter
; hardware and starting to transmit data. Defined in periods of TMR3.
 #define	TX_DELAY	d'5000'			; about 5ms

; Preamble (Synchronization Header)
; These variables define the preamble that will be sent at the start of 
; every packet.
; The encoder will sent (PREAMBLE_LEN) bytes of PREAMBLE_BYTE,
; followed by one byte of SOF_BYTE.
; The decoder will detect one byte of PREAMBLE_BYTE immediately
; followed by one byte of SOF_BYTE.
 #define	PREAMBLE_BYTE	0x55	; 10101010 LSB first
 #define	SOF_BYTE		0x00	; 00000000 LSB first

; CSMA: number of times to retry before giving up.
 #define	RETRY_LIMIT		d'10'	; retry 10 times.

; Transmit/Recieve buffer sizes
RXBUFLEN 		equ 0x40		; 64 bytes
TXBUFLEN 		equ 0x40		; 64 bytes

; MAC Constants - in TMR0 periods. TMR0 is running on a 1:256 prescaler.
; Amount of time to stay awake for.
WAKETIME		equ	d'39'			; 10 milliseconds
; Amount of time to sleep for during low traffic.
SLEEPTIME 		equ d'742'			; 190 milliseconds
; Amount of time to sleep for during high traffic.
SLEEPTIME2		equ d'156'			; 40 milliseconds

; Preamble length during low traffic.
PREAMBLELEN		equ	d'125'			; 125 bytes = 200ms @ 5000 baud
; Preamble length during high traffic.
PREAMBLELEN2	equ	d'32'			; 32 bytes = 50ms @ 5000 baud


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

 #define rx_get_sample	CCP1CON, 3		; Have we been waiting to get a sample?

; Frame types
NORMAL			equ 0x00
CLOCKSYNC		equ 0x01

;******************************************************************************
; IMPORTED VARIABLES

 extern		TMRF4, TMRL4, TMRH4

;******************************************************************************
; GLOBAL VARIABLES

; Transmit/Receive Buffers
.mediabuffers UDATA
rxbuf			res RXBUFLEN	; Receive Buffer	
txbuf			res TXBUFLEN	; Transmit Buffer	

 global rxbuf, txbuf

 UDATA_ACS
; Status flags.
media_status			res 1
 #define	packet_received		media_status, 0	; Indicates that we have received a packet.
 #define	packet_transmitted	media_status, 1	; Indicates that we are ready to send a packet.
 #define	retry_limit_reached	media_status, 2	; Indicates that we reached the retry limit.

;******************************************************************************
; LOCAL VARIABLES

; Shared Variables
bitcount		res 1		; Bit counter
bytecount		res 1		; Byte counter

shiftreg_l		res 1		; A 16 bit shift register.
shiftreg_h		res 1		; ...

; Transmitter Variables
txbyte			res 1
tx_flags		res 1
 #define tx_clock		tx_flags, 0	; Do we have to do a toggle?
 #define preamble_sent	tx_flags, 1	; Set when we have sent the start of frame.
 #define framelen_sent	tx_flags, 2	; Set when we have sent the frame length.
 #define frame_sent		tx_flags, 3	; Set when we have sent all the data bytes.
 #define txbyte_empty	tx_flags, 4 ; Set when the manchester encoder is ready for another byte.

preamble_len		res 1	; Preamble length

; Receiver Flags
rxbyte			res 1
rx_flags		res 1
 #define got_sof		rx_flags, 0	; Set when we have detected the start of frame.
 #define got_framelen	rx_flags, 1	; Set when we have received frame length.
 #define got_frame		rx_flags, 2 ; Set when we have received an entire frame.
 #define rxbyte_full	rx_flags, 3	; Set when the manchester decoder has decoded a byte.
 #define clock_detect	RXLED


; CSMA Variables
retry_count		res 1		; Count how many times we have tried to transmit.
timeout_mask	res 1		; Retry timeout mask.
 #define	csma_active		tx_flags, 7		; Set when the CSMA protcol is running.

; CRC Variables
crclow			res 1 		; CRC accumulator low byte.
crchigh			res 1 		; CRC accumulator high byte.
newdata			res 1 		; New data to feed into CRC (Destroyed)

; MAC Variables
mac_flags		res 1
	#define mac_windowopen	mac_flags, 0	; Set to indicate that the window is open.

sleeptime_l		res 1		; Sleep value in use.
sleeptime_h		res 1		; ...


delaycountl res 1
delaycounth res 1
;******************************************************************************
; IMPORTED SUBROUTINES

 #include "Buffers.inc"			; Software Buffers Header File

; From <RTC.asm>
 extern RTC_CLEAR_IRQS, RTC_GET_STATUS, RTC_GET_CLOCK, RTC_SET_CLOCK

;******************************************************************************
; EXPORTED SUBROUTINES

 global		MEDIA_INIT												; Initialisation
 global 	MEDIA_HIPRI_ISR, CSMA, WINDOW_TIMER, CHANGE_SLEEPTIME		; ISR's
 global		RX_DISABLE, TX_DISABLE, RX_ENABLE, TX_ENABLE, TX_PACKET		; User subroutines




;******************************************************************************
; START OF CODE

 CODE

 global MEDIA_TEST
MEDIA_TEST:
	

;	bcf		PIE1, TMR1IE

;	call	RX_ENABLE
	
	call	TX_SYNC_FRAME	


blah:
	bra	blah

delay:
	decfsz delaycountl
	bra delay
	decfsz delaycounth
	bra delay

	; Create a test packet
	BUF_SEL	txbuf, 0x40
	BUF_CLEAR
	movlw	0x00
	BUF_PUT
	movlw	0x11
	BUF_PUT
	movlw	0x22
	BUF_PUT
	movlw	0x33
	BUF_PUT
	movlw	0x44
	BUF_PUT
	movlw	0x55
	BUF_PUT
	movlw	0x66
	BUF_PUT
	movlw	0x77
	BUF_PUT
	movlw	0x88
	BUF_PUT
	movlw	0x99
	BUF_PUT
	movlw	0xAA
	BUF_PUT
	movlw	0xBB
	BUF_PUT
	movlw	0xCC
	BUF_PUT
	movlw	0xDD
	BUF_PUT
	movlw	0xEE
	BUF_PUT
	movlw	0xFF
	BUF_PUT
	BUF_MARKEND

	call CALC_CRC
	movf	crclow, W		; Load the crc low byte...
	BUF_PUT					; And append it to the buffer. <BUFPUT in buffers.asm>
	movf	crchigh, W		; Load the crc high byte...
	BUF_PUT					; And append it to the buffer. <BUFPUT in buffers.asm>
	BUF_MARKEND

	call	TX_ENABLE

Loopy2:
	btfss	frame_sent
	bra		Loopy2
	reset
	

;***************************************************************
; SUBROUTINE: 	MEDIA_INIT
; 
; Description:	Initialisation for physical layer.
; Precond'ns:	-
; Postcond'ns:	
; Regs Used:	WREG, CCP1 Registers, CCP2 Registers, TMR3 Registers
;***************************************************************
MEDIA_INIT:

	; Clear all variables.
	clrf	media_flags
	clrf	rx_flags
	clrf	tx_flags
	clrf	bitcount
	clrf	retry_count
	clrf	shiftreg_l
	clrf	shiftreg_h
	movlw	0x01

	; Set Sleep interval and preamble length for low traffic
	call	SET_LOW_TRAFFIC
			
	; Configure Transceiver pins.
	CONFIG_TRANSCIEVER_PINS

	; Configure TIMER0 (Window Timer)
	movlw	b'10000111'			; Load config byte for TIMER0.
	;		 '1-------'			; Enable Timer0.
	;		 '-0------'			; Configure as 16-bit counter.
	;		 '--0-----'			; Use internal clock (Fosc/4).
	;		 '----0---'			; Use Prescaler.
	;		 '-----111'			; 1:256 Prescale.
	movwf	T0CON

	; Configure TIMER3 (Chiprate Generator)
	movlw	b'11000001'			; Load config byte for TIMER3.
	;		 '1-------'			; Enable 16-bit Read/Write.
	;		 '-1--X---'			; Use TIMER3 for both CCP modules.
	;		 '--00----'			; 1:1 Prescale.
	;		 '------0-'			; Use internal clock (Fosc/4)
	;		 '-------1'			; Turn Timer3 on
	movwf	T3CON				
	
	; Set up interrupts.
	bcf		PIE1, CCP1IE 		; Disable the CCP1 interrupt.
	bcf		PIE2, CCP2IE 		; Disable the CCP2 interrupt.
	bcf		PIE1, TMR1IE		; Disable the TMR1 interrupt.
	bcf		PIE2, TMR3IE		; Disable the TMR3 interrupt.

	bsf		IPR1, CCP1IP		; CCP1 (Receive) is high priority.
	bsf		IPR2, CCP2IP		; CCP2 (Transmit) is high priority.
	bcf		INTCON2, TMR0IP		; TMR0 (Window Tmr) is low priority.

	; Make sure the TIMER0 Interrupt runs straight away to initialise TMR0
	bsf		INTCON, TMR0IF		; SET!!! the TIMER0 Interrupt flag.
	bsf		INTCON, TMR0IE		; Enable the TIMER0 Interrupt.

	return


;***************************************************************
; SUBROUTINE: 	MEDIA_ISR
; 
; Description:	High priority ISR for the media access layer.
; Precond'ns:	-
; Postcond'ns:	
; Regs Used:	
;***************************************************************
MEDIA_HIPRI_ISR:

; Check if CCP1 (Receive Clock) caused an interrupt.
_CCP1_ISR:
	btfss	PIR1, CCP1IF		; Is the interrupt flag set?
	bra		_CCP2_ISR			; NO - skip to checking CCP2.
	bcf		PIR1, CCP1IF		; YES - clear it.

	btfss	PIE1, CCP1IE		; Was the interrupt enabled?
	bra		_CCP2_ISR			; NO - skip to checking CCP2.

	call	MANCHESTER_DECODER	; YES - Call the manchester decoding routine.
	call	FRAME_DECODER		; Call the Frame decoding routine.
	return

; Check if CCP2 (Transmit Clock / Receive Clock Watchdog) caused an interrupt.
_CCP2_ISR:
	btfss	PIR2, CCP2IF		; Is the interrupt flag set?
	return						; NO - return.
	bcf		PIR2, CCP2IF		; YES - clear it.
	btfss	PIE2, CCP2IE		; Was the interrupt enabled?
	return						; NO - return.

	; Check if we should run the encoder routine or the watchdog routine.
	btfsc	NTXEN				; Are we in transmit mode?
	bra		CLOCK_WATCHDOG		; NO - run the clock watchdog.

	call	MANCHESTER_ENCODER	; YES - Call the manchester encoding routine.
	call	FRAME_ENCODER		; Call the frame encoding routine.
	return








; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                           HARDWARE CONTROL ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################

;***************************************************************
; SUBROUTINE: 	RX_ENABLE
; 
; Description:	Enable the receiver.
; Precond'ns:	
; Postcond'ns:	- Transmitter is disabled.
;				- /RXEN is activated (set low).
;				- CCP1 is set to interrupt in 5ms.
;				- CCP1 Interrupt is enabled (PIE1<CCP1IE> is set).
;				- TMR1 Interrupt is enabled (PIE1<TMR1IE> is set).
; Regs Used:	
;***************************************************************
RX_ENABLE:
	call	TX_DISABLE				; Disable the Transmitter.
	bcf		NRXEN					; Activate /RXEN.		

	; Reset the state of the decoder.
	bcf		packet_received			; Clear the packet_received flag
	clrf	rx_flags				; Clear all flags.
	clrf	bitcount				; Reset the bit counter.

	; Set CCP1 to match in about 5ms.
	; This gives the transmitter enough time to power up.
	bcf		PIE1, CCP1IE			; Disable the interrupt to stop it going off.

	movlw	low TX_DELAY			; Load the low byte of 5ms.
	addwf	TMR3L, W				; Add low byte of TMR3 value.
	movwf	CCPR1L					; Store it in low byte of CCPR1

	movlw	high TX_DELAY			; Load the high byte of 5ms.
	addwfc	TMR3H, W				; Add the high byte of TMR3 value with carry.
	movwf	CCPR1H					; Store it in high byte of CCPR1

	; Enable CCP1 Interrupt.
	movlw	CCP_IRP					; Set CCP1 to generate a software interrupt on match.
	movwf	CCP1CON
	bcf		PIR1, CCP1IF			; Clear the flag so we don't get a false interrupt.
	bsf		PIE1, CCP1IE			; Enable the interrupt.

	; Enable CCP2 Interrupt.
	movlw	CCP_IRP					; Set CCP2 to generate a software interrupt on match.
	movwf	CCP2CON					; ...
	bcf		PIR2, CCP2IF			; Clear the flag so we don't get a false interrupt.
	bsf		PIE2, CCP2IE			; Enable the CCP2 interrupt.
	
	return

;***************************************************************
; SUBROUTINES: 	TX_ENABLE
; 
; Description:	Enable the transmitter.
; Precond'ns:	
; Postcond'ns:	- Receiver is disabled.
;				- /TXEN is activated (set low).
;				- VGA is powered up.
;				- CCP2 is set to interrupt in 5ms.
;				- CCP2 Interrupt is enabled (PIE2<CCP2IE> is set).
; Regs Used:	
;***************************************************************
TX_ENABLE:
	; Enable the transmitter hardware.
	call	RX_DISABLE				; Disable the Receiver.
	bsf		PWUP					; Power up the VGA.
	bcf		NTXEN					; Activate /TXEN.

	; Reset the state of the encoder.
	clrf	tx_flags
	clrf	bytecount
	clrf	bitcount

	; Set CCP2 to match in about 5ms.
	; This gives the transmitter enough time to power up.
	bcf		PIE2, CCP2IE			; Disable the interrupt to stop it going off.

	movlw	low TX_DELAY			; Load the low byte of 5ms.
	addwf	TMR3L, W				; Add low byte of TMR3 value.
	movwf	CCPR2L					; Store it in low byte of CCPR2.

	movlw	high TX_DELAY			; Load the high byte of 5ms.
	addwfc	TMR3H, W				; Add the high byte of TMR3 value with carry.
	movwf	CCPR2H					; Store it in high byte of CCPR2.
	
	; Enable CCP2 Interrupt.
	movlw	CCP_TOGGLE				; Set CCP2 to toggle TxD on a match
	movwf	CCP2CON					; ...
	bcf		PIR2, CCP2IF			; Clear the flag so we don't get a false interrupt.
	bsf		PIE2, CCP2IE			; Enable the CCP2 interrupt.
	
	return

;***************************************************************
; SUBROUTINES: 	RX_DISABLE
; 
; Description:	Disable the receiver.
; Precond'ns:	
; Postcond'ns:	- TMR1 Interrupt is disabled.
;				- CCP1 Interrupt is disabled.
;				- /TXEN is deactivated (set high).
; Regs Used:	
;***************************************************************
RX_DISABLE:
	bcf		PIE1, CCP1IE			; Disable CCP1 Interrupt.
	bsf		NRXEN					; Deactivate /RXEN.
	return

;***************************************************************
; SUBROUTINES: 	TX_DISABLE
; 
; Description:	Disable the transmitter.
; Precond'ns:	
; Postcond'ns:	- CCP2 Interrupt is disabled.
;				- VGA is powered down.
;				- /TXEN is deactivated (set high).
; Regs Used:	
;***************************************************************
TX_DISABLE:
	bcf		PIE2, CCP2IE			; Disable CCP2 Interrupt.
	movlw	CCP_IRP					; ...
	movwf	CCP2CON					; ...

	bcf		PWUP					; Power down the VGA.
	bsf		NTXEN					; Deactivate /TXEN.
	return






; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                         MAC TIMING ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################



;***************************************************************
; SUBROUTINE: 	WINDOW_TIMER (ISR)
; 
; Description:	Use TIMER1 to open and close the window.
; Precond'ns:
; Postcond'ns:	
; Regs Used:	
;***************************************************************
WINDOW_TIMER:

; Service the TMR0 Interrupt.
	SERVICE_IRP INTCON, TMR0IF, INTCON, TMR0IE

; If CSMA is running, we don't want to touch the transceiver.
	btfsc	csma_active			; Is CSMA running?
	bra		_DONTCLOSE_WINDOW	; YES - don't change anything

; If we are transmitting, we don't want to touch the transceiver.
	btfss	NTXEN				; Is the transmitter active (low)?
	bra		_DONTCLOSE_WINDOW	; YES - don't change anything

; Check if the window is open.
	btfsc	mac_windowopen		; Is the window open?
	bra		_CLOSE_WINDOW		; YES - close it.

; Open the window.
_OPEN_WINDOW:
	call	RX_ENABLE			; Enable the receiver.
	bsf		mac_windowopen		; Set the window open flag.

	; Set TMR0 to overflow in WAKETIME.
	; When TMR0 overflows the window will be closed again.
	bcf		T0CON, TMR0ON			; Stop the timer
	nop								; Wait for it to stop.
	movlw	high (0xFFFF - WAKETIME)
	movwf	TMR0H
	movlw	low (0xFFFF - WAKETIME)
	movwf	TMR0L
	bsf		T0CON, TMR0ON			; Start the timer

	return
	
; Close the window
_CLOSE_WINDOW:
	btfsc	clock_detect		; Do we have a valid signal?
	bra		_DONTCLOSE_WINDOW	; YES - don't close the window.
	call	RX_DISABLE			; NO - Sleep the receiver.
	bcf		mac_windowopen		; Clear the window open flag.
	
	; Set TMR0 to overflow in SLEEPTIME.
	; When TMR0 overflows, the window will be opened again.
	bcf		T0CON, TMR0ON			; Stop the timer
	nop								; Wait for it to stop.
	movff	sleeptime_h, TMR0H
	movff	sleeptime_l, TMR0L
	bsf		T0CON, TMR0ON			; Start the timer

	return

; Set TMR0 to overflow in SLEEPTIME + WAKETIME.
; When TMR0 overflows, we will try to close the window again.
_DONTCLOSE_WINDOW:
	bcf		T0CON, TMR0ON			; Stop the timer
	nop								; Wait for it to stop.
	movlw	high WAKETIME
	subwf	sleeptime_h, w
	movwf	TMR0H
	movlw	low WAKETIME
	subwf	sleeptime_l, w
	movwf	TMR0L
	bsf		T0CON, TMR0ON			; Start the timer

	return

	
;***************************************************************
; SUBROUTINE: 	CHANGE_SLEEPTIME (ISR)
; 
; Description:	Switch between high and low traffic modes.
; Precond'ns:
; Postcond'ns:	
; Regs Used:	
;***************************************************************
CHANGE_SLEEPTIME:
	
	; Service the INT1 Interrupt.
	SERVICE_IRP INTCON3, INT1IF, INTCON3, INT1IE

	; Get the status register and stack it.
	call	RTC_GET_STATUS		
	movwf	PREINC2				

	; Clear the RTC IRQ flags
	call	RTC_CLEAR_IRQS

	; Work out which alarm caused the interrupt.	
	movf	POSTDEC2, W			; Get the status register off the stack.
	btfsc	WREG, 0				; Was it Alarm 0?
	bra 	SET_LOW_TRAFFIC		; It was Alarm 0 - Set low traffic mode.
	btfsc	WREG, 1				; Was it Alarm 1?
	bra 	SET_HIGH_TRAFFIC	; It was Alarm 1 - Set high traffic mode.
	return	; Should't ever get here.

SET_LOW_TRAFFIC:
	; Set the sleep interval for low traffic.
	movlw	low (0xFFFF - SLEEPTIME)
	movwf	sleeptime_l
	movlw	high (0xFFFF - SLEEPTIME)
	movwf	sleeptime_h
	
	; Set the preamble length for low traffic.
	movlw	PREAMBLELEN
	movwf	preamble_len

	; return
	return

SET_HIGH_TRAFFIC:
	; Set the sleep interval for high traffic.
	movlw	low (0xFFFF - SLEEPTIME2)
	movwf	sleeptime_l
	movlw	high (0xFFFF - SLEEPTIME2)
	movwf	sleeptime_h
	
	; Set the preamble length for high traffic.
	movlw	PREAMBLELEN2
	movwf	preamble_len

	; return
	return
	
;***************************************************************
; SUBROUTINE: 	TX_SYNC_FRAME:
; 
; Description:	Transmit a clock sync frame.
; Precond'ns:
; Postcond'ns:	
; Regs Used:	
;***************************************************************
TX_SYNC_FRAME:
	
	; Select the TX Buffer and clear it.
	BUF_SEL txbuf, TXBUFLEN		; Select the transmit buffer.
	BUF_CLEAR					; Clear it.

	; Put the frame type into the buffer
	movlw	CLOCKSYNC
	BUF_PUT

	; Load the current clock value into the buffer
	call	RTC_GET_CLOCK

	; Transmit the packet
	call	TX_PACKET

	; Set low traffic mode (make sure our preamble is long enough for all nodes)
	call	SET_LOW_TRAFFIC

	return

	




; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                         MEDIA ACCESS CONTROL ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################


;***************************************************************
; SUBROUTINE: 	TX_PACKET
; 
; Description:	Transmit a packet.
; Precond'ns:	An outgoing packet has been placed in Buffer 1.
; Postcond'ns:	CRC is appended to buffer and packet transmission is initiated.
; Regs Used:	
;***************************************************************
TX_PACKET:

	; Calculate the CRC on the packet.
	BUF_SEL txbuf, TXBUFLEN ; Select the transmit buffer
	call	CALC_CRC		; Calculate the CRC on the buffer.

	movf	crclow, W		; Load the crc low byte...
	BUF_PUT					; And append it to the buffer. <BUFPUT in buffers.asm>
	movf	crchigh, W		; Load the crc high byte...
	BUF_PUT					; And append it to the buffer. <BUFPUT in buffers.asm>
	BUF_MARKEND

_TX_PACKET_TEST:
	; Reset the retry count and timeout mask and run the CSMA routine.
	clrf	retry_count 	; Clear the retry counter.
	clrf	timeout_mask	; Clear the retry timeout mask.
	bsf		csma_active		; Set the flag that lets the CSMA subroutine run.
	clrf	TMRF4			; Stop Timer4
	return

	
;***************************************************************
; SUBROUTINE: 	CSMA
; 	
; Description:	- Checks if the medium is free.
;				- If the medium is free, transmission is initiated.
;				- If it is not free, a random timeout is set.
;				- If after the timeout the medium is still not free, 
;					the timeout is doubled and reset.
;
;				- This routine is designed to be called once from normal
;				  code, and subsequently from interrupt on TIMER2.
;
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
CSMA:

	; Only run this subroutine if CSMA is active, and Timer4 has expired.
	btfss	csma_active		; Is CSMA running?
	return					; NO - get outta here.
	btfsc	TMRF4, 0		; Is TIMER4 running?
	return					; YES - get outta here.

	; Make sure that the receiver is turned on so we can check if the
	; carrier is free.
	btfsc	NRXEN			; Is the receiver enabled?
	bra		_CSMA_RXEN		; NO - turn it on.

	; Check if the media is clear.
;	btfss	NCD				; Not used becase of noise problems.
	btfsc	clock_detect	; Is there a valid clock?
	bra		_CSMA_WAIT		; YES - set a timeout.
	
; ---------------------------------------------------------
; We are Clear To Send, initiate transmission.
_CSMA_CTS:

	; It is important that NTXEN is asserted before csma_active is deasserted, as it means
	; there is no opportunity for the traffic level mode to be changed.
	call	TX_ENABLE		; Enable the transmitter.
	bcf		csma_active		; We don't need to run CSMA any more.
	
	return	

; ---------------------------------------------------------
; Receiver is not turned on, turn it on and wait for it to settle.
_CSMA_RXEN:
	; Enable the receiver.
	call	RX_ENABLE			

	; Set a 10ms timeout (to wait for a valid CLOCK)
	movlw	0x00			; Load 0 to the high byte of the counter.
	movwf	TMRH4			; ...
	movlw	0x0A			; Load 10 to the low byte of the counter.
	movwf	TMRL4			; ...
	setf	TMRF4			; Start the timer.
	
	; Set a 1-2ms timeout (to wait for a valid CD/RSSI)
;	movlw	0x00			; Load 0 to the high byte of the counter.
;	movwf	TMRH4			; ...
;	movlw	0x02			; Load 2 to the low byte of the counter.
;	movwf	TMRL4			; ...
;	setf	TMRF4			; Start the timer.

	; TIMER2 could be ready to overflow any minute, so we will get a timeout
	; of at least 1ms and at most 2ms.

	return
			
; ---------------------------------------------------------
; Media is busy, set a random timeout with exponential backoff.
; Max wait Time = 2^retry_count milliseconds
_CSMA_WAIT:

	; Check how many times we have tried to transmit.
	movlw	RETRY_LIMIT		; Load the retry limit.
	cpfslt	retry_count		; Have we reached the retry limit?
	bra		_CSMA_GIVEUP	; YES - give up.

	; We want to set a random timeout, but we don't have enough resources
	; for a dedicated random number generator. However, we can use the contents
	; of the crc register as a pseudo random number.
	; Set the low byte, so the delay will be a multiple of 256 ms.
	setf	TMRL4				
	movff	crchigh, TMRH4

	; Truncate (mask) the number so that it is at most 256 * 2^retry_count - 1
	movf	timeout_mask, W
	andwf	TMRH4

	; Increment the retry counter and timeout mask.
	incf	retry_count			; Increment retry counter.
	bsf		STATUS, C			; Set the carry bit
	rlcf	timeout_mask		; Left shift the mask.

	; Start the timer.
	setf	TMRF4
	
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
;                                  ENCODING ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################

;***************************************************************
; SUBROUTINE: 	MANCHESTER_ENCODER
; 
; Description:	- Encodes manchester encoded bits from shiftreg_l.
;
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
MANCHESTER_ENCODER:

; ---------------------------------------------------------------
; DECISION: Check if we are encoding a clock chip or a data chip.
	btfsc	tx_clock			; Are we encoding a clock chip?
	bra		_TX_CLOCKCHIP		; Yes - encode a clock chip.

; ---------------------------------------------------------------
; DECISION: Check if we need to get another data byte.
	tstfsz	bitcount			; Does bitcount = 0?
	bra		_TX_DATACHIP		; NO - don't get another byte.

; ---------------------------------------------------------------
; DECISION: Check if we have another data byte to transmit.
	btfsc	txbyte_empty		; Is there a transmit byte?
	bra		TX_DISABLE			; NO - disable the transmitter.
	
; ---------------------------------------------------------------
; PROCESS: Fetch the next data byte.
	movff	txbyte, shiftreg_l	; Fetch the next byte from the buffer.
	movlw	d'08'				; Reset the bit counter.
	movwf	bitcount			; ...
	bsf		txbyte_empty		; Set a flag to say we are ready for the next byte.

; ---------------------------------------------------------------
; PROCESS: Encode a data chip on the next cycle.
_TX_DATACHIP:
	decf	bitcount			; Decrement the bit counter.
	rrcf	shiftreg_l			; Pop a the LSB off the end of the data byte.
	bc		_DATACHIP_HIGH		; Data bit was high.
;	bnc		_DATACHIP_LOW		; Data bit was low.

_DATACHIP_LOW:
	btfss	PORTC, CCP2			; Is TxD also low?
	bra		_TX_TOGGLE_IN_TWO	; Yes - we can skip the next cycle.
	bsf		tx_clock			; NO - set the clock flag ...
	bra		_TX_TOGGLE_IN_ONE	; ... and Toggle TxD in one chip period.

_DATACHIP_HIGH:
	btfsc	PORTC, CCP2			; Is TxD also high?
	bra		_TX_TOGGLE_IN_TWO	; Yes - we can skip the next cycle.
	bsf		tx_clock			; NO - set the clock flag ...
	bra		_TX_TOGGLE_IN_ONE	; ... and Toggle TxD in one chip period.

; ---------------------------------------------------------------
; PROCESS: Set CCP2 (Chiprate Generator) to toggle TxD in one chip period.
_TX_TOGGLE_IN_ONE:
	movlw	low CHIPTIME		; Load up the low byte of the chip time.
	addwf	CCPR2L				; Add it to the low byte of CCPR2
	movlw	high CHIPTIME		; Load up the high byte of the chip time.
	addwfc	CCPR2H				; Add it to the high byte of CCPR2 with carry.
	return

; ---------------------------------------------------------------
; PROCESS: Set CCP2 (Chiprate Generator) to toggle TxD in two chip periods.
_TX_TOGGLE_IN_TWO:
	movlw	low CHIPTIME*2		; Load up the low byte of the chip time * 2.
	addwf	CCPR2L				; Add it to the low byte of CCPR2
	movlw	high CHIPTIME*2		; Load up the high byte of the chip time * 2.
	addwfc	CCPR2H				; Add it to the high byte of CCPR2 with carry.
	return

; ---------------------------------------------------------------
; PROCESS: Encode a clock chip.
_TX_CLOCKCHIP:

	; Clear the clock flag.
	bcf		tx_clock			

	; Toggle TxD in one chip period.
	movlw	low CHIPTIME		; Load up the low byte of the chip time.
	addwf	CCPR2L				; Add it to the low byte of CCPR2
	movlw	high CHIPTIME		; Load up the high byte of the chip time.
	addwfc	CCPR2H				; Add it to the high byte of CCPR2 with carry.

	return


;***************************************************************
; Subroutine: 	FRAME_ENCODER
; 
; Description:	Encodes Frames. Should be called each time another
;				bit is encoded.
;
; Precond'ns:	
;
; Postcond'ns:	
;
; Regs Used:	WREG, FSR0
;***************************************************************
FRAME_ENCODER:

; ---------------------------------------------------------------
; DECISION: Check if the manchester encoder is ready for the next byte,
; and if we have a byte to give.
	btfss	txbyte_empty		; Is the transmit buffer empty?
	return						; NO - return.
	btfsc	frame_sent			; Do we have any data to transmit?
	return						; NO - return.
	bcf		txbyte_empty		; YES - clear the flag and continue.

; ---------------------------------------------------------------
; DECISION: Check if we have transmitted the preamble and SOF yet.
	btfsc	framelen_sent		; Have we sent the frame length yet?
	bra		_TX_DATABYTE		; YES - transmit a data byte.
								; NO - continue.

; ---------------------------------------------------------------
; DECISION: Check if we have transmitted the preamble and SOF yet.
	btfsc	preamble_sent		; Have we sent the SOF yet?
	bra		_TX_FRAMELEN		; YES - transmit the frame length.
								; NO - continue.

; ---------------------------------------------------------------
; DECISION: Check if we have transmitted enough preamble bytes yet.
	movf	preamble_len, w		; Get the preamble length we are currently using.
	cpfslt	bytecount			; Have we exceeded it yet?
	bra		_TX_SOF				; YES - send a SOF byte.
;	bra		_TX_PREAMBLE		; NO - send a preamble byte.

; ---------------------------------------------------------------
; PROCESS: Send a preamble byte.
_TX_PREAMBLE:
	movlw	PREAMBLE_BYTE		; Copy the preamble byte ...
	movwf	txbyte				; ... to the transmit reigster.
	incf	bytecount			; Increment the counter.
	return

; ---------------------------------------------------------------
; PROCESS: Send a SOF byte.
_TX_SOF:
	movlw	SOF_BYTE			; Copy the SOF byte ...
	movwf	txbyte				; ... to the transmit reigster.
	bsf		preamble_sent		; Set the flag to say we have sent the preamble.
	return

; ---------------------------------------------------------------
; PROCESS: Send a Frame Length byte.
_TX_FRAMELEN:
	BUF_SEL	txbuf, TXBUFLEN		; Select the transmit buffer.
	BUF_GETEND					; Get the size of the buffer.
	movwf	txbyte				; Place it in the transmit reigster.
	bsf		framelen_sent		; Set the flag to say that we have sent the frame length.
	movlw	0x00				; Reset the buffer cursor.
	BUF_SETCURSOR				; ...
	return

; ---------------------------------------------------------------
; PROCESS: Send a DATA byte.
_TX_DATABYTE:
	BUF_SEL	txbuf, TXBUFLEN		; Select the transmit buffer.
	BUF_GET						; Get a byte from the buffer.
	movwf	txbyte				; Place it in the transmit reigster.
	BUF_SNEOF					; Was that the last byte in the buffer?
	bsf		frame_sent			; YES - set the frame sent flag.
	return						; NO - return.




; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                                  DECODING ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################

;***************************************************************
; SUBROUTINE: 	MANCHESTER_DECODER
; 
; Description:	- Decodes manchester encoded bits and shifts them
;					into shiftreg_h and shiftreg_l.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
MANCHESTER_DECODER:

; -------------------------------------------------------------------------------
; DECISION: Check if CCP1 was set to capture or compare mode.
; Compare mode (CCP1CON<3> is set) - Go to the sampling routine.
; Capture mode (CCP1CON<3> is clear) - Go to the clock detect routine.
	btfsc	rx_get_sample		; Were we waiting to get a sample?
	bra		_SAMPLE				; YES - Take a sample.
	;bra	_CLOCK_DETECT		; NO - Synchronise to the clock edge.

; -------------------------------------------------------------------------------
; PROCESS: Synchronise to the manchester clock (i.e. the edges in the middle of each data bit).
; Even if the edge we just captured is not the correct clock edge, it will sort itself
; out as soon as the data bit changes.
; a) Set CCP2 (Clock Watchdog) to match in 2.2 chip periods
; b) Set CCP1 (Chiprate Generator) to match in 1.5 chip periods
_CLOCK_DETECT:

	; a) Set CCP2 (Clock Watchdog) to match in 2.2 chip periods.
	; If don't detect a clock edge and run this routine ever two chip periods,
	; CCP2 will generate an interrupt, indicating we have lost the clock.
	movlw	low CHIPTIME*22/10	; Load the low byte of 2.2 chiptimes.
	addwf	CCPR1L, W			; Add the low byte of the current time.
	movwf	CCPR2L				; Store it to CCPR2.

	movlw	high CHIPTIME*22/10	; Load the high byte of 2.2 chiptimes.
	addwfc	CCPR1H, W			; Add the high byte of the current time with carry.
	movwf	CCPR2H				; Store it to CCPR2.

	; This is done in RX_ENABLE
	movlw	CCP_IRP				; Set CCP2 to generate a software interrupt on match.
	movwf	CCP2CON				; ...
	bcf		PIR2, CCP2IF		; Make sure we don't get a false interrupt after changing modes.

	; b) Set CCP1 (Chiprate Generator) to match in 1.5 chip periods.
	; CCP1 should generate an interrupt in the middle of the first chip of the
	; next bit... the perfect place to sample.
	movlw	low CHIPTIME*3/2	; Load low byte of 1.5 chiptimes.
	addwf	CCPR1L				; Add it to the low byte of CCPR1.

	movlw	high CHIPTIME*3/2	; Load the high byte of the chiptime.
	addwfc	CCPR1H				; Add it to the high byte of CCPR1 with carry.

	movlw	CCP_IRP				; Set CCP1 to generate a software interrupt on match.
	movwf	CCP1CON				; ...
	bcf		PIR1, CCP1IF		; Make sure we don't get a false interrupt after changing modes.

	return						; Done.

; -------------------------------------------------------------------------------
; PROCESS: Take a sample and place it into the shift register.
; a) Test the value of RxD.
;		High: set STATUS<C>, set CCP1 to capture the next falling edge.
;		Low: clear STATUS<C>, set CCP1 to capture the next rising edge.
; b) Rotate shiftreg_h and shiftreg_l right through the carry bit.
; c) Increment the bit counter.
;
; NB - RRCF shifts the carry bit into the register - this is why we are writing to STATUS<C>.
; Make sure any instructions do not change STATUS<C> until the RRCF is done.
_SAMPLE:
	btfss	PORTC, CCP1			; Is the TxD high or low?
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
	bcf		PIR2, CCP1IF		; Make sure we don't get a false interrupt after changing modes.
	rrcf	shiftreg_h			; Right shift the carry bit into the shiftreg.
	rrcf	shiftreg_l			; ...
	incf	bitcount			; Increment the bit counter to say we received a bit.

; -------------------------------------------------------------------------------
; DECISION: Check if we have received 8 bits.
; YES - Reset the bit counter, copy the byte into rxbyte, and set rxbyte_full.
; NO - increment the bit counter and return.
	movlw	d'07'				;
	cpfsgt	bitcount			; Is bitcount >= 8?
	return						; NO - return.
	
	; Process the byte we just received.
	clrf	bitcount			; Clear the bit counter.
	movff	shiftreg_h, rxbyte	; Copy the received byte into rxbyte.
	bsf		rxbyte_full			; Set a flag to say we have received a byte.
	bsf		clock_detect		; Set the clock detect flag.
	return						; Bugga orf.


;***************************************************************
; SUBROUTINE: 	FRAME_DECODER:
; 
; Description:	- Decodes frames. 
;				- Should be called every time a new bit is decoded.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
FRAME_DECODER:

; -------------------------------------------------------------------------------
; DECISION: Check if we are currently in the process of receiving a frame.
; got_sof is clear: Run the SOF detect routine.
; got_sof is set: Run the frame receive routine.
	btfss	got_sof				; Have we already detected the SOF pattern?
	bra		_GET_SOF			; NO - look for the SOF pattern.

; -------------------------------------------------------------------------------
; DECISION: Check if we have received a byte.
; YES - process the byte.
; NO - return.
	btfss	rxbyte_full			; Have we decoded a full byte?
	return						; NO - wait till we have.
	bcf		rxbyte_full			; YES - Clear the flag ...

; -------------------------------------------------------------------------------
; DECISION: Check if we have received the frame length byte yet.
; got_framelen is clear: Run the Frame Length byte receive routine.
; got_framelen is set: Run the data byte receive routine.
	btfss	got_framelen		; Have we received the frame length yet?
	bra		_GET_FRAMELEN		; NO - receive the frame length byte.
	;bra	_GET_DATABYTE		; YES - receive a data byte.

; -------------------------------------------------------------------------------
; PROCESS: Read the data byte into the buffer.
_GET_DATABYTE:
	BUF_SEL	rxbuf, RXBUFLEN		; Select the receive buffer.
	movf	rxbyte, w			; Load up the byte we just received.
	BUF_PUT						; Put it into the buffer. <BUFPUT in buffers.asm>

	BUF_SNFULL					; Is the buffer full?
	bra		_GOT_FRAME			; YES - we have received as much of the frame as possible.

	dcfsnz	bytecount			; Have we received all the bytes in the frame?
	bra		_GOT_FRAME			; YES - we have received the entire frame.

	return						; NO - keep receiving bytes.

; -------------------------------------------------------------------------------
; PROCESS: Check if we have received the start of frame pattern.
_GET_SOF:

	; Don't try and detect the SOF until we have received a new bit.
	movlw	0x00				;
	cpfsgt	bitcount			; Is bitcount > 0?
	return						; NO - return.
	clrf	bitcount			; YES - clear bitcount and try to detect SOF.

	; Try and detect the preamble byte.
	movlw	PREAMBLE_BYTE		; Load up the preamble byte.
	cpfseq	shiftreg_l			; Compare with the low byte of the shiftreg.
	return						; It wasn't equal, so just try again next time.
	bsf		clock_detect		; We have preamble - set the clock detect flag.

	; We have the preamble byte in the low byte of the shiftreg, 
	; so try and detect the SOF (start of frame) byte.
	movlw	SOF_BYTE			; Load up the start byte.
	cpfseq	shiftreg_h			; Compare with the high byte of the shiftreg.
	return						; It wasn't equal, so just try again next time.
	
	; We have a winner!
	bsf		got_sof				; Set the Got Start Of Frame flag.
	clrf	bitcount			; Reset the bit counter.
	bsf		TXLED 				; Turn on the RED LED to say we got the SOF.
	return


; -------------------------------------------------------------------------------
; PROCESS: Read the frame length, and store it so we know how many bytes to expect.
; Set a flag to say we know how many bytes to receive, and reset the buffer.
_GET_FRAMELEN:
	movff	rxbyte, bytecount		; Save the frame length
	bsf		got_framelen			; Set the flag to say we know the frame length

	BUF_SEL		rxbuf, RXBUFLEN		; Select the Receive Buffer.
	BUF_CLEAR						; Reset the buffer.

	return
	
; -------------------------------------------------------------------------------
; We have received the entire frame - Verify the CRC.
_GOT_FRAME:

	bcf		TXLED

	; Mark the end of the receive buffer
	; (Buffer should still be selected).
	BUF_MARKEND

	; Calculate the CRC on the receive buffer.
	call	CALC_CRC		; Calculate the CRC on the buffer.

	; Verify that the CRC for the packet is zero.
	tstfsz	crchigh			; Is the high byte zero?
	bra		_CRC_BAD		; NO - CRC is bad.
	tstfsz	crclow			; Is the low byte zero?
	bra		_CRC_BAD		; NO - CRC is bad.
		
; The CRC was good - check the frame type.
_CRC_GOOD:
	
	; Remove the last two bytes (the CRC) from the buffer.
	BUF_GETEND			; Get the end pointer.
	sublw 	d'2'		; Shorten the buffer by 2
	BUF_MARKEND			; Set the end pointer.

	; Go back to the beginning of the buffer
	movlw 	0x00
	BUF_SETCURSOR
	
	; Get the first byte (frame type) and check it
	BUF_GET
	sublw	CLOCKSYNC		; Is it a clocksync packet?
	bz		_UPDATE_CLOCK	; YES - sync the clock.
			
	; NO - its just a normal packet.
	bsf		packet_received	; Set the packet received flag.
	return					; Go pikachu!

; The CRC was bad - wait for a new frame.
_CRC_BAD:
	clrf	rx_flags		; Clear all flags.
	clrf	bitcount		; Reset the bit counter.
	return					; Turns the antenna into a mushroom.

_UPDATE_CLOCK:
	call	RTC_SET_CLOCK
	return

;***************************************************************
; Subroutine: 	CLOCK_WATCHDOG
; 
; Description:	Called when CCP2 matches in receive mode, indicating that
;				clock synchronisation has been lost in the receiver.
; Precond'ns:	
; Postcond'ns:	
;
; Regs Used:	
;***************************************************************
CLOCK_WATCHDOG:
	; We have lost the clock, so reset the decoder.
	clrf	rx_flags			; Clear all flags.
	clrf	bitcount			; Reset the bit counter.
	clrf	shiftreg_l			; Clear the shift registers
	clrf	shiftreg_h			; ...
	bcf		clock_detect		; Reset the clock detect flag.
	bcf		TXLED	

	return





; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################
;
;                            ERROR CHECKING ROUTINES
;
; #####################################################################################
; #####################################################################################
; #####################################################################################
; #####################################################################################

; --------------------------------------------------------------------------
; The following code was sourced from the internet by Steven Sloots (2004).
;

;**********************************************************************
;                                                                     *
; Sub_Routine:     CrcHDLC                                            *
;                                                                     *
;            Macro for ^16+^12+^5+1, "CITT CRC16" - HDLC, X.25        *
;                                                                     *
; Arguments: newData    - data byte (destroyed)                       *
;            crcLow     - CRC accumulator low byte                    *
;            crcHigh    - 	CRC accumulator high byte                 *
;                                                                     *
;**********************************************************************
;                                                                     *
; Description: Macros to implement byte at a time ^16+^12+^5+1,       *
;              "CITT CRC16" - HDLC, X.25 16 bit CRC generation        *
;                                                                     *
;**********************************************************************
;    File Version:  1                                                 *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
CRCHDLC macro
;       oldCRC ^= newData
;       Modified to  - newData ^= oldCRC
	movf    crclow,W
	xorwf   newdata,F

;       oldCRC  = (oldCRC >> 8) | (oldCRC << 8)
;       Modified to - oldCRC  = (oldCRC >> 8) | newdata << 8)
	movf    crchigh,W
	movwf   crclow
	movf    newdata,W
	movwf   crchigh

;       oldCRC ^= (oldCRC & 0xFF00) << 4
	swapf   crchigh,W
	andlw   0xF0
	xorwf   crchigh,F

;       oldCRC ^= oldCRC >> 12
	swapf   crchigh,W
	andlw   0x0F
	xorwf   crclow,F

;       oldCRC ^= (oldCRC & 0xFF00) >> 5
	swapf   crchigh,F
	bcf     STATUS,C
	btfsc   crchigh,0
	bsf     STATUS,C
	rrcf     crchigh,W
	andlw   0xF8
	xorwf   crclow,F
	rrcf     crchigh,W
	andlw   0x07
	swapf   crchigh,F
	xorwf   crchigh,F

 endm
;	return




;***************************************************************
; SUBROUTINE: 	CALC_CRC
; 	
; Description:	Calculate the CRC on a buffer
; Precond'ns:	A buffer has been selected using BUFSELx in <buffers.asm>.
; Postcond'ns:	crclow and crchigh contain the calculated crc.
; Regs Used:	
;***************************************************************
CALC_CRC:
	movlw	0x00			; Reset the buffer cursor
	BUF_SETCURSOR			; ...

	clrf	crclow			; Clear the CRC registers.
	clrf	crchigh			; ...

_CALC_CRC_LOOP:
	BUF_GET					; Get a data byte. 
	movwf	newdata			; data byte -> newdata.
	CRCHDLC					; Process the data byte.

	; Loop until we have processed all the data in the buffer.
	BUF_SNEOF				; Was that the last byte in the buffer?
	return					; NO - all done.
	bra		_CALC_CRC_LOOP	; YES - process the next data byte.
	





















;*********************************************************************************
; END OF CODE

 END
