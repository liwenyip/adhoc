
; ---------------------------------------------------------------
; Check which step we are up, and jump to the required routine using a 
; jump table.

_TX_JUMPTBL:
	movlw	tx_step				; Load the step we are up to
	addwf	pcl					; White PCL can't jump!
	bra		_LOAD_PREAMBLE		; Step 0: INIT: Load the preamble byte.
	bra		_SEND_PREAMBLE		; Step 1: Send a preamble byte.
	bra		_SEND_SOF			; Step 2: Send a SOF byte.
	bra		_SEND_SLOT_L		; Step 3: Send the SLOT number low byte.
	bra		_SEND_SLOT_H		; Step 4: Send the SLOT number high byte.
	bra		_SEND_PLDLEN		; Step 5: Send the Payload Length.
	bra		_SEND_PAYLOAD		; Step 6: Send the Payload.
	bra		_SEND_CRC_L			; Step 7: Send the Low CRC Byte
	bra		_SEND_CRC_H			; Step 8: Send the High CRC Byte
	bra		_END_FRAME			; Step 9: End the Frame and turn off the transmitter.
	return						; Step 10: Do nothing.

    ; Make sure the table doesn't cross a page boundary
	IF ((HIGH ($)) != (HIGH (_TX_JUMPTBL)))
	    ERROR "table1 CROSSES PAGE BOUNDARY!"
    ENDIF


; ---------------------------------------------------------------
; Step 0: INIT: Load the preamble byte and initialise the preamble counter.
_LOAD_PREAMBLE:	
	movlw	PREAMBLE_BYTE		; Load the preamble byte into the transmit register.
	movwf	txbyte				; ...
	movlw	PREAMBLE_LEN - 1	; Load the preamble length into the byte counter.
	movwf	bytecount			; ...
	incf	tx_step				; Set tx_step = 1

; ---------------------------------------------------------------
; Step 1: Send a preamble byte.
_SEND_PREAMBLE:
	decfsz	bytecount			; Is bytecount-- = 0?
	return						; NO - keep transmitting preamble bytes.
	incf	tx_step				; YES - Set tx_step = 2
	return

; ---------------------------------------------------------------
; Step 2: Send a SOF byte.
_SEND_SOF:
	movlw	SOF_BYTE			; Load the SOF byte into the transmit register.
	movwf	txbyte				; ...
	incf	tx_step				; Set tx_step = 3
	return

; ---------------------------------------------------------------
; Step 3: Send the SLOT number low byte.
_SEND_SLOT_L:
	movf	slotnum_l, W		; Load the slot num low byte into the tx register.
	movwf	txbyte				; ...
	incf	tx_step				; Set tx_step = 4
	return

; ---------------------------------------------------------------
; Step 4: Send the SLOT number high byte.
_SEND_SLOT_H:
	movf	slotnum_h, W		; Load the slot num high byte into the tx register.
	movwf	txbyte				; ...
	incf	tx_step				; Set tx_step = 5
	return

; ---------------------------------------------------------------
; Step 5: Send the Payload Length.
_SEND_PLDLEN:
	BUF_SEL	txbuf, TXBUFLEN		; Select the transmit buffer.
	BUF_GETEND					; Get the size of the buffer.
	movwf	txbyte				; Place it in the transmit reigster.
	movlw	0x00				; Reset the buffer cursor.
	BUF_SETCURSOR				; ...
	incf	tx_step				; Set tx_step = 6
	return

; ---------------------------------------------------------------
; Step 6: Send the Payload.
_SEND_PAYLOAD:
	BUF_SEL	txbuf, TXBUFLEN		; Select the transmit buffer.
	BUF_GET						; Get a byte from the buffer.
	movwf	txbyte				; Place it in the transmit reigster.
	BUF_SNEOF					; Was that the last byte in the buffer?
	incf	tx_step				; YES - Set tx_step = 7
	return						; NO - return.

; ---------------------------------------------------------------
; Step 7: Send the CRC LOW byte.
_SEND_CRC_L:
	movf	txcrclow			; Load the low crc byte

; ---------------------------------------------------------------
; Step 8: Send the CRC HIGH byte.


























; Get the value of TIMER3 when the overflow occured.
	
;	; Get both low bytes first, which puts the high bytes in the latch.	
;	movff	TMR1L, TMR1L_TMP
;	movff	TMR3L, TMR3L_TMP
;
;	; Get the high bytes.
;	movff	TMR1H, TMR1H_TMP
;	movff	TMR3H, TMR3H_TMP
;
;	; Add 2 to TMR1 TEMP, as it was read 2 cycles before TMR3
;	movlw	low d'02'
;	addwf	TMR1L_TMP
;	movlw	high d'02'
;	addwfc	TMR1H_TMP
;
;	; Do TMR3_TMP - TMR1_TMP ; this gives the value of TMR3 when TMR1 overflowed.
;	movf	TMR1L_TMP, W
;	subwf	TMR3L_TMP
;	movf	TMR1H_TMP, W
;	subwfb	TMR3H_TMP


















;***************************************************************
; SUBROUTINE: 	SLOT_TIMER (ISR) (Not used at the moment)
; 
; Description:	Set timer1 to overflow every 400us (1 slot time)
; Precond'ns:
; Postcond'ns:	
; Regs Used:	
;***************************************************************
SLOT_TIMER:

; To make it overflow in 100us, we will add 0xFFFF - 100 to TIMER1
_ST1:
	bcf		T1CON, TMR1ON			; Stop the timer
	nop								; Wait for it to stop.
	movlw	low (0xFFFF - SLOTLEN + (_ST2-_ST1)/2)
	addwf	TMR1L
	movlw	high (0xFFFF - SLOTLEN + (_ST2-_ST1)/2)
	addwfc	TMR1H
	bsf		T1CON, TMR1ON			; Start the timer
	nop								; Wait for it to start.
_ST2:	
	
	; Increment the slot counter.
	infsnz	slot_counter_l		; Increment the low byte.
	incf	slot_counter_h		; Increment the high byte if neccesary.

	; Do a software compare.
	movf	slot_compare_l, W	; Load the low compare byte.
	cpfseq	slot_counter_l		; Does it equal the low counter byte?
	return						; NO - ciao.
	movf	slot_compare_h, W	; Load the high compare byte.
	cpfseq	slot_counter_h		; Does it equal the high counter byte?
	return						; NO - au revior.

; -----------------------------------------------------------------
; Check what state we are currently in.
_MAC_CHECKSTATE:
	btfsc	mac_winclose		; Do we need to close the window?
	bra		_WIN_CLOSE			; YES - close it.

	btfsc	mac_winwarmup		; Do we need to warm up the receiver?
	bra		_WIN_WARMUP			; Yes - start warming up the receiver.

;	btfsc	mac_winstart		; Do we need to start a new window?
;	bra		_WIN_START			; Yes - start a new window.

	; Shouldn't ever get to here...
;	return

; -----------------------------------------------------------------
; We are ready to start a new window, so reset the slot counter.
_WIN_START:
	bcf		mac_winstart		; Clear the flag that got us here.
	clrf	slot_counter_l		; Clear the slot counter.
	clrf	slot_counter_h		; ...

	; Wait 128 slots, then close the window if there is no signal.
	movlw	low d'128'			; Load 128 to the compare register.
	movwf	slot_compare_l		; ...
	movlw	high d'128'			; ...
	movwf	slot_compare_h		; ...
	bsf		mac_winclose		; Close the window upon next match.

	return
	
; -----------------------------------------------------------------
; We are at the end of the window (Slot 128). If we have a valid signal, keep
; listening, otherwise turn off the receiver.
_WIN_CLOSE:

	bcf		mac_winclose		; Clear the flag that got us here.
	btfsc	clock_detect		; Do we have a valid signal?
	bra		_WIN_DONTCLOSE		; YES - don't close the window.
	call	RX_DISABLE			; NO - Sleep the receiver.
	
	; Warm up the receiver at slot 950
	movlw	low d'950'			; Load 950 to the compare register.
	movwf	slot_compare_l		; ...
	movlw	high d'950'			; ...
	movwf	slot_compare_h		; ...
	bsf		mac_winwarmup		; Start warming up the recever upon next match.

	return

_WIN_DONTCLOSE:
	movlw	low d'1000'			; Load 1000 to the compare register.
	movwf	slot_compare_l		; ...
	movlw	high d'1000'		; ...
	movwf	slot_compare_h		; ...
	bsf		mac_winstart		; Start a new window upon next match.


; -----------------------------------------------------------------
; We are nearly ready to start a new window (Slot 950), so warm up the receiver.
_WIN_WARMUP:
	bcf		mac_winwarmup		; Clear the flag that got us here.
	call	RX_ENABLE			; Enable the receiver.

	; Wait 50 Slots (5ms for 100us slots) , then start a new window.
	; This pus us at slot 1000.
	movlw	low d'1000'			; Load 1000 to the compare register.
	movwf	slot_compare_l		; ...
	movlw	high d'1000'		; ...
	movwf	slot_compare_h		; ...
	bsf		mac_winstart		; Start a new window upon next match.

	return
	


	





