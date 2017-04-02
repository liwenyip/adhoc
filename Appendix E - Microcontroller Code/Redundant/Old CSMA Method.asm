; ---------------------------------------------------------
; Media is busy, set a random timeout with exponential backoff.
_CSMA_WAIT:
	
	tstfsz	retry_count			; Is this the first time we have tried to transmit?
	bra		_CSMA_RETRY			; NO
	
_CSMA_FIRST_TRY:	
	; We want to set a random timeout, but we don't have enough resources
	; for a dedicated random number generator. However, we can use the crc
	; byte as a pseudo random number. 
	clrf	timeout_h			; Clear the high byte of the timeout.
	movf	crclow, W			; Get the low crc byte.
	movwf	timeout_l			; Stick in the low byte of the timeout.
	bra		_CSMA_LOAD_TIMEOUT	; Kick off the timer.
	
_CSMA_RETRY:
	; Check if we have reached the retry limit
	

	; Increment the timeout (multiply by two).
	bcf		STATUS, C			; Clear the Carry bit.
	rlcf	timeout_l			; Rotate left through carry.
	rlcf	timeout_h			; ""

_CSMA_LOAD_TIMEOUT:
	; Load the timer value and start it up.
	movff	timeout_l, TMRL4	; Set the timeout period on TMR4.
	movff	timeout_h, TMRH4	; ...
	setf	TMRF4				; Start the timer.
				; TMR4F will clear itself when the timeout has expired.

	; Increment the retry counter.
	incf	retry_count
