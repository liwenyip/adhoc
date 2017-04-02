
; Configure UART
	movlw	b'00100100'
	;		 '-0------'			; Select 8 Bit Transmission
	;		 '--1-----'			; Enable Transmit
	;		 '---0----'			; Asynchronous Mode
	;		 '-----1--'			; Select high baud rate.
	movwf	TXSTA
	
	movlw	b'10000000'
	;		 '1-------'			; Enable Serial Port
	;		 '-0------'			; Select 8 Bit Receive
	;		 '--------'			; 
	;		 '--------'			; 
	movwf	RCSTA

	movlw	d'25'				; Set 9600bps
	movwf	SPBRG				; ...
	
