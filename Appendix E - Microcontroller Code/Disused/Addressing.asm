;*************************************************************************
; <NAME OF MODULE>
; 
; Version x.xx
; dd/mm/yyyy
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

AREP_RETRY_LIMIT	equ	d'8'
AREP_RETRY_TIMEOUT	equ	d'5000'

UID_L equ d'12'
UID_H equ d'34'

;*************************************************************************
; CONSTANTS

;******************************************************************************
; IMPORTED VARIABLES

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES


;******************************************************************************
; LOCAL VARIABLES

my_first_addr	res 1		; My address range.
my_last_addr	res 1		; ...
first_alloc		res 1		; Address range under allocation.
last_alloc		res 1		; ...
broadcast_addr	res 1		; Broadcast address.

addr_flags		res 1		; Addressing Status Flags
 #define init_state		addr_flags, 0	; In initialisation state.
 #define idle_state		addr_flags, 1	; In idle state.
 #define alloc_state 	addr_flags, 2 	; In address allocation state.
 #define packet_av		addr_flags, 3	; Packet available for processing.

init_flags		res 1



; Fields for ADDR packets
operation_id	res 1		; Operation identifier.

arep_retry_count res 1		; AREQ retry counter

; Working registers for AREP processing
arep_source_addr	res 1	; The address of the offering node.
arep_first_addr		res 1	; The start of the offered address range.
arep_last_addr		res 1	; The end of the offered address range.
arep_best_source	res 1	; The address of the best offer.
arep_best_size		res 1	; The size of the best offer

; AREP buffer - provide enough room for 8 AREP messages.
.addrbuf UDATA
AREP_BUFFER_LEN	equ d'24'
arep_buffer		res AREP_BUFFER_LEN


;******************************************************************************
; IMPORTED SUBROUTINES

;******************************************************************************
; EXPORTED SUBROUTINES

;******************************************************************************
; START OF CODE

 CODE

;***************************************************************
; SUBROUTINE: 	ADDRESSING
; 
; Description:	Run the addressing algorithm.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
ADDRESSING:

	; First order of business is to check what state we are in.
	; We will branch to a different subroutine depending on what
	; state we are in.

	btfsc	alloc_state		; Are we in the allocation state?
	bra		ALLOC_STATE		; YES.
	btfsc	idle_state		; Are we in the idle state?
	bra		IDLE_STATE		; YES.

	bra		INIT_STATE		; We must be in the initialisation state.


;***************************************************************
; SUBROUTINE: 	INIT_STATE
; 
; Description:	Subroutine for the initialisation state.
;				- If we are in this state we don't yet have a valid
;					network address, so we are trying to aqurie one.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
INIT_STATE:

	; If an addressing packet has arrived, process it.
	btfsc	packet_av		
	bra		_INIT_PKTIN	

	; If we are waiting for AREP_TIMEOUT to expire, just exit this routine.
	btfsc	TMR5, 0			; Is Timer 5 running?
	return					; YES - bugger off.
	
	; YES - AREP_TIMEOUT has expired.

	; Check if we have anything in the AREP buffer.
	BUF_SEL		arep_buffer		; Select the AREP buffer.
	BUF_GETEND					; Check how many bytes are in the buffer.
	bz			_SEND_AREQ		; If the buffer is empty, send an AREQ.

; ------------------------------------------------------------------
; We have got some AREPs (Address Replies). Yay, we have friends!
; Sort through the offers, reply to the largest offer with an AACK (Address Accept),
; and reply to all the others with AREJ (Address Reject).
_INIT_GOT_AREP:	
	


; ------------------------------------------------------------------
; Process an incoming packet.
_INIT_PKTIN:
	
	; Check the packet type. In the initialisation state, we are only interested
	; in AREP (Address Reply), so discard the packet if it's anything else.
	movlw	AREQ	
	cpfseq	packet_id					; Is the packet an AREQ?
	bra		DISCARD_PACKET_AND_RETURN	; No.
	
	; Read the 
	










 
;***************************************************************
; SUBROUTINE: 	PROCESS_ADDR_PACKET
; 
; Description:	Process an addressing packet.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
PROCESS_ADDR_PACKET:

	; Load the operation identifier.
	; (We are assuming that Buffer 0 is still selected, and the last
	; thing read from it was the DA, SA, and PID.
	call	BUFGET
	movwf	operation_id

	; Check if we have an AREQ (Address Request) Packet.
	movf	packet_id, W
	sublw	AREQ		
	bz		PROCESS_AREQ:

	; Check if we have an AREP (Address Reply) Packet.
	movf	packet_id, W
	sublw	AREP
	bz		PROCESS_AREP:

	; Check if we have an AACK (Address Accept) Packet.
	movf	packet_id, W
	sublw	AACK
	bz		PROCESS_AACK:
	
	; Check if we have an AREJ (Address Reject) Packet.
	movf	packet_id, W
	sublw	AREJ
	bz		PROCESS_AREJ:

	; Check if we have an ARET (Address Return) Packet.
	movf	packet_id, W
	sublw	ARET
	bz		PROCESS_ARET:



;***************************************************************
; SUBROUTINE: 	PROCESS_AREQ:
; 
; Description:	Process an AREQ (Address Request) Packet.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
PROCESS_AREQ:

; --------------------------------------------------------------
; Check if we have any free addresses.

	; Calculate how many spare addresses we have.
	; (Last Address - First Address -> WREG)
	movf	my_first_addr	; Load up the first address in our address block.
	subwf	my_last_Addr, W	; Subtract it from the last address in our address block.
	
	; Do we have any spare addresses?
	bnz		_AREQ_ALLLOC 	; YES - allocate addresses to the requesting node.
	bz		_AREQ_FORWARD	; NO - forward on the address request.
	
; --------------------------------------------------------------
; Mark half of our addresses as under allocation, and send an address reply.
; Start a timeout.
_AREQ_ALLOC:

	; Split up our address range.
	; Let X = last_addr - first_addr = The number of spare addresses we have.
	; Our available address range will become (first_addr) to (first_addr + X/2).
	; Our address range under allocation will be (first_addr + x/2 + 1) to (last_addr)

	; WREG = X from last instruction.
	movff	my_last_addr, last_alloc	; last_alloc = last_addr
	bcf		STATUS, C		; Clear the Carry bit.
	rrcf	WREG			; WREG = X/2 (rounded down).
	addwf	my_first_addr,W ; WREG = first_addr + X/2
	movwf	my_last_addr,W	; last_addr = first_addr + X/2
	addlw	0x01			; WREG = first_addr + X/2 + 1
	movwf	first_alloc		; first_alloc = first_addr + X/2 + 1

	; Enter the allocation state.
	clrf	arep_retry_count		; Reset the AREP retry counter
	clrf	TMRF5					; Make sure Timer 5 is stopped.
	bsf		allocation_state		; Set the allocation state flag.

	; We will wait for the ALLOCATION_STATE routine to start under interrupt.
	; The time it takes for ALLOCATION_STATE to run for the first time will depend
	; on the value of TIMER2, so hopefully all the nodes won't try to send their AREP
	; messages all at once.

; --------------------------------------------------------------
; Forward on the address request.
_AREQ_FORWARD:

	

	
;***************************************************************
; SUBROUTINE: 	ALLOCATION_STATE
; 
; Description:	Send an AREP (Address Reply) Packet.
;				Called under software timer interrupt.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
ALLOCATION_STATE:

	; Only run this subroutine if we are in the allocation state
	; Timer5 has expired, and the media layer is ready to transmit.
	btfss	allocation_state	; Are we in the allocation state?
	return						; NO - get outta here.
	btfsc	TMRF5, 0			; Is TIMER5 running?
	return						; YES - get outta here.
	btfss	packet_transmitted	; Is the media layer ready to transmit?
	return						; NO - get outta here.

	



	; Check how many times we have sent the AREP packet.
	incf	arep_retry_count	; Increment the retry counter
	movlw	AREP_RETRY_LIMIT	; Load the retry limit.
	cpfslt	arep_retry_count	; Have we reached the retry limit?
	bra		_AREP_GIVEUP

	; Build the AREP packet in BUFFER1
	; |  DA  |      SA       | PID  | OPID |          ALLOC          |
	; | 0x00 | my_first_addr | AREP | OPID | first_alloc, last_alloc |
	call	BUFSEL1				; Select Buffer 1.
	call	BUFCLR				; Clear out the buffer.

	movlw	0x00				; Load the destination address (anonymous node).
	call	BUFPUT				; ...
	movf	my_first_addr, W	; Load the source address (that's my address).
	call	BUFPUT				; ...
	movlw	AREP				; Load the packet type (Address Reply).
	call	BUFPUT				; ...
	movf	operation_id		; Load the operation_id.
	call	BUFPUT				; ...
	movf	first_alloc			; Load the first allocated address.
	call	BUFPUT				; ...
	movf	last_alloc			; Load the last allocated address.
	call	BUFPUT				; ...

	; Load TIMER5, but don't start it yet. We want to wait until the
	; packet has actually been transmitted before we start the timer.
	movlw	low AREP_RETRY_TIMEOUT
	movwf	TMRL5
	movlw	high AREP_RETRY_TIMEOUT
	movwf	TMRH5

	; Initiate transmission of the packet.
	call	TX_PACKET
	return

