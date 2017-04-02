;*************************************************************************
; NETWORK LAYER
; 
; Version 1.00
; 29/08/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************
;
; Description: 
;	This code module implements the network layers.
; 	It provides transparent network access to the upper layers.
;
; Functions:
;
; Dependencies:
;	- Buffers.asm
;	- MediaAccess.asm
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

BROADCAST_ADDR	equ	0xFF

;*************************************************************************
; CONSTANTS

; PACKET IDs
RREQ			equ	0x00
RREP			equ	0x01
DATA_PKT		equ	0x02
ACK				equ	0x03
RMOD_ACK		equ	0x04
RMOD			equ	0x05

AREQ			equ 0x80
AREP			equ 0x81

;******************************************************************************
; IMPORTED VARIABLES

extern rxbuf, txbuf

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES



;******************************************************************************
; LOCAL VARIABLES

; Packet fields
dest_addr		res 1		; Destination Address.
source_addr		res 1		; Source Address.
packet_id		res 1		; Packet Type.





;******************************************************************************
; IMPORTED SUBROUTINES

 #include "Buffers.inc"			; Software Buffers Header File

;******************************************************************************
; EXPORTED SUBROUTINES












;******************************************************************************
; START OF CODE

 CODE




;***************************************************************
; SUBROUTINE: 	NETWORK_LAYER
; 
; Description:	The executive loop for the network layer.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************


;***************************************************************
; SUBROUTINE: 	RX_PACKET
; 
; Description:	Processes an incoming packet.
; Precond'ns:	
; Postcond'ns:	
; Regs Used:	
;***************************************************************
RX_PACKET:

; ---------------------------------------------------------
; Point to the beginning of the receive buffer, and retrieve
; the packet type, destination address, and source address.
	BUF_SEL	rxbuf				; Select the receive buffer
	movlw	0x00				; Reset the cursor
	BUF_SETCURSOR				; ...
	BUF_GET						; Get the destination address.
	movwf	dest_addr			; Store it.
	BUF_GET						; Get the source address.
	movwf	source_addr			; Store it.
	BUF_GET						; Get the packet id.
	movwf	packet_id			; Store it.


	
; ---------------------------------------------------------
; Check the destination address on the packet
_RX_CHK_DEST:
	movf	dest_addr, W		; Load the destination address.
	subwf	my_addr, W			; Compare it to my address.
	bz		_RX_CHK_TYPE		; If they match, check the packet type.
	
	movf	dest_addr, W		; Load the destination address.
	sublw	BROADCAST_ADDR		; Compare it to the broadcast address.
	bz		_RX_CHK_TYPE		; If they match, check the packet type.
	
	;bra		_RX_PARO			; If the packet is not addressed to us,
								; just run the PARO routine.
	return ; DEBUG
	
; ---------------------------------------------------------
; Check the packet type. 
_RX_CHK_PID:

	; Check if this is a DSR packet or a ADDR packet.
	; 0xxxxxxx = DSR Packet; 1xxxxxxx  = ADDR packet.
	btfss	packet_id, 7		
	;bra		PROCESS_DSR_PACKET
	return
	bra		ADDRESSING

