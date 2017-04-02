;*************************************************************************
; ANALOG TO DIGITAL CONVERSION MODULE
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
;	
;
; Functions:
;
;
; Dependencies:
;
;
; Resources Used:
;
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


;*************************************************************************
; CONSTANTS


;******************************************************************************
; IMPORTED VARIABLES

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES




;******************************************************************************
; LOCAL VARIABLES



;******************************************************************************
; IMPORTED SUBROUTINES



;******************************************************************************
; EXPORTED SUBROUTINES

 



;******************************************************************************
; START OF CODE

 CODE



;***************************************************************
; SUBROUTINE: 	ADC_INIT
; 
; Description:	Initialisation for a/d converter
; Precond'ns:	-
; Postcond'ns:	
; Regs Used:	
;***************************************************************
ADC_INIT:
	setf	TRISA			; Set all PORTA pins as inputs.

	movlw	b'00001000'		; ADCON1 Configuruation Word.
	;		 '0-------'		; Left justified result.
	;		 '-0------'		; Convert at Fosc/32
	;		 '----1000'		; AN2 = VrefH, AN3 = VrefL, AN0-AN1, AN4-AN8 = Analog Inputs.
	movwf	ADCON1

