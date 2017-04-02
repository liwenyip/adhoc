;******************************************************************************
;   This file is a basic template for creating relocatable assembly code for  *
;   a PIC18C452. Copy this file into your project directory and modify or     *
;   add to it as needed. Create a project with MPLINK as the language tool    *
;   for the hex file. Add this file and the 18C452.LKR file to the project.   *
;                                                                             *
;   The PIC18CXXX architecture allows two interrupt configurations. This      *
;   template code is written for priority interrupt levels and the IPEN bit   *
;   in the RCON register must be set to enable priority levels. If IPEN is    *
;   left in its default zero state, only the interrupt vector at 0x008 will   *
;   be used and the WREG_TEMP, BSR_TEMP and STATUS_TEMP variables will not    *
;   be needed.                                                                *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on the         *
;   features of the assembler and linker.                                     *
;                                                                             *
;   Refer to the PIC18CXX2 Data Sheet for additional information on the       *
;   architecture and instruction set.                                         *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:                                                                *
;    Date:                                                                    *
;    File Version:                                                            *
;                                                                             *
;    Author:                                                                  *
;    Company:                                                                 *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files required:         P18C452.INC                                      *
;                            18C452.LKR                                       *
;                                                                             *
;******************************************************************************

 LIST P=18C452, F=INHX32	; directive to define processor and file format
 #include <P18C452.INC>		; processor specific variable definitions
	
 #include "macrolib.inc"	; commonly used macros
 #include "swstack.inc"		; software stack
 #include "buffers.inc"		; Labels for buffers.asm

;********************************************************************************
; EXTERNAL LABELS

;******************************************************************************
;Configuration bits
; The __CONFIG directive defines configuration data within the .ASM file.
; The labels following the directive are defined in the P18C452.INC file.
; The PIC18CXX2 Data Sheet explains the functions of the configuration bits.
; Change the following lines to suit your application.

;	__CONFIG	_CONFIG0, _CP_OFF_0
;	__CONFIG	_CONFIG1, _OSCS_OFF_1 & _RCIO_OSC_1
;	__CONFIG	_CONFIG2, _BOR_ON_2 & _BORV_25_2 & _PWRT_OFF_2
;	__CONFIG	_CONFIG3, _WDT_ON_3 & _WDTPS_128_3
;	__CONFIG	_CONFIG5, _CCP2MX_ON_5
;	__CONFIG	_CONFIG6, _STVR_ON_6

;******************************************************************************
;Variable definitions
; These variables are only needed if low priority interrupts are used. 
; More variables may be needed to store other special function registers used
; in the interrupt routines.

MAIN_UDATA	UDATA

WREG_TEMP	RES	1	;variable in RAM for context saving 
STATUS_TEMP	RES	1	;variable in RAM for context saving
BSR_TEMP	RES	1	;variable in RAM for context saving

;******************************************************************************
;Reset vector
; This code will start executing when a reset occurs.

RESET_VECTOR	CODE 0x0000

	goto	Main		;go to start of main code

;******************************************************************************
;High priority interrupt vector
; This code will start executing when a high priority interrupt occurs or
; when any interrupt occurs if interrupt priorities are not enabled.

HI_INT_VECTOR	CODE 0x0008

	bra		HighInt		;go to high priority interrupt routine

;******************************************************************************
;Low priority interrupt vector
; This code will start executing when a low priority interrupt occurs.
; This code can be removed if low priority interrupts are not used.

LOW_INT_VECTOR	CODE 0x0018

	bra		LowInt		;go to low priority interrupt routine






 CODE





;******************************************************************************
;High priority interrupt routine
; The high priority interrupt code is placed here.


HighInt:

;	*** high priority interrupt code goes here ***

	retfie	FAST

;******************************************************************************
;Low priority interrupt routine
; The low priority interrupt code is placed here.
; This code can be removed if low priority interrupts are not used.

LowInt:
	movff	STATUS,STATUS_TEMP	;save STATUS register
	movff	WREG,WREG_TEMP		;save working register
	movff	BSR,BSR_TEMP		;save BSR register

;	*** low priority interrupt code goes here ***


	movff	BSR_TEMP,BSR		;restore BSR register
	movf	WREG_TEMP,W			;restore working register
	movff	STATUS_TEMP,STATUS	;restore STATUS register
	retfie

;******************************************************************************
;Start of main program
; The main program code is placed here.

Main:

	movlw 3
	movwf WREG_TEMP
Loopy:
	decf	WREG_TEMP
	bra Loopy
	

	STACKINIT 0x500				; Initialise the stack
	

	bsf 	INTCON, GIEH		; enable global interrupts
	bsf 	INTCON, GIEL		; enable peripheral interrupts

	call	BUFSEL0
	call 	BUFCLR
	movlw	0x04
	call	BUFCSR

	movlw	0x23
	call 	BUFPUT
	movlw	0x09
	call 	BUFPUT
	movlw	0x85
	call 	BUFPUT

	movlw	0x00
	call	BUFCSR

	call 	BUFGET
	call 	BUFGET
	call 	BUFGET



Loop:
	clrwdt
	goto 	Main


;******************************************************************************
;End of program

 END
