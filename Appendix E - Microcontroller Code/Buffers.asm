;*************************************************************************
; SOFTWARE BUFFERS MODULE
; 
; Version 0.50
; 22/08/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;
; The RAM allocated for each buffer must not overlap two RAM banks. This allows
; faster loading and saving of the pointer values.
;
; Resources Requiring Exclusive Use:
; RAM Bank 4, FSR0
;
; Notes:
; 
;
; What you need to understand to work with this code:
; 
; 
; 
;
;*************************************************************************
; INCLUDE FILES
 LIST P=18C452, F=INHX32	;directive to define processor and file format
 #include <P18C452.INC>		;processor specific variable definitions

;*************************************************************************
; CONFIGURATION SECTION


;*************************************************************************
; CONSTANTS

;*************************************************************************
; VARIABLES

; Access RAM
 udata_acs
buflen		res 1	; Length of the data area of the current buffer.


;*************************************************************************
; GLOBALLY AVAILABLE LABELS

 global buflen

;*************************************************************************
; START OF CODE

 CODE

 end
