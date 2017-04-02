;*************************************************************************
; Master Header File
; Version 1.00
; 16/09/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************

 LIST P=18C452, F=INHX32        ; directive to define processor and file format
 #include <P18C452.INC>         ; processor specific variable definitions

 #include "macrolib.inc"        ; common macros
 #include "pinconnections.inc"  ; Pin connections
 #include "swstack.inc"         ; Software Stack
