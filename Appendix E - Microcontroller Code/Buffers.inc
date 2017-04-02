;*************************************************************************
; Software Buffers Header File
; 
; Version 1.00
; 14/08/2005
;
; Li-Wen Yip
; Ad Hoc Radio Networking Research Project
; School Of Engineering
; James Cook University
;
;*************************************************************************

  extern        buflen 

;***************************************************************
; MACRO:        BUF_SEL
; 
; Description:  Select a buffer
; Arguments:    addr - the address of the buffer.
;               size - the size of the buffer.
; Postcond'ns:  - Address of buffer has been loaded to FSR0.
;               - Address of cursor and END has been loaded to FSR1.
;               - Length of buffer has been loaded to buflen.
; Regs Ch'd: WREG
;***************************************************************
; CURSOR(1) : END(1) : DATA(VAR) ;
; ^ FSR1               ^ FSR0
BUF_SEL macro addr, size
    lfsr    0, addr + 2         ; Load address of buffer data area to FSR0.
    lfsr    1, addr             ; Load address of buffer variables to FSR1.
    movlw   size - 2            ; Load the size of the buffer data area.
    movwf   buflen              ; Store it in buflen.
 endm

;***************************************************************
; MACRO:        BUF_GETCURSOR
; 
; Description:  Gets the cursor value of the currently selected buffer.
; Arguments:    None.
; Precond'ns:   A buffer has been selected with BUF_SEL
; Postcond'ns:  cursor -> WREG
; Regs Used:    WREG
;***************************************************************
; INDF1 = cursor
BUF_GETCURSOR macro
    movf    INDF1, W
 endm

;***************************************************************
; MACRO:        BUF_SETCURSOR
; 
; Description:  Sets the cursor value of the currently selected buffer.
; Arguments:    Cursor value in WREG.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  WREG -> cursor
; Regs Used:    None.
;***************************************************************
; INDF1 = cursor
BUF_SETCURSOR macro
    movwf   INDF1
 endm

;***************************************************************
; MACRO:        BUF_CLEAR
; 
; Description:  Clears the currently selected buffer.
; Arguments:    None.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  0 -> cursor, 0 -> end.
; Regs Used:    None.
;***************************************************************
; POSTINC1 = cursor, POSTDEC1 = end
BUF_CLEAR macro
    clrf    POSTINC1            ; 0 -> Cursor
    clrf    POSTDEC1            ; 0 -> End
 endm

    
;***************************************************************
; MACRO:        BUF_MARKEND
; 
; Description:  Mark the current cursor position as the end of the 
;                   currently selected buffer.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  - Cursor -> End
; Regs Ch'd:    WREG
;***************************************************************
; POSTINC1 = cursor, POSTDEC1 = end
BUF_MARKEND macro
    movf    POSTINC1, W         ; Cursor -> WREG
    movwf   POSTDEC1            ; WREG -> End
 endm

;***************************************************************
; MACRO:        BUF_GETEND
; 
; Description:  Get the end index of the buffer (i.e. the length of the 
;                   data contained in the buffer).
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  End -> WREG
; Regs Ch'd:    WREG
;***************************************************************
; POSTINC1 = cursor, POSTDEC1 = end
BUF_GETEND macro
    movf    POSTINC1, F         ; Increment FSR1
    movf    POSTDEC1, W         ; End -> WREG
 endm


;***************************************************************
; MACRO:        BUF_PUT
; 
; Description:  Write a byte to the buffer at the current cursor location
;               and increment the cursor by one.
; Arguments:    Data byte in WREG.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  WREG -> Buffer[Cursor++]
; Regs Ch'd:    WREG
;***************************************************************
; INDF1 = cursor, PLUSW0 = Buffer[Cursor]
BUF_PUT macro
    movwf   PREINC2             ; Push our data byte onto the stack.
    movf    INDF1, W            ; Load the cursor to WREG.
    movff   POSTDEC2, PLUSW0    ; Pop our data byte on the buffer.
    incf    INDF1               ; Increment the cursor.
 endm

;***************************************************************
; MACRO:    BUF_GET
; 
; Description:  Get a byte from the buffer at the current cursor
;                   location and increment the cursor by one.
; Arguments:    Data byte in WREG.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  Buffer[Cursor++] -> WREG
; Regs Ch'd:    WREG
;***************************************************************
; INDF1 = cursor, PLUSW0 = Buffer[Cursor]
BUF_GET macro
    ; Read a byte from the buffer.
    movf    INDF1, W            ; Load the cursor to WREG.
    movf    PLUSW0, W           ; Read a byte from WREG.
    ; Increment the cursor.
    incf    INDF1, F        
 endm

;***************************************************************
; MACRO:    BUF_FREE
; 
; Description:  Calculate the amount of free space in the buffer.
; Arguments:    None.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  buflen - end -> WREG
; Regs Ch'd:    WREG
;***************************************************************
BUF_FREE macro
    BUF_GETEND                  ; Load the end position.
    subwf       buflen, W       ; buflen 0 end -> WREG
 endm

;***************************************************************
; MACRO:    BUF_SNEOF (Skip if cursor is not at end of data)
; 
; Description:  Skip if cursor < end
; Arguments:    None.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  
; Regs Ch'd:    WREG.
;***************************************************************
; POSTINC1 = cursor, POSTDEC1 = end
BUF_SNEOF macro
    movf    POSTINC1, W         ; Load the cursor to WREG.
    cpfsgt  POSTDEC1            ; Skip if end > cursor.
 endm

;***************************************************************
; MACRO:    BUF_SNFULL (Skip if buffer is not full)
; 
; Description:  Skip if cursor < buflen
; Arguments:    None.
; Precond'ns:   A buffer has been selected with BUF_SEL.
; Postcond'ns:  
; Regs Ch'd:    WREG.
;***************************************************************
; INDF1 = cursor
BUF_SNFULL macro
    movf    INDF1, W            ; Load the cursor to WREG.
    cpfsgt  buflen              ; Skip if buflen > cursor.
 endm
