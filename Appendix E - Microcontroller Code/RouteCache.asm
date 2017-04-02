;*************************************************************************
; ROUTE CACHE MODULE
; 
; Version 0.10
; 17/07/2005
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
;   Use locations in FRAM which are 4 bytes long.
;
; Dependencies:
;   a) SPI.ASM  -   SPI routines and initlialisation.
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

;*************************************************************************
; CONSTANTS

; FRAM Op-codes
OP_WRSR     equ 0x01        ; Write Status Register
OP_WRITE    equ 0x02        ; Write Memory Data
OP_READ     equ 0x03        ; Read Memory Data
OP_WRDI     equ 0x04        ; Write Disable
OP_RDSR     equ 0x05        ; Read Status Register
OP_WREN     equ 0x06        ; Set Write Enable Latch

;******************************************************************************
; IMPORTED VARIABLES

 UDATA_ACS
;******************************************************************************
; GLOBAL VARIABLES

; Fields in each route record
source_addr     res 1
dest_addr       res 1
next_hop        res 1
tx_power        res 1

 global source_addr, dest_addr, next_hop, tx_power

;******************************************************************************
; LOCAL VARIABLES

cursor_l        res 1   
cursor_h        res 1   
end_l           res 1
end_h           res 1

table_ptr       res 1       ; Routing table pointer



 
;******************************************************************************
; IMPORTED SUBROUTINES

; From SPI.asm
 extern SPIRW

;******************************************************************************
; EXPORTED SUBROUTINES

 global TEST_ROUTECACHE, FRAM_INIT

;******************************************************************************
; START OF CODE

 CODE

;***************************************************************
; SUBROUTINE:   FRAM_INIT
; 
; Description:  - Initialise the FRAM.
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
FRAM_INIT:

    ; Set up FRAM pins.
    CONFIG_FRAM_PINS

    return

;***************************************************************
; MACRO:        FRAM_READ
; 
; Description:  Tell the FRAM we want to read from the specified address.
; Arguments:    
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
FRAM_READ macro addr_h, addr_l
    
    ; Start an FRAM operation (Set /CS low)
    bcf     FRAM_NCS        

    ; Send the READ op-code to the FRAM.
    movlw   OP_READ         ; Tell FRAM we want to read.
    call    SPIRW           ; ...

    ; Send the memory address to the FRAM.
    movf    addr_h, W       ; From this high memory location.
    call    SPIRW           ; ...
    movf    addr_l, W       ; And this low memory location.
    call    SPIRW           ; ...

 endm
 
;***************************************************************
; MACRO:        FRAM_WRITE
; 
; Description:  Tell the FRAM we want to write at the specified address.
; Arguments:    
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
FRAM_WRITE macro addr_h, addr_l
 
    ; Send the WRITE ENABLE op-code to the FRAM.
    bcf     FRAM_NCS        ; Start an FRAM operation (Set /CS low)
    movlw   OP_WREN         ; Enable write operations
    call    SPIRW           ; ...
    bsf     FRAM_NCS        ; Terminate the FRAM operation (Set /CS high)

    ; Wait a little while before starting the next command.
    nop
    nop
    bcf     FRAM_NCS        ; Start an FRAM operation (Set /CS low)

    ; Send the WRITE op-code to the FRAM.
    movlw   OP_WRITE        ; Tell FRAM we want to write.
    call    SPIRW           ; ...

    ; Send the memory address to the FRAM.
    movf    addr_h, W       ; To this high memory location.
    call    SPIRW           ; ...
    movf    addr_l, W       ; And this low memory location.
    call    SPIRW           ; ...

 endm
 

;***************************************************************
; SUBROUTINE:   GET_END
; 
; Description:  - Get the end pointer.
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
GET_END:

    ; Put the address 0x0000 on the software stack.
    movlw       0x00    
    movwf       PREINC2
    movwf       PREINC2
    
    ; Tell the FRAM we want to start reading from 0x0000
    FRAM_READ   POSTDEC2, POSTDEC2

    ; Read 2 bytes from the FRAM.
    call    SPIRW               ; Read the low byte of the end address.
    movwf   end_l               ; ...
    call    SPIRW               ; Read the high byte of the end address.
    movwf   end_h               ; ...

    ; Terminate the FRAM operation (Set /CS high)
    bsf     FRAM_NCS        

    return
    
;***************************************************************
; SUBROUTINE:   SET_END
; 
; Description:  - Mark the current cursor position as the new end of the table
; Precond'ns:   
; Postcond'ns:  
; Regs Used:    
;***************************************************************
SET_END:

    ; Put the address 0x0000 on the software stack.
    movlw       0x00    
    movwf       PREINC2
    movwf       PREINC2
    
    ; Tell the FRAM we want to start writing at 0x0000
    FRAM_WRITE  POSTDEC2, POSTDEC2

    ; Write 2 bytes of data to the FRAM.
    movf    end_l, W            ; Write the low byte of the end address.
    call    SPIRW               ; ...
    movf    end_h, W            ; Write the high byte of the end address.
    call    SPIRW               ; ...

    ; Terminate the FRAM operation (Set /CS high)
    bsf     FRAM_NCS        


    return

;***************************************************************
; SUBROUTINE:   ROUTE_ADD
; 
; Description:  - Adds a route to the routing table.
; Precond'ns:   - 
; Postcond'ns:  - 
; Regs Used:    
;***************************************************************
ROUTE_ADD:

    ; Get end address of the data stored in the FRAM, which is where
    ; we want to store the new record.
    call    GET_END 
        
    ; Because RAM always comes in sizes so that the full address space is used, 
    ; we can check if we have gone past the end of the address space by testing
    ; a single bit. In this case, we have a 2048 byte RAM, so the highest address
    ; is 0x07FF, or 0000 0111 1111 1111. When we go one past this address, it will
    ; be 0x0800, or 0000 1000 0000 0000. Therefore, by testing bit 3 of the high
    ; byte of the address, we can check if we have reached the end of the memory space.
    btfsc   end_h, 3            ; Are we at 0x800?
    return                      ; YES - memory is full, exit without adding route.

    ; Tell the FRAM we want to start writing at the end of the table.
    FRAM_WRITE  end_h, end_l

    ; Write the record to the FRAM.
    movf    source_addr, W      ; Load the source address and write it.
    call    SPIRW               ; ...
    movf    dest_addr, W        ; Load the dest address and write it.
    call    SPIRW               ; ...
    movf    next_hop, W         ; Load the next hop and write it.
    call    SPIRW               ; ...
    movf    tx_power, W         ; Load the transmit power and write it.
    call    SPIRW               ; ...

    ; Terminate the FRAM operation (Set /CS high)
    bsf     FRAM_NCS        
    
    ; Mark the new end address of the data stored in FRAM.
    movlw   0x04                ; Add 4 to the end address.
    addwf   end_l               ; ...
    movlw   0x00                ; ...
    addwfc  end_h               ; ...
    call    SET_END             ; Set the end address in the FRAM chip.
    
    return



;***************************************************************
; SUBROUTINE:   ROUTE_ERASE
; 
; Description:  - Delete the route currently pointed to by the cursor.
; Precond'ns:   - 
; Postcond'ns:  - 
; Regs Used:    
;***************************************************************
ROUTE_ERASE:

    ; We will delete the record, and move the last record into the empty space.

    ; Get the address of the last record in the table, which is at (END - 4)
    call    GET_END             ; Get the end address stored in the FRAM.
    movlw   0x04                ; Subtract 4 from the end address.
    subwf   end_l               ; ...
    movlw   0x00                ; ...
    subwfb  end_h               ; ...
    
    ; Mark this as the new end of the table.
    call    SET_END

    ; Tell the FRAM we want to start reading the last record in the table.
    FRAM_READ   end_h, end_l
    
    ; Read the record at the end of the table.
    call    SPIRW               ; Read the source address and save it.
    movwf   source_addr         ; ...
    call    SPIRW               ; Read the destination address and save it.
    movwf   dest_addr           ; ...
    call    SPIRW               ; Read the next hop and save it.
    movwf   next_hop            ; ...
    call    SPIRW               ; Read the transmit power and save it.
    movwf   tx_power            ; ...

    ; Terminate the FRAM operation (Set /CS high)
    bsf     FRAM_NCS        
    
    ; Tell the FRAM we want to start writing at the current cursor position.
    FRAM_WRITE  cursor_h, cursor_l
    
    ; Write over the record at the current cursor position.
    movf    source_addr, W      ; Load the source address and write it.
    call    SPIRW               ; ...
    movf    dest_addr, W        ; Load the dest address and write it.
    call    SPIRW               ; ...
    movf    next_hop, W         ; Load the next hop and write it.
    call    SPIRW               ; ...
    movf    tx_power, W         ; Load the transmit power and write it.
    call    SPIRW               ; ...

    ; Terminate the FRAM operation (Set /CS high)
    bsf     FRAM_NCS        

    return



;***************************************************************
; SUBROUTINE:   ROUTE_SEARCH
; 
; Description:  - Search for a route in the routing table.
; Precond'ns:   - 
; Postcond'ns:  - 
; Regs Used:    
;***************************************************************
ROUTE_SEARCH:

    ; Make sure we have the correct end pointer for the table.
    call    GET_END
        
    ; We will do the entire search with the one FRAM command, so we
    ; will use cursor_l and cursor_h as a local memory tracker.
    clrf    cursor_h            ; Reset the cursor to the first record in the table.
    movlw   0x04                ; ...
    movwf   cursor_l            ; ...

    ; Tell the FRAM we want to start reading the first record in the table.
    FRAM_READ   cursor_h, cursor_l

_ROUTE_SEARCH_LOOP:

    ; Check if we have reached the end of the table.
    movf    cursor_h, W         ; Compare the high bytes of the cursor and
    cpfseq  end_h               ; the end pointer. Are they equal?
    bra     _ROUTE_SEARCH_CHK   ; NO - read the current record.
    
    movf    cursor_l, W         ; Compare the low bytes of the cursor and
    cpfseq  end_l               ;   the end pointer. Are they equal?
    bra     _ROUTE_SEARCH_CHK   ; NO - read the current record.

    ; We didn't find the requested route in the table.
    bsf     FRAM_NCS            ; Terminate the FRAM operation (Set /CS high)
    return

_ROUTE_SEARCH_CHK:

    ; Check source address of the record.
    call    SPIRW               ; Read the source address.
    cpfseq  source_addr         ; Is it the source address we're looking for?
    bra     _SKIP_3             ; NO - skip the next 3 bytes and try again.
    
    ; Check destination address of the record
    call    SPIRW               ; Read the destination address.
    cpfseq  dest_addr           ; Is it the destination address we're looking for?
    bra     _SKIP_2             ; NO - skip the next 2 bytes and try again.

    ; We have found a route - read the hop count and tx power
    call    SPIRW               ; Read the next hop and save it.
    movwf   next_hop            ; ...
    call    SPIRW               ; Read the transmit power and save it.
    movwf   tx_power            ; ...
    
    ; Terminate the FRAM operation (Set /CS high)
    bsf     FRAM_NCS        

    return
    
_SKIP_3:
    ; Skip the next 3 memory locations and check the next record.
    call    SPIRW
_SKIP_2:
    ; Skip the next 2 memory locations and check the next record.
    call    SPIRW
    call    SPIRW

    ; Advance the LOCAL memory tracker to the next record.
    movlw   0x04
    addwf   cursor_l
    movlw   0x00
    addwfc  cursor_h

    bra     _ROUTE_SEARCH_LOOP


;***************************************************************
; SUBROUTINE:   ROUTE_ERASE_ALL
; 
; Description:  - Delete all routes.
; Precond'ns:   - 
; Postcond'ns:  - 
; Regs Used:    
;***************************************************************
ROUTE_ERASE_ALL:
    
    ; Set the beginning of the table as the end.
    clrf    end_h               ; Say that the first record is the end of the table.
    movlw   0x04                ; ...
    movwf   end_l               ; 

    ; Write the end address of the table data.
    call    SET_END
    return

 global TEST_ROUTECACHE

TEST_ROUTECACHE:
    call    FRAM_INIT

    call    ROUTE_ERASE_ALL

    movlw   0x11
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x22
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x33
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x44
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x55
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x66
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x77
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x88
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    movlw   0x99
    movwf   source_addr
    movwf   dest_addr
    movwf   next_hop
    movwf   tx_power
    call    ROUTE_ADD

    call    ROUTE_SEARCH

    bra     TEST_ROUTECACHE

    return

 end
