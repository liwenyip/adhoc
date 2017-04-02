
;Ad Hoc Radio Network
;Single Node Code
;by Steven Sloots
;for John Wicking 2004
 title "Ad_Hoc_V1"

 list p=18c452 ;Select device. 

#include <p18c452.inc> ;Include standard header file for the selected device.

;+-----------------------------------------------------------------------------------------------------------------------------------+
;| Dest. Add. (1)| Source Addr. (1) | Packet ID (1) | Tx. Pwr (1) | Route ID (2) | Hop Count (1) | Packet Info (Addresses/Data) (VAR)|
;+-----------------------------------------------------------------------------------------------------------------------------------+
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!REMEMBER, HOP_COUNT IS INCREMENTED AS THE PACKET LEAVES!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;*********************************************************************
 org		0x00
	goto	device_init
 org 		0x08
	goto	IRPSERVICE
 org		0x18
	goto	LOW_PRIORITY
;*********************************************************************
	CBLOCK		0x20
;***Timing Registers***
current_rreq_timer:		1		;1 byte for timing the validity of routes
route_finder_timer: 	1		;1 byte to time the discovery of routes
backoff_timer:	1				;timer for exponential backoff CDMA algorithm
ACK_timer:		1				;timer to time the timeout on an ACK from each data packet
Reply_timer:	1
validity_count:	1
settler_count:	1
sleep_count:	1				;timer to count down to nap time
;**********************
;***Flag Registers***
error_flags:	1		;contains error flags
dsr_flags: 		1		;buffer to store the flags associated with dsr module
flag_buff:		1		;buffer to store other flags that we will use
flag_buff2:		1		;second generic flag buffer
flag_buff3:		1		;third generic flag buffer
manch_buff:		1		;buffer for flags
MAC_buff: 		1		;buffer for MAC layer related flags
timer_flags:	1		;buffer for timing related flags
tmr_on_flags:	1		;bu
;***Generic DSR Registers***
preamble_count:	1		;contains buffer flags that control layers
retry_counter: 	1		;1 byte to count retries
;ACK_requests:	1
rxbufflen:		1		;length of received bytes buffer
txbufflen: 		1		;length of packet to be sent
loadinglen:		1		;copy of txbufflen that is used to load the backup buffer
radioout_offset:1	;offset length of the byte we must transmit from the start of radioout
data_len: 		1		;length of data_bank data to be used
FSR0_H_TEMP: 	1		;to store FSR0 contents before entering sub-routine		
FSR0_L_TEMP: 	1		;ditto
highest_mem_h: 	1		;2 bytes to store the last location that we wrote a route to
highest_mem_l: 	1		;to store routing information
current_mem_h:	1		;used to track the memory location we are looking at to
current_mem_l:	1		;as above
replace_mem_l:	1		;used to mark the start of the erase procedure
replace_mem_h:	1		;as above
shuffle_buff:	1		;buffer to temorarily store the bytes from the FRAM
node_id: 		1		;This nodes ID address
latest_route_d: 1		;reg to store the latest RREQ ID
latest_route_s: 1		;reg to store the latest RREQ ID
target_addr: 	1		;Node that data packet must go to - detrermined outside this layer
RREQ_cause_addr:1
;***MAC Layer Registers***
backoff_level: 	1		;register to hold the level of back off before loading into backoff_timer
;***PARO registers***
RSSI_value: 	1		;Analog value of RSSI gained from AD conversion
tbl_RSSI_val:	1		;working register for holding the value of dB taken from lookup tables
tbl_dB_val:		1		;working register for holding the value of mW taken from lookup tables
work_dB_val:	1		;value in dB that we are working with
tbl_mW_val:		1
work_mW_val:	1
PARO_source: 	1		;the source of the packet that originated the PARO process
PARO_dest: 		1		;and the coressponding destination
Rx_power: 		1		;power level on reception
sent_power: 	1		;Power used on sending packet
reply_power:	1		;Transmission power on reply to sent packet
mod_power_A: 	1		;modified power on 1st hop
mod_A_mW:		1		;modified power for 1st hop converted to a mW reading
mod_power_B: 	1		;modified power on second hop
mod_B_mW:		1		;modified power for 2nd hop converted to a mW reading
total_init_pwr:	1		;total initial power
total_mod_pwr:	1		;total power of modified route
tbl_offset: 	1		;offset pointer by this value
min_rx_pwr: 	1
VGA_gain_set:	1
Tx_power_level:	1
RREQ_Tx_pwr:	1		;to store the power that we transmitted/forwarded the RREQ at so we can cache it on the way back
PARO_rt_id_d:	1
PARO_rt_id_s:	1
PARO_mod_loc_h:	1
PARO_mod_loc_l:	1
MOD_next_hop:	1
MOD_pwr:		1
RMOD_id_d:		1		;registers used to shuffle route ID's for memory cacheing when routes
RMOD_id_s:		1		;are modified
;***DSR AND PARO BUFFERS***
;radioin:		32		;radio input buffer - In data block for the meantime
;radioout:		32		;radio output buffer
;Used to Build Packets
dest_addr: 		1		;byte for the next address to hop to
source_addr:	1		;byte to store sender info
pkt_id: 		1		;byte to store the packet id
tx_pwr:			1
route_id_d: 	1		;reg to store the ID of current ROUTE - Destination
route_id_s:		1		;as above but for the source
hop_count:		1		;buffer to store the hop count for a RREQ packet
v_count_val:	1
v_count_diff:	1
;***Packet Identifiers***
RREQ:			1		;Code for Route Request
RREP:			1		;Route Reply
RUPD:			1		;Route Update - Maintenance
ACK:			1		;Acknowledge packet, for data transactions
DATA_PKT:		1		;data packet ID
RMOD: 			1		;route modify - from PARO
RMOD_ACK:		1		;also from PARO - yep change accepted TONTO
;***Physical Layer Registers***
coded_bits: 	1		;reserve space for the sampled input waveform
new_byte: 		1		;Reserve space to place all bits decoded from manchester into
temp_reg:		1		;temporary register used throughout for compare op's
current_bit: 	1		;bit to keep track of current symbol bit - a 1 in the current intersymbol bit location
bit_count:		1		;keep track of the number of bits decoded in byte
start_stop_bit: 1		;record of whether we are in the start/stop bit. A register so can be more that 1 period to give pause
symbol: 		1		;count of the symbols encoded (2, 1 or 0)
eor_bit: 		1		;bit used to manipulate the output
bit_tracker: 	1
send_byte: 		1		;PLACE BYTE TO TX NEXT IN HERE
receive_byte:	20		;buffer to store sample bytes in
sampled_bit:	1
wait_time:		1
rxbyte_offset:	1
fresh_byte:		1
current_sample:	1
temp_control:	1		;register to hold the layer control status so we can re-instate it once the Rx is through
timeout_counter:1
backup_len:		1
MAC_Stash_offset: 	1	;register to store to offset value that we have to load into from the start of radioin
WREG_Temp:		1
;****************************
;CRC Generation and Checking Registers
newData: 		1
crcLow:			1
crcHigh:		1
crcBytes:		1
Rx_Bytes:		1
Tx_Bytes:		1
receiveCRCLow:	1
receiveCRCHigh:	1
;****************************
;SPI Routine Registers
TXDATA:			1
RXDATA:			1
;****************************
;USART Registers
Tx_data_len:	1
Std_value:		1
ASCII_value:	1
USART_Tx_byte:	1		;byte to send over the UART
Low_String:		1
High_String:	1
;****************************
;***On-Board Route Storage***
current_on_board_h:		1
current_on_board_l:		1
max_location_h:			1
max_location_l:			1
local_search_pt_l:		1
local_search_pt_h:		1
on_board_route_space:	200
;*****************************************
;***DSR AND PARO BUFFERS***
radioin:		32		;radio input buffer - In data block for the meantime
radioout:		32		;radio output buffer
;***Other Buffers***
backup_buff:	32		;buffer to store last transmitted packet untill we get an acknowledge or RREP etc
No_route_buff:	32		;buffer to store packet whilst we are looking for a route for it
addr_buff:		12		;buffer to shuffle addresses accumulated in RREQ procedure
route_space:	10		;10 byte to stash routes in - will be replaced by FRAM
data_bank:		5		;bank to store data bytes for new transmissions
data_bank_ldr:	1
;Data Storage
data_store:				50
;*****************************************
;SOFTWARE SIMULATION EXTRAS
SIM_PACKET_INDEX:	1
SIM_INPUT_BUFFER:	32
;*****************************************

	ENDC
;*********************************************************************
	ORG	0x20
;************************
;PACKET IDS
RREQ_V			equ	0x01
RREP_V			equ	0x02
DATA_PKT_V		equ	0x04
ACK_V			equ	0x05
RMOD_ACK_V		equ	0x06
RMOD_V			equ	0x07
;************************
;PARO Control Values
STOP_INC		equ	0x00

;************************
;***dsr_flags flag bit definitions***
RREQ_SENT		equ	0x00	;flag to signal one of our RREQ packets is out there somewhere
INITIATE		equ 0x01	;initiate a data transaction
PACKET_ARRIVE	equ 0x02	;packet has arrived, process it
PACKET_SEND		equ 0x03	;packet ready for sending
USE_ADDR_BUFF	equ	0x04	;use the addresses stashed in the addr_buff
SEND_PACKET		equ	0x05	;flag to mark that we need to initiate a packet send
NEW_PACKET		equ 0x06	;flag to mark a new packet so BUILD_PACKET transferres from data bank, not radioin
FOUND_ROUTE		equ	0x07	;flag to signal that we found a route to the destination from MEM_SEARCH
;***********************
;***flag_buff flag bit definitions***
BACKOFF_ON		equ	0x00	;flag bit to signal we are in the middle of MAC backoff
STOPPEDNWAITING	equ	0x01	;flag to signal we are busy with a PARO calc already
;***Layer Control Flag Bit Definitions***
MAC				equ	0x02
Physical		equ	0x03
Network			equ 0x04
Data_Link		equ	0x05
;***Networking Layer extras***
ACK_SEND		equ	0x06
DATA_RESEND		equ	0x07
;***********************
;Manchester Encode/Decode Flag buffer definitions
PACKET_DONE		equ	0x00	
STOP_BIT_NOW	equ	0x01	
SPARE1			equ	0x02	
BYTE_DONE		equ	0x03	;the manchester decoded byte is done
BIT_DONE		equ	0x04	;we have assembled 8 samples to form 1 bit
CURRENT_BIT		equ	0x05
RAISE_SPECIAL	equ	0x06
MAC_WANTS_BACK	equ	0x07
;***********************
;ERROR flags for use all over
BUFFER_ORUN		equ	0x00
MANCH_ERROR		equ	0x01
NO_MEM_ERROR	equ	0x02	;location within flag_buffer for startbit error
ON_BOARD_FULL	equ	0x03	;location within flag_buffer for stopbit error
FRAME_ERROR		equ	0x04	;location within flag_buffer for framing error
CRC_ERROR		equ	0x05
CRITICAL_ERROR	equ	0x06
;***********************
;MAC_flag bit definitions (for MAC and DSR layers)
BYTE_IN			equ	0x01	;we have a byte coming into the MAC layer - into radioin
PACKET_OUT		equ	0x02	;the MAC layer needs to send a packet out - from radioout
MEDIA_BUSY		equ 0x03	;flag to signal CD was high
RREQ_STORE		equ	0x04	;we have data in No_route_buff - must service once we get a RREP
BACKUP_FULL		equ	0x05	;have data in the holding buffer - clear once we get an ACK
CONTROL_IS_OURS	equ	0x06
RREQ_SEARCH		equ	0x07
;***********************
;Timer_flags bit definitions
GENERAL_TIMEOUT	equ	0x00
RREQ_TIMEOUT	equ	0x01
ACK_TIMEOUT		equ	0x02
BACKOFF_TIMEOUT	equ	0x03
ROUTE_MEM_TIME	equ	0x04
PACKET_RETRY	equ	0x05
ROUTE_EXPIRED	equ	0x06	
;***********************
;tmr_on_flags bit definitions
BACKOFF_ON		equ	0x00
RREQ_TIMEOUT_ON	equ	0x01
ACK_TIMEOUT_ON	equ	0x02
RREQ_ID_TMR_ON	equ	0x03
RELAY_WAIT		equ	0x04	
;***********************
;flag_buff2 definitions
FIRST_NON_DATA	equ	0x00
FOUND_ONBOARD	equ	0x01
RECEIVE_CRC		equ 0x02
TRANSMIT_CRC	equ	0x03
NOTIFY_STG1		equ	0x04
PARO_BUILD		equ	0x05
PARO_ON			equ	0x06
NEG_1ST_LEG		equ	0x07
;***********************
;flag_buff3 definitions
RMOD_ACK_SEND	equ	0x00
INCOMMING_PKT	equ	0x01
;***********************
;SPI FRAM Commands
WREN			equ	B'00000110'
WRDI			equ	B'00000100'
RDSR			equ	B'00000101'
WRSR			equ	B'00000001'
READ			equ	B'00000011'
WRITE			equ	B'00000010'
;***********************
BIT0			equ	0x00
BIT1			equ	0x01
BIT2			equ	0x02
BIT3			equ	0x03
BIT4			equ	0x04
BIT5			equ	0x05
BIT6			equ	0x06
BIT7			equ	0x07
;***********************
;Transceiver Control Pins - PORTB
RX_ENABLE		equ	0x03	;Receiver Enable - Active LOW
TX_ENABLE		equ	0x04	;Transmitter Enable - Active LOW
;***********************
;VGA Control Pins and words - PORTD
VGA_ON			equ	0x05
GAIN_WRITE		equ	0x04
;Gain Control Words - XOR'ed with port
;Set all gain to attenuations for benck top testing so we don't
;overload the front end of the receivers
GAIN_14		equ	0x00
;Pin for the Output Relay
RELAY_CLOSE		equ	0x06
;***********************
;Port Pin Labels - PORTC
NCS				equ	0x00
NWP				equ	0x01
HOLD			equ	0x02
;***********************
;Port Pin Labels - PORTB
RX_LED			equ 0x06
TX_LED			equ	0x07
;***********************
;MACROS
RESTART_Rx	macro
	bsf		INTCON3, INT2IE				;Re-enable the INT2 IRP
	clrf	flag_buff
	bsf		flag_buff, Physical
	endm

RESET_RADIOIN macro
	movlw 	high radioin 				;otherwise, if its greater than the holding length
	movwf 	FSR0H 						;reset the counters to the start of the buffer
	movlw 	low radioin 				;Move the low byte of the RCREG address into the FSR1
	movwf 	FSR0L
	endm

CS_SETUP macro
	bsf		PORTC, NCS					;raise chip select - disarm device
	nop									;only needs 90ns but give it 2 mic's
	nop
	bcf		PORTC, NCS					;and clear again to activate device
	endm

INCREMENT_MEM_TRACKER macro
	incf	current_mem_l, 1			;increment low byte	memory marker
	btfsc	STATUS, C					;if it overflowed
	incf	current_mem_h, 1			;increment high byte as well; otherwise.....
	endm

INC_LOCAL_TRACKER macro
	incf	local_search_pt_l, 1		;as for above
	btfsc	STATUS, C
	incf	local_search_pt_h, 1
	endm

SET_SAMPLE_ZERO macro
	comf	current_bit, 0				;take the inversion of the current bit tracker
	andwf	fresh_byte, 0				;AND to clear the bit
	movwf	fresh_byte
	endm

SET_SAMPLE_ONE macro
	movf	current_bit, 0				;move in the current bit
	xorwf	fresh_byte, 0				;xor with a one in current sample location to set it
	movwf	fresh_byte					;and store again
	endm

TRANSMIT_ENABLE:
	bcf		INTCON3, INT2IE				;Turn off the Incoming data Interrupt
	bsf		LATD, VGA_ON				;Turn on the VGA...
	bsf		LATD, GAIN_WRITE			;tell it we are gonna set the gain
	movf	LATD, 0					;load up port D
	andlw	B'11110000'					;clear the lower nibble of PORTD
	xorwf	VGA_gain_set, 0				;XOR to set the lower bits that we need to
	movwf	LATD						;and store it
	bcf		LATD, GAIN_WRITE			;turn on the gain write function
	nop									;wait for it to take effect
	bsf		LATD, GAIN_WRITE			;and then turn it off
	bsf		tmr_on_flags, RELAY_WAIT	;set a flag to hold us out
	clrf	TMR2						;clear the timer register
	bsf		PIE1, TMR2IE				;enable the overflow IRP
	bsf		T2CON, TMR2ON				;and turn it on
	bsf		LATD, RELAY_CLOSE
WAIT_CLOSE:
	btfsc	tmr_on_flags, RELAY_WAIT	;is the bit still set?
	bra		WAIT_CLOSE					;yep, relay wouldn't have closed yet
	bsf		PORTB, RX_ENABLE			;Disable the receiver and...
	bcf		PORTB, TX_ENABLE			;Enable the transmitter
	return
	
RECEIVE_ENABLE macro
	bcf		INTCON3, INT2IE				;Disable the Rising Edge IRP - Account for garbage sent out.
	bcf		PORTD, VGA_ON				;Turn off the VGA
	bsf		PORTB, TX_ENABLE			;Set high to disable Transmitter
	bcf		PORTB, RX_ENABLE			;Take Low to enable Receiver
	bcf		PORTD, RELAY_CLOSE
	nop
	bsf		INTCON3, INT2IE
	endm

ADDR_SET macro addr
	movlw	low addr
	movwf	FSR0L
	movlw	high addr
	movwf	FSR0H
	movlw	0x07
	addwf	FSR0L, 1
	btfsc	STATUS, C
	incf	FSR0H, 1
	endm

STR_ADDR_SET macro addr
	movlw	low addr
	movwf	Low_String
	movlw	high addr
	movwf	High_String
	endm

NEW_LINE macro
	STR_ADDR_SET strLF
	call	SEND_STRING
	STR_ADDR_SET strCR
	call	SEND_STRING
	endm
;***********************************************************************

DISABLE_TRANSCEIVER macro
	bcf		PORTD, VGA_ON				;Turn off the VGA
	bsf		PORTB, TX_ENABLE
	bsf		PORTB, RX_ENABLE
	bcf		INTCON3, INT2IE				;Disable the incoming data IRP
	endm
;***********************
device_init:
;Port A - A/D Converter
	clrf	LATA
	movlw 	B'10000001'		;Turn A/D on, converting at Fosc/32. Test bit 2 for "conversion done"
	movwf	ADCON0
	movlw	B'10001000'		;Right justify, last bit of conversion speed select and select Ref. Pins.
	movwf	ADCON1
	clrf	PORTA
	bsf		TRISA, RA0		;Select RA0 as an input
;Port B - Data in and Out and IRP's (set up later)
	clrf	LATB
	clrf	PORTB
	movlw	B'00000110'
	movwf	TRISB			;Select INT1 and INT2 as input. Rest are outputs
;Port C - To MAX232 for PC comms.
	clrf	LATB
	clrf	PORTB
	movlw	B'10010000'		;set up SPI outputs and USART outputs
	movwf	TRISC
	;USART set-up
	movlw	D'103'			;Scaling factor to deliver 2400 Baud with BRGH = 1
	movwf	SPBRG			;Store in the Baud Rate Generator Scaler Register
	movlw	B'00100100'		;Enable trasmit, synch and BRGH = 1
	movwf	TXSTA
	movlw	B'10010000'		;Enable serial port, in 8 bit mode with continuous receive
	movwf	RCSTA
	;SPI set-up
	movlw	B'11000000'		;Sample at end of period and transfer on rising edge
	movwf	SSPSTAT
	movlw	B'00100000'		;enable SPI pins, idle low for clk and SPI master
	movwf	SSPCON1
	clrf	SSPCON2			;don't care, only used in I^2C
;Port D - To VGA
	clrf	LATD
	clrf	TRISD
	clrf	TRISE			;All port E pins are outputs
;Enable Interupts
	clrf	INTCON
	bsf		RCON, IPEN
	;external IRP's
	movlw	B'10011010'		;INT2 on rising edge
	movwf	INTCON2
	movlw	B'11010000'		;INT1 & 2 high priority, and enable INT2. Test Bits 0 and 1 for INT1 & 2 flags.
	movwf	INTCON3
	;peripheral IRP's
;	movlw	(1<<TMR2IE)		;enable TMR1 Overflow IRP, disable CCP1 IRP at the moment
	movlw	0x00
	movwf	PIE1
	movlw	0x00			;Disable CCP2 and TMR3 o'flow, bus collision and low V IRP's
	movwf	PIE2
	;Peripheral IRP Priorities
	movlw	(1<<CCP1IP)		;CCP1 IRP = High Priority, TMR1 O'low = Low Priority
	movwf	IPR1
	movlw	(1<<CCP2IP)		;CCP2 IRP = High Priority
	movwf	IPR2
;Set Up Timers and Comparators for Physical Layer
	movlw	B'00001010'		;Generate a software IRP on each CCP1 compare match
	movwf	CCP1CON
	movlw	B'00001010'		;Generate a software IRP on each CCP2 compare match
	movwf	CCP2CON
	movlw	B'01100101'		;Select TMR3 for all CCP, use internal clk, enable TMR3 with a 1:1 ratio (f = Fosc/4)
	movwf	T3CON
;	movlw	B'11111101'		;Select TMR3 for all CCP, use internal clk, enable TMR3 with a 1:8 ratio (f = Fosc/32)
;	movwf	T3CON
;TIMER 0 FOR PACKET TIMEOUTS
	movlw	B'01000111'		;Set Up Timer 0 with a 1:256 pre-scale, on each inst. cycle at 8 bits - 65.536msec per o'flow
	movwf	T0CON
;TIMER 2 FOR RELAY CLOSING
	movlw	B'00111001'		;Set up counter for Fosc/24 and off for now. This = overflow every 6.144 milliseconds
	movwf	T2CON

	DISABLE_TRANSCEIVER
;*****************************
	clrf	error_flags		;just make sure all the flag buffers are empty
	clrf	dsr_flags		
	clrf	flag_buff
	clrf	flag_buff2
	clrf	manch_buff
	clrf	MAC_buff
	clrf	timer_flags
	clrf	validity_count	;reset the route validity counter
	clrf	radioout_offset
	movlw	0x05			;load up the preamble counter
	movwf	preamble_count
	movlw	0x03			;and the retry counter
	movwf	retry_counter
	movlw	0x80
	movwf	sleep_count

	movlw	low on_board_route_space
	addlw	D'200'
	movwf	max_location_l
	btfsc	STATUS, C
	incf	max_location_h, 1
	movlw	low on_board_route_space
	movwf	current_on_board_l
	movlw	high on_board_route_space
	movwf	current_on_board_h

;Set Up data Table for packet ID values
	movlw	RREQ_V
	movwf	RREQ
	movlw	RREP_V
	movwf	RREP
	movlw	DATA_PKT_V
	movwf	DATA_PKT
	movlw	ACK_V
	movwf	ACK
	movlw	RMOD_ACK_V
	movwf	RMOD_ACK
	movlw	RMOD_V
	movwf	RMOD
;****************************************
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;****************************************
	call	SIM_SETUP
;	call	SIM_LOAD_UP
;***********************************
;Enable Interupts
	movlw	B'11000000'		;Enable all IRP's, peripheral IRP's, disable TMR0, INT0 and RBx Change IRP's
	movwf	INTCON
;****************************************
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;****************************************
;******VGA Testing ONLY*****************
;	movlw	0xAA
;	movwf	send_byte
;	bsf		PORTB, RX_ENABLE			;Disable the receiver and...
;	bcf		PORTB, TX_ENABLE			;Enable the transmitter
;	bcf		INTCON3, INT2IE
;	bsf		flag_buff, Physical
;	bsf		dsr_flags, INITIATE
;	movlw	0x01						;Gain setting value - 0x01 ??????
;	movwf	VGA_gain_set				;move it in
;	call	TRANSMIT_ENABLE
;****************************
	RECEIVE_ENABLE
EXEC_LOOP:
	clrwdt
	call BYTE_Tx
	btfsc	flag_buff, Physical
	bra		PHYSICAL_LAYER_LOOP
	btfsc	flag_buff, MAC
	bra		MAC_LAYER_LOOP
	btfsc	flag_buff, Data_Link
	bra		DATA_LINK_LAYER_LOOP
	btfsc	flag_buff, Network
	bra		NETWORK_LAYER_LOOP
	btfsc	timer_flags, GENERAL_TIMEOUT
	call	TIMEOUT_HANDLER
	decf	sleep_count, 1
	bnz		EXEC_LOOP
	movlw	0x80
	movwf	sleep_count
	bcf		WDTCON, SWDTEN
	sleep
	bra		EXEC_LOOP
;****************************************
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;****************************************
;**********PHYSICAL LAYER LOOP***********
PHYSICAL_LAYER_LOOP:
	btfsc	dsr_flags, INITIATE
	call	BYTE_Tx
	btfss	flag_buff, Physical			;if flag is still set but we aren't initiating - waiting for an Rx
	bra		EXEC_LOOP
	bra		PHYSICAL_LAYER_LOOP
;****************************************
;*************MAC LAYER LOOP*************
MAC_LAYER_LOOP:
	call	MAC_LAYER
	bra		EXEC_LOOP
;****************************************
;*************DATA LINK LAYER LOOP*************
DATA_LINK_LAYER_LOOP:
	btfsc	flag_buff, Data_Link
	call	DATA_LINK_LAYER
	bra		EXEC_LOOP
;****************************************
;****************************************
;***********NETWORK LAYER LOOP***********
NETWORK_LAYER_LOOP:
	btfsc	flag_buff, Network
	call	NETWORK_LAYER
	bra		EXEC_LOOP
;***********************************************************************
;***********************************************************************		
;ISR's
IRPSERVICE:
	clrwdt
	call 	Tst_INT2_Edge						; Check the edge
	call 	Tst_CCP2							; Test timer for Manchester Decoding
	call 	Tst_CCP1							; Test timer for Manchester Encoding
	retfie 1
;***********************************************************************
LOW_PRIORITY:
	clrwdt
	call	Tst_TMR0
	call	Tst_TMR2
END_LOW:
	retfie 1
;***********************************************************************
;;**********************************************************************
Tst_INT2_Edge:
	btfss	INTCON3, INT2IF				;was the interrupt from INT2 - rising edge on Rx Data
	return								; No
	bcf		INTCON3, INT2IF				;yep, if it wasn't enabled it didn't trigger but it may still av happened - clear the flag
    btfss   INTCON3, INT2IE				; Was the interrupt enabled
	return								; No return!
	bcf		INTCON3, INT2IF			    ;clear the flag
	bcf		INTCON3, INT2IE				; Yep, it was INT2. Now, disable it cause timing will handle it from here
	call	RX_BYTE_SETUP
	movlw	D'30'
	movwf	wait_time
	call	CCP2_INC
	bcf 	PIR2, CCP2IF				; Clear the flag
	bsf 	PIE2, CCP2IE				; Turn on the ouptut compare IRP on CCP2
	movlw	0x80
	movwf	current_bit
	btfss	ADCON0, ADON				;Is the A/D converter already on - 1st run through?
	bsf		ADCON0, ADON				;No, then turn on A/D converter
	bsf		ADCON0, 2					;and initiate the conversion
	clrf	sampled_bit
	clrf	flag_buff					;and now rob control
	bsf		flag_buff, Physical			;Retain control in the Physical Layer
	return								; Leave	
;***********************************************************************
Tst_CCP2:			;MANCHESTER DECODING
	btfss	PIR2, CCP2IF				; Was it a CCP Interrupt
	return								; No
	bcf		PIR2, CCP2IF				;clear the flag
	btfss	PIE2,CCP2IE					; Was it enabled
	return								; No!
	bcf 	PIE2,CCP2IE					; Disable compare interrupt
	bcf		PIR2, CCP2IF				; Clear the flag
	movlw	D'95'						;load the offset
	movwf	wait_time					
	call	CCP2_INC					;and increment the CCP before doing the math
	bsf		PIE2, CCP2IE
	call	SAMPLE_DATA					;go and take a sample	
END_TEST_CCP:
	return								; Done	
;***********************************************************************
Tst_CCP1:			;MANCHESTER ENCODING
	btfss	PIR1, CCP1IF				;did the IRP come from CCP1 
	return								;No
	bcf		PIR1, CCP1IF				;clear the flag
	btfss	PIE1, CCP1IE				;Was it enabled
	return								;No
CCP1_IRP:
	bcf		PIR1, CCP1IF				;clear the IRP flag
	movf	start_stop_bit, 1			;set condition codes based on this flag
	bnz		SPECIAL_BIT					;if this isn't clear, need to deal with a start of stop bit
	decf	symbol, 1					;else, decrement the symbol counter
	movf	eor_bit, 0					;load up the exor bit
	xorlw	(1<<RB5)					;xor with 1 in the output pin position in PORTB to toggle bit
	movwf	eor_bit						;store it again
	bra		SET_OUTPUT
SPECIAL_BIT:
	btfsc	manch_buff, STOP_BIT_NOW
	bra		RAISE_STOP_BIT
	bcf		PORTB, RB5					;no, clear it
	decf	start_stop_bit, 1			;work towards clearing this counter.
	movf	eor_bit, 0					;load up the exor bit
	xorlw	(1<<RB5)					;xor with 1 in the output pin position in PORTB to toggle bit
	movwf	eor_bit						;store it again
	bra		FINAL_END
RAISE_STOP_BIT:
	bsf		PORTB, RB5
	decf	start_stop_bit, 1
	bcf		manch_buff, STOP_BIT_NOW
	movf	eor_bit, 0					;load up the exor bit
	xorlw	(1<<RB5)					;xor with 1 in the output pin position in PORTB to toggle bit
	movwf	eor_bit						;store it again
	bra		FINAL_END
SET_OUTPUT:
	movf	current_bit, 0
	xorwf	eor_bit, 0
	movwf	PORTB
FINAL_END:
	movf	CCPR1L, 0
	addlw	d'52'						;increment to the middle of the next bits first symbol (1 bit period later)
	movwf	CCPR1L
	btfss	STATUS, C					;if there was no carry, skip on
	bra		END_HIGH_IRP
	incf	CCPR1H,	1					;Other wise increment the high byte
END_HIGH_IRP:		
	return
;***********************************************************************
Tst_TMR2:			;TIMERS AND TIMEOUTS
	btfss	PIR1, TMR2IF				;Timer 2 overflow will occur every 6.144 milliseconds
	return			
	bcf		PIR1, TMR2IF				;clear the interrupt flag		
	btfss	PIE1, TMR2IE			
	return
	bcf		tmr_on_flags, RELAY_WAIT	;release control to proceed with transmission
	bcf		PIE1, TMR2IE
	bcf		T2CON, TMR2ON
	return
;***********************************************************************
Tst_TMR0:			;TIMERS AND TIMEOUTS
	btfss	INTCON, TMR0IF				;Did the timer overflow?
	return								;nop, get out
	bcf		INTCON, TMR0IF				;yep, clear the interrupt flag		
	btfss	INTCON, TMR0IE			
	return
	decf	timeout_counter, 1			;decrement the counter
	bnz		END_TMR0_IRP
	bsf		timer_flags, GENERAL_TIMEOUT
	bcf		INTCON, TMR0IE				;disable the overflow IRP
	bcf		T0CON, TMR0ON				;and turn off the timer
	movlw	D'20'
	movwf	timeout_counter
END_TMR0_IRP:
	retfie 1
;***********************************************************************
RX_BYTE_SETUP:
	clrf	rxbyte_offset
	movlw	0x80
	movwf	current_bit
	return
;************************
CCP2_INC:
	bcf 	T3CON,TMR3ON				; Turn it off
    nop									; Wait for it
    nop									; wait
    movf    TMR3H,0						; Get the high byte
    movwf   CCPR2H						; Set the MSB 
	movf	TMR3L, 0
	addwf	wait_time, 0				;increment to the middle of first symbol (1 bit period later)
	movwf	CCPR2L
	btfsc 	STATUS,C					; Was carry set
    incf 	CCPR2H,1					; Yes, increment
	bsf 	T3CON,TMR3ON				; Turn it on
	return	
;***********************************************************************
BYTE_Tx:
	bsf		PORTB, TX_LED
	bcf		dsr_flags, INITIATE			;we're here now so clear the flag
	movlw	(1<<RB5)
	movwf	current_bit
	call	CLK_STRT					;start the ccp1 ouptut compare process
	bsf		PORTB, RB5					;raise the output
	clrf	bit_tracker					;reset the bit tracker
	movlw	0x01
	movwf	symbol
	movwf	start_stop_bit
	clrf	eor_bit						;clear the bit used for exclusive or'ing
START_LOOP:
	movf	start_stop_bit, 1			;load up the counter to set the condition codes
	bnz		START_LOOP					;if its not zero, loop
START_DATA:
	movlw	0x80
	movwf	bit_tracker					;reset the bit_tracker operator
LOOP1:
	movlw	0x02
	movwf	symbol						;load the symbol counter over itself to set condition codes
	movf	send_byte, 0				;load the byte we are sending
	andwf	bit_tracker, 0				;and set condition codes based on current bit
	bnz		TX_ONE						;if the result is not zero, send a one
	clrf	current_bit
	bra		LOOPING
TX_ONE:
	bsf		current_bit, RB5			;set a one in the place of RB5 in current_bit operator
LOOPING:
	movf	symbol, 1					;have we sent out all symbols?
	bnz		LOOPING						;nope, wait till we have
	rrcf	bit_tracker, 1				;yes, we have sent all, now move on to the next bit in the byte
	btfsc	STATUS, C
	bra		STOP_BIT					;if it overflowed, then we have finished the data byte
	bra		LOOP1						;otherwise, do another bit
STOP_BIT:
	bsf		manch_buff, STOP_BIT_NOW
	movlw	0x08
	movwf	start_stop_bit
STOP_LOOP:
	movf	start_stop_bit, 1
	bnz		STOP_LOOP					;loop here till stop bit is finished
	bcf		PORTB, TX_LED
	btfsc	manch_buff, MAC_WANTS_BACK	;Does the MAC layer wants control again?
	bra		BACK_TO_MAC
	bra		RELEASE_PHYS_CTRL			;Nope, just release it
BACK_TO_MAC:
;***VGA TESTING***
;	bsf		flag_buff, Physical
;	bsf		dsr_flags, INITIATE
;*****************
	bsf		flag_buff, MAC				;Yep give control back to MAC layer	
	bcf		flag_buff, Physical			;and release our stangle hold
	bcf		PIE1, CCP1IE				;and turn off the interrupt
	return
RELEASE_PHYS_CTRL:
;***VGA TESTING***
;	bsf		flag_buff, Physical
;	bsf		dsr_flags, INITIATE
;*****************
	bcf		flag_buff, Physical			;and release our stangle hold
	bcf		PIE1, CCP1IE
	RECEIVE_ENABLE						;and let the receiver take over
	call	START_REPLY_TIMER			;call the routine that will start the timer for the reply
	return
;***********************************************************************
CLK_STRT:
	bcf		T3CON, TMR3ON				;Stop the clock to eliminate pre-fetching
	nop									;and wait a while
	nop
	movf	TMR3H, 0					;catch the high byte
	movwf	CCPR1H
	movf	TMR3L, 0					;and the low
	addlw	d'52'						;increment to the middle of the next bits first symbol (1 bit period later)
	movwf	CCPR1L
	btfss	STATUS, C					;if there was no carry, skip on
	bra		END_CLK_STRT
	incf	CCPR1H, 1					;otherwise, increment the high byte
END_CLK_STRT:
	bcf		PIR1, CCP1IF				;clear the flag first, then
	bsf		PIE1, CCP1IE				;enable the IRP
	bsf		T3CON, TMR3ON
	return
;***********************************************************************
SAMPLE_DATA:
	bsf		PORTB, RB5				;for diagnostic purposes only
SAMPLE_START:
	btfss	PORTB, RB2					;is the input a high
	bra		SAMPLE_ZERO					;no, then mark sample as a zero
SAMPLE_ONE:								;yes, mark as a one
	movlw	0xFF
	movwf	sampled_bit
	bra		FINISH_SAMPLE				;and move on
SAMPLE_ZERO:
	movlw	0xEE
	movwf	sampled_bit
FINISH_SAMPLE:
	call	ADD_SAMPLED_BIT
END_SAMPLE:
	bcf		PORTB, RB5				;for diagnostic purposes only
	return	
;***********************************************************************
ADD_SAMPLED_BIT:
;satsh the sample in the buffer PLUSWX location and
;increment the offset. If offset is ten then reset and reenable the INT2
;IRP.
	movlw	low	receive_byte			;load up the start of the buffer
	movwf	FSR0L
	movlw	high receive_byte
	movwf	FSR0H
	movf	rxbyte_offset, 0			;load up the offset
	addwf	FSR0L, 0
	movwf	FSR0L
	btfsc	STATUS, C
	incf	FSR0H, 1
	movf	sampled_bit, 0
	movwf	INDF0						;move in the sampled bit stream
	incf	rxbyte_offset, 1			;increment the offset
	movlw	D'20'
	cpfseq	rxbyte_offset
	bra		RESTART_SAMPLING	
	clrf	rxbyte_offset
	bcf		PIR2, CCP2IF
	bcf		PIE2, CCP2IE
	call	FORM_BYTE
	bcf		INTCON3, INT2IF
	bsf		INTCON3, INT2IE
RESTART_SAMPLING:
	return
;***********************************************************************
FORM_BYTE:
	movlw	low	receive_byte			;load up the start of the buffer
	movwf	FSR0L
	movlw	high receive_byte
	movwf	FSR0H
	clrf	rxbyte_offset
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xFF
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
	incf	rxbyte_offset, 1
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xEE
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
	incf	rxbyte_offset, 1
	movlw	0x80
	movwf	current_bit
	clrf	fresh_byte
DATA_LOOP:
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xFF
	cpfseq	current_sample
	bra		ZERO_TEST
	incf	rxbyte_offset, 1
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xEE
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
	incf	rxbyte_offset, 1
	SET_SAMPLE_ONE
	bra		NEXT_BIT
ZERO_TEST:
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xEE
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
	incf	rxbyte_offset, 1
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xFF
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
	incf	rxbyte_offset, 1	
	SET_SAMPLE_ZERO
NEXT_BIT:
	rrcf	current_bit, 1
	bnc		DATA_LOOP
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xFF
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
	incf	rxbyte_offset, 1
	movf	rxbyte_offset, 0
	movf	PLUSW0, 0
	movwf	current_sample
	movlw	0xEE
	cpfseq	current_sample
	bsf		error_flags, MANCH_ERROR
END_BYTE_FORM:
	bsf		flag_buff, MAC				;pass control to MAC layer
	bcf		flag_buff, Physical			;and release control
	bsf		MAC_buff, BYTE_IN			;and signal that there is a fresh packet
	call	FSR0_UNSTACK
	bcf		PORTB, RX_LED
	return
;***********************************************************************
;***********************************************************************
MAC_LAYER:
	call	FSR0_STACK
	btfsc	error_flags, MANCH_ERROR	;was there an error in the manchester decoding process
	bra		ERROR_DECODE				;if ANY error has occured, dont stash it.
	btfsc	MAC_buff, BYTE_IN			;is the byte coming in?
	bra		MAC_BYTE_IN					;yep, process it
	btfsc	MAC_buff, PACKET_OUT		;nope, is it going out?
	bra		MAC_PACKET_OUT				;yep, get it outta here
	bcf		flag_buff, MAC
	return
MAC_BYTE_IN:
	bcf		MAC_buff, BYTE_IN			;clear the reason we got here
	movf	preamble_count, 1			;copy to set condition codes
	bnz		PREAMBLE_IN
	bra		PREAMBLE_SATISFIED			;its still zero so go onward good man
PREAMBLE_IN:
	clrf	MAC_Stash_offset			;reset the offset pointer
	movlw	0xAA						;load up the preamble value
	cpfseq	fresh_byte					;compare to the value of the input byte
	bra		PREAMBLE_SHAGGED			;if not equal, go and do something?????
	clrf	rxbufflen					;cause this is the pre-amble, clear the offset ready to load in again
	decf	preamble_count, 1			;decrement the tracker and store it again - nothing with flags cause Rx is all under IRP
	bcf		flag_buff, MAC				;get out of MAC loop
	bsf		flag_buff, Physical			;and smash about in the physical layer loop
	RESTART_Rx
;	call	SIM_LOAD_UP					;*******************************************		
	return
PREAMBLE_SHAGGED:
	movlw	0x03
	movwf	preamble_count				;reset the preamble counter
	call	FSR0_UNSTACK				;unstack FSR
	bcf		flag_buff, MAC				;release control
;	call	SIM_LOAD_UP					;*******************************************
	return								;and get out
PREAMBLE_SATISFIED:				;getting here means we have recieved the minimum number of preamble bytes
	movlw	0xAA						;is the next byte a special byte?
	cpfseq	fresh_byte
	bra		NOT_SPECIAL					;nope, then proceed and load in the data packet
	movf	rxbufflen, 1				;load up the Rx buffer length
	bnz		TERMINATING_CHAR			;if its not zero then special char occured AFTER data reception
;	call	SIM_LOAD_UP					;*******************************************
	return								;if we haven't yet got a non 0xAA byte then this is excess preamble - get out	
TERMINATING_CHAR:
	movlw	0x05						;otherwise it was a terminating char so....
	movwf	preamble_count				;reset the preamble counter
	bcf		flag_buff, MAC				;get out of MAC loop
	bsf		flag_buff, Data_Link		;pass control to the Data Link layer
	bsf		flag_buff2, RECEIVE_CRC		;for calculation of CRC on a reception basis
	return
NOT_SPECIAL:
	movlw	D'32'						;load a value equal to the receive buffer length (radioin)
	cpfsgt	rxbufflen					;compare to the current length of actual characters
	bra		JUST_STASH					;if we haven't yet reached capacity, just stash the value
	RESET_RADIOIN
	bsf		error_flags, BUFFER_ORUN	;and mark that we had an overrun
JUST_STASH:
	RESET_RADIOIN
	movf	MAC_Stash_offset, 0	
	addwf	FSR0L, 0
	movwf	FSR0L
	btfsc	STATUS, C
	incf	FSR0H, 1
	movf	fresh_byte, 0
	movwf	INDF0
	incf 	rxbufflen, 1				;Increment the buffer length counter
	incf	MAC_Stash_offset, 1
	RESTART_Rx
	call	FSR0_UNSTACK
;	call	SIM_LOAD_UP					;*******************************************
	return
ERROR_DECODE:
	bcf		error_flags, MANCH_ERROR
	clrf	MAC_Stash_offset
	bcf		flag_buff, MAC
	call	FSR0_UNSTACK
	return
MAC_PACKET_OUT:
	btfsc	MAC_buff, CONTROL_IS_OURS	;do we have the Media?
	bra		CARRIER_FREE				;Yep, go and do it
	btfss	PORTB, RB1					;no, this must be the first time round. Test the carrier sense bit
	bra		CARRIER_FREE
	bsf		MAC_buff, MEDIA_BUSY		;set the flag that signals the media is in contention
	movlw	0x01
	movwf	backoff_level				;set the initial backoff level
	movff	backoff_level, backoff_timer
	bsf		flag_buff, MAC				;be sure we retain control in this layer untill packet is gooooooone
	return
CARRIER_FREE:
	call	TRANSMIT_ENABLE
	bsf		MAC_buff, CONTROL_IS_OURS	;Mark we have the Media
	movf	preamble_count, 0			;load up the preamble counter	
	bz		PREAMBLE_SENT				;if its gooooone then skip on
	movlw	0xAA						;otherwise, load the preamble value	
	movwf	send_byte					;put it in the passing register
	decf	preamble_count, 1			;decrement the counter
	bsf		flag_buff, Physical			;pass control to physical layer
	bsf		dsr_flags, INITIATE			;and double check it will go where we want it
	bsf		manch_buff, MAC_WANTS_BACK	;mark that the MAC layer wants control again
	return
PREAMBLE_SENT:
	movlw 	high radioout 				;set the pointers to the start of the buffer
	movwf 	FSR0H 					
	movlw 	low radioout				
	movwf 	FSR0L
	movf	radioout_offset, 0
	movf	PLUSW0, 0
	movwf	send_byte					;move the next Tx byte into the register to pass to physical layer	
	incf	radioout_offset, 1			;increment the offset pointer
	decf	txbufflen, 1				;decrement the Tx buffer length indicator
	bnn		TX_BUFF_NOT_EMPTY			;branch to tx buffer not empty if its not
	movlw	0xAA						;load up the terminating character
	movwf	send_byte					;and set him up to be sent
	bcf		flag_buff, MAC				;let go of the token; and
	bsf		flag_buff, Physical			;pass control to physical layer
	bsf		dsr_flags, INITIATE			;and double check it will go where we want it - BYTE_Tx
	bcf		manch_buff, MAC_WANTS_BACK	;mark that the MAC no longer needs control - we're done
	bcf		MAC_buff, PACKET_OUT		;remove flag telling the MAC layer that the packet will be leaving it
	clrf	radioout_offset				;reset the offset pointer as well
	call	START_REPLY_TIMER			;and start the timer for the return of a reply
	return
TX_BUFF_NOT_EMPTY:
	bsf		flag_buff, Physical			;pass control to physical layer
	bsf		dsr_flags, INITIATE			;and double check it will go where we want it - BYTE_Tx
	bsf		manch_buff, MAC_WANTS_BACK	;mark that the MAC layer wants control again
	return
;***********************************************************************
DATA_LINK_LAYER:
	call	FSR0_STACK
	bcf		flag_buff, Data_Link
	btfsc	flag_buff2, RECEIVE_CRC		;Did we get here on a receive packet command??
	bra		CHECK_CRC					;yep, go and do it
	btfss	flag_buff2, TRANSMIT_CRC	;no, was it cause we need to build and append it??
	return								;no, dunno why we got here, but get out now
	bra		CREATE_CRC
CHECK_CRC:
	movlw	high radioin
	movwf	FSR0H
	movlw	low radioin
	movwf	FSR0L
	bcf		flag_buff2, RECEIVE_CRC
	movlw	0x02						;load up two
	subwf	rxbufflen, 0				;and subtract from the overall number of received bytes - remove CRC
	movwf	Rx_Bytes					;and store it for later use
	movf	PLUSW0, 0					;use it as an offset from the first byte to pull out the high CRC byte
	movwf	receiveCRCHigh				;and store it
	incf	Rx_Bytes, 0					;increment the offset and store in the WREG
	movf	PLUSW0, 0					;move in the low CRC byte
	movwf	receiveCRCLow				;and store it
	clrf	crcLow						;clear out the CRC low and high bytes to ensure
	clrf	crcHigh						;we start with a clean slate and don't garble shite
CRC_Rx_GEN_LOOP:
	movf	POSTINC0, 0					;move in the next data byte
	movwf	newData						;and store in the location that CRC will operate on
	call	CrcHDLC
	decf	Rx_Bytes, 1				;decrement the counter
	bnz		CRC_Rx_GEN_LOOP				;if we still got bytes to go, do em ay
CRC_COMPARE:
	movf	crcLow, 0					;load up the generated CRC byte
	cpfseq	receiveCRCLow				;compare it to the received CRC byte
	bra		CRC_ERROR_DETECT			;if they don't match - ERROR so mark it
	movf	crcHigh, 0					;else, check the high byte in the same way ay
	cpfseq	receiveCRCHigh
	bra		CRC_ERROR_DETECT
	bsf		flag_buff, Network			;pass control to the Networking layer
	bsf		dsr_flags, PACKET_ARRIVE	;do this when we have a full packet all good
	return
CRC_ERROR_DETECT:
	bsf		error_flags, CRC_ERROR
	return		;Test MAC-buff, BACKUP_FULL. if its set we are waiting for an ACK and can't process	another data packet
CREATE_CRC:
	bcf		flag_buff2, TRANSMIT_CRC
	clrf	crcHigh
	clrf	crcLow
	movlw	high radioout
	movwf	FSR0H
	movlw	low radioout
	movwf	FSR0L
	movf	txbufflen, 0
	movwf	Tx_Bytes
	incf	Tx_Bytes, 1
CRC_Tx_GEN_LOOP:
	movf	POSTINC0, 0					;move in the next data byte
	movwf	newData						;and store in the location that CRC will operate on
	call	CrcHDLC
	decf	Tx_Bytes, 1					;decrement the counter
	bnz		CRC_Tx_GEN_LOOP				;if we still got bytes to go, do em ay
CRC_GEN_DONE:
	movf	crcHigh, 0					;we get here when the CRC has been generated.
	movwf	POSTINC0					;so append it to the Tx byte
	movf	crcLow, 0					;do the same with the low byte of the CRC calculated
	movwf	INDF0
	movf	txbufflen, 0				;add the two bytes of CRC to the overall output buffer length
	addlw	0x02
	movwf	txbufflen
	bsf		flag_buff, MAC				;and pass control down through the layers
	bsf		MAC_buff, PACKET_OUT		;tell MAC that the packet is leaving	
	return
;***********************************************************************
;**********************************************************************
;                                                                     *
;Sub_Routine:     CrcHDLC                                                  *
;                                                                     *
;            Macro for ^16+^12+^5+1, "CITT CRC16" - HDLC, X.25        *
;                                                                     *
; Arguments: newData    - data byte (destroyed)                       *
;            crcLow     - CRC accumulator low byte                    *
;            crcHigh    - CRC accumulator high byte                   *
;                                                                     *
;**********************************************************************
;                                                                     *
; Description: Macros to implement byte at a time ^16+^12+^5+1,       *
;              "CITT CRC16" - HDLC, X.25 16 bit CRC generation        *
;                                                                     *
;**********************************************************************
;    File Version:  1                                                 *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
CrcHDLC:
;       oldCRC ^= newData
;       Modified to  - newData ^= oldCRC
	movf    crcLow,W
	xorwf   newData,F

;       oldCRC  = (oldCRC >> 8) | (oldCRC << 8)
;       Modified to - oldCRC  = (oldCRC >> 8) | newData << 8)
	movf    crcHigh,W
	movwf   crcLow
	movf    newData,W
	movwf   crcHigh

;       oldCRC ^= (oldCRC & 0xFF00) << 4
	swapf   crcHigh,W
	andlw   0xF0
	xorwf   crcHigh,F

;       oldCRC ^= oldCRC >> 12
	swapf   crcHigh,W
	andlw   0x0F
	xorwf   crcLow,F

;       oldCRC ^= (oldCRC & 0xFF00) >> 5
	swapf   crcHigh,F
	bcf     STATUS,C
	btfsc   crcHigh,0
	bsf     STATUS,C
	rrcf     crcHigh,W
	andlw   0xF8
	xorwf   crcLow,F
	rrcf     crcHigh,W
	andlw   0x07
	swapf   crcHigh,F
	xorwf   crcHigh,F
	return
;***********************************************************************
NETWORK_LAYER:
	bcf		flag_buff, Network			;!!!!!!!!!!!Just Added!!!!!!!!!!!!!!!!!!!!!
	btfss	error_flags, CRC_ERROR		;was there a CRC error??
	bra		NO_CRC_ERROR
	bcf		error_flags, CRC_ERROR		;yep, clear the error and
	return								;then just get out cause nothing can be trusted - discard the packet
NO_CRC_ERROR:
	btfsc	dsr_flags, PACKET_ARRIVE	;Did the packet come in?
	bra		INCOMING					;if not zero, we have an input packet - process it
	btfsc	dsr_flags, SEND_PACKET		;Must the packet leave from this layer?
	bra		DSR_SEND_PACKET				;initiate the process to send a packet
	return								;dunno why we would ever get here...get out again
INCOMING:
	bcf		dsr_flags, PACKET_ARRIVE	;clear the flag that caused us to get to here
	movlw 	high radioin 				;Move the high byte of the incomming data buffer address into the FSR1
	movwf 	FSR1H 
	movlw 	low radioin 				;Move the low byte of the RCREG address into the FSR1
	movwf 	FSR1L	
	movff	POSTINC1, dest_addr			;Store the destination address
	movff	POSTINC1, source_addr		;Store the source Address
	movff	POSTINC1, pkt_id			;store the packet id
	movff	POSTINC1, tx_pwr			;store the packet id
	movff	POSTINC1, route_id_d		;store route id - destination portion
	movff	POSTINC1, route_id_s		;and source portion
	movff	POSTINC1, hop_count			;store hop count and point to first DATA BYTE

;***********
	movlw	0x09
	subwf	rxbufflen, 0
	movwf	Tx_data_len
	ADDR_SET radioin
	bsf		flag_buff3, INCOMMING_PKT
	call	DISP_PACKET
;***********
	movf	dest_addr, 0				;load up the final destination address	
	cpfseq	node_id						;compare the packet destination address to the node id
	bra		NOT_FINAL_DEST				;if it's not the final destination address, act accordingly
ADDR_MATCH:					;Yes, the destination address matches the NODE ID
	movf	pkt_id, 0					;load up PACKET ID
	cpfseq	DATA_PKT					;compare the packet ID to that of a data packet
	bra		NOT_DATA					;if its not a data packet, act accordingly
	movf	route_id_d, 0				;final destination
	cpfseq	node_id						;is it meant for us?
	bra		SEND_IT_ON
	bra		FINAL_DATA
SEND_IT_ON:								;if the "remaining hops" count is zero, we have reaced the destination
	call	PACKET_FORWARD				;otherwise, forward the data packet
	return								;and go back to the start loop point
FINAL_DATA:					;the remaining hop count is zero and we are dealing with data
	call	DATA_CACHE					;if we are at the final destination, call a SR to stash the info
	bsf		flag_buff, ACK_SEND			;go and build and send and ACK packet
	bsf		flag_buff, Network			;retain program control in this layer
	bsf		dsr_flags, SEND_PACKET		;but move on to send the packet
	return								;and go back to the start loop point
NOT_DATA:					;address matches but packet is not data
	movf	pkt_id, 0					;re-load the packet ID
	cpfseq	RREQ						;is it a RREQ?
	bra		RREP_TEST					;No, is it a RREP?

RREQ_REP:
	movlw 	high addr_buff 				;If we get here, packet is a RREQ to this node - reverse route										
	movwf 	FSR0H 						;Move the high byte of the temporary holding buffer into FSR0
	movlw 	low addr_buff 				;Move the low byte temp address buffer into FSR0
	movwf 	FSR0L
	movf	node_id, 0
	movwf	POSTINC0
	decf	hop_count, 1				;load the hop count into WREG and modify in prep for PLUSW1 instruction
	movf	hop_count, 0
	movf	PLUSW1, 0					;move from the offset buffer location to WREG
	movwf	dest_addr					;next hop on the way home
;	decf	hop_count, 1				;remove one to allow for the reverse of the route INCLUDING this node
ROUTE_REVERSE:				;If RREQ Packet and address matches, reverse it
	movf	hop_count, 0				;load the hop count into WREG in prep for PLUSW1 instruction
	movf	PLUSW1, 0					;move from the offset buffer location to WREG
	movwf	POSTINC0					;move from WREG to load the last address added as the first return address
	decf	hop_count, 1					;decrement the hop counter
	bnz		ROUTE_REVERSE				;if we still have destinations left, suffle em.
	movf	hop_count, 0				;load the hop count into WREG in prep for PLUSW1 instruction
	movf	PLUSW1, 0					;move from the offset buffer location to WREG
	movwf	POSTINC0					;move from WREG to load the last address added as the first return address
	bsf		dsr_flags, USE_ADDR_BUFF	;signal that we must use the reversed addresses
	movlw	RREP_V						;When it goes back it will be a RREP
	movwf	pkt_id
	movff	node_id, source_addr		;and we are the source
	movff	tx_pwr, Tx_power_level		;send back at the same power
	call	BUILD_PACKET				;build the packet
	return								;go back to the start loop
RREP_TEST:					;packet isn't data or RREQ - RREP?
	movf	pkt_id, 0					;load up the packet id
	cpfseq	RREP						;is the packet a Route REPly Packet?
	bra		ACK_TEST					;No, is it an ACK packet
	movf	route_id_s, 0				;load the source of the route
	cpfseq	node_id						;did the RREQ come from us?
	bra		FWD_RREP					;if not, go to here
	movf	RREQ_cause_addr, 0			;load the target address that caused us to put out a RREQ
	cpfseq	route_id_d					;should be the source of this RREP
	return								;if not, get out!!!!!!!!!!!!!!!!!!
	btfss	dsr_flags, RREQ_SENT		;Did we actually send one out?
	return								;No, dunno why we got here but please show yourself out
	call	ON_BOARD_ROUTE_CACHE		;Yes, it came for us to start with and matched the stashed one
	bcf		dsr_flags, RREQ_SENT		;clear the flags that signals a RREQ is out there cause we got it
	bcf		flag_buff, STOPPEDNWAITING	;release the Stop and Wait flag as well
	bcf		tmr_on_flags, RREQ_TIMEOUT_ON	;turn off the "timer"
;Check the buffered data that invoked the RREQ in the first place
	movlw 	high No_route_buff			;Move the high byte of the holding buffer that we stuck stuff
	movwf 	FSR0H 						;in before we went out and looked for the route
	movlw 	low No_route_buff				
	movwf 	FSR0L
	movlw	0x02
	movf	PLUSW0, 0					;load up the packet ID that we were originally dealing with
	cpfseq	DATA_PKT
	bra		ACK_STACKED?				;if it wasn't a data packet, was it an ACK
	bsf		flag_buff, DATA_RESEND		;and mark that its goin out
	bcf		flag_buff, ACK_SEND			;make sure we don't send an ACK
	bsf		flag_buff, Network			;retain networking layer control
	bsf		dsr_flags, SEND_PACKET		;and mark that we need to send a packet
	return
ACK_STACKED?:
	cpfseq	ACK
	return								;should never get here - if we do something went wrong (only data or ACK can initiate RREQ)
	movff	INDF0, route_id_s			;when building ACK packet, this will get reversed, but is stacked the right way
	movff	PREINC0, route_id_d			;likewise with the last part
	bsf		flag_buff, ACK_SEND			;make sure we send an ACK
	bsf		flag_buff, Network			;retain networking layer control
	bsf		dsr_flags, SEND_PACKET		;and mark that we need to send a packet
	bra		EXEC_LOOP
FWD_RREP:
	call	ROUTE_CACHE					;Store the route for further use
	call	BUILD_PACKET				;build the packet and send it out
	call	ROUTE_CACHE_TEST	;TEST ROUTINE - Test the route Cache to ensure that we cached properly
	return
ACK_TEST:
	movf	pkt_id, 0					;if not what we expect
	cpfseq	ACK
	bra		RMOD_TEST					;Test for the route modify packet type
	movlw 	high backup_buff			;Move the high byte of the holding buffer to FSR
	movwf 	FSR0H 
	movlw 	low backup_buff				;Move the low byte of the holding buffer
	addlw	0x03
	movwf	FSR0L
	btfsc	STATUS, C					;Did it overflow?
	incf	FSR0H, 1					;yep, best increment the high byte as well ay
	movf	POSTINC0, 0					;load up route_id_destination from stashed DATA packet	
	cpfseq	route_id_s					;this sould be the source of the reply
	return								;It's not - get out. CAN ONLY HANDLE 1 DATA PACKET AT A TIME
	movf	POSTINC0, 0					;load up stashed route_id_s
	cpfseq	route_id_d					;should be the final destination 
	return
	bcf		flag_buff, STOPPEDNWAITING	;we got the ACK, so clear the flag holding us out
	return								;and get out
RMOD_TEST:
	movf	pkt_id, 0					;double check that we have the route modify packet
	cpfseq	RMOD
	bra		RMOD_ACK_TEST
	call	MEM_SEARCH					;go and find the route that we are looking to modify
	movf	current_mem_h, 0			;move in the high byte of the location that the search
	movwf	PARO_mod_loc_h				;landed us at and save it
	movlw	0x02						;add two to the wreg
	subwf	current_mem_l, 0			;and subtract from the memory tracker
	movwf	PARO_mod_loc_l				;save this as the location that we'll start modifying from
	btfsc	STATUS, N					;did the subtraction cause a borrow op?
	decf	PARO_mod_loc_h, 0			;if so, decrement the high byte of the location 
	;Now write to the FRAM to modify the route	
	bsf		PORTC, NWP					;disable write protect
	bsf		PORTC, HOLD					;and don't halt the operation
	CS_SETUP							;toggle the !CS to enable chip
	movlw	WREN						;Enable writes
	call	SPI_ROUTINE
	CS_SETUP
	movlw	WRITE						;Say were gonna write
	call	SPI_ROUTINE	
	movf	PARO_mod_loc_h, 0
	call	SPI_ROUTINE
	movf	PARO_mod_loc_l, 0
	call	SPI_ROUTINE
	movf	source_addr, 0				;load up the source of the RMOD packet - this will now be the next hop
	call	SPI_ROUTINE
	movf	tx_pwr, 0					;load up the first byte of the data packet - Tx_power info
	call	SPI_ROUTINE					;and stash this too
	bsf		flag_buff3, RMOD_ACK_SEND
	bsf		flag_buff, Network
	bsf		dsr_flags, SEND_PACKET
	call	ROUTE_CACHE_TEST	;TEST ROUTINE - Test the route Cache to ensure that we cached properly
	return
RMOD_ACK_TEST:
	btfsc	flag_buff2, NOTIFY_STG1		;are we in the second stage of the modification process?
	bra		SECOND_STG_RMOD				;yep. get there
	movf	source_addr, 0				;No, in first stage. Sooo, RMOD_ACK should be from the PARO source node
	cpfseq	PARO_source					;check if it was
	return								;nup, wrong node so get out.
	bcf		flag_buff, STOPPEDNWAITING	;clear the stop and wait flag cause we got our reply
	bsf		flag_buff2, NOTIFY_STG1		;but set the stage two initiating flag
	call	NOTIFY						;and send out the other notification
	return									
SECOND_STG_RMOD:
	movf	source_addr, 0				;since we are now in the second stage of the route mod. process
	cpfseq	PARO_dest					;the ACK should be coming from the PARO destination
	return								;nope, got garbage so get out
	bcf		flag_buff, STOPPEDNWAITING	;clear the stop and wait flag cause we got our reply
;cache the route in the forward direction
	movff	PARO_rt_id_d, RMOD_id_d		;in the forward direction 
	movff	PARO_rt_id_s, RMOD_id_s		;everything is as it seems
	movff	PARO_dest, MOD_next_hop		;the destination address originally examined becomes the next hop in the route
	movff	mod_power_B, MOD_pwr		;and the power is that determined from the reply packet
	call	RMOD_ROUTE_CACHE
;and then in the reverse direction
	movff	PARO_rt_id_s, RMOD_id_d		;in the reverse direction 
	movff	PARO_rt_id_d, RMOD_id_s		;its all ass about
	movff	PARO_source, MOD_next_hop	;the orign address originally examined becomes the next hop in the reversed route
	movff	mod_power_A, MOD_pwr		;and the power is that determined from the original packet
	call	RMOD_ROUTE_CACHE
	return
;If this node is not the final target destination in the flood
NOT_FINAL_DEST:				;address does not match NODE ID
	movf	pkt_id, 0					;re-load the packet ID
	cpfseq	RREQ						;is it a RREQ??
	bra		PARO						;If final address doesn't match node ID and its not a RREQ to be flooded - PARO
RREQ_FWD:					;forwarding the RREQ packet if needed
	btfsc	error_flags, NO_MEM_ERROR	;have we still got memory space?
	return								;no, don't participate
	bsf		MAC_buff, RREQ_SEARCH		;mark flag so we don't alter destination address
	call	MEM_SEARCH					;check on the route space
	bcf		MAC_buff, RREQ_SEARCH		;and clear the flag
	btfsc	dsr_flags, FOUND_ROUTE		;do we already have this route cached? - if so source of route sent RREQ cause its expired
	call	ROUTE_ERASE					;get rid of the route cause its now extinct. then continue normally
	movf	route_id_d, 0				;yes, skip forward to the route ID
	cpfseq	latest_route_d				;have we recieved this packet earlier?
	bra		FIRST_RREQ					;no, process it
	movf	route_id_s, 0
	cpfseq	latest_route_s
	bra		FIRST_RREQ
	return								;yes, we need to nothing else because it has already been here
FIRST_RREQ:					;Yep, first time we got it so append our address and move on
	movff	route_id_d, latest_route_d	;store in the latest route register
	movff	route_id_s, latest_route_s	;store the last bit of the route ID
	movlw	0x02
	movwf	current_rreq_timer			;set up the timer for 1 minute
	incf	rxbufflen, 1				;add another byte to the length of the received data stream
	movff	node_id, PLUSW1				;append this nodes id to the packet in radioin
	incf	hop_count, 1				;add another hop to the sequence
	movlw	D'14'						;load up the max power level (20dB) and belt out that packet
	movwf	RREQ_Tx_pwr					;and store it
	call	BUILD_PACKET				;call a SR to build the packet and continue the flooding
	return
;***********************************************************************
PARO:
	btfsc	flag_buff2, PARO_ON			;are we in the middle of a routine already? - TIME OUT TO CLEAR THIS!!!!!!!!!!!!!!!!!!!!
	bra		FINISH_PARO					;yep, then check for return address I guess
	movf	tx_pwr, 0					;load in the power the packet was sent out with
	movwf	sent_power					;and store it as the original packet power
	movff	source_addr, PARO_source	;store the source of this PARO analysed packet
	movff	dest_addr, PARO_dest		;store the intended destination of the packet
	movff	route_id_d, PARO_rt_id_d	;store the route id in memory so that we can send a RMOD back with
	movff	route_id_s, PARO_rt_id_s	;the same key
	call	ADREAD
	movf	Rx_power, 0					;load up the minimum received power level (Do in revers because of dB scale and larger = small)
	subwf	min_rx_pwr, 0				;and subtract it from the actual to give us the overhead (in dB)
	subwf	sent_power, 0				;then subtract the overhead from the Tx power to get the modified power that would be needed to make the distance (dB)
	btfsc	STATUS, N					;was the result negative?
	movlw	0x00						;yep, then reduce to the lowest setting possible - 0dB
	movwf	mod_power_A					;store this as the proposed new power consumption for leg A
	call	DB_TO_MW_CONV				;convert the dB reading to a mW level to make calculations valid
	movwf	mod_A_mW					;save the returned value as the mW value of the mod Power
	bsf		flag_buff2, PARO_ON
	bra		PARO_END
FINISH_PARO:
	movf	tx_pwr, 0					;load up the power that the packet was sent out with
	movwf	reply_power,1				;and store it as the power level coming in
	movf	source_addr, 0				;move the source of this packet into WREG
	cpfseq	PARO_dest					;compare to the destination of the previous packet
	bra		EXEC_LOOP					;if this isn't the right packet, get out and wait for right one - we should time out of this
	bcf		flag_buff2, PARO_ON			;if its the right fella talkin, clear the flag
	call	ADREAD						;read the A/D value
	movf	Rx_power, 0					;load up the minimum received power level (Do in revers because of dB scale and larger = small)
	subwf	min_rx_pwr, 0				;and subtract it from the actual to give us the overhead
	subwf	reply_power, 0				;then subtract the overhead from the Tx power to get the modified power
	btfsc	STATUS, N					;was the result negative?
	movlw	0x00						;yep, then reduce to the lowest setting possible - 0dB
	movwf	mod_power_B					;store this as the proposed new power consumption for leg A
	call	DB_TO_MW_CONV				;convert the dB reading to a mW level to make calculations valid
	movwf	mod_B_mW					;save the returned value as the mW value of the mod Power
;*********PARO CALCULATIONS**************
;Now check the route powers to see if modifiaction is reccomended
	movf	sent_power, 0				;load up the power used on sending
 	call	DB_TO_MW_CONV				;convert the dB reading to a mW level to make calculations valid
	movwf	sent_power
	movf	reply_power, 0
	call	DB_TO_MW_CONV				;convert the dB reading to a mW level to make calculations valid
	addwf	sent_power, 0				;add the power used in the reply  (returned in wreg) and sent power
	movwf	total_init_pwr				;and save this as the total power used in the current route
	movf	mod_A_mW, 0					;load up the power used alond the proposed route leg A
	addwf	mod_B_mW, 0					;and add the leg B power
	mullw	0x02						;multiply this by two
	movf	PRODL, 0					;load up the result (will never be bigger then 256 so PRODH is useless)
	cpfsgt	total_init_pwr				;compare to the original sending power
	bra		PARO_END					;if initial pwr is not greater than the potential modified power, then no need - get out
	call	NOTIFY						;if initial power is greater, then we can save power by participating. Notify nodes
PARO_END:
	bcf		ADCON0, ADON				;turn off the A/D converter
	bcf		flag_buff, Network
	return
CHECK_PARO_RETURN:
	return								;otherwise get out
;***********************************************************************
NOTIFY:
	btfsc	flag_buff2, NOTIFY_STG1
	bra		NOTIFY_STG2
	bsf		flag_buff2, NOTIFY_STG1
	movf	mod_A_mW, 0
	call	mW_TO_dB
	movwf	mod_power_A
	movf	mod_B_mW, 0
	call	mW_TO_dB
	movwf	mod_power_B
	movf	PARO_source, 0
	movwf	dest_addr					;set the route id - final destination
	movf	node_id, 0
	movwf	source_addr					;and originating source
	movf	PARO_rt_id_d, 0
	movwf	route_id_d					;load up the same route ID that caused the mod.
	movf	PARO_rt_id_s, 0
	movwf	route_id_s
	movf	RMOD, 0
	movwf	pkt_id;						;and load up the modify packet ID
	bsf		flag_buff2, PARO_BUILD		;set the bit to ensure we load from the data bank
	movf	mod_power_A, 0
	movwf	Tx_power_level
	bra		END_NOTIFY	
NOTIFY_STG2:							;MOVE IN ROUTE MODIFICATION DATA AND CALL BUILD PACKET
	bcf		flag_buff2, NOTIFY_STG1
	movf	PARO_dest, 0
	movwf	route_id_d					;set the route id - final destination
	movf	node_id, 0
	movwf	route_id_s					;and originating source
	movf	PARO_rt_id_d, 0
	movwf	route_id_d					;load up the same route id that cause the mod.
	movf	PARO_rt_id_s, 0
	movwf	route_id_s
	movf	RMOD, 0
	movwf	pkt_id						;and load up the modify packet ID
	bsf		flag_buff2, PARO_BUILD		;set the bit to ensure we load from the data bank
	movf	mod_power_B, 0
	movwf	Tx_power_level	
	clrf	PARO_source
	clrf	PARO_dest				
END_NOTIFY:
	call	BUILD_PACKET
	return
;***********************************************************************
DSR_SEND_PACKET:		;will be sending either DATA or ACK's
	bcf		dsr_flags, SEND_PACKET		;clear the flag
	btfsc	flag_buff, ACK_SEND			;do we need to send an acknowledge?
	bra		ACK_REPLY
	btfsc	flag_buff3, RMOD_ACK_SEND
	bra		RMOD_ACK_REPLY
	btfsc	flag_buff, DATA_RESEND		;do we need to send an acknowledge?
	bra		DATA_RETRY
	btfss	timer_flags, PACKET_RETRY
	bra		DATA_PACKET_SEND
	call	SEND_PACKET_RETRY	
	return
DATA_RETRY:
	movlw 	high No_route_buff			;Move the high byte of the holding buffer
	movwf 	FSR2H 
	movlw 	low No_route_buff			;Move the low byte of the holding buffer
	movwf 	FSR2L
	movff	POSTINC2, route_id_d
	movff	POSTINC2, route_id_s
	movff	POSTINC2, pkt_id
	call	LOCAL_MEM_SEARCH			;then look for this route in memory - will get it cause when get here if RREP comes back
	movlw 	high data_bank				;Move the high byte of the data buffer
	movwf 	FSR1H 
	movlw 	low data_bank				;Move the low byte of the data buffer
	movwf 	FSR1L
DATA_RELOAD:
	movff	POSTINC2, POSTINC1			;load up the data again - NOTE - ANYTHING IN DATA BUFFER WILL BE LOST - SESSION LAYER PROBLEM
	decfsz	data_bank_ldr, 1			;decrement the counter, is it zero?
	bra		DATA_RELOAD					;nope, comtinue loading
	bra		DATA_COMMON_PT				;yes, load up finished - continue as if it were a normal data packet
DATA_PACKET_SEND:
	movff	target_addr, route_id_d		;set the route id - final destination
	movff	node_id, route_id_s			;and originating source
	call	LOCAL_MEM_SEARCH			;then look for this route in memory
	btfss	flag_buff2, FOUND_ONBOARD	;did we find a route in the onboard cache?
	bra		INVOKE_RREQ					;no, go and find one
	movf	validity_count, 0			;load up the value of the validity counter when the route was discovered
	subwf	v_count_val, 0			;subtract this from the current value of the counter
	movwf	v_count_diff				;and store it
	movlw	D'120'		;1 hour route validity
	cpfsgt	v_count_diff
	bra		STILL_VALID
	call	ERASE_ONBOARD_ROUTE
	bra		INVOKE_RREQ
STILL_VALID:	
	bcf		flag_buff2, FOUND_ONBOARD	;clear the flag to stop another erraneous decision
	movlw	DATA_PKT_V					;Yes we found one	
	movwf	pkt_id						;set the packet ID
DATA_COMMON_PT:
	bcf		flag_buff2, FIRST_NON_DATA	;make sure we dont get confused here
	bsf		dsr_flags, NEW_PACKET		;mark this as a new packet so we transfer from data_bank
	bsf		tmr_on_flags, ACK_TIMEOUT_ON;and "turn on" the ACK timer
	movlw	0x04
	movwf	ACK_timer					;allow 2 minutes for the ACK to come back to us
	bra		FINISH_SEND					;and now do the common stuff
ACK_REPLY:				;get here cause last packet Rx was data - need to reply
	movf	route_id_s, 0				;and reverse the route ID
	movff	route_id_d, route_id_s
	movwf	route_id_d
	call	LOCAL_MEM_SEARCH
	btfss	flag_buff2, FOUND_ONBOARD	;did we find one?
	bra		INVOKE_RREQ					;no - then do a RREQ
	movf	validity_count, 0			;load up the value of the validity counter when the route was discovered
	subwf	v_count_val, 0			;subtract this from the current value of the counter
	movwf	v_count_diff				;and store it
	movlw	D'120'		;1 hour route validity
	cpfsgt	v_count_diff
	bra		STILL_VALID1
	call	ERASE_ONBOARD_ROUTE
	bra		INVOKE_RREQ
STILL_VALID1:
	bcf		flag_buff, ACK_SEND			;yep, found a route, so we don't need to come back here
	bcf		dsr_flags, FOUND_ROUTE		
	movlw	ACK_V						;load up acknowledge id
	movwf	pkt_id						;and mark a packet with it
	bra		FINISH_SEND
RMOD_ACK_REPLY:
	bcf		flag_buff3, RMOD_ACK_SEND
	movff	source_addr, dest_addr		;flip it so we reply directly
	movlw	RMOD_ACK_V					;mark it as an ACK to the RMOD request
	movwf	pkt_id
	bra		FINISH_SEND
INVOKE_RREQ:			;don't know a valid route, find one
	call	LOAD_No_route_buff			;load up the buffer to store data whilst RREQ is out there
	movff	route_id_d, RREQ_cause_addr;and mark the offending address
	movlw	RREQ_V
	movwf	pkt_id						;mark it as a RREQ packet
	movff	route_id_d, dest_addr		;mark the destination
	movlw	0x07						
	movwf	rxbufflen					;set the rxbufflen counter so we don't try and load data in BUILD_PACKET
	movlw	high addr_buff				;set pointers to start of address buffer
	movwf	FSR1H
	movlw	low addr_buff 
	movwf	FSR1L
	movf	node_id, 0
	movwf	INDF1
	movlw	GAIN_14						;move in the value corresponding to max power level
	movwf	VGA_gain_set				;and set it so we Tx at this on the first attempt
	movlw	D'14'						;load up the corresponding power level (20dB)
	movwf	RREQ_Tx_pwr					;and store it
	movwf	Tx_power_level
	bsf		dsr_flags, USE_ADDR_BUFF
	bsf		dsr_flags, RREQ_SENT		;and mark that its goin out
	bsf		flag_buff2, FIRST_NON_DATA	;set the flag so that we don't get overhead
FINISH_SEND:
	movff	node_id, source_addr		;set this nodes id as the source address for the next hop
	clrf	hop_count					;clear the hop count cause we are starting it
	call	BUILD_PACKET				;and build the damn thing
	return								;and get out
;***********************************************************************
SEND_PACKET_RETRY:
	call	FSR0_STACK
	movlw 	high backup_buff			;Move the high byte of the holding buffer to FSR
	movwf 	FSR0H 
	movlw 	low backup_buff				;Move the low byte of the holding buffer
	movwf 	FSR0L
	movlw 	high radioout 				;Move the high byte of the output data buffer address into the FSR1
	movwf 	FSR1H 
	movlw 	low radioout
	movwf 	FSR1L
RELOAD_LOOP:
	movff	POSTINC1, POSTINC0			;transfer 1 byte
	decf	backup_len, 1
	bnz		RELOAD_LOOP					;if transfer isn't complete - keep going
	call	FSR0_UNSTACK				;unstack the previous contents of the FSR
	bsf		dsr_flags, INITIATE			;Indicate we need to initiate a Tx
	bsf		flag_buff, MAC				;and pass control down through the layers
	bsf		MAC_buff, PACKET_OUT		;tell MAC that the packet is leaving
	movlw	0x05						;load up 5
	movwf	preamble_count				;store it to mark sending out 5 preamble bytes
	return
;***********************************************************************
ADREAD:
	call 	FSR0_STACK
ADREAD_WAIT:
	btfss	ADCON0, 2					;Is the conversion done?
	bra		CONV_DONE
	bra		ADREAD_WAIT					;nope, loop till it is	
CONV_DONE:				
	movf	ADRESL, 0
	movwf	RSSI_value					;store the values
	rrncf	RSSI_value, 1				;rotate right to roll of the two LSB's, once
	rrncf	RSSI_value, 1				;and again
	bcf		RSSI_value, 7				;clear the 2 MSB's
	bcf		RSSI_value, 6
	btfsc	ADRESH, 0					;now translate the high byte of the AD result
	bsf		RSSI_value, 6
	btfsc	ADRESH, 1
	bsf		RSSI_value, 7	
	clrf	tbl_offset
	movlw	upper RSSI_mins				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;minimum RSSI boundaries
	movlw	high RSSI_mins
	movwf	TBLPTRH
	movlw	low RSSI_mins
	movwf	TBLPTRL
AD_LOOPa:
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	movwf	tbl_RSSI_val				;and move it to a more solid location
	tblrd*+								;increment the table value the second time and...
	incf	tbl_offset, 1				;increment our offset counter
	incf	tbl_offset, 1				;increment our offset counter
	movf	RSSI_value, 0				;load up our measured value
	cpfsgt	tbl_RSSI_val				;loop untill the value exceeds a table value
	bra		AD_LOOPa					;as soon as the reading exceeds a table value we break out
	decf	tbl_offset, 1				;decrement to account for placement of increment
	decf	tbl_offset, 1				;decrement to account for placement of increment
	movlw	upper RSSI_maxs				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;maximum RSSI boundaries
	movlw	high RSSI_maxs
	movwf	TBLPTRH
	movlw	low RSSI_maxs
	addwf	tbl_offset, 0					;add the offset we went to previously
	movwf	TBLPTRL
	btfsc	STATUS, C					;if we carried
	incf	TBLPTRH, 1					;increment
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	cpfslt	RSSI_value					;skip if RSSI value is less than the max value
	bra		AD_LOOPa	
	;By this stage we have check that the RSSI reading is in the tolerance we want and know the offset required
	movlw	upper power_values			;Load up the table pointer so it points at the
	movwf	TBLPTRU						;maximum RSSI boundaries
	movlw	high power_values
	movwf	TBLPTRH
	movlw	low power_values
	addwf	tbl_offset, 0				;add the offset we went to previously
	movwf	TBLPTRL
	btfsc	STATUS, C					;if we carried
	incf	TBLPTRH, 1					;increment
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	movwf	Rx_power
	call 	FSR0_UNSTACK
	return
;***********************************************************************
PACKET_FORWARD:
	call	MEM_SEARCH
	call	BUILD_PACKET				;build the packet using the SR
	return
;***********************************************************************
BUILD_PACKET:
	movlw	high radioout
	movwf	FSR0H						;load up the location of the next available route cache location
	movlw	low radioout
	movwf	FSR0L	
	incf	hop_count, 1
	movff	dest_addr, POSTINC0			;load the destination address
	movff	node_id, POSTINC0			;load the source address - our ID
	movff	pkt_id, POSTINC0			;load the packet id
	movff	Tx_power_level, POSTINC0	;load the transmit power level
	movff	route_id_d, POSTINC0		;load the route id part 1
	movff	route_id_s, POSTINC0		;load the route id part 2
	movff	hop_count, POSTINC0			;load the hop count
	movlw	0x06						;decrement the rxbufflen counter by the same number of bytes we've done
	subwf	rxbufflen
	movwf	rxbufflen
	movlw	0x06
	movwf	txbufflen					;mark that so far there are 6 bytes to Tx
	btfsc	flag_buff2, FIRST_NON_DATA	;is this the end of the data we need to Tx?
	bra		END_PKT_BUILD				;yep
	btfsc	dsr_flags, USE_ADDR_BUFF	;no, continue...do we need to load a reversed address sequence for a RREP?
	bra		LOAD_ADDR_BUFF				;Yes, do it
	btfsc	flag_buff2, PARO_BUILD		;Are we building a PARO packet??
	bra		PARO_ASSEMBLE
	btfsc	dsr_flags, NEW_PACKET		;do we need to transfer new data from the data bank?
	bra		FROM_BANK
	bra		END_PKT_BUILD				;We are sending an ACK or RMOD_ACK so need nothing in the tail of the packet
DATA_LOAD_LOOP:				;load data straight from in to out
	bcf		dsr_flags, NEW_PACKET		;clear the reason we got here
	movff	POSTINC1, POSTINC0			;move from radioin to radioout
	incf	txbufflen, 1				;increment each time a byte is sent
	decf	rxbufflen, 1				;decrement the counter
	bnz		DATA_LOAD_LOOP				;if we still have bytes, transfer them
	bra		END_PKT_BUILD				;otherwise we are done
FROM_BANK:					;load data from a bank
	movlw	high data_bank				;point to the data bank
	movwf	FSR1H
	movlw	low data_bank
	movwf	FSR1L
BANK_LOOP:
	movff	POSTINC1, POSTINC0			;move from data_bank to radioout
	incf	txbufflen, 1	
	decf	data_len, 1					;decrement the gas gauge
	bnz		BANK_LOOP					;if bank not empty, empty it
	bra		END_PKT_BUILD
PARO_ASSEMBLE:
	bcf		flag_buff2, PARO_BUILD
	movf	Tx_power_level, 0
	movwf	POSTINC0
	incf	txbufflen, 1
	bra		END_PKT_BUILD
LOAD_ADDR_BUFF:				;load the reversed address list
	movlw	high addr_buff
	movwf	FSR1H
	movlw	low addr_buff 
	movwf	FSR1L
	incf	rxbufflen, 1
ADDR_LOOP:
	movff	POSTINC1, POSTINC0			;move from address buffer to radioout buffer
	movf	INDF1, 0					;load the current addr_buff value
	decf	rxbufflen, 1				;decrement the received buffer length counter
	incf	txbufflen, 1
	bnz		ADDR_LOOP
	bcf		dsr_flags, USE_ADDR_BUFF
END_PKT_BUILD:				;....and....we're done!
	call	LOAD_BACKUP					;load up the backup buffer
	bcf		flag_buff, Network			;release layer control
	bsf		dsr_flags, INITIATE			;Indicate we need to initiate a Tx
	bsf		flag_buff, Data_Link
	bsf		flag_buff2, TRANSMIT_CRC
	movlw	0x05						;load up 5
	movwf	preamble_count				;store it to mark sending out 5 preamble bytes
	call	dB_TO_GAIN					;translate the power level into a gain for the VGA
;***********
;	movlw	0x0A
	movlw	0x08
	subwf	rxbufflen, 0
	movwf	Tx_data_len
	ADDR_SET radioout
	call	DISP_PACKET
;***********
	return
;***********************************************************************
ON_BOARD_ROUTE_CACHE:
	call	FSR0_STACK
	movf	current_on_board_l, 0		;move in the low space address byte	
	movwf	FSR0L
	movf	current_on_board_h, 0		;move in the low space address byte	
	movwf	FSR0H
	movf	current_on_board_h, 0
	movwf	local_search_pt_h
	movf	current_on_board_l, 0		;check if we have room to store another 5 bytes of info
	addlw	0x05
	movwf	local_search_pt_l			;use this in this temporary case
	btfss	STATUS, C
	bra		TEST_FOR_SPACE
	incf	local_search_pt_h, 1
TEST_FOR_SPACE:
	movf	max_location_h, 0
	cpfsgt	local_search_pt_h
	bra		TEST_ON_PT
	bra		MEM_FRESH_OUT
TEST_ON_PT:
	movf	local_search_pt_l, 0
	cpfslt	max_location_l
	bra		MEM_FRESH_OUT
	movlw	0x00
	movwf	POSTINC0
	movf	route_id_d, 0
	movwf	POSTINC0
	movf	route_id_s, 0
	movwf	POSTINC0
	movlw	0x01						;load up 1 cause we need to step back one to the previous hop
	subwf	hop_count, 0				;subtract from the WREG and stick it in the WREG
	movf	PLUSW1, 0					;use it as an offset
	movwf	POSTINC0					;and stash the pointed to value
	movf	RREQ_Tx_pwr, 0				;load up the level we transmitted at						
	movwf	POSTINC0
	movf	validity_count, 0			;load up the current value of the validity counter
	movwf	POSTINC0
	movf	current_on_board_l, 0
	addlw	0x05
	movwf	current_on_board_l
	btfsc	STATUS, C
	incf	current_on_board_h, 1
	bra		END_ON_BOARD_STASH
MEM_FRESH_OUT
	bsf		error_flags, ON_BOARD_FULL
END_ON_BOARD_STASH:
	call 	FSR0_UNSTACK
	return
;***********************************************************************
ROUTE_CACHE:	
	bsf		PORTC, NWP				;disable write protect
	bsf		PORTC, HOLD				;and don't halt the operation
	CS_SETUP						;toggle the !CS to enable chip
	movlw	WREN					;Enable writes
	call	SPI_ROUTINE
	CS_SETUP
	movlw	WRITE					;Say were gonna write
	call	SPI_ROUTINE	
	movf	highest_mem_h, 0
	call	SPI_ROUTINE
	movf	highest_mem_l, 0
	call	SPI_ROUTINE
	movlw	0x00
	call	SPI_ROUTINE
	movf	route_id_d, 0
	call	SPI_ROUTINE
	movf	route_id_s, 0
	call	SPI_ROUTINE
	movlw	0x01						;load up 1 cause we need to step back one to the previous hop
	subwf	hop_count, 0				;subtract from the WREG and stick it in the WREG
	movf	PLUSW1, 0					;use it as an offset
	call	SPI_ROUTINE
	movf	RREQ_Tx_pwr, 0				;load up the power level that we transmitted at
	call	SPI_ROUTINE					;and stash it for further use
	bsf		PORTC, NCS					;turn chip off again to terminate write
	movf	highest_mem_l, 0			;set the memory markers again
	addlw	0x04						;by adding the four places we've gone
	movwf	highest_mem_l				;stashing it
	btfsc	STATUS, C					;did we get a carry
	incf	highest_mem_h, 1			;yep, increment high byte and store it
	movlw	0x08
	cpfslt	highest_mem_h				;is last_route_h less than 0x08?
	bra		OUT_OF_SPACE				;no, he have run out of memory!!!!
	movf	route_id_s, 0				;and store it
	cpfseq	node_id						;if this is the final node in the chain then return
	bra		NO_ROUTES_LEFT	
	nop				;Indicate that we have a route and send the data packet here					
	return
NO_ROUTES_LEFT:
	movlw	0x04						;load an offset value to take us back to the next hop on the RREP line
	movf	PLUSW1, 0					;load up the next hop
	movwf	dest_addr					;otherwise, load up the next hop in the RREP line
	return
OUT_OF_SPACE:
	bsf		error_flags, NO_MEM_ERROR	;set an error flag of some sort
	return
;***********************************************************************
;Used to cache the route information when we participate in modifiying the route
;the information for the route - i.e. the next hop when we become a detour, is
;cached using this routine
RMOD_ROUTE_CACHE:	
	bsf		PORTC, NWP				;disable write protect
	bsf		PORTC, HOLD				;and don't halt the operation
	CS_SETUP						;toggle the !CS to enable chip
	movlw	WREN					;Enable writes
	call	SPI_ROUTINE
	CS_SETUP
	movlw	WRITE					;Say were gonna write
	call	SPI_ROUTINE	
	movf	highest_mem_h, 0
	call	SPI_ROUTINE
	movf	highest_mem_l, 0
	call	SPI_ROUTINE
	movlw	0x00
	call	SPI_ROUTINE
	movf	RMOD_id_d, 0
	call	SPI_ROUTINE
	movf	RMOD_id_s, 0
	call	SPI_ROUTINE
	movf	MOD_next_hop
	call	SPI_ROUTINE
	movf	MOD_pwr, 0				;load up the power level that we transmitted at
	call	SPI_ROUTINE					;and stash it for further use
	bsf		PORTC, NCS					;turn chip off again to terminate write
	movf	highest_mem_l, 0			;set the memory markers again
	addlw	0x04						;by adding the four places we've gone
	movwf	highest_mem_l				;stashing it
	btfsc	STATUS, C					;did we get a carry
	incf	highest_mem_h, 1			;yep, increment high byte and store it
	movlw	0x08
	cpfslt	highest_mem_h				;is last_route_h less than 0x08?
	bra		OUT_OF_SPACE_RMOD			;no, he have run out of memory!!!!
	movf	route_id_s, 0				;and store it
	cpfseq	node_id						;if this is the final node in the chain then return
	bra		NO_ROUTES_LEFT_RMOD	
	nop				;Indicate that we have a route and send the data packet here					
	return
NO_ROUTES_LEFT_RMOD:
	movlw	0x04						;load an offset value to take us back to the next hop on the RREP line
	movf	PLUSW1, 0					;load up the next hop
	movwf	dest_addr					;otherwise, load up the next hop in the RREP line
	return
OUT_OF_SPACE_RMOD:
	bsf		error_flags, NO_MEM_ERROR	;set an error flag of some sort
	return
;***********************************************************************
ERASE_ONBOARD_ROUTE:
	movf	local_search_pt_h, 0
	movwf	FSR0H
	movlw	0x05
	subwf	local_search_pt_l, 0
	movwf	FSR0L
	btfsc	STATUS, N
	decf	FSR0H, 1
	movf	local_search_pt_l, 0
	movwf	FSR1L
	movf	local_search_pt_h, 0
	movwf	FSR1H
ERASE_CHECK_PT:
	movf	POSTINC1, 0
	movwf	POSTINC0
	movf	current_on_board_h, 0
	cpfseq	FSR1H
	bra		ERASE_CHECK_PT
	bra		CHECK_LOW_BIT
CHECK_LOW_BIT:
	movf	current_on_board_l, 0
	cpfsgt	FSR1L
	bra		ERASE_CHECK_PT
	movf	FSR0H, 0
	movwf	current_on_board_h
	movf	FSR0L, 0
	movwf	current_on_board_l
	decf	current_on_board_l, 1			;decrement by one to allow for the POSTINC'ing
	btfsc	STATUS, N						;was there a negative underflow?
	decf	current_on_board_h, 1			;yep
	return									;no
;***********************************************************************
ROUTE_ERASE:
	movf	current_mem_h, 0
	movwf	replace_mem_h	;copy the current location in memory - 
	movf	current_mem_l, 0
	movwf	replace_mem_l	;this will be the expired route info start
	movlw	0x05	
	subwf	replace_mem_l, 0				;subtract four to get back to the start of the expired route info and store again
	movwf	replace_mem_l
	btfsc	STATUS, N
	decf	replace_mem_h, 1				;if we went under the carry, decrement the high byte as well and store it
SHUFFLE_START:
	CS_SETUP
	movlw	READ						;Tell FRAM that we want to read
	call	SPI_ROUTINE
	movf	current_mem_h, 0			;from this high memory location
	call	SPI_ROUTINE
	movf	current_mem_l, 0			;and this low location
	call	SPI_ROUTINE
	call	SPI_ROUTINE					;and read back one byte
	movf	RXDATA, 0
	movwf	shuffle_buff		;and stash the byte on the move
	incf	current_mem_l, 1			;increment the read-from location
	btfsc	STATUS, C					;did we get a carry
	incf	current_mem_h, 1			;yep, increment high as well ay.
WRITE_OVER:
	CS_SETUP							;clear the !CS to enable chip
	movlw	WREN						;Enable writes
	call	SPI_ROUTINE
	CS_SETUP
	movlw	WRITE						;hey FRAM...gonna write to ya
	call	SPI_ROUTINE
	movf	replace_mem_h, 0			;starting here
	call	SPI_ROUTINE
	movf	replace_mem_l, 0			;and here
	call	SPI_ROUTINE
	movf	shuffle_buff, 0				;load up the byte to transfer
	call	SPI_ROUTINE					;and do it
	incf	replace_mem_l, 1			;increment replace memory location low byte 
	btfsc	STATUS, C					;did it overflow????
	incf	replace_mem_h, 1			;yep, increment high byte as well
	movf	highest_mem_h, 0			;move in the highest location we've written to
	cpfslt	current_mem_h				;are we less than this on the high byte?
	bra		CHECK_LOW_MEM				;no, better check low byte
	bra		SHUFFLE_START				;yes. if high byte is less than, then we still have more to go
CHECK_LOW_MEM:
	movf	highest_mem_l, 0			;move in the low byte of the highest location
	cpfsgt	current_mem_l				
	bra		SHUFFLE_START				;if we are less than this we still have more to go
;	call	ROUTE_CACHE_TEST	;TEST ROUTINE - Test the route Cache to ensure that we cached properly
	movlw	0x05
	subwf	highest_mem_l, 0
	movwf	highest_mem_l
	btfsc	STATUS, N
	decf	highest_mem_h, 1
	return								;if not less and high byte isn't less, we're done - get out

;***********************************************************************
LOCAL_MEM_SEARCH:
	call	FSR0_STACK
	movlw	low on_board_route_space
	movwf	local_search_pt_l
	movlw	high on_board_route_space
	movwf	local_search_pt_h
	movlw	low on_board_route_space
	movwf	FSR0L
	movlw	high on_board_route_space
	movwf	FSR0H
LOCAL_ZERO_WT:
	movf	current_on_board_h, 0
	cpfsgt	local_search_pt_h
	bra		LOW_CHECK_PT
	bra		ON_BOARD_EMPTY
LOW_CHECK_PT:
	movf	current_on_board_l, 0
	cpfslt	local_search_pt_l
	bra		ON_BOARD_EMPTY
	INC_LOCAL_TRACKER
	movf	POSTINC0, 0					;load in the byte we're looking at - set condition codes based on it
	bnz		LOCAL_ZERO_WT				;if its not zero, wait for one to come along
	INC_LOCAL_TRACKER
	movf	POSTINC0, 0					;else, load up the next one
	cpfseq	route_id_d					;is it what we're looking for??
	bra		LOCAL_ZERO_WT				;nope, wait again
	INC_LOCAL_TRACKER
	movf	POSTINC0, 0
	cpfseq	route_id_s
	bra		LOCAL_ZERO_WT
	INC_LOCAL_TRACKER
	movf	POSTINC0, 0
	movwf	dest_addr
	INC_LOCAL_TRACKER
	movf	POSTINC0, 0
	movwf	Tx_power_level
	INC_LOCAL_TRACKER
	movf	INDF0, 0
	movwf	v_count_val
	bsf		flag_buff2, FOUND_ONBOARD	;set the flag saying that we got a route
	bra		END_LOCAL_SEARCH
ON_BOARD_EMPTY:
	bcf		flag_buff2, FOUND_ONBOARD	;make sure we dont think we got a route
END_LOCAL_SEARCH:
	call	FSR0_UNSTACK
	return
;***********************************************************************
MEM_SEARCH:
	clrf	current_mem_h				;reset memory markers
	clrf	current_mem_l
	CS_SETUP
	movlw	READ						;gonna read from ya
	call	SPI_ROUTINE
	movlw 	0x00						;Start at bottom of memory
	call	SPI_ROUTINE
	movlw 	0x00						;Likewise with the low byte
	call	SPI_ROUTINE
WAIT_ZERO:
	movf	highest_mem_h, 0
	cpfsgt	current_mem_h
	bra		CHECK_LOW_WORD
	bra		NO_MEM_LEFT
CHECK_LOW_WORD:
	movf	highest_mem_l, 0
	cpfslt	current_mem_l
	bra		NO_MEM_LEFT
ROUTES_LEFT:
	call	SPI_ROUTINE					;get stuff via SPI
	INCREMENT_MEM_TRACKER
	movf	RXDATA, 1					;set condition codes based on Received data
	bnz		WAIT_ZERO					;is it the route deliminator? No branch and look at next byte
	call	SPI_ROUTINE					;yep, got the deliminator, read next byte
	INCREMENT_MEM_TRACKER
	movf	RXDATA, 0					;and move it into the WREG
	cpfseq	route_id_d					;is the route ID the same?
	bra		WAIT_ZERO					;NO, not the right destination route, skip out and find the next
	call	SPI_ROUTINE					;read next byte
	INCREMENT_MEM_TRACKER
	movf	RXDATA, 0					;and move it into the WREG
	cpfseq	route_id_s					;is it the right destination id
	bra		WAIT_ZERO					;no, wait for the next
	call	SPI_ROUTINE					;Yep, read next byte. This will be the next destination in the route
	INCREMENT_MEM_TRACKER
	movf	RXDATA, 0
	btfss	MAC_buff, RREQ_SEARCH		;if we're looking on a RREQ, don't alter the destination address
	movwf	dest_addr					;set it as the destination
	call	SPI_ROUTINE					;Yep, read next byte. This will be the Tx power level
	INCREMENT_MEM_TRACKER
	movf	RXDATA, 0
	movwf	Tx_power_level				;set it as the power level
	bsf		dsr_flags, FOUND_ROUTE		;set the flag if there is a route cached
	bra		END_MEM_SEARCH
NO_MEM_LEFT:
	bcf		dsr_flags, FOUND_ROUTE
END_MEM_SEARCH:
	return
;***********************************************************************
SPI_ROUTINE:
	MOVWF 	SSPBUF 					;New data to xmit
SPI_LOOP:
	BTFSS 	SSPSTAT, BF 			;Has data been received (transmit complete)?
	GOTO 	SPI_LOOP				;No
	MOVF 	SSPBUF, W 				;WREG reg = contents of SSPBUF
	MOVWF 	RXDATA 					;Save in user RAM, if data is meaningful
	return
;***********************************************************************
LOAD_No_route_buff:
	call	FSR0_STACK
	movlw 	high No_route_buff			;Move the high byte of the holding buffer
	movwf 	FSR0H 
	movlw 	low No_route_buff			;Move the low byte of the holding buffer
	movwf 	FSR0L
	movff	route_id_d, POSTINC0
	movff	route_id_s, POSTINC0
	movff	pkt_id, POSTINC0
	movlw	0x05
	movwf	data_bank_ldr
	movlw	high data_bank
	movwf	FSR2H
	movlw	low data_bank
	movwf	FSR2L
INFO_TRANSFER:
	movff	POSTINC2, POSTINC0
	decfsz	data_bank_ldr, 1
	bra		INFO_TRANSFER
	bsf		MAC_buff, RREQ_STORE	;set flag to say we have somthing in this buffer
	call	FSR0_UNSTACK
	return
;***********************************************************************
LOAD_BACKUP:
	call	FSR0_STACK
	movlw 	high backup_buff		;Move the high byte of the holding buffer to FSR
	movwf 	FSR0H 
	movlw 	low backup_buff			;Move the low byte of the holding buffer
	movwf 	FSR0L
	movlw 	high radioout 			;Move the high byte of the output data buffer address into the FSR1
	movwf 	FSR1H 
	movlw 	low radioout
	movwf 	FSR1L
	movf	txbufflen, 0
	addlw	0x01
	movwf	loadinglen
LOADING_LOOP:
	movff	POSTINC1, POSTINC0		;transfer 1 byte
	incf	backup_len, 1
	decf	loadinglen, 1			;decrement the count
	bnz		LOADING_LOOP			;if transfer isn't complete - keep going
	call	FSR0_UNSTACK			;unstack the previous contents of the FSR
	bsf		MAC_buff, BACKUP_FULL	;set the flag saying that the backup buffer is full
	return
;***********************************************************************
TIMEOUT_HANDLER:			;SET TO FIRE EVERY 1.31 seconds when enabled (20 times the TMR0 IRP)
	bcf		INTCON, TMR0IF				;yep, clear the interrupt flag		
	bcf		INTCON, TMR0IE		
	bcf		timer_flags, GENERAL_TIMEOUT
	bcf		flag_buff2, PARO_ON
	clrf	PARO_source
	clrf	PARO_dest
	btfsc	flag_buff, Physical				;check that nothing else is pending in physical layer
	bra		END_HANDLER
	btfsc	flag_buff, MAC					;or media access
	bra		END_HANDLER
	btfsc	flag_buff, Data_Link			;or Data Link
	bra		END_HANDLER
	btfsc	flag_buff, Network				;or Network Layers
	bra		END_HANDLER
	btfsc	tmr_on_flags, BACKOFF_ON
	call	BACKOFF_TMR_TIMEOUT
	btfsc	tmr_on_flags, RREQ_ID_TMR_ON
	call	RREQ_ID_TIMEOUT
	incf	validity_count, 1
	call	STACKED_PACKET_TIMEOUT
END_HANDLER:
	return
;***********************************************************************
START_REPLY_TIMER:
	movlw	D'20'
	movwf	timeout_counter
	clrf	TMR0L
	bcf		INTCON, TMR0IF				;before we turn it all on, clear the interrupt flag
	bsf		INTCON, TMR0IE				;now enable the overflow IRP
	bsf		T0CON, TMR0ON				;and turn on the timer
	bsf		flag_buff, STOPPEDNWAITING
	return
;***********************************************************************
BACKOFF_TMR_TIMEOUT:
	decf	backoff_timer, 1			;decrement the counter
	btfss	STATUS, Z					;did it zero?
	return								;no, get out
	btfsc	MAC_buff, MEDIA_BUSY		;are wil monitoring contention?
	return								;no, get out
	btfss	PORTB, RB1					;test the carrier sense bit
	bra		GAIN_CONTROL_OF_MEDIA
	rlcf	backoff_level, 1
	movff	backoff_level, backoff_timer
GAIN_CONTROL_OF_MEDIA:
	bcf		MAC_buff, MEDIA_BUSY
	bcf		tmr_on_flags, BACKOFF_ON
	bsf		MAC_buff, CONTROL_IS_OURS
	return
;***********************************************************************
RREQ_ID_TIMEOUT:						;time that we remember RREQ ID and don't forward others
	decf	current_rreq_timer, 1
	btfss	STATUS, Z
	return
	clrf	latest_route_d
	clrf	latest_route_s
	return
;***********************************************************************
;When we get here, the timer has expired and the program is still stopped and waiting
;for acknowledgement of the last packet we sent out. So, resend the packet that
;has been stacked in the backup buffer. Do this by setting timer_flags, PACKET_RETRY flag
;and passing control to the network layer.
STACKED_PACKET_TIMEOUT:
	btfss	flag_buff, STOPPEDNWAITING	;just double check we are waiting, should ever get here if we aren't
	return								;but if we do, just get out
	movf	retry_counter, 1			;load the retry counter over itself to set condition codes
	bz		NO_MORE_RETRIES				;if its zero, our time is up. Nothing left in the tanks 
	bsf		timer_flags, PACKET_RETRY	;otherwise, set the flag to retry the packet
	bsf		flag_buff, Network			;and hand control to the network layer
	bsf		dsr_flags, SEND_PACKET		;and set the falg that will let us break into the send packet routine
	decf	retry_counter, 1			;decrement the retry counter
	bcf		flag_buff, STOPPEDNWAITING	;remove the flag holdong out the network layer transmit
	return								;and get out: BUILD_PACKET will turn it all on again
NO_MORE_RETRIES:
	bcf		flag_buff, STOPPEDNWAITING	;no more stopping here
	movlw	0x03						;reset the retry counter
	movwf	retry_counter
	return								;and get out
;***********************************************************************
FSR0_STACK: 
	movff	FSR0H, FSR0_H_TEMP
	movff	FSR0L, FSR0_L_TEMP
	return

FSR0_UNSTACK:
	movff	FSR0_H_TEMP, FSR0H
	movff	FSR0_L_TEMP, FSR0L
	return
;***********************************************************************
DATA_CACHE:								;place data in its rightfull location
	movlw	low data_store
	movwf	FSR0L
	movlw	high data_store
	movwf	FSR0H
	movlw	0x09						;load in nine
	subwf	rxbufflen, 0				;and subtract it from the number of received bytes to account for header and CRC bytes
	movwf	data_bank_ldr				;and store
DATA_STORE_LOOP:
	movf	POSTINC1, 0					;move from radio in, to
	movwf	POSTINC0					;data bank
	decf	data_bank_ldr, 1			;decrement the counter
	bnz		DATA_STORE_LOOP				;and if we aren't done, loop til we are
	return
;***********************************************************************
;mod_power_X comes in in WREG - pass mod_X_mW back in WREG
DB_TO_MW_CONV:
	movwf	work_dB_val					;save the dB value we are passed
	movf	work_dB_val, 1				;load over itself to set condition codes
	bnz		NON_ZERO_CONV				;if the value wasn't negative, branch and go on
	movlw	D'10'						;if it was, load scaled version of 1mW into wreg
	return								;and return with it.
NON_ZERO_CONV:
	movlw	upper dB_mins				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;minimum RSSI boundaries
	movlw	high dB_mins
	movwf	TBLPTRH
	movlw	low dB_mins
	movwf	TBLPTRL
	clrf	tbl_offset					;reset the offset pointer
DB_MIN_LOOP:
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	movwf	tbl_dB_val					;and then somewhere secure
	tblrd*+								;increment the table pointer again to get around empty byte
	incf	tbl_offset, 1				;increment our offset counter
	incf	tbl_offset, 1				;increment our offset counter
	movf	work_dB_val, 0				;load up our measured value
	cpfslt	tbl_dB_val					;loop untill the value is greater than or egual to (not less than) a table value
	bra		FOUND_VALUE					;then get out
	bra		DB_MIN_LOOP					;as soon as the reading exceeds a table value we break out. otherwise, keep looking
FOUND_VALUE:
	movlw	0x02
	subwf	tbl_offset, 0
	movwf	tbl_offset
	;By this stage we have check that the RSSI reading is in the tolerance we want and know the offset required
	movlw	upper mW_values			;Load up the table pointer so it points at the
	movwf	TBLPTRU						;maximum RSSI boundaries
	movlw	high mW_values
	movwf	TBLPTRH
	movlw	low mW_values
	addwf	tbl_offset, 0				;add the offset we went to previously
	movwf	TBLPTRL
	btfsc	STATUS, C					;if we carried
	incf	TBLPTRH, 1					;increment
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG to return with it
	return
;***********************************************************************
mW_TO_dB:
	movwf	work_mW_val
	movlw	upper mW_values				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;minimum RSSI boundaries
	movlw	high mW_values
	movwf	TBLPTRH
	movlw	low mW_values
	movwf	TBLPTRL
	clrf	tbl_offset					;reset the offset pointer
FIND_mW_LOOP:
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	movwf	tbl_mW_val					;and then somewhere secure
	tblrd*+								;increment the table pointer again to get around empty byte
	incf	tbl_offset, 1				;increment our offset counter
	incf	tbl_offset, 1				;increment our offset counter
	movf	work_mW_val, 0				;load up our measured value
	cpfseq	tbl_mW_val					;loop untill the value is greater than or egual to (not less than) a table value
	bra		FIND_mW_LOOP				;then get out
	movlw	0x02
	subwf	tbl_offset, 0
	movwf	tbl_offset
	;By this stage we have check that the mW reading that corresponds to the dB power
	movlw	upper dB_values				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;maximum RSSI boundaries
	movlw	high dB_values
	movwf	TBLPTRH
	movlw	low dB_values
	addwf	tbl_offset, 0				;add the offset we went to previously
	movwf	TBLPTRL
	btfsc	STATUS, C					;if we carried
	incf	TBLPTRH, 1					;increment
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG to return with it
	return
;***********************************************************************
dB_TO_GAIN:
	movlw	upper dB_values				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;minimum RSSI boundaries
	movlw	high dB_values
	movwf	TBLPTRH
	movlw	low dB_values
	movwf	TBLPTRL
	clrf	tbl_offset					;reset the offset pointer
FIND_GAIN_LOOP:
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	movwf	tbl_dB_val					;and then somewhere secure
	tblrd*+								;increment the table pointer again to get around empty byte
	incf	tbl_offset, 1				;increment our offset counter
	incf	tbl_offset, 1				;increment our offset counter
	movf	Tx_power_level, 0			;load up our power level in dB
	cpfseq	tbl_dB_val					;loop untill the value is greater than or egual to (not less than) a table value
	bra		FIND_GAIN_LOOP				;then get out
	movlw	0x02
	subwf	tbl_offset, 0
	movwf	tbl_offset
	;By this stage we have check that the mW reading that corresponds to the dB power
	movlw	upper VGA_gains				;Load up the table pointer so it points at the
	movwf	TBLPTRU						;maximum RSSI boundaries
	movlw	high VGA_gains
	movwf	TBLPTRH
	movlw	low VGA_gains
	addwf	tbl_offset, 0				;add the offset we went to previously
	movwf	TBLPTRL
	btfsc	STATUS, C					;if we carried
	incf	TBLPTRH, 1					;increment
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG to return with it
	movwf	VGA_gain_set
	return	
;***********************************************************************
DISP_PACKET:
	NEW_LINE
	NEW_LINE
	STR_ADDR_SET 	strIntro
	call	SEND_STRING
	btfss	flag_buff3, INCOMMING_PKT
	bra		OUTGOING_PKT
	bcf		flag_buff3, INCOMMING_PKT
	NEW_LINE
	STR_ADDR_SET 	strInCome
	call	SEND_STRING
	bra		INTRO_DONE
OUTGOING_PKT:
	NEW_LINE
	STR_ADDR_SET 	strOutGo
	call	SEND_STRING
	movf	node_id, 0
	movwf	source_addr
INTRO_DONE:
	NEW_LINE
	STR_ADDR_SET 	strDest
	call	SEND_STRING
	movf	dest_addr, 0
	movwf	USART_Tx_byte
	call	BYTE_SEND
	NEW_LINE
	STR_ADDR_SET 	strSource
	call	SEND_STRING
	movf	source_addr, 0				;!!!!!!!!!!!!!!!!
	movwf	USART_Tx_byte
	call	BYTE_SEND
	NEW_LINE
	STR_ADDR_SET 	strPktTyp
	call	SEND_STRING
	movf	pkt_id, 0
	movwf	USART_Tx_byte
	call	BYTE_SEND
	NEW_LINE
	STR_ADDR_SET 	strRtIDd
	call	SEND_STRING
	movf	route_id_d, 0
	movwf	USART_Tx_byte
	call	BYTE_SEND
	NEW_LINE
	STR_ADDR_SET 	strRtIDs
	call	SEND_STRING
	movf	route_id_s, 0
	movwf	USART_Tx_byte
	call	BYTE_SEND
	NEW_LINE
	STR_ADDR_SET 	strHops
	call	SEND_STRING
	movf	hop_count, 0
	movwf	USART_Tx_byte
	call	BYTE_SEND
	NEW_LINE
	STR_ADDR_SET 	strData1
	call	SEND_STRING
DATA_TAIL:
	movf	POSTINC0, 0
	movwf	USART_Tx_byte
	call	BYTE_SEND
	STR_ADDR_SET 	strSpace
	call	SEND_STRING
	decf	Tx_data_len, 1
	btfss	STATUS, Z				;was the result zero?
	bra 	DATA_TAIL				;Nope, loop back to top
	return



;***********************************************************************
BYTE_SEND:
	movf	USART_Tx_byte, 0
	andlw	0xF0
	movwf	Std_value
	swapf	Std_value, 1
	call	ASCII_CONVERT
	movf	ASCII_value, 0
	movwf	TXREG
	bsf		TXSTA, TXEN
SEND_LOOP:
	btfss	TXSTA, TRMT				;Is the transmit register still full?
	bra		SEND_LOOP				;yep, loop till it's emptied
	movf	USART_Tx_byte, 0
	andlw	0x0F
	movwf	Std_value
	call	ASCII_CONVERT
	movf	ASCII_value, 0
	movwf	TXREG
	bsf		TXSTA, TXEN
SEND_LOOP1:
	btfss	TXSTA, TRMT				;Is the transmit register still full?
	bra		SEND_LOOP1				;yep, loop till it's emptied
END_STRING
	return							; return
;;***********************************************************************
SEND_STRING:
	clrwdt
	movlw 	0x0
	movwf 	TBLPTRU,0
	movf 	Low_String, 0
	movwf 	TBLPTRL
	movf 	High_String, 0
	movwf 	TBLPTRH
POP_ANOTHER:
	tblrd*
	movlw 	0x00					; Load W with 0
	cpfseq 	TABLAT					; Test if the read value was zero and skip it was
	bra 	SEND_CHAR				; Jump to the transmit section
	bra 	END_STRING1				; Exit the send string func.	
SEND_CHAR:
	movf	TABLAT, 0
	movwf	TXREG
	bsf		TXSTA, TXEN
SEND_LOOP2:
	btfss	TXSTA, TRMT				;Is the transmit register still full?
	bra		SEND_LOOP2				;yep, loop till it's emptied
	infsnz 	TBLPTRL,f 				; Increment point and skip if it isn't zero
	incf 	TBLPTRH,f				; Increment the high byte of pointer
	bra 	POP_ANOTHER				; Loop back to top
END_STRING1:
	return					; return
;***********************************************************************
ASCII_CONVERT:
	movlw	upper Hex_Values			;Load up the table pointer so it points at the
	movwf	TBLPTRU						;minimum RSSI boundaries
	movlw	high Hex_Values
	movwf	TBLPTRH
	movlw	low Hex_Values
	movwf	TBLPTRL
	clrf	tbl_offset					;reset the offset pointer
ASCII_LOOP:
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG temporarily
	tblrd*+								;increment the table pointer again to get around empty byte
	incf	tbl_offset, 1				;increment our offset counter
	incf	tbl_offset, 1				;increment our offset counter
	cpfseq	Std_value					;loop untill the value is the same
	bra		ASCII_LOOP					;then get out
	movlw	0x02
	subwf	tbl_offset, 0
	movwf	tbl_offset
	movlw	upper ASCII_Values			;Load up the table pointer so it points at the
	movwf	TBLPTRU						;maximum RSSI boundaries
	movlw	high ASCII_Values
	movwf	TBLPTRH
	movlw	low ASCII_Values
	addwf	tbl_offset, 0				;add the offset we went to previously
	movwf	TBLPTRL
	btfsc	STATUS, C					;if we carried
	incf	TBLPTRH, 1					;increment
	tblrd*+								;read in the table value and...
	movf	TABLAT, 0					;stash it in the WREG to return with it
	movwf	ASCII_value					;this is the ASCII value converted to Hex
	return
;***********************************************************************
;Testing Routines - To be removed in final version
SIM_LOAD_UP:
	call	FSR0_STACK
	movlw	high SIM_INPUT_BUFFER
	movwf	FSR0H
	movlw	low SIM_INPUT_BUFFER
	movwf	FSR0L
	movf	SIM_PACKET_INDEX, 0
	movf	PLUSW0, 0
	movwf	fresh_byte
	incf	SIM_PACKET_INDEX, 1
	movlw	D'32'
	cpfslt	SIM_PACKET_INDEX
	nop		;halt here
	clrf	flag_buff
	bsf		flag_buff, MAC
	bsf		MAC_buff, BYTE_IN
	call	FSR0_UNSTACK
	return
;***********************************************************************
ROUTE_CACHE_TEST:
	CS_SETUP
	movlw	READ					;gonna read from ya
	call	SPI_ROUTINE
	movlw	0x00					;starting here
	call	SPI_ROUTINE
	movlw	0x00					;and here
	call	SPI_ROUTINE
READ_LOOP:
	call	SPI_ROUTINE				;should be:
	movf	RXDATA, 0				;deliminator - 0x00
	bra		READ_LOOP
	return
;***********************************************************************

SIM_SETUP:
	clrwdt
	clrf	dsr_flags
	clrf	latest_route_d
	clrf	latest_route_s
	clrf	flag_buff
	clrf	SIM_PACKET_INDEX
	movlw	0x01
	movwf	node_id

;Set up the route cache
	clrwdt
	bsf		PORTC, NWP					;disable write protect
	bsf		PORTC, HOLD					;and don't halt the operation
	bsf		PORTC, NCS
	CS_SETUP							;clear the !CS to enable chip
	movlw	WREN						;Enable writes
	call	SPI_ROUTINE
	CS_SETUP
	movlw	WRITE						;Say were gonna write
	call	SPI_ROUTINE
	movlw	0x00						;high target address byte
	call	SPI_ROUTINE
	movlw	0x00						;low byte
	call	SPI_ROUTINE
	movlw	0xFE						;load out FE
	call	SPI_ROUTINE
	movlw	0xAA						;load out AA
	call	SPI_ROUTINE
	movlw	0x01						;Junk
	call	SPI_ROUTINE
	movlw	0x00						;Deliminator
	call	SPI_ROUTINE
	movlw	0x08						;Route ID - destination
	call	SPI_ROUTINE
	movlw	0x05						;Route ID - source
	call	SPI_ROUTINE
	movlw	0xEE						;Next Hop 
	call	SPI_ROUTINE
	movlw	0x01						;Tx Power 
	call	SPI_ROUTINE
	movlw	0x00						;Deliminator
	call	SPI_ROUTINE
	movlw	0x03						;Route ID - destination
	call	SPI_ROUTINE
	movlw	0x09						;Route ID - source!!!used to be 4
	call	SPI_ROUTINE
	movlw	0xF8						;Next Hop 
	call	SPI_ROUTINE
	movlw	0x01						;Tx Power 
	call	SPI_ROUTINE
	movlw	0x00						;Deliminator
	call	SPI_ROUTINE
	movlw	0x05						;Route ID - destination
	call	SPI_ROUTINE
	movlw	0x13						;Route ID - source
	call	SPI_ROUTINE
	movlw	0xAB						;Next Hop 
	call	SPI_ROUTINE
	movlw	0x01						;Tx Power 
	call	SPI_ROUTINE
	bsf		PORTC, NCS					;turn chip off again to terminate write	
	

	movlw	0x12
	movwf	highest_mem_l
;	clrf	highest_mem_h
;;****************************************
	movlw	high data_bank 
	movwf	FSR0H
	movlw	low data_bank 
	movwf	FSR0L
	movlw	0xAA						;Data Stuff
	movwf	POSTINC0
	movlw	0xCC						;Deliminator
	movwf	POSTINC0
	movlw	0xAA						;Data Stuff
	movwf	POSTINC0
	movlw	0xCC						;Deliminator
	movwf	POSTINC0
	movlw	0x04
	movwf	data_len
	movlw	0x05
	movwf	data_bank_ldr

	movlw	0x35
	movwf	target_addr
	movlw	0x04
	movwf	RREQ_cause_addr
	bsf		dsr_flags, RREQ_SENT

;	bsf		dsr_flags, SEND_PACKET
;	bsf		flag_buff, Network
;
	movlw	D'105'
	movwf	min_rx_pwr

;;SIMULATION INPUT PACKET - CALL_SIMINPUT ROLLS ANOTHER OF THESE BYTES OFF THE STACK
;;Set up the radioin buffer
	movlw	high SIM_INPUT_BUFFER
	movwf	FSR0H
	movlw	low SIM_INPUT_BUFFER
	movwf	FSR0L
	movlw	0xAA						;Pre-Amble
	movwf	POSTINC0					;Pre-Amble
	movwf	POSTINC0					;Pre-Amble
	movwf	POSTINC0					;Pre-Amble
	movwf	POSTINC0					;Pre-Amble
	movwf	POSTINC0					;Pre-Amble
	movlw	0x01						;Destination Address
	movwf	POSTINC0
	movlw	0x74						;Source Address
	movwf	POSTINC0
	movlw	RREP_V						;Packet ID
	movwf	POSTINC0
	movlw	0x0E						;Tx Pwr
	movwf	POSTINC0
	movlw	0x01						;Route ID - destination
	movwf	POSTINC0
	movlw	0x04						;Route ID - source
	movwf	POSTINC0
	movlw	0x05						;Hop Count
	movwf	POSTINC0
	movlw	0x04						;Data Field
	movwf	POSTINC0
	movlw	0xA1						;Data Field
	movwf	POSTINC0
	movlw	0xBB						;Data Field
	movwf	POSTINC0
	movlw	0xCC						;Data Field
	movwf	POSTINC0
	movlw	0xDD						;Data Field
	movwf	POSTINC0
	movlw	0xD9						;CRC High
	movwf	POSTINC0
	movlw	0xCC						;CRC Low
	movwf	POSTINC0
	movlw	0xAA						;End-Of-Frame Character
	movwf	INDF0

	movlw	0x0E
	movwf	rxbufflen
;
;	movlw	0xFF
;	movwf	RREQ_cause_addr
;;	bsf		dsr_flags, RREQ_SENT
;
;	movlw 	high on_board_route_space			;Move the high byte of the holding buffer that we stuck stuff
;	movwf 	FSR0H 						;in before we went out and looked for the route
;	movlw 	low on_board_route_space				
;	movwf 	FSR0L
;	movlw	0x00
;	movwf	POSTINC0
;	movlw	0x08						;Route ID - Destination Address
;	movwf	POSTINC0
;	movlw	0x01						;Route ID - Source Address
;	movwf	POSTINC0
;	movlw	0x26						;Next Hop
;	movwf	POSTINC0
;	movlw	D'14'						;Tx Power Level
;	movwf	POSTINC0
;	movlw	0xB4						;counter value
;	movwf	POSTINC0
;	movlw	0x00
;	movwf	POSTINC0
;	movlw	0x05						;Route ID - Destination Address
;	movwf	POSTINC0
;	movlw	0x01						;Route ID - Source Address
;	movwf	POSTINC0
;	movlw	0xEE						;Next Hop
;	movwf	POSTINC0
;	movlw	D'14'						;Tx Power Level
;	movwf	POSTINC0
;	movlw	0xF2						;counter value
;	movwf	POSTINC0
;	movlw	0x00
;	movwf	POSTINC0
;	movlw	0x35						;Route ID - Destination Address
;	movwf	POSTINC0
;	movlw	0x01						;Route ID - Source Address
;	movwf	POSTINC0
;	movlw	0xFE						;Next Hop
;	movwf	POSTINC0
;	movlw	D'14'						;Tx Power Level
;	movwf	POSTINC0
;	movlw	0x14						;counter value
;	movwf	POSTINC0
;
	movlw	high on_board_route_space
	movwf	current_on_board_h 
	movlw	low on_board_route_space
	addlw	0x12
	movwf	current_on_board_l
	btfsc	STATUS, C
	incf	current_on_board_h, 1
	return

;***DATA VALUES IN LOOKUP TABLES***
;Used to determine a dB Value from the RSSI analog input
;Minimum bounds
RSSI_mins:
	DATA 	D'0'
	DATA	D'78'
	DATA 	D'92'
	DATA	D'119'
	DATA 	D'148'
	DATA	D'175'
	DATA 	D'203'
	DATA	D'226'
	DATA 	D'237'
	DATA	D'255'			;Value to max out at and force the search on
;Maximum Bounds
RSSI_maxs:
	DATA	D'78'
	DATA 	D'92'
	DATA	D'119'
	DATA 	D'148'
	DATA	D'175'
	DATA 	D'203'
	DATA	D'226'
	DATA 	D'237'
	DATA 	D'255'
;Corresponding dB values
power_values:
	DATA	D'105'
	DATA 	D'100'
	DATA	D'90'
	DATA 	D'80'
	DATA	D'70'
	DATA 	D'60'
	DATA	D'50'
	DATA 	D'40'
	DATA 	D'30'
;Values used to convert from dB to mW values
dB_mins:
	DATA	D'0'
	DATA 	D'2'
	DATA	D'5'
	DATA 	D'8'
	DATA 	D'11'
	DATA	D'15'		;Value to max it out and force it on

dB_maxs:
	DATA	D'2'
	DATA 	D'5'
	DATA	D'8'
	DATA 	D'11'
	DATA 	D'14'

mW_values:
	DATA	D'1'	
	DATA	D'2'		;Used to be 1
	DATA 	D'3'
	DATA	D'6'
	DATA 	D'12'
	DATA 	D'25'

dB_values:
	DATA	D'0'	
	DATA	D'2'		
	DATA 	D'5'
	DATA	D'8'
	DATA 	D'11'
	DATA 	D'14'

VGA_gains:
;Values for bench top so we don't overload receiver front end
	DATA	0x00		;Neg 1 dB - as good as zero: -11
	DATA	0x08		;2dB:	-7
	DATA	0x06		;5dB:	-4
	DATA	0x0C		;8dB:	-1
	DATA	0x0C		;11dB:	-1
	DATA	0x0C		;14d:	-1
;True Values
;	DATA	0x0C		;Neg 1 dB - as good as zero
;	DATA	0x02		;2dB
;	DATA	0x0A		;5dB
;	DATA	0x06		;8dB
;	DATA	0x0E		;11dB
;	DATA	0x01		;14dB
ASCII_Values:
	DATA	0x30						;ASCII 0
	DATA	0x31						;ASCII 1
	DATA	0x32						;ASCII 2
	DATA	0x33						;ASCII 3
	DATA	0x34						;ASCII 4
	DATA	0x35						;ASCII 5
	DATA	0x36						;ASCII 6
	DATA	0x37						;ASCII 7
	DATA	0x38						;ASCII 8
	DATA	0x39						;ASCII 9
	DATA	0x41						;ASCII A
	DATA	0x42						;ASCII B
	DATA	0x43						;ASCII C
	DATA	0x44						;ASCII D
	DATA	0x45						;ASCII E
	DATA	0x46						;ASCII F

Hex_Values:
	DATA	0x00						;0
	DATA	0x01						;1
	DATA	0x02						;2
	DATA	0x03						;3
	DATA	0x04						;4
	DATA	0x05						;5
	DATA	0x06						;6
	DATA	0x07						;7
	DATA	0x08						;8
	DATA	0x09						;9
	DATA	0x0A						;A
	DATA	0x0B						;B
	DATA	0x0C						;C
	DATA	0x0D						;D
	DATA	0x0E						;E
	DATA	0x0F						;F

strCR:		DATA 0x0D, 0
strLF:		DATA 0x0A, 0
strSpace:	DATA " ",0
strIntro:	DATA "Ad Hoc Network Node Interface", 0
strInCome:	DATA "Incomming Packet Display:", 0
strOutGo:	DATA "Outgoing Packet Display:", 0
strDest:	DATA "Destination Address: ", 0
strSource:	DATA "Source Address: ", 0
strPktTyp:	DATA "Packet Type: ", 0
strPwr:		DATA "Transmit Power Level: ", 0
strRtIDd:	DATA "Route ID (destination): ", 0
strRtIDs:	DATA "Route ID (source): ", 0
strHops:	DATA "Hop Count: ", 0
strData1:	DATA "Data Tail: ", 0

 end
