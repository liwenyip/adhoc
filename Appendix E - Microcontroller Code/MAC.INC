;*************************************************************************
; MAC Layer Header File
; Version 1.00
; 16/09/2005

 extern     MAC_INIT, MAC_TEST                              ; Initialisation
 extern     FRAME_ENCODER, FRAME_DECODER, RESET_DECODER     ; High Prioirity ISR's
 extern     CSMA, TMR0_ISR, INT1_ISR, TRAFFIC_CONTROL       ; Low Priority ISR's
 extern     rxbuf, txbuf, mac_status
 #define    packet_received     mac_status, 0  ;Indicates we have received a packet.
 #define    packet_transmitted  mac_status, 1  ;Indicates we are ready to send a packet.
 #define    retry_limit_reached mac_status, 2  ;Indicates we reached the retry limit.


