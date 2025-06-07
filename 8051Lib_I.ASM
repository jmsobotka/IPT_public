;///////////////////////////////////////////////////////////////////////////
;  8051 ASM Library file
;  Date:    Sept 27, 2007
;  Version: 1.0
;
;  SelectCode                 for
;  Bruce Hansen               DFW Instruments
;  5013 Peninsula Way         2544 Tarpley Rd # 116
;  Garland TX                 Carrollton, TX 75006
;  Copyright 2008
;
;  Change History
;  Version  Date      		Auth  Description
;  1.00     Sept 27, 2007	BH	  Initial
;
;     Peripheral Map
;     -----------------------------------------------------------------------
;     Peripheral | Uses                   | Interrupt  | For
;     -----------------------------------------------------------------------
;     UART0      | TIMER1 (baud)          | REC/TRN    | 485 Host
;     UART1      | TIMER1 (baud)          | REC/TRN    | 232 Maint
;     TIMER1     |                        | NONE       | Baudrate Gen
;
;//////////////////////////////////////////////////////////////////////////
$NOMOD51
$INCLUDE (C8051F020.INC)

; Bit checks in the accumulator for Serial Control bits SCON
RI1    BIT  ACC.0
TI1    BIT  ACC.1
REN1   BIT  ACC.4

EXTRN DATA (InRp, OutRp, InTp, OutTp, LError)
EXTRN BIT (TX_Int_Restart_Flag)
EXTRN XDATA (Rb,Tb)
EXTRN BIT (TX_Int_Restart_Flag1)

; MAXIM 3162 232 and 485 RX and TX Enables on UART1, Shared with C code
;TE232 BIT   P3.0  ;
;RE232 BIT   P3.1
DE485 BIT   P0.7  ; driver enable
RE485 BIT   P0.6        ;
; ADM489AR 485 RX and TX Enable on UART0, Shared with C code
;DR485 BIT   P3.7

; C sbit variables cannot be exported to ASM, but they can be
; exported from ASM to C. Since these bits are the critical RX and TX
; enables, to prevent any possiblity of mis-alignment they are declared
; and made public here
;PUBLIC TE232,RE232,DE485,RE485,DR485
PUBLIC DE485,RE485


;------------------------------------------------------------------------------
; Interrupt Vector Table for ASM handlers
;------------------------------------------------------------------------------
CSEG AT 23H     ; 23H is address for UART0 interrupt 8051F021
LJMP com_isr

;------------------------------------------------------------------------------
; Give each interrupt function its own code segment.
;------------------------------------------------------------------------------
int4_code_seg SEGMENT CODE ; segment for interrupt function
RSEG int4_code_seg ; switch to this code segment
USING 0 ; register bank for interrupt routine
;********************************************************************************
; This routine interfaces with the 8051 Library through Rb, Tb buffers in xdata
; This data MUST be located at xdata addresses 0x0000 and 0x0100 for the interrupt
; optimizations to work. If a different buffer size is required, just add a mask
;********************************************************************************
com_isr:
   PUSH  ACC
   PUSH  PSW
   MOV   PSW,#00H
   PUSH  DPL
   PUSH  DPH
; Receiver data interrupt?
   JNB   RI,Com_isr25      ; receive flag set?
   CLR   RI                ; clear interrupt
;
; Get UART0 character put it in the circle buffer (xdata 0 to ff)
   MOV   DPH, #1h          ; load high byte of tx circle buf 
   MOV   DPL,InRp          ; load low byte of address into R0
   INC   InRp              ; let the byte index wrap at 256
   MOV   A,SBUF0           ; get byte
   MOVX  @DPTR,A           ; save to buffer

;  On overflow, lose old data
   MOV   A,InRp
   CJNE  A,OutRp,Com_isr25
   INC   OutRp
   MOV   LError,#01H       ; set error code
Com_isr25:
;
;  Transmitter data interrupt.
   JNB   TI,Com_isr30      ; transmit flag set?
   CLR   TI                ; clear interrupt
   MOV   A,InTp         
   XRL   A,OutTp
   JZ    Com_isr28
   SETB  DE485             ; tx enable, rx disable
   SETB  RE485
   MOV   DPH, #2h          ; load high byte of tran circle buf address into internal 021 address register
   MOV   DPL,OutTp         ; load low byte of address into R0
   INC   OutTp             ; let the byte index wrap at 256
   MOVX  A,@DPTR           ; get byte
   MOV   SBUF0,A           ; send it
   CLR   TX_Int_Restart_Flag  ; clear restart flag

;~~~~~~~~~~~~~~~~~~~~~~~~~~
  MOV   A,#0h             ; slow down transmission
Com_isrSlow1:             ; to prevent byte overrun
  INC   A                 ; 51 can transmit faster than
  JNZ   Com_isrSlow1      ; than some receivers can receive
  MOV   A,#0h
Com_isrSlow2:
  INC   A
  JNZ   Com_isrSlow2
;~~~~~~~~~~~~~~~~~~~~~~~~~~
  SJMP  Com_isr30

Com_isr28:
   SETB  TX_Int_Restart_Flag ; set restart flag
   CLR  DE485               ; transmit off
   CLR  RE485             

Com_isr30:
   POP   DPH
   POP   DPL
   POP   PSW
   POP   ACC
   RETI

END

;*****************************************************************************
;
;  Description:
;
;  Arguments:
;  Returns:
;*****************************************************************************/
