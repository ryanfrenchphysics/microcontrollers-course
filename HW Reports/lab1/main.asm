;-------------------------------------------------------------------------------
; Definitions
;-------------------------------------------------------------------------------

            .cdecls C,LIST,"msp430.h"       ; Include device header file
            ;.cdecls C,LIST,"defines.h"		; Include custom I2C definitions
            
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.
;-------------------------------------------------------------------------------
; Setup
;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer
			bic.b	#LOCKLPM5, &PM5CTL0

SDA_PIN			.set	BIT2
SCK_PIN			.set	BIT3
LED_PIN			.set	01h
DELAYLOOPS		.set	1

ACK				.set	0
NACK			.set	1

init:
			bic.w 	#0001h, &PM5CTL0      	; Disable GPIO power-on default high-Z mode
			bis.b 	#SDA_PIN,  &P1DIR		; Set P1.2, 1.3 as output
			bis.b	#SCK_PIN,	&P1DIR		; 1.2: SDA, 1.3:SCK
			bis.b	#LED_PIN,	&P1DIR		; Test pin (flash LED)

			bic.b	#SDA_PIN, &P1OUT			; Set SDA,
			bic.b	#SCK_PIN, &P1OUT			; SCK,
			bic.b	#LED_PIN, &P1OUT			; LED low

			bis.w	#TBCLR, &TB0CTL
			bis.w	#TBSSEL__ACLK, &TB0CTL
			bis.w	#MC__CONTINUOUS, &TB0CTL
			bis.w	#TBIE, &TB0CTL			; Overflow interrupt
			bis.w	#CNTL_1, &TB0CTL
			bis.w	#TBIFG, &TB0CTL
			bis.w	#ID__8_L, &TB0CTL	; Prescalar: 8
			nop
			;bis.w	#GIE, SR				; Enable interrupts
			nop



;-------------------------------------------------------------------------------
; Main loop
;-------------------------------------------------------------------------------
main:


			call	#start		; 00000101 (address = 5)

Addr:
			call 	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#low
			call	#high
			call	#low

			call	#ReadAck		; ACK
			cmp.b	#0,	R6
			jz		Restart


Byte1:
			call	#low		; Send 0x01
			call	#low
			call	#low
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart


Byte2:
			call	#low		; Send 0x02
			call	#low
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#low

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte3:
			call	#low		; Send 0x03
			call	#low
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#high

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte4:
			call	#low		; Send 0x04
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#low
			call	#low

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte5:
			call	#low		; Send 0x05
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#low
			call	#high

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte6:
			call	#low		; Send 0x06
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#high
			call	#low

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte7:

			call	#low		; Send 0x07
			call	#low
			call	#low
			call	#low
			call	#low
			call	#high
			call	#high
			call	#high

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte8:
			call	#low		; Send 0x08
			call	#low
			call	#low
			call	#low
			call	#high
			call	#low
			call	#low
			call	#low

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Byte9:
			call	#low		; Send 0x09
			call	#low
			call	#low
			call	#low
			call	#high
			call	#low
			call	#low
			call	#high

			call	#ReadAck
			cmp.b	#0,	R6
			jz		Restart

Restart:
			call	#stop
			call	#LongDelay
			call	#LongDelay
			jmp		main



;-------------------------------------------------------------------------------
; I2C Functions
;-------------------------------------------------------------------------------
idle:
			bis.b	#SDA_PIN, &P1OUT
			bis.b	#SCK_PIN, &P1OUT
			ret

start:
			bic.b	#SDA_PIN, &P1OUT
			call	#ShortDelay
			bic.b	#SCK_PIN, &P1OUT
			call	#ShortDelay
			ret

stop:
			;bic.b	#SCK_PIN, &P1OUT
			;call	#ShortDelay
			bic.b	#SDA_PIN, &P1OUT
			call	#ShortDelay
			bis.b	#SCK_PIN, &P1OUT
			call	#ShortDelay
			bis.b	#SDA_PIN, &P1OUT
			ret

ReadBit:
			call	#high
			ret

WriteBit:
			call	#low
			ret

ReadAck:
			bic.b	#SDA_PIN, &P1DIR		; Set SDA input
			call	#ShortDelay
			bis.b	#SCK_PIN, &P1OUT
			bit.b	#SDA_PIN, &P1IN
			jz		ContReadAck
			bis.b	#SDA_PIN, &P1DIR		; Set SDA output
			bic.b	#SDA_PIN, &P1OUT
			call	#ShortDelay
			bic.b	#SCK_PIN, &P1OUT
			mov.b	#1, R6
			call	#ShortDelay
			ret

ContReadAck:
			bis.b	#SDA_PIN, &P1DIR		; Set SDA output
			bic.b	#SDA_PIN, &P1OUT
			call	#ShortDelay
			bic.b	#SCK_PIN, &P1OUT
			mov.b	#0, R6				; Set ACK/NACK flag
			call	#ShortDelay
			ret



high:
			bis.b	#SDA_PIN, &P1OUT
			call 	#ShortDelay
			bis.b	#SCK_PIN, &P1OUT
			call	#ShortDelay
			bic.b	#SCK_PIN, &P1OUT
			call	#ShortDelay
			ret

low:
			bic.b	#SDA_PIN, &P1OUT
			call 	#ShortDelay
			bis.b	#SCK_PIN, &P1OUT
			call	#ShortDelay
			bic.b	#SCK_PIN, &P1OUT
			call	#ShortDelay
			ret



;-------------------------------------
; DELAY
;-------------------------------------
ShortDelay:
			mov.w	#800,	R8
			call	#Delay
			ret

LongDelay:
			mov.w	#5000,	R8
			call	#Delay
			ret


Delay:
			mov.w	#DELAYLOOPS, R4
			call	#LoopTest
			ret
DelayLoop:
			dec.w	R8					; Contains Delay Counter Val
			jnz		DelayLoop
			dec.w	R4
LoopTest:
			cmp.w	#0, R4
			jnz		DelayLoop
			ret





;isr:
;			xor.b	#SCK_PIN, &P1OUT
;			xor.b	#SDA_PIN,	&P1OUT
;			bic.w	#TBIFG, &TB0CTL
;			reti

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack
            

;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET
            
            ;.sect ".int42"
            ;.short isr



