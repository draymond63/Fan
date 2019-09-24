;PROGRAM	:LCDfromADCinClass
;PURPOSE	:Obtains ADC of POT readings and configures it for LCD presentation
;AUTHOR		:C. D'Arcy
;DATE		:2019 04 30 
;DEVICE		:Arduino + POT + LCD
;MCU		:328p
;COURSE		:ICS4U
;STATUS		:Working
;REFERENCE	:https://www.microchip.com/webdoc/avrassembler/
;NOTES		:DD stands for Double Dabble (Shift/Add3) algorithm that converts binary (8 or 16) to packed BCD (3 or 5)
;NOTES		:Add include path to Project Properties dialog 
#include	<prescalers.inc>
#define		TOP			80		;16MHz/8/25000 = 80 = TOP (OCR2A)
.equ		cTHREE		=0x03	;DD: CONSTANTS for Double Dabble algorithm
.equ		cTHREEZERO	=0x30	;DD: ditto
.equ		POTAddress	=0x00	;LCD cursor addresses for labels
.equ		DCAddress	=0x08	;	"
.equ		RPMAddress	=0x40	;	"
.equ		TACHAddress	=0x48	;	"
.def		index		=r14	;used as a Z register address offset into the arrays
.def		temp		=r15	;temporary usage
.def		util		=r16	;readability is enhanced through the use of aliases for GP Registers
.def		count		=r17	;countdown support for various activities
.def		bin0		=r18	;DD: binary LOW byte
.def		bin1		=r19	;DD: binary HIGH byte
.def		BCD10		=r20	;DD: BCD: 2 Least significant BC Digits
.def		BCD32		=r21	;DD: BCD: Middle BC Digits
.def		BCD4		=r22	;DD: BCD: 1 Most Significant BC Digit
.def		three		=r23	;DD Registers: assigned the constant 0x03
.def		threeZero	=r24	;DD Registers: assign the constant 0x30
; ***** INTERRUPT VECTORS ************************************************
.org		0x0000				;start of vector jump table
	rjmp	reset				;lowest interrupt address => highest priority!
.org		ADCCaddr			;
	rjmp	ADC_vect			;
.org		INT_VECTORS_SIZE
; ***** LCD STRING LABELS ************************************************
POTLabel:
.db         "POT:",0,0			;must be null-terminated for Weiman's LCD routines (with even padding)
DCLabel:
.db         "DUTY:",0			;must be null-terminated for Weiman's LCD routines (with even padding)
RPMLabel:
.db			"RPM:",0,0			;ditto
TACHLabel:
.db         "TCH:",0,0			;ditto
DUTYTable:						;p.11:http://mail.rsgc.on.ca/~cdarcy/PDFs/Sunon12VDCFanSpec.pdf
.db			" 010203040506070809010"
RPMTable:						;p.11:http://mail.rsgc.on.ca/~cdarcy/PDFs/Sunon12VDCFanSpec.pdf
.db			" 900 900 900 9001200200027003400405045004800"

reset:							;PC jumps to here on reset interrupt...
    ldi     util,low(RAMEND)	;initialize the stack pointer to the highest RAM address
    out     SPL,util			;
    ldi     util,high(RAMEND)	;
    out     SPH,util			;

	ldi		three,cTHREE		;DD Support
	ldi		threeZero,cTHREEZERO;ditto
	
	call	ADCSetup			;Configure the ADC peripheral
	call	LCD_init			;initialize devices and MCU peripherals	
	sei
wait:
	rjmp	wait				;repeat or hold...
	ret							;unreachable

ADCSetup:
	ldi		util,1<<MUX0		;turn off digital use of A0 tp save power
	sts		DIDR0,util			;disable digital use of A0 (pin 14)
	ldi		util,1<<ADEN		;enable the A/D peripheral
	sts		ADCSRA,util			;
	ori		util,1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0	;divide down AD clock to 125kHz (recommended for accuracy)
	sts		ADCSRA,util			;set it

	clr		util				;not sure what the ACME bit is for so ignore it
	sts		ADCSRB,util			;free running, for now

	ldi		util, 1<<REFS0 | 1<<ADLAR	;AVCC reference (5V) and Left Adjust
	sts		ADMUX,util			;

	lds		util,ADCSRA			;prepare for a dummy conversion (recommended)
	ori		util,1<<ADSC		;set the SC flag
	sts		ADCSRA,util			;tstart your covnversion
dummy:
	lds		util,ADCSRA			;read the register
	sbrc	util,ADSC			;
	rjmp	dummy				;
	lds		util,ADCSRA			;
	ori		util,1<<ADSC|1<<ADIE;Start first (real) Conversion and enable interrupt
	sts		ADCSRA,util			;
	ret

T2Setup:
	sbi		DDRD,PD3			;PWM on pin 3 (OC2B)
	ldi		util, 1<<COM2B1 | 1<<WGM21 | 1<<WGM20	;
	sts		TCCR2A,util								;
	ldi		util, 1<<WGM22 | T2ps8					;
	sts		TCCR2B,util								;
	ldi		util,TOP								;
	sts		OCR2A,util								;
	clr		util									;
	sts		OCR2B,util								;
	ldi		util,OCIE2B								;
	sts		TIMSK2,util								;
	ret

ADC_vect:
	lds		temp,ADCH			;Grab the most significant 8 bits
	mov		util,temp			;let's determine the duty cycle...
	clr		r17					;
	ldi		r18,TOP				;
	clr		r19					;
	mul		r18,util			;
	sts		OCR2B,r1			;set the duty cycle
;perform the Double Dabble algorithm on the ADC value
	mov		bin0,temp			;restore the ADCH value and preapre for DD algorithm
	clr		bin1				;prepare bin1 (bin0 is already to go)
	call	bin16BCD			;convert to BCD
	call	LCDUpdate			;update the LCD
	lds		util,ADCSRA			;start the next conversion
	ori		util,1<<ADSC		;
	sts		ADCSRA,util			;
	reti

LCDUpdate:
	ldi     ZH, high(POTLabel)		;point to the base address of the POT Label
    ldi     ZL, low(POTLabel)		;	"
	ldi		util, POTAddress		;set the target address on the LCD 
    call    lcd_write_string_4d		;display it
	mov		util,BCD32				;obtain the hundreds (and thousands) digits
	andi	util,0x0F				;we only want BCD2 (the low nibble of BCD32)
	ori		util,0x30				;add 48 to get the ASCII value (addi doesn't exist, so we are dangerously adding)
	call	LCDWriteCharacter		;write it
	mov		util,BCD10				;obtain the units and tens digits
	swap	util					;work with the tens digit first			
	andi	util,0x0F				;mask it
	ori		util,0x30				;add 48 to get the ASCII value
	call	LCDWriteCharacter		;write it
	mov		util,BCD10				;reload 
	andi	util,0x0F				;mask off the tens digit exposing the units digit 
	ori		util,0x30				;add 48 to get the ASCII value
	call	LCDWriteCharacter		;write it

	ret								;return

LCDWriteCharacter:
	call	lcd_write_character_4d		;display the character
	ldi		util, 80					;40uS delay (min)
	call	delayTx1uS 
	ret
; ****************************** End of Main Program Code *******************
#include	<DoubleDabble.asm>
/*
#define		LOCALLCD		;This allows user code to override default (Appliance) wiring
.equ    lcd_D7_port         = PORTB		;lcd D7 connection
.equ    lcd_D7_bit          = PORTB4
.equ    lcd_D7_ddr          = DDRB

.equ    lcd_D6_port         = PORTB		;lcd D6 connection
.equ    lcd_D6_bit          = PORTB3
.equ    lcd_D6_ddr          = DDRB

.equ    lcd_D5_port         = PORTB		;lcd D5 connection
.equ    lcd_D5_bit          = PORTB2
.equ    lcd_D5_ddr          = DDRB

.equ    lcd_D4_port         = PORTB		;lcd D4 connection
.equ    lcd_D4_bit          = PORTB1
.equ    lcd_D4_ddr          = DDRB

.equ    lcd_E_port          = PORTD		;lcd Enable pin
.equ    lcd_E_bit           = PORTD7
.equ    lcd_E_ddr           = DDRD

.equ    lcd_RS_port         = PORTD		;lcd Register Select pin
.equ    lcd_RS_bit          = PORTD6
.equ    lcd_RS_ddr          = DDRD
*/
#include	<LCDLib.asm>				;insert additional code here (Weinman's LCD library code)
