.nolist
.include "m328Pdef.inc"
.list

; Delay for ADC convert
.equ	DELAY = $62 ; At 1024 divide, 97 is 10hz
.equ	HAS_DATA = 0
.equ	OVERFILL = 1
.equ	SEND_READY = 2
.equ	BASE_ADDRESS = SRAM_START + 0x0010
.equ	BUFFER_SIZE = 0x20

.def	TEMP = R16
.def	ADC_L = R17
.def	ADC_H = R18
.def	SRAM_TEMP = R19
.def	DATA = R20
.def	COUNTER_WRITE = R21
.def	COUNTER_READ = R22
.def	SREAD_POINTERL = R14
.def	SREAD_POINTERH = R15
.def	SWRITE_POINTERL = R11
.def	SWRITE_POINTERH = R12

.equ	SRAM_SREG = GPIOR0

.org	$0000
		rjmp	RESET

.org	ADCCaddr
		rjmp	ADC_COMPLETE

.org	OVF0addr
		reti

.org	SPIaddr
		rjmp	SPI_COMPLETE

.org	INT_VECTORS_SIZE

RESET:

		; Set ready to send data
		sbi		SRAM_SREG, SEND_READY

		; Set 'custom' SRAM pointer
		ldi		TEMP, low(BASE_ADDRESS)
		mov		SREAD_POINTERL, TEMP
		mov		SWRITE_POINTERL, TEMP
		
		ldi		TEMP, high(BASE_ADDRESS)
		mov		SREAD_POINTERH, TEMP
		mov		SWRITE_POINTERH, TEMP

		; Load counter with a 0 to signify at begining
		ldi		COUNTER_READ, 0x00
		ldi		COUNTER_WRITE, 0x00

		; Clear PRADC bit (to disable power saving)
		lds		TEMP, PRR
		andi	TEMP, ~(1 << PRADC)
		sts		PRR, TEMP

		; Set stack pointer
		ldi		TEMP, low(RAMEND)
		out		SPL, TEMP

		ldi		TEMP, high(RAMEND)
		out		SPH, TEMP

		; Set PC0 (ADC0) as an input
		in		TEMP, DDRC
		ori		TEMP, (0 << PORTC0)
		out		DDRC, TEMP 

		; Set PB1 (For my master SS) as an output
		; Also set PB6 as an output for ADC interrupt
		; Set PB3 as an output for my MOSI
		; Set PB5 as an output for SCK
		in		TEMP, DDRB
		ori		TEMP, (1 << PORTB1) | (1 << PORTB3) | (1 << PORTB5) |\
					(1 << PORTB6)
		out		DDRB, TEMP
		
		; Set it to HIGH and enable MISO pullup
		in		TEMP, PORTB
		ori		TEMP, (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB4)
		out		PORTB, TEMP


		; Setup ADC MUX
		ldi		TEMP, (0 << REFS1) | (0 << REFS0) | (0 << ADLAR) |\
					(0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0)
		sts		ADMUX, TEMP
		
		// Set Timer 0 Overflow
		ldi		TEMP, (1 << ACME) | (1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0)
		sts		ADCSRB, TEMP

		// Enable the ADC
		ldi		TEMP, (1 << ADEN) | (0 << ADSC) | (1 << ADATE) | (1 << ADIE) |\
					(0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)

		sts		ADCSRA, TEMP

		; Configure serial
		ldi		TEMP, (1 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) |\
					(0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (0 << SPR0)
		out		SPCR, TEMP

		ldi		TEMP, (0 << SPI2X)
		out		SPSR, TEMP

		; Setup timer 0
		ldi		TEMP, DELAY
		out		OCR0A, TEMP		

		ldi		TEMP, (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) |\
					(0 << COM0B0) | (1 << WGM01) | (1 << WGM00)
		out		TCCR0A, TEMP

		ldi		TEMP, (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0)
		sts		TIMSK0, TEMP

		ldi		TEMP, (0 << FOC0A) | (0 << FOC0B) | (1 << WGM02) |\
					(1 << CS02) | (0 << CS01) | (1 << CS00)
		out		TCCR0B, TEMP

		; Clear Serial Interrupt flag
		in		TEMP, SPSR
		ori		TEMP, (1 << SPIF)
		out		SPSR, TEMP

		; Enable global interrupts
		sei

MAIN:

		; Check to see if data needs to be sent
		sbis	SRAM_SREG, HAS_DATA
		rjmp	MAIN
		
		; Check to see if SEND_READY is set
		sbis	SRAM_SREG, SEND_READY
		rjmp	MAIN

		; If we get here, then we have data to send via serial
		cli		; Disable global interrupts for now

		; Get data from serial
		rcall	SRAM_READ

		; Set SS low
		cbi		PORTB, PORTB1

		; Toggle the pin to indicate reaching the interrupt
		sbi		PINB, PINB6

		; Begin serial transmission
		out		SPDR, SRAM_TEMP

		; Clear send ready
		cbi		SRAM_SREG, SEND_READY

		; Re-enable global interrupts
		sei

		; Jump back to main and wait for data to send
		rjmp	MAIN

ADC_COMPLETE:

		push	SRAM_TEMP
		push	ADC_L
		push	ADC_H
		push	YL
		push	YH

		; Build syncronized byte
		ldi		SRAM_TEMP, 0xFF
		rcall	SRAM_STORE

		; Analog to Ditital conversion has been completed
		lds		SRAM_TEMP, ADCL
		
		; Store in SRAM
		rcall	SRAM_STORE

		; Get High byte
		lds		SRAM_TEMP, ADCH	
			
		rcall	SRAM_STORE

		; Set the bit here. Check to see if bit is sent in main loop
		; and send it over. Check to make sure interrupt wont be an issue

		pop		YH
		pop		YL
		pop		ADC_H
		pop		ADC_L
		pop		SRAM_TEMP		

		reti

SRAM_STORE:

		sbic	SRAM_SREG, OVERFILL
		rjmp	S_STORE_SKIP

		; Store the data in SRAM and set HAS_DATA bit in RAM SREG
		push	TEMP
		push	YL
		push	YH		

		; Get the address ready
		mov		YL, SWRITE_POINTERL
		mov		YH, SWRITE_POINTERH

		st		Y+, SRAM_TEMP

		; Increment one to counter, and it with buffer size, and see
		; if it's zero
		inc		COUNTER_WRITE

		andi	COUNTER_WRITE, (BUFFER_SIZE - 1)

		; Check if counter is 0
		tst		COUNTER_WRITE

		; Now do some branching to see if it is zero
		breq	POINTER_RESET
		rjmp	NO_POINTER_RESET
	
	POINTER_RESET:

		ldi		TEMP, low(BASE_ADDRESS)
		mov		SWRITE_POINTERL, TEMP

		ldi		TEMP, high(BASE_ADDRESS)
		mov		SWRITE_POINTERH, TEMP

		rjmp	SKIP_WPOINTER_COPY

	NO_POINTER_RESET:

		; Copy over new pointer
		mov		SWRITE_POINTERL, YL
		mov		SWRITE_POINTERH, YH
		
	SKIP_WPOINTER_COPY:

		; Set HAS_DATA in the SREG
		sbi		SRAM_SREG, HAS_DATA
		
		; Check to see if write address = read address (to make sure)
		; we don't overwrite data
		cp		SWRITE_POINTERL, SREAD_POINTERL
		cpc		SWRITE_POINTERH, SREAD_POINTERH
		
		breq	BUFFER_FULL
		rjmp	S_STORE_END

	BUFFER_FULL:
		
		; Set OVERFILL flag in SREG
		sbi		SRAM_SREG, OVERFILL

	S_STORE_END:

		pop		YH
		pop		YL
		pop		TEMP
	
	S_STORE_SKIP:

		ret

SRAM_READ:

		; Check to see if there is data to read
		sbis	SRAM_SREG, HAS_DATA
		rjmp	SRAM_EMPTY

		push	TEMP
		push	YL
		push	YH

		; Load up address for SRAM
		mov		YL, SREAD_POINTERL
		mov		YH, SREAD_POINTERH

		ld		SRAM_TEMP, Y+

		; Increment the counter
		inc		COUNTER_READ

		; And the counter with the buffer size
		andi	COUNTER_READ, (BUFFER_SIZE - 1)

		; Test to see if the counter is at 0
		tst		COUNTER_READ

		; Branching
		breq	RESET_READ_POINTER
		rjmp	NO_READ_RESET

	RESET_READ_POINTER:
	
		; RESET pointer addresses to base address	
		ldi		TEMP, low(BASE_ADDRESS)
		mov		SREAD_POINTERL, TEMP

		ldi		TEMP, high(BASE_ADDRESS)
		mov		SREAD_POINTERH, TEMP

		rjmp	NO_READ_POINTER_COPY

	NO_READ_RESET:

		; Copy over current pointer data if no reset was done
		mov		SREAD_POINTERL, YL
		mov		SREAD_POINTERH, YH
		
	NO_READ_POINTER_COPY:

		; Clear the overfill bit (in case it was set)
		cbi		SRAM_SREG, OVERFILL

		; Check to see if we have read all the data
		cp		SREAD_POINTERL, SWRITE_POINTERL
		cpc		SREAD_POINTERH, SWRITE_POINTERH

		breq	SRAM_CLEARED
		rjmp	S_READ_END

	SRAM_CLEARED:
	
		; Set SREG for SRAM to empty		
		cbi		SRAM_SREG, HAS_DATA

	S_READ_END:

		pop		YH
		pop		YL
		pop		TEMP

	 SRAM_EMPTY:

		ret

SPI_COMPLETE:

		; set SS
		; Set it to HIGH
		sbi		PORTB, PORTB1

		; Set send ready to be complete
		sbi		SRAM_SREG, SEND_READY

		reti
