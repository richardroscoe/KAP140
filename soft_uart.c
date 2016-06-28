////////////////////////////////////////////////////////////////////////////////////////////
//
// Software uart routines
//
// we need two serial lines, and there's only one usart on the M32
// so we implement another in software
//
// we set up a clock with a timeout of 883 cycles - four times baud rate at 2400 baud - 
// and use it to sample an input line and clock an output line
// we also use it to maintain a 20mS tick for timouts and such
//
///////////////////////////////////////////////////////////////////////////////////////////
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "soft_uart.h"

// clock times
static volatile uint8_t ticks;			// 192 of these make 1/50th of a second

// software uart registers
static volatile uint8_t	uart_txd;		// the byte being transmitted
static volatile uint8_t	uart_txtick;	// tick count for the transmitter
static volatile uint8_t	uart_txbit;		// bit count for tx
static volatile uint8_t	uart_status;	// uart status

#define UART_TX_BUFFER_SIZE  64
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

// status bits
#define txbusy 0				// set if a byte is in transmission

//ISR(SIG_OUTPUT_COMPARE1A)
void soft_uart_isr(void)
{
	unsigned char tmptail;

	// the interrupt signal on a compare for the timer 1


	if (!(uart_status & _BV(txbusy))) {
	    if ( UART_TxHead != UART_TxTail) {
	        /* calculate and store new buffer index */
	        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
	        UART_TxTail = tmptail;
	        /* get one byte from buffer and write it to UART */
	        uart_txd = UART_TxBuf[tmptail];  /* start transmission */

			uart_status |= _BV(txbusy);
	    }
	}
	
	// we arrive here four times per baud rate
	// we first send the output signal, if there's anything to send,
	// since it needs to be somewhere close to accurate...
	// then we read the incoming signal, if there is one,
	// and finally we update the nearly-real-time clock
	// (we never display the clock but it timestamps the output data)
	
	if (uart_status & _BV(txbusy)) {
		// we're transmitting something
		// increment the tick - we only do things every four ticks
		uart_txtick++;
		if (uart_txtick == 4) {
			// okay, we've work to do
			uart_txtick = 0;
	
			// is it the start bit?
			if (uart_txbit == 0) {
				// yes...
				PORTD &= ~(_BV(uarttx));		// clear the start bit output
				uart_txbit++;
			} else {
				if (uart_txbit != 9){
					// deal with the data bits
					if (uart_txd & 1)		// low bit set?
						PORTD |= _BV(uarttx);	// then set the data stream bit
					else
						PORTD &= ~(_BV(uarttx));
												// or clear, as required
					uart_txbit++;				// increment the bit count
					
					// and shift the data right
					uart_txd /= 2;
				} else {
					// deal with the stop bit
					PORTD |= _BV(uarttx);
					uart_txbit++;
				}
			}
			
			// and finally, if txbit is more than 9, we've done
			if (uart_txbit > 9) {
				uart_txbit = 0;					// clear the bit counter
				uart_status &= ~(_BV(txbusy)); // and the busy status
			}
		}
	}
	
	// increment the clock
	ticks++;
}

void soft_uart_init (void)
{
	// we wake up the timer, preset the clock and uart variables, and enable the ocr1a interrupt
	ticks = 0;

/*
	TCCR1A = 0;							// ctc, all pins normal
	TCCR1B = _BV(CS10) | _BV(WGM12);	// ctc, no prescaler

	OCR1A = (F_CPU / (SOFT_BAUD_RATE * 4)) - 1;

    // enable Output Compare 1 overflow interrupt
#if defined(__AVR_ATmega128__)
    TIMSK  |= _BV(OCIE1A);
#elif defined(__AVR_ATmega644__)
	TIMSK1  |= _BV(OCIE1A);
#endif
*/								// allow interrupts on output mask a
	uart_status = 0;			// nothing happening either tx or rx

    UART_TxHead = 0;
    UART_TxTail = 0;
	
	DDRD |= _BV(uarttx);		// and set the input and output pins
	PORTD = _BV(uarttx);		// we're not using port d for anything else and
								// the usart overrides the pin directions anyway
}


void soft_uart_putc (char ch)
{
    unsigned char tmphead;

    
    tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;
    
    while ( tmphead == UART_TxTail ){
        ;/* wait for free space in buffer */
    }
    
    UART_TxBuf[tmphead] = ch;
    UART_TxHead = tmphead;
}

void soft_uart_print (char * t)
{
	// print a string of characters to the soft uart
	while (*t) {
		soft_uart_putc(*t);
		t++;
	}
}


int soft_uart_putchar(char c, FILE *stream)
{
	soft_uart_putc(c);
	return 0;
}
