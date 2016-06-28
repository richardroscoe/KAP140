/*
 * This file contains the code to create a clock and handle timed events.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "event.h"
#include "switches.h"
#include "soft_uart.h"

#define CLOCK	200L		// clock 200Hz = 5msec

void clock_isr(void);

/*
 * These variables count are used to count the number of ticks a button was pressed and are used
 * for debouncing
 */
void clock_init(void)
{

	printf("clock_init()\n\r");


	TCCR1A = 0;							// ctc, all pins normal
	TCCR1B = _BV(CS10) | _BV(WGM12);	// ctc, no prescaler

	OCR1A = (F_CPU / (SOFT_BAUD_RATE * 4)) - 1;

	/*

    // use CLK/8 prescale value, clear timer/counter on compareA match                               
	TCCR1B =  _BV(CS11)  | _BV(WGM12);
    
    // preset timer1 high/low byte
    OCR1A = ((F_CPU/8/CLOCK) - 1 );   
*/

    // enable Output Compare 1 overflow interrupt
#if defined(__AVR_ATmega128__)
    TIMSK  |= _BV(OCIE1A);
#elif defined(__AVR_ATmega644__)
	TIMSK1  |= _BV(OCIE1A);
#endif

	/* Register the switch reading routine */
	printf("clock_init() - exit\n\r");
}

static uint8_t ev_ticks = CLOCK/EVENT_HZ;

ISR(TIMER1_COMPA_vect)
{
	static uint16_t clock = 0;

	clock++;
	soft_uart_isr();

	if (clock == ((SOFT_BAUD_RATE * 4) / CLOCK)) {
		clock = 0;
		clock_isr();
	}
}



void clock_isr(void)
{

	/*
	 * We have two things to deal with
	 *   1. Key debouncing
	 *   2. Our event infrastructure
	 *
	 * Why aren't we dealing with these the same way? Well the event
	 * overhead isn't tiny and the keyboard interrupts are needed every 5ms.
	 * So the answer is performance
	 */

	// Key debouncing
	switches_tick();

	sei();

	// Now the event handler
	ev_ticks--;

	if (ev_ticks == 0) {
		ev_ticks = CLOCK/EVENT_HZ;
		event_tick();
	}
}
