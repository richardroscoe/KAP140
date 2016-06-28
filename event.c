#include <stdlib.h>
#include <avr/io.h>
//#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "event.h"
/*
 *
 *		1.	Provides a service, where you can register a function to be called every so
 *			many ticks
 *		2.	Another service where you register a function to be called so many ticks
 *			from now.
 *
 *	The event handle is the index + 1. This is so that the consumer can assume non null
 *  for a useful handle.
 */

static volatile event_t event_list[EVENT_MAX];

static volatile uint16_t tick = 0;

static uint8_t event_service = 0;

event_handle event_register(void (*func)(), uint8_t period, uint8_t occurrences)
{
	uint8_t i;

//	printf("event_register()\n\r");
	cli();
	for (i = EVENT_MAX - 1; i > 0; i--) {
		if (!event_list[i].e_func) {
			event_list[i].e_func = func;
			event_list[i].e_when = tick + period;
			event_list[i].e_period = period;
			event_list[i].e_occurrences = occurrences;
			break;
		}
	}
	sei();
//	printf("event_register: regisitered func %p, handle %d\n\r", func, i);
	return (event_handle) i + 1;
}

/*
 * Reset a registered event such that the starting point for a count down
 * is now
 */
void event_reset(event_handle h)
{
	h--;
	cli();
	event_list[h].e_when = tick + event_list[h].e_period;
	sei();
}

/*
 * Cancel a registered event
 */
uint8_t event_cancel(volatile event_handle *h)
{
//	printf("event_cancel(%d)\n\r", *h);
	cli();
	if (*h) {
		event_list[(*h) - 1].e_func = NULL;
		*h = 0;
	}
	sei();
	return 0;
}


void inline
event_init()
{
	uint8_t i;
/*
    // use CLK/64 prescale value, clear timer/counter on compareA match                               
    TCCR1B = _BV(CS10) | _BV(CS11)  | _BV(WGM12);
    
    // preset timer1 high/low byte
    OCR1A = ((F_CPU/2/64/EVENT_HZ) - 1 );   

    // enable Output Compare 1 overflow interrupt
    TIMSK  |= _BV(OCIE1A);
// Was a straight =
*/	

/*
    // use CLK/1024 prescale value, clear timer/counter on  match                               
    TCCR2 = _BV(CS22) | _BV(WGM21);
    
    // preset timer1 high/low byte
    OCR2 = (F_CPU / (2 * 1024 * EVENT_HZ)) - 1;
	
//	((F_CPU/2/1024/DEBOUNCE) - 1 );   
printf("event_init() - OCR2 = %d\n\r", (F_CPU / (2 * 1024 * EVENT_HZ)) - 1);  
    // enable Output Compare 2 overflow interrupt
    TIMSK  |= _BV(OCIE2);
*/

	for (i = EVENT_MAX; i > 0; i--) {
		event_list[i - 1].e_func = NULL;
	}
}

/*
 * Interrupt handler for our timer
 */
//ISR(TIMER1_COMPA_vect)    // handler for Output Compare 1 overflow interrupt
/*ISR(TIMER2_COMP_vect)*/
void event_tick()
{
	uint8_t i;
	void (*func)();

//static 	uint16_t	e_when = 0;			/* Time when this needs to happen */
//static void		(*e_func)() = NULL;

	cli();
	if (event_service) {
		sei();
		return;
	}
	event_service = 1;

	/* Do whatever needs to be done this tick */
	tick++;


	for (i = EVENT_MAX - 1; i > 0; i--) {
		cli();
/*
		if (i == 3 && e_when != event_list[i].e_when)
			printf("et: i = %d, f = %p, lw = %d, w = %d, tick = %d\n\r",
				i, event_list[i].e_func, e_when, event_list[i].e_when, tick);
*/
		if (event_list[i].e_func && event_list[i].e_when == tick) {

//			if (i == 3)
//				e_when = event_list[i].e_when;

			func = event_list[i].e_func;

			if (event_list[i].e_occurrences) {

				event_list[i].e_occurrences--;

				if (event_list[i].e_occurrences == 0) {
					/* This is the last time */
					event_list[i].e_func = NULL;
				} else {
					event_list[i].e_when = tick + event_list[i].e_period;
				}
			} else
				event_list[i].e_when = tick + event_list[i].e_period;

			sei();
			(*func)(tick);
		} else {
//			if (i == 3)
//				e_when = event_list[i].e_when;
		}
		sei();
	}
	cli();
	event_service = 0;
	sei();
}


