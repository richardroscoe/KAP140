#ifndef _EVENT_H_
#define _EVENT_H_

#define EVENT_HZ       10     // frequency in 1Hz
#define EVENT_MAX 10

typedef struct event_s {
	uint16_t	e_when;			/* Time when this needs to happen */
	uint8_t		e_period;		/* Periodicity (ticks) */
	uint8_t		e_occurrences;	/* Number of times (zero means forever) */
	void		(*e_func)();	/* Function to call - NULL indicates a free slot*/
} event_t;

typedef uint8_t event_handle;

extern event_handle event_register(void (*func)(), uint8_t period, uint8_t occurrences);
extern uint8_t event_cancel(volatile event_handle *h);
extern void event_reset(event_handle h);
extern void inline event_init();
extern void event_tick();
#endif
