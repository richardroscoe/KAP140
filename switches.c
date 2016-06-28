#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "event.h"


/*
 * Protos
 */
//void switches_read(uint16_t t);

/*
 * Global variables, representing the debounced switch state and changes
 */

volatile uint8_t sw_porta = 0;
volatile uint8_t sw_portb = 0;
volatile int8_t sw_enc_delta;

/*
 * These variables hold the current state of the keys and is used to check
 * If a key is pressed at a point in time.
 */
volatile uint8_t sw_porta_state = 0, sw_portb_state = 0;

void inline switches_init(void)
{

	printf("switches_init()\n\r");

	/* Setup the ports */
	DDRA = 0x00;	// All pins are INPUT
	PORTA = 0xFF;	// Enable pull ups
	DDRB = 0;		// All pins are INPUT
	PORTB = 0xFF;	// Enable pull ups

	printf("switches_init() - exit\n\r");
}


static inline void switches_encoder()
{  
	static uint8_t last_state = 0,last_cnt = 0;
	uint8_t new_state;



	new_state=PINB & (_BV(PINB1) | _BV(PINB0));
	if ((new_state^last_cnt)==(_BV(PINB1) | _BV(PINB0)) )
	{
		if ((new_state ^ last_state)==_BV(PINB1))
		{
			sw_enc_delta += 1;
		}
		else
		{
			sw_enc_delta -= 1;
		}



		last_cnt=new_state;
	}



	last_state=new_state;
}

/*
 * This function is called by the Timer 1 interrupt service routine
 * to handle the button debouncing
 */
void switches_tick()
{
	/* Read the input pins */

	
	static uint8_t porta_ct0 = 0, portb_ct0 = 0, porta_ct1 = 0, portb_ct1 = 0;
	uint8_t porta_now, portb_now;

	/*
	 * read current state of keys (active-low),
	 * clear corresponding bit in i when key has changed
	 */
	porta_now = sw_porta_state ^ ~PINA;   // key changed ?
  
	/* 
	 * ct0 and ct1 form a two bit counter for each key,  
	 * where ct0 holds LSB and ct1 holds MSB
	 * After a key is pressed longer than four times the
	 * sampling period, the corresponding bit in key_state is set
	 */
	porta_ct0 = ~( porta_ct0 & porta_now );			// reset or count ct0
	porta_ct1 = (porta_ct0 ^ porta_ct1) & porta_now;	    // reset or count ct1  
	porta_now &= porta_ct0 & porta_ct1;			    // count until roll over ?
	sw_porta_state ^= porta_now;			    // then toggle debounced state

	/*
	 * To notify main program of pressed key, the correspondig bit
	 * in global variable key_press is set.
	 * The main loop needs to clear this bit
	 */
	sw_porta |= sw_porta_state & porta_now;	// 0->1: key press detect


	/*
	 * And now PORTB
	 */

	/*
	 * read current state of keys (active-low),
	 * clear corresponding bit in i when key has changed
	 */
	portb_now = sw_portb_state ^ ~PINB;   // key changed ?
  
	/* 
	 * ct0 and ct1 form a two bit counter for each key,  
	 * where ct0 holds LSB and ct1 holds MSB
	 * After a key is pressed longer than four times the
	 * sampling period, the corresponding bit in key_state is set
	 */
	portb_ct0 = ~( portb_ct0 & portb_now );			// reset or count ct0
	portb_ct1 = (portb_ct0 ^ portb_ct1) & portb_now;	    // reset or count ct1  
	portb_now &= portb_ct0 & portb_ct1;			    // count until roll over ?
	sw_portb_state ^= portb_now;			    // then toggle debounced state

	/*
	 * To notify main program of pressed key, the correspondig bit
	 * in global variable key_press is set.
	 * The main loop needs to clear this bit
	 */
	sw_portb |= sw_portb_state & portb_now;

	switches_encoder();
}



