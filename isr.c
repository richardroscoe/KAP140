// Interrupt service routines
#include <htc.h>
#include "defs.h"
#include "fsbus.h"

void interrupt my_isr(void)
{	
	// No interrupts generated

/*	if (T0IF) {
		avb[AVB_ALT]++;
	
		TMR0 = 0;
		T0IF = 0;
	}
	*/
	if (RCIF) {
		fsbus_rcv(RCREG);
	}
}
