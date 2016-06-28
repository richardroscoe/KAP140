#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>

//#define DEBUG

#include "delay.h"
#include "lcd.h"
#include "uart.h"

#ifndef DEBUG
#include "soft_uart.h"
#endif

#include "event.h"

#include "kap.h"
#include "switches.h"
#include "fsbus.h"
#include "clock.h"

#ifndef F_CPU
#error "F_CPU Must be defined!"
#endif




/*
 * I/O Configuration
 *
 * Usart 0 - FSBUS
 * Software UART - Debugging
 * 
 */

#define FSBUS_BAUD_RATE	19200
#define TERM_BAUD_RATE	4800 

#ifdef DEBUG
static FILE term_str = FDEV_SETUP_STREAM(soft_uart_putchar, NULL, _FDEV_SETUP_WRITE);
#endif

static void inline init_lcd()
{
//	printf("initialising LCD\n\r");

	lcd_init (LCD_DISP_ON);
	lcd_home ();
	lcd_puts_P("Initialised");
	lcd_gotoxy(2,1);
	lcd_puts_P("KAP-140");

	delay_ms(500);

	lcd_clrscr(); // We don't need any messages now
}

int
main(void)
{
    /*
     *  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro 
     *  UART_BAUD_SELECT() (normal speed mode )
     *  or 
     *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
     */
    uart_init( UART_BAUD_SELECT(FSBUS_BAUD_RATE,F_CPU), 2 );

	soft_uart_init(); 	

	event_init();

    /*
     * now enable interrupt, since UART library is interrupt controlled
     */
    sei();

	init_lcd();

	switches_init();

	clock_init();	

#ifdef DEBUG 
	stdout = stdin = stderr = &term_str;	
	printf("kap140 project\n\r");
	printf("==============\n\r");
#endif

	kap_init();
	fsbus_main();
	return 0;
}
