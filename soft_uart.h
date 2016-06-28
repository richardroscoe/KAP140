#ifndef _SOFT_UART_H_
#define _SOFT_UART_H_


#define SOFT_BAUD_RATE 4800

#define uartrx PD2				// will be PD2,3 to talk to the laptop
#define	uarttx PD3


void soft_uart_init(void);
int soft_uart_putchar(char c, FILE *stream);
void soft_uart_isr(void);

#endif
