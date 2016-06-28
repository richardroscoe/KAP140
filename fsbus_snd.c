/*
 * This file contains the code to send an R-command,
 */
#include <stdlib.h>
//#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>
#include "uart.h"
#include "fsbus.h"


void fsbus_snd(uint8_t cid, uint8_t rcmd, int8_t rcmd_v, uint8_t rcmd_len)
{
	uint8_t snd_buf[4];
	uint8_t i;

	snd_buf[0] = FS_DF_START | (cid << 2) |
					 (rcmd >> 7) | (rcmd_v & FS_DF_B1_V0);
	snd_buf[1] = rcmd & FS_DF_B2_CMD_MASK;
	snd_buf[2] = ((rcmd_v >> 1) & FS_DF_B3_V1_7);

//	printf("Sending ");
	for (i = 0; i < rcmd_len; i++) {
//		printf(" 0x%x", snd_buf[i]);
		uart_putc(snd_buf[i]);
	}
//	printf("\n\r");
}
