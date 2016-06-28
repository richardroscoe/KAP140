/*
 * This file contains the code to decode the FSBUS DIO Commands
 */
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>
#include "fsbus.h"


void fsbus_dio_decode(fsbus_block_t *fs_blk)
{
	int i;
	fsbus_dio_t *iop;

	printf("fsbus_dio_decode:enter\n\r");

	iop = &fs_blk->fs_dio;

	switch (fs_blk->fs_rcmd) {
	case FS_RCMD_RESET:
//		printf("fsbus_dio_decode:FS_RCMD_RESET\n\r");
		for (i = 0; i < 4; i++)
			iop->fs_dout[i] = 0;

		for (i = 0; i < 8; i++)
			iop->fs_aout[i] = 0;
		break;

	case FS_RCMD_SETCID:
//		printf("fsbus_dio_decode:FS_RCMD_SETCID\n\r");
		break;

	case FS_RCMD_SETBRIGHT:
//		printf("fsbus_dio_decode:FS_RCMD_SETBRIGHT (%d)\n\r", fs_blk->fs_rcmd_v);
		break;

	case FS_RCMD_SETPOWER:
//		printf("fsbus_dio_decode:FS_RCMD_SETPOWER (%d)\n\r", fs_blk->fs_rcmd_v);
		break;

	case FS_RCMD_SETDECIMALPOINT:
//		printf("fsbus_dio_decode:FS_RCMD_SETDECIMALPOINT (%d)\n\r", fs_blk->fs_rcmd_v);
		break;

	case FS_RCMD_SETBASEBRIGHT:
//		printf("fsbus_dio_decode:FS_RCMD_SETBASEBRIGHT (%d)\n\r", fs_blk->fs_rcmd_v);
		break;

	default:
		if (fs_blk->fs_rcmd >= FS_RCMD_A_OUT_0 && fs_blk->fs_rcmd <= FS_RCMD_A_OUT_7) {
			i = fs_blk->fs_rcmd - FS_RCMD_A_OUT_0;
			iop->fs_aout[i] = fs_blk->fs_rcmd_v;
//			printf("fsbus_dio_decode: AOUT %d = 0x%x\n\r", i, fs_blk->fs_rcmd_v);

		} else if (fs_blk->fs_rcmd >= FS_RCMD_D_OUTBIT0_0 && fs_blk->fs_rcmd <= FS_RCMD_D_OUTBIT3_7) {
			i = fs_blk->fs_rcmd - FS_RCMD_D_OUTBIT0_0;
			if (fs_blk->fs_rcmd_v == 0)
				iop->fs_dout[i/8] &= ~(_BV(i % 8));
			else
				iop->fs_dout[i/8] |= _BV(i % 8);
			printf("fsbus_dio_decode: (OUTBIT) DOUT %d, bit %d = 0x%x\n\r", i/8, i % 8, fs_blk->fs_rcmd_v);			
		} else if (fs_blk->fs_rcmd >= FS_RCMD_D_OUTBYTE0 && fs_blk->fs_rcmd <= FS_RCMD_D_OUTBYTE3) {
			i = fs_blk->fs_rcmd - FS_RCMD_D_OUTBYTE0;
			iop->fs_dout[i] = fs_blk->fs_rcmd_v;
			printf("fsbus_dio_decode: (OUTBYTE) DOUT %d = 0x%x\n\r", i, fs_blk->fs_rcmd_v);

		} else {
//			printf("fsbus_dio_decode: Unknown command\n\r");
		}
		break;
	}

	printf("fsbus_dio_decode: Bytes 0x ");

	for (i = 0; i < fs_blk->fs_rcv_len; i++) {
		printf("%x ", fs_blk->fs_rcv_buf[i]); 
	}
	printf("\n\r");

	printf("fsbus_dio_decode:exit\n\r");
}
