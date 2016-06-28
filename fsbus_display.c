/*
 * This file contains the code to decode the FSBUS Display Commands
 */
#include <stdlib.h>
//#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>

#include "fsbus.h"
 

static const unsigned char fs_display_conv[] = { '0', '1', '2', '3',
										  '4', '5', '6', '7',
										  '8', '9', '-', 'S',
										  't', 'd', 'E', ' ' };

#define DIG_A 5
#define DIG_B 4
#define DIG_C 3
#define DIG_D 2
#define DIG_E 1
#define DIG_F 0
#define DIG_NULL 6


void fsbus_display_decode(fsbus_block_t *fs_blk)
{

	fsbus_display_t *dp;


	dp = &fs_blk->fs_display;

	switch (fs_blk->fs_rcmd) {
	case FS_RCMD_RESET:
//		printf("fsbus_display_decode:FS_RCMD_RESET\n\r");
		dp->fs_power = 100;
		dp->fs_decimal_point = 0; // OFF
		dp->fs_digits[DIG_A] = '0';
		dp->fs_digits[DIG_B] = '0';
		dp->fs_digits[DIG_C] = '0';
		dp->fs_digits[DIG_D] = '0';
		dp->fs_digits[DIG_E] = '0';
		dp->fs_digits[DIG_F] = '0';
		break;

	case FS_RCMD_SETCID:
//		printf("fsbus_display_decode:FS_RCMD_SETCID\n\r");
		break;

	case FS_RCMD_SETBRIGHT:
//		printf("fsbus_display_decode:FS_RCMD_SETBRIGHT (%d)\n\r", fs_blk->fs_rcmd_v);
		dp->fs_bright = fs_blk->fs_rcmd_v;
		break;

	case FS_RCMD_SETPOWER:
//		printf("fsbus_display_decode:FS_RCMD_SETPOWER (%d)\n\r", fs_blk->fs_rcmd_v);
		dp->fs_power = fs_blk->fs_rcmd_v;
		break;

	case FS_RCMD_SETDECIMALPOINT:
//		printf("fsbus_display_decode:FS_RCMD_SETDECIMALPOINT (%d)\n\r", fs_blk->fs_rcmd_v);
		dp->fs_decimal_point = fs_blk->fs_rcmd_v;
		break;

	case FS_RCMD_SETBASEBRIGHT:
//		printf("fsbus_display_decode:FS_RCMD_SETBASEBRIGHT (%d)\n\r", fs_blk->fs_rcmd_v);
		dp->fs_base_bright = fs_blk->fs_rcmd_v;
		break;

	case FS_RCMD_DISPLAY:
		//printf("fsbus_display_decode:FS_RCMD_DISPLAY\n\r");

		dp->fs_digits[DIG_A] = fs_display_conv[((fs_blk->fs_rcv_buf[1] & 0x30) >> 2) | ((fs_blk->fs_rcv_buf[2] & 0x30) >> 4)];
		dp->fs_digits[DIG_B] = fs_display_conv[(fs_blk->fs_rcv_buf[1] & 0x0F)];
		dp->fs_digits[DIG_C] = fs_display_conv[(fs_blk->fs_rcv_buf[2] & 0x0F)];

		dp->fs_digits[DIG_D] = fs_display_conv[((fs_blk->fs_rcv_buf[3] &0x30) >> 2) | ((fs_blk->fs_rcv_buf[4] & 0x30) >> 4)];
		dp->fs_digits[DIG_E] = fs_display_conv[(fs_blk->fs_rcv_buf[3] & 0x0F)];
		dp->fs_digits[DIG_F] = fs_display_conv[(fs_blk->fs_rcv_buf[4] & 0x0F)];

		dp->fs_digits[DIG_NULL] = 0;

		//printf("fsbus_display_decode: Display '%s'\n\r", dp->fs_digits);
		
		break;

	default:
//		printf("fsbus_display_decode: Unknown command\n\r");
		break;
	}

//	printf("fsbus_display_decode: Bytes 0x ");

//	for (i = 0; i < fs_blk->fs_rcv_len; i++) {
//		printf("%x ", fs_blk->fs_rcv_buf[i]); 
//	}
//	printf("\n\r");
}
