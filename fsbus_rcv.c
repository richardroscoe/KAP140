/*
 * The code in this file deals with the reception of data from the FSBUS
 * When complete commands have been collected they are passed off for decoding 
 */
#include <stdlib.h>
//#include <stdio.h>
#include <avr/io.h>
#include <stdint.h>
#include "uart.h"
#include "fsbus.h"


/*
* Routines to handle incoming fsbus data
*/

//extern uint8_t fs_cid;
//extern fsbus_block_t *fs_blk;

uint8_t fs_cid;


// Special for displays
#define FS_DISPLAY_START 	FS_DF_B1_CMD_MASK
#define FS_DISPLAY_END		0x40

// Lookup table for length of a dataframe for a command
const static uint8_t fs_rcmd_dflen[6] = { 2, 3, 3, 3, 3, 3 };

/*
 * Receive handler functions
 */
void fs_rcv_dio(uint8_t c, fsbus_block_t *fs_blk);
void fs_rcv_display(uint8_t c, fsbus_block_t *fs_blk);

/*
 * Receive handler functions pointer table
 */
static void (*fs_rcv_func[])(uint8_t c, fsbus_block_t *fs_blk) = { fs_rcv_dio, fs_rcv_display, NULL };

#define FS_RCMD_A_OUT_0		80
#define FS_RCMD_A_OUT_7		87
#define FS_RCMD_D_OUTBIT0_0	88
#define FS_RCMD_D_OUTBIT0_7	95
#define FS_RCMD_D_OUTBIT1_0	96
#define FS_RCMD_D_OUTBIT1_7	103
#define FS_RCMD_D_OUTBIT2_0	104
#define FS_RCMD_D_OUTBIT2_7	111
#define FS_RCMD_D_OUTBIT3_0	112
#define FS_RCMD_D_OUTBIT3_7	119
#define FS_RCMD_D_OUTBYTE0	120
#define FS_RCMD_D_OUTBYTE1	121
#define FS_RCMD_D_OUTBYTE2	122
#define FS_RCMD_D_OUTBYTE3	123

/*
 * This function is the Digital I/O controller receive routine.
 */
void fs_rcv_dio(uint8_t c, fsbus_block_t *blk)
{
//	printf("fs_rcv_dio(%d) enter\n\r", c);

 	if (c & FS_DF_START) {
//		printf("fs_rcv_dio() got data frame start\n\r");

		blk->fs_rcv_len = 0;
 		blk->fs_rcmd = (c & FS_DF_B1_CMD_MASK) << 7 ; 
 		blk->fs_rcmd_len = 0; // Minimum length
		blk->fs_rcmd_v = c & FS_DF_B1_V0;	// Get the LSB of the value 
 	}
	blk->fs_rcv_buf[blk->fs_rcv_len] = c;
	blk->fs_rcv_len++;
	
	//printf("fs_rcv_dio: fs_rcv_len %d, exp len %d\n\r", fs_blk->fs_rcv_len, fs_blk->fs_rcmd_len);

	/* See if we know the command */
	if (blk->fs_rcv_len == 2) {

		blk->fs_rcmd |= c & FS_DF_B2_CMD_MASK;
		
		/* Get the expected length */
		if (blk->fs_rcmd >= 	FS_RCMD_RESET && blk->fs_rcmd <= FS_RCMD_SETBASEBRIGHT)
			blk->fs_rcmd_len = fs_rcmd_dflen[blk->fs_rcmd - FS_RCMD_RESET];
		else if (blk->fs_rcmd >= FS_RCMD_A_OUT_0 && blk->fs_rcmd <= FS_RCMD_A_OUT_7)
			blk->fs_rcmd_len = 3;
		else if (blk->fs_rcmd >= FS_RCMD_D_OUTBIT0_0 && blk->fs_rcmd <= FS_RCMD_D_OUTBIT3_7)
			blk->fs_rcmd_len = 2;
		else if (blk->fs_rcmd >= FS_RCMD_D_OUTBYTE0 && blk->fs_rcmd <= FS_RCMD_D_OUTBYTE3)
			blk->fs_rcmd_len = 3;	


		//printf("fs_rcv_dio: Got 2nd byte, command = %d, exp len = %d\n\r", blk->fs_rcmd, blk->fs_rcmd_len);
	}
 		
	/* See if we have received a complete command */

	if (blk->fs_rcmd_len && blk->fs_rcmd_len == blk->fs_rcv_len) {
		/* It's here !*/
		if (blk->fs_rcmd_len == 3)
			blk->fs_rcmd_v |= (c & FS_DF_B3_V1_7) << 1;
//		printf("fs_rcv_dio() got complete command (%d, v = 0x%x)\n\r", blk->fs_rcmd, blk->fs_rcmd_v);
		fsbus_dio_decode(blk);
		if (blk->fs_callback)
			(*blk->fs_callback)(blk);
	}
	
//	printf("fs_rcv_dio exit\n\r");
}


/*
 * This function is the display controller receive routine.
 */
void fs_rcv_display(uint8_t c, fsbus_block_t *blk)
{
//	printf("fs_rcv_display(%d) enter\n\r", c);

 	if (c & FS_DF_START) {
//		printf("fs_rcv_display() got data frame start\n\r");

		blk->fs_rcv_len = 0;
 		blk->fs_rcmd = 0;
 		blk->fs_rcmd_len = 0;
 		blk->fs_rcmd_v = 0;

 		if ((c & FS_DISPLAY_START) == 0) {
			// We have a display dataframe
//			printf("fs_rcv_display() got FS_DISPLAY_START c = 0x%x, mask = 0x%x\n\r", c, FS_DISPLAY_START);
			blk->fs_rcmd = FS_RCMD_DISPLAY;
 		} else {
 			blk->fs_rcmd_v = c & FS_DF_B1_V0;	// Get the LSB of the value 
		}
 	}
	blk->fs_rcv_buf[blk->fs_rcv_len] = c;
	blk->fs_rcv_len++;
 		
	/* See if we know the command */
	if (blk->fs_rcv_len == 2 && !blk->fs_rcmd) {

		blk->fs_rcmd = (c & FS_DF_B2_CMD_MASK) | 0x80;
		
		/* Get the expected length */
		if (blk->fs_rcmd >= 	FS_RCMD_RESET && blk->fs_rcmd <= FS_RCMD_SETBASEBRIGHT)
			blk->fs_rcmd_len = fs_rcmd_dflen[blk->fs_rcmd - FS_RCMD_RESET];
	}
 		
	/* See if we have received a complete command */

	if (blk->fs_rcmd_len && blk->fs_rcmd_len == blk->fs_rcv_len) {
		/* It's here !*/
		if (blk->fs_rcmd_len == 3)
			blk->fs_rcmd_v |= (c & FS_DF_B3_V1_7) << 1;
//		printf("fs_rcv_display() got complete command (%d) callback %p\n\r", blk->fs_rcmd, blk->fs_callback);
		fsbus_display_decode(blk);
		if (blk->fs_callback)
			(*blk->fs_callback)(blk);
	}
	
	/* Check if we are receiving a display dataframe ... and it's now complete */
	
	if (blk->fs_rcmd == FS_RCMD_DISPLAY && (c & FS_DISPLAY_END)) {
		// It's here!
//		printf("fs_rcv_display() got complete FS_RCMD_DISPLAY command (%d) callback %p\n\r", blk->fs_rcmd, blk->fs_callback);
		fsbus_display_decode(blk);
		if (blk->fs_callback)
			(*blk->fs_callback)(blk);
	}
//	printf("fs_rcv_display() exit\n\r");
}

extern fsbus_handle next_handle;
extern fsbus_block_t blocks[];

void fsbus_rcv_all(uint8_t c)
{
	uint8_t i;

	for (i = 0; i < next_handle; i++) {
		(*fs_rcv_func[blocks[i].fs_ctrl_type])(c, &blocks[i]);
	}
}

static fsbus_block_t *fs_blk = NULL;

/*
 *	Handles the initial reception of bytes and hands them off for further processing
 */
void fsbus_rcv(uint8_t c)
{

//	printf("fsbus_rcv(0x%x) enter\n\r", c);

 	if (c & FS_DF_START) {
		// This is a start of frame. We stop what we were doing and start afresh.

		fs_cid = (c & FS_DF_CID_MASK) >> 2;
		fs_blk = fs_get_blk(fs_cid);

//		printf("fsbus_rcv() got data frame start, cid = %d\n\r", fs_cid);
	}

	if (fs_cid == 0) {
		// Post to all our registered controllers
//		printf("fsbus_rcv() Posting character to all registered controllers, fs_cid = %d, fs_blk = %p\n\r", fs_cid, fs_blk);
		fsbus_rcv_all(c);
	} else {
		if (fs_blk) {
//			printf("fsbus_rcv() Posting character\n\r");

			(*fs_rcv_func[fs_blk->fs_ctrl_type])(c, fs_blk);
		} else {
//			printf("fsbus_rcv() No controller for this cid, fs_cid = %d, fs_blk = %p\n\r", fs_cid, fs_blk);
		}
	}

//	printf("fsbus_rcv() exit\n\r");
}


