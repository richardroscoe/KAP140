/*
 * This file contains the registration and general FSBUS routines
 */
#include <stdlib.h>
#include <avr/io.h>
#include <stdint.h>

#include "fsbus.h"
#include "uart.h"


#define MAX_RCV_CONTROLLERS 10

fsbus_handle next_handle;
fsbus_block_t blocks[MAX_RCV_CONTROLLERS]; 

fsbus_block_t *fsbus_register(uint8_t cid, uint8_t ctrl_type, void (*update)(fsbus_block_t *fs_blk))
{
	fsbus_handle this_handle;
	
	if (next_handle == MAX_RCV_CONTROLLERS) {
		return NULL;
	}

	this_handle = next_handle;
	next_handle++;
	
	blocks[this_handle].fs_cid = cid;
	blocks[this_handle].fs_ctrl_type = ctrl_type;
	blocks[this_handle].fs_callback = update;
	
	//printf("fsbus_register: cid %d, type %d, update 0x%p\n\r", cid, ctrl_type, update);
	return(&blocks[this_handle]);
}

void fsbus_main()
{
	while (1) {
			fsbus_rcv(uart_getc());
	}
}


void fsbus_init(void)
{
	next_handle = 0;
}

/*
 *	Find the controller with the specifed CID.
 *	Rerurn NULL if not found otherwise a pointer to the controllers block
 *
 */
fsbus_block_t *fs_get_blk(uint8_t cid)
{
	uint8_t i;

	for (i = 0; i < next_handle; i++) {
		if (cid == blocks[i].fs_cid) {
			return &blocks[i];
		}
	}
	return NULL;
}
