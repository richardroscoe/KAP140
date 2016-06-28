#ifndef FSBUS_H_
#define FSBUS_H_


/*
** These are the types of Controller
**
*/
#define FS_CTRL_DIO		0
#define FS_CTRL_DISPLAY	1


// These values define the DIO R-commands
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

// These values define the common R-commands
#define FS_RCMD_RESET 128
#define FS_RCMD_SETCID 129
#define FS_RCMD_SETBRIGHT 130
#define FS_RCMD_SETPOWER 131
#define FS_RCMD_SETDECIMALPOINT 132
#define FS_RCMD_SETBASEBRIGHT 133
#define FS_RCMD_DISPLAY 200 // Special one!

// Bit layouts for on the wire R-commands
#define FS_DF_START 		0x80		// Top bit set indicates the start of a dataframe
#define FS_DF_CID_MASK 		0x7C //b'01111100'	// Mask for CID
#define FS_DF_B1_CMD_MASK 	0x02		// Byte one command mask
#define FS_DF_B2_CMD_MASK 	0x7F		// Byte two command mask
#define FS_DF_B1_V0			0x01		// V0
#define FS_DF_B3_V1_7		0x7F		// V1 to V7


typedef struct fsbus_display {
	uint8_t fs_bright;
	uint8_t fs_power;
	uint8_t fs_decimal_point;
	uint8_t fs_base_bright;
	uint8_t fs_digits[7];
} fsbus_display_t;

typedef struct fsbus_dio {
	uint8_t	fs_dout[4];
	uint8_t fs_aout[8];
} fsbus_dio_t;

typedef struct fsbus_block_s {
	uint8_t fs_cid;							// The Controller ID
	uint8_t fs_ctrl_type;

	uint8_t fs_rcv_buf[6]; 
	uint8_t fs_rcv_len;
	uint8_t fs_rcmd;
	uint8_t fs_rcmd_len;
	uint8_t fs_rcmd_v;

	void (*fs_callback)(struct fsbus_block_s *fs_blk);	// Callback function to send incoming strings

	union {
		fsbus_display_t fs_display;
		fsbus_dio_t fs_dio;
	};
} fsbus_block_t;

typedef unsigned char fsbus_handle;


void fsbus_rcv(uint8_t c);
void fsbus_snd(uint8_t cid, uint8_t rcmd, int8_t rcmd_v, uint8_t rcmd_len);
void fsbus_init(void);
void fsbus_main(void);
fsbus_block_t *fsbus_register(uint8_t cid, uint8_t ctrl_type, void (*update)(fsbus_block_t *fs_blk));
fsbus_block_t *fs_get_blk(uint8_t cid);

void fsbus_display_decode(fsbus_block_t *fs_blk);
void fsbus_dio_decode(fsbus_block_t *fs_blk);

#endif /*FSBUS_H_*/
