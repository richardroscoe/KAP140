/*
 * This file contains the code to deal with the attached switches on the KAP140 panel.
 *
 */
#include <stdlib.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "event.h"
#include "lcd.h"
#include "uart.h"
#include "kap.h"
#include "fsbus.h"
#include "switches.h"
#include "pid.h"

#define DEBUG
#ifdef DEBUG
#include <stdio.h>
#endif


/*
 * Display positions
 */
#define DP_ROLL_MODE		0,0
#define DP_PITCH_MODE		5,0
#define DP_PITCH_TRIM		8,0
#define DP_ALERT			9,0
#define DP_RHS				10,0

#define DP_ROLL_ARM_MODE	0,1
#define DP_PITCH_ARM_MODE	5,1
//#define DP_AP_ENABLED		10,1

/*
 * These are the controller ID's
 */
#define KAP_BASE_CID		10
#define KAP_ALT_CID 		KAP_BASE_CID + 0
#define KAP_VS_CID			KAP_BASE_CID + 1
#define KAP_BARO_HPA_CID	KAP_BASE_CID + 2
#define KAP_BARO_INHG_CID	KAP_BASE_CID + 3
#define KAP_DIO_CID			KAP_BASE_CID + 4
#define KAP_AIR_ALT_CID 	KAP_BASE_CID + 5
#define KAP_AIR_VS_CID		KAP_BASE_CID + 6
#define KAP_ELEV_TRIM_CID	KAP_BASE_CID + 7

/*
 * The switch numbers for the DIO controller
 */
#define DIO_SW_APMASTER		0	// Auto Pilot On/Off
#define DIO_SW_HDG			1
#define DIO_SW_NAV			2
#define DIO_SW_APR			3
#define DIO_SW_REV			4
#define DIO_SW_ALT			5
#define DIO_SW_WINGLEVEL	6
#define DIO_SW_VSHOLD		7

#define DIO_SW_VS_UP		10
#define DIO_SW_VS_DOWN		11

#define DIO_SW_ALT_ENC_20	12	/* Encoder with 20 ft changes */
#define DIO_SW_ALT_ENC_500	14	/* Encoder with 500 ft changes */

#define DIO_SW_ELEV_TRIM	16 	/* Virtual Encoder to modify elevaor pitch for VS mode */

#define DIO_SW_BARO_HPA		17
#define DIO_SW_BARO_INHG	19

/*
 *
 * 8. ALTITUDE HOLD/VERTICAL SPEED DISPLAY
 *		In VS hold, displays the commanded vertical speed. In altitude hold,
 *		displays the selected reference altitude.
 *
 * 9. PITCH MODE DISPLAY
 *		Displays the active and armed pitch modes (VS, ALT, ARM and GS).
 *
 * 10. ROLL MODE DISPLAY
 *		Displays the active and armed roll modes (ROL, HDG, NAV ARM, NAV,
 *		APR ARM, APR, REV ARM, REV, GS ARM).
 *
 * 11. AUTOPILOT ENGAGED (AP)
 *		ANNUNCIATION - Illuminates whenever the autopilot is engaged.
 */

/*
 * We need to manage the FSX AP button states so that we can map our state to the FSX one
 */
#define FSX_AP		_BV(DIO_SW_APMASTER)
#define FSX_HDG		_BV(DIO_SW_HDG)
#define FSX_NAV		_BV(DIO_SW_NAV)
#define FSX_APR		_BV(DIO_SW_APR)
#define FSX_REV		_BV(DIO_SW_REV)
#define FSX_ALT 	_BV(DIO_SW_ALT)
#define FSX_WGL		_BV(DIO_SW_WINGLEVEL)
#define FSX_VS		_BV(DIO_SW_VSHOLD)
#define FSX_BITS	DIO_SW_WINGLEVEL + 1

static volatile uint8_t fsx_buttons; /* If a bit as defined above is set, then that button needs to be down on FSX */

/*
 * Used to index into require/disallow tables for mappings
 */
#define FSX_REQUIRE 0
#define FSX_DISALLOW 1

// AP Mode
#define AP_DISABLED		0x00
#define AP_ENABLED		0x01
#define AP_TRANSITION	0x40	/* State is transitioning */
#define AP_CHANGED		0x80	/* State has changed */

#define AP_MODE			~(AP_TRANSITION | AP_CHANGED)

static volatile uint8_t ap_mode;

// Roll mode

#define RM_ROL		0x00
#define RM_HDG		0x01
#define RM_NAV		0x02
#define RM_APR		0x03
#define RM_REV		0x04
#define RM_CLR		0x05 // The string to display to clear any text
#define RM_CHANGED	0x80


static volatile uint8_t roll_mode;
static volatile uint8_t roll_arm_mode;

static const char roll_mode_txt[6][4] PROGMEM = { "ROL", "HDG", "NAV", "APR", "REV", "   " };

static const uint8_t roll_mode_fsx[5][2] = { 
												{ /* ROL */ FSX_WGL, FSX_HDG | FSX_NAV | FSX_APR | FSX_REV },
												{ /* HDG */ FSX_HDG, FSX_NAV | FSX_APR | FSX_REV | FSX_WGL },
												{ /* NAV */ FSX_NAV, FSX_APR | FSX_REV | FSX_WGL },
												{ /* APR */ FSX_APR, FSX_REV | FSX_NAV | FSX_ALT | FSX_WGL },
												{ /* REV */ FSX_REV, FSX_NAV | FSX_WGL },
										 };

// Pitch mode
#define PM_ALT 	0x00
#define PM_VS	0x01
#define PM_GS	0x02
#define PM_CLR	0x03 // The string to display to clear any text
#define PM_CHANGED 0x80

static volatile uint8_t pitch_mode;
static volatile uint8_t pitch_arm_mode;

static const char pitch_mode_txt[4][4] PROGMEM = { "ALT", " VS", " GS", "   " };

static const uint8_t pitch_mode_fsx[3][2] = { 
												{ /* ALT */ FSX_ALT,	FSX_VS },
												{ /* VS  */ FSX_VS,		FSX_ALT },
												{ /* GS  */ 0,			FSX_ALT | FSX_VS },
											};

// Barometer mode
#define BARO_HPA	0x00
#define BARO_INHG	0x01

static volatile uint8_t	baro_mode;

/*
 * RHS Display mode
 *
 * The right hand side of the display is used to display the pitch mode or the barometer
 * information. This variable indicates the current usage.
 * 
 * The top bit indicates that the usage has recently changed, and that the display needs
 * updating.
 */
#define RHS_ALT		0x00
#define RHS_VS		0x01
#define RHS_BARO	0x02
#define RHS_CHANGED	0x80

static volatile uint8_t rhs_mode;

/*
 * Digits Changed Flags
 *
 * When set, they indicate that the LCD needs updating or that the fsblk digits have changed
 */
#define KAP_DC_ALT			0x01
#define KAP_DC_VS			0x02
#define KAP_DC_BARO_HPA		0x04
#define KAP_DC_BARO_INHG	0x08
#define KAP_DC_AIR_ALT		0x10

static volatile uint8_t kap_disp_flags = 0;

#define KAP_DIO_CHANGED 0x80

static volatile uint8_t kap_dio_flags = 0;

fsbus_block_t 	*kap_alt_fs_blk,
				*kap_vs_fs_blk,
				*kap_baro_hpa_fs_blk,
				*kap_baro_inhg_fs_blk,
				*kap_air_alt_fs_blk,
				*kap_dio_blk,
				*kap_elev_trim_blk,
				*kap_air_vs_blk;

/*
 * The actual VS in feet (-9900 to 99900)
 */
static volatile int16_t vs = 0,			/* This is the value displayed by us */
						elev_trim = 0,	/* This is the value received from FSBUS */
						air_vs = 0;		/* This is the aircrafts vertical speed */
static volatile int32_t	alt_disp = 0,		/* This is the value displayed by us */
						alt_rcv = 0,	/* This is the value received from FSBUS */
						air_alt = 0;	/* This is the aircrafts actual altitude */
static volatile int16_t	baro_disp_hpa = 0,
						baro_disp_inhg = 0,
						baro_hpa = 0,
						baro_inhg = 0;

#define ALT_INCR_SLOW 100 // 20 ft - (now 100ft to match FSX) changes when altering the Altitude
#define ALT_INCR_FAST 500




/*
 * User defined characters
 */
#define UDCS_F 		0
#define UDCS_T		1
#define UDCS_P		2
#define UDCS_M		3
//#define UDCS_A_BR	4
//#define UDCS_P_BR	5
#define UDCS_ARM	4
#define UDCS_PT_UP	5
#define UDCS_PT_DN	6

static const PROGMEM unsigned char kap_udcs[] =
{
//FT
	0x07, 0x04, 0x6, 0x4, 0x4, 0x0, 0x0, 0x0,
	0x1C, 0x8, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00,

//FPM
	// Use F from above
	0xe, 0xa, 0xe, 0x8, 0x8,0,0,0, //P
	0x14, 0x1C, 0x14, 0x14, 0x14,0,0,0, //M

// AP
//	0xf, 0x10, 0x12, 0x15, 0x17, 0x15, 0x10, 0xf,
//	0x1e, 0x1, 0x1d, 0x15, 0x1d, 0x11, 0x1, 0x1e,

// ARM
	0x4, 0xa, 0xe, 0xa, 0, 0xa, 0xe, 0xa,

// Pitch Trim
	0x4, 0xe, 0x1f, 0, 0xc,0xa,0xc,0x8,		// Up
	0xc, 0xa, 0xc, 0x8, 0, 0x1f, 0xe, 0x4,	// Down

};


/*
 * Pitch trim
 */
#define PT_NONE	0x00
#define PT_UP	0x01
#define PT_DOWN 0x02

static volatile uint8_t pitch_trim;

#define PT_RND	20 /* Round the delta to the nearest PT_RND feet */

static volatile event_handle kap_pt_alert = 0;

static const char pt_txt[3] = { ' ', UDCS_PT_UP, UDCS_PT_DN };

/*
 * Altitude alert
 */
#define ALT_AT			0x01	// At requested altitude
#define ALT_200_1000	0x02

#define ALT_REACHED	0x40

static volatile uint8_t alt_alert = 0;




/*
 * The event to cancel RHS baro display
 */
static volatile event_handle kap_baro_cancel;

/*
 * The event to cancel baro mode check
 */
static volatile event_handle baro_mode_check_cancel;

/*
 * The event to cancel RHS Vertical Speed display
 */
static volatile event_handle kap_vs_cancel;

/*
 * The event to cancel Vertical Speed PID control
 */
static volatile event_handle kap_vs_pid;

/*
 * The event to cancel blinking of the current roll mode
 */
static volatile event_handle kap_rm_blink_cancel;

/*
 * The event to cancel blinking of the AP during disabling
 */
static volatile event_handle kap_ap_blink_cancel;

/*
 * The event control the ALERT flashing
 */
static volatile event_handle kap_alert;

/*
 * Encoder toggle status
 */
static volatile uint8_t kap_enc_toggle_status; /* Toggle status */

/*
 * The status of the AP button needs to be maintained so that both button
 * down and UP can be detected
 */
static volatile uint8_t kap_ap_button = 0; // 0 up, 1 down;

/*
 * Protos
 */
static void kap_display();
static void kap_ap_disable();
static void kap_end_baro();
//static void kap_vs_pid_event(void);
//static void kap_vs_pid_enable(void);
//static void kap_vs_pid_disable(void);

#define RM_BLINK_ON 2	// Tick number to turn text ON
#define RM_BLINK_OUT_OF 8 // Number of ticks before we wrap and turn text off

/*
 * This rountine is used to blink the current roll mode
 */

static void kap_roll_mode_blink()
{
	static uint8_t count = 0;

	count++;
	if (count == RM_BLINK_OUT_OF) {
		count = 0;
		lcd_gotoxy(DP_ROLL_MODE);
		lcd_puts_p(roll_mode_txt[RM_CLR]);
	}

	if (count == RM_BLINK_ON) {
		lcd_gotoxy(DP_ROLL_MODE);
		lcd_puts_p(roll_mode_txt[roll_mode & ~RM_CHANGED]);
	}
}

#define AP_BLINK_ON 6	// Tick number to turn text ON
#define AP_BLINK_OUT_OF 8 // Number of ticks before we wrap and turn text off

/*
 * This rountine is used to blink AP as it is being disabled
 */
static void kap_ap_off_blink()
{
	static uint8_t count = 0;

	count++;
	if (count == AP_BLINK_OUT_OF) {
		count = 0;
		lcd_gotoxy(0,0);
		lcd_puts("AP");
		//lcd_putc(UDCS_A_BR);
		//lcd_putc(UDCS_P_BR);
	}

	if (count == AP_BLINK_ON) {
		lcd_gotoxy(0,0);
		lcd_puts("  ");
	}
}

/*
 * This routine blinks the pitch trim
 */
#define PT_BLINK_ON 6/ Tick number to turn text ON
#define PT_BLINK_OUT_OF 8 // Number of ticks before we wrap and turn text off

static void kap_pt_display()
{
	static uint8_t count = 0;

	if (pitch_trim == PT_NONE) {
		lcd_gotoxy(DP_PITCH_TRIM);
		lcd_putc(' ');

		event_cancel(&kap_pt_alert);
		return;
	}

	count++;
	if (count == AP_BLINK_OUT_OF) {
		count = 0;
		lcd_gotoxy(DP_PITCH_TRIM);
		lcd_putc(pt_txt[pitch_trim]);
	}

	if (count == AP_BLINK_ON) {
		lcd_gotoxy(DP_PITCH_TRIM);
		lcd_putc(' ');
	}
}

/*
 * This routine blinks the pitch trim
 */
#define AL_BLINK_ON 6 // Tick number to turn text ON
#define AL_BLINK_OUT_OF 8 // Number of ticks before we wrap and turn text off

static void kap_alert_flash()
{
	static uint8_t count = 0;

	count++;
	if (count == AL_BLINK_OUT_OF) {
		count = 0;
		lcd_gotoxy(DP_ALERT);
		lcd_putc('A');
	}

	if (count == AL_BLINK_ON) {
		lcd_gotoxy(DP_ALERT);
		lcd_putc(' ');
	}
}

int32_t my_atol(signed char *buff)
{
	int32_t r = 0;
	uint8_t minus = 0;
	char c;

	while ((c = *buff)) {
		if (isdigit(c)) {
			r *= 10;
			r += c - '0';
		} else if (c == '-')
			minus = 1;
		buff++;
	}
	if (minus)
		r *= -1;
	return r;
}

int16_t my_atoi(signed char *buff)
{
	int16_t r = 0;
	uint8_t minus = 0;
	char c;

	while ((c = *buff)) {
		if (isdigit(c)) {
			r *= 10;
			r += c - '0';
		} else if (c == '-')
			minus = 1;
		buff++;
	}
	if (minus)
		r *= -1;
	return r;
}

/******************************* FSBUS CALLBACK ROUTINES ************************************/

/*
 * This is the callback routine for the Altitude Controller
 */
static void kap_rcv_alt()
{
//	printf("kap_rcv_alt - enter\n\r");
	kap_disp_flags |= KAP_DC_ALT;
	alt_rcv = my_atol((signed char *)kap_alt_fs_blk->fs_display.fs_digits);
//	alt_disp = alt_rcv;
//	printf("kap_rcv_alt - exit\n\r");
}

/*
 * This is the callback routine for the Aircraft Altitude Controller (as opposed to the AP alt)
 */
static void kap_rcv_air_alt()
{
//	printf("kap_rcv_air_alt - enter\n\r");
	kap_disp_flags |= KAP_DC_AIR_ALT;
	air_alt = my_atol((signed char *)kap_air_alt_fs_blk->fs_display.fs_digits);
//	printf("kap_rcv_air_alt - %d exit\n\r", air_alt);
}

/*
 * This is the callback routine for the VS Controller
 */
static void kap_rcv_vs()
{
//	printf("kap_rcv_vs - enter\n\r");
	kap_disp_flags |= KAP_DC_VS;
	vs = my_atoi((signed char *)kap_vs_fs_blk->fs_display.fs_digits);
//	printf("kap_rcv_vs - exit\n\r");
}

/*
 * This is the callback routine for the HPA Baro Controller
 */
static void kap_rcv_baro_hpa()
{
//	printf("kap_rcv_baro_hpa - enter\n\r");
	kap_disp_flags |= KAP_DC_BARO_HPA;
	baro_hpa = my_atoi((signed char *)kap_baro_hpa_fs_blk->fs_display.fs_digits);
//	printf("kap_rcv_baro_hpa - exit\n\r");
}

/*
 * This is the callback routine for the inhg Baro Controller
 */
static void kap_rcv_baro_inhg()
{
//	printf("kap_rcv_baro_inhg - enter\n\r");
	kap_disp_flags |= KAP_DC_BARO_INHG;
	baro_inhg = my_atoi((signed char *)kap_baro_inhg_fs_blk->fs_display.fs_digits);
//	printf("kap_rcv_baro_inhg - exit\n\r");
}


/*
 * This is the callback routine for the inhg Baro Controller
 */
static void kap_rcv_dio()
{
	printf("kap_rcv_dio - enter\n\r");
//	kap_dio_flags |= KAP_DIO_CHANGED;

	printf("kap_rcv_dio: DIO Port A: 0x%x\n\r", kap_dio_blk->fs_dio.fs_dout[0]);

	// Only deal with the AP being turned off
	if (!(kap_dio_blk->fs_dio.fs_dout[0] & FSX_AP)) {
		printf("kap_rcv_dio - AP NOW OFF!!!!\n\r");
		kap_ap_disable();
	}

	printf("kap_rcv_dio - exit\n\r");
}

/*
 * This is the callback routine for the the Elevator Trim Controller
 */
static void kap_rcv_elev_trim()
{
//	printf("kap_rcv_elev_trim - enter\n\r");

	elev_trim = my_atoi((signed char *)kap_elev_trim_blk->fs_display.fs_digits);

//	printf("kap_rcv_elev_trim - exit\n\r");
}

/*
 * This is the callback routine for the Aircrafts Vertical Speed Controller
 */
static void kap_rcv_air_vs()
{
//	printf("kap_rcv_air_vs - enter\n\r");
	
	air_vs = my_atoi((signed char *)kap_air_vs_blk->fs_display.fs_digits);

//	printf("kap_rcv_air_vs - exit\n\r");
}

/******************************* BUTTON ROUTINES ************************************/
/*
 * KAP Button routines
 */

/*
 * 7. VERTICAL TRIM (UP/DN) BUTTONS
 *		- The action of these buttons is dependent upon the vertical mode
 *		present when pressed. If VS mode is active, immediate button strokes
 *		will increment the vertical speed commanded either up or down at the
 *		rate of 100 ft/min per button press. If ALT mode is active, incremental
 *		button strokes will move the altitude hold reference altitude either
 *		up or down at 20 feet per press.
 */
static void kap_button_up()
{
	printf("kap_button_up()\n\r");
	if ((pitch_mode & ~PM_CHANGED) != PM_GS) {

		if ((rhs_mode & ~RHS_CHANGED) == RHS_VS) {
			event_reset(kap_vs_cancel); // Don't go back to the Altitude display yet	
			fsbus_snd(KAP_DIO_CID, DIO_SW_VS_UP, 1, 3);

			/* FlightSim doesn't update the display until we have
			 * finished pressing buttons, which means our display
			 * doesn't reflect the correct value after a key press
			 * so we need to improvise
			 */
			vs += 100;
			kap_disp_flags |= KAP_DC_VS;
		} else {
			rhs_mode = RHS_VS | RHS_CHANGED;
		}
	}	
}

static void kap_button_down()
{
	printf("kap_button_down()\n\r");
	if ((pitch_mode & ~PM_CHANGED) != PM_GS) {

		if ((rhs_mode & ~RHS_CHANGED) == RHS_VS) {
			event_reset(kap_vs_cancel); // Don't go back to the Altitude display yet
			fsbus_snd(KAP_DIO_CID, DIO_SW_VS_DOWN, 1, 3);

			/* FlightSim doesn't update the display until we have
			 * finished pressing buttons, which means our display
			 * doesn't reflect the correct value after a key press
			 * so we need to improvise
			 */
			vs -= 100;
			kap_disp_flags |= KAP_DC_VS;
		} else {
			rhs_mode = RHS_VS | RHS_CHANGED;
		}	
	}
}


static void kap_button_encoder_toggle()
{
	printf("kap_button_encoder_toggle()\n\r");

	if ((rhs_mode & ~RHS_CHANGED) != RHS_BARO) {

		if (kap_enc_toggle_status != 0)
			kap_enc_toggle_status = 0;
		else
			kap_enc_toggle_status = 1;
	}
}


static void baro_mode_check()
{
	printf("baro_mode_check() - enter\n\r");

	if (sw_porta_state & _BV(1)) {
		// Still pressed

		if (baro_mode == BARO_HPA)
			baro_mode = BARO_INHG;
		else
			baro_mode = BARO_HPA;

		// Reset the switch back to VS/ALT
		event_reset(kap_baro_cancel);

		rhs_mode |= RHS_CHANGED;
	}

	baro_mode_check_cancel = 0;

	printf("baro_mode_check() - exit\n\r");
}


/*
 * The BARO button. When pressed, switches to barometer mode. If held down
 * for 2 seconds, it toggles between HPa and In Hg modes
 */
static void kap_button_baro()
{
	printf("kap_button_baro()\n\r");

	if ((rhs_mode & ~RHS_CHANGED) != RHS_BARO) {
		rhs_mode = RHS_CHANGED | RHS_BARO;

		baro_mode_check_cancel = event_register(baro_mode_check, EVENT_HZ * 2, 1);
	}
}

/*
 * Arm altitude mode toggle
 */
static void kap_button_arm()
{
	printf("kap_button_arm()\n\r");

	if ((pitch_arm_mode & ~PM_CHANGED) == PM_CLR)
		pitch_arm_mode = PM_ALT | PM_CHANGED;
	else {
		pitch_arm_mode = PM_CLR | PM_CHANGED;
		alt_alert = 0;	// Ensure we aren't assuming we are now at the specified altitude
	}

}

/*
 * 1. AUTOPILOT ENGAGE/DISENGAGE
 *		(AP) BUTTON - When pushed, engages autopilot if all logic conditions
 *		are met. The autopilot will engage in the basic roll (ROL) mode which
 *		functions as a wing leveler. When pressed again, will disengage the
 *		autopilot. The AP button must be pressed and held for 0.25 seconds
 *		to engage the autopilot.
 */
static void kap_ap_on()
{
	printf("kap_ap_on() - enter\n\r");

	if (sw_porta_state & _BV(5)) {
		// Still pressed

		if ((ap_mode & AP_MODE) == AP_DISABLED) {
			ap_mode = AP_ENABLED | AP_CHANGED;			
		}
	}
	ap_mode &= ~AP_TRANSITION;

	printf("kap_ap_on() - exit\n\r");
}



static void kap_button_ap()
{
	printf("kap_button_ap()\n\r");

	// Ignore key presses whilst transitioning
	if (ap_mode & AP_TRANSITION)
		return;

	if ((ap_mode & AP_MODE) == AP_DISABLED) {
		// Check button still pressed 0.25 seconds from now,
		ap_mode |= AP_TRANSITION;
		event_register(kap_ap_on, EVENT_HZ / 4, 1);
	} else {
		ap_mode = AP_DISABLED | AP_CHANGED | AP_TRANSITION; // Not fully off until blinking done
	}
}

/*
 * 2. HEADING (HDG) MODE SELECTOR
 *		BUTTON - When pushed, will arm the Heading mode, which commands the
 *		airplane to turn to and maintain the heading selected by the heading
 *		bug on either the DG or HSI. A new heading may be selected at any
 *		time and will result in the airplane turning to the new heading.
 *		Button can also be used to toggle between HDG and ROL modes.
 */
static void kap_button_hdg()
{
	printf("kap_button_hdg()\n\r");

	if ((roll_mode & ~RM_CHANGED) == RM_ROL) {
		roll_mode = RM_HDG | RM_CHANGED;
	} else if ((roll_mode & ~RM_CHANGED) == RM_HDG) {
		roll_mode = RM_ROL | RM_CHANGED;
	} else {
		if ((roll_mode & ~RM_CHANGED) == RM_APR) {
			pitch_mode = PM_VS | PM_CHANGED; // We were on GS
		}
		roll_mode = RM_HDG | RM_CHANGED;
	}
}

/*
 * 3. NAVIGATION (NAV) MODE
 *		SELECTOR BUTTON - When pushed, will arm the navigation mode. The mode
 *		provides automatic beam capture and tracking of VOR, LOC or GPS as
 *		selected for presentation on the HSI or CDI. NAV mode is recommended
 *		for enroute navigation tracking. NAV mode may also be used for front
 *		course LOC tracking when GS tracking is not desired.
 */
static void kap_button_nav()
{
	printf("kap_button_nav()\n\r");

	/* We can switch from ROL mode, but not other modes */

	if ((roll_mode & ~RM_CHANGED) == RM_ROL)
		roll_mode = RM_HDG | RM_CHANGED;
	
	if ((roll_mode & ~RM_CHANGED) == RM_HDG && roll_arm_mode == RM_CLR) {
		roll_arm_mode = RM_NAV | RM_CHANGED;
	}
}

/*
 * 4. APPROACH (APR) MODE
 *		SELECTOR BUTTON - When pushed, will arm the Approach mode. This mode
 *		provides automatic beam capture and tracking of VOR, GPS, LOC, and
 *		Glideslope (GS) on an ILS, as selected for presentation on the HSI or
 *		CDI. APR mode is recommended for instrument approaches.
 */
static void kap_button_apr()
{
	printf("kap_button_apr()\n\r");
	/* We can switch from ROL mode, but not other modes */

	if ((roll_mode & ~RM_CHANGED) == RM_ROL)
		roll_mode = RM_HDG | RM_CHANGED;
	
	if ((roll_mode & ~RM_CHANGED) == RM_HDG && roll_arm_mode == RM_CLR) {
		roll_arm_mode = RM_APR | RM_CHANGED;
	}
}

/*
 * The encoder is used to change the altitude
 */


static void kap_button_enc(int8_t delta)
{
	printf("kap_button_enc(%d)\n\r", delta);


	if ((rhs_mode & ~RHS_CHANGED) != RHS_BARO) {

		/* If the alt arm mode isn't enabled, enable it */

		if ((pitch_arm_mode & ~PM_CHANGED) != PM_ALT) {
			pitch_arm_mode = PM_ALT | PM_CHANGED;
			rhs_mode = RHS_ALT | RHS_CHANGED;
		}

		if (kap_enc_toggle_status == 0)
			alt_disp += delta * ALT_INCR_SLOW;
		else
			alt_disp += delta * ALT_INCR_FAST;

		kap_disp_flags |= KAP_DC_ALT;

	} else {

		event_reset(kap_baro_cancel);

		if (baro_mode == BARO_HPA)
			fsbus_snd(KAP_DIO_CID, DIO_SW_BARO_HPA, delta, 3);
		else
			fsbus_snd(KAP_DIO_CID, DIO_SW_BARO_INHG, delta, 3);

	}
}


/*
 * 6. ALTITUDE HOLD (ALT) MODE
 *		SELECT BUTTON - When pushed, will select the Altitude Hold mode. This
 *		mode provides tracking of the selected altitude.
 */
static void kap_button_alt()
{
	printf("kap_button_alt()\n\r");

	// Toggle the pitch mode

	if ((pitch_mode & ~PM_CHANGED) == PM_VS) {
		pitch_mode = PM_ALT | PM_CHANGED;			

// Do we manage the button state?
//		fsbus_snd(KAP_DIO_CID, DIO_SW_ALT, 1, 3);


	} else {
		pitch_mode = PM_VS | PM_CHANGED;

		// Cause the RHS to display the VS for a while
		rhs_mode = RHS_VS | RHS_CHANGED;

//		fsbus_snd(KAP_DIO_CID, DIO_SW_ALT, 0, 3);
	}
}

/*
 * 5. BACK COURSE APPROACH
 *		(REV) MODE SELECTOR BUTTON - When pushed, will arm the Back Course
 *		approach mode. This mode functions similarly to the approach mode
 *		except that the autopilot response to LOC signals is reversed, and
 *		GS is disabled.
 (*/
static void kap_button_rev()
{
	printf("kap_button_rev()\n\r");
	/* We can switch from ROL mode, but not other modes */

	if ((roll_mode & ~RM_CHANGED) == RM_ROL)
		roll_mode = RM_HDG | RM_CHANGED;
	
	if ((roll_mode & ~RM_CHANGED) == RM_HDG && roll_arm_mode == RM_CLR) {
		roll_arm_mode = RM_REV | RM_CHANGED;
	}
}

/*
		PA0	Enc D
		  1 Baro
		  2 Up
		  3 Arm
		  4 Apr
		  5 AP
		  6 Nav
		  7 Hdg

		PB0 Enc A
		  1 Enc B
		  2 Down
		  3 Rev
		  4 Alt
		  5
		  6
		  7
 */
static void kap_buttons()
{
	int8_t delta;

	//printf("kap_buttons: porta 0x%x, portb 0x%x\n\r", sw_porta, sw_portb);


	/* Port A keys */

	if (sw_porta & _BV(5)) {
		sw_porta ^= _BV(5);

		/* Only act on the button if it wasn't already pressed */
		if (!kap_ap_button) {
			kap_ap_button = 1;
			kap_button_ap();
		} 		
	} else if ((sw_porta_state & _BV(5)) == 0)
		kap_ap_button = 0;

	// Only deal with the other buttons if AP is enabled

	if ((ap_mode & ~AP_CHANGED) == AP_ENABLED) { 

		if (sw_porta & _BV(0)) {
			sw_porta ^= _BV(0);
			kap_button_encoder_toggle();
		}
		if (sw_porta & _BV(1)) {
			sw_porta ^= _BV(1);
			kap_button_baro();
		}
		if (sw_porta & _BV(2)) {
			sw_porta ^= _BV(2);
			kap_button_up();
		}
		if (sw_porta & _BV(3)) {
			sw_porta ^= _BV(3);
			kap_button_arm();
		}

		if (sw_porta & _BV(4)) {
			sw_porta ^= _BV(4);
			kap_button_apr();
		}
		if (sw_porta & _BV(6)) {
			sw_porta ^= _BV(6);
			kap_button_nav();
		}
		if (sw_porta & _BV(7)) {
			sw_porta ^= _BV(7);
			kap_button_hdg();
		}
		
//printf("kap_buttons: enc now %d\n\r", sw_enc_delta);

		if (sw_enc_delta != 0) {
			delta = sw_enc_delta;
			sw_enc_delta = 0;
			kap_button_enc(delta);
		}
		if (sw_portb & _BV(2)) {
			sw_portb ^= _BV(2);
			kap_button_down();
		}
		if (sw_portb & _BV(3)) {
			sw_portb ^= _BV(3);
			kap_button_rev();
		}
		if (sw_portb & _BV(4)) {
			sw_portb ^= _BV(4);
			kap_button_alt();
		}
	} else {
		// AP not enabled, clear the events
		sw_porta = 0;
		sw_portb = 0;
	}
}

/******************************* DISPLAY ROUTINES ************************************/

/*
 * This function is called to revert the RHS to displaying
 * the altitude after having displayed the vertical speed
 */
static void kap_vs_end()
{
	kap_vs_cancel = 0;

	printf("kap_vs_end() - rhs_mode =0x%x\n\r", rhs_mode);
	if ((rhs_mode & ~RHS_CHANGED) == RHS_VS) {
		printf("kap_vs_end() - vs mode ending - back to ALT\n\r");

		rhs_mode = RHS_ALT | RHS_CHANGED;
	}
	
	printf("kap_vs_end() - exit\n\r");
}


/*
 * Output an integer value into a character buffer
 *
 * Right justified, use a comma to seperate 1000s.
 * Cope with negative values
 * Display width is 6 characters
 */
#define BUF_LEN 7

static void displ_val(char *buf, int32_t v)
{
	int8_t i = BUF_LEN - 2;
	uint8_t minus = (v < 0);
	char c;
	
	if (minus)
		v *= -1;

	*(buf+ BUF_LEN - 1) = 0;

	do {
//printf("displ_val: v = %d, mod 10 = %d, div 10 = %d\n\r", v, (v %10), v / 10);

		c = (v % 10) + '0';
		v = v / 10;

		if (i == 2) {
			*(buf + i) = ',';
			i--;
		}

		*(buf + i) = c;
		i--;

	} while (v != 0 && i >= 0);

	if (minus && i >= 0) {
		*(buf + i) = '-';
		i--;
	}

	while (i >= 0) {
		*(buf + i) = ' '; 
		i--;
	}

	//printf("buffer = '%s'\n\r", buf);
}


static void inline kap_displ_vs()
{
	char out_buf[BUF_LEN];

//	printf("kap_displ_vs - enter\n\r");

	if (rhs_mode & RHS_CHANGED) {
		rhs_mode ^= RHS_CHANGED;

		kap_disp_flags |= KAP_DC_VS; // Ensure the digits are displayed

		// Display the FPM
		lcd_gotoxy(13, 1);
		//lcd_puts("FPM");
		lcd_putc(UDCS_F);
		lcd_putc(UDCS_P);
		lcd_putc(UDCS_M);

		printf("kap_displ_vs - setup for kap_vs_end - 3 seconds\n\r");

		// Revert after 3 seconds
		kap_vs_cancel = event_register(kap_vs_end, 3 * EVENT_HZ, 1);

		printf("kap_displ_vs - event handle = %d\n\r", kap_vs_cancel);

	}

	if (kap_disp_flags & KAP_DC_VS) {

		lcd_gotoxy(DP_RHS);

		printf("kap_displ_vs: vs = %d\n\r", vs);

		displ_val(out_buf, vs);

		lcd_puts(out_buf);

		kap_disp_flags ^= KAP_DC_VS;
	}

//	printf("kap_displ_vs - exit\n\r");
}

static void inline kap_displ_alt()
{
	char out_buf[BUF_LEN];

//	printf("kap_displ_alt - enter\n\r");
	if (rhs_mode & RHS_CHANGED) {
		rhs_mode ^= RHS_CHANGED;

		kap_disp_flags |= KAP_DC_ALT; // Ensure the digits are displayed

		// Display the FPM
		lcd_gotoxy(13, 1);
		lcd_putc(' ');
		lcd_putc(UDCS_F);
		lcd_putc(UDCS_T);
		//lcd_puts(" FT");

	}

	if (kap_disp_flags & KAP_DC_ALT) {

		/* Update the displayed value with the received value if we aren't in ALT ARM */

		if ((pitch_arm_mode & ~PM_CHANGED) == PM_CLR)
			alt_disp = alt_rcv;

		lcd_gotoxy(DP_RHS);

		displ_val(out_buf, alt_disp);

		lcd_puts(out_buf);

		kap_disp_flags ^= KAP_DC_ALT;
	}

//	printf("kap_displ_alt - exit\n\r");
}


static void inline kap_displ_baro_hpa()
{
	//printf("kap_displ_baro_hpa - enter\n\r");

	if (kap_disp_flags & KAP_DC_BARO_HPA) {
		lcd_gotoxy(DP_RHS);
		lcd_puts((const char *)kap_baro_hpa_fs_blk->fs_display.fs_digits);

		kap_disp_flags ^= KAP_DC_BARO_HPA;
	}

	//printf("kap_displ_baro_hpa - exit\n\r");
}


static void inline kap_displ_baro_inhg()
{
	//printf("kap_displ_baro_inhg - enter\n\r");

	if (kap_disp_flags & KAP_DC_BARO_INHG) {
		lcd_gotoxy(DP_RHS);
		lcd_puts((const char *)kap_baro_inhg_fs_blk->fs_display.fs_digits);

		kap_disp_flags ^= KAP_DC_BARO_INHG;
	}

	//printf("kap_displ_baro_inhg - exit\n\r");
}

/*
 * Display the roll mode
 */
static void kap_display_roll()
{
	uint8_t m, i;

	if (roll_mode & RM_CHANGED) {
		roll_mode ^= RM_CHANGED;

		lcd_gotoxy(DP_ROLL_MODE);
		lcd_puts_p(roll_mode_txt[roll_mode & ~RM_CHANGED]);

		// Now we need to send an IO to FSBUS to let flight sim know which mode we want

		// First check the Required buttons

		m = roll_mode_fsx[roll_mode & ~RM_CHANGED][FSX_REQUIRE];

		m = ~((m & fsx_buttons) | ~m);

		if (m) {
			for (i = 1; i < FSX_BITS; i++ ) {
				if (m & _BV(i)) {
					// Turn this bit on
					fsbus_snd(KAP_DIO_CID, i, 1, 3);
					fsx_buttons ^= _BV(i);
				}
			}
		}

		// Then check the Disallowed buttons

		m = roll_mode_fsx[roll_mode & ~RM_CHANGED][FSX_DISALLOW];

		m = m & fsx_buttons;

		if (m) {
			for (i = 1; i < FSX_BITS; i++ ) {
				if (m & _BV(i)) {
					// Turn this bit off
					fsbus_snd(KAP_DIO_CID, i, 0, 3);
					fsx_buttons ^= _BV(i);
				}
			}
		}
	}
}


/*
 * When ready, commits the current roll_arm_mode to the roll_mode
 */
static void kap_roll_arm_commit()
{
//printf("kap_roll_arm_commit: enter, roll_mode = 0x%x, roll_arm_mode = 0x%x\n\r", roll_mode, roll_arm_mode);

	// Cancel HDG blink

	if (kap_rm_blink_cancel) {
		event_cancel(&kap_rm_blink_cancel);
	}

	roll_mode = roll_arm_mode | RM_CHANGED;
	roll_arm_mode = RM_CLR | RM_CHANGED;

	if ((roll_mode & ~RM_CHANGED) == RM_APR) {
		// Change the pitch mode to Glide slope ... we follow the glide slope in
		pitch_mode = PM_GS | PM_CHANGED;
	}
}

/*
 * Display the roll arm mode
 */
static void kap_display_roll_arm()
{
	if (roll_arm_mode & RM_CHANGED) {
		roll_arm_mode ^= RM_CHANGED;

//printf("kap_display_roll_arm: roll_arm_mode changed, now 0x%x\n\r", roll_arm_mode);

		lcd_gotoxy(DP_ROLL_ARM_MODE);
		lcd_puts_p(roll_mode_txt[roll_arm_mode & ~RM_CHANGED]);

		if ((roll_arm_mode & ~RM_CHANGED) == RM_CLR) {
			lcd_putc(' '); // Clear the ARM ... do whatever to clear the special chars
		} else {
			lcd_putc(UDCS_ARM);	// Need special character defining

//printf("kap_display_roll_arm: Setup blinking and commit callback\n\r");

			kap_rm_blink_cancel = event_register(kap_roll_mode_blink, EVENT_HZ / 5, 0);

			event_register(kap_roll_arm_commit, 5 * EVENT_HZ, 1);

		}
	}
}


/*
 * Display the pitch mode
 */
static void kap_display_pitch()
{
	uint8_t m, i;
	
	if (pitch_mode & PM_CHANGED) {
		pitch_mode ^= PM_CHANGED;

		lcd_gotoxy(DP_PITCH_MODE);
		lcd_puts_p(pitch_mode_txt[pitch_mode & ~PM_CHANGED]);
/*
		if ((pitch_mode & ~PM_CHANGED) == PM_VS) {
			kap_vs_pid_enable();
		} else {
			kap_vs_pid_disable();
		}
*/
		
		// Now we need to send an IO to FSBUS to let flight sim know which mode we want
		// First check the Required buttons

		m = pitch_mode_fsx[pitch_mode & ~PM_CHANGED][FSX_REQUIRE];

//printf("kap_display_pitch: Require 0x%x, got 0x%x,", m, fsx_buttons);

		m = ~((m & fsx_buttons) | ~m);

//printf("todo 0x%x\n\r", m);

		if (m) {
			for (i = 1; i < FSX_BITS; i++ ) {
				if (m & _BV(i)) {
					// Turn this bit on
					fsbus_snd(KAP_DIO_CID, i, 1, 3);
					fsx_buttons ^= _BV(i);
				}
			}
		}

		// Then check the Disallowed buttons

		m = pitch_mode_fsx[pitch_mode & ~PM_CHANGED][FSX_DISALLOW];

//printf("kap_display_pitch: Disallow 0x%x, got 0x%x,", m, fsx_buttons);

		m = m & fsx_buttons;

//printf("todo 0x%x\n\r", m);

		if (m) {
			for (i = 1; i < FSX_BITS; i++ ) {
				if (m & _BV(i)) {
					// Turn this bit off
					fsbus_snd(KAP_DIO_CID, i, 0, 3);
					fsx_buttons ^= _BV(i);
				}
			}
		}	
	}
}


/*
 * Display the pitch arm mode
 */
static void kap_display_pitch_arm()
{
	int32_t delta;
	int8_t slow_ticks, fast_ticks;

	if (pitch_arm_mode & PM_CHANGED) {
		pitch_arm_mode ^= PM_CHANGED;

		//printf("kap_display_pitch_arm: roll_arm_mode changed, now 0x%x\n\r", pitch_arm_mode);

		// Display ALT
		lcd_gotoxy(DP_PITCH_ARM_MODE);
		lcd_puts_p(pitch_mode_txt[pitch_arm_mode & ~PM_CHANGED]);

		if ((pitch_arm_mode & ~PM_CHANGED) == PM_CLR) {
			lcd_putc(' '); 

			delta = alt_disp - alt_rcv;

			slow_ticks = (delta % ALT_INCR_FAST) / ALT_INCR_SLOW;
			fast_ticks = delta / ALT_INCR_FAST;

			//printf("kap_display_pitch_arm: Commit, delta = %ld, slow = %d, fast = %d\n\r", delta, slow_ticks, fast_ticks);

			// Send an IO to Flight Sim
			if (slow_ticks != 0)
				fsbus_snd(KAP_DIO_CID, DIO_SW_ALT_ENC_20, slow_ticks, 3);

			if (fast_ticks != 0)
				fsbus_snd(KAP_DIO_CID, DIO_SW_ALT_ENC_500, fast_ticks, 3);
					
		} else {
			lcd_putc(UDCS_ARM);
		}
	}
}

/*
 * Revert from Barometer mode to pitch
 */
static void kap_end_baro()
{
	kap_baro_cancel = 0;
//	printf("kap_end_baro: enter\n\r");

	if ((rhs_mode & ~RHS_CHANGED) == RHS_BARO) {

		printf("kap_end_baro: baro ended\n\r");

		rhs_mode = RHS_ALT | RHS_CHANGED;
		kap_disp_flags = 0xFF;
	}

//	printf("kap_end_baro: exit\n\r");
}

/*
 * Display the barometer mode information
 */
static void kap_display_baro()
{
	//printf("kap_display_baro: enter\n\r");

	if (rhs_mode & RHS_CHANGED) {
		rhs_mode &= ~RHS_CHANGED;

		printf("kap_display_baro: Mode recently changed\n\r");

		lcd_gotoxy(13, 1);
///************
		//Need to show HPA/ inhg characters

		if (baro_mode == BARO_HPA)
			lcd_puts("HPA");
		else
			lcd_puts("INHG");

		// End baro mode in 3 seconds (unless someone changes it)
		kap_baro_cancel = event_register(kap_end_baro, 3 * EVENT_HZ, 1);

		kap_disp_flags = 0xFF; // Let the display routines update the digits
	}

	if (baro_mode == BARO_HPA) 
		kap_displ_baro_hpa();
	else
		kap_displ_baro_inhg();

	//printf("kap_display_baro: exit\n\r");
}

/*
 * Display the RHS information
 */
static void kap_display_rhs()
{
	switch (rhs_mode & ~RHS_CHANGED) {
	case RHS_ALT:
		kap_displ_alt();
		break;
	case RHS_VS:
		kap_displ_vs();
		break;
	case RHS_BARO:
		kap_display_baro();
		break;
	}
}

/*
 * Called to cancel the AP disable blink
 * and to clear the display
 */
static void kap_ap_disable()
{

	if (kap_ap_blink_cancel)
		event_cancel(&kap_ap_blink_cancel);


	lcd_clrscr();
	ap_mode = AP_DISABLED;
	fsx_buttons = 0;
}


static void kap_extingush_alert()
{
	lcd_gotoxy(DP_ALERT);
	lcd_puts(" ");
}


static void kap_display_alerts()
{
	//printf("kap_display_alerts - enter\n\r");
	int32_t delta;

	if (kap_disp_flags & KAP_DC_AIR_ALT) {
		//lcd_gotoxy(10, 0);
		//lcd_puts((signed char *)kap_baro_inhg_fs_blk->fs_display.fs_digits);

		//printf("kap_display_alerts: ALT now %s\n\r", kap_air_alt_fs_blk->fs_display.fs_digits);
		kap_disp_flags ^= KAP_DC_AIR_ALT;
	}


	delta = alt_rcv - air_alt; /* Positive means we need to go up */

	delta = (delta / PT_RND) * PT_RND;

	/* Pitch Trim alerts */

	if (delta > 0) {
		// Up
		pitch_trim = PT_UP;
		if (!kap_pt_alert)
			kap_pt_alert = event_register(kap_pt_display, EVENT_HZ / 4, 0);
	} else if (delta < 0) {
		// Down
		pitch_trim = PT_DOWN;
		if (!kap_pt_alert)
			kap_pt_alert = event_register(kap_pt_display, EVENT_HZ / 4, 0);
	} else {
		// Level
		pitch_trim = PT_NONE;
	}
	
	/* Altitude alerts */

	//	Prior to arriving at required alt
	//		Alert is illuminated 1000ft prior to required altitude.
	// 		Extingushes 200ft before require alt.
	// 		Illuminates momentarily on arrival at selected alt.
	//	After arrival
	//		Flashing alert indicated more than 200ft from selected alt
	//		Alert extinguished if >= 1000ft from selected alt

	if (delta < 0)
		delta *= -1;

	if (alt_alert & ALT_REACHED) {

		if (delta > 200 && delta <= 1000) {
			if ((alt_alert & ~ALT_REACHED) != ALT_200_1000) {
				alt_alert = ALT_200_1000 | ALT_REACHED;
				kap_alert = event_register(kap_alert_flash, EVENT_HZ / 4, 0);
			}
		} else {
			if (kap_alert)
				event_cancel(&kap_alert);

			if (delta >= 1000)
				alt_alert = 0;
			else
				alt_alert = ALT_REACHED;
		}

	} else {

		if (delta == 0) {
			alt_alert = ALT_REACHED | ALT_AT;

			// Illuminate ALERT momentarily
			lcd_gotoxy(DP_ALERT);
			lcd_puts("A");
			event_register(kap_extingush_alert, EVENT_HZ, 1);

		} else if (delta > 200 && delta <= 1000) {
			// Illuminate ALERT continuously

			if (alt_alert != ALT_200_1000) {
				alt_alert = ALT_200_1000;
				lcd_gotoxy(DP_ALERT);
				lcd_puts("A");
			}

		} else {
			// Extingush ALERT
			if (alt_alert != 0) {
				alt_alert = 0;
				lcd_gotoxy(DP_ALERT);
				lcd_puts(" ");
			}
		}

	}

	//printf("kap_display_alerts - exit\n\r");
}

/*
 * This function displays the values on the LCD
 *
 */
static void kap_display()
{	
	// Auto pilot enabled status
	if (ap_mode & AP_CHANGED) {
		if ((ap_mode & AP_MODE) == AP_ENABLED) {

			// Display (AP) at the appropriate position
//			lcd_gotoxy(DP_AP_ENABLED);
//			lcd_putc(UDCS_A_BR);
//			lcd_putc(UDCS_P_BR);
			
			roll_mode = RM_ROL | RM_CHANGED;
			roll_arm_mode = RM_CLR;
			pitch_mode = PM_VS | PM_CHANGED;
			pitch_arm_mode = RM_CLR;
			baro_mode = BARO_INHG; // The default
			rhs_mode = RHS_VS | RHS_CHANGED;
			kap_disp_flags = KAP_DC_ALT | KAP_DC_VS;
			alt_disp = 0;
			vs = 0;
			kap_enc_toggle_status = 0;
			sw_enc_delta = 0;
			fsx_buttons = _BV(DIO_SW_APMASTER);

			// Tell FSBUS that we are now enabled
			fsbus_snd(KAP_DIO_CID, DIO_SW_APMASTER, 1, 3);

		} else {
			lcd_clrscr();

			// Tell FSBUS that we are now enabled
			fsbus_snd(KAP_DIO_CID, DIO_SW_APMASTER, 0, 3);

			// Disable potential events
			if (kap_pt_alert)
				event_cancel(&kap_pt_alert);

			if (kap_baro_cancel)
				event_cancel(&kap_baro_cancel);

			if (kap_vs_cancel)
				event_cancel(&kap_vs_cancel);

			if (kap_rm_blink_cancel)
				event_cancel(&kap_rm_blink_cancel);

			if (kap_vs_pid)
				event_cancel(&kap_vs_pid);

			if (kap_alert)
				event_cancel(&kap_alert);

			// Blink AP

			kap_ap_blink_cancel = event_register(kap_ap_off_blink, EVENT_HZ / 5, 0);
			event_register(kap_ap_disable, 4 * EVENT_HZ, 1);
		}
		ap_mode ^= AP_CHANGED;
	}

	if (ap_mode == AP_ENABLED) {

		kap_display_roll();

		kap_display_roll_arm();

		kap_display_pitch();

		kap_display_pitch_arm();

		kap_display_rhs();

		kap_display_alerts();
	}
}


/************************************ PID Code for VS mode ascent and decents ******************************/

#ifdef NOT

#define K_P     0.10
#define K_I     0.00
#define K_D     0.00

//Parameters for regulator
static struct PID_DATA pidData;

/* Demo of PID controller
 */
static void kap_vs_pid_event(void)
{
	int16_t inputValue;

printf("kap_vs_pid_event - enter\n\r");

	inputValue = pid_Controller(vs, air_vs, &pidData);

printf("kap_vs_pid_event: vs %d, air_vs %d, inputValue %d\n\r", vs, air_vs, inputValue);

	//Set_Input(inputValue);

	fsbus_snd(KAP_DIO_CID, DIO_SW_ELEV_TRIM, inputValue, 3);

printf("kap_vs_pid_event - exit\n\r");
}

static void kap_vs_pid_enable(void)
{
printf("kap_vs_pid_enable - enter\n\r");

	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

	kap_vs_pid = event_register(kap_vs_pid_event, EVENT_HZ * 1, 0);
printf("kap_vs_pid_enable - exit\n\r");
}

static void kap_vs_pid_disable(void)
{
printf("kap_vs_pid_disable - enter\n\r");
	event_cancel(&kap_vs_pid);
printf("kap_vs_pid_disable - exit\n\r");
}

#endif

/************************************ END PID *******************************************/

static void kap_lcd_pgm_udcs(uint8_t start, uint8_t num)
{
	uint8_t i;
	
	// Convert start and num to bytes
	start *= 8;
	num *= 8;

	lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
	for(i = 0; i < num; i++)
           lcd_data(pgm_read_byte_near(&kap_udcs[start + i]));

}

/*
 * The KAP initialisation routine.
 * The AP is off, register the virtual controllers and the main events
 */
void kap_init()
{
	ap_mode = AP_DISABLED;
	lcd_clrscr();
	kap_lcd_pgm_udcs(0, 7);

	kap_alt_fs_blk =		fsbus_register(KAP_ALT_CID, 		FS_CTRL_DISPLAY, kap_rcv_alt);
	kap_vs_fs_blk =			fsbus_register(KAP_VS_CID, 			FS_CTRL_DISPLAY, kap_rcv_vs);
	kap_baro_hpa_fs_blk =	fsbus_register(KAP_BARO_HPA_CID,	FS_CTRL_DISPLAY, kap_rcv_baro_hpa);
	kap_baro_inhg_fs_blk =	fsbus_register(KAP_BARO_INHG_CID, 	FS_CTRL_DISPLAY, kap_rcv_baro_inhg);
	kap_air_alt_fs_blk = 	fsbus_register(KAP_AIR_ALT_CID, 	FS_CTRL_DISPLAY, kap_rcv_air_alt);
	kap_dio_blk = 			fsbus_register(KAP_DIO_CID, 		FS_CTRL_DIO, 	 kap_rcv_dio);
	kap_elev_trim_blk =		fsbus_register(KAP_ELEV_TRIM_CID,	FS_CTRL_DISPLAY, kap_rcv_elev_trim);
	kap_air_vs_blk =		fsbus_register(KAP_AIR_VS_CID,		FS_CTRL_DISPLAY, kap_rcv_air_vs);

	// Register the regular display updates
	event_register(kap_display, EVENT_HZ / 8, 0);
	event_register(kap_buttons, EVENT_HZ / 10, 0);
}
