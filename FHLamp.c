/*
			 LUFA Library
	 Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
		   www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdio.h>

#include "Descriptors.h"
#include "Pixel.h"
#include "Timer.h"
#include "Util.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#define LED1_DDR DDRB
#define LED1_PORT PORTB
#define LED1_BIT (1 << PB0)
#define LED2_DDR DDRD
#define LED2_PORT PORTD
#define LED2_BIT (1 << PD5)

#define ROT_PORT PORTB
#define ROT_DDR DDRB
#define ROT_PIN PINB
#define ROT1_BIT (1 << PB2)
#define ROT2_BIT (1 << PB3)
#define ROT_PCMSK ((1 << PCINT2) | (1 << PCINT3))
#define ROT_STEPS 20

#define BTN_PORT PORTD
#define BTN_DDR DDRD
#define BTN_PIN PIND
#define BTN_BIT (1 << PD1)
#define BTN_INT INT1
#define BTN_ISC0 (1 << ISC10)
#define BTN_ISC1 (1 << ISC11)
#define BTN_INT_vect INT1_vect

#define LUM_STEP 8
#define SAT_STEP 8
#define HUE_STEP 2
#define MAX_HSL 128

#define DOUBLE_CLICK_INTERVAL 500
#define STANDBY_TIME 10000

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config =
		{
			.ControlInterfaceNumber = INTERFACE_ID_CDC_CCI,
			.DataINEndpoint =
				{
					.Address = CDC_TX_EPADDR,
					.Size    = CDC_TXRX_EPSIZE,
					.Banks   = 1,
				},
			.DataOUTEndpoint =
				{
					.Address = CDC_RX_EPADDR,
					.Size    = CDC_TXRX_EPSIZE,
					.Banks   = 1,
				},
			.NotificationEndpoint =
				{
					.Address = CDC_NOTIFICATION_EPADDR,
					.Size    = CDC_NOTIFICATION_EPSIZE,
					.Banks   = 1,
				},
		},
};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;

static struct
{
	volatile uint8_t rotation;
	volatile bool pressed;

	/// Upper rotation limit.
	/// Needs to be <127.
	uint8_t max;
	bool clamp;

	// internal:
	uint8_t rotstate;
	int8_t dir;
} RotButton;

enum mode
{
	STANDBY_MODE,
	MENU_MODE,
	ANIM_MODE,
	SET_HUE_MODE,
	SET_SAT_MODE,
	DEFAULT_MODE,
};

enum mode MainMenu[] = {
	SET_HUE_MODE,
	SET_SAT_MODE,
	DEFAULT_MODE,
	ANIM_MODE,
};

#define MENU_SIZE (sizeof(MainMenu) / sizeof(MainMenu[0]))

static struct
{
	/// Current HSL values.
	/// Max is MAX_HSL (inclusive).
	uint8_t hue;
	uint8_t saturation;
	uint8_t luminance;
	struct { uint8_t r, g, b; } color;

	/// Last button state.
	bool button;
	/// Last rotation.
	uint8_t rotation;

	enum mode mode;
	enum mode last_mode;
	uint16_t anim_time;
	/// Mode updated.
	bool update;

	/// Time of last button press.
	uint16_t last_button;

	bool log_enabled;
} State = {
	.mode = DEFAULT_MODE,
};

ISR(BTN_INT_vect)
{
	RotButton.pressed = (BTN_BIT & BTN_PIN) == 0;
}

ISR(PCINT0_vect)
{
	PORTD |= (1 << PD7);
	uint8_t pins = ROT_PIN;
	uint8_t rotstate = pins & (ROT1_BIT | ROT2_BIT);

	switch (RotButton.rotstate ^ rotstate)
	{
	case ROT1_BIT:
		RotButton.dir += (((rotstate & ROT1_BIT) ? 1 : 0) ^ ((rotstate & ROT2_BIT) ? 1 : 0)) * 2 - 1;
		break;
	case ROT2_BIT:
		RotButton.dir += (((rotstate & ROT1_BIT) ? 0 : 1) ^ ((rotstate & ROT2_BIT) ? 1 : 0)) * 2 - 1;
		break;
	case 0:
		// no change in rotation
		return;
	default:
		break;
	}

	RotButton.rotstate = rotstate;

	if (rotstate == (ROT1_BIT | ROT2_BIT) && RotButton.dir != 0)
	{
		int8_t rot = RotButton.rotation + (RotButton.dir > 0 ? 1 : -1);
		if (rot > RotButton.max)
		{
			rot = RotButton.clamp ? RotButton.max: 0;
		}
		else if (rot < 0)
		{
			rot = RotButton.clamp ? 0 : RotButton.max;
		}

		RotButton.rotation = (uint8_t)rot;
		RotButton.dir = 0;
	}
	PORTD &= ~(1 << PD7);
}

#define LOG(fmt, ...) logmsg(FSTR(fmt), ##__VA_ARGS__)

static void logmsg(const __flash char *fmt, ...)
{
	if (State.log_enabled)
	{
		va_list args;
		va_start(args, fmt);
		vfprintf_P(&USBSerialStream, fmt, args);
		va_end(args);
		fputc('\n', &USBSerialStream);
	}
}


// on Pro Micro board: LEDs are on when port is low
static inline void led1_on(void)
{
	LED1_PORT &= ~LED1_BIT;
}

static inline void led1_off(void)
{
	LED1_PORT |= LED1_BIT;
}

static inline void led2_on(void)
{
	LED2_PORT &= ~LED2_BIT;
}

static inline void led2_off(void)
{
	LED2_PORT |= LED2_BIT;
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
static void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */

	Pixel_Setup();

	// configure button pin
	// set as input
	BTN_DDR &= ~BTN_BIT;
	// enable pull-up
	BTN_PORT |= BTN_BIT;
	// enable interrupt on any edge
	EICRA = BTN_ISC0;
	// enable button interrupt
	EIMSK = (1 << BTN_INT);

	// configure pins for rotation signals
	// set as input
	ROT_DDR &= ~(ROT1_BIT | ROT2_BIT | BTN_BIT);
	// enable pull-ups
	ROT_PORT |= ROT1_BIT | ROT2_BIT | BTN_BIT;
	// enable pin change interrupts
	PCMSK0 |= ROT_PCMSK;
	PCICR |= (1 << PCIE0);

	USB_Init();

	// onboard LEDs
	led1_off();
	led2_off();
	LED1_DDR |= LED1_BIT;
	LED2_DDR |= LED2_BIT;

	StartTimer();

	// Power reduction
	PRR0 = (1 << PRTWI) | (1 << PRSPI) | (1 << PRADC);
	PRR1 = (1 << PRUSART1);
}

static inline int16_t divRound(int16_t a, int16_t b)
{
	return (a + b / 2) / b;
}

static uint8_t hue2rgb(int16_t p, int16_t q, int16_t t)
{
	if (t < 0) t += MAX_HSL;
	if (t > MAX_HSL) t -= MAX_HSL;
	if (t < MAX_HSL / 6)
		return p + divRound((q - p) * 6 * t, MAX_HSL);
	if (t < MAX_HSL / 2) return q;
	if (t < MAX_HSL * 2 / 3)
		return p + divRound((q - p) * (MAX_HSL * 2 / 3 - t) * 6, MAX_HSL);
	return p;
}

static void updateRGB(uint8_t hue, uint8_t sat, uint8_t lum)
{
	if (sat == 0)
	{
		State.color.r = State.color.g = State.color.b = lum;
	}
	else
	{
		uint16_t l = lum;
		uint16_t s = sat;
		uint16_t q = l < MAX_HSL / 2 ? divRound(l * (MAX_HSL + s), MAX_HSL)
									 : l + s - divRound(l * s, MAX_HSL);
		uint16_t p = 2 * l - q;

		State.color.r = hue2rgb(p, q, hue + MAX_HSL / 3);
		State.color.g = hue2rgb(p, q, hue);
		State.color.b = hue2rgb(p, q, hue - MAX_HSL / 3);
	}
}

static void flushlog(void)
{
	if (State.log_enabled)
	{
		fflush(&USBSerialStream);
		CDC_Device_Flush(&VirtualSerial_CDC_Interface);
	}
}

static void updatePixel(void)
{
	flushlog();
	Pixel_Write();
}

static void setAllPixel(void)
{
	Pixel_SetAll(State.color.r, State.color.g, State.color.b);
	updatePixel();
}

static void setMode(enum mode mode, uint8_t rot, uint8_t max, bool clamp)
{
	LOG("mode: %#x, %u, %u", mode, rot, max);
	State.mode = mode;
	State.rotation = rot;
	State.update = true;
	LOCKI();
	RotButton.rotation = State.rotation;
	RotButton.max = max;
	RotButton.clamp = clamp;
	UNLOCKI();
}

static void setLuminanceMode(enum mode mode)
{
	if (State.luminance == 0) State.luminance = LUM_STEP;
	setMode(mode, State.luminance / LUM_STEP, MAX_HSL / LUM_STEP, true);
}

static void setHueMode(void)
{
	setMode(SET_HUE_MODE, State.hue / HUE_STEP, MAX_HSL / HUE_STEP, false);
}

static void setSaturationMode(void)
{
	setMode(SET_SAT_MODE, State.saturation / SAT_STEP, MAX_HSL / SAT_STEP, true);
}

static void setStandbyMode(void)
{
	State.anim_time = GetTime();
	State.last_mode = State.mode;
	setMode(STANDBY_MODE, 0, 1, true);
	Pixel_Clear();
	flushlog();
	Pixel_Write();
}

static void updateLuminance(void)
{
	State.luminance = (uint8_t)State.rotation * LUM_STEP;
	LOG("l: %d", State.luminance);
	updateRGB(State.hue, State.saturation, State.luminance);
	setAllPixel();

	if (State.luminance == 0)
		setStandbyMode();
}

static void updateHue(void)
{
	State.hue = (uint8_t)State.rotation * HUE_STEP;
	// LOG("h: %d", State.hue);
	updateRGB(State.hue, MAX_HSL, MAX_HSL / 2);
	Pixel_SetAll(State.color.r, State.color.g, State.color.b);

	// animate luminance of front pixel
	uint16_t dt = GetTime() - State.anim_time;
	uint8_t a = MAX_HSL / 2 - ((dt >> 4) & (MAX_HSL / 2 - 1));
	updateRGB(State.hue, MAX_HSL, a);
	Pixel_Set(FRONT_PIXEL - 1, State.color.r, State.color.g, State.color.b);
	Pixel_Set(FRONT_PIXEL + 0, State.color.r, State.color.g, State.color.b);
	Pixel_Set(FRONT_PIXEL + 1, State.color.r, State.color.g, State.color.b);

	updatePixel();
}

static void updateSaturation(void)
{
	State.saturation = (uint8_t)State.rotation * SAT_STEP;
	// LOG("s: %d", State.saturation);
	updateRGB(State.hue, State.saturation, MAX_HSL / 2);
	Pixel_SetAll(State.color.r, State.color.g, State.color.b);

	// animate luminance of front pixel
	uint16_t dt = GetTime() - State.anim_time;
	uint8_t a = MAX_HSL / 2 - ((dt >> 4) & (MAX_HSL / 2 - 1));
	updateRGB(State.hue, State.saturation, a);
	Pixel_Set(FRONT_PIXEL - 1, State.color.r, State.color.g, State.color.b);
	Pixel_Set(FRONT_PIXEL + 0, State.color.r, State.color.g, State.color.b);
	Pixel_Set(FRONT_PIXEL + 1, State.color.r, State.color.g, State.color.b);

	updatePixel();
}

static uint8_t rnd(void)
{
	static uint8_t s = 0xaa, a = 0;
	s ^= s << 3;
	s ^= s >> 5;
	s ^= a++ >> 2;
	return s;
}

static uint8_t flickering(uint8_t max)
{
	static uint16_t last = 0;
	static uint8_t acc = 0;
	static uint8_t interval = 0;
	uint16_t t = GetTime();
	uint16_t dt = (t - last) / 4;

	if (dt > interval)
	{
		last = t;
		interval = rnd() / 4 + 16;
		uint8_t b = rnd() & (MAX_HSL - 1);
		if (dt + b > (uint16_t)acc)
			acc = b;
		else
			acc -= dt;
		dt = 0;
	};

	return (uint8_t)(
		((uint16_t)(acc > dt ? acc - dt : 0) * (uint16_t)max + MAX_HSL / 2) /
		(uint16_t)MAX_HSL) | 1;
}

static void updateFlickering(void)
{
	State.luminance = (uint8_t)State.rotation * LUM_STEP;
	// LOG("l: %d", State.luminance);

	uint8_t lum = flickering(State.luminance);
	updateRGB(State.hue, State.saturation, lum);
	setAllPixel();

	if (State.luminance == 0)
		setStandbyMode();
}

static void updateMenu(bool changed)
{
	uint16_t t = GetTime();
	if (changed) State.anim_time = t;

	uint16_t dt = t - State.anim_time;

	uint8_t h = State.hue;
	uint8_t s = State.saturation;
	uint8_t l = State.luminance;

	enum mode entry = MainMenu[State.rotation];

	if (changed) {
		LOG("menu: %#x", entry);
	}

	switch (entry) {
	case SET_HUE_MODE:
		// animate hue
		h = (dt >> 4) & (MAX_HSL - 1);
		// set full saturation
		s = MAX_HSL;
		// set half luminance for full color
		l = MAX_HSL / 2;
		break;
	case SET_SAT_MODE:
		// animate saturation
		s = MAX_HSL - ((dt >> 4) & (MAX_HSL - 1));
		// set half luminance for full color
		l = MAX_HSL / 2;
		break;
	case DEFAULT_MODE:
		// animate luminance
		l = ((dt >> 4) & (MAX_HSL / 2 - 1));
		if (l < MAX_HSL / 4) l = MAX_HSL / 2 - l;
		l += MAX_HSL / 4;
		break;
	case ANIM_MODE:
		// animate flickering
		l = MAX_HSL - (((dt >> 1) & (MAX_HSL - 1)) ^ (MAX_HSL / 4));
		break;
	default:
		// should never happen
		LOG("illegal menu state: %#x", entry);
		break;
	}

	uint8_t index = (NUM_PIXELS * State.rotation + MENU_SIZE / 2) / MENU_SIZE;

	updateRGB(h, s, l);

	if (changed) {
		LOG("menucolor: (%u %u %u) (%u %u %u)",
				  h, s, l, State.color.r, State.color.g, State.color.b);
	}

	Pixel_Clear();
	Pixel_Set(index - 1, State.color.r, State.color.g, State.color.b);
	Pixel_Set(index + 0, State.color.r, State.color.g, State.color.b);
	Pixel_Set(index + 1, State.color.r, State.color.g, State.color.b);
	updatePixel();
}

static bool checkSleep(void)
{
	cli();
	bool cond = State.button == RotButton.pressed &&
		State.rotation == RotButton.rotation;
	if (cond)
	{
		// sleep when no updates
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
	}
	else
	{
		sei();
	}

	return cond;
}

static void updateStandby(bool rotated)
{
	uint16_t t = GetTime();
	uint16_t dt = t - State.anim_time;
	if (!rotated && dt > STANDBY_TIME) {
		Pixel_SetPower(false);

		LOG("power off");
		flushlog();
		State.log_enabled = false;

		USB_Disable();

		// set_sleep_mode(SLEEP_MODE_IDLE);
		set_sleep_mode(SLEEP_MODE_STANDBY);
		while (checkSleep())
			;

		USB_Init();

		rotated = RotButton.rotation != State.rotation;
		Pixel_SetPower(true);

		LOG("wake up");
	}

	if (rotated) {
		State.luminance = LUM_STEP * RotButton.rotation;
		setLuminanceMode(State.last_mode);
	}
}

static void handleMode(void)
{
	uint8_t newrot = RotButton.rotation;
	bool rotated = (newrot != State.rotation);
	bool changed = rotated || State.update;
	State.rotation = newrot;
	State.update = false;

	if (changed)
		LOG("rot: %u", newrot);

	switch (State.mode)
	{
	case STANDBY_MODE:
		updateStandby(rotated);
		break;
	case DEFAULT_MODE:
		if (changed) updateLuminance();
		break;
	case SET_HUE_MODE:
		updateHue();
		break;
	case SET_SAT_MODE:
		updateSaturation();
		break;
	case MENU_MODE:
		updateMenu(changed);
		break;
	case ANIM_MODE:
		updateFlickering();
		break;
	default:
		break;
	}
}

static void menuSelect(void)
{
	enum mode entry = MainMenu[State.rotation];

	switch (entry) {
	case SET_HUE_MODE:
		setHueMode();
		break;
	case SET_SAT_MODE:
		setSaturationMode();
		break;
	case DEFAULT_MODE: // fallthrough
	case ANIM_MODE:
		setLuminanceMode(entry);
		break;
	default:
		// should never happen
		LOG("illegal menu select: %#x", entry);
		setLuminanceMode(DEFAULT_MODE);
		break;
	}
}

static void enterMenu(void)
{
	uint8_t e = 0;
	// find current mode in MainMenu
	for (uint8_t i = 0; i < MENU_SIZE; ++i)
		if (State.mode == MainMenu[i]) {
			e = i;
			break;
		}

	setMode(MENU_MODE, e, MENU_SIZE - 1, false);
}

static void handleButton(void)
{
	bool newbutton = RotButton.pressed;
	bool click = false;

	if (State.button != newbutton)
	{
		State.button = newbutton;
		LOG("b: %u", newbutton ? 1 : 0);
		flushlog();
		click = newbutton == 0; // button release?
	}

	if (!click) return;

	uint16_t t = GetTime();
	uint16_t dt = t - State.last_button;
	State.last_button = t;

	if (dt < DOUBLE_CLICK_INTERVAL)
	{
		enterMenu();
	}
	else
	{
		switch (State.mode)
		{
		case ANIM_MODE: // fallthrough
		case DEFAULT_MODE: setStandbyMode(); break;
		case STANDBY_MODE: setLuminanceMode(State.last_mode); break;
		case MENU_MODE: menuSelect(); break;
		default:
			// get back to main menu
			enterMenu();
			break;
		}
	}
}

static void processHSL(const char *buf)
{
	uint16_t h, s, l;
	if (sscanf(buf, "%u %u %u", &h, &s, &l) == 3 &&
	    h < 256 && s < 256 && l < 256)
	{
		State.hue = h;
		State.saturation = s;
		State.luminance = l;

		updateRGB(State.hue, State.saturation, State.luminance);
		setAllPixel();
	}
	else
	{
		LOG("\nexpected; <hue> <sat> <lum>");
	}
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	/* Create a regular blocking character stream for the interface so that it can be used with the stdio.h functions */
	// CDC_Device_CreateBlockingStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	GlobalInterruptEnable();

	led2_on();

	char linebuf[64];
	uint8_t bufpos = 0;
	enum {
		NO_PROMPT,
		HSL_PROMPT,
	} prompt = NO_PROMPT;

	Pixel_SetPower(true);
	setStandbyMode();



	// if ( == NEW_LUFA_SIGNATURE) {
	// 	_updatedLUFAbootloader = true;
	// }

	led2_off();

	for (;;)
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
		checkSleep();

		handleButton();
		handleMode();

		int c = fgetc(&USBSerialStream);
		if (c != EOF)
		{
			// led1_on();

			if (prompt != NO_PROMPT)
			{
				if (c == '\n' || c == '\r')
				{
					fputc('\r', &USBSerialStream);
					fputc('\n', &USBSerialStream);
					flushlog();
					if (bufpos > 0)
					{
						linebuf[bufpos] = '\0';
						switch (prompt)
						{
						case HSL_PROMPT: processHSL(linebuf); break;
						default: break;
						}
						flushlog();
						bufpos = 0;
					}
					prompt = false;
				}
				else if (bufpos < sizeof(linebuf) - 1)
				{
					linebuf[bufpos] = c;
					++bufpos;
				}

				// echo char
				fputc(c, &USBSerialStream);
				flushlog();
			}
			else
			{
				switch (c)
				{
				case 't':
					LOG("time: %u", GetTime());
					break;
				case 'i':
					LOG("hsl> ");
					prompt = HSL_PROMPT;
					break;
				case 'd':
					LOG("hsl: (%u %u %u) (%u %u %u)", State.hue,
						State.saturation, State.luminance, State.color.r,
						State.color.g, State.color.b);
					break;
				case 'l':
					setLuminanceMode(DEFAULT_MODE);
					break;
				case 'h':
					setHueMode();
					break;
				case 's':
					setSaturationMode();
					break;
				case 'a':
					setLuminanceMode(ANIM_MODE);
					break;
				case 'p':
					if (State.mode == STANDBY_MODE) setLuminanceMode(State.last_mode);
					else setStandbyMode();
					break;
				}
			}

			// led1_off();
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	}
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

static inline void checkSerial(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	State.log_enabled =
		(CDCInterfaceInfo->State.ControlLineStates.HostToDevice &
		 CDC_CONTROL_LINE_OUT_DTR) != 0 &&
		CDCInterfaceInfo->State.LineEncoding.BaudRateBPS > 1200;
}

static bool bps1200active = false;

void EVENT_CDC_Device_ControLineStateChanged(
	USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	checkSerial(CDCInterfaceInfo);


	uint16_t *const pmagic = (uint16_t *)0x0800;
	if (bps1200active &&
		(CDCInterfaceInfo->State.ControlLineStates.HostToDevice &
		 CDC_CONTROL_LINE_OUT_DTR) == 0)
	{
		led1_on();
		*pmagic = 0x7777;
		wdt_enable(WDTO_120MS);
	}
	else
	{
		led1_off();
		wdt_disable();
		wdt_reset();
		*pmagic = 0;
	}
}

void EVENT_CDC_Device_LineEncodingChanged(
	USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	checkSerial(CDCInterfaceInfo);

	bps1200active = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice &
					 CDC_CONTROL_LINE_OUT_DTR) != 0 &&
					CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 1200;

}
