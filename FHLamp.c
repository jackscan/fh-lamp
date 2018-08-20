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
	volatile int8_t rotation;
	volatile bool pressed;
	uint8_t rotstate;
	int8_t dir;
} RotButton;

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
		RotButton.rotation += (RotButton.dir > 0 ? 1 : -1);
		RotButton.dir = 0;
	}
	PORTD &= ~(1 << PD7);
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

// static void processCmd(const char *buf)
// {
// 	fprintf(&USBSerialStream, "\nprocessing: %s\n", buf);
// 	fflush(&USBSerialStream);

// 	unsigned num, r, g, b, w;
// 	if (sscanf(buf, "%u %u %u %u %u", &num, &r, &g, &b, &w) == 5)
// 	{
// 		setPixel(num, r, g, b, w);
// 		writePixels();
// 	}
// 	else if (sscanf(buf, "a %u %u %u %u", &r, &g, &b, &w) == 4)
// 	{
// 		for (uint8_t i = 0; i < NUM_PIXELS; ++i)
// 			setPixel(i, r, g, b, w);
// 		writePixels();
// 	}
// 	else
// 	{
// 		fprintf(&USBSerialStream, "invalid command\n");
// 		fflush(&USBSerialStream);
// 	}
// }

static uint8_t updateLuminance(uint8_t rot)
{
	const uint8_t step = 25;
	const uint8_t max = NUM_LUMINANCE_STEPS / step;
	LOCKI();
	if (RotButton.rotation >= max) RotButton.rotation = max - 1;
	else if (RotButton.rotation < 0) RotButton.rotation = 0;
	uint8_t newrot = RotButton.rotation;
	UNLOCKI();

	if (rot != newrot)
	{
		fprintf(&USBSerialStream, "s: %#x\n", newrot);
		fflush(&USBSerialStream);
		CDC_Device_Flush(&VirtualSerial_CDC_Interface);
		Pixel_SetLuminance(newrot * step);
	}
	return newrot;
}

// static void animateLuminance(void)
// {
// 	static const uint8_t l[16] = {
// 		0, 9, 8, 7, 6, 9, 8, 6, 2, 0, 8, 7, 6, 1, 0, 5,
// 	};
// 	uint16_t t = GetTime();
// 	uint8_t i = t / 128;
// 	uint8_t r = t % 128;
// 	uint16_t a = l[i % 16] * 25;
// 	uint16_t b = l[(i + 1) % 16]*25;
// 	uint8_t c = (uint8_t)((a * (128 - r) + b * r) / 128);
// 	Pixel_SetLuminance(c);
// }

static uint8_t rnd(void)
{
	static uint8_t s = 0xaa, a = 0;
	s ^= s << 3;
	s ^= s >> 5;
	s ^= a++ >> 2;
	return s;
}

static void animateLuminance(void)
{
	static uint16_t last = 0;
	static uint8_t acc = 0;
	static uint8_t interval = 0;
	uint16_t t = GetTime();
	uint16_t dt = (t - last) / 4;

	if (dt > interval)
	{
		last = t;
		// interval = rnd() / 2 + 16;
		// uint8_t b = rnd() / 16;
		// if ((uint16_t)acc + (uint16_t)b < 256)
		// 	acc += b;
		// else
		// 	acc = 255;
		interval = rnd() / 4 + 16;
		uint8_t b = rnd();
		if (dt + b > (uint16_t)acc)
			acc = b;
		else
			acc -= dt;
		dt = 0;
	};
	uint8_t l = acc > dt ? acc - dt : 0;
	Pixel_SetLuminance(l);
}

static uint8_t rotateLight(uint8_t rot)
{
	const uint8_t max = NUM_PIXELS;
	LOCKI();
	if (RotButton.rotation >= max) RotButton.rotation = 0;
	else if (RotButton.rotation < 0) RotButton.rotation = max - 1;
	uint8_t newrot = RotButton.rotation;
	UNLOCKI();

	if (newrot != rot)
	{
		rot = newrot;
		Pixel_Clear();
		Pixel_Set(newrot, 255, 255, 255);
		Pixel_Write();
	}
	return newrot;
}

static uint8_t hue2rgb(uint16_t p, uint16_t q, int16_t t)
{
	if (t < 0) t += 255;
	if (t > 255) t -= 255;
	if (t < 255 / 6) return p + ((q - p) * 6 * t + 128) / 255;
	if (t < 255 / 2) return q;
	if (t < 255 * 2 / 3)
		return p + ((q - p) * (255 * 2 / 3 - t) * 6 + 128) / 255;
	return p;
}

static void processHSL(const char *buf)
{
	uint16_t h, s, l;
	if (sscanf(buf, "%u %u %u", &h, &s, &l) == 3 &&
	    h < 256 && s < 256 && l < 256)
	{
		// Pixel_SetHSL(h, s, l);
		// Pixel_Write();

		uint8_t r, g, b;
		if (s == 0)
		{
			r = g = b = l; // achromatic
		}
		else
		{
			uint16_t q = l < 128 ? (l * (255 + s) + 128) / 255
								: l + s - (l * s + 128) / 255;
			uint16_t p = 2 * l - q;
			r          = (hue2rgb(p, q, h + 255 / 3));
			g          = (hue2rgb(p, q, h));
			b          = (hue2rgb(p, q, h - 255 / 3));
		}

		fprintf_P(&USBSerialStream, FSTR("\n\r%u %u %u\n"), r, g, b);
		fflush(&USBSerialStream);
		CDC_Device_Flush(&VirtualSerial_CDC_Interface);

		Pixel_SetAll(r, g, b);
		Pixel_Write();
	}
	else
	{
		fprintf_P(&USBSerialStream, FSTR("\nexpected; <hue> <sat> <lum>\n"));
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

	char linebuf[64];
	uint8_t bufpos = 0;
	enum {
		NO_PROMPT,
		HSL_PROMPT,
	} prompt = NO_PROMPT;

	uint8_t rot = 0;
	bool btn = false;
	bool powered = false;

	enum {
		DEFAULT_MODE,
		ANIM_MODE,
		ROTATE_MODE,
	} mode = DEFAULT_MODE;

	for (;;)
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
        // check updates
        cli();
        if (btn == RotButton.pressed && rot == RotButton.rotation)
		{
            // sleep when no updates
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
		}

		if (btn != RotButton.pressed)
		{
			btn = RotButton.pressed;
			fprintf(&USBSerialStream, "b: %u\n", btn ? 1 : 0);
			fflush(&USBSerialStream);
			if (btn == 0) // toggle pixel on button release
			{
				powered = !powered;
				Pixel_SetPower(powered);
			}
		}

		if (powered)
		{
			switch (mode)
			{
			case DEFAULT_MODE:
				rot = updateLuminance(rot);
				break;
			case ANIM_MODE:
				animateLuminance();
				break;
			case ROTATE_MODE:
				rot = rotateLight(rot);
				break;
			default:
				break;
			}
		}

		int c = fgetc(&USBSerialStream);
		if (c != EOF)
		{
			led1_on();

			if (prompt != NO_PROMPT)
			{
				if (c == '\n' || c == '\r')
				{
					fputc('\r', &USBSerialStream);
					fputc('\n', &USBSerialStream);
					fflush(&USBSerialStream);
					if (bufpos > 0)
					{
						linebuf[bufpos] = '\0';
						switch (prompt)
						{
						case HSL_PROMPT: processHSL(linebuf); break;
						default: break;
						}
						fflush(&USBSerialStream);
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
				fflush(&USBSerialStream);
			}
			else
			{
				switch (c)
				{
				case 't':
					fprintf_P(&USBSerialStream, FSTR("time: %u\n"), GetTime());
					break;
				case 'h':
					fprintf_P(&USBSerialStream, FSTR("hsl> "));
					prompt = HSL_PROMPT;
					break;
				case 'r':
					mode = mode != ROTATE_MODE ? ROTATE_MODE : DEFAULT_MODE;
					fprintf_P(&USBSerialStream, FSTR("rotate: %u\n"), mode == ROTATE_MODE);
					break;
				case 'a':
					mode = mode != ANIM_MODE ? ANIM_MODE : DEFAULT_MODE;
					fprintf_P(&USBSerialStream, FSTR("anim: %u\n"), mode == ANIM_MODE);
					break;
				case 'p':
					powered = !powered;
					Pixel_SetPower(powered);
					break;
				}
			}

			led1_off();
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
