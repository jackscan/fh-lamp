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
#include <string.h>
#include <stdio.h>

#include "Descriptors.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#define PIXEL_PORT PORTD
#define PIXEL_DDR DDRD
#define PIXEL_BIT (1 << PD0)
#define NUM_PIXELS 12
#define BYTES_PER_PIXEL 4
#define F_PIXEL 800000
#define PIXEL_PWR_PORT PORTB
#define PIXEL_PWR_DDR DDRB
#define PIXEL_PWR_BIT (1 << PB6)

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
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

/** Counter for the software PWM. */
static volatile uint8_t SoftPWM_Count;

/** Duty cycle for the first software PWM channel. */
static volatile uint8_t SoftPWM_Channel1_Duty;

/** Duty cycle for the second software PWM channel. */
static volatile uint8_t SoftPWM_Channel2_Duty;

/** Duty cycle for the third software PWM channel. */
static volatile uint8_t SoftPWM_Channel3_Duty;

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;


/** Interrupt handler for managing the software PWM channels for the LEDs */
// ISR(TIMER0_COMPA_vect, ISR_BLOCK)
// {
// 	uint8_t LEDMask = LEDS_ALL_LEDS;

// 	if (++SoftPWM_Count == 0b00011111)
// 	  SoftPWM_Count = 0;

// 	if (SoftPWM_Count >= SoftPWM_Channel1_Duty)
// 	  LEDMask &= ~LEDS_LED1;

// 	if (SoftPWM_Count >= SoftPWM_Channel2_Duty)
// 	  LEDMask &= ~LEDS_LED2;

// 	if (SoftPWM_Count >= SoftPWM_Channel3_Duty)
// 	  LEDMask &= ~LEDS_LED3;

// 	LEDs_SetAllLEDs(LEDMask);
// }


// enum
// {
// 	ROT1 = 0x1,
// 	ROT2 = 0x2,
// 	BTN = 0x4,
// };

static struct
{
	volatile uint8_t rotation;
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
		int8_t rot = (int8_t)RotButton.rotation + (RotButton.dir > 0 ? 1 : -1);
		if (rot < 0) rot = ROT_STEPS - 1;
		else if (rot >= ROT_STEPS) rot = 0;
		RotButton.rotation = rot;
		RotButton.dir = 0;
	}
	PORTD &= ~(1 << PD7);
}

static struct
{
	uint8_t data[NUM_PIXELS * BYTES_PER_PIXEL];
	uint8_t luminance;
	bool on;
} pixel;

// on Pro Micro board: LEDs or on when port is low
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

	// set power pin high output to close mosfet
	PIXEL_PWR_PORT |= PIXEL_PWR_BIT;
	PIXEL_PWR_DDR |= PIXEL_PWR_BIT;

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

	// LEDs_Init();
	USB_Init();

	// running timer 0 for on a 800kHz interval
	// clear OC0B on compare match
	// fast pwm with OCR0A as top
	OCR0A  = F_CPU / F_PIXEL - 1;
	OCR0B = 0;
	TCNT0 = 0;
	TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << WGM02);

	// PIXEL_PORT &= ~(PIXEL_BIT);
	PIXEL_DDR |= PIXEL_BIT;

	// onboard LEDs
	led1_off();
	led2_off();
	LED1_DDR |= LED1_BIT;
	LED2_DDR |= LED2_BIT;

	// // for debugging
	// DDRD |= (1 << PD7);
	// PORTD &= ~(1 << PD7);
}

static void writePixels(void)
{
	CDC_Device_Flush(&VirtualSerial_CDC_Interface);
	GlobalInterruptDisable();

	const uint8_t zero = F_CPU / (F_PIXEL * 4) - 1;
	const uint8_t one = F_CPU / (F_PIXEL * 2) - 1;

	GCC_MEMORY_BARRIER();

	uint8_t *p = pixel.data;
	uint8_t c = *p;
	// set up timer
	// set OCR0B for first cycle
	OCR0B = (c & 0x80) != 0 ? one : zero;
	TCNT0 = 0;
	TIFR0 = (1 << TOV0);
	TCCR0B |= (1 << CS00);

	// byte counter
	uint8_t i = NUM_PIXELS * BYTES_PER_PIXEL;
	// bit counter
	uint8_t b = 7;
	c <<= 1;
	// loop over bytes
	while (true)
	{
		// loop over bits
		while (true)
		{
			// prepare for next cycle
			uint8_t ocr0b = (c & 0x80) != 0 ? one : zero;
			// wait for previous cycle to finish
			while ((TIFR0 & (1 << TOV0)) == 0)
				;
			OCR0B = ocr0b;
			// reset overflow flag
			TIFR0 = (1 << TOV0);

			if (--b == 0) break;
			c <<= 1;
		}

		if (--i == 0) break;
		b = 8;
		c = *(++p);
	}

	// wait for start of last cycle
	while ((TIFR0 & (1 << TOV0)) == 0)
		;

	// wait for last compare match where OC0B is cleared
	TIFR0 = (1 << OCF0B);
	while ((TIFR0 & (1 << OCF0B)) == 0)
		;

	// stop timer
	TCCR0B &= ~(1 << CS00);
	// OC0B shall be zero
	OCR0B = 0;
	TCNT0 = 0;

	GlobalInterruptEnable();
	// wait at least 80us for pixel reset
	_delay_us(80);
}

static inline uint8_t minu8(uint8_t a, uint8_t b)
{
	return a < b ? a : b;
}

static inline void setPixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	pixel.data[i * BYTES_PER_PIXEL + 0] = g;
	pixel.data[i * BYTES_PER_PIXEL + 1] = r;
	pixel.data[i * BYTES_PER_PIXEL + 2] = b;
#if BYTES_PER_PIXEL > 3
	pixel.data[i * BYTES_PER_PIXEL + 3] = w;
#endif
}

static void setAllPixel(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t w = minu8(minu8(r, g), b);
	for (uint8_t i = 0; i < NUM_PIXELS; ++i)
		setPixel(i, r, g, b, w);

}

static void togglePixel(void)
{
	if (pixel.on)
	{
		pixel.on = false;
		PIXEL_PWR_PORT |= PIXEL_PWR_BIT;
	}
	else
	{
		pixel.on = true;
		PIXEL_PWR_PORT &= ~PIXEL_PWR_BIT;

		bool allOff = true;
		for (uint8_t i = 0; i < NUM_PIXELS * BYTES_PER_PIXEL; ++i)
			if (pixel.data[i] > 0) allOff = false;

		if (allOff) setAllPixel(1, 1, 1);
		writePixels();
	}
}

static void setLuminance(uint8_t l)
{
	pixel.luminance = l;
	setAllPixel(l, l, l);
	writePixels();
}

static void processCmd(const char *buf)
{
	fprintf(&USBSerialStream, "\nprocessing: %s\n", buf);
	fflush(&USBSerialStream);

	unsigned num, r, g, b, w;
	if (sscanf(buf, "%u %u %u %u %u", &num, &r, &g, &b, &w) == 5)
	{
		setPixel(num, r, g, b, w);
		writePixels();
	}
	else if (sscanf(buf, "a %u %u %u %u", &r, &g, &b, &w) == 4)
	{
		for (uint8_t i = 0; i < NUM_PIXELS; ++i)
			setPixel(i, r, g, b, w);
		writePixels();
	}
	else
	{
		fprintf(&USBSerialStream, "invalid command\n");
		fflush(&USBSerialStream);
	}
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	/* Create a regular blocking character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateBlockingStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	// CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	GlobalInterruptEnable();

	// printf("hello world\n");


	// char linebuf[64];
	// uint8_t bufpos = 0;

	uint8_t rot = 0;
	// uint8_t state = 0;
	bool btn = false;

	for (;;)
	{
		// if (rot != RotButton.rotation)
		// {
		// 	rot = RotButton.rotation;
		// 	fprintf(&USBSerialStream, "r: %u\n", rot);
		// 	fflush(&USBSerialStream);
		// }

		uint8_t newrot = RotButton.rotation;

		if (rot != newrot)
		{
			rot = newrot;
			fprintf(&USBSerialStream, "s: %#x\n", rot);
			fflush(&USBSerialStream);
			setLuminance(RotButton.rotation * 255 / 20);
		}

		if (btn != RotButton.pressed)
		{
			btn = RotButton.pressed;
			fprintf(&USBSerialStream, "b: %u\n", btn ? 1 : 0);
			fflush(&USBSerialStream);
			if (btn == 0) togglePixel();
		}

		/*// LEDs_TurnOnLEDs(LEDS_LED1);
		int c = fgetc(&USBSerialStream);
		// LEDs_TurnOnLEDs(LEDS_LED1);

		led1_on();

		if (c == '\n' || c == '\r')
		{
			fputc('\r', &USBSerialStream);
			fflush(&USBSerialStream);
			if (bufpos > 0)
			{
				linebuf[bufpos] = '\0';
				processCmd(linebuf);
				fprintf(&USBSerialStream, "done.\n");
				fflush(&USBSerialStream);
				bufpos = 0;
			}
		}
		else if (bufpos < sizeof(linebuf) - 1)
		{
			linebuf[bufpos] = c;
			++bufpos;
		}

		// echo char
		fputc(c, &USBSerialStream);
		fflush(&USBSerialStream);

		led1_off();

		// if (c != EOF)
		// {
		// 	fprintf(&USBSerialStream, "> %#x\n", c);

		// }
		// else
		// {
		// 	fputc('.', &USBSerialStream);
		// 	fflush(&USBSerialStream);
		// }*/

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

