#include "Pixel.h"
#include "Util.h"

#include <avr/io.h>
#include <util/delay.h>

#define PIXEL_PORT PORTD
#define PIXEL_DDR DDRD
#define PIXEL_BIT (1 << PD0)
#define BYTES_PER_PIXEL 4
#define F_PIXEL 800000
#define PIXEL_PWR_PORT PORTB
#define PIXEL_PWR_DDR DDRB
#define PIXEL_PWR_BIT (1 << PB6)

static struct
{
	uint8_t data[NUM_PIXELS * BYTES_PER_PIXEL];
    uint8_t hue;
    uint8_t saturation;
	uint8_t luminance;
} pixel;

const __flash uint16_t GammaTable[256] = {
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
	1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   3,   3,   3,   4,
	4,   4,   5,   5,   5,   6,   6,   7,   7,   8,   8,   9,   9,   10,  11,
	11,  12,  13,  13,  14,  15,  16,  16,  17,  18,  19,  20,  21,  22,  23,
	24,  25,  26,  27,  28,  29,  30,  32,  33,  34,  35,  37,  38,  39,  41,
	42,  44,  45,  47,  48,  50,  52,  53,  55,  57,  59,  60,  62,  64,  66,
	68,  70,  72,  74,  76,  78,  80,  83,  85,  87,  89,  92,  94,  97,  99,
	102, 104, 107, 109, 112, 115, 117, 120, 123, 126, 129, 132, 135, 138, 141,
	144, 147, 150, 153, 156, 160, 163, 167, 170, 173, 177, 181, 184, 188, 191,
	195, 199, 203, 207, 211, 215, 219, 223, 227, 231, 235, 239, 244, 248, 252,
	257, 261, 266, 270, 275, 280, 284, 289, 294, 299, 304, 309, 314, 319, 324,
	329, 334, 339, 345, 350, 355, 361, 366, 372, 378, 383, 389, 395, 401, 406,
	412, 418, 424, 430, 437, 443, 449, 455, 462, 468, 475, 481, 488, 494, 501,
	508, 515, 521, 528, 535, 542, 549, 557, 564, 571, 578, 586, 593, 601, 608,
	616, 623, 631, 639, 647, 655, 662, 670, 679, 687, 695, 703, 711, 720, 728,
	737, 745, 754, 762, 771, 780, 789, 798, 807, 816, 825, 834, 843, 853, 862,
	871, 881, 890, 900, 910, 919, 929, 939, 949, 959, 969, 979, 989, 999, 1010,
	1020};

void Pixel_Setup(void)
{
	// set power pin high output to close mosfet
	PIXEL_PWR_PORT |= PIXEL_PWR_BIT;
	PIXEL_PWR_DDR |= PIXEL_PWR_BIT;

    // configure timer 0
	// clear OC0B on compare match
	// fast pwm with OCR0A as top
	TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << WGM02);
	// set top for 800kHz interval
	OCR0A  = F_CPU / F_PIXEL - 1;
	OCR0B = 0;
	TCNT0 = 0;

	// PIXEL_PORT &= ~(PIXEL_BIT);
	PIXEL_DDR |= PIXEL_BIT;
}

static void writePixels(void)
{
    LOCKI();

	const uint8_t zero = F_CPU / (F_PIXEL * 4) - 1;
	const uint8_t one = F_CPU / (F_PIXEL * 2) - 1;

	// GCC_MEMORY_BARRIER();

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

	UNLOCKI();
	// wait at least 80us for pixel reset
	_delay_us(80);
}

static uint8_t ditheredGammaLookup(uint8_t i, uint8_t c)
{
	uint16_t g = GammaTable[c];
	uint8_t a = (uint8_t)(g >> 2);
	uint8_t b = (uint8_t)(g & 0x3);
	if ((i & 0x3) < b) ++a;
	return a;
}

static inline void setPixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	pixel.data[i * BYTES_PER_PIXEL + 0] = ditheredGammaLookup(i, g);
	pixel.data[i * BYTES_PER_PIXEL + 1] = ditheredGammaLookup(i, r);
	pixel.data[i * BYTES_PER_PIXEL + 2] = ditheredGammaLookup(i, b);
#if BYTES_PER_PIXEL > 3
	pixel.data[i * BYTES_PER_PIXEL + 3] = ditheredGammaLookup(i, w);
#endif
}

static inline uint8_t minu8(uint8_t a, uint8_t b)
{
	return a < b ? a : b;
}

static void setAllPixel(uint8_t r, uint8_t g, uint8_t b)
{
#if BYTES_PER_PIXEL > 3
	uint8_t w = minu8(minu8(r, g), b);
	r -= w;
	g -= w;
	b -= w;
#else
	uint8_t w = 0;
#endif
	for (uint8_t i = 0; i < NUM_PIXELS; ++i)
		setPixel(i, r, g, b, w);
}

void Pixel_SetPower(bool on)
{
	if (on)
	{
		PIXEL_PWR_PORT &= ~PIXEL_PWR_BIT;

		// wait at least 14ms until capacitor is charged
		// wait 32ms to be on the save side
		_delay_ms(32);
		writePixels();
	}
	else
	{
		PIXEL_PWR_PORT |= PIXEL_PWR_BIT;
	}
}

void Pixel_Clear(void)
{
    setAllPixel(0, 0, 0);
}

void Pixel_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    setAllPixel(r, g, b);
}

void Pixel_Set(int8_t index, uint8_t r, uint8_t g, uint8_t b)
{
	while (index < 0) index += NUM_PIXELS;
	while (index >= NUM_PIXELS) index -= NUM_PIXELS;

#if BYTES_PER_PIXEL > 3
	uint8_t w = minu8(minu8(r, g), b);
	r -= w;
	g -= w;
	b -= w;
#else
	uint8_t w = 0;
#endif
	setPixel(index, r, g, b, w);
}

void Pixel_Write(void)
{
    writePixels();
}
