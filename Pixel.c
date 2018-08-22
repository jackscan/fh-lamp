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

// const __flash uint8_t GammaTable[20] = {
// 	0,  1,  2,  3,  4,   8,   13,  19,  27,  37,
// 	48, 62, 77, 95, 115, 138, 163, 191, 222, 255,
// };

// const __flash uint8_t GammaTable[NUM_LUMINANCE_STEPS] = {
//     0,   1,   5,  15,  31,  55,  89, 133, 188, 255
// };

const __flash uint8_t GammaTable[NUM_LUMINANCE_STEPS] = {
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
	1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,
	3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,
	6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
	11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,
	17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
	25,  26,  27,  27,  28,  29,  29,  30,  31,  31,  32,  33,  34,  34,  35,
	36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,
	49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
	64,  65,  66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,
	82,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99,  100, 102,
	103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 124, 125,
	127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150, 152,
	154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
	184, 186, 188, 191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
	218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252,
	255,
};

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

		// bool allOff = true;
		// for (uint8_t i = 0; i < NUM_PIXELS * BYTES_PER_PIXEL; ++i)
		// 	if (pixel.data[i] > 0) allOff = false;

		// if (allOff) setAllPixel(1, 1, 1);

		// wait 16ms for charging capacitor
		_delay_ms(32);
		writePixels();
	}
	else
	{
		PIXEL_PWR_PORT |= PIXEL_PWR_BIT;
	}
}



static uint8_t gammaLookup(uint8_t c)
{
	return GammaTable[c];
}





void Pixel_Clear(void)
{
    setAllPixel(0, 0, 0);
}

void Pixel_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    setAllPixel(gammaLookup(r), gammaLookup(g), gammaLookup(b));
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
	setPixel(index, gammaLookup(r), gammaLookup(g), gammaLookup(b),
			 gammaLookup(w));
}

void Pixel_Write(void)
{
    writePixels();
}
