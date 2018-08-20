#ifndef _PIXEL_H_
#define _PIXEL_H_

#include <stdbool.h>
#include <stdint.h>

#define NUM_PIXELS 12
#define NUM_LUMINANCE_STEPS 256

void Pixel_Setup(void);
void Pixel_SetPower(bool);
void Pixel_SetLuminance(uint8_t);
void Pixel_SetHSL(uint8_t hue, uint8_t sat, uint8_t lum);
void Pixel_Clear(void);
void Pixel_Set(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void Pixel_SetAll(uint8_t r, uint8_t g, uint8_t b);
void Pixel_Write(void);

#endif
