#ifndef _PIXEL_H_
#define _PIXEL_H_

#include <stdbool.h>
#include <stdint.h>

#define NUM_PIXELS 12
#define FRONT_PIXEL 9

void Pixel_Setup(void);
void Pixel_SetPower(bool);
void Pixel_Clear(void);
void Pixel_Set(int8_t index, uint8_t r, uint8_t g, uint8_t b);
void Pixel_SetAll(uint8_t r, uint8_t g, uint8_t b);
void Pixel_Write(void);

#endif
