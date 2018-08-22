#include "Timer.h"
#include "Util.h"

#include <avr/io.h>

static volatile uint16_t time = 0;

ISR(TIMER3_COMPA_vect)
{
    ++time;
}

void StartTimer(void)
{
    // enable timer compare match A interrupt
    TIMSK3 = (1 << OCIE3A);
    // set counter top for 1ms at F_CPU/64
    OCR3A = F_CPU / 1000 / 64 - 1;
    // clear timer on compare match, clock div 64
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30);
}

uint16_t GetTime(void)
{
    LOCKI();
    uint16_t t = time;
    UNLOCKI();
    return t;
}
