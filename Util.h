#ifndef _UTIL_H_
#define _UTIL_H_

#include <avr/interrupt.h>

#define LOCKI() uint8_t _sreg = SREG; cli()
#define RELOCKI() _sreg = SREG; cli()
#define UNLOCKI() SREG = _sreg

#define FSTR(str) ({static const __flash char s[] = str; &s[0];})

#endif
