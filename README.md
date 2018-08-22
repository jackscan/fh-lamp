# FH-Lamp

## Build
- clone lufa sources (https://github.com/abcminiuser/lufa.git)
- checkout tag LUFA-170418
- edit makefile in fh-lamp and set LUFA_PATH to LUFA subfolder of lufa sources
- make

## Flash
```
export DEV=<tty>
./kick2bl.py $DEV
avrdude -p atmega32u4 -P $DEV -c avr109 -v -b 57600 -D -U flash:w:FHLamp.hex:i
```
