#!/bin/bash
# Shell script to set fuse bits and program arduino Bootloader in Avr Atmega328p
# Sets internal oscillator to 1MHz and serial uploads to 9600 baud.

"/Applications/Arduino 1.6.4.app/Contents/Java/hardware/tools/avr/bin/avrdude" -C/Applications/Arduino\ 1.6.4.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf -v -patmega328p -cusbtiny -e -Ulock:w:0x3F:m -Uefuse:w:0x06:m -Uhfuse:w:0xD4:m -Ulfuse:w:0x62:m && \
"/Applications/Arduino 1.6.4.app/Contents/Java/hardware/tools/avr/bin/avrdude" -C"/Applications/Arduino 1.6.4.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf" -v -patmega328p -cusbtiny -Uflash:w:./optiboot_atmega328_96_1.hex:i -Ulock:w:0x0F:m && \
# /Applications/Arduino\ 1.6.4.app/Contents/Java/hardware/tools/avr/bin/avrdude -C/Applications/Arduino\ 1.6.4.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf -v -patmega328p -cusbtiny -D -Uflash:w:./radio_receve_test_led.cpp.hex:i && \
say "Now insert next Atmega"
