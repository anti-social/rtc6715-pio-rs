# RTC 6705/6715 rust embassy driver for Raspberrypi pico

RTC 6705/6715 are vtx and vrx devices that use a non-standard spi protocol.
The protocol uses 5 bits for a command and 20 bits for data (both for reading and writing).

This library implements the protocol utilizing Raspberry pico state machine
and their pio assembler.

