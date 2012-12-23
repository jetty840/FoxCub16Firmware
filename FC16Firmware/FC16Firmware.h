// Fox Cub 16 Firmware for Atmega328P-PU
//
// Copyright: Jetty 2012, V0.9Alpha
// License: Creative Commons - Non Commercial - Attribution - ShareAlike
// http://www.creativecommons.org/
//
// DISCLAIMER: USE THIS SOFTWARE AT YOUR OWN RISK AND YOUR OWN LIABILITIY.
// IF YOU DON'T UNDERSTAND WHAT YOU'RE DOING WITH ELECTRONICS AND POWER, GET PROFESSIONAL ADVICE.
// IF YOU'RE USING 240V, YOU WILL NEED TO CHANGE THE CIRCUIT, FIRMWARE AND APPROACH.  240V CONTAINS
// CONSIDERABLY MORE ENERGY THAN 110V.

//
// You likely don't need to change anything in here
//

// Timings for the AC cycle for 50Hz and 60Hz
// Note: Calculate by twice the power frequency (as sine wave has 2 halfs
// (neg and pos) per cycle, because we zero cross on both halfs.
// Example:   (16000000 (F_CPU) / (((2 * 60Hz) * 255(brightness levels))) - 1 = 521.87581
// then round up to the nearest integer = 522
#define FREQ_50HZ_TIMER 627
#define FREQ_60HZ_TIMER 522

// Defines for Arduino pins
#define D0  0
#define D1  1
#define D2  2
#define D3  3
#define D4  4
#define D5  5
#define D6  6
#define D7  7
#define D8  8
#define D9  9
#define D10 10
#define D11 11
#define D12 12
#define D13 13
/*
A0-A5 already defined in pins_arduino.h in the IDE, no need to redefine here
#define A0  14
#define A1  15
#define A2  16
#define A3  17
#define A4  18
#define A5  19
*/
