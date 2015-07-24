//
// Copyright (c) 2015 Jon Escombe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef WEIGHFI_H
#define WEIGHFI_H

#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include "wd.h"
#include "i2cmaster.h"
#include "uart.h"
#include "lcd.h"
#include "eeprom.h"
#include "wlan.h"
#include "adc.h"
#include "terminal.h"
#include "lufa.h"
#include "Descriptors.h"

volatile uint16_t      g_ms;                                // Free running millisecond counter
volatile uint8_t       vibes;                               // Incremented by vibration switch interrupt

#define VIBRATION_THRESHOLD 5

#define VERSION_MAJOR 0
#define VERSION_MINOR 1

#define UART_BAUD_RATE 9600                                // Maybe dodgy with a 16MHz clock?

typedef enum {
    KILOS,
    POUNDS,
    STONES,
} DisplayUnits_t;

// Function Prototypes
unsigned int GetMilliSeconds(void);

#endif //WEIGHFI_H
