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

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

#include "i2cmaster.h"
#include "Descriptors.h"

#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#define ADC_DATA_BITS 18
#define ADC_SPARE_BITS 1

#define ADC_SPEED_LOW 0
#define ADC_SPEED_HIGH 1

#define LCD_DEV_ID     0x7C                                 // Device address of LCD driver, see datasheet

#define LCD_CHAR_0     0b11011110
#define LCD_CHAR_1     0b00000110
#define LCD_CHAR_2     0b01111010
#define LCD_CHAR_3     0b00111110
#define LCD_CHAR_4     0b10100110
#define LCD_CHAR_5     0b10111100
#define LCD_CHAR_6     0b11111100
#define LCD_CHAR_7     0b00001110
#define LCD_CHAR_8     0b11111110
#define LCD_CHAR_9     0b10101110
#define LCD_CHAR_BLANK 0b00000000
#define LCD_CHAR_SEP   0b00000001
#define LCD_CHAR_KG    0b10000000
#define LCD_CHAR_LB    0b00100000
#define LCD_CHAR_ST    0b01000000

// LUFA LED support macros
#define LEDMASK_USB_NOTREADY     LEDS_LED1                  // Interface not ready
#define LEDMASK_USB_ENUMERATING (LEDS_LED2 | LEDS_LED3)     // Interface is enumerating
#define LEDMASK_USB_READY       (LEDS_LED2 | LEDS_LED4)     // Interface is ready
#define LEDMASK_USB_ERROR       (LEDS_LED1 | LEDS_LED3)     // Error has occurred

// LUFA Function Prototypes
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);
