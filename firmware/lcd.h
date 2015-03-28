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

#ifndef LCD_H
#define LCD_H

#define LCD_DEV_ID     0x7C                                 // Device address of LCD driver, see datasheet

#define LCD_FLAG_KG    0b00000001
#define LCD_FLAG_LB    0b00000010
#define LCD_FLAG_ST    0b00000100
#define LCD_FLAG_DATA  0b00001000
#define LCD_FLAG_BLANK 0b00010000
#define LCD_FLAG_FILL  0b00100000
#define LCD_FLAG_BLINK 0b01000000

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
#define LCD_CHAR_MINUS 0b00100000

#define LCD_CHAR_E     0b11111000
#define LCD_CHAR_N     0b01100100
#define LCD_CHAR_T     0b11110000
#define LCD_CHAR_R     0b01100000

typedef struct DisplayData
{
    uint8_t Char1;
    uint8_t Char2;
    uint8_t Char3;
    uint8_t Char4;
    uint8_t Char5;                                          // May not need this one if units are defined in flags
    uint8_t Flags;
} DisplayData_t;

// Function definitions
extern uint8_t LCDTranslateDigit(uint8_t);
extern void LCDEnable(uint8_t);
extern void LCDUpdate(DisplayData_t *);

#endif //LCD_H
