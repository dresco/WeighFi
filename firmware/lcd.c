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

#include "WeighFi.h"

uint8_t LCDTranslateDigit(uint8_t ascii_char)
{
    uint8_t lcd_char;

    switch (ascii_char) {

        case '0':
            lcd_char = LCD_CHAR_0;
            break;

        case '1':
            lcd_char = LCD_CHAR_1;
            break;

        case '2':
            lcd_char = LCD_CHAR_2;
            break;

        case '3':
            lcd_char = LCD_CHAR_3;
            break;

        case '4':
            lcd_char = LCD_CHAR_4;
            break;

        case '5':
        case 'S':
            lcd_char = LCD_CHAR_5;
            break;

        case '6':
            lcd_char = LCD_CHAR_6;
            break;

        case '7':
            lcd_char = LCD_CHAR_7;
            break;

        case '8':
            lcd_char = LCD_CHAR_8;
            break;

        case '9':
            lcd_char = LCD_CHAR_9;
            break;

        case '-':
            lcd_char = LCD_CHAR_MINUS;
            break;

        case 'E':
            lcd_char = LCD_CHAR_E;
            break;

        case 'N':
            lcd_char = LCD_CHAR_N;
            break;

        case 'T':
            lcd_char = LCD_CHAR_T;
            break;

        case 'R':
            lcd_char = LCD_CHAR_R;
            break;

        default:
            lcd_char = LCD_CHAR_BLANK;
            break;
    }

    return(lcd_char);
}

void LCDEnable(uint8_t enable)
{
    if (enable)
    {
        // Clear PB0 to power up the LCD supply
        PORTB &= ~(1 << 0);

        // Wait 1ms before transferring data..
        _delay_ms(1);
    }
    else
    {
        // Set PB0 to power down the LCD
        PORTB |= (1 << 0);
    }
}

void LCDUpdate(DisplayData_t * DisplayData)
{
    i2c_init();                                             // Initialise I2C library
    i2c_start_wait(LCD_DEV_ID+I2C_WRITE);                   // Set device address and write mode

    //i2c_write(0xC8);                                      // Enable display + continuation bit
    i2c_write(0xD8);                                        // Enable display + low power + continuation bit

    if (DisplayData->Flags & LCD_FLAG_BLINK)
        i2c_write(0xF2);                                    // Set 1Hz blink mode + continuation bit
    else
        i2c_write(0xF0);                                    // Disable blink mode + continuation bit

    i2c_write(0x00);                                        // Set data pointer to 0, ready for data

    if (DisplayData->Flags & LCD_FLAG_BLANK)
    {
        for (int i = 0; i < 5; i++)                         // Blank the display
            i2c_write(0x00);
    }
    else if (DisplayData->Flags & LCD_FLAG_FILL)
    {
        for (int i = 0; i < 5; i++)                         // Turn on all segments
            i2c_write(0xFF);
    }
    else if (DisplayData->Flags & LCD_FLAG_DATA)
    {
        i2c_write(LCDTranslateDigit(DisplayData->Char1));

        i2c_write(LCDTranslateDigit(DisplayData->Char2) +
                ((DisplayData->Flags & LCD_FLAG_ST) ? LCD_CHAR_SEP : 0));

        i2c_write(LCDTranslateDigit(DisplayData->Char3) +
                (((DisplayData->Flags & LCD_FLAG_KG) ||
                  (DisplayData->Flags & LCD_FLAG_LB)) ? LCD_CHAR_SEP : 0));

        i2c_write(LCDTranslateDigit(DisplayData->Char4));

        if (DisplayData->Flags & LCD_FLAG_KG)
            i2c_write(LCD_CHAR_KG);
        else if (DisplayData->Flags & LCD_FLAG_LB)
            i2c_write(LCD_CHAR_LB);
        else if (DisplayData->Flags & LCD_FLAG_ST)
            i2c_write(LCD_CHAR_ST);
        else
            i2c_write(LCD_CHAR_BLANK);
    }

    i2c_stop();                                             // Set stop condition = release bus
}

