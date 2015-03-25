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
//
//
// Using alternate watchdog routines in wd.h to enable watchdog interrupt wakes
//
//
// ADC
// ---
// TI ADS1131
//  - 18-bit delta sigma ADC
//
// Bit banging the SPI interface. Is slightly odd as there is no MOSI, and the
// ADC uses MISO to signify conversion data is ready, as well as for data..
//
// LCD
// ---
// NXP PCF8566 LCD driver IC
//  - 1:4 multiplex mode, 1/3 bias configuration
//  - 4 backplane lines, 9 segment lines
//
// Using the Peter Fleury I2C library
//
// WIFI
// ----
//
// Using an ESP8266 wireless module, connected to UART0
//
// Using the Peter Fleury uart library
//
// Pin assignments
// ---------------
// PB0 - output - LCD enable
// PD0 -        - I2C clock
// PD1 -        - I2C data
// PD2 -        - UART
// PD3 -        - UART
// PD4 - output - ADC speed control
// PD5 - output - ADC power control
// PD6 - output - SPI serial clock
// PD7 - input  - SPI data in (MISO)
// PF0 - output - WLAN module enable
// PF7 - output - debug LED
//

#include "WeighFi.h"

// LUFA CDC Class driver interface configuration and state information. This structure is
// passed to all CDC Class driver functions, so that multiple instances of the same class
// within a device can be differentiated from one another.
//
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
    .Config =
    {
        .ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
        .DataINEndpoint           =
        {
            .Address          = CDC_TX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
        },
        .DataOUTEndpoint =
        {
            .Address          = CDC_RX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
        },
        .NotificationEndpoint =
        {
            .Address          = CDC_NOTIFICATION_EPADDR,
            .Size             = CDC_NOTIFICATION_EPSIZE,
            .Banks            = 1,
        },
    },
};

// Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
// used like any regular character stream in the C APIs.
static FILE USBSerialStream;

ISR (WDT_vect)
{
}

ISR(TIMER1_COMPA_vect)
{
    g_ms++;
    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7
}

unsigned int GetMilliSeconds(void)
{
    return g_ms;
}

int UART_ReceiveLine(unsigned char * buffer, unsigned int length, unsigned int timeout)
{
    // Try to read a line from the UART, ending with an ASCII newline character

    unsigned int c;
    unsigned char bufpos = 0;
    unsigned int start = GetMilliSeconds();

    while (GetMilliSeconds() - start < timeout)
    {
        // Get received character from ringbuffer
        // uart_getc() returns in the lower byte the received character and
        // in the higher byte (bitmask) the last receive error
        // UART_NO_DATA is returned when no data is available.
        c = uart_getc();

        if ( c & UART_NO_DATA )
        {
            // no data available from UART
        }
        else
        {
            // new data available from UART
            // check for Frame or Overrun error
            if ( c & UART_FRAME_ERROR )
            {
                // Framing Error detected, i.e no stop bit detected

                //uart_puts("UART Frame Error: ");
            }
            if ( c & UART_OVERRUN_ERROR )
            {
                // Overrun, a character already present in the UART UDR register was
                // not read by the interrupt handler before the next character arrived,
                // one or more received characters have been dropped

                //uart_puts("UART Overrun Error: ");
            }
            if ( c & UART_BUFFER_OVERFLOW )
            {
                // We are not reading the receive buffer fast enough,
                // one or more received character have been dropped

                //uart_puts("Buffer overflow error: ");
            }

            // Add received character to local buffer if not full
            if (bufpos < length)
            {
                buffer[bufpos++] = (unsigned char)c;
                if (bufpos == length)
                {
                    // null terminate the final character
                    // in case we try to read as a string later
                    buffer[bufpos] = 0x00;
                }
            }

            // If received new line ASCII char
            if ((unsigned char)c == 0x0A)
            {
                // write no. of chars in buffer
                //uart_putc(bufpos);

                // if buffer is not already full, add a null  terminator
                // in case we try to read as a string later
                if (bufpos < length)
                {
                    buffer[bufpos+1] = 0x00;
                }

                return(bufpos);
            }
        }
    }
    return 0;
}

int UART_ReceiveDataPrompt(unsigned char * buffer, unsigned int length, unsigned int timeout)
{
    // Try to read a line from the UART, ending with an ASCII newline character

    unsigned int c;
    unsigned char bufpos = 0;
    unsigned int start = GetMilliSeconds();

    while (GetMilliSeconds() - start < timeout)
    {
        // Get received character from ringbuffer
        // uart_getc() returns in the lower byte the received character and
        // in the higher byte (bitmask) the last receive error
        // UART_NO_DATA is returned when no data is available.
        c = uart_getc();

        if ( c & UART_NO_DATA )
        {
            // no data available from UART
        }
        else
        {
            // new data available from UART
            // check for Frame or Overrun error
            if ( c & UART_FRAME_ERROR )
            {
                // Framing Error detected, i.e no stop bit detected

                //uart_puts("UART Frame Error: ");
            }
            if ( c & UART_OVERRUN_ERROR )
            {
                // Overrun, a character already present in the UART UDR register was
                // not read by the interrupt handler before the next character arrived,
                // one or more received characters have been dropped

                //uart_puts("UART Overrun Error: ");
            }
            if ( c & UART_BUFFER_OVERFLOW )
            {
                // We are not reading the receive buffer fast enough,
                // one or more received character have been dropped

                //uart_puts("Buffer overflow error: ");
            }

            // Add received character to local buffer if not full
            if (bufpos < length)
            {
                buffer[bufpos++] = (unsigned char)c;
                if (bufpos == length)
                {
                    // null terminate the final character
                    // in case we try to read as a string later
                    buffer[bufpos] = 0x00;
                }
            }

            // If received data prompt
            if ((buffer[0] == '>') && buffer[1] == ' ')
            {
                // write no. of chars in buffer
                //uart_putc(bufpos);

                // if buffer is not already full, add a null  terminator
                // in case we try to read as a string later
                if (bufpos < length)
                {
                    buffer[bufpos+1] = 0x00;
                }

                return(bufpos);
            }
        }
    }
    return 0;
}


int UART_ReceiveChar(unsigned char * character, unsigned int timeout)
{
    // Try to read a single character from the UART

    unsigned int c;
    unsigned int start = GetMilliSeconds();

    while (GetMilliSeconds() - start < timeout)
    {
        // Get received character from ringbuffer
        // uart_getc() returns in the lower byte the received character and
        // in the higher byte (bitmask) the last receive error
        // UART_NO_DATA is returned when no data is available.
        c = uart_getc();

        if ( c & UART_NO_DATA )
        {
            // no data available from UART
        }
        else
        {
            // new data available from UART
            // check for Frame or Overrun error
            if ( c & UART_FRAME_ERROR )
            {
                // Framing Error detected, i.e no stop bit detected

                //uart_puts("UART Frame Error: ");
            }
            if ( c & UART_OVERRUN_ERROR )
            {
                // Overrun, a character already present in the UART UDR register was
                // not read by the interrupt handler before the next character arrived,
                // one or more received characters have been dropped

                //uart_puts("UART Overrun Error: ");
            }
            if ( c & UART_BUFFER_OVERFLOW )
            {
                // We are not reading the receive buffer fast enough,
                // one or more received character have been dropped

                //uart_puts("Buffer overflow error: ");
            }

            // Return received character
            character[0] = (unsigned char)c;
            return 1;
        }
    }
    return 0;
}

void TimerSetup(void)
{
    TCCR1B |= (1 << WGM12);                     // Configure 16bit timer1 for CTC mode 4
    TIMSK1 |= (1 << OCIE1A);                    // Enable CTC interrupt
    OCR1A = 250;                                // Set output compare value
    TCCR1B |= (1 << CS11) | (1 << CS10);        // Start timer with x64 prescaler for a 1MHz timer
}

void WLANEnable(int enable)
{
    if (enable)
        PORTF |= (1 << 0);                      // Set F0 to enable WLAN module
    else
        PORTF &= ~(1 << 0);                     // Clear F0 to disable WLAN module
}

uint8_t WLANTransmit(int32_t Weight, char * SiteKey, char * DeviceID)
{
    WLANState_t state = INIT;

    unsigned char buff[NETWORK_BUFLEN];
    unsigned int start;

    char weight_str[8];
    char wlan_tx_data[80];
    char wlan_tx_data_len[8];

    //ltoa(Weight, wlan_tx_data, 10);
    //ltoa(strlen(wlan_tx_data)+2, wlan_tx_data_len, 10); //crlf

    while (1)
    {

        switch (state) {

            case INIT:

                // Turn on the wireless module
                WLANEnable(1);

                // Wait for the ready prompt
                // max wait of 3 seconds before error
                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        if (strstr((char*)buff, "ready"))
                        {
                            state = READY;
                            break;                          // note: breaks out of while loop only
                        }
                    }
                }

                // If we got here without setting the state to READY, then error.
                if (state != READY)
                    state = ERROR;

                break;

            case READY:
                uart_puts("AT\r\n");

                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        if (strstr((char*)buff, "OK"))
                        {
                            state = OKAY;
                            break;
                        }
                    }
                }

                // If we got here without setting the state to ONLINE, then error.
                if (state != OKAY)
                    state = ERROR;


                break;

            case OKAY:

#ifdef ONETIME
                // client mode
                uart_puts("AT+CWMODE=1\r\n");
                start = MilliSeconds();
                while (MilliSeconds() - start < 3000)
                {
                    PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                    }
                }

                // join ssid
                uart_puts("AT+CWJAP=\"SSID\",\"PASSWORD\"\r\n");
                start = MilliSeconds();
                while (MilliSeconds() - start < 3000)
                {
                    PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                    }
                }

                // reset
                uart_puts("AT+RST\r\n");
                start = MilliSeconds();
                while (MilliSeconds() - start < 3000)
                {
                    PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                    }
                }

                state = ERROR;
                break;
#endif


                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 5000)
                {
                    uart_puts("AT+CIFSR\r\n");

                    // have to read multiple lines for address
                    for (int i = 0; i < 3 ; i++)
                    {
                        if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                        {

                            //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                            // Things we might get back..
                            //
                            // ERROR
                            // Error
                            // busy now ...
                            // <ip addr>
                            //
                            // Just looking for a dot in the IP address for now - but watch out for "busy now ..."

                            if (strstr((char*)buff, "."))
                            {
                                if (!strstr((char*)buff, "busy"))
                                {
                                    state = ONLINE;
                                    break;                      // note: breaks out of for loop only
                                }
                            }
                        }
                    }

                    if (state == ONLINE)
                        break;          // note: break out of while loop

                    _delay_ms(200);
                }

                // If we got here without setting the state to ONLINE, then error.
                if (state != ONLINE)
                    state = ERROR;
                break;

            case ONLINE:

                // Try to establish a TCP connection to server
                //uart_puts("AT+CIPSTART=\"TCP\",\"192.168.100.20\",80\r\n");
                uart_puts("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");

                // max wait of 3 seconds before error
                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        if (strstr((char*)buff, "Linked"))
                        {
                            state = CONNECTED;
                            break;                          // note: breaks out of while loop only
                        }
                    }
                }

                // If we got here without setting the state to CONNECTED, then error.
                if (state != CONNECTED)
                    state = ERROR;

                break;

            case CONNECTING:    // not using yet
                break;

            case CONNECTED:

                // construct the data to send
                ltoa(Weight, weight_str, 10);

                memset(wlan_tx_data, 0x00, sizeof(wlan_tx_data));
                strcat(wlan_tx_data, "GET /update?key=");
                strcat(wlan_tx_data, SiteKey);
                strcat(wlan_tx_data, "&");
                strcat(wlan_tx_data, DeviceID);
                strcat(wlan_tx_data, "=");
                strcat(wlan_tx_data, weight_str);
                strcat(wlan_tx_data, "\n\r");

                ltoa(strlen(wlan_tx_data), wlan_tx_data_len, 10);

                // prepare to send some data
                uart_puts("AT+CIPSEND=");
                uart_puts(wlan_tx_data_len);
                uart_puts("\r\n");

                // max wait of 3 seconds before error
                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        if (strstr((char*)buff, "AT+CIPSEND"))
                        {
                            // send command echoed back, next data should
                            // be "> " with no new line termination
                            if (UART_ReceiveDataPrompt(buff, NETWORK_BUFLEN, 100))
                            {
                                uart_puts(wlan_tx_data);

                                // max wait of 3 seconds before error
                                start = GetMilliSeconds();
                                while (GetMilliSeconds() - start < 3000)
                                {
                                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                                    {
                                        if (strstr((char*)buff, "SEND OK"))
                                        {
                                            break;                          // note: breaks out of while loop only
                                        }
                                    }
                                }
                                break;                          // note: breaks out of while loop only
                            }
                        }
                    }
                }

                state = DISCONNECTING;
                break;

            case DISCONNECTING:

                // Close TCP connection
                uart_puts("AT+CIPCLOSE\r\n");

                // max wait of 3 seconds before error
                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        if (strstr((char*)buff, "Unlink"))  // note: sometimes see "Linked" instead?
                        {
                            state = DISCONNECTED;
                            break;                          // note: breaks out of while loop only
                        }
                    }
                }

                // If we got here without setting the state to CONNECTED, then error.
                if (state != DISCONNECTED)
                    state = ERROR;

                break;

            case DISCONNECTED:
                state = DONE;
                break;

            case DONE:
                // Turn off wireless module
                WLANEnable(0);
                return(1);
                break;

            case ERROR:
                //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7
                WLANEnable(0); // do nothing further
                return(0);
                break;

            default:
                break;

        }
    }
}

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

int32_t GetADCValue(uint8_t ADCHighSpeed, uint8_t NumSamples)
{
    int32_t adc_value;                                      // signed 32 bit integer for ADC output
    int32_t adc_final;
    int64_t adc_total;
    int samples, i, bit;

    if (ADCHighSpeed)
        PORTD |= (1 << 4);                                  // Set PD4 for 80 samples per second
    else
        PORTD &= ~(1 << 4);                                 // Clear PD4 for 10 samples per second


    PORTD |= (1 << 5);                                      // Set PD5 to power up ADC

    adc_total = 0;
    for (samples=0; samples < NumSamples; samples++)
    {
        adc_value = 0x00;                                   // Zero out the previous ADC reading

        while((PIND & (1<<7)))
            ;                                               // Wait for PD7 to go low (data ready to be read)

        // datasheet specs 100ns minimum negative or positive pulse width,
        // asm volatile ("nop"); would give 62.5ns @ 16MHz
        // (as cycle time on a 16MHz processor is 62.5ns)
        //
        // so for each bit -
        // - set serial clock high
        // - wait 100ns+
        // - read bit
        // - serial clock low
        // - wait 100ns+

        //
        // slacker approach would be to just shift the most significant
        // 16 bits into a signed 16bit int and ignore the last 2 bits
        //
        // for full resolution, we shift all 18bits into a 32bit signed
        // int & extend the sign appropriately
        //
        // sign extension is just filling the additional high bits with the
        // same value as the sign bit. For our 18bit signed int, 0x20000
        // would be the sign bit, so if this is set OR the result with
        // 0xFFFC0000
        //

        for (i=0; i<ADC_DATA_BITS; i++)
        {
            PORTD |= (1 << 6);                              // Set PD6 - drive serial clock high
            asm volatile ("nop");
            asm volatile ("nop");

            PORTD &= ~(1 << 6);                             // Clear PD6 - drive serial clock low

            if (PIND & (1 << 7))                            // Read PD7 (SPI data in)
                bit = 1;
            else
                bit = 0;

            adc_value = (adc_value << 1) + bit;             // Left shift the existing value and add the new LSB
                                                            //  - ok for the 1st bit as initial value is all zero
            asm volatile ("nop");
            asm volatile ("nop");
        }

        //
        // Generate whatever additional serial clocks are required
        // - to read out any data bits we're ignoring
        // - to ensure the data line is left high (+1)
        //
        for (i=0; i<ADC_SPARE_BITS+1; i++)
        {
            PORTD |= (1 << 6);                              // Set PD6 - drive serial clock high
            asm volatile ("nop");
            asm volatile ("nop");
            PORTD &= ~(1 << 6);                             // Clear PD6 - drive serial clock low
            asm volatile ("nop");
            asm volatile ("nop");
        }

        // Perform the sign extension on the final result
        // (see fuller description above)
        if (adc_value & 0x20000)
            adc_value |= 0xFFFC0000;

        // Add the current value to the running total for the average
        adc_total+= adc_value;
    }

    PORTD &= ~(1 << 5);                                     // Clear PD5 to power down ADC

    adc_final = adc_total / NumSamples;                     // average the ADC readings
    return(adc_final);
}

void PortSetup(void)
{
    DDRB |= (1 << 0);                                       // Configure PB0 as output for LCD power supply
    PORTB |= (1 << 0);                                      // Set PB0 to ensure LCD is powered down

    DDRD |= (1 << 4);                                       // Configure PD4 as output to control the ADC speed
    PORTD |= (1 << 4);                                      // Set PD4 for 80 samples per second

    DDRD |= (1 << 5);                                       // Configure PD5 as output to enable the ADC
    PORTD &= ~(1 << 5);                                     // Clear PD5 to ensure ADC is powered down

    DDRD |= (1 << 6);                                       // Configure PD6 as output for SPI SCK

    DDRF |= (1 << 0);                                       // Configure PF0 as output to enable WLAN module

    DDRF |= (1 << 7);                                       // Configure PF7 as output for debug LED
}

void SetupHardware(void)
{
    // Disable watchdog if enabled by bootloader/fuses
    MCUSR &= ~(1 << WDRF);
    WD_DISABLE();

    // Disable clock division
    clock_prescale_set(clock_div_1);

    // Hardware Initialization
    LEDs_Init();
    USB_Init();

    PortSetup();
    TimerSetup();

    LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);

    WD_SET(WD_IRQ,WDTO_1S);                     // Setup Watchdog interrupt for 1 second interval

    GlobalInterruptEnable();

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);        // Set POWER DOWN sleep mode

}

int32_t DivideAndRoundToClosest(const int32_t n, const int32_t d)
{
    return ((n < 0) ^ (d < 0)) ? ((n - d/2)/d) : ((n + d/2)/d);
}


void PrepareDisplayData(int32_t Weight, DisplayUnits_t DisplayUnits, DisplayData_t *DisplayData)
{
    char DisplayString[6];

    memset(DisplayData, 0x00, sizeof(DisplayData_t));

    // Routines for converting to appropriate units for display on LCD
    // Rounding up/down where appropriate..
    if (DisplayUnits == KILOS)
    {
        // Written to the LCD in units of 100 grams,
        // display routine will insert the decimal point
        int tenthkilos = DivideAndRoundToClosest(Weight, 100);

        if (USB_DeviceState == DEVICE_STATE_Configured)
            fprintf(&USBSerialStream, "grams: %ld, tenthkilos: %d\n\r", Weight, tenthkilos);

        DisplayData->Flags |= LCD_FLAG_KG;
        sprintf(DisplayString, "%4d", tenthkilos);
    }
    else if (DisplayUnits == POUNDS)
    {
        // Written to the LCD in units of 1/10th of a pound,
        // display routine will insert the decimal point
        int tenthpounds = DivideAndRoundToClosest(Weight*100, 4536);

        if (USB_DeviceState == DEVICE_STATE_Configured)
            fprintf(&USBSerialStream, "grams: %ld, tenthpounds: %d\n\r", Weight, tenthpounds);

        DisplayData->Flags |= LCD_FLAG_LB;
        sprintf(DisplayString, "%4d", tenthpounds);
    }
    else if (DisplayUnits == STONES)
    {
        // Written to the LCD in stones and whole pounds
        int tenthpounds = DivideAndRoundToClosest(Weight*100, 4536);
        int stones = tenthpounds / 140;             // must round down
        int remainder = tenthpounds % 140;
        int pounds = DivideAndRoundToClosest(remainder, 10);

        if (USB_DeviceState == DEVICE_STATE_Configured)
            fprintf(&USBSerialStream, "tenthpounds: %d, stones: %d, remainder %d, pounds: %d\n\r",
                    tenthpounds, stones, remainder, pounds);

        DisplayData->Flags |= LCD_FLAG_ST;
        sprintf(DisplayString, "%2d%02d", stones, pounds);
    }

    DisplayData->Flags |= LCD_FLAG_DATA;
    DisplayData->Char1 = DisplayString[0];
    DisplayData->Char2 = DisplayString[1];
    DisplayData->Char3 = DisplayString[2];
    DisplayData->Char4 = DisplayString[3];

    // As we're not using printf to insert the decimal point and handle any leading zeros, we need
    // to take care of the situation where we have just the decimal with no preceding characters
    if (DisplayUnits == KILOS || DisplayUnits == POUNDS)
    {
        // Positive
        if (DisplayData->Char1 == ' ' && DisplayData->Char2 == ' ' && DisplayData->Char3 == ' ')
        {
            DisplayData->Char3 = '0';
        }
        // Negative
        if (DisplayData->Char1 == ' ' && DisplayData->Char2 == ' ' && DisplayData->Char3 == '-')
        {
            DisplayData->Char3 = '0';
            DisplayData->Char2 = '-';
        }
    }
}

int32_t WeighAndDisplay(EEPROMData_t * EEPROMData)
{
    DisplayUnits_t DisplayUnits = KILOS;
    DisplayData_t DisplayData = {0};

    int32_t Weight;
    int32_t ADCResult;
    int32_t ADCDelta;
    int32_t ADCZeroReading;
    int32_t ADCLastResult = 0;

    // Power up the LCD
    LCDEnable(1);

    // Fill the display and pause for 1 second
    DisplayData.Flags = LCD_FLAG_FILL;
    LCDUpdate(&DisplayData);
    _delay_ms(1000);

    // Get a more accurate reading from the ADC, configured for low speed and averaging multiple
    // readings for accuracy. Also waiting for the reading to stabilise before accepting it.
    for (int i = 0 ; i < ADC_MAX_RETRIES ; i++)
    {
        ADCResult = GetADCValue(ADC_SPEED_LOW, 3);
        if (abs(ADCResult - ADCLastResult) < ADC_STABLE_THRESHOLD)
            break;
        ADCLastResult = ADCResult;
    }

    // Only set a new zero reading if close enough to previous zero reading, else might be weighted already
    // if (abs(ADCResult - ADCZeroReading) < ADC_WAKE_THRESHOLD)
    //     ADCZeroReading = ADCResult;

    ADCZeroReading = ADCResult;
    Weight = 0;

    // Format and display the zero weight
    PrepareDisplayData(Weight, DisplayUnits, &DisplayData);
    LCDUpdate(&DisplayData);
    _delay_ms(2000);

    // Get a more accurate reading from the ADC, configured for low speed and averaging multiple
    // readings for accuracy. Also waiting for the reading to stabilise before accepting it.
    // todo: probably ought to filter out negative weight values
    for (int i = 0 ; i < ADC_MAX_RETRIES ; i++)
    {
        ADCResult = GetADCValue(ADC_SPEED_LOW, 3);

        // Calculate the weight in grams
        ADCDelta = ADCResult - ADCZeroReading;
        Weight = DivideAndRoundToClosest((ADCDelta * 1000), ADC_COUNTS_PER_KG);

        // Format and display the current weight
        PrepareDisplayData(Weight, DisplayUnits, &DisplayData);
        LCDUpdate(&DisplayData);

        // If weight is stable AND above wakeup threshold (i.e. actually weighing something)
        if ((abs(ADCResult - ADCLastResult) < ADC_STABLE_THRESHOLD) && (abs(ADCResult - ADCZeroReading) > ADC_WAKE_THRESHOLD))
            break;

        ADCLastResult = ADCResult;
    }

    // Weight has settled, blink the display and pause for 5 seconds
    PrepareDisplayData(Weight, DisplayUnits, &DisplayData);
    DisplayData.Flags |= LCD_FLAG_BLINK;
    LCDUpdate(&DisplayData);
    _delay_ms(5000);

    // todo: upload weight data to network if not 0.0
    WLANTransmit(Weight, (char *)EEPROMData->SRAM_SiteKey, (char *)EEPROMData->SRAM_DeviceID);

    // Blank the display
    //DisplayData.Flags = LCD_FLAG_BLANK;
    //LCDUpdate(&DisplayData);

    // Power down the LCD
    LCDEnable(0);

    return(Weight);
}

void FetchEEPROMData(EEPROMData_t *EEPROMData)
{
    EEPROMData->SRAM_VersionMajor = eeprom_read_byte(&EEPROM_VersionMajor);
    EEPROMData->SRAM_VersionMinor = eeprom_read_byte(&EEPROM_VersionMinor);

    EEPROMData->SRAM_Calibration = eeprom_read_byte(&EEPROM_Calibration);

    eeprom_read_block((void*)EEPROMData->SRAM_SiteID, (const void *)EEPROM_SiteID, 36);
    eeprom_read_block((void*)EEPROMData->SRAM_SiteKey, (const void *)EEPROM_SiteKey, 36);
    eeprom_read_block((void*)EEPROMData->SRAM_DeviceID, (const void *)EEPROM_DeviceID, 36);
    eeprom_read_block((void*)EEPROMData->SRAM_Reserved, (const void *)EEPROM_Reserved, 36);

    eeprom_read_block((void*)EEPROMData->SRAM_WiFi_SSID, (const void *)EEPROM_WiFi_SSID, 32);
    eeprom_read_block((void*)EEPROMData->SRAM_WiFi_PASS, (const void *)EEPROM_WiFi_PASS, 64);
}

void UpdateEEPROMData(EEPROMData_t *EEPROMData)
{
    eeprom_update_byte(&EEPROM_VersionMajor, EEPROMData->SRAM_VersionMajor);
    eeprom_update_byte(&EEPROM_VersionMinor, EEPROMData->SRAM_VersionMinor);

    eeprom_update_byte(&EEPROM_Calibration, EEPROMData->SRAM_Calibration);

    eeprom_update_block((const void *)EEPROMData->SRAM_SiteID, (void *)EEPROM_SiteID, 36);
    eeprom_update_block((const void *)EEPROMData->SRAM_SiteKey, (void *)EEPROM_SiteKey, 36);
    eeprom_update_block((const void *)EEPROMData->SRAM_DeviceID, (void *)EEPROM_DeviceID, 36);
    eeprom_update_block((const void *)EEPROMData->SRAM_Reserved, (void *)EEPROM_Reserved, 36);

    eeprom_update_block((const void *)EEPROMData->SRAM_WiFi_SSID, (void *)EEPROM_WiFi_SSID, 32);
    eeprom_update_block((const void *)EEPROMData->SRAM_WiFi_PASS, (void *)EEPROM_WiFi_PASS, 64);
}

// Event handler for the library USB Connection event.
void EVENT_USB_Device_Connect(void)
{
    LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

// Event handler for the library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void)
{
    LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

// Event handler for the library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

    LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

// Event handler for the library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int main(void)
{
    SystemState_t SystemState = IDLE;
    EEPROMData_t  EEPROMData = {0};

    //int32_t ADCZeroReading;
    int32_t Weight;
    int32_t ADCResult = 0;
    int32_t ADCInitialResult;
    int32_t ADCLastResult;

    SetupHardware();

    // one-off update of set some EEPROM defaults
    //EEPROMData.SRAM_VersionMajor = 1;
    //EEPROMData.SRAM_VersionMinor = 0;
    //EEPROMData.SRAM_Calibration = 140;
    //strcpy((char*)EEPROMData.SRAM_SiteKey,"xxxxxxxxxxxx");
    //strcpy((char*)EEPROMData.SRAM_DeviceID,"field1");
    //UpdateEEPROMData(&EEPROMData);

    // Get default settings from EEPROM
    FetchEEPROMData(&EEPROMData);

    // Initialise the serial library
    // todo: port to LUFA serial library
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

    // Create a regular character stream for the interface so that it can be used with the stdio.h functions
    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    // Blank the LCD display
    //DisplayData.Flags = LCD_FLAG_BLANK;
    //LCDUpdate(&DisplayData);

    // Get a one off reading to populate the variables before fist use..
    ADCInitialResult = ADCLastResult = GetADCValue(ADC_SPEED_HIGH, 1);

    while (1)
    {
        //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

        //
        // Note that the weighing and displaying process is synchronous (and slow).
        // This would cause problems for any active USB connection, as our LUFA
        // implementation is polling, so we can't block in the main program loop.
        // todo: move LUFA processing to timer interrupts?
        //
        if (USB_DeviceState == DEVICE_STATE_Unattached)
        {
            // Not connected to USB host (normal state of operation), so see
            // whether we are being woken up for a measurement

            // Perform a quick ADC reading for comparison
            ADCResult = GetADCValue(ADC_SPEED_HIGH, 1);

            // If we have previously been weighing, don't start again until weight removed from scales.
            // Using the 'initial' zero value in case the 'accurate' zero was set incorrectly due to
            // already weighted scales (which would make it difficult to change back to idle state).
            if ((SystemState == WEIGHING) && (abs(ADCResult - ADCLastResult) < ADC_WAKE_THRESHOLD))
            {
                ADCLastResult = ADCResult;
                SystemState = IDLE;
            }

            if ((abs(ADCResult - ADCLastResult) > ADC_WAKE_THRESHOLD) && (SystemState == IDLE))
            {
                // Seems that we've been prodded to wake us
                SystemState = WEIGHING;

                Weight = WeighAndDisplay(&EEPROMData);
            }

            // Save current reading for next comparison
            // ADCLastResult = ADCResult;

            // Only set a new zero reading if close enough to previous zero reading, else might be weighted already
            if (abs(ADCResult - ADCLastResult) < ADC_WAKE_THRESHOLD)
                ADCLastResult = ADCResult;

        }

        // Wait for a character from the host, and send back an ADC result
        // fixme: does this block if connected ???
        int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

        if (!(ReceivedByte < 0))
        {
            // Debug dump of EEPROM settings to USB output
            if (USB_DeviceState == DEVICE_STATE_Configured)
            {
                fprintf(&USBSerialStream, "Version Major: %d\n\r", EEPROMData.SRAM_VersionMajor);
                fprintf(&USBSerialStream, "Version Minor: %d\n\r", EEPROMData.SRAM_VersionMinor);
                fprintf(&USBSerialStream, "Calibration  : %d\n\r", EEPROMData.SRAM_Calibration);
                fprintf(&USBSerialStream, "Site ID      : %s\n\r", EEPROMData.SRAM_SiteID);
                fprintf(&USBSerialStream, "Site Key     : %s\n\r", EEPROMData.SRAM_SiteKey);
                fprintf(&USBSerialStream, "Device ID    : %s\n\r", EEPROMData.SRAM_DeviceID);
                fprintf(&USBSerialStream, "Network      : %s\n\r", EEPROMData.SRAM_WiFi_SSID);
                fprintf(&USBSerialStream, "Passphrase   : %s\n\r", EEPROMData.SRAM_WiFi_PASS);
            }

            fprintf(&USBSerialStream, "Free RAM     : %d\n\r", freeRam());

            fprintf(&USBSerialStream, "ms           : %u\n\r", GetMilliSeconds());

            // todo: add a simple terminal interface to manage EEPROM configuration data

            // Get a reading from the ADC, configured for low speed and
            // averaging multiple readings for accuracy
            int32_t ADCResult = GetADCValue(ADC_SPEED_LOW, 3);
            //int32_t ADCDelta = ADCResult - ADCZeroReading;

            // Write the raw ADC value to USB output if connected
            if (USB_DeviceState == DEVICE_STATE_Configured)
                fprintf(&USBSerialStream, "ADCResult    : %ld\n\r", ADCResult);

            //WLANTransmit(ADCResult);

            // Write the raw ADC difference to USB output if connected
            //if (USB_DeviceState == DEVICE_STATE_Configured)
            //    fprintf(&USBSerialStream, "%ld\n\r", ADCDelta);

            // Calculate the weight in grams
            //Weight = DivideAndRoundToClosest((ADCDelta * 1000), ADC_COUNTS_PER_KG);

            // Format and display the current weight
            //PrepareDisplayData(Weight, DisplayUnits, &DisplayData);
            //LCDUpdate(&DisplayData);
        }

        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();

        // Sleep if not USB connected, woken each second by watchdog interrupt
        if (USB_DeviceState == DEVICE_STATE_Unattached)
            sleep_mode();
    }
}
