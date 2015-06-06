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

int UART_ReceiveLine(unsigned char * buffer, unsigned int length, unsigned int timeout)
{
    // Try to read a line from the UART, ending with an ASCII newline character

    unsigned int c;
    unsigned char bufpos = 0;
    unsigned int start = GetMilliSeconds();

    memset(buffer, 0x00, length);

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
            }

            // If received new line ASCII char
            if ((unsigned char)c == 0x0A)
            {
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

void WLANEnable(int enable)
{
    if (enable)
        PORTF |= (1 << 0);                      // Set F0 to enable WLAN module
    else
        PORTF &= ~(1 << 0);                     // Clear F0 to disable WLAN module
}

uint8_t WLANConfigure(char * ssid, char * passphrase)
{
    WLANState_t state = INIT;
    WLANError_t result = ERR_NO_ERROR;

    unsigned char buff[NETWORK_BUFLEN];
    unsigned int start;

    char wlan_tx_data[80];

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
                {
                    state = ERROR;
                    result = ERR_NOT_READY;
                }
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
                {
                    state = ERROR;
                    result = ERR_NOT_OKAY;
                }


                break;

            case OKAY:
                // todo: check these responses

                // client only mode
                uart_puts("AT+CWMODE=1\r\n");
                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        break;
                    }
                }

                // join ssid
                memset(wlan_tx_data, 0x00, sizeof(wlan_tx_data));
                strcat(wlan_tx_data,"AT+CWJAP=\"");
                strcat(wlan_tx_data, ssid);
                strcat(wlan_tx_data, "\",\"");
                strcat(wlan_tx_data, passphrase);
                strcat(wlan_tx_data, "\"\r\n");

                uart_puts(wlan_tx_data);

                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 15000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        break;
                    }
                }

                // reset
                uart_puts("AT+RST\r\n");
                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 3000)
                {
                    //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                    if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                    {
                        break;
                    }
                }

                state = DONE;
                break;

            case ERROR:
                //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7
                state = DONE;
                break;

            case DONE:
                // Turn off wireless module
                WLANEnable(0);
                return(result);
                break;

            default:
                break;

        }
    }
}

uint8_t WLANTransmit(int32_t Weight, uint16_t Battery, char * SiteKey, char * DeviceID)
{
    WLANState_t state = INIT;
    WLANError_t result = ERR_NO_ERROR;

    unsigned char buff[NETWORK_BUFLEN];
    unsigned int start;

    char weight_str[8];
    char battery_str[8];
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
                {
                    state = ERROR;
                    result = ERR_NOT_READY;
                }
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
                {
                    state = ERROR;
                    result = ERR_NOT_OKAY;
                }


                break;

            case OKAY:

                start = GetMilliSeconds();
                while (GetMilliSeconds() - start < 5000)
                {
                    uart_puts("AT+CIFSR\r\n");

                    // have to read multiple lines for address
                    for (int i = 0; i < 4 ; i++)
                    {
                        if (UART_ReceiveLine(buff, NETWORK_BUFLEN, 100))
                        {

                            //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7

                            // Things we might get back..
                            //
                            // ERROR
                            // Error
                            // busy now ...
                            // 0.0.0.0
                            // <ip addr>
                            //
                            // Just looking for a dot in the IP address for now - but watch out for "busy now ..." or "0.0.0.0"

                            if (strstr((char*)buff, "."))
                            {
                                if (!strstr((char*)buff, "0.0.0.0") && !strstr((char*)buff, "busy"))
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
                {
                    state = ERROR;
                    result = ERR_NOT_ONLINE;
                }
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
                {
                    state = ERROR;
                    result = ERR_NOT_CONNECTED;
                }

                break;

            case CONNECTING:    // not using yet
                break;

            case CONNECTED:
                // construct the data to send
                ltoa(Weight, weight_str, 10);
                itoa(Battery, battery_str, 10);

                memset(wlan_tx_data, 0x00, sizeof(wlan_tx_data));
                strcat(wlan_tx_data, "GET /update?key=");
                strcat(wlan_tx_data, SiteKey);
                strcat(wlan_tx_data, "&field1=");
                strcat(wlan_tx_data, weight_str);
                strcat(wlan_tx_data, "&field2=");
                strcat(wlan_tx_data, battery_str);
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
                                            state = SENT;
                                            break;                          // note: breaks out of while loop only
                                        }
                                    }
                                }
                                break;                          // note: breaks out of while loop only
                            }
                        }
                    }
                }

                // If we got here without setting the state to SENT, then error.
                if (state != SENT)
                {
                    state = ERROR;
                    result = ERR_NOT_SENT;
                }
                break;

            case SENT:
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
                        if (strstr((char*)buff, "Unlink") || strstr((char*)buff, "Linked"))
                        {
                            state = DISCONNECTED;
                            break;                          // note: breaks out of while loop only
                        }
                    }
                }

                // If we got here without setting the state to DISCONNECTED, then error.
                if (state != DISCONNECTED)
                {
                    state = ERROR;
                    result = ERR_NOT_DISCONNECTED;
                }

                break;

            case DISCONNECTED:
                state = DONE;
                break;

            case ERROR:
                //PORTF ^= (1 << 7);                      // Toggle the debug LED on port F7
                state = DONE;
                break;

            case DONE:
                // Turn off wireless module
                WLANEnable(0);
                return(result);
                break;

            default:
                break;

        }
    }
}

