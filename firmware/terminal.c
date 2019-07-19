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

const char TerminalDefaultPrompt[] = "\r\n"
                                     "WeighFi>";

const char TerminalHelpPrompt[]    = "Usage:\r\n"
                                     "?             - display this help text\r\n"
                                     "t             - test network configuration\r\n"
                                     "\r\n"
                                     "v             - display firmware version\r\n"
                                     "z             - display vibration sensor count\r\n"
                                     "l             - display ADC reading for load (wheatstone bridge)\r\n"
                                     "b             - display ADC reading for battery\r\n"
                                     "a             - display all configuration options\r\n"
                                     "\r\n"
                                     "s [value]     - display or update vibration sensor sensitivity\r\n"
                                     "c [value]     - display or update scale calibration value\r\n"
                                     "i [value]     - display or update site id\r\n"
                                     "k [value]     - display or update site key\r\n"
                                     "d [value]     - display or update device id\r\n"
                                     "n [value]     - display or update wifi network name\r\n"
                                     "p [value]     - display or update wifi passphrase\r\n"
                                     "\r\n"
                                     "w             - write new settings to non-volatile storage\r\n";

uint8_t TerminalReceiveChar(unsigned int timeout)
{
    // Try to read a character from the USB library

    unsigned int start = GetMilliSeconds();

    while (GetMilliSeconds() - start < timeout)
    {
        int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

        if (!(ReceivedByte < 0))
        {
            // got a character
            return((unsigned char)ReceivedByte);
        }
    }
    return 0;
}

void TerminalCheckInput(EEPROMData_t * EEPROMData)
{
    static unsigned char buffer[TERMINAL_BUFLEN];
    static unsigned int  bufpos;

    static unsigned char cmd;
    static unsigned char arg[TERMINAL_BUFLEN];

    unsigned char byte = TerminalReceiveChar(100);

    if (byte)
    {
        // todo: throw away 0x0A if first character?
        // handling both unix and dos line endings..

        // echo character back...
        fprintf(&USBSerialStream, "%c", byte);

        // add this new character to the buffer
        if (bufpos < TERMINAL_BUFLEN-1)
        {
            buffer[bufpos++] = byte;

            if (bufpos == TERMINAL_BUFLEN-1)
            {
                // null terminate the final character
                // in case we try to read as a string later
                buffer[bufpos] = 0x00;
            }
        }
        else
        {
            // now what - the buffer is full...?

            // throw any more characters away...?

            // or start over with next command...?
            memset(buffer, 0x00, sizeof(buffer));
            bufpos = 0;
        }

        // If received carriage return ASCII char
        if (byte == 0x0D)
        {
            // if buffer is not already full, add a null  terminator
            // in case we try to read as a string later
            if (bufpos < TERMINAL_BUFLEN-1)
            {
                buffer[bufpos+1] = 0x00;
            }

            fprintf(&USBSerialStream,"\r\n");

            //fprintf(&USBSerialStream, "0x%02x\r\n", buffer[0]);
            //fprintf(&USBSerialStream, "%d\r\n", strlen((char *)buffer));

            //
            // process the buffer
            //
            switch(buffer[0])
            {
                case '?':
                    fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 'a':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        fprintf(&USBSerialStream, "Version Major: %d\n\r", EEPROMData->SRAM_VersionMajor);
                        fprintf(&USBSerialStream, "Version Minor: %d\n\r", EEPROMData->SRAM_VersionMinor);
                        fprintf(&USBSerialStream, "Sensitivity  : %d\n\r", EEPROMData->SRAM_Sensitivity);
                        fprintf(&USBSerialStream, "Calibration  : %d\n\r", EEPROMData->SRAM_Calibration);
                        fprintf(&USBSerialStream, "Site ID      : %s\n\r", EEPROMData->SRAM_SiteID);
                        fprintf(&USBSerialStream, "Site Key     : %s\n\r", EEPROMData->SRAM_SiteKey);
                        fprintf(&USBSerialStream, "Device ID    : %s\n\r", EEPROMData->SRAM_DeviceID);
                        fprintf(&USBSerialStream, "Network      : %s\n\r", EEPROMData->SRAM_WiFi_SSID);
                        fprintf(&USBSerialStream, "Passphrase   : %s\n\r", EEPROMData->SRAM_WiFi_PASS);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 'b':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        uint16_t Battery = GetIntADCValue(ADC_BATTERY_SAMPLES);
                        fprintf(&USBSerialStream, "Battery ADC raw value: %i\r\n", Battery);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 'v':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        fprintf(&USBSerialStream, "Version %d.%d\r\n", VERSION_MAJOR, VERSION_MINOR);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 'z':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        fprintf(&USBSerialStream, "Vibration (wake) sensor event count %u\r\n", vibes);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 'l':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        int32_t Load = GetExtADCValue(ADC_SPEED_LOW, 3);
                        fprintf(&USBSerialStream, "Load ADC raw value: %li\r\n", Load);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 't':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        fprintf(&USBSerialStream, "Testing network connectivity...\r\n");

                        // test data upload, site should discard the zero value
                        uint8_t WLANResult = WLANTransmit(0, 0, (char *)EEPROMData->SRAM_SiteKey, (char *)EEPROMData->SRAM_DeviceID);
                        if (!WLANResult)
                            fprintf(&USBSerialStream, "Network test successful\r\n");
                        else
                            fprintf(&USBSerialStream, "Network test FAILED, error %u\r\n", WLANResult);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                case 's':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        // no new value provided, display current parameter
                        fprintf(&USBSerialStream, "Sensitivity  : %d\n\r", EEPROMData->SRAM_Sensitivity);
                    }
                    else
                    {
                        // parse new value out of command and update SRAM configuration data
                        uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                        // got some values
                        if (count == 2)
                        {
                            // todo: atoi is a bit crap, perhaps use strtol instead..
                            uint8_t val = atoi((char *)arg);

                            if (val)
                                EEPROMData->SRAM_Sensitivity = val;

                        }
                        else
                        {
                            fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                        }

                    }
                    break;

                case 'c':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        // no new value provided, display current parameter
                        fprintf(&USBSerialStream, "Calibration  : %d\n\r", EEPROMData->SRAM_Calibration);
                    }
                    else
                    {
                        // parse new value out of command and update SRAM configuration data
                        uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                        // got some values
                        if (count == 2)
                        {
                            // todo: atoi is a bit crap, perhaps use strtol instead..
                            uint16_t val = atoi((char *)arg);

                            if (val)
                                EEPROMData->SRAM_Calibration = val;

                        }
                        else
                        {
                            fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                        }

                    }
                    break;

                case 'i':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        // no new value provided, display current parameter
                        fprintf(&USBSerialStream, "Site ID      : %s\n\r", EEPROMData->SRAM_SiteID);
                    }
                    else
                    {
                        // parse new value out of command and update SRAM configuration data
                        uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                        // got some values
                        if (count == 2)
                        {
                            uint8_t length = strlen((char *)arg);
                            if (length)
                                strncpy((char*)EEPROMData->SRAM_SiteID, (char *)arg, sizeof(EEPROMData->SRAM_SiteID));
                        }
                        else
                        {
                            fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                        }

                    }
                    break;

                case 'k':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        // no new value provided, display current parameter
                        fprintf(&USBSerialStream, "Site Key     : %s\n\r", EEPROMData->SRAM_SiteKey);
                    }
                    else
                    {
                        // parse new value out of command and update SRAM configuration data
                        uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                        // got some values
                        if (count == 2)
                        {
                            uint8_t length = strlen((char *)arg);
                            if (length)
                                strncpy((char*)EEPROMData->SRAM_SiteKey, (char *)arg, sizeof(EEPROMData->SRAM_SiteKey));
                        }
                        else
                        {
                            fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                        }

                    }
                    break;

                case 'd':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        // no new value provided, display current parameter
                        fprintf(&USBSerialStream, "Device ID    : %s\n\r", EEPROMData->SRAM_DeviceID);
                    }
                    else
                    {
                        // parse new value out of command and update SRAM configuration data
                        uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                        // got some values
                        if (count == 2)
                        {
                            uint8_t length = strlen((char *)arg);
                            if (length)
                                strncpy((char*)EEPROMData->SRAM_DeviceID, (char *)arg, sizeof(EEPROMData->SRAM_DeviceID));
                        }
                        else
                        {
                            fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                        }

                    }
                    break;

                case 'n':
                     if (strlen((char *)buffer) == CMD_ONLY)
                     {
                         // no new value provided, display current parameter
                         fprintf(&USBSerialStream, "Network      : %s\n\r", EEPROMData->SRAM_WiFi_SSID);
                     }
                     else
                     {
                         // parse new value out of command and update SRAM configuration data
                         uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                         // got some values
                         if (count == 2)
                         {
                             uint8_t length = strlen((char *)arg);
                             if (length)
                                 strncpy((char*)EEPROMData->SRAM_WiFi_SSID, (char *)arg, sizeof(EEPROMData->SRAM_WiFi_SSID));
                         }
                         else
                         {
                             fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                         }

                     }
                     break;

                case 'p':
                     if (strlen((char *)buffer) == CMD_ONLY)
                     {
                         // no new value provided, display current parameter
                         fprintf(&USBSerialStream, "Passphrase   : %s\n\r", EEPROMData->SRAM_WiFi_PASS);
                     }
                     else
                     {
                         // parse new value out of command and update SRAM configuration data
                         uint8_t count = sscanf((char *)buffer, "%c %s", &cmd, arg);

                         // got some values
                         if (count == 2)
                         {
                             uint8_t length = strlen((char *)arg);
                             if (length)
                                 strncpy((char*)EEPROMData->SRAM_WiFi_PASS, (char *)arg, sizeof(EEPROMData->SRAM_WiFi_PASS));
                         }
                         else
                         {
                             fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                         }

                     }
                     break;

                case 'w':
                    if (strlen((char *)buffer) == CMD_ONLY)
                    {
                        // commit the new settings to EEPROM
                        fprintf(&USBSerialStream, "Storing new EEPROM settings...\r\n");
                        UpdateEEPROMData(EEPROMData);

                        // commit network configuration changes
                        fprintf(&USBSerialStream, "Storing new network settings...\r\n");
                        WLANConfigure((char *)EEPROMData->SRAM_WiFi_SSID, (char *)EEPROMData->SRAM_WiFi_PASS);
                    }
                    else
                        fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;

                default:
                    fprintf(&USBSerialStream, "%s", TerminalHelpPrompt);
                    break;
            }

            fprintf(&USBSerialStream, "%s", TerminalDefaultPrompt);

            // start over with next command...
            memset(buffer, 0x00, sizeof(buffer));
            bufpos = 0;
        }
    }
}
