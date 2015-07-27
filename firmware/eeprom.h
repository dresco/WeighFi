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

#ifndef EEPROM_H
#define EEPROM_H

typedef struct EEPROMData
{
    uint8_t  SRAM_VersionMajor;
    uint8_t  SRAM_VersionMinor;
    uint16_t SRAM_Calibration;
    uint8_t  SRAM_SiteID[36];
    uint8_t  SRAM_SiteKey[36];
    uint8_t  SRAM_DeviceID[36];
    uint8_t  SRAM_Reserved[36];
    uint8_t  SRAM_WiFi_SSID[32];
    uint8_t  SRAM_WiFi_PASS[64];
} EEPROMData_t;

// function prototypes
void FetchEEPROMData(EEPROMData_t *);
void UpdateEEPROMData(EEPROMData_t *);

#endif //EEPROM_H
