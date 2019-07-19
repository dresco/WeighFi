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

// Let GCC allocate the EEPROM offsets for us..
uint8_t  EEMEM EEPROM_VersionMajor  = 1;
uint8_t  EEMEM EEPROM_VersionMinor  = 0;
uint8_t  EEMEM EEPROM_Sensitivity   = 100;
uint16_t EEMEM EEPROM_Calibration   = 700;
uint8_t  EEMEM EEPROM_SiteID[36]    = "1000";
uint8_t  EEMEM EEPROM_SiteKey[36]   = "12345678";
uint8_t  EEMEM EEPROM_DeviceID[36]  = "1010";
uint8_t  EEMEM EEPROM_Reserved[36];
uint8_t  EEMEM EEPROM_WiFi_SSID[32] = "SSID";
uint8_t  EEMEM EEPROM_WiFi_PASS[64] = "PASSWORD";

void FetchEEPROMData(EEPROMData_t *EEPROMData)
{
    EEPROMData->SRAM_VersionMajor = eeprom_read_byte(&EEPROM_VersionMajor);
    EEPROMData->SRAM_VersionMinor = eeprom_read_byte(&EEPROM_VersionMinor);
    EEPROMData->SRAM_Sensitivity = eeprom_read_byte(&EEPROM_Sensitivity);

    EEPROMData->SRAM_Calibration = eeprom_read_word(&EEPROM_Calibration);

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
    eeprom_update_byte(&EEPROM_Sensitivity, EEPROMData->SRAM_Sensitivity);

    eeprom_update_word(&EEPROM_Calibration, EEPROMData->SRAM_Calibration);

    eeprom_update_block((const void *)EEPROMData->SRAM_SiteID, (void *)EEPROM_SiteID, 36);
    eeprom_update_block((const void *)EEPROMData->SRAM_SiteKey, (void *)EEPROM_SiteKey, 36);
    eeprom_update_block((const void *)EEPROMData->SRAM_DeviceID, (void *)EEPROM_DeviceID, 36);
    eeprom_update_block((const void *)EEPROMData->SRAM_Reserved, (void *)EEPROM_Reserved, 36);

    eeprom_update_block((const void *)EEPROMData->SRAM_WiFi_SSID, (void *)EEPROM_WiFi_SSID, 32);
    eeprom_update_block((const void *)EEPROMData->SRAM_WiFi_PASS, (void *)EEPROM_WiFi_PASS, 64);
}

