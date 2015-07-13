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
// PC6 - output - LCD enable
// PD0 -        - I2C clock
// PD1 -        - I2C data
// PD2 -        - UART
// PD3 -        - UART
// PD4 - output - External ADC speed control
// PB4 - output - External ADC enable
// PD6 - output - SPI serial clock
// PD7 - input  - SPI data in (MISO)
// PF0 - output - WLAN module enable
// PF1 - input  - Internal ADC channel 1 - battery voltage
// PB0 - output - Status LED
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
FILE USBSerialStream;

ISR (WDT_vect)
{
}

ISR(TIMER1_COMPA_vect)
{
    g_ms++;
    //PORTB ^= (1 << 0);                      // Toggle the status LED on port B0
}

unsigned int GetMilliSeconds(void)
{
    return g_ms;
}

void TimerSetup(void)
{
    TCCR1B |= (1 << WGM12);                     // Configure 16bit timer1 for CTC mode 4
    TIMSK1 |= (1 << OCIE1A);                    // Enable CTC interrupt
    OCR1A = 250;                                // Set output compare value
    TCCR1B |= (1 << CS11) | (1 << CS10);        // Start timer with x64 prescaler for a 1MHz timer
}

void PortSetup(void)
{
    DDRC |= (1 << 6);                                       // Configure PC6 as output to enable the LCD power supply
    PORTC |= (1 << 6);                                      // Set PC6 to ensure LCD is powered down

    DDRD |= (1 << 4);                                       // Configure PD4 as output to control the ADC speed
    PORTD |= (1 << 4);                                      // Set PD4 for 80 samples per second

    DDRB |= (1 << 4);                                       // Configure PB4 as output to enable the ADC
    PORTB &= ~(1 << 4);                                     // Clear PB4 to ensure ADC is powered down

    DDRD |= (1 << 6);                                       // Configure PD6 as output for SPI SCK

    DDRF |= (1 << 0);                                       // Configure PF0 as output to enable WLAN module

    DDRB |= (1 << 0);                                       // Configure PB0 as output for status LED
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
    ADCSetup();

    //LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);

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
    uint8_t WLANResult = 0;

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
        ADCResult = GetExtADCValue(ADC_SPEED_LOW, 3);
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
    for (int i = 0 ; i < ADC_MAX_RETRIES ; i++)
    {
        ADCResult = GetExtADCValue(ADC_SPEED_LOW, 3);

        // Calculate the weight in grams
        // todo: increase calibration range for finer tuning of weight;
        //       from uint8_t to uint16_t
        //       counts per 5KG instead of counts per 1KG
        //
        ADCDelta = ADCResult - ADCZeroReading;
        Weight = DivideAndRoundToClosest((ADCDelta * 1000), EEPROMData->SRAM_Calibration);

        // Filter out any negative values
        if (Weight < 0)
            Weight = 0;

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

    uint16_t Battery = GetIntADCValue(ADC_BATTERY_SAMPLES);

    // Convert to millivolts
    // 0 = 0v, 1023 = 3.0v ADC reference
    // Expecting battery voltage up to ~5v
    // Using 670k:470k voltage divider to give full scale at 5.1v
    //  (note: high impedance to minimise battery drain in prototype,
    //         using 10uF capacitor to hold the voltage steady as ADC samples)
    // Close enough to multiply the ADC reading up by 5
    Battery = Battery * 5;

    uint32_t start = GetMilliSeconds();

    // Upload weight data to network, ignore reading if less than 1 KG
    if (Weight > 1000)
        WLANResult = WLANTransmit(Weight, Battery, (char *)EEPROMData->SRAM_SiteKey, (char *)EEPROMData->SRAM_DeviceID);

    // Ensure display blinks for at least 5 seconds
    while (GetMilliSeconds() - start < 5000)
        ;

    // Update display to indicate a successful (or otherwise) upload
    if (Weight)
    {
        if (!WLANResult)
        {
            // Successful upload
            memset(&DisplayData, 0x00, sizeof(DisplayData_t));
            DisplayData.Flags |= LCD_FLAG_DATA;
            DisplayData.Flags |= LCD_FLAG_BLINK;
            DisplayData.Char1 = 'S';
            DisplayData.Char2 = 'E';
            DisplayData.Char3 = 'N';
            DisplayData.Char4 = 'T';
            LCDUpdate(&DisplayData);
        }
        else
        {
            // Upload failed
            memset(&DisplayData, 0x00, sizeof(DisplayData_t));
            DisplayData.Flags |= LCD_FLAG_DATA;
            DisplayData.Flags |= LCD_FLAG_BLINK;
            DisplayData.Char1 = 'E';
            DisplayData.Char2 = 'R';
            DisplayData.Char3 = 'R';
            DisplayData.Char4 = '0' + WLANResult;
            LCDUpdate(&DisplayData);
        }
        _delay_ms(3000);
    }

    // Blank the display
    //DisplayData.Flags = LCD_FLAG_BLANK;
    //LCDUpdate(&DisplayData);

    // Power down the LCD
    LCDEnable(0);

    return(Weight);
}

// Event handler for the library USB Connection event.
void EVENT_USB_Device_Connect(void)
{
    //LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

// Event handler for the library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void)
{
    //LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

// Event handler for the library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

    //LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
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

    // Get default settings from EEPROM
    FetchEEPROMData(&EEPROMData);

    // If firmware and EEPROM *major* versions differ then erase all EEPROM content
    // to allow for changes in the size/position of any members between versions..
    if (EEPROMData.SRAM_VersionMajor |= VERSION_MAJOR)
        memset(&EEPROMData, 0x00, sizeof(EEPROMData));

    // Ensure code and EEPROM versions are in sync
    EEPROMData.SRAM_VersionMajor = VERSION_MAJOR;
    EEPROMData.SRAM_VersionMinor = VERSION_MINOR;
    UpdateEEPROMData(&EEPROMData);

    // Initialise the serial library
    // todo: port to LUFA serial library
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

    // Create a regular character stream for the interface so that it can be used with the stdio.h functions
    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    // Blank the LCD display
    //DisplayData.Flags = LCD_FLAG_BLANK;
    //LCDUpdate(&DisplayData);

    // Get a one off reading to populate the variables before fist use..
    ADCInitialResult = ADCLastResult = GetExtADCValue(ADC_SPEED_HIGH, 1);

    while (1)
    {
        //PORTB ^= (1 << 0);                      // Toggle the status LED on port B0

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
            ADCResult = GetExtADCValue(ADC_SPEED_HIGH, 1);

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

        if (USB_DeviceState == DEVICE_STATE_Configured)
        {
            TerminalCheckInput(&EEPROMData);
        }

        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();

        // Sleep if not USB connected, woken each second by watchdog interrupt
        if (USB_DeviceState == DEVICE_STATE_Unattached)
            sleep_mode();
    }
}
