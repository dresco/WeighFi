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
// Bit banging the SPI interface. Is slightly odd as there is no MOSI, and the
// ADC uses MISO to signify conversion data is ready, as well as for data..
//
// PD4 - output - ADC speed control
// PD5 - output - ADC power control
// PD6 - output - SPI serial clock
// PD7 - input  - SPI data in (MISO)
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
    DDRD |= (1 << 4);                                       // Configure PD4 as output to control the ADC speed
    PORTD |= (1 << 4);                                      // Set PD4 for 80 samples per second

    DDRD |= (1 << 5);                                       // Configure PD5 as output to enable the ADC
    PORTD &= ~(1 << 5);                                     // Clear PD5 to ensure ADC is powered down

    DDRD |= (1 << 6);                                       // Configure PD6 as output for SPI SCK
}

void SetupHardware(void)
{
    // Disable watchdog if enabled by bootloader/fuses
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    // Disable clock division
    clock_prescale_set(clock_div_1);

    // Hardware Initialization
    LEDs_Init();
    USB_Init();

    PortSetup();
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

int main(void)
{
    SetupHardware();

    // Create a regular character stream for the interface so that it can be used with the stdio.h functions
    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
    GlobalInterruptEnable();

    while (1)
    {
        // Wait for a character from the host, and send back an ADC result
        // fixme: does this block ???
        int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

        if (!(ReceivedByte < 0))
        {
            // Get a reading from the ADC, configured for high speed and a
            // single sample (no averaging) - worst combination for accuracy..
            int32_t ADCResult = GetADCValue(ADC_SPEED_HIGH, 1);

            // Write the a debug char to USB output if connected
            if (USB_DeviceState == DEVICE_STATE_Configured)
                fprintf(&USBSerialStream, "%ld\n\r", ADCResult);
        }

        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
    }
}
