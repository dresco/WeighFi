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

void ADCSetup(void)
{

    ADMUX |= (1<<REFS0);                                    // Use AVcc with external capacitor at AREF pin
    ADMUX  |= (1 << MUX0);                                  // Select ADC channel 1 (PF1)

    ADCSRA |= (1<<ADPS2) | (1<<ADPS1)
                         | (1<<ADPS1);                      // ADC clock prescaler = 16MHz/128 = 125kHz
}

int16_t GetIntADCValue(uint8_t NumSamples)
{
    int16_t result = 0;

    ADCSRA |= (1 << ADEN);                                  // Enable the ADC

    for (uint8_t adc_count = 0; adc_count < NumSamples; adc_count++)
    {
        ADCSRA |= (1 << ADSC);                              // Start ADC conversion
        while(ADCSRA & (1<<ADSC));                          // Wait for conversion to finish
        result += ADC;                                      // Add conversion result
    }

    ADCSRA &= ~(1 << ADEN);                                 // Disable the ADC

    result = result / NumSamples;

    return(result);
}

int32_t GetExtADCValue(uint8_t ADCHighSpeed, uint8_t NumSamples)
{
    int32_t adc_value;                                      // signed 32 bit integer for ADC output
    int32_t adc_final;
    int64_t adc_total;
    int samples, i, bit;

    if (ADCHighSpeed)
        PORTD |= (1 << 4);                                  // Set PD4 for 80 samples per second
    else
        PORTD &= ~(1 << 4);                                 // Clear PD4 for 10 samples per second


    PORTB |= (1 << 4);                                      // Set PB4 to power up ADC

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

    PORTB &= ~(1 << 4);                                     // Clear PB4 to power down ADC

    adc_final = adc_total / NumSamples;                     // average the ADC readings
    return(adc_final);
}

