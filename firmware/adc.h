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

#ifndef ADC_H
#define ADC_H

#define ADC_DATA_BITS 18
#define ADC_SPARE_BITS 1

#define ADC_SPEED_LOW 0
#define ADC_SPEED_HIGH 1

#define ADC_COUNTS_PER_KG  140                              // Will move to EEPROM in due course
#define ADC_WAKE_THRESHOLD 100                              // Difference in periodic readings that will start processing
#define ADC_STABLE_THRESHOLD 2                              // Max difference in stable readings before accepting result
#define ADC_MAX_RETRIES     10                              // Max attempts whilst waiting for weight to settle

// function prototypes

int32_t GetADCValue(uint8_t, uint8_t);

#endif //ADC_H


