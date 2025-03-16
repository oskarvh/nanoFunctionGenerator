/*
MIT License

Copyright (c) 2025 Oskar von Heideken

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H
// C
#include <stdlib.h>
#include <stdint.h>

// RP2040:
#include "pico/stdlib.h"
#include "hardware/pwm.h"


//! defgroup PWM_output PWM Output
//! @{
//! @brief Pin mappings for the PWM output
#define NUM_PWM_CHANNELS 8
#define PWM_PIN_0A 16 // PWM channel 0A
#define PWM_PIN_1A 2  // PWM channel 1A
#define PWM_PIN_2A 4  // PWM channel 2A
#define PWM_PIN_3A 6  // PWM channel 3A
#define PWM_PIN_4A 8  // PWM channel 4A
#define PWM_PIN_5A 10 // PWM channel 5A
#define PWM_PIN_6A 12 // PWM channel 6A
#define PWM_PIN_7A 14 // PWM channel 7A
// Optional pins for the B channels of the PWM channels
#define PWM_PIN_0B 17 // PWM channel 0B
#define PWM_PIN_1B 3  // PWM channel 1B
#define PWM_PIN_2B 5  // PWM channel 2B
#define PWM_PIN_3B 7  // PWM channel 3B
#define PWM_PIN_4B 9  // PWM channel 4B
#define PWM_PIN_5B 11 // PWM channel 5B
#define PWM_PIN_6B 13 // PWM channel 6B
#define PWM_PIN_7B 15 // PWM channel 7B
//! @}

//! RP2040 PWM frequency
#define FREQ_PWM 125000000 // 125 MHz

//! Signal type configuration
typedef enum {
    SIGNAL_CONFIG_DC = 0,
    SIGNAL_CONFIG_SINE = 1,
    SIGNAL_CONFIG_TRIANGLE = 2,
    SIGNAL_CONFIG_SQUARE = 3,
} pwm_signal_config_t;

//! Configuration for the PWM output
typedef struct pwm_channel_config {
    uint8_t channel_num;      //!< The PWM channel to configure
    uint8_t channel_out;       //!< The channel side to configure. A or B
    pwm_signal_config_t signal_config; //!< The type of signal to generate
    uint16_t numBits;  //!< The wrap count for the PWM signal. This correponds to the number of bits.
    float frequencyHz;  //!< The frequency of the sine/triangle/square wave signal
    float dc_offset;  //!< The normalized DC offset of the sine/triangle/square wave signal
    float phase_offset;  //!< The phase offset of the sine/triangle/square wave signal in radians
    float amplitude;  //!< Normalized amplitude of the sine/triangle/square wave signal
    uint32_t slice_num; //!< The slice number of the PWM signal
} pwm_channel_config_t;




void init_pwm();

#endif // PWM_OUTPUT_H