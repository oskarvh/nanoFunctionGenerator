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

#include "pwm_output.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/pwm.h"

//! Default configuration array of the PWM channels
pwm_channel_config_t pwm_config_table[NUM_PWM_CHANNELS] = {
    {PWM_PIN_0A, PWM_CHAN_A, 5000, 50.0}, 
    {PWM_PIN_1A, PWM_CHAN_A, 5000, 50.0}, 
    {PWM_PIN_2A, PWM_CHAN_A, 5000, 20.0},
    {PWM_PIN_3A, PWM_CHAN_A, 5000, 20.0},
    {PWM_PIN_4A, PWM_CHAN_A, 5000, 20.0},
    {PWM_PIN_5A, PWM_CHAN_A, 5000, 20.0},
    {PWM_PIN_6A, PWM_CHAN_A, 5000, 20.0},
    {PWM_PIN_7A, PWM_CHAN_A, 5000, 20.0},
};

uint32_t get_pwm_wrap_count_from_frequency(uint32_t frequencyHz){
    // This returns the closest wrap count to the desired frequency 
    // expected wrap_count = 1/8ns * 1/frequency = 125MHz * 1/frequency
    return 125000000/frequencyHz;
}

uint32_t get_pwm_set_point_from_duty_cycle(uint32_t wrap_count, float duty_cycle){
    // This returns the set point for the desired duty cycle
    return wrap_count*duty_cycle/100;
}

void init_pwm(){
    /*
    The RP2040 has 8 PWM channels, each with an A and B output.
    The counter increments evey 8ns (125MHz) up to 65535, then resets.
    Therefore, the period of the PWM signal is set by the wrap count, 
    meaning that the wrap count is calculated as expected_period/8ns = wrap_count.

    The set point is the value at which the PWM signal output toggles, meaning that's what
    is used to create the duty cycle. The set point is calculated as duty_cycle[%]*wrap_count.
    */

    for(uint8_t i = 0; i < NUM_PWM_CHANNELS; i++){
        // Initialize the PWM hardware
        uint8_t channel_num = pwm_config_table[i].channel_num;
        gpio_set_function(pwm_config_table[i].channel_num, GPIO_FUNC_PWM);
        // Get the slice
        uint32_t slice_num = pwm_gpio_to_slice_num(pwm_config_table[i].channel_num);

        // Enable the PWM channel
        pwm_set_enabled(slice_num, true);

        // Set the wrap count
        uint32_t wrap_count = get_pwm_wrap_count_from_frequency(pwm_config_table[i].frequencyHz);
        printf("Wrap count: %i\n", wrap_count);
        pwm_set_wrap(slice_num, wrap_count);

        // Set the set point
        uint32_t set_point = get_pwm_set_point_from_duty_cycle(wrap_count, pwm_config_table[i].duty_cycle);
        printf("Set point: %i\n", set_point);
        //pwm_set_gpio_level(channel_num, set_point);
        pwm_set_chan_level(slice_num, pwm_config_table[i].channel_out, set_point);
    }
}