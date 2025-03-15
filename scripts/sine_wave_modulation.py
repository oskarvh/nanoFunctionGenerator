"""
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

This script is a simple example of how to modulate a sine wave using a PWM signal. 
It's intended to be messed around with such that the user can get a feel for how 
this modulation scheme works.
"""

# Requirements
import numpy as np
import matplotlib.pyplot as plt
import argparse
from scipy import signal

def parse_args():
    """
    Parse the arguments
    :return: args
    """
    parser = argparse.ArgumentParser(description="Modulate a sine wave using a PWM signal.")
    parser.add_argument("--pwm_frequency_hz", type=int, default=1000, required=False, help="PWM freqency in Hz. This is the frequency of the PWM clock.")
    parser.add_argument("--sine_wave_frequency_hz", type=int, default=10, required=False, help="Sine wave frequency in Hz.")
    return parser.parse_args()

def generate_pwm_sine_wave():
    """
    Generate a sine wave modulated by a PWM signal.
    """
    # Constants for RP2040 and some other things that needn't be changed
    MAX_PWM_COUNT = 65535
    PWM_FREQUENCY_RP2040 = 125E6 # 125 MHz
    PWM_PERIOD = 1/PWM_FREQUENCY_RP2040
    NUM_PERIODS = 2 # Number of periods to generate

    # TODO: These should be argument later on
    num_points_per_period = 2**8 # This is the wrap count of the PWM counter
    dc_offset = 0.5 # Normalized DC offset
    amplitude = 0.5 # Normalized amplitude
    sine_wave_frequency_hz = 20000 # Sine wave frequency in Hz
    phase_offset_rad = np.pi*0.0 # Phase offset in radians

    assert num_points_per_period < MAX_PWM_COUNT, f"num_points_per_period must be less than {MAX_PWM_COUNT}"

    sine_wave_period_time = NUM_PERIODS/sine_wave_frequency_hz
    pwm_period_time = NUM_PERIODS*num_points_per_period/PWM_FREQUENCY_RP2040
    num_pwm_periods_per_sine_wave_period = int(sine_wave_period_time/pwm_period_time)
    num_pwm_samples = NUM_PERIODS*num_pwm_periods_per_sine_wave_period*num_points_per_period


    # Calculate the maximum sine wave frequency we can generate with a num_pwm_periods_per_sine_wave_period of at least 25:
    wanted_pwm_periods_per_sine_wave_period = 25
    max_sine_wave_frequency_hz = np.round(NUM_PERIODS/(wanted_pwm_periods_per_sine_wave_period*pwm_period_time),2)

    print(f"Maximum sine wave frequency with num_pwm_periods_per_sine_wave_period of at least 25 for a wrap count of {num_points_per_period}: {max_sine_wave_frequency_hz} Hz")
    print(f"Number of PWM periods per sine wave period: {num_pwm_periods_per_sine_wave_period}")

    sine_wave_time = np.linspace(0, NUM_PERIODS/sine_wave_frequency_hz, int(num_pwm_samples/num_points_per_period))
    # Generate the sine wave
    w = 2*np.pi*sine_wave_frequency_hz
    sine_wave = amplitude*np.sin(w*sine_wave_time + phase_offset_rad)+dc_offset

    # Given the sine wave, we can now modulate the duty cycle of the PWM signal
    # with the sine wave, since the length of the vector is equal to the points per period
    duty_cycle = sine_wave
    
    # Now generate the PWM signal. For each num_points_per_period, we'll sample the sine wave
    # and set the duty cycle of the PWM signal to that value
    
    
    pwm_time = np.linspace(0, NUM_PERIODS/sine_wave_frequency_hz, num_pwm_samples)

    sine_wave_idx = 0
    pwm_signal = np.zeros(num_pwm_samples)
    duty_cycle_signal = np.zeros(num_pwm_samples)
    duty_cycle = sine_wave[0]
    duty_cycle_value = duty_cycle
    pwm_signal_value = 1
    duty_cycle_set_point = 0
    set_point_counter = 0
    for i in range(0, num_pwm_samples):
        if i % num_points_per_period == 0:
            # Sample the sine wave, truncated to the number of decimal points we'd get
            duty_cycle = sine_wave[sine_wave_idx]
            sine_wave_idx += 1

            # Reset the PWM signals
            pwm_signal_value = 1
            set_point_counter = 0

            # Calcualte the duty cycle set point
            duty_cycle_set_point = int(num_points_per_period*duty_cycle)
            duty_cycle_value = np.round(100*duty_cycle_set_point/num_points_per_period)/100
        duty_cycle_signal[i] = duty_cycle_value
        set_point_counter += 1
        if set_point_counter > duty_cycle_set_point:
            pwm_signal_value = 0
        pwm_signal[i] = pwm_signal_value
    
    plt.subplot(311)
    pwm_plot = plt.plot(pwm_time, pwm_signal)
    plt.grid()
    plt.ylabel("Normalized PWM amplitude")
    plt.legend(["PWM signal"], loc = "upper right")
    plt.subplot(312)
    sine_plot = plt.plot(sine_wave_time, sine_wave)
    dc_plot = plt.plot(pwm_time, duty_cycle_signal)
    plt.legend(["Sine Wave", "PWM Signal duty cycle value"], loc = "upper right")
    plt.grid()
    plt.xlabel("Time [s]")
    plt.ylabel("Normalized Amplitude")

    cutoff_frequency = PWM_FREQUENCY_RP2040/num_points_per_period/10
    print(f"Filter cutoff frequency={cutoff_frequency} Hz")

    normalized_cutoff_frequency = (cutoff_frequency)/(PWM_FREQUENCY_RP2040)
    # Filter with a butterworth filter with a cutoff frequency at 1.25 MHz to get the simulated analog output
    filt = signal.butter(2, normalized_cutoff_frequency, 'low', output='sos')
    filtered = signal.sosfilt(filt, pwm_signal)
    plt.subplot(313)
    filtered_plot = plt.plot(pwm_time, filtered)
    plt.grid()
    plt.xlabel("Time [s]")
    plt.ylabel("Filtered PWM signal")
    plt.legend(["Filtered PWM signal"], loc = "upper right")
    plt.show()
    pass

if __name__ == "__main__":
    # Parse arguments
    args = parse_args()
    generate_pwm_sine_wave()

    