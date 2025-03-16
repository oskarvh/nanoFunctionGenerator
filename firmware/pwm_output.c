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
// C
#include <math.h>
#include <stdio.h>

// Pico SDK
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"

// Header
#include "pwm_output.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"

#define MAX_NUM_ENTIRES_LUT 2048 // <! Maximum LUT size

xSemaphoreHandle configMutex;

//! Default configuration array of the PWM channels
pwm_channel_config_t pwm_config_table[NUM_PWM_CHANNELS] = {
    {
        .channel_num = PWM_PIN_0A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
    {
        .channel_num = PWM_PIN_1A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_SINE, 
        .numBits = 256, 
        .frequencyHz = 1000.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 1.0
    },
    {
        .channel_num = PWM_PIN_2A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
    {
        .channel_num = PWM_PIN_3A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
    {
        .channel_num = PWM_PIN_4A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
    {
        .channel_num = PWM_PIN_5A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
    {
        .channel_num = PWM_PIN_6A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
    {
        .channel_num = PWM_PIN_7A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0
    }, 
};

//! LUT of the pointers to the LUT for functions:
uint16_t* pLUTs[NUM_PWM_SLICES] = {NULL};
//! Global table for keeping track of the intended trans_count per DMA channel
uint32_t dmaTransCount[NUM_DMA_CHANNELS] = {0};

//! Function to get the set point from the duty cycle
//! @brief This function calculates the initial set point from the duty cycle
//! @param channel_config The configuration of the PWM channel
//! @return The set point value for the PWM channel
static uint32_t get_pwm_set_point_from_config(pwm_channel_config_t channel_config){
    return (uint32_t)(channel_config.dc_offset * channel_config.numBits);
    if(channel_config.signal_config == SIGNAL_CONFIG_DC){
        return (uint32_t)(channel_config.dc_offset * channel_config.numBits);
    }
    // else if (channel_config.signal_config == SIGNAL_CONFIG_SINE){
    //     // Initialize the sine wave, based on DC offset, phase offset, and amplitude
    //     return (uint32_t)(channel_config.dc_offset + channel_config.amplitude * sin(currentPhaseOffset[channel_config.slice_num]));
    // }
    // else if (channel_config.signal_config == SIGNAL_CONFIG_TRIANGLE){
    //     // Initialize the triangle wave, based on DC offset, phase offset, and amplitude
    //     return (uint32_t)(channel_config.dc_offset + channel_config.amplitude * (1.0 - fabs(2.0 * (currentTriangleValue[channel_config.slice_num] - 0.5))));
    // }
    // else if (channel_config.signal_config == SIGNAL_CONFIG_SQUARE){
    //     // Initialize the square wave, based on DC offset, phase offset, and amplitude
    //     return (uint32_t)(channel_config.dc_offset + channel_config.amplitude * (currentSquareValue[channel_config.slice_num] > 0.5 ? 1.0 : -1.0));
    // }
    else{
        printf("Error: Invalid signal configuration\r\n");
        while(1);
    }
}

static uint8_t slice_to_config_channel(uint8_t slice_num){
    for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++){
        if(pwm_config_table[i].slice_num == slice_num){
            return i;
        }
    }
    return 0xFF; // Error
}

uint16_t wrap_count = 100;
void pwm_callback(void){
    // This function is called when the PWM wraps
    // It can be used to update the PWM signal
    int irq;
    // Get the IRQ status to check which slice caused the interrupt

    // Set GP3 pin high to indicate the interrupt
    gpio_put(3, 1); // DEBUG
    //xSemaphoreTakeFromISR(configMutex, NULL);
    for (int slice=0; slice<8; slice++)
    {
        irq = pwm_get_irq_status_mask();
        
        if (irq & (1<<slice))
        {
            //uint8_t configChannel = slice_to_config_channel(slice);   
            // Get the set point from the 
            //uint32_t set_point = get_pwm_set_point_from_config(pwm_config_table[configChannel]);
            //pwm_set_chan_level(slice, pwm_config_table[i].channel_out, set_point);
            pwm_clear_irq(slice);
        }
    }
    //xSemaphoreGiveFromISR(configMutex, NULL);
    // Clear the pin to indicate the interrupt ending
    gpio_put(3, 0); //DEBUG
}

void dmaIrqHandler(void){
    // The DMA only has two IRQ channels, DMA_IRQ_0 and DMA_IRQ_1
    // DMA_IRQ_1 is not used here, so we need to get by using only one IRQ channel
    // This means that any DMA channel could cause this interrupt, 

    // Check which channel caused the interrupt
    for(int channel = 0; channel < NUM_DMA_CHANNELS; channel++){
        if(dma_channel_get_irq0_status(channel)){
            dma_channel_acknowledge_irq0(channel);
            // This channel caused the interrupt
            // Restart the DMA transfer
            dma_channel_set_trans_count(channel, dmaTransCount[channel], false);
            // Start the transfer
            dma_channel_start(channel);
            // Clear the interrupt
            
        }
    }
}

static uint32_t calculateNumEntires(pwm_channel_config_t *pPwmConfig){
    return (uint32_t)(((float)FREQ_PWM)/(pPwmConfig->numBits*pPwmConfig->frequencyHz));
}

uint32_t generate_sine_wave_lut(pwm_channel_config_t *pPwmConfig){
    // Calculate the number of entries in the LUT
    // TODO: Dynamically change the numBits. 
    uint32_t numEntries = calculateNumEntires(pPwmConfig);
    // numEntires determines the size of the LUT, where the size of the LUT scales with the period of the
    // signal in that a longer period -> more data. 
    // However, it also scales with the number of bits, so if we have a long signal we can scale the number of bits
    // accordingly. One entry takes 2 bytes, and the maximum size should be 1kB, which is 2048 entries. 
    // Likewise, if the sine wave frequency is too high in comparison with the number of bits, 
    // we can decrease the number of bits.
    while(numEntries < 25 && pPwmConfig->numBits > 32){
        // decrease the number of bits and recalculate
        pPwmConfig->numBits -= 1;
        numEntries = calculateNumEntires(pPwmConfig);
    }
    while(numEntries > MAX_NUM_ENTIRES_LUT && pPwmConfig->numBits < 65535){
        // Increase the number of bits and recalculate
        pPwmConfig->numBits += 1;
        numEntries = calculateNumEntires(pPwmConfig);
    }
    // Check if we could complete the calculation
    if(numEntries < 25 || numEntries > MAX_NUM_ENTIRES_LUT){
        // We failed to resolve a sine wave! 
        return 0;
    }
    if(pPwmConfig->numBits > 65535 || pPwmConfig->numBits < 32){
        // We failed to resolve a sine wave! 
        return 0;
    }
    printf("Num entries: %i\r\n",numEntries);
    pLUTs[pPwmConfig->slice_num] = (uint16_t*)malloc(numEntries*sizeof(uint16_t));
    if(pLUTs[pPwmConfig->slice_num] == NULL){
        // Unable to allocate. Return an entry size of 0
        return 0;
    }
    // Initialize the LUT
    //printf("------------START OF LUT------------------\r\n");
    for(uint32_t j = 0; j < numEntries; j++){
        pLUTs[pPwmConfig->slice_num][j] = 
        (uint16_t)((pPwmConfig->numBits)*(pPwmConfig->dc_offset + (pPwmConfig->amplitude/2) * sin((2.0 * M_PI * j) / numEntries + pPwmConfig->phase_offset)));
        //printf("%i\r\n",pLUTs[pwmSlice][j]);
    }
    //printf("-------------END OF LUT-------------------\r\n");
    return numEntries;
}

uint32_t generate_ramp_lut(pwm_channel_config_t *pPwmConfig){
    // Calculate the number of entries in the LUT
    uint32_t numEntries = calculateNumEntires(pPwmConfig);
    printf("Num entries: %i\r\n",numEntries);
    pLUTs[pPwmConfig->slice_num] = (uint16_t*)malloc(numEntries*sizeof(uint16_t));
    if(pLUTs[pPwmConfig->slice_num] == NULL){
        // Unable to allocate. Return an entry size of 0
        return 0;
    }
    pLUTs[pPwmConfig->slice_num][0] = 0;
    float increment = ((float)numEntries)/((float)pPwmConfig->numBits);
    for(uint32_t i = 1 ; i < numEntries ; i++){
        pLUTs[pPwmConfig->slice_num][i] = (uint32_t)((float)(pLUTs[pPwmConfig->slice_num][i-1]) + increment);
    }
    return numEntries;
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

    uint8_t sliceBitMask = 0;
    for(uint8_t i = 0; i < NUM_PWM_CHANNELS; i++){
        // Initialize the PWM hardware
        gpio_set_function(pwm_config_table[i].channel_num, GPIO_FUNC_PWM);

        // Get the slice
        uint32_t slice_num = pwm_gpio_to_slice_num(pwm_config_table[i].channel_num);

        // Make sure that the slice is not already used
        if ((1<<slice_num)&sliceBitMask != 0){
            while(1);
        }
        sliceBitMask |= (1<<slice_num);

        // Save the slice number of this channel
        pwm_config_table[i].slice_num = slice_num;

        // Enable the PWM IRQ and configure, but only if this isn't a DC signal:
        if(pwm_config_table[i].signal_config != SIGNAL_CONFIG_DC){
            // Enable the PWM IRQ
            // pwm_set_irq_enabled(slice_num, true);
            // irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), pwm_callback);
            // irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
            uint32_t numEntries = 0;
            // Create the LUTs if not DC signal
            if(pwm_config_table[i].signal_config == SIGNAL_CONFIG_SINE){
                numEntries = generate_sine_wave_lut(&(pwm_config_table[i]));
            }
            if(numEntries == 0){
                // There has been an error! 
                while(1);
            }

            // Set up a DMA channel to feed the PWM set point with the LUT that was just created. 
            int pwm_dma_chan = dma_claim_unused_channel(true);
            int dma_ctrl_chan = dma_claim_unused_channel(true);
            if(pwm_dma_chan < 0 || dma_ctrl_chan < 0){
                // No channel available. TODO: Handle this. 
                while(1);
            }
            dma_channel_config pwm_dma_chan_config = dma_channel_get_default_config(pwm_dma_chan);
            dma_channel_config dma_ctrl_chan_config = dma_channel_get_default_config(dma_ctrl_chan);

            // Set up the data channel: the LUT is 16 bits. 
            channel_config_set_transfer_data_size(&pwm_dma_chan_config, DMA_SIZE_16);
            // The read pointer shall increment while the write pointer stays fixed.
            channel_config_set_read_increment(&pwm_dma_chan_config, true);
            channel_config_set_write_increment(&pwm_dma_chan_config, false);
            // Transfer when PWM slice that is connected to the PWM asks for a new value
            channel_config_set_dreq(&pwm_dma_chan_config, DREQ_PWM_WRAP0 + pwm_config_table[i].slice_num);
            // Chain the data channel to the control channel
            channel_config_set_chain_to(&pwm_dma_chan_config, dma_ctrl_chan);
            // configure the data channel
            dma_channel_configure(
                pwm_dma_chan,
                &pwm_dma_chan_config,
                &pwm_hw->slice[slice_num].cc, // Write to PWM counter compare
                pLUTs[slice_num], // Read values from LUT
                numEntries, // Read the number of entries
                false // Don't start immediately.
            );

            // Set up the control channel
            channel_config_set_transfer_data_size(&dma_ctrl_chan_config, DMA_SIZE_32);             // 32-bit txfers
            channel_config_set_read_increment(&dma_ctrl_chan_config, false);                       // no read incrementing
            channel_config_set_write_increment(&dma_ctrl_chan_config, false);                      // no write incrementing
            channel_config_set_chain_to(&dma_ctrl_chan_config, pwm_dma_chan);                         // chain to data channel

            dma_channel_configure(
                dma_ctrl_chan,                      // Channel to be configured
                &dma_ctrl_chan_config,              // The configuration we just created
                &dma_hw->ch[pwm_dma_chan].read_addr,// Write address (data channel read address)
                &(pLUTs[slice_num]),                // Read address (POINTER TO AN ADDRESS)
                1,                                  // One single transfer
                false                               // Don't start immediately
            );
            
            // dmaTransCount[pwm_dma_chan] = numEntries;
            // // Set up the IRQ for the DMA channel to restart the DMA transfer_count
            // dma_channel_set_irq0_enabled(pwm_dma_chan, true);
            // irq_set_exclusive_handler(DMA_IRQ_0, dmaIrqHandler);
            // irq_set_enabled(DMA_IRQ_0, true);

            // // Start the channel
            dma_channel_start(dma_ctrl_chan);
        }
        else{
            // Set the set point
            uint32_t set_point = get_pwm_set_point_from_config(pwm_config_table[i]);
            //pwm_set_gpio_level(channel_num, set_point);
            //pwm_set_chan_level(slice_num, pwm_config_table[i].channel_out, set_point);
        }
        // Set the wrap based on numBits
        pwm_set_wrap(slice_num, pwm_config_table[i].numBits);

        
    }
    for(uint8_t i = 0; i < NUM_PWM_CHANNELS; i++){
        // Enable the PWM channel
        pwm_set_enabled(pwm_config_table[i].slice_num, true);
    }
}