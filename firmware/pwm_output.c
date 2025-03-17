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
#include <string.h>

// Pico SDK
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"

// Header
#include "pwm_output.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"

#define MAX_NUM_ENTIRES_LUT 4098 // <! Maximum LUT size

// Mutex for PWM config
xSemaphoreHandle PWMConfigMutex;

// Queue for incoming settings
QueueHandle_t pwmSettingsQueue;

// Config table
pwm_channel_config_t pwm_config_table[NUM_PWM_CHANNELS] = {
    {
        .channel_num = PWM_PIN_0A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 0,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
    {
        .channel_num = PWM_PIN_1A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_SINE, 
        .numBits = 256, 
        .frequencyHz = 1000.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 1.0,
        .output_channel_num = 1,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    },
    {
        .channel_num = PWM_PIN_2A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 2,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
    {
        .channel_num = PWM_PIN_3A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 3,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
    {
        .channel_num = PWM_PIN_4A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 4,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
    {
        .channel_num = PWM_PIN_5A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 5,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
    {
        .channel_num = PWM_PIN_6A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 6,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
    {
        .channel_num = PWM_PIN_7A, 
        .channel_out = PWM_CHAN_A, 
        .signal_config = SIGNAL_CONFIG_DC, 
        .numBits = 256, 
        .frequencyHz = 0.0, 
        .dc_offset = 0.5, 
        .phase_offset = 0.0, 
        .amplitude = 0.0,
        .output_channel_num = 7,
        .pwm_dma_channel = -1,
        .dma_ctrl_channel = -1
    }, 
};

//! LUT of the pointers to the LUT for functions:
static uint16_t* pLUTs[NUM_PWM_SLICES] = {NULL};


//! @brief Calculate the number of LUT entries based on frequency
//! @param pPwmConfig Pointer to the PWM config
//! @return Number of entries required
static uint32_t calculateNumEntires(pwm_channel_config_t *pPwmConfig){
    uint32_t numEntries = (uint32_t)(((float)FREQ_PWM)/(pPwmConfig->numBits*pPwmConfig->frequencyHz));
    if(numEntries > MAX_NUM_ENTIRES_LUT){
        // We're going to have issues with allocate enough data for the LUT.
        return 0;
    }
    return numEntries;
}

//! @brief Allocate the LUT
//! @param slice_num LUT is indexed by PWM slice. 
//! @param numEntries Number of entries required in the LUT
//! @return True if successful, otherwise false
static bool allocate_lut(uint8_t slice_num, uint32_t numEntries){
    pLUTs[slice_num] = (uint16_t*)malloc(numEntries*sizeof(uint16_t));
    if(pLUTs[slice_num] == NULL){
        // Unable to allocate
        return false;
    }
    return true;
}

//! @brief Generate a sine wave in the LUT
//! @param pPwmConfig Pointer to the PWM config
//! @return Number of entries created for the signal
static uint32_t generate_sine_wave_lut(pwm_channel_config_t *pPwmConfig){
    // Calculate the number of entries in the LUT

    uint32_t numEntries = calculateNumEntires(pPwmConfig);
    // Check if we could complete the calculation
    if(numEntries < 25){
        // We're going to fail to generate a good sine wave! 
        return 0;
    }
    if(!allocate_lut(pPwmConfig->slice_num, numEntries)){
        // Unable to allocate. Return an entry size of 0
        return 0;
    }
    // Initialize the LUT
    for(uint32_t j = 0; j < numEntries; j++){
        pLUTs[pPwmConfig->slice_num][j] = 
        (uint16_t)((pPwmConfig->numBits)*(pPwmConfig->dc_offset + (pPwmConfig->amplitude/2) * sin((2.0 * M_PI * j) / numEntries + pPwmConfig->phase_offset)));
    }
    return numEntries;
}

//! @brief Generate a ramp/saw wave in the LUT
//! @param pPwmConfig Pointer to the PWM config
//! @return Number of entries created for the signal
static uint32_t generate_ramp_lut(pwm_channel_config_t *pPwmConfig){
    // Calculate the number of entries in the LUT
    uint32_t numEntries = calculateNumEntires(pPwmConfig);
    if(!allocate_lut(pPwmConfig->slice_num, numEntries)){
        // Unable to allocate. Return an entry size of 0
        return 0;
    }
    float value = pPwmConfig->dc_offset - pPwmConfig->amplitude/2;
    pLUTs[pPwmConfig->slice_num][0] = (uint16_t)value;
    float increment = pPwmConfig->numBits*pPwmConfig->amplitude/((float)numEntries);
    for(uint32_t i = 1 ; i < numEntries ; i++){
        value += increment;
        pLUTs[pPwmConfig->slice_num][i] = (uint16_t)(value);
    }
    return numEntries;
}

static uint32_t generate_triangle_lut(pwm_channel_config_t *pPwmConfig){
    // Calculate the number of entries in the LUT
    uint32_t numEntries = calculateNumEntires(pPwmConfig);
    if(!allocate_lut(pPwmConfig->slice_num, numEntries)){
        // Unable to allocate. Return an entry size of 0
        return 0;
    }
    float value = pPwmConfig->dc_offset - pPwmConfig->amplitude/2;
    pLUTs[pPwmConfig->slice_num][0] = (uint16_t)value;
    float increment = pPwmConfig->numBits*2*pPwmConfig->amplitude/((float)numEntries);
    for(uint32_t i = 1 ; i < numEntries/2 ; i++){
        value += increment;
        pLUTs[pPwmConfig->slice_num][i] = (uint16_t)(value);
    }
    for(uint32_t i = numEntries/2 ; i < numEntries ; i++){
        value -= increment;
        pLUTs[pPwmConfig->slice_num][i] = (uint16_t)(value);
    }
    return numEntries;
}

static uint32_t generate_square_wave_lut(pwm_channel_config_t *pPwmConfig){
    // Calculate the number of entries in the LUT
    uint32_t numEntries = calculateNumEntires(pPwmConfig);
    if(!allocate_lut(pPwmConfig->slice_num, numEntries)){
        // Unable to allocate. Return an entry size of 0
        return 0;
    }
    // Set the first half of the LUT to 0, and the second half to 1
    memset(&(pLUTs[pPwmConfig->slice_num][0]), (uint16_t)(pPwmConfig->dc_offset-pPwmConfig->amplitude/2), (size_t)(sizeof(uint16_t)*numEntries/2));
    memset(&(pLUTs[pPwmConfig->slice_num][numEntries/2]), (uint16_t)(pPwmConfig->dc_offset+pPwmConfig->amplitude/2), (size_t)(sizeof(uint16_t)*numEntries/2));
    return numEntries;
}

//! @brief Configure a channel based on the PWM config pointer
//!        This assumes any PWM or DMA channels this uses are OFF
//! @param pPwmConfig Pointer to the configuration
//! @return <>
static void configureChannel(pwm_channel_config_t *pPwmConfig){
    
    // Check if this is a DC signal
    if(pPwmConfig->signal_config != SIGNAL_CONFIG_DC){
        uint32_t numEntries = 0;
        // Create the LUTs if not DC signal
        switch (pPwmConfig->signal_config)
        {
        case SIGNAL_CONFIG_SINE:
            numEntries = generate_sine_wave_lut(pPwmConfig);
            break;
        case SIGNAL_CONFIG_TRIANGLE:
            numEntries = generate_triangle_lut(pPwmConfig);
            break;
        case SIGNAL_CONFIG_SQUARE:
            numEntries = generate_square_wave_lut(pPwmConfig);
            break;
        case SIGNAL_CONFIG_SAW:
            numEntries = generate_ramp_lut(pPwmConfig);
            break;
        
        default:
            numEntries = 0;
            break;
        }
#if false // ONLY FOR DEBUG
        printf("----------------- START OF LUT -----------------\r\n");
        for(uint32_t i = 0 ; i < numEntries;i++){
            printf("%i\r\n", pLUTs[pPwmConfig->slice_num][i]);
        }
        printf("------------------ END OF LUT ------------------\r\n");
#endif
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
        channel_config_set_dreq(&pwm_dma_chan_config, DREQ_PWM_WRAP0 + pPwmConfig->slice_num);
        // Chain the data channel to the control channel
        channel_config_set_chain_to(&pwm_dma_chan_config, dma_ctrl_chan);
        // configure the data channel
        dma_channel_configure(
            pwm_dma_chan,
            &pwm_dma_chan_config,
            &pwm_hw->slice[pPwmConfig->slice_num].cc, // Write to PWM counter compare
            pLUTs[pPwmConfig->slice_num], // Read values from LUT
            numEntries, // Read the number of entries
            false // Don't start immediately.
        );

        // Set up the control channel
        channel_config_set_transfer_data_size(&dma_ctrl_chan_config, DMA_SIZE_32);  // 32-bit txfers
        channel_config_set_read_increment(&dma_ctrl_chan_config, false);    // no read incrementing
        channel_config_set_write_increment(&dma_ctrl_chan_config, false);   // no write incrementing
        channel_config_set_chain_to(&dma_ctrl_chan_config, pwm_dma_chan);   // chain to data channel

        dma_channel_configure(
            dma_ctrl_chan,                      // Channel to be configured
            &dma_ctrl_chan_config,              // The configuration we just created
            &dma_hw->ch[pwm_dma_chan].read_addr,// Write address (data channel read address)
            &(pLUTs[pPwmConfig->slice_num]),    // Read address (POINTER TO AN ADDRESS)
            1,                                  // One single transfer
            false                               // Don't start immediately
        );

        // Save the data and control channel numbers
        pPwmConfig->pwm_dma_channel = pwm_dma_chan;
        pPwmConfig->dma_ctrl_channel = dma_ctrl_chan;

        // Start the control channel
        dma_channel_start(dma_ctrl_chan);
    }
    else{
        // Set the set point
        uint32_t set_point = (uint32_t)(pPwmConfig->dc_offset * pPwmConfig->numBits);
    }

    // Set the wrap based on numBits
    pwm_set_wrap(pPwmConfig->slice_num, pPwmConfig->numBits);
}

//! @brief Re-onfigure a channel based on the PWM config pointer. This halts any ongoing
//!        active output.
//! @param pPwmConfig Pointer to the configuration
//! @return <>
static void reconfigureChannel(pwm_channel_config_t *pOldPwmConfig, pwm_channel_config_t *pNewPwmConfig){
    
    // Check if there's an active DMA channel in the old config
    if(pOldPwmConfig->dma_ctrl_channel >= 0){
        // Stop the control channel
        dma_channel_abort(pOldPwmConfig->dma_ctrl_channel);
    }
    if(pOldPwmConfig->pwm_dma_channel >= 0){
        // Stop the data channel
        dma_channel_abort(pOldPwmConfig->pwm_dma_channel);
    }
    // Stop the PWM
    pwm_set_enabled(pOldPwmConfig->slice_num, false);
    
    // Unclaim the DMA channels
    if(pOldPwmConfig->dma_ctrl_channel >= 0){
        dma_channel_unclaim(pOldPwmConfig->dma_ctrl_channel);
    }
    if(pOldPwmConfig->pwm_dma_channel >= 0){
        dma_channel_unclaim(pOldPwmConfig->pwm_dma_channel);
    }

    // The old config is now disabled, configure the new one
    configureChannel(pNewPwmConfig);

    // Enable the PWM channel again
    pwm_set_enabled(pNewPwmConfig->slice_num, true);

} 

//! @brief Initialize the PWM channels based on defalt config.
static void init_pwm(){
    // The RP2040 has 8 PWM channels, each with an A and B output.
    // The counter increments evey 8ns (125MHz) up to 65535, then resets.
    // Therefore, the period of the PWM signal is set by the wrap count, 
    // meaning that the wrap count is calculated as expected_period/8ns = wrap_count.

    // The set point is the value at which the PWM signal output toggles, meaning that's what
    // is used to create the duty cycle. The set point is calculated as duty_cycle[%]*wrap_count. 


    uint8_t sliceBitMask = 0;
    if(xSemaphoreTake(PWMConfigMutex, portMAX_DELAY) == pdTRUE){
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

            // Configure the channel
            configureChannel(&(pwm_config_table[i]));        
        }

        // Start each PWM channel
        for(uint8_t i = 0; i < NUM_PWM_CHANNELS; i++){
            // Enable the PWM channel
            pwm_set_enabled(pwm_config_table[i].slice_num, true);
        }
        xSemaphoreGive(PWMConfigMutex);
    } else {
        printf("ERROR: UNABLE TO OBTAIN PWMConfigMutex SEMAPHORE IN init_pwm");
        while(1); // TODO: Handle this error
    }
}


void outputHandler(void *p){
    sleep_ms(200);
    // Initialize the PWM to default settings
    init_pwm();

    // Temporary setting to be written to the config
    pwm_channel_config_t newPwmConfig;
    while(1){
        // Receive new settings from the queue
        while(xQueueReceive(pwmSettingsQueue, &newPwmConfig, portMAX_DELAY)){
            // We've received a new message!
            // Filter out the unique channel number that was used, and copy over the settings
            if (xSemaphoreTake(PWMConfigMutex, 1000*portTICK_PERIOD_MS) == pdTRUE) {
                for(uint8_t i = 0; i < NUM_PWM_CHANNELS; i++){
                    if(pwm_config_table[i].output_channel_num == newPwmConfig.output_channel_num && newPwmConfig.output_channel_num != -1){
                        // Copy the old table to retain a copy
                        pwm_channel_config_t copyOfTheOldConfig;
                        memcpy(&copyOfTheOldConfig, &(pwm_config_table[i]), sizeof(pwm_channel_config_t));
                        
                        // Give back the semaphore
                        xSemaphoreGive(PWMConfigMutex);

                        // Reconfigure the channel
                        reconfigureChannel(&copyOfTheOldConfig, &newPwmConfig);
                        
                        // Copy over the new config
                        if (xSemaphoreTake(PWMConfigMutex, 1000*portTICK_PERIOD_MS) == pdTRUE){
                            memcpy(&(pwm_config_table[i]), &newPwmConfig, sizeof(pwm_channel_config_t));
                            xSemaphoreGive(PWMConfigMutex);
                        } else {
                            while(1); // TODO: Handle the error
                        }
                        break;
                    }
                }
            } else {
                // Semaphore could not be given after 1s. Error out
                printf("ERROR: UNABLE TO OBTAIN PWMConfigMutex SEMAPHORE IN outputHandler");
                while(1); // TODO: Handle this
            }
        }
        
    }
}