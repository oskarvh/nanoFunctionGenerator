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
#include <stdio.h>
#include <stdlib.h>

// Freertos:
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"

// RP2040 stdlib
#include "pico/stdlib.h"

// SCPI library
#include "scpi/scpi.h"
#include "scpi_interface.h"

// PWM output
#include "pwm_output.h"

//! Semaphore to protect the serial port
xSemaphoreHandle serialMutex;

//! Hardware settings are read with pin 26 and 27
#define HARDWARE_SETTING_0 26
//! Hardware settings are read with pin 26 and 27
#define HARDWARE_SETTING_1 27
//! Number of hardware configurations
#define NUM_SUPPORTED_HARDWARE 1

static uint8_t hardwareSettingIndex = 0;

//! Hardware settings, will be read from the board! 
typedef struct hardwareSettings {
    float pwmPinOutputVoltageMax;   //! Maximum voltage on the PWM output pin, in V
    float pwmPinOutputVoltageMin;   //! Minimum voltage on the PWM output pin, in V
    float outputVoltageMax;         //! Maximum output voltage after buffers, in V
    float outputVoltageMin;         //! Minimum output voltage after buffers, in V 
    float outputVoltageOffset;      //! Buffered voltage static offset, in V
    char  name[30];                 //! The name of this configuration
} hardwareSettings_t;

hardwareSettings_t hardwareSettings[NUM_SUPPORTED_HARDWARE] = {
    {
        .pwmPinOutputVoltageMax = 3.3,  // Maximum output voltage of the PWM pin
        .pwmPinOutputVoltageMin = 0.0,  // Minimum output voltage of the PWM pin
        .outputVoltageMax = 3.3,        // This config has no amplification
        .outputVoltageMin = 0.0,        // This config has no amplification or offset.
        .outputVoltageOffset = 1.65,    // The middle voltage is 3.3V/2, since that's our normalized 0 level.
        .name = "NO_GAIN"               // No gain here
    },
};

scpi_result_t SCPI_setVoltage(scpi_t * context){
    // CONF:VOLT:CHAN:DC 0,1.65
    if (xSemaphoreTake(serialMutex, 10*portTICK_PERIOD_MS) == pdTRUE) {
        printf("CONFigure:VOLTage:CHANnel:DC called\r\n");
        xSemaphoreGive(serialMutex);
    }
    uint32_t channel = 0;
    if (SCPI_ParamUInt32(context, &channel, TRUE)) {
        if (xSemaphoreTake(serialMutex, 10*portTICK_PERIOD_MS) == pdTRUE) {
            printf("Requested Channel: %i\r\n", channel);
            xSemaphoreGive(serialMutex);
        }
    }
    double requestedVoltage = 0.0;
    if (SCPI_ParamDouble(context, &requestedVoltage, TRUE)) {
        if (xSemaphoreTake(serialMutex, 10*portTICK_PERIOD_MS) == pdTRUE) {
            printf("Requested voltage: %f\r\n", requestedVoltage);
            xSemaphoreGive(serialMutex);
        }
    }
    // Make a copy of the channel config
    pwm_channel_config_t pwmChannelConfigCopy;
    if (xSemaphoreTake(PWMConfigMutex, 1000*portTICK_PERIOD_MS) == pdTRUE) {
        // Find the config for the channel and copy over to a local copy.
        bool channelFound = false;
        for(int i = 0 ; i < NUM_PWM_CHANNELS ; i++){
            if(pwm_config_table[i].output_channel_num == channel){
                memcpy(&pwmChannelConfigCopy, &(pwm_config_table[i]), sizeof(pwm_channel_config_t));
                channelFound = true;
                break;
            }
        }
        // Give back the semaphore
        xSemaphoreGive(PWMConfigMutex);
        
        // Check if the channel wasn't found
        if(!channelFound){
            return SCPI_RES_ERR;
        }
    
        // Based on the requested voltage, calculate the normalized voltage DC offset
        // This is going to be the DC offset in volts from the bottom divided by the full scope of the 
        // output voltage. So a DC offset of 1.65V on a 3.3V span is 1.65-0/(3.3-0) = 0.5. 
        // For something like a +/-10V, a 5V offset is going to be (5-(-10))/(10-(-10)) = 15/20
        pwmChannelConfigCopy.dc_offset = (requestedVoltage-hardwareSettings[hardwareSettingIndex].outputVoltageMin)/(hardwareSettings[hardwareSettingIndex].outputVoltageMax-hardwareSettings[hardwareSettingIndex].outputVoltageMin);
        
        // Send the config to the PWM handler:
        xQueueSend(pwmSettingsQueue, &pwmChannelConfigCopy, portMAX_DELAY);
    }
    else{
        printf("ERROR: UNABLE TO OBTAIN PWMConfigMutex SEMAPHORE IN SCPI_setVoltage");
        while(1);
        return SCPI_RES_ERR;
    }
    // Change the pwmChannelConfigCopy 


    // TODO: take the config mutex and set the parameters in the pwm config, and trigger an update
    return SCPI_RES_OK;
}



// SCPI command bindings
scpi_command_t scpi_commands[SCPI_MAX_NUM_COMMANDS] = {
    // IEEE Mandated Commands (SCPI std V1999.0 4.1.1)
    { .pattern = "*CLS", .callback = SCPI_CoreCls,},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    { .pattern = "*RST", .callback = SCPI_CoreRst,},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    { .pattern = "*TST?", .callback = NULL,}, // TODO: Implement this self test command.
    { .pattern = "*WAI", .callback = SCPI_CoreWai,},

	// Configure voltage DC
	{ .pattern = "CONFigure:VOLTage:CHANnel:DC", .callback = SCPI_setVoltage,},
    { .pattern = "CONFigure:VOLTage:CHANnel:DC?", .callback = NULL,},

    // Configure voltage AC
	{ .pattern = "CONFigure:VOLTage:CHANnel:AC", .callback = NULL,},
    { .pattern = "CONFigure:VOLTage:CHANnel:AC?", .callback = NULL,},

    // Configure frequency AC
	{ .pattern = "CONFigure:FREQuency:CHANnel:HZ", .callback = NULL,},
    { .pattern = "CONFigure:FREQuency:CHANnel:HZ?", .callback = NULL,},

    // Configure phase AC
	{ .pattern = "CONFigure:PHASe:CHANnel:DEGrees", .callback = NULL,},
    { .pattern = "CONFigure:PHASe:CHANnel:DEGrees?", .callback = NULL,},

    // Configure function
	{ .pattern = "CONFigure:FUNCtion:CHANnel", .callback = NULL,},
    { .pattern = "CONFigure:FUNCtion:CHANnel?", .callback = NULL,},

    // End of list
	SCPI_CMD_LIST_END
};

size_t scpi_return_successful(scpi_t * context, const char * data, size_t len) {
    (void) context;
    if (xSemaphoreTake(serialMutex, 10*portTICK_PERIOD_MS) == pdTRUE) {
        printf("%.*s", len, data);
        xSemaphoreGive(serialMutex);
    }
    return 0;//fwrite(data, 1, len, stdout);
}

int scpiErrorHandler(scpi_t * context, int_fast16_t err) {
    (void) context;
    if (xSemaphoreTake(serialMutex, 10*portTICK_PERIOD_MS) == pdTRUE) {
        printf("myError called. Error: %i\r\n", err);
        xSemaphoreGive(serialMutex);
    }
    return 0;//fwrite(data, 1, len, stdout);
}

// SCPI interfaces
scpi_interface_t scpi_interface = {
	.write = scpi_return_successful,
	.error = scpiErrorHandler,
	.reset = NULL,
};

/**
 * @brief This hook is called by FreeRTOS when an stack overflow error is
 * detected.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    while (1) {}
}

/**
 * @brief This hook is called by FreeRTOS when an stack overflow error is
 * detected.
 */
void vApplicationMallocFailedHook(void) {
    while (1) {}
}

//! Maximum number of characters for the serial input buffer
#define MAX_NUM_INPUT_CHARS 255
//! Bitmask for char ready in USB buffer
#define USB_NEW_DATA_IN 1 << 0
//! Bitmask for string ready in rx queue
#define USB_STRING_READY 1 << 1
//! Queue handle for the UART receive queue
QueueHandle_t uartReceiveQueue;
//! Event group handle for the USB read event
EventGroupHandle_t usbReadEvent;
//! SCPI context
scpi_t scpi_context;

/**
 * @brief Callback for when a character is available in the USB buffer
 * @return Nothing
 */
void stdio_callback(void *param) {
    xEventGroupSetBits(usbReadEvent, USB_NEW_DATA_IN);
}

/**
 * @brief Task that reads the USB buffer and puts the characters in a queue
 * @param p Unused
 */
static void usbReadTask(void *p) {
    while(1){
        uint32_t eventbits = xEventGroupWaitBits(
            usbReadEvent, USB_NEW_DATA_IN, pdTRUE, pdFALSE, portMAX_DELAY);
        if (eventbits & USB_NEW_DATA_IN) {
            char rxChar = getchar_timeout_us(100); // Read the input character
            if(rxChar == '\n' || rxChar == '\r'){
                xEventGroupSetBits(usbReadEvent, USB_STRING_READY);
            }
            else{
                xQueueSend(uartReceiveQueue, &rxChar, portMAX_DELAY);
            }
        }
    }
}

/**
 * @brief Task that handles the incoming strings to SCPI commands
 * @param p Unused
 */
static void scpiHandler(void *p){
    while(1){
        uint32_t eventbits = xEventGroupWaitBits(
            usbReadEvent, USB_STRING_READY, pdTRUE, pdFALSE, portMAX_DELAY);
        if (eventbits & USB_STRING_READY) {
            // Read the string from the queue
            char rxString[MAX_NUM_INPUT_CHARS] = {0};
            char* pRxString = rxString;
            while (xQueueReceive(uartReceiveQueue, pRxString++, (TickType_t)10)) {}
            *pRxString = '\0';
            if (xSemaphoreTake(serialMutex, 10*portTICK_PERIOD_MS) == pdTRUE) {
                printf("Received string: %s\n", rxString);
                xSemaphoreGive(serialMutex);
            }
            // Call the parser with the received string
            SCPI_Parse(&scpi_context, rxString, strlen(rxString));
        }
    }
}



/**
 * @brief Main function
 * @details Initializes the hardware and starts the FreeRTOS scheduler
 * @return 0
 */
int main() {
    // Init MCU hardware
    stdio_init_all();

    // Wait a minute to initialize the serial port
    sleep_ms(2000);

    // Initialize the SCPI library
    scpi_init(&scpi_context);

    // Create the semaphore to protect the serial port
    serialMutex = xSemaphoreCreateBinary();
    if (serialMutex == NULL) {
        while (1);
    }
    xSemaphoreGive(serialMutex);

    // Create the mutex for the PWM config array
    PWMConfigMutex = xSemaphoreCreateBinary();
    if (PWMConfigMutex == NULL) {
        while (1);
    }
    xSemaphoreGive(PWMConfigMutex);

    // Create the queue for the UART receive queue:
    uartReceiveQueue = xQueueCreate(MAX_NUM_INPUT_CHARS, sizeof(char));
    if (uartReceiveQueue == NULL) {
        while (1);
    }

    pwmSettingsQueue = xQueueCreate(MAX_NUM_INCOMING_SETTINGS, sizeof(pwm_channel_config_t));
    if (pwmSettingsQueue == NULL) {
        while (1);
    }
    
    // Incoming serial data event group and task
    usbReadEvent = xEventGroupCreate();
    TaskHandle_t usbReadTaskHandle = NULL;
    xTaskCreate(
        usbReadTask,           // Function that implements the task.
        "USB READ TASK",       // Text name for the task.
        300,                   // Stack size in words, not bytes.
        NULL,                  // Parameter passed into the task.
        tskIDLE_PRIORITY + 10, // Priority at which the task is created.
        &usbReadTaskHandle     // Used to pass out the created task's handle.
    );

    // SCPI handler task
    TaskHandle_t scpiHandlerHandle = NULL;
    xTaskCreate(
        scpiHandler,            // Function that implements the task.
        "SCPI HANDLER TASK",    // Text name for the task.
        1024,                   // Stack size in words, not bytes.
        NULL,                   // Parameter passed into the task.
        tskIDLE_PRIORITY + 5,   // Priority at which the task is created.
        &scpiHandlerHandle      // Used to pass out the created task's handle.
    );

    // SCPI handler task
    TaskHandle_t outputHandlerHandle = NULL;
    xTaskCreate(
        outputHandler,              // Function that implements the task.
        "OUTPUT HANDLER TASK",      // Text name for the task.
        1024,                       // Stack size in words, not bytes.
        NULL,                       // Parameter passed into the task.
        tskIDLE_PRIORITY + 1,       // Priority at which the task is created.
        &outputHandlerHandle        // Used to pass out the created task's handle.
    );

    // Set the USB callback:
    stdio_set_chars_available_callback(stdio_callback, NULL);

    // Start scheduler
    vTaskStartScheduler();
}