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
#include "task.h"
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

//! Mutex to protect the configuration
xSemaphoreHandle PWMConfigMutex;

scpi_result_t SCPI_setVoltage(scpi_t * context){
    // CONF:VOLT:CHAN:DC 0,3.3
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
void stdio_callback(void) {
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

static void outputHandler(void *p){
    // Initialize the PWM hardware
    // int config = pwm_get_default_config();
    // pwm_init(pwm_gpio_to_slice_num(26), &config, true);
    // pwm_set_clkdiv(pwm_gpio_to_slice_num(26), 16.0f);

    while(1){
        
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

    // Init the PWMs
    init_pwm();

    // Create the semaphore to protect the serial port
    serialMutex = xSemaphoreCreateBinary();
    if (serialMutex == NULL) {
        while (1);
    }
    xSemaphoreGive(serialMutex);

    // Create the queue for the UART receive queue:
    uartReceiveQueue = xQueueCreate(MAX_NUM_INPUT_CHARS, sizeof(char));
    if (uartReceiveQueue == NULL) {
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