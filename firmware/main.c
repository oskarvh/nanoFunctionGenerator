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

// RP2040 stdlib
#include "pico/stdlib.h"

// SCPI library
#include "scpi/scpi.h"
#include "scpi_interface.h"

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
	{ .pattern = "CONFigure:VOLTage:CHANnel:DC", .callback = NULL,},
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

size_t myWrite(scpi_t * context, const char * data, size_t len) {
    (void) context;
    printf("%.*s", len, data);
    return 0;//fwrite(data, 1, len, stdout);
}

// SCPI interfaces
scpi_interface_t scpi_interface = {
	.write = myWrite, //TODO: Implement this function
	.error = NULL,
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

#define USB_NEW_DATA_IN 1 << 0
#define NEW_STRING_IN 1 << 0
QueueHandle_t uartReceiveQueue;
EventGroupHandle_t usbReadEvent;
EventGroupHandle_t usbStringReceivedEvent;
/**
 * @brief Callback for when a character is available in the USB buffer
 * @return Nothing
 */
void stdio_callback(void) {
    xEventGroupSetBits(usbReadEvent, USB_NEW_DATA_IN);
}



static void usbReadTask(void *p) {
    while(1){
        uint32_t eventbits = xEventGroupWaitBits(
            usbReadEvent, USB_NEW_DATA_IN, pdTRUE, pdFALSE, portMAX_DELAY);
        if (eventbits & USB_NEW_DATA_IN) {
            char rxChar = getchar_timeout_us(100); // Read the input character
            xQueueSend(uartReceiveQueue, &rxChar, portMAX_DELAY);
            if(rxChar == '\n' || rxChar == '\r'){
                xEventGroupSetBits(usbStringReceivedEvent, NEW_STRING_IN);
            }
            else{
                printf("USB data in event. Read %c\n", rxChar);
            }
        }
    }
}

static void usbStringRead(void *p){
    while(1){
        uint32_t eventbits = xEventGroupWaitBits(
            usbStringReceivedEvent, NEW_STRING_IN, pdTRUE, pdFALSE, portMAX_DELAY);
        if (eventbits & NEW_STRING_IN) {
            // Read the string from the queue
            char rxString[100] = {0};
            char* pRxString = rxString;
            while (xQueueReceive(uartReceiveQueue, pRxString++, (TickType_t)10)) {}
            *pRxString = '\0';
            printf("Received string: %s\n", rxString);
        }
    }
}

void mainThread(void *pvParameters) {
    // Print a message
    printf("Hello, world!\n");

    // Create the queue for the UART receive queue:
    uartReceiveQueue = xQueueCreate(100, sizeof(char));
    if (uartReceiveQueue == NULL) {
        while (1)
            ;
    }
    // Initialize UART.
    // Initialize the USB data in event group
    usbReadEvent = xEventGroupCreate();
    usbStringReceivedEvent = xEventGroupCreate();
    TaskHandle_t usbReadTaskHandle = NULL;
    xTaskCreate(
        usbReadTask,          // Function that implements the task.
        "USB_READ_TASK",      // Text name for the task.
        300,                  // Stack size in words, not bytes.
        NULL,            // Parameter passed into the task.
        tskIDLE_PRIORITY + 2, // Priority at which the task is created.
        &usbReadTaskHandle    // Used to pass out the created task's handle.
    );

    TaskHandle_t usbReadStringHandle = NULL;
    xTaskCreate(
        usbStringRead,          // Function that implements the task.
        "USB STRING_READ_TASK",      // Text name for the task.
        300,                  // Stack size in words, not bytes.
        NULL,            // Parameter passed into the task.
        tskIDLE_PRIORITY + 1, // Priority at which the task is created.
        &usbReadStringHandle    // Used to pass out the created task's handle.
    );

    // Set the USB callback:
    stdio_set_chars_available_callback(stdio_callback, NULL);


    // Loop forever
    while (1) {
    }
}
/*
 * RUNTIME START
 */
int main() {
    // Init MCU hardware
    stdio_init_all();
    // Wait a minute to initialize the serial port
    sleep_ms(2000);

    // Start the main thread
    TaskHandle_t mainThreadHandle = NULL;
    xTaskCreate(mainThread, "MAIN_TASK", 200, (void *)1, tskIDLE_PRIORITY,
                &mainThreadHandle);

    // Start scheduler
    vTaskStartScheduler();
}