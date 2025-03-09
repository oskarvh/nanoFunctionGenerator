/*
MIT License

Copyright (c) 2023 Oskar von Heideken

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

// RP2040 stdlib
#include "pico/stdlib.h"


/**
 * @brief This hook is called by FreeRTOS when an stack overflow error is
 * detected.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    // UARTprintf("\n\n======================WARNING======================\n");
    // UARTprintf("Task %s had a stack overflow :(", pcTaskName);
    // UARTprintf("\n\n===================================================\n");
    while (1) {
    }
}

/**
 * @brief This hook is called by FreeRTOS when an stack overflow error is
 * detected.
 */
void vApplicationMallocFailedHook(void) {
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    // UARTprintf("\n\n======================WARNING======================\n");
    // UARTprintf("Task %s had a stack overflow :(", pcTaskName);
    // UARTprintf("\n\n===================================================\n");
    while (1) {
    }
}
void mainThread(void *pvParameters) {
    // Print a message
    printf("Hello, world!\n");

    // Loop forever
    while (1) {
        // Print a message
        printf("Hello, world!\n");
        // Sleep for a second
        sleep_ms(1000);
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