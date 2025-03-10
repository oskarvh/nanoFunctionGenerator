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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi_interface.h"

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];

scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];


void scpi_init(scpi_t *pContext){
    // Initialize the SCPI library
    SCPI_Init(pContext,
        scpi_commands,
        &scpi_interface,
        scpi_units_def,
        SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
        scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
        scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
}