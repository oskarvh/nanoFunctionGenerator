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
#ifndef SCPI_INTERFACE_H
#define SCPI_INTERFACE_H
//! This devices identity
#define SCPI_IDN1 "OSKARVH"
#define SCPI_IDN2 "NANO_FUNCTION_GENERATOR"
#define SCPI_IDN3 "https://github.com/oskarvh/nanoFunctionGenerator"
#define SCPI_IDN4 "00-01"

//! SCPI command length
#define SCPI_MAX_NUM_COMMANDS 30

//! SCPI command definitions
extern scpi_command_t scpi_commands[SCPI_MAX_NUM_COMMANDS];

//! SCPI interface definition
extern scpi_interface_t scpi_interface;

//! SCPI input buffer length
#define SCPI_INPUT_BUFFER_LENGTH 256
//! SCPI input buffer
extern char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];

//! SCPI error buffer length
#define SCPI_ERROR_QUEUE_SIZE 17
//! SCPI error buffer
extern scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

/**
 * Set register bits
 * @param pContext - Pointer to the SCPI library context
 */
void scpi_init(scpi_t *pContext);

#endif // SCPI_INTERFACE_H