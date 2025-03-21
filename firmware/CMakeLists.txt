cmake_minimum_required(VERSION 3.27)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

## ---------- DEPENDENCIES ----------
# Pull in dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    freertos_kernel
    GIT_REPOSITORY "https://github.com/FreeRTOS/FreeRTOS-Kernel.git"
    GIT_TAG "V10.6.1"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
FetchContent_Declare(
    scpi_parser
    GIT_REPOSITORY "https://github.com/j123b567/scpi-parser.git"
    GIT_TAG "v2.3"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)
# Fetch the content without building it
FetchContent_Populate(freertos_kernel)
FetchContent_Populate(scpi_parser)
message(STATUS "freertos_kernel source dir: ${freertos_kernel_SOURCE_DIR}")
message(STATUS "scpi_parser source dir: ${scpi_parser_SOURCE_DIR}")

## ---------- PROJECT ----------
# Define the project
project(nanoFunctionGenerator_firmware C CXX ASM)

# Check the pico sdk version
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the pico SDK
pico_sdk_init()

## ---------- TARGETS ----------
# Add FreeRTOS as a library
add_library(freertos STATIC
    ${freertos_kernel_SOURCE_DIR}/event_groups.c
    ${freertos_kernel_SOURCE_DIR}/list.c
    ${freertos_kernel_SOURCE_DIR}/queue.c
    ${freertos_kernel_SOURCE_DIR}/stream_buffer.c
    ${freertos_kernel_SOURCE_DIR}/tasks.c
    ${freertos_kernel_SOURCE_DIR}/timers.c
    ${freertos_kernel_SOURCE_DIR}/portable/MemMang/heap_3.c
    ${freertos_kernel_SOURCE_DIR}/portable/GCC/ARM_CM0/port.c
)
target_include_directories(freertos PUBLIC
    ${CMAKE_SOURCE_DIR}/firmware/
    ${freertos_kernel_SOURCE_DIR}/include/
    ${freertos_kernel_SOURCE_DIR}/portable/GCC/ARM_CM0/
)

# Add the scpi-parser as a library
add_library(scpi_lib STATIC
    ${scpi_parser_SOURCE_DIR}/libscpi/src/parser.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/minimal.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/ieee488.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/utils.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/units.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/error.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/lexer.c
    ${scpi_parser_SOURCE_DIR}/libscpi/src/fifo.c
)
target_include_directories(scpi_lib PUBLIC
    ${scpi_parser_SOURCE_DIR}/libscpi/src/
    ${scpi_parser_SOURCE_DIR}/libscpi/inc/
    ${scpi_parser_SOURCE_DIR}/libscpi/src/
)

add_executable(
    nanoFunctionGenerator_firmware 
    ${CMAKE_SOURCE_DIR}/firmware/main.c
    ${CMAKE_SOURCE_DIR}/firmware/scpi_interface.c
    ${CMAKE_SOURCE_DIR}/firmware/pwm_output.c
)
target_link_libraries(
    nanoFunctionGenerator_firmware 
    PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_dma
    freertos
    scpi_lib
)
target_include_directories(nanoFunctionGenerator_firmware PUBLIC 
    ${CMAKE_SOURCE_DIR}/firmware/
    ${freertos_kernel_SOURCE_DIR}/include/
)

pico_enable_stdio_usb(nanoFunctionGenerator_firmware 1)
pico_enable_stdio_uart(nanoFunctionGenerator_firmware 1)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()