cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)

project(ohpossum-controls)

pico_sdk_init()

add_executable(ohpossum-controls 
    encoder.c
    PID.c
)

target_link_libraries(ohpossum-controls 
    pico_stdlib
    hardware_i2c
)

target_include_directories(ohpossum-controls 
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}
)

pico_enable_stdio_usb(ohpossum-controls 1)
pico_enable_stdio_uart(ohpossum-controls 0)

# generate .uf2 .elf .bin
pico_add_extra_outputs(ohpossum-controls)