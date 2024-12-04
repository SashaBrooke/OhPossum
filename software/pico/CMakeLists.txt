cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)

project(ohpossum-controls)

pico_sdk_init()

set(SOURCES
    src/encoder.c
    src/PID.c
)

# Optional: Automatically collect all .c files in the src/ directory
# file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.c)

add_executable(ohpossum-controls 
    ${SOURCES}
)

target_link_libraries(ohpossum-controls
    pico_stdlib
    hardware_i2c
)

target_include_directories(ohpossum-controls 
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_enable_stdio_usb(ohpossum-controls 1)
pico_enable_stdio_uart(ohpossum-controls 0)

# moves everything except the .uf2 (only output used) into /bin
set_target_properties(ohpossum-controls PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# generate .uf2 .elf .bin
pico_add_extra_outputs(ohpossum-controls)