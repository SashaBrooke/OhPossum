cmake_minimum_required(VERSION 3.19)

include(pico_sdk_import.cmake)

project(ohpossum-controls)

pico_sdk_init()

add_executable(ohpossum-controls main.c)

target_link_libraries(ohpossum-controls pico_stdlib)

# generate .uf2 .elf .bin
pico_add_extra_outputs(ohpossum-controls)