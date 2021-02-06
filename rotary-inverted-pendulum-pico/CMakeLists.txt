cmake_minimum_required(VERSION 3.13)

if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same.")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico-sdk/pico_sdk_init.cmake)
project(my_project)

pico_sdk_init()

add_executable(rotary_inverted_pendulum
  main.c
)

target_compile_options(
  rotary_inverted_pendulum PRIVATE
  -Wall
)

pico_set_program_name(rotary_inverted_pendulum "rotary_inverted_pendulum")
pico_set_program_version(rotary_inverted_pendulum "0.1")

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rotary_inverted_pendulum)

pico_enable_stdio_uart(rotary_inverted_pendulum 0)
pico_enable_stdio_usb(rotary_inverted_pendulum 1)

target_link_libraries(rotary_inverted_pendulum pico_stdlib hardware_i2c)
