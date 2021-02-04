cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)
project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(rotary_inverted_pendulum
  main.c
)

pico_set_program_name(rotary_inverted_pendulum "rotary_inverted_pendulum")
pico_set_program_version(rotary_inverted_pendulum "0.1")

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(rotary_inverted_pendulum pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rotary_inverted_pendulum)

pico_enable_stdio_uart(rotary_inverted_pendulum 0)
pico_enable_stdio_usb(rotary_inverted_pendulum 1)

target_link_libraries(rotary_inverted_pendulum pico_stdlib)

pico_add_extra_outputs(rotary_inverted_pendulum)