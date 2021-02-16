cmake_minimum_required(VERSION 3.13)

if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same.")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico-sdk/pico_sdk_init.cmake)
project(rotary_inverted_pendulum)


pico_sdk_init()

file(GLOB_RECURSE LIB_SOURCE
  lib/*.c*
)

add_executable(rotary_inverted_pendulum
  main.c
  lib/motor_driver/motor_driver.c
  lib/arm_encoder/arm_encoder.c
  lib/motor_encoder/motor_encoder.c
  lib/pid_controller/pid_controller.c
  lib/cmsis/math_helper.c
)

target_include_directories(
  rotary_inverted_pendulum PRIVATE 
  lib/motor_driver 
  lib/motor_encoder
  lib/pid_controller
  lib/arm_encoder
  lib/cmsis/include/core
  lib/cmsis/include/dsp
  )

target_compile_definitions(rotary_inverted_pendulum PUBLIC ARM_MATH_CM0PLUS)

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

target_link_libraries(
  rotary_inverted_pendulum
  pico_stdlib
  hardware_i2c
  hardware_pwm
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPTransform.a
  ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPBasicMath.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPBayes.a
  ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPCommon.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPComplexMath.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPController.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPDistance.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPFastMath.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPFiltering.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPInterpolation.a
  ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPMatrix.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPQuaternionMath.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPStatistics.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPSupport.a
  # ${CMAKE_SOURCE_DIR}/lib/cmsis/libCMSISDSPSVM.a
)
