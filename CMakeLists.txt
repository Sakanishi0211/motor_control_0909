cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(motor_control C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(motor_control
        motor_control.c
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(motor_control pico_stdlib hardware_uart hardware_pwm)

# enable usb output, disable uart output
pico_enable_stdio_usb(motor_control 1)
pico_enable_stdio_uart(motor_control 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(motor_control)
