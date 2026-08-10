# SAINT.OS — Pimoroni Servo 2040 board firmware (I2C target)
#
# Standalone Pico-SDK + pimoroni-pico project. NOT part of the SAINT.OS
# node build (rp2040/teensy41) — this image is flashed onto the Servo
# 2040 board itself, once, by hand. See README.md.
#
# Build:
#   export PICO_SDK_PATH=/path/to/pico-sdk
#   export PIMORONI_PICO_PATH=/path/to/pimoroni-pico
#   cmake -B build -DPICO_BOARD=pico -DPICO_SDK_PATH=$PICO_SDK_PATH \
#         -DPIMORONI_PICO_PATH=$PIMORONI_PICO_PATH
#   cmake --build build
# Produces build/saint_servo2040.uf2 — drag onto the RP2040 BOOTSEL drive.

cmake_minimum_required(VERSION 3.12)

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

project(saint_servo2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Pimoroni drivers. servo2040.cmake pulls in servo, servo_cluster, and
# plasma (WS2812); analog/analogmux/button are the current-sense stack.
include(libraries/servo2040/servo2040.cmake)
include(drivers/analog/analog.cmake)
include(drivers/analogmux/analogmux.cmake)
include(drivers/button/button.cmake)

add_executable(saint_servo2040 main.cpp)

# The shared wire-contract header, single source of truth with the host
# driver (firmware/shared/src/pimoroni_servo2040_driver.c).
target_include_directories(saint_servo2040 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../shared/include)

target_link_libraries(saint_servo2040
    pico_stdlib
    hardware_i2c
    hardware_flash
    servo2040
    analog
    analogmux
    button)

# The host link is I2C on the Qwiic pins (GP20/21); keep stdio off the
# servo/UART pins entirely (USB only) so nothing fights the servos.
pico_enable_stdio_usb(saint_servo2040 1)
pico_enable_stdio_uart(saint_servo2040 0)
pico_add_extra_outputs(saint_servo2040)
