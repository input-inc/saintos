# SAINT.OS Node Firmware for RP2040
#
# Build with micro-ROS and Pico SDK for Adafruit Feather RP2040 + Ethernet FeatherWing
#
# Prerequisites:
#   - Pico SDK (set PICO_SDK_PATH)
#   - micro-ROS Pico SDK (set MICRO_ROS_PATH)

cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK (must be before project())
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(saint_node C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Firmware version
set(FIRMWARE_VERSION_MAJOR 1)
set(FIRMWARE_VERSION_MINOR 0)
set(FIRMWARE_VERSION_PATCH 0)

# Configure version header
configure_file(
    ${CMAKE_SOURCE_DIR}/include/version.h.in
    ${CMAKE_BINARY_DIR}/generated/version.h
)

# =============================================================================
# micro-ROS Library
# =============================================================================

# Link to prebuilt micro-ROS library
link_directories($ENV{MICRO_ROS_PATH}/libmicroros)

# =============================================================================
# W5500 Ethernet Driver
# =============================================================================

add_library(w5500_driver STATIC
    transport/w5500_driver.c
)

target_include_directories(w5500_driver PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(w5500_driver
    pico_stdlib
    hardware_spi
)

# =============================================================================
# Main Executable
# =============================================================================

add_executable(saint_node
    src/main.c
    src/node_state.c
    src/hardware.c
    src/led_status.c
    src/pico_clock.c
    transport/transport_w5500.c
)

target_include_directories(saint_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated
    $ENV{MICRO_ROS_PATH}/libmicroros/include
)

# =============================================================================
# Pico SDK Libraries
# =============================================================================

target_link_libraries(saint_node
    pico_stdlib
    pico_unique_id
    hardware_spi
    hardware_gpio
    hardware_adc
    hardware_pwm
    hardware_pio
    w5500_driver
    microros
)

# Enable USB serial output
pico_enable_stdio_usb(saint_node 1)
pico_enable_stdio_uart(saint_node 0)

# Disable CRLF conversion for micro-ROS compatibility
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Compiler flags for size optimization
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# =============================================================================
# Build Outputs
# =============================================================================

# Create UF2 file for drag-and-drop flashing
pico_add_extra_outputs(saint_node)

# Print build info
message(STATUS "SAINT.OS Node Firmware v${FIRMWARE_VERSION_MAJOR}.${FIRMWARE_VERSION_MINOR}.${FIRMWARE_VERSION_PATCH}")
message(STATUS "Target: Adafruit Feather RP2040 + Ethernet FeatherWing")
message(STATUS "PICO_SDK_PATH: $ENV{PICO_SDK_PATH}")
message(STATUS "MICRO_ROS_PATH: $ENV{MICRO_ROS_PATH}")
