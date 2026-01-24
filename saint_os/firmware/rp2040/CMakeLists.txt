# SAINT.OS Node Firmware for RP2040
#
# Build with micro-ROS and Pico SDK for Adafruit Feather RP2040 + Ethernet FeatherWing
#
# Prerequisites:
#   - Pico SDK
#   - micro-ROS for Pico
#   - W5500 ethernet library

cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK (must be before project())
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Include micro-ROS CMake helpers
include($ENV{MICRO_ROS_PATH}/micro_ros_cmake.cmake)

project(saint_node C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
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
# Main Executable
# =============================================================================

add_executable(saint_node
    src/main.c
    src/node_state.c
    src/hardware.c
    src/led_status.c
    transport/transport_w5500.c
    transport/w5500_spi.c
)

target_include_directories(saint_node PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated
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
)

# Enable USB serial output
pico_enable_stdio_usb(saint_node 1)
pico_enable_stdio_uart(saint_node 0)

# =============================================================================
# micro-ROS
# =============================================================================

# Link micro-ROS
target_link_libraries(saint_node
    microros
)

# Add micro-ROS message dependencies
# These are generated from the saint_os package messages
target_link_libraries(saint_node
    saint_os__rosidl_typesupport_c
)

# =============================================================================
# W5500 Ethernet
# =============================================================================

# W5500 driver library
add_library(w5500_driver STATIC
    transport/w5500_driver.c
)

target_include_directories(w5500_driver PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(w5500_driver
    pico_stdlib
    hardware_spi
)

target_link_libraries(saint_node w5500_driver)

# =============================================================================
# Build Outputs
# =============================================================================

# Create UF2 file for drag-and-drop flashing
pico_add_extra_outputs(saint_node)

# Print build info
message(STATUS "SAINT.OS Node Firmware v${FIRMWARE_VERSION_MAJOR}.${FIRMWARE_VERSION_MINOR}.${FIRMWARE_VERSION_PATCH}")
message(STATUS "Target: Adafruit Feather RP2040 + Ethernet FeatherWing")
