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

# Build option for Renode simulation (uses UART instead of USB)
option(SIMULATION "Build for Renode simulation with UART output" OFF)

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

# Common source files
set(SAINT_NODE_SOURCES
    src/main.c
    src/node_state.c
    src/hardware.c
    src/led_status.c
    src/pico_clock.c
    src/flash_storage.c
)

# Add transport based on build mode
if(SIMULATION)
    list(APPEND SAINT_NODE_SOURCES transport/transport_udp_bridge.c)
else()
    list(APPEND SAINT_NODE_SOURCES transport/transport_w5500.c)
endif()

add_executable(saint_node ${SAINT_NODE_SOURCES})

target_include_directories(saint_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated
    $ENV{MICRO_ROS_PATH}/libmicroros/include
)

# =============================================================================
# Pico SDK Libraries
# =============================================================================

# Base libraries
target_link_libraries(saint_node
    pico_stdlib
    pico_unique_id
    hardware_spi
    hardware_gpio
    hardware_adc
    hardware_pwm
    hardware_pio
    hardware_flash
    hardware_sync
    microros
)

# Add W5500 driver only for hardware builds
if(NOT SIMULATION)
    target_link_libraries(saint_node w5500_driver)
endif()

# Configure stdio output and defines based on build target
if(SIMULATION)
    # Renode simulation: use UART (USB not emulated)
    pico_enable_stdio_usb(saint_node 0)
    pico_enable_stdio_uart(saint_node 1)
    # Define SIMULATION for conditional compilation in source
    target_compile_definitions(saint_node PRIVATE SIMULATION=1)
    message(STATUS "Build mode: SIMULATION (UART output, serial transport)")
else()
    # Real hardware: use USB serial
    pico_enable_stdio_usb(saint_node 1)
    pico_enable_stdio_uart(saint_node 0)
    message(STATUS "Build mode: HARDWARE (USB output, W5500 transport)")
endif()

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
