# SAINT.OS Node Firmware for RP2040
#
# Build with micro-ROS and Pico SDK for Adafruit Feather RP2040 + Ethernet FeatherWing
#
# Prerequisites:
#   - Pico SDK (set PICO_SDK_PATH or place in lib/pico-sdk)
#   - micro-ROS Pico SDK (included in lib/micro_ros_raspberrypi_pico_sdk)

cmake_minimum_required(VERSION 3.13)

# Set PICO_SDK_PATH if not already set
if(NOT DEFINED ENV{PICO_SDK_PATH})
    if(EXISTS "${CMAKE_SOURCE_DIR}/lib/pico-sdk")
        set(ENV{PICO_SDK_PATH} "${CMAKE_SOURCE_DIR}/lib/pico-sdk")
    elseif(EXISTS "$ENV{HOME}/pico-sdk")
        set(ENV{PICO_SDK_PATH} "$ENV{HOME}/pico-sdk")
    endif()
endif()

# Set MICRO_ROS_PATH - prefer local lib directory
if(EXISTS "${CMAKE_SOURCE_DIR}/lib/micro_ros_raspberrypi_pico_sdk")
    set(MICRO_ROS_PATH "${CMAKE_SOURCE_DIR}/lib/micro_ros_raspberrypi_pico_sdk")
elseif(DEFINED ENV{MICRO_ROS_PATH})
    set(MICRO_ROS_PATH "$ENV{MICRO_ROS_PATH}")
else()
    message(FATAL_ERROR "micro-ROS SDK not found. Place it in lib/micro_ros_raspberrypi_pico_sdk or set MICRO_ROS_PATH")
endif()

# Initialize Pico SDK (must be before project())
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(saint_node C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Firmware version
set(FIRMWARE_VERSION_MAJOR 1)
set(FIRMWARE_VERSION_MINOR 2)
set(FIRMWARE_VERSION_PATCH 0)

# Build option for Renode simulation (uses UART instead of USB)
option(SIMULATION "Build for Renode simulation with UART output" OFF)

# Create directory for generated files
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/generated)

# Generate version header at build time (not configure time)
# This ensures git hash and timestamp are always current
add_custom_target(generate_version ALL
    COMMAND ${CMAKE_COMMAND}
        -DVERSION_MAJOR=${FIRMWARE_VERSION_MAJOR}
        -DVERSION_MINOR=${FIRMWARE_VERSION_MINOR}
        -DVERSION_PATCH=${FIRMWARE_VERSION_PATCH}
        -DSIMULATION=${SIMULATION}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${CMAKE_BINARY_DIR}/generated/version.h
        -P ${CMAKE_SOURCE_DIR}/cmake/GenerateVersion.cmake
    COMMENT "Generating version.h with current git hash and timestamp"
)

# =============================================================================
# micro-ROS Library
# =============================================================================

# Link to prebuilt micro-ROS library
link_directories(${MICRO_ROS_PATH}/libmicroros)

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
    src/pin_config.c
    src/pin_control.c
)

# Add transport based on build mode
if(SIMULATION)
    list(APPEND SAINT_NODE_SOURCES transport/transport_udp_bridge.c)
else()
    list(APPEND SAINT_NODE_SOURCES transport/transport_w5500.c)
endif()

add_executable(saint_node ${SAINT_NODE_SOURCES})

# Ensure version.h is generated before compiling
add_dependencies(saint_node generate_version)

target_include_directories(saint_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated
    ${MICRO_ROS_PATH}/libmicroros/include
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

# =============================================================================
# Install Targets
# =============================================================================

# Install directory for simulation firmware
set(SIM_INSTALL_DIR "${CMAKE_SOURCE_DIR}/install/simulation")

# Custom target to install simulation firmware
# Usage: make install_sim
add_custom_target(install_sim
    COMMAND ${CMAKE_COMMAND} -E make_directory "${SIM_INSTALL_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/saint_node.elf" "${SIM_INSTALL_DIR}/saint_node.elf"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/generated/version.h" "${SIM_INSTALL_DIR}/version.h"
    COMMAND ${CMAKE_COMMAND} -E echo "Installed simulation firmware to ${SIM_INSTALL_DIR}"
    DEPENDS saint_node
    COMMENT "Installing simulation firmware to ${SIM_INSTALL_DIR}"
)

# Print build info
message(STATUS "SAINT.OS Node Firmware v${FIRMWARE_VERSION_MAJOR}.${FIRMWARE_VERSION_MINOR}.${FIRMWARE_VERSION_PATCH}")
message(STATUS "Target: Adafruit Feather RP2040 + Ethernet FeatherWing")
message(STATUS "PICO_SDK_PATH: $ENV{PICO_SDK_PATH}")
message(STATUS "MICRO_ROS_PATH: ${MICRO_ROS_PATH}")
