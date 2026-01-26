# GenerateVersion.cmake
# Generates version.h with current git hash and timestamp
# This is called at BUILD time, not configure time

# Get current timestamp
string(TIMESTAMP BUILD_DATE "%Y-%m-%d")
string(TIMESTAMP BUILD_TIME "%H:%M:%S")

# Get git hash
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${SOURCE_DIR}
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_RESULT
)

if(NOT GIT_RESULT EQUAL 0 OR NOT GIT_HASH)
    set(GIT_HASH "unknown")
endif()

# Check if there are uncommitted changes (dirty)
execute_process(
    COMMAND git diff --quiet HEAD
    WORKING_DIRECTORY ${SOURCE_DIR}
    RESULT_VARIABLE GIT_DIRTY_RESULT
)

if(NOT GIT_DIRTY_RESULT EQUAL 0)
    set(GIT_HASH "${GIT_HASH}-dirty")
endif()

# Version suffix (includes git hash for simulation/dev builds)
if(SIMULATION)
    set(VERSION_SUFFIX "-${GIT_HASH}")
else()
    set(VERSION_SUFFIX "")
endif()

# Generate version.h content
set(VERSION_CONTENT "/**
 * SAINT.OS Node Firmware - Version Information
 *
 * This file is auto-generated at build time. Do not edit directly.
 */

#ifndef SAINT_VERSION_H
#define SAINT_VERSION_H

#define FIRMWARE_VERSION_MAJOR ${VERSION_MAJOR}
#define FIRMWARE_VERSION_MINOR ${VERSION_MINOR}
#define FIRMWARE_VERSION_PATCH ${VERSION_PATCH}

#define FIRMWARE_VERSION_STRING \"${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}\"

// Build timestamp (generated at build time)
#define FIRMWARE_BUILD_DATE \"${BUILD_DATE}\"
#define FIRMWARE_BUILD_TIME \"${BUILD_TIME}\"
#define FIRMWARE_BUILD_TIMESTAMP \"${BUILD_DATE} ${BUILD_TIME}\"

// Git commit hash (includes -dirty if uncommitted changes)
#define FIRMWARE_GIT_HASH \"${GIT_HASH}\"

// Full version string with build info
#define FIRMWARE_VERSION_FULL \"${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}${VERSION_SUFFIX}\"

#define HARDWARE_MODEL \"Adafruit Feather RP2040 + Ethernet FeatherWing\"

#endif // SAINT_VERSION_H
")

# Write the file
file(WRITE ${OUTPUT_FILE} "${VERSION_CONTENT}")

message(STATUS "Generated version.h: v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}${VERSION_SUFFIX} (${BUILD_DATE} ${BUILD_TIME})")
