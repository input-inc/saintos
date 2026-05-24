# GenerateVersion.cmake
# Generates version.h with unix timestamp for unique build identification
# This is called at BUILD time, not configure time

# Get current timestamp in human-readable format
string(TIMESTAMP BUILD_DATE "%Y-%m-%d")
string(TIMESTAMP BUILD_TIME "%H:%M:%S")

# Get unix timestamp for unique build identification
string(TIMESTAMP BUILD_UNIX "%s")

# Get git hash (still useful for reference, but not used in version string)
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

# Check if there are uncommitted changes (dirty) - informational only
execute_process(
    COMMAND git diff --quiet HEAD
    WORKING_DIRECTORY ${SOURCE_DIR}
    RESULT_VARIABLE GIT_DIRTY_RESULT
)

set(GIT_DIRTY "false")
if(NOT GIT_DIRTY_RESULT EQUAL 0)
    set(GIT_DIRTY "true")
endif()

# Version suffix - use unix timestamp for unique build detection
set(VERSION_SUFFIX "-${BUILD_UNIX}")

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
#define FIRMWARE_BUILD_UNIX ${BUILD_UNIX}

// Git info (for reference only, not used in version comparison)
#define FIRMWARE_GIT_HASH \"${GIT_HASH}\"
#define FIRMWARE_GIT_DIRTY ${GIT_DIRTY}

// Full version string with unix timestamp for unique build identification
#define FIRMWARE_VERSION_FULL \"${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}${VERSION_SUFFIX}\"

#define HARDWARE_MODEL \"Adafruit Feather RP2040 + Ethernet FeatherWing\"

#endif // SAINT_VERSION_H
")

# Write the file
file(WRITE ${OUTPUT_FILE} "${VERSION_CONTENT}")

message(STATUS "Generated version.h: v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}${VERSION_SUFFIX} (${BUILD_DATE} ${BUILD_TIME}, git: ${GIT_HASH})")
