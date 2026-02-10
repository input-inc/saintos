/**
 * SAINT.OS Firmware - Platform Abstraction
 *
 * Each platform must provide its own platform.h that defines:
 *   PLATFORM_SLEEP_MS(ms)  - Sleep for milliseconds
 *   PLATFORM_MILLIS()      - Current time in milliseconds
 *   PLATFORM_PRINTF        - printf function
 *
 * This file is not meant to be included directly.
 * Include the platform-specific version from the firmware's include/ directory.
 */

#ifndef PLATFORM_H
#error "Include the platform-specific platform.h, not the shared stub"
#endif
