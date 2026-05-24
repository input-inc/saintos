/* Bootloader-info descriptor.
 *
 * Lives at a fixed flash address (BOOTLOADER_INFO_ADDR, see
 * bootloader_info.h) so the runtime app can read which bootloader is
 * installed. Without this, a deployed board's bootloader is opaque —
 * the bootloader can't be OTA-updated, so visibility from the app is
 * the only way to know which boards in the field have which
 * bootloader code.
 *
 * Placement is enforced by bootloader_shell.ld via the .bl_info
 * section directive. The build-time version strings come from
 * `bl_version.h`, which CMakeLists.txt regenerates each build via
 * GenerateVersion.cmake (the same script the app uses for its own
 * version.h).
 */

#include "bootloader_info.h"
#include "bl_version.h"

__attribute__((section(".bl_info"), used))
const bootloader_info_t saint_bl_info = {
    .magic           = BOOTLOADER_INFO_MAGIC,
    .struct_version  = BOOTLOADER_INFO_STRUCT_VERSION,
    .version_string  = FIRMWARE_VERSION_FULL,
    .build_timestamp = FIRMWARE_BUILD_TIMESTAMP,
    .git_hash        = FIRMWARE_GIT_HASH,
    .build_unix      = FIRMWARE_BUILD_UNIX,
    ._reserved       = { 0, 0, 0 },
};
