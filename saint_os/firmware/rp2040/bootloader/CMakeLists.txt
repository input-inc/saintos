# SAINT.OS RP2040 OTA Bootloader
#
# Builds:
#   - saint_ota_bootloader  : the bootloader binary (lives at 0x10000000,
#                             ~16 KB, programmed once per board via BOOTSEL)
#   - picowota_reboot       : tiny static lib the SAINT.OS app links against
#                             to enter OTA mode (set scratch + reboot)
#
# Macros for the parent firmware build:
#   - saint_ota_apply_app(<target>)
#       Apply the standalone linker script and image-header post-build to
#       <target> so it can run AFTER the bootloader at 0x10004000.
#   - saint_ota_build_combined(<app_target>)
#       Produce a combined .uf2 (bootloader + app) for first-time BOOTSEL
#       flashing. After that initial flash, future updates go over OTA.

cmake_minimum_required(VERSION 3.13)

# We assume this is included via add_subdirectory() from the parent
# firmware CMakeLists.txt, AFTER pico_sdk_init().

# Use a CACHE INTERNAL variable so the helper functions below resolve
# this path correctly when called from the parent CMakeLists.txt (where
# CMAKE_CURRENT_SOURCE_DIR would otherwise point at the parent dir).
set(SAINT_OTA_BL_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")
set(BL_DIR ${SAINT_OTA_BL_DIR})

# -----------------------------------------------------------------------------
# picowota_reboot — small static lib the APP links against
# -----------------------------------------------------------------------------
add_library(picowota_reboot STATIC
    ${SAINT_OTA_BL_DIR}/picowota_reboot/reboot.c
)
target_include_directories(picowota_reboot PUBLIC
    ${SAINT_OTA_BL_DIR}/picowota_reboot/include
)
target_link_libraries(picowota_reboot PUBLIC
    pico_stdlib
    hardware_watchdog
)

# -----------------------------------------------------------------------------
# Bootloader version header — regenerated each build via the same
# GenerateVersion.cmake script the app uses. The bootloader gets its own
# unix-timestamp build ID so we can tell two bootloader builds apart in
# the field, and the version numbers (BL_VERSION_*) are independent of
# the app's (FIRMWARE_VERSION_*) so a bootloader release can move on
# its own schedule. Output goes to the per-target generated/ dir so the
# bootloader and app version.h files don't collide.
# -----------------------------------------------------------------------------
set(BL_VERSION_MAJOR 1 CACHE STRING "Bootloader major version")
set(BL_VERSION_MINOR 0 CACHE STRING "Bootloader minor version")
set(BL_VERSION_PATCH 0 CACHE STRING "Bootloader patch version")

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bootloader/generated)

add_custom_target(generate_bl_version ALL
    COMMAND ${CMAKE_COMMAND}
        -DVERSION_MAJOR=${BL_VERSION_MAJOR}
        -DVERSION_MINOR=${BL_VERSION_MINOR}
        -DVERSION_PATCH=${BL_VERSION_PATCH}
        -DSIMULATION=OFF
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${CMAKE_BINARY_DIR}/bootloader/generated/bl_version.h
        -P ${CMAKE_SOURCE_DIR}/cmake/GenerateVersion.cmake
    COMMENT "Generating bl_version.h (bootloader)"
)

# -----------------------------------------------------------------------------
# saint_ota_bootloader — the OTA bootloader itself
# -----------------------------------------------------------------------------
add_executable(saint_ota_bootloader
    ${SAINT_OTA_BL_DIR}/main.c
    ${SAINT_OTA_BL_DIR}/bootloader_w5500_init.c
    ${SAINT_OTA_BL_DIR}/http_transport_w5500.c
    ${SAINT_OTA_BL_DIR}/bl_info.c
    # Shared HTTP client
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/src/http_client.c
)
add_dependencies(saint_ota_bootloader generate_bl_version)
target_include_directories(saint_ota_bootloader PRIVATE
    ${BL_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_BINARY_DIR}/bootloader/generated
)
target_link_libraries(saint_ota_bootloader
    pico_stdlib
    pico_unique_id
    cmsis_core             # exposes RP2040.h / SCB / NVIC / SysTick
    hardware_flash
    hardware_resets
    hardware_watchdog
    hardware_sync
    hardware_spi
    hardware_dma
    picowota_reboot
    wiznet_iolibrary       # provided by parent CMakeLists (transport/CMakeLists)
)
# Stdio over USB so we can capture bootloader logs with a serial monitor.
pico_enable_stdio_usb(saint_ota_bootloader 1)
pico_enable_stdio_uart(saint_ota_bootloader 0)

# Use the bootloader linker script (bootloader lives at 0x10000000).
pico_set_linker_script(saint_ota_bootloader ${SAINT_OTA_BL_DIR}/bootloader_shell.ld)

# Emit a linker map alongside the ELF — saint_ota_build_combined reads
# it to find the .app_bin section's actual flash address.
target_link_options(saint_ota_bootloader PRIVATE
    -Wl,-Map=$<TARGET_FILE:saint_ota_bootloader>.map
)

pico_add_extra_outputs(saint_ota_bootloader)

# -----------------------------------------------------------------------------
# Macros for the app build
# -----------------------------------------------------------------------------

# Apply the standalone linker script + image-header post-build to <target>.
# After this, <target>.bin starts at 0x10004000 and has a trailing image
# header that the bootloader can validate.
function(saint_ota_apply_app target)
    pico_set_linker_script(${target} ${SAINT_OTA_BL_DIR}/standalone.ld)

    # gen_imghdr.py reads the .bin, computes vtor/size/crc, and produces
    # an "imghdr.bin" sidecar — the bootloader stages this verbatim into
    # IMAGE_HEADER_ADDR after a successful download.
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}>
                $<TARGET_FILE_DIR:${target}>/${target}_app.bin
        COMMAND ${Python3_EXECUTABLE} ${SAINT_OTA_BL_DIR}/gen_imghdr.py
                $<TARGET_FILE_DIR:${target}>/${target}_app.bin
                $<TARGET_FILE_DIR:${target}>/${target}_imghdr.bin
        COMMENT "Generating OTA image header for ${target}"
        VERBATIM
    )
endfunction()

# Build a combined .uf2: bootloader's ELF, with its `.app_hdr` and
# `.app_bin` placeholder sections patched in-place to carry the real
# image-header (vtor/size/CRC32 from gen_imghdr.py) and the real app
# binary. Then convert the patched ELF to .uf2.
#
# This is the only safe way to produce a single first-flash artifact —
# the placeholder sections in the bootloader linker output (filled with
# 0xdeaddead) MUST be overwritten or the bootloader sees a bad header,
# fails image_header_ok(), and sits in OTA-wait forever.
function(saint_ota_build_combined app_target)
    # Use literal paths — generator expressions can't appear in
    # add_custom_command(OUTPUT ...) because the dependency graph is
    # built before they're evaluated. Both the app and the bootloader
    # land in the top-level build dir under known subpaths.
    set(BL_ELF       ${CMAKE_BINARY_DIR}/bootloader/saint_ota_bootloader.elf)
    set(BL_MAP       ${BL_ELF}.map)
    set(APP_BIN      ${CMAKE_BINARY_DIR}/${app_target}_app.bin)
    set(HDR_BIN      ${CMAKE_BINARY_DIR}/${app_target}_imghdr_combined.bin)
    set(COMBINED_ELF ${CMAKE_BINARY_DIR}/${app_target}_combined.elf)
    set(COMBINED_UF2 ${CMAKE_BINARY_DIR}/${app_target}_combined.uf2)

    # Modern Pico SDKs ship picotool instead of elf2uf2; the parent build
    # already triggers find_package(picotool) via pico_add_extra_outputs,
    # so the `picotool` imported target should exist by the time we get
    # here. Use $<TARGET_FILE:picotool> to resolve its executable path.
    if (NOT TARGET picotool)
        find_package(picotool QUIET)
    endif()
    if (NOT TARGET picotool)
        message(WARNING "picotool not found — combined .uf2 won't be built")
        return()
    endif()

    # Regenerate the image header using the actual app slot address read
    # from the bootloader's linker map (.app_bin section).
    add_custom_command(
        OUTPUT ${HDR_BIN}
        COMMAND ${Python3_EXECUTABLE} ${SAINT_OTA_BL_DIR}/gen_imghdr.py
                --map ${BL_MAP} --section .app_bin
                ${APP_BIN} ${HDR_BIN}
        DEPENDS ${app_target} saint_ota_bootloader
        COMMENT "Computing combined image header for ${app_target}"
        VERBATIM
    )

    # Patch the bootloader's ELF, replacing the placeholder .app_hdr and
    # .app_bin sections with the real header bytes and the real app
    # bytes. Then convert the patched ELF to UF2.
    add_custom_command(
        OUTPUT ${COMBINED_UF2}
        COMMAND ${CMAKE_OBJCOPY}
                --update-section .app_hdr=${HDR_BIN}
                --update-section .app_bin=${APP_BIN}
                ${BL_ELF} ${COMBINED_ELF}
        COMMAND $<TARGET_FILE:picotool> uf2 convert -t elf
                ${COMBINED_ELF} ${COMBINED_UF2}
        DEPENDS ${HDR_BIN} ${BL_ELF} ${APP_BIN}
        COMMENT "Building combined bootloader+app .uf2 (${app_target}_combined.uf2)"
        VERBATIM
    )

    add_custom_target(${app_target}_combined ALL DEPENDS ${COMBINED_UF2})
endfunction()

# Make the Python interpreter available for gen_imghdr.py.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
