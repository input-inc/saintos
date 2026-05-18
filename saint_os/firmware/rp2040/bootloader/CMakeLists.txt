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
# saint_ota_bootloader — the OTA bootloader itself
# -----------------------------------------------------------------------------
add_executable(saint_ota_bootloader
    ${SAINT_OTA_BL_DIR}/main.c
    ${SAINT_OTA_BL_DIR}/bootloader_w5500_init.c
    ${SAINT_OTA_BL_DIR}/http_transport_w5500.c
    # Shared HTTP client
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/src/http_client.c
)
target_include_directories(saint_ota_bootloader PRIVATE
    ${BL_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
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

# Build a combined .uf2: bootloader + app at their respective slot offsets.
# Use for first-time BOOTSEL flash; subsequent updates use OTA.
function(saint_ota_build_combined app_target)
    set(combined_uf2 ${CMAKE_CURRENT_BINARY_DIR}/${app_target}_combined.uf2)
    add_custom_target(${app_target}_combined ALL
        COMMAND ${Python3_EXECUTABLE} ${SAINT_OTA_BL_DIR}/../tools/combine_uf2.py
                ${combined_uf2}
                $<TARGET_FILE_DIR:saint_ota_bootloader>/saint_ota_bootloader.uf2
                $<TARGET_FILE_DIR:${app_target}>/${app_target}.uf2
        DEPENDS saint_ota_bootloader ${app_target}
        COMMENT "Combining bootloader + ${app_target} into single .uf2"
        VERBATIM
    )
endfunction()

# Make the Python interpreter available for gen_imghdr.py.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
