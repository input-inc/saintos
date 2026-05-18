/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SAINT.OS fork: added saint_ota_reboot_with_image() so the app can
 * pass image metadata (size / CRC32) to the bootloader via watchdog
 * scratch registers, then reboot into it to perform an HTTP OTA
 * download. The app slot address is owned by the bootloader's linker
 * script — the app doesn't transmit it.
 */

#ifndef __PICOWOTA_REBOOT_H__
#define __PICOWOTA_REBOOT_H__

#include <stdbool.h>
#include <stdint.h>

#define PICOWOTA_BOOTLOADER_ENTRY_MAGIC 0xb105f00d

#ifdef __cplusplus
extern "C" {
#endif

/** Plain reboot. If `to_bootloader` is true, the bootloader stays in
 *  bootloader mode on the next boot. SAINT.OS callers should use
 *  saint_ota_reboot_with_image() instead so the bootloader also has
 *  the image metadata it needs to fetch + verify. */
void picowota_reboot(bool to_bootloader);

/** Reboot into the bootloader for an HTTP OTA download. The bootloader
 *  reads these fields out of watchdog scratch registers after reset:
 *
 *      scratch[0]  expected image size in bytes
 *      scratch[1]  expected image CRC32
 *      scratch[5]  magic ( PICOWOTA_BOOTLOADER_ENTRY_MAGIC )
 *      scratch[6]  ~magic ( verification )
 *
 *  Server IP is discovered via the DHCP gateway in the bootloader.
 *  Firmware path is hardcoded to /api/firmware/rp2040/saint_node.bin. */
void saint_ota_reboot_with_image(uint32_t size, uint32_t crc32);

#ifdef __cplusplus
}
#endif

#endif /* __PICOWOTA_REBOOT_H__ */
