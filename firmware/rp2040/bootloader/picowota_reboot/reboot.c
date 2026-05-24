/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SAINT.OS fork: added saint_ota_reboot_with_image() which stashes
 * image metadata in watchdog scratch[0..1] before triggering the
 * bootloader entry. The app load address is owned by the bootloader,
 * not passed in via scratch.
 */

#include "pico/stdlib.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"

#include "picowota/reboot.h"

void picowota_reboot(bool to_bootloader)
{
	hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
	if (to_bootloader) {
		watchdog_hw->scratch[5] = PICOWOTA_BOOTLOADER_ENTRY_MAGIC;
		watchdog_hw->scratch[6] = ~PICOWOTA_BOOTLOADER_ENTRY_MAGIC;
	} else {
		watchdog_hw->scratch[5] = 0;
		watchdog_hw->scratch[6] = 0;
	}
	watchdog_reboot(0, 0, 0);
	while (1) {
		tight_loop_contents();
		asm("");
	}
}

void saint_ota_reboot_with_image(uint32_t size, uint32_t crc32)
{
	hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
	/* Image metadata for the bootloader to validate the download. */
	watchdog_hw->scratch[0] = size;
	watchdog_hw->scratch[1] = crc32;
	/* Fresh OTA cycle — reset the bootloader's retry counter and
	 * any stale "gave up" sentinel from a previous failed attempt.
	 * Without this, retries from a prior failed run would carry
	 * over and the new attempt could short-circuit to fallback. */
	watchdog_hw->scratch[3] = 0;
	watchdog_hw->scratch[4] = 0;
	/* Magic to keep the bootloader in OTA mode after reset. */
	watchdog_hw->scratch[5] = PICOWOTA_BOOTLOADER_ENTRY_MAGIC;
	watchdog_hw->scratch[6] = ~PICOWOTA_BOOTLOADER_ENTRY_MAGIC;
	watchdog_reboot(0, 0, 0);
	while (1) {
		tight_loop_contents();
		asm("");
	}
}
