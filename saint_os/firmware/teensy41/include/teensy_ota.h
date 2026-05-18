/**
 * SAINT.OS Firmware - Teensy 4.1 in-app OTA driver
 *
 * Combines FlashTxx (in-application flash writes) with the shared
 * http_client (over NativeEthernet) to fetch a new firmware image
 * from the SAINT.OS server and swap it in.
 *
 * Teensy 4 doesn't support a custom bootloader (the HalfKay chip on
 * the board owns USB programming), so the swap happens from inside
 * the running app: we stage the new image in flash above the running
 * app, validate it, then call flash_move() which overwrites the app
 * region and reboots.
 *
 * Brick risk: the same single-slot window the RP2040 has — if power
 * dies during flash_move() (~200ms of erase+program), the chip needs
 * to be reflashed via Teensy Loader + USB.
 */

#ifndef SAINT_TEENSY_OTA_H
#define SAINT_TEENSY_OTA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAINT_OTA_OK             =  0,
    SAINT_OTA_ERR_BUFFER     = -1,   /* firmware_buffer_init failed     */
    SAINT_OTA_ERR_DHCP       = -2,   /* could not discover gateway      */
    SAINT_OTA_ERR_DOWNLOAD   = -3,   /* http_get returned non-OK        */
    SAINT_OTA_ERR_SIZE       = -4,   /* bytes_received != expected_size */
    SAINT_OTA_ERR_CRC        = -5,   /* CRC32 mismatch                  */
    SAINT_OTA_ERR_FLASH      = -6,   /* flash_write_block failed        */
} saint_ota_result_t;

/** Perform a complete OTA cycle. Returns SAINT_OTA_OK on success and
 *  reboots into the new image (does not return). On any failure
 *  returns the corresponding error code; the running app keeps going. */
saint_ota_result_t saint_ota_perform(uint32_t expected_size,
                                      uint32_t expected_crc32);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_TEENSY_OTA_H */
