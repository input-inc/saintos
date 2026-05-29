/**
 * SAINT.OS Firmware - CRC32 (shared)
 *
 * Reflected CRC-32 (IEEE 802.3 / zlib polynomial 0xEDB88320, init 0,
 * xorout 0xFFFFFFFF). The server-side firmware-info computation and
 * gen_imghdr.py produce the same value over the .bin, so the in-app
 * OTA paths on every node platform can verify with bit-exact equality.
 *
 * Incremental: call repeatedly with crc seeded to 0 on the first call
 * and the previous return value on subsequent calls.
 */

#ifndef SAINT_CRC32_H
#define SAINT_CRC32_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t saint_crc32_update(uint32_t crc, const uint8_t* buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_CRC32_H */
