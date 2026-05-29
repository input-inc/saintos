/**
 * SAINT.OS Firmware - Teensy 4.1 in-app OTA driver
 *
 * Drives the full update cycle:
 *   1. firmware_buffer_init() allocates a staging buffer in flash
 *      above the running app
 *   2. discover server IP via the NativeEthernet gateway (set by DHCP
 *      during app startup)
 *   3. http_get() against /api/firmware/teensy41/saint_node.bin streams
 *      the body through stage_body_cb, which accumulates into a small
 *      word-aligned page buffer and forwards 4-byte-aligned blocks to
 *      FlashTxx — flash_write_block rejects any call with count % 4 != 0
 *   4. validate size + CRC against the values from the server's
 *      firmware_update control message, plus check_flash_id() to make
 *      sure the staged image was actually built for this MCU
 *   5. flash_move() erases the app region, copies the buffer down,
 *      and reboots into the new image (does not return)
 *
 * Wire format and CRC32 polynomial match the RP2040 OTA path; see
 * firmware/rp2040/bootloader/main.c.
 */

#include "teensy_ota.h"

#include <Arduino.h>
#include <NativeEthernet.h>

extern "C" {
#include "FlashTxx.h"
#include "http_client.h"
#include "crc32.h"
#include "ota_streamer.h"
}
#include "http_transport_ethernet.h"

#include <stdint.h>
#include <string.h>

#ifndef OTA_FIRMWARE_PATH
#define OTA_FIRMWARE_PATH "/api/firmware/teensy41/saint_node.bin"
#endif

#ifndef OTA_FIRMWARE_PORT
#define OTA_FIRMWARE_PORT 80
#endif

/* Page size for buffering writes into FlashTxx. Must be a multiple of 4
 * (the FlashTxx word size on iMXRT1062). 256 mirrors the RP2040 OTA
 * bootloader's FLASH_PAGE_SIZE and amortizes flash_write_block overhead. */
#define OTA_PAGE_SIZE  256u

/* Force FLASH_ID into the staged binary so check_flash_id() has something
 * to find. Without ((used)) the linker drops the unreferenced symbol from
 * the compiled image and the post-download target check would always fail. */
__attribute__((used))
static const char saint_ota_flash_id[] = FLASH_ID;

/* Adapter: the shared ota_streamer flushes 4-byte-aligned chunks into
 * this callback, which forwards them to FlashTxx. The streamer caller
 * sets ctx to the absolute flash address of the staging buffer. */
static int teensy_flash_write(void* ctx, uint32_t write_offset,
                               const uint8_t* data, uint32_t len)
{
    uint32_t base_addr = (uint32_t)(uintptr_t)ctx;
    int rc = flash_write_block(base_addr + write_offset,
                                (char*)data, len);
    if (rc != 0) {
        Serial.printf("OTA: flash_write_block rc=%d at +%lu (len=%lu)\n",
                      rc, (unsigned long)write_offset,
                      (unsigned long)len);
    }
    return rc;
}

static int stage_body_cb(void* user, const uint8_t* data, size_t len)
{
    ota_streamer_t* s = static_cast<ota_streamer_t*>(user);
    ota_stream_status_t st = ota_streamer_feed(s, data, len);
    if (st == OTA_STREAM_OK) return 0;
    Serial.printf("OTA: stream feed error %d at offset %lu\n",
                  (int)st, (unsigned long)s->bytes_streamed);
    return -1;
}

extern "C" saint_ota_result_t saint_ota_perform(uint32_t expected_size,
                                                 uint32_t expected_crc32)
{
    Serial.printf("OTA: starting (size=%lu crc=0x%08lx)\n",
                  (unsigned long)expected_size,
                  (unsigned long)expected_crc32);

    /* 1. Allocate staging buffer in flash. */
    uint32_t buffer_addr = 0, buffer_size = 0;
    int br = firmware_buffer_init(&buffer_addr, &buffer_size);
    if (br != 0 || buffer_size < expected_size) {
        Serial.printf("OTA: firmware_buffer_init rc=%d, size=%lu < expected=%lu\n",
                      br, (unsigned long)buffer_size,
                      (unsigned long)expected_size);
        if (buffer_addr) firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_BUFFER;
    }
    Serial.printf("OTA: buffer 0x%08lX size=%lu bytes\n",
                  (unsigned long)buffer_addr, (unsigned long)buffer_size);

    /* 2. Server IP = current gateway (the Pi running dnsmasq). */
    IPAddress gw = Ethernet.gatewayIP();
    if (gw == IPAddress(0, 0, 0, 0)) {
        firmware_buffer_free(buffer_addr, buffer_size);
        Serial.println("OTA: gateway IP is 0.0.0.0 — DHCP not bound?");
        return SAINT_OTA_ERR_DHCP;
    }
    char host[16];
    snprintf(host, sizeof(host), "%u.%u.%u.%u", gw[0], gw[1], gw[2], gw[3]);
    Serial.printf("OTA: GET http://%s%s\n", host, OTA_FIRMWARE_PATH);

    /* 3. Stream the download into flash via the shared streamer. The
     *    streamer owns the page buffer + CRC + alignment-padding; the
     *    body callback just hands every chunk to ota_streamer_feed. */
    static uint8_t s_page_buf[OTA_PAGE_SIZE];
    ota_streamer_t streamer;
    ota_streamer_init(&streamer, s_page_buf, OTA_PAGE_SIZE,
                       /*pad_granularity=*/ 4u,
                       buffer_size,
                       teensy_flash_write,
                       (void*)(uintptr_t)buffer_addr);

    http_ethernet_state_t tcp_state;
    http_transport_t      transport;
    http_transport_ethernet_init(&transport, &tcp_state);

    http_get_request_t req = {
        /*.host      =*/ host,
        /*.port      =*/ OTA_FIRMWARE_PORT,
        /*.path      =*/ OTA_FIRMWARE_PATH,
        /*.transport =*/ &transport,
        /*.body_cb   =*/ stage_body_cb,
        /*.user      =*/ &streamer,
    };
    http_get_result_t res;
    http_status_t s = http_get(&req, &res);

    if (s != HTTP_OK) {
        Serial.printf("OTA: http_get failed s=%d code=%d recv=%lu/%lu\n",
                      (int)s, res.http_code,
                      (unsigned long)res.bytes_received,
                      (unsigned long)res.content_length);
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_DOWNLOAD;
    }

    /* 4. Flush the residual page tail (padded to a 4-byte boundary
     *    with 0xFF, outside the bytes_streamed CRC window). */
    if (ota_streamer_finalize(&streamer) != OTA_STREAM_OK) {
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_FLASH;
    }

    /* 5. Verify size + CRC. */
    if (streamer.bytes_streamed != expected_size) {
        Serial.printf("OTA: size mismatch got=%lu expected=%lu\n",
                      (unsigned long)streamer.bytes_streamed,
                      (unsigned long)expected_size);
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_SIZE;
    }
    if (streamer.crc32 != expected_crc32) {
        Serial.printf("OTA: CRC mismatch got=0x%08lx expected=0x%08lx\n",
                      (unsigned long)streamer.crc32,
                      (unsigned long)expected_crc32);
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_CRC;
    }

    /* 6. Sanity-check the staged image was built for THIS MCU. FlashTxx
     *    embeds FLASH_ID ("fw_teensy41") in each compiled binary via the
     *    saint_ota_flash_id symbol above; check_flash_id() greps for it
     *    in the staged buffer. Cheap insurance against staging the wrong
     *    platform's .bin in resources/firmware/teensy41/. */
    if (!check_flash_id(buffer_addr, expected_size)) {
        Serial.printf("OTA: target ID '%s' not found in staged image — "
                      "wrong platform's .bin?\n", FLASH_ID);
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_TARGET;
    }

    /* 7. Swap in. flash_move erases the app region, copies the staging
     *    buffer down, and reboots. Does not return on success. */
    Serial.println("OTA: download verified. Swapping in new firmware (chip will reboot)...");
    Serial.flush();
    delay(100);

    flash_move(FLASH_BASE_ADDR, buffer_addr, expected_size);
    /* not reached on success — flash_move reboots */

    Serial.println("OTA: flash_move returned (this is a hard failure)");
    return SAINT_OTA_ERR_FLASH;
}
