/**
 * SAINT.OS Firmware - Teensy 4.1 in-app OTA driver
 *
 * Drives the full update cycle:
 *   1. firmware_buffer_init() allocates a staging buffer in flash
 *      above the running app
 *   2. discover server IP via the NativeEthernet gateway (set by DHCP
 *      during app startup)
 *   3. http_get() against /api/firmware/teensy41/saint_node.bin streams
 *      the body through stage_body_cb, which writes 256-byte chunks
 *      into the flash buffer and accumulates a CRC32
 *   4. validate size + CRC against the values from the server's
 *      firmware_update control message
 *   5. flash_move() erases the app region, copies the buffer down,
 *      and reboots into the new image (does not return)
 */

#include "teensy_ota.h"

#include <Arduino.h>
#include <NativeEthernet.h>

extern "C" {
#include "FlashTxx.h"
#include "http_client.h"
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

/* Same CRC32 used on the server + by the bootloader on RP2040. */
static uint32_t crc32_update(uint32_t crc, const uint8_t* buf, size_t len)
{
    crc ^= 0xFFFFFFFFu;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

struct stage_ctx {
    uint32_t buffer_addr;        /* start of flash staging buffer        */
    uint32_t buffer_size;        /* size of staging buffer in bytes      */
    uint32_t write_offset;       /* bytes written so far                 */
    uint32_t crc32_running;      /* incremental CRC32 over the stream    */
    bool     error;
};

static int stage_body_cb(void* user, const uint8_t* data, size_t len)
{
    stage_ctx* st = static_cast<stage_ctx*>(user);
    if (st->error) return -1;

    /* Bounds check */
    if (st->write_offset + len > st->buffer_size) {
        Serial.printf("OTA: image larger than buffer (%lu + %u > %lu)\n",
                      (unsigned long)st->write_offset, (unsigned)len,
                      (unsigned long)st->buffer_size);
        st->error = true;
        return -1;
    }

    /* flash_write_block expects an absolute flash address, char* data,
     * and a count. The function handles its own sector erase + word
     * alignment internally. */
    int rc = flash_write_block(st->buffer_addr + st->write_offset,
                                (char*)const_cast<uint8_t*>(data),
                                (uint32_t)len);
    if (rc != 0) {
        Serial.printf("OTA: flash_write_block rc=%d at +%lu\n",
                      rc, (unsigned long)st->write_offset);
        st->error = true;
        return -1;
    }

    st->crc32_running = crc32_update(st->crc32_running, data, len);
    st->write_offset += (uint32_t)len;
    return 0;
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

    /* 3. Stream the download into flash. */
    stage_ctx st;
    memset(&st, 0, sizeof(st));
    st.buffer_addr  = buffer_addr;
    st.buffer_size  = buffer_size;

    http_ethernet_state_t tcp_state;
    http_transport_t      transport;
    http_transport_ethernet_init(&transport, &tcp_state);

    http_get_request_t req = {
        /*.host      =*/ host,
        /*.port      =*/ OTA_FIRMWARE_PORT,
        /*.path      =*/ OTA_FIRMWARE_PATH,
        /*.transport =*/ &transport,
        /*.body_cb   =*/ stage_body_cb,
        /*.user      =*/ &st,
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

    /* 4. Verify size + CRC. */
    if (st.write_offset != expected_size) {
        Serial.printf("OTA: size mismatch got=%lu expected=%lu\n",
                      (unsigned long)st.write_offset,
                      (unsigned long)expected_size);
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_SIZE;
    }
    if (st.crc32_running != expected_crc32) {
        Serial.printf("OTA: CRC mismatch got=0x%08lx expected=0x%08lx\n",
                      (unsigned long)st.crc32_running,
                      (unsigned long)expected_crc32);
        firmware_buffer_free(buffer_addr, buffer_size);
        return SAINT_OTA_ERR_CRC;
    }

    /* 5. Swap in. flash_move erases the app region, copies the staging
     *    buffer down, and reboots. Does not return on success. */
    Serial.println("OTA: download verified. Swapping in new firmware (chip will reboot)...");
    Serial.flush();
    delay(100);

    flash_move(FLASH_BASE_ADDR, buffer_addr, expected_size);
    /* not reached on success — flash_move reboots */

    Serial.println("OTA: flash_move returned (this is a hard failure)");
    return SAINT_OTA_ERR_FLASH;
}
