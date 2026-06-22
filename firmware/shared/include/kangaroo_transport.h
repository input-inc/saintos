/**
 * SAINT.OS Firmware - Kangaroo transport ops (shared)
 *
 * The Kangaroo speaks the same packet / simplified serial protocol on
 * every host; what differs is how each platform opens, writes, and
 * READS its UART. Unlike the SyRen (write-only), the Kangaroo replies
 * to Get/Status requests, so this transport — like the Tic's — needs a
 * read op.
 *
 *   - RP2040: pico-sdk uart_init + uart_write_blocking + uart_getc
 *   - Teensy 4.1: HardwareSerial::begin / ::write / ::read
 *
 * The shared driver core (shared/src/kangaroo_driver.c) dispatches all
 * UART I/O through this ops table. Each platform supplies one
 * implementation via kangaroo_get_transport().
 */

#ifndef SAINT_KANGAROO_TRANSPORT_H
#define SAINT_KANGAROO_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct kangaroo_transport_ops {
    const char* name;             /* "uart" — used in log lines */

    /* Open the UART with the given pin pair + baud. Each platform
     * resolves tx/rx to its internal UART instance via
     * uart_pin_pair_lookup. Returns true on success. Idempotent:
     * a second call with the same pins/baud is a no-op. Baud is a
     * uint32 because the Kangaroo can run up to 115200. */
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);

    /* Has open() been called successfully? */
    bool (*is_open)(void);

    /* Send raw bytes. Returns true if all bytes were accepted. */
    bool (*write)(const uint8_t* data, size_t len);

    /* Non-blocking read of up to max_len available bytes. Returns the
     * number actually read (0 if none waiting). */
    size_t (*read)(uint8_t* data, size_t max_len);

    /* Resolved UART instance number, for diagnostics + flash save.
     * Returns 0xFF if the transport hasn't been opened yet. */
    uint8_t (*resolved_instance)(void);
} kangaroo_transport_ops_t;

/* Platform-supplied. Each platform's kangaroo_transport.{c,cpp}
 * defines this returning a static const ops table. */
const kangaroo_transport_ops_t* kangaroo_get_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_KANGAROO_TRANSPORT_H */
