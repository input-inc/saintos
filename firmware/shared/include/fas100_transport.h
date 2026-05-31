/**
 * SAINT.OS Firmware - FAS100 transport ops (shared)
 *
 * The FAS100 uses half-duplex inverted UART. The protocol layer is
 * the same on every host — what differs is how each platform expresses
 * "open the UART with inverted signaling" and how it pumps bytes:
 *
 *   - RP2040: uart_init + gpio_set_outover/inover(INVERT) + uart_set_baudrate
 *             + uart_is_readable / uart_getc / uart_write_blocking
 *   - Teensy: Serial.begin(baud, SERIAL_8N1_RXINV_TXINV) + write/read/available
 *
 * The auto-detect state machine in the shared driver switches baud
 * between S.Port (57600) and FBUS (460800) probes via set_baud, then
 * locks once a frame validates.
 *
 * Echo-drain pattern: FAS100 wiring loops TX→RX through a 1k pull, so
 * every byte the MCU sends echoes back through the RX path. The shared
 * driver writes a poll, calls flush() to wait for the bytes to leave
 * the FIFO, then reads back N bytes to drain the echo before parsing.
 */

#ifndef SAINT_FAS100_TRANSPORT_H
#define SAINT_FAS100_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fas100_transport_ops {
    const char* name;

    /* Open the UART with `baud` and inverted signaling iff `invert`
     * is true. Same pins/baud/invert as the last successful open is a
     * no-op. Different params tear down the prior binding first. */
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud, bool invert);

    /* Switch baud rate on the already-open UART. Drains the RX FIFO
     * so half-decoded bytes from the old rate don't corrupt the new
     * stream. */
    void (*set_baud)(uint32_t baud);

    bool (*is_open)(void);

    /* Synchronous write. Bytes may sit in the TX FIFO when this
     * returns; call flush() to be sure they've left the wire. */
    bool (*write)(const uint8_t* data, size_t len);

    /* Block until the TX FIFO drains physically. Needed before the
     * echo-drain loop so we don't read echoes of bytes still queued. */
    void (*flush)(void);

    /* Non-blocking read. Returns count read (possibly 0). */
    size_t (*read)(uint8_t* data, size_t max_len);

    uint8_t (*resolved_instance)(void);
} fas100_transport_ops_t;

const fas100_transport_ops_t* fas100_get_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_FAS100_TRANSPORT_H */
