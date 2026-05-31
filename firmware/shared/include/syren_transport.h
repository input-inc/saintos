/**
 * SAINT.OS Firmware - SyRen transport ops (shared)
 *
 * The SyRen 50 speaks the same 4-byte packetized serial protocol on
 * every host (address + command + value + checksum). What differs is
 * how each platform opens and writes to that UART:
 *
 *   - RP2040: pico-sdk uart_init + gpio_set_function + uart_write_blocking
 *   - Teensy 4.1: HardwareSerial::begin + HardwareSerial::write
 *
 * The shared driver core (shared/src/syren_driver.c) dispatches all
 * UART I/O through this ops table. Each platform supplies one
 * implementation via syren_get_transport().
 *
 * SyRen is write-only — there are no responses from the controller,
 * so this transport has no read op. The autobaud byte (0xAA) sent
 * right after open is just another write from the shared driver's
 * perspective.
 */

#ifndef SAINT_SYREN_TRANSPORT_H
#define SAINT_SYREN_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct syren_transport_ops {
    const char* name;             /* "uart" — used in log lines       */

    /* Open the UART with the given pin pair + baud. Each platform
     * resolves pin_tx/pin_rx to its internal UART instance via
     * uart_pin_pair_lookup. Returns true on success. Idempotent:
     * a second call with the same pins is a no-op. */
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint16_t baud);

    /* Has open() been called successfully? */
    bool (*is_open)(void);

    /* Send raw bytes. Returns true if all bytes were accepted. */
    bool (*write)(const uint8_t* data, size_t len);

    /* Resolved UART instance number, for diagnostics + flash save.
     * Returns 0xFF if the transport hasn't been opened yet. */
    uint8_t (*resolved_instance)(void);
} syren_transport_ops_t;

/* Platform-supplied. Each platform's syren_transport.{c,cpp}
 * defines this returning a static const ops table. */
const syren_transport_ops_t* syren_get_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_SYREN_TRANSPORT_H */
