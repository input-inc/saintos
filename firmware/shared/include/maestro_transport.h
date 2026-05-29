/**
 * SAINT.OS Firmware - Maestro transport ops (shared)
 *
 * The Pololu Maestro speaks the same compact-protocol byte stream over
 * either its USB CDC interface OR its UART TTL pins. Both are valid
 * carriers — the shared driver core (shared/src/maestro_driver.c) is
 * transport-agnostic and dispatches reads/writes through one of these
 * ops tables.
 *
 * Each platform supplies whichever transport(s) it can physically
 * provide:
 *   - Teensy 4.1 has a USB host port → maestro_get_transport_usb_host()
 *   - RP2040 + Teensy 4.1 both have hardware UARTs → maestro_get_transport_uart()
 *
 * The active transport is selected at peripheral init time based on the
 * flash_maestro_config_t.transport_mode field (USB_HOST | UART). If a
 * platform is asked for a transport it can't provide, the registry
 * returns NULL and the driver logs a config error.
 *
 * Hot-plug semantics live in the transport: USB host returns true from
 * is_connected only when the device has been enumerated; UART returns
 * true whenever the port is open (there's no out-of-band link signal).
 * Hot-plug-aware UI features (descriptor probe, connect/disconnect
 * announcement) are gated on supports_hotplug() so the dashboard
 * doesn't paint stale state on UART links.
 */

#ifndef SAINT_MAESTRO_TRANSPORT_H
#define SAINT_MAESTRO_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flash_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct maestro_transport_ops {
    const char* name;             /* "usb_host" | "uart" — used in logs */

    /* Open the transport using whatever fields apply from the saved
     * config (serial_port + uart_pins for UART, nothing for USB host).
     * Returns true on success. Called from maestro_drv_init. */
    bool (*open)(const flash_storage_data_t* storage);

    /* Drive any periodic transport work (USBHost::Task on the Teensy
     * USB host stack; no-op for UART). Called from peripheral_update_all. */
    void (*update)(void);

    /* Is the Maestro currently reachable? USB returns false until the
     * device enumerates; UART returns true whenever the port is open. */
    bool (*is_connected)(void);

    /* Best-effort write; returns true if every byte was accepted. */
    bool (*write)(const uint8_t* data, size_t len);

    /* Read up to `len` bytes within `timeout_ms`. Returns count read. */
    size_t (*read)(uint8_t* data, size_t len, uint32_t timeout_ms);

    /* True if the transport reports out-of-band connect/disconnect
     * events (USB host). Drives connect-announce log lines and the
     * /announce JSON's maestro_connected flag. */
    bool (*supports_hotplug)(void);
} maestro_transport_ops_t;

/* Platform-provided transport lookups. A platform that cannot provide
 * a particular transport returns NULL — the shared driver then logs
 * a configuration error rather than crashing.
 *
 * Both functions are declared unconditionally so the shared driver
 * compiles unchanged; the per-platform definition is the one that
 * decides what to actually return. */
const maestro_transport_ops_t* maestro_get_transport_usb_host(void);
const maestro_transport_ops_t* maestro_get_transport_uart(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_MAESTRO_TRANSPORT_H */
