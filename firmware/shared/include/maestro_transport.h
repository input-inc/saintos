/**
 * SAINT.OS Firmware - Maestro transport ops (shared)
 *
 * The Pololu Maestro speaks Pololu Compact Protocol over either its
 * USB CDC interface OR its UART TTL pins. Both are valid carriers —
 * the shared driver core (shared/src/maestro_driver.c) is
 * transport-agnostic and dispatches reads/writes through one of these
 * ops tables.
 *
 * In addition, the Maestro accepts Pololu vendor control requests on
 * endpoint 0 (e.g. 0x85 Set Target, 0x81 Get Parameter, 0x83 Get
 * Variables). The byte-stream Compact Protocol is enough for runtime
 * ops, but GET_PARAMETER (EEPROM readback) is ONLY available via
 * vendor requests. Transports that can issue them populate
 * `ctrl_xfer`; transports that can't (UART, USB CDC) leave it NULL.
 *
 * Each platform supplies whichever transport(s) it can physically
 * provide:
 *   - Teensy 4.1 has a USB host port → maestro_get_transport_usb_cdc()
 *     (via USBSerial_BigBuffer) AND maestro_get_transport_usb_vendor()
 *     (via USBHost_t36 queue_Control_Transfer).
 *   - RP2040 + Teensy 4.1 both have hardware UARTs → maestro_get_transport_uart()
 *
 * The active transport is selected at peripheral init time based on the
 * flash_maestro_config_t.transport_mode field (USB_CDC | UART |
 * USB_VENDOR). If a platform is asked for a transport it can't
 * provide, the registry returns NULL and the driver logs a config
 * error.
 *
 * Hot-plug semantics live in the transport: USB returns true from
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
    const char* name;             /* "usb_cdc" | "uart" | "usb_vendor" — used in logs */

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

    /* Best-effort write; returns true if every byte was accepted.
     * Carries Pololu Compact Protocol byte stream on uart / usb_cdc
     * transports. On the usb_vendor transport, `write` translates the
     * outgoing Compact Protocol command into the equivalent vendor
     * request on EP0 (set_target → REQUEST 0x85; etc.) so the driver
     * core stays transport-agnostic. */
    bool (*write)(const uint8_t* data, size_t len);

    /* Read up to `len` bytes within `timeout_ms`. Returns count read.
     * Same Compact-Protocol-stream emulation on the usb_vendor
     * transport: a `read` after a Get-Position request returns the
     * 2 bytes from a corresponding vendor IN transfer. */
    size_t (*read)(uint8_t* data, size_t len, uint32_t timeout_ms);

    /* True if the transport reports out-of-band connect/disconnect
     * events (USB host). Drives connect-announce log lines and the
     * /announce JSON's maestro_connected flag. */
    bool (*supports_hotplug)(void);

    /* OPTIONAL — only usb_vendor transports populate this. NULL on
     * uart and usb_cdc transports (those carriers can't issue EP0
     * control transfers at all). Used by the shared driver to call
     * Pololu's GET_PARAMETER (0x81) for EEPROM readback — see
     * docs/MAESTRO_BRINGUP.md.
     *
     * Semantics mirror libusb_control_transfer / pyusb.ctrl_transfer:
     *   bmRequestType: 0xC0 for IN, 0x40 for OUT
     *   bRequest:      Pololu vendor request code (0x81 GET_PARAMETER,
     *                  0x82 SET_PARAMETER, 0x83 GET_VARIABLES, etc.)
     *   wValue/wIndex: per-request parameters
     *   buf, wLength:  data stage buffer (IN: filled; OUT: source)
     *   timeout_ms:    bounded wait — vendor calls must never block
     *                  the loop indefinitely
     * Returns the number of bytes transferred on success, or < 0 on
     * error (timeout, STALL, transport down). */
    int (*ctrl_xfer)(uint8_t bmRequestType,
                     uint8_t bRequest,
                     uint16_t wValue,
                     uint16_t wIndex,
                     uint8_t* buf,
                     uint16_t wLength,
                     uint32_t timeout_ms);
} maestro_transport_ops_t;

/* Platform-provided transport lookups. A platform that cannot provide
 * a particular transport returns NULL — the shared driver then logs
 * a configuration error rather than crashing.
 *
 * All functions are declared unconditionally so the shared driver
 * compiles unchanged; the per-platform definition is the one that
 * decides what to actually return.
 *
 * `maestro_get_transport_usb_host` is the legacy alias for the CDC
 * ACM transport (called `usb_cdc` in the new flash enum). The legacy
 * getter remains because existing main.c on every platform
 * references it; new code should use the explicit usb_cdc /
 * usb_vendor names. */
const maestro_transport_ops_t* maestro_get_transport_usb_host(void);   /* legacy alias for usb_cdc */
const maestro_transport_ops_t* maestro_get_transport_usb_cdc(void);
const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void);
const maestro_transport_ops_t* maestro_get_transport_uart(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_MAESTRO_TRANSPORT_H */
