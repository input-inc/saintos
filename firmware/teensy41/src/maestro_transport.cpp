/**
 * SAINT.OS Firmware - Teensy 4.1 Maestro transport
 *
 * Provides three transports satisfying the shared
 * maestro_transport_ops contract:
 *   - usb_cdc:    Pololu Compact Protocol byte stream over USB host
 *                 CDC ACM (USBHost_t36 + USBSerial_BigBuffer). Needs
 *                 the Maestro's Serial Mode set to "USB Dual Port"
 *                 (or "USB Chained") in MCC — otherwise bytes go out
 *                 the TTL pins instead of into the Maestro's command
 *                 interpreter. Cannot do EEPROM (GET_PARAMETER) ops.
 *   - usb_vendor: EP0 control transfers (Set Target via 0x85, GET/SET
 *                 PARAMETER, GET_VARIABLES). Works in any Serial Mode.
 *                 Enables EEPROM readback. **Not yet implemented**
 *                 — see task #17 / docs/MAESTRO_BRINGUP.md.
 *   - uart:       Pololu Compact Protocol over the Teensy's hardware
 *                 serial pins (Serial1..Serial8). Use when the Maestro
 *                 is wired to a TTL UART instead of plugged into the
 *                 USB host port. The Maestro's TTL RX always accepts
 *                 Compact Protocol regardless of its USB Serial Mode.
 */

#include <Arduino.h>
#include "platform.h"

#ifndef SIMULATION
#include <USBHost_t36.h>
#endif

extern "C" {
#include "flash_types.h"
#include "maestro_transport.h"
#include "maestro_driver.h"  /* MAESTRO_DEFAULT_UART_BAUD */
}

/* ── USB host objects ────────────────────────────────────────────── */

#ifndef SIMULATION
static USBHost myusb;
static USBHub  hub1(myusb);
/* USBSerial_BigBuffer(host, min_rxtx=1) — `1` is the MIN_RXTX hint
 * (lets it also handle devices that report small max packet sizes,
 * which the Maestro does on its CDC ACM endpoints). It is NOT a port
 * index; the driver attaches to the first CDC ACM interface it finds.
 * On the Mini Maestro in USB Dual Port mode that's the Command Port
 * (the TTL Port comes second), so commands like Set Target reach the
 * Maestro's command interpreter. */
static USBSerial_BigBuffer maestroSerial(myusb, 1);
#endif

static bool g_initialized = false;

static bool usb_open(const flash_storage_data_t* storage)
{
    (void)storage;
#ifndef SIMULATION
    if (!g_initialized) {
        myusb.begin();
        g_initialized = true;
    }
    return true;
#else
    return false;
#endif
}

static void usb_update(void)
{
#ifndef SIMULATION
    if (!g_initialized) return;
    myusb.Task();
#endif
}

static bool usb_is_connected(void)
{
#ifndef SIMULATION
    return (bool)maestroSerial;
#else
    return false;
#endif
}

static bool usb_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!usb_is_connected()) return false;
    return maestroSerial.write(data, len) == len;
#else
    (void)data; (void)len;
    return false;
#endif
}

static size_t usb_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
#ifndef SIMULATION
    if (!usb_is_connected()) return 0;
    size_t count = 0;
    uint32_t start = millis();
    while (count < len && (millis() - start) < timeout_ms) {
        if (maestroSerial.available()) {
            data[count++] = maestroSerial.read();
        }
    }
    return count;
#else
    (void)data; (void)len; (void)timeout_ms;
    return 0;
#endif
}

static bool usb_supports_hotplug(void)
{
    return true;
}

static const maestro_transport_ops_t usb_cdc_ops = {
    .name             = "usb_cdc",
    .open             = usb_open,
    .update           = usb_update,
    .is_connected     = usb_is_connected,
    .write            = usb_write,
    .read             = usb_read,
    .supports_hotplug = usb_supports_hotplug,
    .ctrl_xfer        = NULL,   /* CDC transport can't issue EP0 control transfers */
};

extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_cdc(void)
{
    return &usb_cdc_ops;
}

/* ── UART transport (HardwareSerial1..Serial8) ──────────────────── */

#ifndef SIMULATION
/* Map serial_port (1..8) → HardwareSerial& via runtime lookup.
 * Returning a pointer rather than a reference so the open() path can
 * cleanly signal "invalid serial_port" by leaving g_uart NULL. */
static HardwareSerial* hw_serial_for(uint8_t serial_port)
{
    switch (serial_port) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        case 8: return &Serial8;
        default: return nullptr;
    }
}

static HardwareSerial* g_uart = nullptr;
static bool g_uart_initialized = false;
#endif  // !SIMULATION

static bool uart_open(const flash_storage_data_t* storage)
{
#ifndef SIMULATION
    /* serial_port: 1..8 (Teensy hardware serial index). 0 is a
     * sentinel for "unset" — bail rather than guess. */
    uint8_t port = 0;
    if (storage) port = storage->maestro_config.serial_port;
    if (port == 0) {
        return false;
    }
    HardwareSerial* hs = hw_serial_for(port);
    if (!hs) {
        return false;   /* invalid port number */
    }
    /* MAESTRO_DEFAULT_UART_BAUD matches both the Maestro factory
     * default and what MCC sets when configuring a fresh device.
     * Compact-protocol auto-detect mode (the Maestro's default
     * factory setting) locks onto the first 0xAA byte we send and
     * works at any common baud, so 9600 here is fine even if the
     * operator changed the Maestro's fixed baud to something else
     * later. */
    if (!g_uart_initialized || g_uart != hs) {
        if (g_uart && g_uart_initialized && g_uart != hs) {
            g_uart->end();
        }
        hs->begin(MAESTRO_DEFAULT_UART_BAUD);
        g_uart = hs;
        g_uart_initialized = true;
    }
    return true;
#else
    (void)storage;
    return false;
#endif
}

static void uart_update(void) { /* HardwareSerial doesn't need pumping. */ }

static bool uart_is_connected(void)
{
#ifndef SIMULATION
    /* HardwareSerial has no link-down signal — once begin() has been
     * called and the pins are driving, we consider the link "up".
     * The Maestro's actual presence is observable only via the data
     * we get back from Get-Position queries; the shared driver core
     * already detects that path via maestro_get_position(). */
    return g_uart_initialized && g_uart != nullptr;
#else
    return false;
#endif
}

static bool uart_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!uart_is_connected()) return false;
    /* HardwareSerial::write returns total bytes written. On Teensy
     * the TX FIFO is large (~512 bytes) but for safety we check the
     * full count rather than fire-and-forget. */
    return g_uart->write(data, len) == len;
#else
    (void)data; (void)len;
    return false;
#endif
}

static size_t uart_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
#ifndef SIMULATION
    if (!uart_is_connected()) return 0;
    size_t count = 0;
    uint32_t start = millis();
    while (count < len && (millis() - start) < timeout_ms) {
        if (g_uart->available()) {
            data[count++] = g_uart->read();
        }
    }
    return count;
#else
    (void)data; (void)len; (void)timeout_ms;
    return 0;
#endif
}

static bool uart_supports_hotplug(void) { return false; }

static const maestro_transport_ops_t uart_ops = {
    .name             = "uart",
    .open             = uart_open,
    .update           = uart_update,
    .is_connected     = uart_is_connected,
    .write            = uart_write,
    .read             = uart_read,
    .supports_hotplug = uart_supports_hotplug,
    .ctrl_xfer        = NULL,   /* UART has no EP0 — never */
};

/* USB vendor (EP0 control transfer) transport — task #17. Stub returns
 * NULL until the USBHost_t36 queue_Control_Transfer implementation
 * lands; the shared driver then falls back to usb_cdc when the
 * operator's saved transport_mode is USB_VENDOR but vendor isn't
 * provided by this platform yet. */
extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void)
{
    return NULL;
}

/* Legacy alias — main.c on the Teensy currently references
 * maestro_get_transport_usb_host(); keep it forwarding to usb_cdc so
 * existing code paths compile unchanged. */
extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_host(void)
{
    return maestro_get_transport_usb_cdc();
}

extern "C" const maestro_transport_ops_t* maestro_get_transport_uart(void)
{
    return &uart_ops;
}
