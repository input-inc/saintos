/**
 * SAINT.OS Firmware - Teensy 4.1 Maestro transport
 *
 * USB host implementation of the shared maestro_transport_ops contract.
 * The Pololu Maestro shows up as a USBSerial CDC device on the Teensy's
 * USB host port; we use USBHost_t36 to enumerate, then exchange compact-
 * protocol bytes over USBSerial_BigBuffer.
 *
 * Also provides maestro_get_transport_uart() returning NULL — the
 * Teensy's hardware Serial1..8 ports could carry the same protocol
 * (the Maestro has TTL pins), but that adapter isn't wired up yet.
 * Setting the saved transport_mode to UART will log an error and
 * leave the driver inert until the UART adapter is added.
 */

#include <Arduino.h>
#include "platform.h"

#ifndef SIMULATION
#include <USBHost_t36.h>
#endif

extern "C" {
#include "flash_types.h"
#include "maestro_transport.h"
}

/* ── USB host objects ────────────────────────────────────────────── */

#ifndef SIMULATION
static USBHost myusb;
static USBHub  hub1(myusb);
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

static const maestro_transport_ops_t usb_ops = {
    .name             = "usb_host",
    .open             = usb_open,
    .update           = usb_update,
    .is_connected     = usb_is_connected,
    .write            = usb_write,
    .read             = usb_read,
    .supports_hotplug = usb_supports_hotplug,
};

extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_host(void)
{
    return &usb_ops;
}

/* UART transport not yet implemented on Teensy — see header comment. */
extern "C" const maestro_transport_ops_t* maestro_get_transport_uart(void)
{
    return NULL;
}
