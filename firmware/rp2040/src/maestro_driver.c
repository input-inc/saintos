/**
 * SAINT.OS Node Firmware - RP2040 Maestro transport
 *
 * The RP2040 has no USB host capability the way the Teensy does, so
 * we drive the Pololu Maestro over its alternate TTL serial interface
 * instead. The compact-protocol byte stream is identical regardless of
 * carrier — the shared maestro_driver core (shared/src/maestro_driver.c)
 * formats commands the same way, then hands the bytes to whichever
 * transport ops table the saved flash_maestro_config.transport_mode
 * picks.
 *
 * Wiring: pick a TX/RX pin pair via the dashboard (or the saved
 * flash_uart_pins_t.maestro_tx_pin / maestro_rx_pin). uart_pin_pair_lookup
 * resolves which pico-sdk uart_inst the pair belongs to (UART0 / UART1).
 * Defaults to GP8/GP9 (UART1 alternate position) — picked so the
 * default doesn't collide with RoboClaw (UART0 GP0/1) on a board
 * carrying both, even though the operator still has to swap to non-
 * conflicting pins if SyRen is also wired up on UART1.
 *
 * Also provides maestro_get_transport_usb_host() returning NULL: the
 * RP2040 cannot enumerate USB devices in any of our current builds.
 * If a flash config was migrated from a Teensy and asks for USB host
 * mode here, the shared driver logs an error and leaves the peripheral
 * inert (no crash, no UART traffic).
 */

#include "flash_types.h"
#include "maestro_transport.h"
#include "platform.h"
#include "uart_pin_pairs.h"

#include <string.h>

#ifndef SIMULATION
#include "hardware/uart.h"
#include "hardware/gpio.h"
#endif

/* ── Defaults & state ───────────────────────────────────────────── */

#define MAESTRO_DEFAULT_TX_PIN  8
#define MAESTRO_DEFAULT_RX_PIN  9
#define MAESTRO_DEFAULT_BAUD    9600

#ifndef SIMULATION
static uart_inst_t* g_uart = NULL;
#endif
static uint8_t  g_tx_pin   = MAESTRO_DEFAULT_TX_PIN;
static uint8_t  g_rx_pin   = MAESTRO_DEFAULT_RX_PIN;
static uint8_t  g_instance = 1;
static uint16_t g_baud     = MAESTRO_DEFAULT_BAUD;
static bool     g_open     = false;

/* ── UART transport ops ─────────────────────────────────────────── */

static bool uart_open(const flash_storage_data_t* storage)
{
#ifndef SIMULATION
    if (g_open) return true;

    /* Pull saved pin/instance values when present; fall back to
     * compile-time defaults when the operator hasn't set them yet.
     * uart_pins.maestro_tx_pin == 0 reads as "not configured" (mirrors
     * the convention already used for fas100/syren/roboclaw). */
    if (storage) {
        if (storage->uart_pins.maestro_tx_pin != 0) g_tx_pin = storage->uart_pins.maestro_tx_pin;
        if (storage->uart_pins.maestro_rx_pin != 0) g_rx_pin = storage->uart_pins.maestro_rx_pin;
    }

    if (!uart_pin_pair_lookup(g_tx_pin, g_rx_pin, &g_instance)) {
        g_tx_pin   = MAESTRO_DEFAULT_TX_PIN;
        g_rx_pin   = MAESTRO_DEFAULT_RX_PIN;
        g_instance = 1;
    }
    g_uart = (g_instance == 0) ? uart0 : uart1;

    uart_init(g_uart, g_baud);
    gpio_set_function(g_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(g_rx_pin, GPIO_FUNC_UART);
    PLATFORM_SLEEP_MS(50);

    g_open = true;
    PLATFORM_PRINTF("Maestro: UART%u opened TX=%u RX=%u at %u baud\n",
                    (unsigned)g_instance, (unsigned)g_tx_pin,
                    (unsigned)g_rx_pin, (unsigned)g_baud);
    return true;
#else
    (void)storage;
    return false;
#endif
}

static void uart_update(void)
{
    /* No periodic transport work on UART — uart_init keeps the FIFOs
     * draining and the shared driver handles state transitions. */
}

static bool uart_is_connected(void)
{
    /* No out-of-band link signal on UART — once we've opened the
     * port, treat the device as reachable. Send/receive failures
     * still bubble up through the shared driver's command paths. */
    return g_open;
}

static bool uart_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!g_open || !g_uart) return false;
    uart_write_blocking(g_uart, data, len);
    return true;
#else
    (void)data; (void)len;
    return false;
#endif
}

static size_t uart_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
#ifndef SIMULATION
    if (!g_open || !g_uart) return 0;

    size_t count = 0;
    uint32_t start = PLATFORM_MILLIS();
    while (count < len && (PLATFORM_MILLIS() - start) < timeout_ms) {
        if (uart_is_readable(g_uart)) {
            data[count++] = (uint8_t)uart_getc(g_uart);
        }
    }
    return count;
#else
    (void)data; (void)len; (void)timeout_ms;
    return 0;
#endif
}

static bool uart_supports_hotplug(void)
{
    /* UART has no link-state line — there's no equivalent of USB
     * enumerate/disconnect events. The shared driver suppresses hot-
     * plug-only behaviors (connect-announce logs, descriptor probe)
     * when this returns false. */
    return false;
}

static const maestro_transport_ops_t uart_ops = {
    .name             = "uart",
    .open             = uart_open,
    .update           = uart_update,
    .is_connected     = uart_is_connected,
    .write            = uart_write,
    .read             = uart_read,
    .supports_hotplug = uart_supports_hotplug,
};

const maestro_transport_ops_t* maestro_get_transport_uart(void)
{
    return &uart_ops;
}

/* USB host transports not available on RP2040 — see file header. The
 * RP2040 doesn't have a USB host controller; only the Pico W gen-2
 * does (PIO-based), and we don't support it from this driver. */
const maestro_transport_ops_t* maestro_get_transport_usb_cdc(void)
{
    return NULL;
}
const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void)
{
    return NULL;
}
/* Legacy alias for older driver code paths. */
const maestro_transport_ops_t* maestro_get_transport_usb_host(void)
{
    return maestro_get_transport_usb_cdc();
}
