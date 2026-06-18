/**
 * Host-runnable smoke test for the RP2040 maestro UART transport.
 *
 * The hardware-facing paths (uart_init, uart_write_blocking, uart_getc)
 * sit under #ifndef SIMULATION and aren't compiled here — same trick
 * the FAS100 and RoboClaw tests use. What we CAN verify host-side:
 *
 *   - maestro_get_transport_uart() returns a non-NULL ops table
 *   - maestro_get_transport_usb_host() returns NULL (RP2040 cannot
 *     enumerate USB devices; the shared driver depends on this null
 *     to skip the USB host transport on this platform)
 *   - The ops table binds all required function pointers (any NULL
 *     here would crash the shared driver dispatch)
 *   - supports_hotplug() returns false — the shared driver gates
 *     connect/disconnect announce log lines on this flag and must
 *     not paint stale state on UART links
 *
 * For exhaustive UART byte-exchange coverage we'd need to lift the
 * #ifndef SIMULATION guards (or extract the hardware calls behind a
 * tiny wrapper) and stub pico-sdk's uart.h primitives. That's a
 * larger refactor than belongs in this pass; the unit-level coverage
 * of the protocol itself lives in shared/tests/test_maestro_driver.c
 * via a fake transport ops table, so the shared-vs-platform split
 * already gives us the high-value tests.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

#define SIMULATION 1

/* ── Stub platform.h ──────────────────────────────────────────────── */
#define PLATFORM_H
static uint32_t test_now_ms = 0;
#define PLATFORM_MILLIS()      (test_now_ms)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF(...)   ((void)0)

/* ── Stub uart_pin_pairs.h ────────────────────────────────────────── */
#define UART_PIN_PAIRS_H
static bool uart_pin_pair_lookup(uint8_t tx, uint8_t rx, uint8_t* inst)
{
    (void)tx; (void)rx;
    *inst = 0;
    return true;
}

/* ── Pull in the unit under test. The pico-sdk hardware/uart.h and
 *   hardware/gpio.h includes are guarded by #ifndef SIMULATION, so
 *   they drop out and the rest of the file compiles host-side. ──── */
#include "../src/maestro_driver.c"

/* ── Test plumbing ────────────────────────────────────────────────── */

static int fail_count = 0;
#define EXPECT(cond, label) do {                                                \
    if (cond) printf("  ok   %s\n", label);                                     \
    else { printf("  FAIL %s\n", label); fail_count++; }                        \
} while (0)

int main(void)
{
    printf("test_maestro_driver (RP2040 UART transport smoke)\n");

    /* The transport-lookup contract — these function names are what
     * shared/src/maestro_driver.c::pick_transport calls. A bad link
     * would leave one or more unresolved; a returns-the-wrong-thing
     * implementation would silently route bytes nowhere. */
    const maestro_transport_ops_t* uart = maestro_get_transport_uart();
    EXPECT(uart != NULL, "maestro_get_transport_uart != NULL");

    /* RP2040 has no USB host controller, so both CDC and vendor (and
     * the legacy usb_host alias) must return NULL. */
    EXPECT(maestro_get_transport_usb_cdc()    == NULL, "usb_cdc == NULL (RP2040 cannot host USB)");
    EXPECT(maestro_get_transport_usb_vendor() == NULL, "usb_vendor == NULL (RP2040 cannot host USB)");
    EXPECT(maestro_get_transport_usb_host()   == NULL, "usb_host alias == NULL (RP2040)");

    if (!uart) {
        printf("\ntest_maestro_driver: %d failure(s) (uart transport missing)\n",
               fail_count);
        return 1;
    }

    /* Every hook the shared driver dispatches through must be bound.
     * NULL here would crash on the first OTA byte or first peripheral
     * update tick — fail loud at link time instead. */
    EXPECT(uart->name != NULL,            "ops.name bound");
    EXPECT(strcmp(uart->name, "uart") == 0, "ops.name is \"uart\"");
    EXPECT(uart->open             != NULL, "ops.open bound");
    EXPECT(uart->update           != NULL, "ops.update bound");
    EXPECT(uart->is_connected     != NULL, "ops.is_connected bound");
    EXPECT(uart->write            != NULL, "ops.write bound");
    EXPECT(uart->read             != NULL, "ops.read bound");
    EXPECT(uart->supports_hotplug != NULL, "ops.supports_hotplug bound");

    /* UART can't tell us about device presence the way USB enumerate
     * does — supports_hotplug returning false is the explicit signal
     * to the shared driver to suppress connect/disconnect log lines
     * (which would otherwise spam every announce tick). */
    EXPECT(uart->supports_hotplug() == false, "supports_hotplug == false");

    /* In SIMULATION, uart_open's hardware path drops out and the
     * function returns false at the bottom. Pre-open the transport
     * must report not-connected (g_open is still false). */
    EXPECT(uart->is_connected() == false,
           "is_connected == false before open");

    if (fail_count) {
        printf("\ntest_maestro_driver: %d failure(s)\n", fail_count);
        return 1;
    }
    printf("\ntest_maestro_driver: OK\n");
    return 0;
}
