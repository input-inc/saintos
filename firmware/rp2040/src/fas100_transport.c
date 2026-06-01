/**
 * SAINT.OS Firmware - FAS100 transport adapter for RP2040
 *
 * Half-duplex inverted UART over a single bidirectional wire (TX → 1 kΩ
 * → RX, with the sensor's signal pin tied to RX). Both the S.Port
 * (57600 baud) and FBUS (460800 baud) protocols use the same physical
 * wiring; the shared driver core switches baud via set_baud() during
 * auto-detection.
 *
 * Inversion is critical and tricky here:
 *   - gpio_set_outover/inover(GPIO_OVERRIDE_INVERT) flips the bit at
 *     the pad level, so the UART block keeps thinking it's talking
 *     plain 8N1 while the wire sees inverted idle.
 *   - The internal pull-up is deliberately disabled. With inversion
 *     enabled the UART's high-idle becomes a physical LOW; an internal
 *     pull-up to 3.3 V would fight that idle AND, empirically, is
 *     enough bias to make the FAS100 ADV decide at power-on that an
 *     FBUS master is talking to it (sensor LED flips from slow→fast
 *     blink). Leave it floating from the SDK's perspective and rely on
 *     the active drive through the 1 kΩ TX→RX path.
 *
 * Rebinding to a different pin pair tears down the old binding
 * (override → NORMAL, function → SIO, dir → IN) so the previous pins
 * stop driving the bus. uart_deinit only fires if the UART instance is
 * actually changing — toggling the same instance briefly would drop a
 * byte in flight.
 */

#include "fas100_transport.h"
#include "uart_pin_pairs.h"
#include "saint_log.h"

#include <string.h>
#include <stddef.h>

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/* Force exclusive ownership of `target_inst` (UART0 or UART1) by
 * detaching every OTHER pad on the platform that is currently muxed
 * to that same UART block. RP2040's IO_BANK0 lets multiple pads claim
 * GPIO_FUNC_UART for the same instance simultaneously — they all
 * wire-OR into the block's RX input, and a stray pad with no driver
 * pulls RX to a garbage state that swallows every real byte. This
 * regression bit FAS100 after the driver consolidation: Maestro (or
 * any other UART peripheral whose drv_load opens UART0 first) leaves
 * its TX/RX pads in GPIO_FUNC_UART even after the FAS100 transport
 * binds its own 28/29 pair. Symptom: polls_sent climbs, echo_bytes
 * stays 0 — the firmware writes go out, but FAS100's loopback echo
 * can't fight the floating ghost-pad on RX.
 *
 * We can't ask the SDK "which pads are currently routed to UART<N>?"
 * directly. Instead we walk the platform's known UART pin pair table
 * (uart_pin_pairs_table) and, for every (tx,rx,instance) entry that
 * matches our target instance but isn't the pair we're about to
 * keep, check the pad's current function. If it's still GPIO_FUNC_UART
 * we kick it back to GPIO_FUNC_SIO + GPIO_IN so it stops driving the
 * shared block. Pads we leave alone stay where the owning driver put
 * them. */
static void detach_other_pads_on_same_uart(uint8_t target_inst,
                                           uint8_t keep_tx,
                                           uint8_t keep_rx)
{
    size_t count = 0;
    const uart_pin_pair_t* table = uart_pin_pairs_table(&count);
    if (!table) return;

    for (size_t i = 0; i < count; i++) {
        const uart_pin_pair_t* p = &table[i];
        if (p->uart_instance != target_inst) continue;
        if (p->tx_pin == keep_tx && p->rx_pin == keep_rx) continue;

        bool detached_any = false;
        if (p->tx_pin != keep_tx && p->tx_pin != keep_rx
            && gpio_get_function(p->tx_pin) == GPIO_FUNC_UART) {
            gpio_set_function(p->tx_pin, GPIO_FUNC_SIO);
            gpio_set_dir(p->tx_pin, GPIO_IN);
            detached_any = true;
        }
        if (p->rx_pin != keep_tx && p->rx_pin != keep_rx
            && gpio_get_function(p->rx_pin) == GPIO_FUNC_UART) {
            gpio_set_function(p->rx_pin, GPIO_FUNC_SIO);
            gpio_set_dir(p->rx_pin, GPIO_IN);
            detached_any = true;
        }
        if (detached_any) {
            saint_log_publish("warn",
                "FAS100: detached stale UART%u pads (tx=%u rx=%u) "
                "before binding %u/%u — another driver had left them claimed",
                (unsigned)target_inst,
                (unsigned)p->tx_pin, (unsigned)p->rx_pin,
                (unsigned)keep_tx, (unsigned)keep_rx);
        }
    }
}

static uart_inst_t* s_uart = NULL;
static uint8_t      s_tx_pin = 0xFF;
static uint8_t      s_rx_pin = 0xFF;
static uint8_t      s_instance = 0xFF;
static uint32_t     s_baud = 0;
static bool         s_invert_active = false;

static bool fas100_rp2040_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud, bool invert)
{
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;

    /* Already bound to the same pair, baud, and inversion — no-op. */
    if (s_uart && s_tx_pin == tx_pin && s_rx_pin == rx_pin
        && s_instance == inst && s_baud == baud
        && s_invert_active == invert) {
        return true;
    }

    /* Detach previous pins so they stop driving the bus when we move to
     * a different pair (or different UART instance). */
    if (s_tx_pin != 0xFF) {
        gpio_set_outover(s_tx_pin, GPIO_OVERRIDE_NORMAL);
        gpio_set_function(s_tx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_tx_pin, GPIO_IN);
    }
    if (s_rx_pin != 0xFF) {
        gpio_set_inover(s_rx_pin, GPIO_OVERRIDE_NORMAL);
        gpio_set_function(s_rx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_rx_pin, GPIO_IN);
    }
    if (s_uart && s_instance != inst) {
        uart_deinit(s_uart);
    }

    s_uart = (inst == 0) ? uart0 : uart1;

    /* Take exclusive ownership of this UART block — see
     * detach_other_pads_on_same_uart for the regression this guards
     * against. Must happen BEFORE uart_init so the block restarts on a
     * clean RX input. */
    detach_other_pads_on_same_uart(inst, tx_pin, rx_pin);

    uart_init(s_uart, baud);

    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    if (invert) {
        gpio_set_outover(tx_pin, GPIO_OVERRIDE_INVERT);
        gpio_set_inover(rx_pin, GPIO_OVERRIDE_INVERT);
    } else {
        gpio_set_outover(tx_pin, GPIO_OVERRIDE_NORMAL);
        gpio_set_inover(rx_pin, GPIO_OVERRIDE_NORMAL);
    }
    gpio_disable_pulls(tx_pin);
    gpio_disable_pulls(rx_pin);

    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_instance = inst;
    s_baud = baud;
    s_invert_active = invert;
    return true;
}

static void fas100_rp2040_set_baud(uint32_t baud)
{
    if (!s_uart) return;
    uart_set_baudrate(s_uart, baud);
    s_baud = baud;
    while (uart_is_readable(s_uart)) (void)uart_getc(s_uart);
}

static bool fas100_rp2040_is_open(void)
{
    return s_uart != NULL;
}

static bool fas100_rp2040_write(const uint8_t* data, size_t len)
{
    if (!s_uart) return false;
    uart_write_blocking(s_uart, data, len);
    return true;
}

static void fas100_rp2040_flush(void)
{
    if (!s_uart) return;
    uart_tx_wait_blocking(s_uart);
}

static size_t fas100_rp2040_read(uint8_t* data, size_t max_len)
{
    if (!s_uart || max_len == 0) return 0;
    size_t n = 0;
    while (n < max_len && uart_is_readable(s_uart)) {
        data[n++] = (uint8_t)uart_getc(s_uart);
    }
    return n;
}

static uint8_t fas100_rp2040_resolved_instance(void)
{
    return s_instance;
}

static const fas100_transport_ops_t s_ops = {
    .name              = "rp2040-uart",
    .open              = fas100_rp2040_open,
    .set_baud          = fas100_rp2040_set_baud,
    .is_open           = fas100_rp2040_is_open,
    .write             = fas100_rp2040_write,
    .flush             = fas100_rp2040_flush,
    .read              = fas100_rp2040_read,
    .resolved_instance = fas100_rp2040_resolved_instance,
};

const fas100_transport_ops_t* fas100_get_transport(void)
{
    return &s_ops;
}
