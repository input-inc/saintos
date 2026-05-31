/**
 * SAINT.OS Firmware - TMC2208 transport adapter for RP2040
 *
 * UART config goes through the Pico SDK hardware UART. STEP/DIR pulse
 * generation uses a single shared repeating timer at 25 kHz that walks
 * all 4 axes Bresenham-style.
 *
 * Bresenham accumulator approach:
 *   each tick:  axis.accum += axis.rate_pps
 *               if axis.accum >= TICK_HZ: fire step, axis.accum -= TICK_HZ
 * This produces accurate step rates at low CPU cost with one timer
 * shared across all axes. Max step rate per axis = TICK_HZ / 2
 * (Nyquist).
 *
 * STEP pulse shape: ~1 µs HIGH (busy_wait inside the ISR), then LOW.
 * TMC2208 min STEP high time is 100 ns, comfortably satisfied.
 *
 * After each emitted pulse, the ISR calls tmc2208_step_done(axis).
 * When that returns false (target_position reached), the axis's rate
 * is set to 0 so future ticks skip it without overhead.
 */

#include "tmc2208_transport.h"
#include "tmc2208_driver.h"
#include "uart_pin_pairs.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <string.h>

/* ── UART state ────────────────────────────────────────────────── */

static uart_inst_t* s_uart = NULL;
static uint8_t      s_tx_pin = 0xFF;
static uint8_t      s_rx_pin = 0xFF;
static uint8_t      s_instance = 0xFF;
static uint32_t     s_baud = 0;

/* ── Step generation state ─────────────────────────────────────── */

#define TMC2208_TICK_HZ        25000u
#define TMC2208_TICK_PERIOD_US (1000000u / TMC2208_TICK_HZ)  /* 40 µs */

typedef struct {
    uint8_t  step_pin;
    uint8_t  dir_pin;
    uint32_t rate_pps;       /* 0 = idle */
    uint32_t accum;
    bool     attached;
} axis_slot_t;

static volatile axis_slot_t s_axis[TMC2208_MAX_AXES];
static repeating_timer_t    s_timer;
static volatile bool        s_timer_active = false;

/* Forward decl for the ISR. */
static bool step_timer_cb(repeating_timer_t* rt);

/* Lazily install the timer the first time any axis becomes active.
 * Avoids running the ISR at all when no Tics are configured. */
static void ensure_timer_running(void)
{
    if (s_timer_active) return;
    if (add_repeating_timer_us(-(int64_t)TMC2208_TICK_PERIOD_US,
                                step_timer_cb, NULL, &s_timer)) {
        s_timer_active = true;
    }
}

static bool step_timer_cb(repeating_timer_t* rt)
{
    (void)rt;
    bool any_active = false;
    for (uint8_t i = 0; i < TMC2208_MAX_AXES; i++) {
        volatile axis_slot_t* slot = &s_axis[i];
        if (!slot->attached || slot->rate_pps == 0) continue;
        any_active = true;

        slot->accum += slot->rate_pps;
        if (slot->accum >= TMC2208_TICK_HZ) {
            slot->accum -= TMC2208_TICK_HZ;
            /* Fire a STEP pulse. Min HIGH time on TMC2208 is 100 ns;
             * a tight busy-wait of 1 µs is safe. */
            gpio_put(slot->step_pin, 1);
            busy_wait_us_32(1);
            gpio_put(slot->step_pin, 0);

            if (!tmc2208_step_done(i)) {
                /* Axis hit target. Stop scheduling. */
                slot->rate_pps = 0;
                slot->accum = 0;
            }
        }
    }
    (void)any_active;  /* keep repeating; ensure_timer_running stays on */
    return true;  /* keep repeating */
}

/* ── UART ops ──────────────────────────────────────────────────── */

static bool tmc_rp2040_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud)
{
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;

    if (s_uart && s_tx_pin == tx_pin && s_rx_pin == rx_pin
        && s_instance == inst && s_baud == baud) {
        return true;
    }

    /* Tear down previous binding. */
    if (s_tx_pin != 0xFF) {
        gpio_set_function(s_tx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_tx_pin, GPIO_IN);
    }
    if (s_rx_pin != 0xFF) {
        gpio_set_function(s_rx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_rx_pin, GPIO_IN);
    }
    if (s_uart && s_instance != inst) {
        uart_deinit(s_uart);
    }

    s_uart = (inst == 0) ? uart0 : uart1;
    uart_init(s_uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin);

    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_instance = inst;
    s_baud = baud;
    return true;
}

static bool tmc_rp2040_is_open(void)
{
    return s_uart != NULL;
}

static uint8_t tmc_rp2040_resolved_instance(void)
{
    return s_instance;
}

static size_t tmc_rp2040_xfer(const uint8_t* tx, size_t tx_len,
                                uint8_t* rx_buf, size_t rx_len)
{
    if (!s_uart) return 0;
    /* Flush stale RX. */
    while (uart_is_readable(s_uart)) (void)uart_getc(s_uart);

    uart_write_blocking(s_uart, tx, tx_len);
    uart_tx_wait_blocking(s_uart);

    /* Drain echo with per-byte timeout. */
    uint32_t per_byte_us = (12u * 1000000u) / s_baud + 200u;
    for (size_t i = 0; i < tx_len; i++) {
        absolute_time_t deadline = make_timeout_time_us(per_byte_us);
        while (!uart_is_readable(s_uart) && !time_reached(deadline)) {
            tight_loop_contents();
        }
        if (!uart_is_readable(s_uart)) return 0;
        (void)uart_getc(s_uart);
    }

    if (rx_len == 0 || rx_buf == NULL) return 0;

    size_t got = 0;
    absolute_time_t deadline = make_timeout_time_ms(TMC2208_READ_TIMEOUT_MS);
    while (got < rx_len) {
        if (uart_is_readable(s_uart)) {
            rx_buf[got++] = (uint8_t)uart_getc(s_uart);
            deadline = make_timeout_time_ms(TMC2208_READ_TIMEOUT_MS);
            continue;
        }
        if (time_reached(deadline)) return got;
    }
    return got;
}

/* ── Step generation ops ───────────────────────────────────────── */

static bool tmc_rp2040_axis_attach(uint8_t axis, uint8_t step_pin, uint8_t dir_pin)
{
    if (axis >= TMC2208_MAX_AXES) return false;
    volatile axis_slot_t* slot = &s_axis[axis];

    /* Tear down old pins if any change. */
    if (slot->attached &&
        (slot->step_pin != step_pin || slot->dir_pin != dir_pin)) {
        slot->rate_pps = 0;
        gpio_set_function(slot->step_pin, GPIO_FUNC_SIO);
        gpio_set_dir(slot->step_pin, GPIO_IN);
        gpio_set_function(slot->dir_pin, GPIO_FUNC_SIO);
        gpio_set_dir(slot->dir_pin, GPIO_IN);
    }

    gpio_init(step_pin);
    gpio_set_dir(step_pin, GPIO_OUT);
    gpio_put(step_pin, 0);

    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, 0);

    slot->step_pin = step_pin;
    slot->dir_pin  = dir_pin;
    slot->accum = 0;
    slot->rate_pps = 0;
    slot->attached = true;

    ensure_timer_running();
    return true;
}

static void tmc_rp2040_axis_detach(uint8_t axis)
{
    if (axis >= TMC2208_MAX_AXES) return;
    volatile axis_slot_t* slot = &s_axis[axis];
    slot->rate_pps = 0;
    if (slot->attached) {
        gpio_set_function(slot->step_pin, GPIO_FUNC_SIO);
        gpio_set_dir(slot->step_pin, GPIO_IN);
        gpio_set_function(slot->dir_pin, GPIO_FUNC_SIO);
        gpio_set_dir(slot->dir_pin, GPIO_IN);
    }
    slot->attached = false;
}

static void tmc_rp2040_axis_set_rate(uint8_t axis, uint32_t pps, bool forward)
{
    if (axis >= TMC2208_MAX_AXES) return;
    volatile axis_slot_t* slot = &s_axis[axis];
    if (!slot->attached) return;

    gpio_put(slot->dir_pin, forward ? 1 : 0);

    /* Clamp at half the tick rate (Nyquist) so each commanded pulse
     * actually corresponds to one tick crossing. */
    uint32_t max_pps = TMC2208_TICK_HZ / 2u;
    if (pps > max_pps) pps = max_pps;

    slot->rate_pps = pps;
    if (pps == 0) {
        slot->accum = 0;
        gpio_put(slot->step_pin, 0);
    }
}

/* ── Ops table ─────────────────────────────────────────────────── */

static const tmc2208_transport_ops_t s_ops = {
    .name              = "rp2040-uart-bresenham",
    .open              = tmc_rp2040_open,
    .is_open           = tmc_rp2040_is_open,
    .resolved_instance = tmc_rp2040_resolved_instance,
    .xfer              = tmc_rp2040_xfer,
    .axis_attach       = tmc_rp2040_axis_attach,
    .axis_detach       = tmc_rp2040_axis_detach,
    .axis_set_rate     = tmc_rp2040_axis_set_rate,
};

const tmc2208_transport_ops_t* tmc2208_get_transport(void)
{
    return &s_ops;
}
