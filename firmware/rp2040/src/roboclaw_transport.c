/**
 * SAINT.OS Firmware - RoboClaw transport adapter for RP2040
 *
 * Two paths under one ops table:
 *
 *   - HW UART: pico_sdk uart_init + gpio_set_function(UART) + RX pull-up.
 *              Pin pair must be one of the silicon-fixed pairs documented
 *              in board YAML (UART0: 0/1, 12/13, 16/17, 28/29; UART1:
 *              4/5, 8/9, 20/21, 24/25).
 *
 *   - PIO UART: pio_uart_init on PIO1, with TX/RX physically SWAPPED
 *               relative to the operator's "tx_pin"/"rx_pin" labels.
 *               Lets PCBs with TX→TX wiring (no crossover) drive the
 *               bus on arbitrary GPIO pairs. Falls back to HW UART
 *               if PIO state-machine allocation fails.
 *
 * verify_binding() cross-checks gpio_get_function() so the idempotent
 * fast-path catches external pad clobbers — at boot a legacy
 * pin_config_apply_hardware sweep can re-route our pads after we've
 * bound them; without this check a later config sync finds active_*
 * matching and returns early while the SM pushes bytes into pads
 * routed elsewhere.
 */

#include "roboclaw_transport.h"
#include "uart_pin_pairs.h"
#include "pio_uart.h"

#include <string.h>

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

static uart_inst_t* s_hw_uart = NULL;
static uint8_t      s_tx_pin = 0xFF;
static uint8_t      s_rx_pin = 0xFF;
static uint8_t      s_instance = 0xFF;
static uint32_t     s_baud = 0;
static bool         s_pio_active = false;

static bool fas100_unused(void); /* dummy fwd to keep -Wunused happy */
static bool fas100_unused(void) { return false; }

static bool roboclaw_rp2040_open(uint8_t tx_pin, uint8_t rx_pin,
                                  uint32_t baud, bool pio_swap)
{
    (void)fas100_unused;
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;

    /* Idempotent fast-path: same params already bound. */
    if (s_tx_pin == tx_pin && s_rx_pin == rx_pin && s_instance == inst
        && s_baud == baud && s_pio_active == pio_swap
        && (s_hw_uart != NULL || s_pio_active)) {
        return true;
    }

    /* Tear down previous binding's pad functions so the old pins stop
     * driving the bus. */
    if (s_tx_pin != 0xFF) {
        gpio_set_function(s_tx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_tx_pin, GPIO_IN);
    }
    if (s_rx_pin != 0xFF) {
        gpio_set_function(s_rx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_rx_pin, GPIO_IN);
    }
    if (s_hw_uart != NULL && s_instance != inst) {
        uart_deinit(s_hw_uart);
    }
    if (s_pio_active && !pio_swap) {
        pio_uart_deinit();
    }

    if (pio_swap) {
        /* PIO path. Operator's tx_pin / rx_pin describe what the firmware
         * thinks is TX/RX based on the HW UART pair table — but on a
         * non-crossover PCB, the *physical* wire labeled tx_pin is
         * actually the controller's TX, so we RECEIVE on it. So:
         *   PIO TX = rx_pin (we drive the wire wired to controller S1)
         *   PIO RX = tx_pin (we listen on the wire wired to controller S2)
         * PIO0 sm0 is owned by the NeoPixel, so hosts both SMs on PIO1. */
        if (!pio_uart_init(pio1,
                           /*tx=*/ rx_pin,
                           /*rx=*/ tx_pin,
                           baud)) {
            /* PIO init failed (no free SMs/instructions). Fall back to
             * HW UART so the caller still gets a working path. */
            s_pio_active = false;
        } else {
            /* Belt-and-suspenders idle-HIGH defense. pio_uart_rx_program_init
             * already pulls up the RX pin; we reassert both pins here in
             * case anything later in boot left the pad in a weird state. */
            gpio_pull_up(tx_pin);  /* physical RX (we receive on this) */
            gpio_pull_up(rx_pin);  /* physical TX (we drive on this)  */
            s_hw_uart = NULL;
            s_pio_active = true;
        }
    }

    if (!s_pio_active) {
        s_hw_uart = (inst == 0) ? uart0 : uart1;
        uart_init(s_hw_uart, baud);
        gpio_set_function(tx_pin, GPIO_FUNC_UART);
        gpio_set_function(rx_pin, GPIO_FUNC_UART);
        /* Defensive pull-up on RX. RoboClaw S2 drives push-pull in
         * Packet Serial mode (manual p. 8), but during boot/OTA/brown-
         * outs the line can briefly float and the UART will mis-trigger
         * on noise. Internal ~50–80 kΩ doesn't fight the controller's
         * driver during normal operation. */
        gpio_pull_up(rx_pin);
    }

    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_instance = inst;
    s_baud = baud;
    return true;
}

static bool roboclaw_rp2040_verify_binding(void)
{
    if (s_tx_pin == 0xFF || s_rx_pin == 0xFF) return false;
    uint expected_fn = s_pio_active ? GPIO_FUNC_PIO1 : GPIO_FUNC_UART;
    uint fn_tx = gpio_get_function(s_tx_pin);
    uint fn_rx = gpio_get_function(s_rx_pin);
    return fn_tx == expected_fn && fn_rx == expected_fn;
}

static bool roboclaw_rp2040_is_open(void)
{
    if (s_pio_active) return pio_uart_is_active();
    return s_hw_uart != NULL;
}

static bool roboclaw_rp2040_write(const uint8_t* data, size_t len)
{
    if (s_pio_active) {
        pio_uart_write_blocking(data, len);
        return true;
    }
    if (!s_hw_uart) return false;
    uart_write_blocking(s_hw_uart, data, len);
    return true;
}

static size_t roboclaw_rp2040_read(uint8_t* data, size_t max_len)
{
    if (max_len == 0) return 0;
    size_t n = 0;
    if (s_pio_active) {
        while (n < max_len && pio_uart_is_readable()) {
            data[n++] = pio_uart_getc();
        }
        return n;
    }
    if (!s_hw_uart) return 0;
    while (n < max_len && uart_is_readable(s_hw_uart)) {
        data[n++] = (uint8_t)uart_getc(s_hw_uart);
    }
    return n;
}

static bool roboclaw_rp2040_pio_swap_active(void)
{
    return s_pio_active;
}

static uint8_t roboclaw_rp2040_resolved_instance(void)
{
    return s_instance;
}

static const roboclaw_transport_ops_t s_ops = {
    .name              = "rp2040-uart",
    .open              = roboclaw_rp2040_open,
    .verify_binding    = roboclaw_rp2040_verify_binding,
    .is_open           = roboclaw_rp2040_is_open,
    .write             = roboclaw_rp2040_write,
    .read              = roboclaw_rp2040_read,
    .pio_swap_active   = roboclaw_rp2040_pio_swap_active,
    .resolved_instance = roboclaw_rp2040_resolved_instance,
};

const roboclaw_transport_ops_t* roboclaw_get_transport(void)
{
    return &s_ops;
}

/* ── E-stop GPIO helpers (referenced as extern from shared driver) ── */

void roboclaw_estop_apply_pin(uint8_t pin)
{
    if (pin == 0 || pin > 29) return;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);   /* LOW = deasserted = motor enabled */
}

void roboclaw_estop_assert_pin(uint8_t pin)
{
    if (pin == 0 || pin > 29) return;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);   /* HIGH = asserted = motor disabled / S3 latched */
}
