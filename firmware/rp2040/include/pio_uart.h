/**
 * SAINT.OS Node Firmware - PIO UART
 *
 * Bit-banged UART over the RP2040's PIO peripheral so we can put TX
 * and RX on any GPIO. Used by the RoboClaw driver when the custom
 * PCB routes wires in a way that doesn't match the hardware UART's
 * silicon-fixed pin map (UART0 TX is GP0 only, UART0 RX is GP1 only).
 *
 * Public surface mirrors the Pico SDK's hardware-UART API the driver
 * already uses (uart_write_blocking / uart_is_readable / uart_getc /
 * uart_tx_wait_blocking) so the driver only needs a single dispatch
 * decision at init time and the per-byte code path stays identical.
 *
 * Resources: one PIO block (PIO1 by default — PIO0 sm0 is taken by
 * the NeoPixel in led_status.c), two state machines (one TX, one
 * RX), ~16 instruction slots out of 32, ~zero CPU at runtime (the
 * SMs run autonomously; bytes land in / leave the FIFO without ISR
 * involvement).
 */

#ifndef PIO_UART_H
#define PIO_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef SIMULATION
#include "hardware/pio.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bind two state machines on the given PIO block to a TX/RX GPIO pair
 * at the supplied baud rate. Safe to re-call with the same args (no-op
 * fast path). Calling with different pins or baud tears down and
 * re-binds. Returns true on success; false if the PIO can't hold both
 * programs (other PIO clients took too many instruction slots).
 *
 * `pio_block` is a pio_hw_t* (pio0 or pio1) on hardware; SIMULATION
 * builds accept NULL and short-circuit to a no-op.
 */
#ifndef SIMULATION
bool pio_uart_init(PIO pio_block, uint tx_pin, uint rx_pin, uint baud_rate);
#else
bool pio_uart_init(void* pio_block, unsigned tx_pin, unsigned rx_pin,
                   unsigned baud_rate);
#endif

/**
 * Tear down both state machines and release their PIO instruction
 * slots. The pins return to SIO (high-Z input). Safe to call when
 * nothing is bound.
 */
void pio_uart_deinit(void);

/**
 * Is the PIO UART currently bound and ready to send/receive?
 */
bool pio_uart_is_active(void);

/**
 * Block until `len` bytes have been pushed into the TX FIFO. Returns
 * once the bytes are queued — not when they've physically transmitted.
 * Use pio_uart_tx_wait_blocking() to flush.
 */
void pio_uart_write_blocking(const uint8_t* src, size_t len);

/**
 * Block until the TX FIFO is empty AND the TX state machine has
 * finished shifting out the last byte. Mirror of uart_tx_wait_blocking().
 */
void pio_uart_tx_wait_blocking(void);

/**
 * Is there at least one byte in the RX FIFO right now?
 */
bool pio_uart_is_readable(void);

/**
 * Pull one byte from the RX FIFO. Blocks if the FIFO is empty.
 * Caller is expected to check pio_uart_is_readable() first when
 * non-blocking behavior is required.
 */
uint8_t pio_uart_getc(void);

#ifdef __cplusplus
}
#endif

#endif // PIO_UART_H
