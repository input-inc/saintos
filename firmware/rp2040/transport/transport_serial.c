/**
 * SAINT.OS Node Firmware - Serial Transport Implementation
 *
 * micro-ROS transport over UART for Renode simulation.
 * Uses UART1 for micro-ROS communication (UART0 is for debug output).
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "saint_node.h"
#include "transport_serial.h"

#include <uxr/client/transport.h>

// =============================================================================
// Configuration
// =============================================================================

// Use UART1 for micro-ROS (UART0 is debug output in simulation mode)
#define UROS_UART           uart1
#define UROS_UART_TX_PIN    4   // GP4
#define UROS_UART_RX_PIN    5   // GP5
#define UROS_UART_BAUD      115200

// =============================================================================
// State Variables
// =============================================================================

static bool initialized = false;

// =============================================================================
// Public Functions - Initialization
// =============================================================================

bool transport_serial_init(void)
{
    printf("Initializing serial transport for simulation...\n");

    // Initialize UART1 for micro-ROS
    uart_init(UROS_UART, UROS_UART_BAUD);

    // Set up GPIO pins for UART1
    gpio_set_function(UROS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UROS_UART_RX_PIN, GPIO_FUNC_UART);

    // Disable flow control
    uart_set_hw_flow(UROS_UART, false, false);

    // Set format: 8N1
    uart_set_format(UROS_UART, 8, 1, UART_PARITY_NONE);

    // Enable FIFO
    uart_set_fifo_enabled(UROS_UART, true);

    printf("Serial transport initialized (UART1 @ %d baud)\n", UROS_UART_BAUD);
    initialized = true;

    return true;
}

bool transport_serial_connect(void)
{
    if (!initialized) {
        return false;
    }

    // Serial is always "connected" - no link detection
    printf("Serial transport ready\n");
    return true;
}

bool transport_serial_is_connected(void)
{
    return initialized;
}

// =============================================================================
// micro-ROS Transport Interface
// =============================================================================

bool transport_serial_open(struct uxrCustomTransport* transport)
{
    (void)transport;

    if (!initialized) {
        printf("Transport open failed: not initialized\n");
        return false;
    }

    printf("micro-ROS serial transport opened\n");
    return true;
}

bool transport_serial_close(struct uxrCustomTransport* transport)
{
    (void)transport;
    printf("micro-ROS serial transport closed\n");
    return true;
}

size_t transport_serial_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err)
{
    (void)transport;

    if (!initialized) {
        *err = 1;
        return 0;
    }

    // Write data to UART
    uart_write_blocking(UROS_UART, buf, len);

    *err = 0;
    return len;
}

size_t transport_serial_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err)
{
    (void)transport;

    if (!initialized) {
        *err = 1;
        return 0;
    }

    size_t received = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (received < len) {
        // Check if data is available
        if (uart_is_readable(UROS_UART)) {
            buf[received++] = uart_getc(UROS_UART);
        } else {
            // Check timeout
            uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start;
            if (elapsed >= (uint32_t)timeout_ms) {
                break;
            }
            // Small delay before retry
            sleep_us(100);
        }
    }

    *err = 0;
    return received;
}
