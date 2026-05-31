/**
 * SAINT.OS Firmware - Pololu Tic transport ops (shared)
 *
 * Standard 8N1 TTL serial. Both platforms use their HW UART:
 *   - RP2040: pico_sdk uart_init + GPIO_FUNC_UART
 *   - Teensy 4.1: HardwareSerial.begin()
 *
 * No PIO swap support — the Tic is typically wired through a level-
 * shifted UART on a development PCB, and we don't have the use case
 * for non-crossover routing yet (unlike RoboClaw). If it comes up,
 * mirror RoboClaw's transport.
 */

#ifndef SAINT_TIC_TRANSPORT_H
#define SAINT_TIC_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tic_transport_ops {
    const char* name;
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
    bool (*is_open)(void);
    bool (*write)(const uint8_t* data, size_t len);
    size_t (*read)(uint8_t* data, size_t max_len);
    uint8_t (*resolved_instance)(void);
} tic_transport_ops_t;

const tic_transport_ops_t* tic_get_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_TIC_TRANSPORT_H */
