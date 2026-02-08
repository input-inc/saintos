/**
 * SAINT.OS Node Firmware - LED Status Indicator
 *
 * Controls the onboard NeoPixel and LED to indicate node state.
 * Uses PIO for WS2812 NeoPixel control.
 *
 * Note: PIO is not emulated in Renode, so LED functions are disabled
 * in simulation mode to avoid warnings.
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "saint_node.h"

// =============================================================================
// State Variables
// =============================================================================

static node_state_t current_state = NODE_STATE_BOOT;

#ifndef SIMULATION
// Hardware-only: PIO for NeoPixel

#include "hardware/pio.h"
#include "hardware/clocks.h"

// =============================================================================
// WS2812 PIO Program
// =============================================================================

// Simple WS2812 PIO program (inline for simplicity)
// Based on Pico examples
static const uint16_t ws2812_program_instructions[] = {
    //     .wrap_target
    0x6221, //  0: out    x, 1            side 0 [2]
    0x1123, //  1: jmp    !x, 3           side 1 [1]
    0x1400, //  2: jmp    0               side 1 [4]
    0xa442, //  3: nop                    side 0 [4]
    //     .wrap
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

static PIO pio = pio0;
static uint sm = 0;
static uint32_t last_update_ms = 0;

#ifndef SIMULATION
// =============================================================================
// Color Definitions (GRB format for WS2812) - Hardware only
// =============================================================================

typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} rgb_color_t;

static const rgb_color_t COLOR_BLUE    = {.g = 0,   .r = 0,   .b = 255};
static const rgb_color_t COLOR_YELLOW  = {.g = 255, .r = 255, .b = 0};
static const rgb_color_t COLOR_RED     = {.g = 0,   .r = 255, .b = 0};
static const rgb_color_t COLOR_ORANGE  = {.g = 165, .r = 255, .b = 0};
static const rgb_color_t COLOR_GREEN   = {.g = 255, .r = 0,   .b = 0};
static const rgb_color_t COLOR_WHITE   = {.g = 255, .r = 255, .b = 255};
static const rgb_color_t COLOR_PURPLE  = {.g = 0,   .r = 128, .b = 128};
static const rgb_color_t COLOR_OFF     = {.g = 0,   .r = 0,   .b = 0};

/**
 * Get color for state.
 */
static rgb_color_t get_state_color(node_state_t state)
{
    switch (state) {
        case NODE_STATE_BOOT:       return COLOR_BLUE;
        case NODE_STATE_CONNECTING: return COLOR_YELLOW;
        case NODE_STATE_UNADOPTED:  return COLOR_ORANGE;
        case NODE_STATE_ADOPTING:   return COLOR_WHITE;
        case NODE_STATE_ACTIVE:     return COLOR_GREEN;
        case NODE_STATE_ERROR:      return COLOR_PURPLE;
        default:                    return COLOR_OFF;
    }
}
#endif // !SIMULATION

// =============================================================================
// Private Functions
// =============================================================================

/**
 * Initialize WS2812 PIO program.
 */
static void ws2812_init(uint pin, float freq)
{
    uint offset = pio_add_program(pio, &ws2812_program);

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = 10;  // 2 + 2 + 6
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

/**
 * Set NeoPixel color.
 */
static void set_neopixel(rgb_color_t color, uint8_t brightness)
{
    // Scale by brightness (0-255)
    uint32_t grb = ((uint32_t)(color.g * brightness / 255) << 16) |
                   ((uint32_t)(color.r * brightness / 255) << 8) |
                   ((uint32_t)(color.b * brightness / 255));

    pio_sm_put_blocking(pio, sm, grb << 8);
}

#endif // !SIMULATION

// =============================================================================
// Public Functions
// =============================================================================

/**
 * Initialize LED subsystem.
 */
void led_init(void)
{
#ifdef SIMULATION
    // PIO not available in simulation
    printf("LED initialized (simulation mode - NeoPixel disabled)\n");
#else
    // Initialize WS2812 NeoPixel
    ws2812_init(NEOPIXEL_PIN, 800000);  // 800kHz for WS2812

    // Initialize standard LED
    gpio_init(GPIO_D13);
    gpio_set_dir(GPIO_D13, GPIO_OUT);

    // Set initial state
    set_neopixel(COLOR_BLUE, 64);
    gpio_put(GPIO_D13, 1);

    printf("LED initialized\n");
#endif
}

/**
 * Set LED state.
 */
void led_set_state(node_state_t state)
{
    current_state = state;
}

/**
 * Flash LED to identify the node.
 * Blocks for the duration of the flashing pattern.
 *
 * @param flash_count Number of flash sequences (default 5)
 */
void led_identify(uint8_t flash_count)
{
    if (flash_count == 0) flash_count = 5;

    printf("LED identify: flashing %d times\n", flash_count);

#ifdef SIMULATION
    // In simulation, just print the identify request
    printf("LED identify: [SIMULATION] Would flash white %d times\n", flash_count);
    sleep_ms(flash_count * 400);  // Simulate timing
#else
    // Save current state to restore after
    node_state_t saved_state = current_state;

    // Flash white on/off pattern - distinctive and visible
    for (uint8_t i = 0; i < flash_count; i++) {
        // Bright white flash
        set_neopixel(COLOR_WHITE, 255);
        gpio_put(GPIO_D13, 1);
        sleep_ms(150);

        // Off
        set_neopixel(COLOR_OFF, 0);
        gpio_put(GPIO_D13, 0);
        sleep_ms(150);

        // Quick double-flash for distinctiveness
        set_neopixel(COLOR_WHITE, 255);
        gpio_put(GPIO_D13, 1);
        sleep_ms(50);

        set_neopixel(COLOR_OFF, 0);
        gpio_put(GPIO_D13, 0);
        sleep_ms(50);
    }

    // Restore previous state
    current_state = saved_state;
#endif
}

/**
 * Update LED animation.
 * Should be called from main loop.
 */
void led_update(void)
{
#ifdef SIMULATION
    // No LED updates in simulation (PIO not emulated)
    (void)current_state;
#else
    uint32_t now = to_ms_since_boot(get_absolute_time());
    rgb_color_t color = get_state_color(current_state);

    switch (current_state) {
        case NODE_STATE_BOOT:
        case NODE_STATE_ACTIVE:
            // Solid color
            set_neopixel(color, 64);
            gpio_put(GPIO_D13, 1);
            break;

        case NODE_STATE_CONNECTING:
        case NODE_STATE_UNADOPTED:
            // Pulsing effect
            {
                // Create pulsing brightness (0-127, 1 second period)
                uint32_t phase = (now % 1000);
                uint8_t brightness;
                if (phase < 500) {
                    brightness = (phase * 127) / 500;
                } else {
                    brightness = ((1000 - phase) * 127) / 500;
                }
                brightness += 16;  // Minimum brightness

                set_neopixel(color, brightness);

                // Blink standard LED at 2Hz
                gpio_put(GPIO_D13, (now / 250) % 2);
            }
            break;

        case NODE_STATE_ADOPTING:
            // Fast flashing
            {
                bool on = (now / 100) % 2;
                set_neopixel(on ? color : COLOR_OFF, 128);
                gpio_put(GPIO_D13, on);
            }
            break;

        case NODE_STATE_ERROR:
            // Slow red pulse
            {
                uint32_t phase = (now % 2000);
                uint8_t brightness;
                if (phase < 1000) {
                    brightness = (phase * 255) / 1000;
                } else {
                    brightness = ((2000 - phase) * 255) / 1000;
                }

                set_neopixel(COLOR_RED, brightness);
                gpio_put(GPIO_D13, (now / 500) % 2);
            }
            break;
    }

    last_update_ms = now;
#endif
}
