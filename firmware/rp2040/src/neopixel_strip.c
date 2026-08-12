/**
 * SAINT.OS RP2040 — external NeoPixel (WS2812) strips.
 *
 * Implements neopixel_strip.h for operator-added NeoPixels on an
 * arbitrary data GPIO (catalog type "neopixel"). Each strip gets its
 * own PIO state machine running the same 4-instruction WS2812 program
 * led_status.c uses for the onboard pixel; the program is loaded at
 * most once per PIO block and shared by every strip on that block.
 * The onboard status pixel is unrelated — it stays on led_status.c's
 * hardcoded pio0/sm0.
 *
 * Resource notes: the RP2040 has 2 PIO blocks x 4 state machines.
 * led_status uses pio0 sm0 WITHOUT claiming it (legacy), so we claim
 * it on its behalf before asking for unused SMs — otherwise
 * pio_claim_unused_sm could hand out sm0 and corrupt the status
 * pixel. The RoboClaw PIO-UART path (uart_swap) claims 2 SMs of its
 * own; with the 4-strip cap there is always headroom.
 *
 * SIMULATION builds are log-only stubs (Renode doesn't model the PIO
 * well enough for WS2812 timing — same guard led_status uses), and
 * neopixel_strip_exists() returns false so set_channel routing in sim
 * matches the Teensy stub's behavior exactly.
 *
 * Lifecycle mirrors the Teensy driver: neopixel_strip_reset() at the
 * head of every config apply, then neopixel_strip_add() per strip in
 * the new peripheral list.
 */

#include <string.h>
#include <stdio.h>

#include "platform.h"
#include "neopixel_strip.h"
#include "saint_log.h"

#define NEOPIXEL_MAX_STRIPS  4
#define NEOPIXEL_MAX_PIXELS  300
#define NEOPIXEL_ID_LEN      32

#ifndef SIMULATION

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/time.h"

/* Same WS2812 program as led_status.c (from the Pico examples). */
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

typedef struct {
    char            id[NEOPIXEL_ID_LEN];
    PIO             pio;
    int             sm;          /* -1 when the slot is free */
    uint8_t         pin;
    uint16_t        count;
    uint8_t         r, g, b;     /* last commanded color (pre-brightness) */
    uint8_t         brightness;
    absolute_time_t last_show_end;
    bool            active;
} strip_t;

static strip_t g_strips[NEOPIXEL_MAX_STRIPS];

/* Per-PIO-block offset of the shared WS2812 program (-1 = not loaded). */
static int g_prog_offset[2] = { -1, -1 };

static int block_index(PIO pio) { return (pio == pio0) ? 0 : 1; }

static strip_t* find(const char* id)
{
    if (!id || !*id) return NULL;
    for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
        if (g_strips[i].active && strcmp(g_strips[i].id, id) == 0) {
            return &g_strips[i];
        }
    }
    return NULL;
}

/* Reserve led_status's unclaimed pio0/sm0 exactly once so
 * pio_claim_unused_sm can never grab the status pixel's SM. */
static void reserve_led_status_sm(void)
{
    static bool done = false;
    if (done) return;
    done = true;
    if (!pio_sm_is_claimed(pio0, 0)) {
        pio_sm_claim(pio0, 0);
    }
}

/* Claim an SM (pio0 first, then pio1) and make sure the WS2812 program
 * is loaded on that block. Returns false when both blocks are out of
 * SMs or instruction space. */
static bool claim_resources(strip_t* s)
{
    PIO blocks[2] = { pio0, pio1 };
    for (int b = 0; b < 2; b++) {
        PIO pio = blocks[b];
        if (g_prog_offset[block_index(pio)] < 0) {
            if (!pio_can_add_program(pio, &ws2812_program)) continue;
            g_prog_offset[block_index(pio)] =
                (int)pio_add_program(pio, &ws2812_program);
        }
        int sm = pio_claim_unused_sm(pio, false);
        if (sm < 0) continue;
        s->pio = pio;
        s->sm  = sm;
        return true;
    }
    return false;
}

static void configure_sm(strip_t* s)
{
    uint offset = (uint)g_prog_offset[block_index(s->pio)];
    pio_gpio_init(s->pio, s->pin);
    pio_sm_set_consecutive_pindirs(s->pio, (uint)s->sm, s->pin, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, s->pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = 10;  /* matches led_status's program timing */
    float div = clock_get_hz(clk_sys) / (800000.0f * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(s->pio, (uint)s->sm, offset, &c);
    pio_sm_set_enabled(s->pio, (uint)s->sm, true);
}

/* Push one full frame: every pixel at (color x brightness). WS2812
 * needs a >50 us low gap to latch the previous frame — wait it out
 * when two renders land back-to-back (color + brightness arrive as
 * separate set_channel writes). Mirrors Adafruit_NeoPixel's endTime
 * guard on the Teensy. */
static void render(strip_t* s, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s->active || s->sm < 0) return;

    busy_wait_until(delayed_by_us(s->last_show_end, 300));

    uint32_t grb = ((uint32_t)(g * s->brightness / 255) << 16) |
                   ((uint32_t)(r * s->brightness / 255) << 8)  |
                   ((uint32_t)(b * s->brightness / 255));
    for (uint16_t i = 0; i < s->count; i++) {
        pio_sm_put_blocking(s->pio, (uint)s->sm, grb << 8);
    }
    /* FIFO drained != wire drained; stamp when the last bit leaves
     * (24 bits / 800 kHz = 30 us per pixel still in the FIFO — 8-deep
     * joined FIFO bounds it, but being generous costs nothing). */
    s->last_show_end = delayed_by_us(get_absolute_time(), 8 * 30);
}

static void release_slot(strip_t* s)
{
    if (s->sm >= 0) {
        /* Blank before tearing down so a removed strip doesn't latch
         * its last frame on the LEDs. */
        render(s, 0, 0, 0);
        pio_sm_set_enabled(s->pio, (uint)s->sm, false);
        pio_sm_unclaim(s->pio, (uint)s->sm);
        s->sm = -1;
    }
    s->active = false;
    s->id[0]  = '\0';
}

void neopixel_strip_reset(void)
{
    for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
        if (g_strips[i].active) release_slot(&g_strips[i]);
    }
}

bool neopixel_strip_add(const char* id, uint8_t pin, uint16_t count)
{
    if (!id || !*id) {
        saint_log_publish("warn", "NeoPixel: strip add ignored — empty id");
        return false;
    }
    if (pin > 29) {
        saint_log_publish("warn",
            "NeoPixel: '%s' data pin %u out of range (RP2040 GPIO 0-29)",
            id, (unsigned)pin);
        return false;
    }
    if (count == 0) count = 1;
    if (count > NEOPIXEL_MAX_PIXELS) {
        saint_log_publish("warn", "NeoPixel: '%s' count %u clamped to %u",
                          id, (unsigned)count, NEOPIXEL_MAX_PIXELS);
        count = NEOPIXEL_MAX_PIXELS;
    }

    reserve_led_status_sm();

    strip_t* s = find(id);
    if (!s) {
        for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
            if (!g_strips[i].active) { s = &g_strips[i]; s->sm = -1; break; }
        }
    }
    if (!s) {
        saint_log_publish("warn",
            "NeoPixel: strip table full (%u) — '%s' on pin %u dropped",
            NEOPIXEL_MAX_STRIPS, id, (unsigned)pin);
        return false;
    }

    /* Re-point: a changed pin needs a fresh SM setup; a changed count
     * only changes how many words render pushes. */
    bool need_sm_setup = (s->sm < 0) || (s->active && s->pin != pin);
    if (s->active && s->pin != pin && s->sm >= 0) {
        pio_sm_set_enabled(s->pio, (uint)s->sm, false);
        pio_sm_unclaim(s->pio, (uint)s->sm);
        s->sm = -1;
    }
    if (need_sm_setup && s->sm < 0) {
        if (!claim_resources(s)) {
            saint_log_publish("warn",
                "NeoPixel: no free PIO state machine for '%s' on pin %u — "
                "dropped (led_status + PIO UART + other strips hold them)",
                id, (unsigned)pin);
            s->active = false;
            return false;
        }
    }

    strncpy(s->id, id, NEOPIXEL_ID_LEN - 1);
    s->id[NEOPIXEL_ID_LEN - 1] = '\0';
    s->pin           = pin;
    s->count         = count;
    s->r = s->g = s->b = 0;
    s->brightness    = 255;
    s->last_show_end = get_absolute_time();
    s->active        = true;
    if (need_sm_setup) configure_sm(s);
    render(s, 0, 0, 0);   /* start dark — operator routes it on */

    saint_log_publish("info", "NeoPixel: strip '%s' = %u px on GPIO %u (PIO%d/sm%d)",
                      id, (unsigned)count, (unsigned)pin,
                      block_index(s->pio), s->sm);
    return true;
}

bool neopixel_strip_exists(const char* id)
{
    return find(id) != NULL;
}

bool neopixel_strip_set_color(const char* id, uint32_t rgb)
{
    strip_t* s = find(id);
    if (!s) return false;
    s->r = (uint8_t)((rgb >> 16) & 0xFF);
    s->g = (uint8_t)((rgb >>  8) & 0xFF);
    s->b = (uint8_t)( rgb        & 0xFF);
    render(s, s->r, s->g, s->b);
    return true;
}

bool neopixel_strip_set_brightness(const char* id, uint8_t brightness)
{
    strip_t* s = find(id);
    if (!s) return false;
    s->brightness = brightness;
    render(s, s->r, s->g, s->b);
    return true;
}

void neopixel_strip_all_off(void)
{
    /* Dark, but keep the stored color — clear_estop + a fresh write
     * restores operator state, same as the Teensy driver. */
    for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
        if (g_strips[i].active) render(&g_strips[i], 0, 0, 0);
    }
}

#else  /* SIMULATION — Renode has no WS2812-capable PIO model; log-only
        * stubs, exists() false so set_channel routing matches the
        * Teensy sim stub. */

void neopixel_strip_reset(void) {}

bool neopixel_strip_add(const char* id, uint8_t pin, uint16_t count)
{
    printf("NeoPixel [SIM]: would add '%s' = %u px on GPIO %u\n",
           id ? id : "?", (unsigned)count, (unsigned)pin);
    return true;
}

bool neopixel_strip_exists(const char* id) { (void)id; return false; }

bool neopixel_strip_set_color(const char* id, uint32_t rgb)
{
    printf("NeoPixel [SIM]: '%s' color=0x%06lX\n",
           id ? id : "?", (unsigned long)(rgb & 0xFFFFFF));
    return true;
}

bool neopixel_strip_set_brightness(const char* id, uint8_t brightness)
{
    printf("NeoPixel [SIM]: '%s' brightness=%u\n",
           id ? id : "?", (unsigned)brightness);
    return true;
}

void neopixel_strip_all_off(void) {}

#endif /* SIMULATION */
