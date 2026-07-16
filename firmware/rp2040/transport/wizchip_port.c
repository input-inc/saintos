/**
 * SAINT.OS Node Firmware - WIZnet ioLibrary Platform Port
 *
 * Provides SPI callbacks for WIZnet ioLibrary on RP2040.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "wizchip_conf.h"
#include "w5500.h"
#include "saint_node.h"

// SPI instance
#define W5500_SPI       spi0

// Critical section for thread safety
static critical_section_t wizchip_crit_sec;
static bool crit_sec_initialized = false;

// Active W5500 chip-select GPIO. Auto-detected in wizchip_port_init() by
// probing candidate pins; the CS callbacks below drive whichever one
// actually found the chip. Defaults to the standard pin so anything that
// reads it before init sees a sane value.
static uint8_t active_cs_pin = ETH_PIN_CS;

// =============================================================================
// Callback Functions for ioLibrary
// =============================================================================

/**
 * Enter critical section (disable interrupts).
 */
static void wizchip_cris_enter(void)
{
    critical_section_enter_blocking(&wizchip_crit_sec);
}

/**
 * Exit critical section (restore interrupts).
 */
static void wizchip_cris_exit(void)
{
    critical_section_exit(&wizchip_crit_sec);
}

/**
 * Select W5500 (CS low).
 */
static void wizchip_cs_select(void)
{
    gpio_put(active_cs_pin, 0);
}

/**
 * Deselect W5500 (CS high).
 */
static void wizchip_cs_deselect(void)
{
    gpio_put(active_cs_pin, 1);
}

/**
 * Read single byte via SPI.
 */
static uint8_t wizchip_spi_readbyte(void)
{
    uint8_t rx;
    spi_read_blocking(W5500_SPI, 0x00, &rx, 1);
    return rx;
}

/**
 * Write single byte via SPI.
 */
static void wizchip_spi_writebyte(uint8_t wb)
{
    spi_write_blocking(W5500_SPI, &wb, 1);
}

/**
 * Read multiple bytes via SPI (burst).
 */
static void wizchip_spi_readburst(uint8_t* pBuf, uint16_t len)
{
    spi_read_blocking(W5500_SPI, 0x00, pBuf, len);
}

/**
 * Write multiple bytes via SPI (burst).
 */
static void wizchip_spi_writeburst(uint8_t* pBuf, uint16_t len)
{
    spi_write_blocking(W5500_SPI, pBuf, len);
}

// =============================================================================
// Public Functions
// =============================================================================

/**
 * Initialize the WIZnet chip platform layer.
 * Must be called before any other wizchip functions.
 */
bool wizchip_port_init(void)
{
    // Initialize critical section
    if (!crit_sec_initialized) {
        critical_section_init(&wizchip_crit_sec);
        crit_sec_initialized = true;
    }

    // Initialize SPI pins (shared by every candidate CS)
    gpio_set_function(ETH_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ETH_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ETH_PIN_MISO, GPIO_FUNC_SPI);

    // Initialize SPI at 10MHz
    spi_init(W5500_SPI, 10 * 1000 * 1000);
    spi_set_format(W5500_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Register callbacks with ioLibrary. The CS callback drives
    // active_cs_pin, which the probe below points at each candidate.
    reg_wizchip_cris_cbfunc(wizchip_cris_enter, wizchip_cris_exit);
    reg_wizchip_cs_cbfunc(wizchip_cs_select, wizchip_cs_deselect);
    reg_wizchip_spi_cbfunc(wizchip_spi_readbyte, wizchip_spi_writebyte);
    reg_wizchip_spiburst_cbfunc(wizchip_spi_readburst, wizchip_spi_writeburst);

    uint8_t memsize[2][8] = {
        {2, 2, 2, 2, 2, 2, 2, 2},  // RX buffer sizes for sockets 0-7 (2KB each)
        {2, 2, 2, 2, 2, 2, 2, 2}   // TX buffer sizes for sockets 0-7 (2KB each)
    };

    // Auto-detect the chip-select pin. Most nodes wire CS to ETH_PIN_CS
    // (pin 10); boards that repurpose pin 10 move CS to ETH_PIN_CS_ALT
    // (pin 22). We probe the ALTERNATE first so that on a repurposed
    // board we never drive pin 10 (which is doing something else there);
    // on a standard board the alt pin is unconnected, reads a bad version,
    // and we fall through to pin 10.
    //
    // No hardware reset is needed: CW_INIT_WIZCHIP performs a software
    // reset over SPI (wizchip_sw_reset -> MR_RST), so probing is purely
    // "select this CS, init, read the version register." The W5500
    // returns 0x04; a wrong/unconnected CS reads 0x00 or 0xFF.
    static const uint8_t cs_candidates[] = { ETH_PIN_CS_ALT, ETH_PIN_CS };

    for (unsigned i = 0; i < sizeof(cs_candidates); i++) {
        uint8_t cs = cs_candidates[i];

        gpio_init(cs);
        gpio_set_dir(cs, GPIO_OUT);
        gpio_put(cs, 1);           // deselect
        active_cs_pin = cs;

        if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1) {
            printf("W5500 init failed on CS pin %u\n", cs);
        } else {
            uint8_t version = getVERSIONR();
            if (version == 0x04) {  // W5500 identifies as 0x04
                printf("W5500 initialized (version: 0x%02X) on CS pin %u\n",
                       version, cs);
                return true;
            }
            printf("No W5500 on CS pin %u (version: 0x%02X)\n", cs, version);
        }

        // Release this pin (back to hi-Z) before trying the next one, so a
        // failed probe doesn't leave a repurposed pin held as an output.
        gpio_deinit(cs);
    }

    active_cs_pin = ETH_PIN_CS;  // restore default for any later reads
    printf("W5500 not detected on any candidate CS pin (%u, %u)\n",
           ETH_PIN_CS_ALT, ETH_PIN_CS);
    return false;
}

/**
 * Check physical link status.
 */
bool wizchip_port_link_up(void)
{
    uint8_t phycfgr = getPHYCFGR();
    return (phycfgr & PHYCFGR_LNK_ON) != 0;
}
