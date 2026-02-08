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
    gpio_put(ETH_PIN_CS, 0);
}

/**
 * Deselect W5500 (CS high).
 */
static void wizchip_cs_deselect(void)
{
    gpio_put(ETH_PIN_CS, 1);
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

    // Initialize SPI pins
    gpio_set_function(ETH_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ETH_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ETH_PIN_MISO, GPIO_FUNC_SPI);

    // Initialize CS pin (directly controlled)
    gpio_init(ETH_PIN_CS);
    gpio_set_dir(ETH_PIN_CS, GPIO_OUT);
    gpio_put(ETH_PIN_CS, 1);  // Deselect

    // Initialize reset pin
    gpio_init(ETH_PIN_RST);
    gpio_set_dir(ETH_PIN_RST, GPIO_OUT);

    // Hardware reset
    gpio_put(ETH_PIN_RST, 0);
    sleep_ms(10);
    gpio_put(ETH_PIN_RST, 1);
    sleep_ms(50);

    // Initialize SPI at 10MHz
    spi_init(W5500_SPI, 10 * 1000 * 1000);
    spi_set_format(W5500_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Register callbacks with ioLibrary
    reg_wizchip_cris_cbfunc(wizchip_cris_enter, wizchip_cris_exit);
    reg_wizchip_cs_cbfunc(wizchip_cs_select, wizchip_cs_deselect);
    reg_wizchip_spi_cbfunc(wizchip_spi_readbyte, wizchip_spi_writebyte);
    reg_wizchip_spiburst_cbfunc(wizchip_spi_readburst, wizchip_spi_writeburst);

    // Initialize W5500
    uint8_t memsize[2][8] = {
        {2, 2, 2, 2, 2, 2, 2, 2},  // RX buffer sizes for sockets 0-7 (2KB each)
        {2, 2, 2, 2, 2, 2, 2, 2}   // TX buffer sizes for sockets 0-7 (2KB each)
    };

    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1) {
        printf("W5500 memory initialization failed\n");
        return false;
    }

    // Check chip version
    uint8_t version = getVERSIONR();
    if (version != 0x04) {  // W5500 should return 0x04
        printf("W5500 not detected (version: 0x%02X)\n", version);
        return false;
    }

    printf("W5500 initialized (version: 0x%02X)\n", version);
    return true;
}

/**
 * Check physical link status.
 */
bool wizchip_port_link_up(void)
{
    uint8_t phycfgr = getPHYCFGR();
    return (phycfgr & PHYCFGR_LNK_ON) != 0;
}
