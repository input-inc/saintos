/**
 * SAINT.OS RP2040 OTA Bootloader - W5500 Init Helper
 *
 * Brings up the W5500 hardware enough to do DHCP and HTTP. Re-uses
 * the existing `wizchip_port_init()` from the runtime firmware (the
 * bootloader links the same wizchip_port.c). MAC address is derived
 * deterministically from the RP2040 unique chip ID so the bootloader
 * and the runtime firmware request the same DHCP lease.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/unique_id.h"
#include "wizchip_conf.h"
#include "socket.h"

/* Provided by transport/wizchip_port.c (linked into the bootloader
 * alongside the W5500 ioLibrary). Configures SPI, CS, RST and the
 * ioLibrary's read/write/cris callbacks. */
extern bool wizchip_port_init(void);
extern bool wizchip_port_link_up(void);

bool bootloader_w5500_init(uint8_t out_mac[6])
{
	if (!wizchip_port_init()) {
		printf("bootloader-w5500: wizchip_port_init() failed\n");
		return false;
	}

	/* Derive a locally administered MAC from the RP2040 unique ID — same
	 * scheme used by transport_w5500.c so a single board keeps the same
	 * DHCP identity across bootloader and runtime. */
	pico_unique_board_id_t uid;
	pico_get_unique_board_id(&uid);
	uint8_t mac[6];
	mac[0] = 0x02;  /* locally administered, unicast */
	for (int i = 0; i < 5; i++) {
		mac[i + 1] = uid.id[i];
	}
	setSHAR(mac);
	memcpy(out_mac, mac, 6);

	/* Wait for ethernet link briefly — the bootloader has no patience
	 * to wait forever; if the cable is unplugged, fail and let the
	 * boot path retry. */
	uint32_t wait_ms = 0;
	while (!wizchip_port_link_up() && wait_ms < 5000) {
		busy_wait_ms(50);
		wait_ms += 50;
	}
	if (!wizchip_port_link_up()) {
		printf("bootloader-w5500: ethernet link down after %ums\n",
		       (unsigned)wait_ms);
		return false;
	}

	printf("bootloader-w5500: MAC %02X:%02X:%02X:%02X:%02X:%02X, link up\n",
	       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return true;
}
