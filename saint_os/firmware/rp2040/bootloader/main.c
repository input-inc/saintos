/**
 * SAINT.OS RP2040 OTA Bootloader
 *
 * Derived from picowota (Brian Starkey, BSD-3-Clause). Replaces its
 * cyw43+lwIP+TCP custom-protocol path with W5500 ioLibrary + an HTTP
 * GET against the gateway (the SAINT.OS Pi server is the DHCP server,
 * so the gateway IS the server). Image metadata (size / CRC32 / vtor)
 * comes through watchdog scratch registers set by the app via
 * saint_ota_reboot_with_image() before reboot.
 *
 * Flash layout:
 *
 *   0x10000000  Bootloader (us)            16 KB    bootloader_shell.ld
 *   0x10004000  image_header               256 B    set last, atomically
 *   0x10004100  App vector table + code    rest of slot
 *   0x101FF000  Persistent storage         4 KB     (existing flash_storage)
 *
 * Boot flow:
 *   1. We run first on every reset (bootloader linked at flash start).
 *   2. If watchdog scratch[5,6] == magic / ~magic:
 *        → enter OTA mode; pick up size/crc/vtor from scratch[0..2]
 *      else if image_header_ok(&app_image_header):
 *        → jump_to_vtor(app_image_header.vtor)  (i.e. run the app)
 *      else:
 *        → enter OTA mode (no valid app — recovery)
 *   3. OTA mode:
 *        - bring up W5500, DHCP-client → IP + gateway
 *        - GET http://<gateway>/api/firmware/rp2040/saint_node.bin
 *        - stream body into flash starting at WRITE_ADDR_MIN
 *          erase-on-sector-boundary, program in FLASH_PAGE_SIZE chunks
 *        - verify size matches Content-Length AND scratch[0]
 *        - verify computed CRC32 == scratch[1]
 *        - on success: write image header (the LAST step) and reboot
 *        - on failure: stay in bootloader, retry on next boot
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "RP2040.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/critical_section.h"
#include "hardware/flash.h"
#include "hardware/resets.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"

#include "wizchip_conf.h"
#include "socket.h"
#include "dhcp.h"

#include "picowota/reboot.h"
#include "http_client.h"
#include "http_transport_w5500.h"

/* ============================================================
 * Image header — placed in its own flash sector at the app slot
 * boundary. Filled in (last) by the bootloader on a successful
 * OTA, read (first) on every boot to decide whether to launch
 * the app.
 * ============================================================ */

struct image_header {
	uint32_t vtor;       /* address of the app's vector table  */
	uint32_t size;       /* size in bytes of the app body      */
	uint32_t crc;        /* CRC32 of bytes [vtor, vtor+size)   */
	uint8_t  pad[FLASH_PAGE_SIZE - 12];
};
_Static_assert(sizeof(struct image_header) == FLASH_PAGE_SIZE,
	       "image_header must equal one flash page");

extern struct image_header app_image_header;  /* placed by linker */

#define IMAGE_HEADER_ADDR    ((uint32_t)&app_image_header)
#define IMAGE_HEADER_OFFSET  (IMAGE_HEADER_ADDR - XIP_BASE)
#define WRITE_ADDR_MIN       (IMAGE_HEADER_ADDR + FLASH_SECTOR_SIZE)
#define FLASH_ADDR_MAX       (XIP_BASE + PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
/* (the trailing FLASH_SECTOR_SIZE is reserved for persistent storage) */

/* ============================================================
 * CRC32 — same polynomial/init/xorout as zlib/crc32; the gen_imghdr.py
 * helper and the server-side firmware-info compute the matching value.
 * Computed incrementally as the body streams in.
 * ============================================================ */

static uint32_t crc32_update(uint32_t crc, const uint8_t* buf, size_t len)
{
	crc ^= 0xFFFFFFFFu;
	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; i++) {
			crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1));
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

/* ============================================================
 * Image header read / write
 * ============================================================ */

static critical_section_t flash_cs;

static bool image_header_ok(const struct image_header* hdr)
{
	if (hdr->vtor < XIP_BASE || hdr->vtor >= FLASH_ADDR_MAX) return false;
	if (hdr->size == 0 || hdr->size > FLASH_ADDR_MAX - hdr->vtor) return false;

	uint32_t crc = crc32_update(0, (const uint8_t*)hdr->vtor, hdr->size);
	if (crc != hdr->crc) return false;

	const uint32_t* vtor = (const uint32_t*)hdr->vtor;
	/* Initial SP must point into SRAM. */
	if (vtor[0] < SRAM_BASE) return false;
	/* Reset handler must be inside the app and have the thumb bit set. */
	if (vtor[1] < hdr->vtor || vtor[1] > hdr->vtor + hdr->size) return false;
	if (!(vtor[1] & 1)) return false;
	return true;
}

static void write_image_header(uint32_t vtor, uint32_t size, uint32_t crc)
{
	struct image_header hdr;
	memset(&hdr, 0, sizeof(hdr));
	hdr.vtor = vtor;
	hdr.size = size;
	hdr.crc  = crc;

	critical_section_enter_blocking(&flash_cs);
	flash_range_erase(IMAGE_HEADER_OFFSET, FLASH_SECTOR_SIZE);
	flash_range_program(IMAGE_HEADER_OFFSET, (const uint8_t*)&hdr, sizeof(hdr));
	critical_section_exit(&flash_cs);
}

static void clear_ota_magic(void)
{
	watchdog_hw->scratch[5] = 0;
	watchdog_hw->scratch[6] = 0;
}

/* ============================================================
 * Jump-to-app
 * ============================================================ */

static void bl_disable_interrupts(void)
{
	SysTick->CTRL &= ~1;
	NVIC->ICER[0] = 0xFFFFFFFFu;
	NVIC->ICPR[0] = 0xFFFFFFFFu;
}

static void reset_peripherals(void)
{
	reset_block(~(
		RESETS_RESET_IO_QSPI_BITS |
		RESETS_RESET_PADS_QSPI_BITS |
		RESETS_RESET_SYSCFG_BITS |
		RESETS_RESET_PLL_SYS_BITS
	));
}

static void __attribute__((noreturn)) jump_to_vtor(uint32_t vtor)
{
	/* Originally derived from the Leaf Labs Cortex-M3 bootloader (MIT),
	 * carried forward from upstream picowota. */
	uint32_t reset_vector = *(volatile uint32_t*)(vtor + 0x04);
	SCB->VTOR = (volatile uint32_t)(vtor);
	asm volatile("msr msp, %0" :: "g" (*(volatile uint32_t*)vtor));
	asm volatile("bx %0" :: "r" (reset_vector));
	__builtin_unreachable();
}

/* ============================================================
 * Flash-staging state for the body callback
 * ============================================================ */

typedef struct {
	uint32_t write_offset;     /* relative to XIP_BASE — where the next byte goes */
	uint32_t crc32_running;    /* incremental CRC32 over what we've written */
	uint32_t bytes_written;    /* count of bytes streamed to flash */
	uint8_t  page_buf[FLASH_PAGE_SIZE];
	uint32_t page_buf_len;     /* 0..FLASH_PAGE_SIZE — bytes pending in page_buf */
	bool     error;            /* set true on any flash op failure */
} stage_ctx_t;

/* Erase exactly one sector at `offset` (relative to XIP_BASE). */
static void erase_sector(uint32_t offset)
{
	critical_section_enter_blocking(&flash_cs);
	flash_range_erase(offset, FLASH_SECTOR_SIZE);
	critical_section_exit(&flash_cs);
}

/* Program FLASH_PAGE_SIZE bytes at `offset`. */
static void program_page(uint32_t offset, const uint8_t* buf)
{
	critical_section_enter_blocking(&flash_cs);
	flash_range_program(offset, buf, FLASH_PAGE_SIZE);
	critical_section_exit(&flash_cs);
}

/* Flush page_buf (whatever's accumulated, zero-padded to FLASH_PAGE_SIZE). */
static void flush_page(stage_ctx_t* st)
{
	if (st->page_buf_len == 0) return;
	if (st->page_buf_len < FLASH_PAGE_SIZE) {
		memset(st->page_buf + st->page_buf_len, 0xFF,
		       FLASH_PAGE_SIZE - st->page_buf_len);
	}
	/* If this write starts a new sector, erase it first. */
	if ((st->write_offset & (FLASH_SECTOR_SIZE - 1)) == 0) {
		erase_sector(st->write_offset);
	}
	program_page(st->write_offset, st->page_buf);
	st->write_offset += FLASH_PAGE_SIZE;
	st->bytes_written += st->page_buf_len;
	st->page_buf_len = 0;
}

static int stage_body_cb(void* user, const uint8_t* data, size_t len)
{
	stage_ctx_t* st = (stage_ctx_t*)user;
	if (st->error) return -1;

	st->crc32_running = crc32_update(st->crc32_running, data, len);

	while (len > 0) {
		size_t free_in_page = FLASH_PAGE_SIZE - st->page_buf_len;
		size_t take = (len < free_in_page) ? len : free_in_page;
		memcpy(st->page_buf + st->page_buf_len, data, take);
		st->page_buf_len += take;
		data += take;
		len -= take;
		if (st->page_buf_len == FLASH_PAGE_SIZE) {
			flush_page(st);
		}
	}
	return 0;
}

/* ============================================================
 * W5500 init + DHCP
 * ============================================================ */

/* External init function we'll provide alongside (sets up SPI etc).
 * For now declared here; we'll define it in a small support file. */
bool bootloader_w5500_init(uint8_t out_mac[6]);

/* DHCP buffer + callback receivers. */
static uint8_t dhcp_buffer[1024];
static volatile bool dhcp_got_ip = false;
static uint8_t local_ip[4]   = {0};
static uint8_t gateway_ip[4] = {0};
static uint8_t subnet[4]     = {0};
static uint8_t dns_ip[4]     = {0};

static void dhcp_assign(void)
{
	getIPfromDHCP(local_ip);
	getGWfromDHCP(gateway_ip);
	getSNfromDHCP(subnet);
	getDNSfromDHCP(dns_ip);
	dhcp_got_ip = true;
}

static void dhcp_update(void) { dhcp_assign(); }
static void dhcp_conflict(void) { /* nothing */ }

static bool run_dhcp(uint32_t timeout_ms)
{
	DHCP_init(6, dhcp_buffer);            /* socket 6 reserved for DHCP */
	reg_dhcp_cbfunc(dhcp_assign, dhcp_update, dhcp_conflict);

	uint32_t start = to_ms_since_boot(get_absolute_time());
	uint32_t last_tick = start;
	dhcp_got_ip = false;
	while (!dhcp_got_ip) {
		uint8_t st = DHCP_run();
		if (st == DHCP_IP_ASSIGN || st == DHCP_IP_CHANGED || st == DHCP_IP_LEASED) {
			return true;
		}
		if (st == DHCP_FAILED || st == DHCP_STOPPED) {
			DHCP_stop();
			return false;
		}
		uint32_t now = to_ms_since_boot(get_absolute_time());
		if (now - last_tick >= 1000) {
			DHCP_time_handler();
			last_tick = now;
		}
		if (now - start >= timeout_ms) {
			DHCP_stop();
			return false;
		}
		sleep_ms(10);
	}
	return true;
}

/* ============================================================
 * OTA download driver
 * ============================================================ */

static bool perform_ota(uint32_t expected_size, uint32_t expected_crc, uint32_t expected_vtor)
{
	printf("OTA: expected size=%u crc=0x%08x vtor=0x%08x\n",
	       (unsigned)expected_size, (unsigned)expected_crc, (unsigned)expected_vtor);

	if (expected_size == 0 || expected_size > FLASH_ADDR_MAX - WRITE_ADDR_MIN) {
		printf("OTA: bad expected_size\n");
		return false;
	}
	if (expected_vtor != WRITE_ADDR_MIN) {
		printf("OTA: vtor 0x%08x doesn't match WRITE_ADDR_MIN 0x%08x\n",
		       (unsigned)expected_vtor, (unsigned)WRITE_ADDR_MIN);
		return false;
	}

	uint8_t mac[6];
	if (!bootloader_w5500_init(mac)) {
		printf("OTA: W5500 init failed\n");
		return false;
	}
	if (!run_dhcp(30000)) {
		printf("OTA: DHCP failed\n");
		return false;
	}
	printf("OTA: IP=%d.%d.%d.%d gw=%d.%d.%d.%d\n",
	       local_ip[0], local_ip[1], local_ip[2], local_ip[3],
	       gateway_ip[0], gateway_ip[1], gateway_ip[2], gateway_ip[3]);

	/* Build "x.y.z.w" host string from gateway. */
	char host[16];
	snprintf(host, sizeof(host), "%d.%d.%d.%d",
	         gateway_ip[0], gateway_ip[1], gateway_ip[2], gateway_ip[3]);

	stage_ctx_t st;
	memset(&st, 0, sizeof(st));
	st.write_offset = WRITE_ADDR_MIN - XIP_BASE;

	http_w5500_state_t tcp_state;
	http_transport_t   transport;
	http_transport_w5500_init(&transport, &tcp_state, /*socket=*/0, /*local_port=*/49152);

	http_get_request_t req = {
		.host      = host,
		.port      = 80,
		.path      = "/api/firmware/rp2040/saint_node.bin",
		.transport = &transport,
		.body_cb   = stage_body_cb,
		.user      = &st,
	};
	http_get_result_t res;
	http_status_t s = http_get(&req, &res);

	if (s != HTTP_OK) {
		printf("OTA: http_get failed s=%d code=%d recv=%u/%u\n",
		       (int)s, res.http_code,
		       (unsigned)res.bytes_received, (unsigned)res.content_length);
		return false;
	}

	/* Flush any trailing partial page so the body is fully committed. */
	flush_page(&st);

	if (st.bytes_written != expected_size) {
		printf("OTA: size mismatch: got %u expected %u\n",
		       (unsigned)st.bytes_written, (unsigned)expected_size);
		return false;
	}
	if (st.crc32_running != expected_crc) {
		printf("OTA: CRC mismatch: got 0x%08x expected 0x%08x\n",
		       (unsigned)st.crc32_running, (unsigned)expected_crc);
		return false;
	}

	/* Image body is in flash. Verify by recomputing CRC over the
	 * flash itself (catches any silent program failure). */
	uint32_t fcrc = crc32_update(0, (const uint8_t*)expected_vtor, expected_size);
	if (fcrc != expected_crc) {
		printf("OTA: post-write flash CRC mismatch (0x%08x)\n", (unsigned)fcrc);
		return false;
	}

	/* All good — write the image header LAST so partial-failure leaves
	 * the old header invalid (CRC of old size over new data won't match),
	 * which keeps us in the bootloader for retry. */
	write_image_header(expected_vtor, expected_size, expected_crc);
	printf("OTA: image header written; rebooting into new app\n");
	return true;
}

/* ============================================================
 * Entry
 * ============================================================ */

static bool ota_requested_by_app(uint32_t* out_size, uint32_t* out_crc, uint32_t* out_vtor)
{
	if (watchdog_hw->scratch[5] != PICOWOTA_BOOTLOADER_ENTRY_MAGIC) return false;
	if (watchdog_hw->scratch[6] != ~PICOWOTA_BOOTLOADER_ENTRY_MAGIC) return false;
	*out_size = watchdog_hw->scratch[0];
	*out_crc  = watchdog_hw->scratch[1];
	*out_vtor = watchdog_hw->scratch[2];
	return true;
}

int main(void)
{
	/* stdio early so panic prints are visible. */
	stdio_init_all();
	sleep_ms(10);

	critical_section_init(&flash_cs);

	uint32_t exp_size, exp_crc, exp_vtor;
	bool ota_now = ota_requested_by_app(&exp_size, &exp_crc, &exp_vtor);

	/* Fast path: app present and not asked to stay in bootloader → run it. */
	if (!ota_now && image_header_ok(&app_image_header)) {
		uint32_t vtor = app_image_header.vtor;
		bl_disable_interrupts();
		reset_peripherals();
		jump_to_vtor(vtor);
		/* not reached */
	}

	printf("\n==== SAINT.OS RP2040 OTA Bootloader ====\n");
	if (ota_now) {
		printf("OTA mode: requested by app\n");
	} else {
		printf("OTA mode: no valid app image, awaiting download\n");
		/* No metadata from the app → we need defaults. The first-time
		 * install case: a fresh board has no app and no scratch values.
		 * We have nothing to verify against, so refuse to proceed and
		 * stay in OTA mode forever. Operator must reflash via BOOTSEL
		 * (which is how the bootloader itself gets onto the board). */
		while (1) {
			tight_loop_contents();
		}
	}

	bool ok = perform_ota(exp_size, exp_crc, exp_vtor);
	clear_ota_magic();

	if (ok) {
		/* Reboot via watchdog — the new image header will be picked
		 * up by image_header_ok() and we'll jump to the new app. */
		watchdog_reboot(0, 0, 0);
	} else {
		/* Stay in OTA mode for retry. We need scratch magic to remain
		 * set so the next boot still enters OTA. Re-set magic but
		 * leave the size/crc/vtor in place so the operator can hit
		 * retry from the server. */
		watchdog_hw->scratch[5] = PICOWOTA_BOOTLOADER_ENTRY_MAGIC;
		watchdog_hw->scratch[6] = ~PICOWOTA_BOOTLOADER_ENTRY_MAGIC;
		watchdog_reboot(0, 0, 0);
	}
	while (1) { tight_loop_contents(); }
}
