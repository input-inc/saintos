/**
 * SAINT.OS Firmware - Teensy 4.1 Maestro transport
 *
 * Provides three transports satisfying the shared
 * maestro_transport_ops contract:
 *   - usb_cdc:    Pololu Compact Protocol byte stream over USB host
 *                 CDC ACM (USBHost_t36 + USBSerial_BigBuffer). Needs
 *                 the Maestro's Serial Mode set to "USB Dual Port"
 *                 (or "USB Chained") in MCC — otherwise bytes go out
 *                 the TTL pins instead of into the Maestro's command
 *                 interpreter. Cannot do EEPROM (GET_PARAMETER) ops.
 *   - usb_vendor: EP0 control transfers (Set Target via 0x85, GET/SET
 *                 PARAMETER, GET_VARIABLES) through a custom
 *                 USBHost_t36 USBDriver (MaestroVendorDriver below).
 *                 Works in any Serial Mode and is the only transport
 *                 that can read EEPROM back (GET_PARAMETER). Decodes
 *                 the shared driver's Compact Protocol byte stream into
 *                 vendor requests — mirror of the Pi-side
 *                 _PyUsbVendorBackend.
 *   - uart:       Pololu Compact Protocol over the Teensy's hardware
 *                 serial pins (Serial1..Serial8). Use when the Maestro
 *                 is wired to a TTL UART instead of plugged into the
 *                 USB host port. The Maestro's TTL RX always accepts
 *                 Compact Protocol regardless of its USB Serial Mode.
 */

#include <Arduino.h>
#include <string.h>   /* memcpy in the vendor ctrl path */
#include "platform.h"

#ifndef SIMULATION
#include <USBHost_t36.h>
#endif

extern "C" {
#include "flash_types.h"
#include "maestro_transport.h"
#include "maestro_driver.h"  /* MAESTRO_DEFAULT_UART_BAUD */
}

/* ── USB host objects ────────────────────────────────────────────── */

#ifndef SIMULATION
static USBHost myusb;
static USBHub  hub1(myusb);
#endif

/* Tracks myusb.begin() — shared by the CDC and vendor transports
 * since there's exactly one USBHost stack on the board. */
static bool g_initialized = false;

#ifndef SIMULATION

/* ── Pololu Maestro USB identity & vendor request codes ──────────────
 *
 * VID/PID and request codes from Pololu's pololu-usb-sdk
 * (Maestro/protocol.h, Maestro/Usc/Usc.cs). The four application PIDs
 * map to the four Maestro models; we claim any of them so one firmware
 * image works across the family, though SaintOS ships on the Mini
 * Maestro 24 (PID 0x008C). The bootloader PIDs (0x0088/8D/8E/8F) are
 * deliberately NOT claimed — a Maestro in bootloader mode isn't a
 * servo controller. */
#define MAESTRO_USB_VID            0x1FFBu
#define MAESTRO_PID_MICRO6         0x0089u
#define MAESTRO_PID_MINI12         0x008Au
#define MAESTRO_PID_MINI18         0x008Bu
#define MAESTRO_PID_MINI24         0x008Cu

/* Vendor (EP0) request codes — Pololu protocol.h `enum uscRequest`. */
#define MAESTRO_REQ_GET_PARAMETER      0x81u
#define MAESTRO_REQ_SET_PARAMETER      0x82u
#define MAESTRO_REQ_GET_VARIABLES      0x83u
#define MAESTRO_REQ_SET_SERVO_VARIABLE 0x84u
#define MAESTRO_REQ_SET_TARGET         0x85u
#define MAESTRO_REQ_REINITIALIZE       0x90u
#define MAESTRO_REQ_SET_SCRIPT_DONE    0xA2u  /* wValue: 0 go, 1 stop */

/* Compact Protocol opcodes the shared driver core emits via write().
 * The vendor transport decodes these and re-issues them as EP0 vendor
 * requests so the driver core (shared/src/maestro_driver.c) stays
 * transport-agnostic — it has no idea it's talking to EP0 rather than
 * a CDC byte stream. */
#define MAESTRO_COMPACT_SET_TARGET   0x84u
#define MAESTRO_COMPACT_SET_SPEED    0x87u
#define MAESTRO_COMPACT_SET_ACCEL    0x89u
#define MAESTRO_COMPACT_GET_POSITION 0x90u
#define MAESTRO_COMPACT_GET_ERRORS   0xA1u
#define MAESTRO_COMPACT_GO_HOME      0xA2u
#define MAESTRO_COMPACT_STOP_SCRIPT  0xA4u
#define MAESTRO_COMPACT_GET_MOVING_STATE 0xA6u

/* ── Vendor (EP0 control transfer) USB host driver ───────────────────
 *
 * A custom USBHost_t36 driver that claims the Maestro at the *device*
 * level (claim type 0) and drives it purely through EP0 vendor control
 * transfers. This is the difference vs the CDC transport: vendor
 * requests are honored in ANY Maestro Serial Mode (the CDC byte stream
 * is interpreted only in "USB Dual Port"/"USB Chained"), and only
 * vendor GET_PARAMETER can read EEPROM back.
 *
 * Driver-claim arbitration: USBSerial also claims CDC ACM at type 0
 * (serial.cpp:92), and claim_drivers (enumeration.cpp:327) offers
 * type 0 to drivers in registration order and binds the FIRST that
 * returns true. So this object is declared *before* maestroSerial
 * below, and its claim() returns true only when g_vendor_mode is set.
 * Vendor mode → we claim first, CDC never sees the interface. CDC mode
 * → we decline at type 0, enumeration falls through to maestroSerial's
 * interface claim exactly as before. One image, both transports, no
 * library patch.
 *
 * Synchronous control transfers: queue_Control_Transfer() is async —
 * completion fires control() from the USB ISR. We expose a blocking
 * ctrl() that queues one transfer and spin-waits on a volatile flag
 * with a timeout, which is fine because (a) SET_* requests carry their
 * payload in wValue/wIndex with a zero-length data stage so they
 * complete in microseconds, and (b) GET_PARAMETER is only issued from
 * the rare channel-edit readback path, not the hot control loop. */

/* True when the operator-selected transport is usb_vendor. Set in
 * vendor_open(), cleared in usb_open() (CDC). claim() consults it so
 * exactly one of the vendor driver / maestroSerial binds the device.
 * Switching transports takes effect on the next Maestro enumeration
 * (re-plug or power cycle) — a device already bound stays bound.
 * Declared before the class so the inline claim() can see it. */
static bool g_vendor_mode = false;

class MaestroVendorDriver : public USBDriver {
public:
    explicit MaestroVendorDriver(USBHost& host) { (void)host; init(); }

    bool connected() const {
        return *(Device_t* volatile*)&device != nullptr;
    }

    /* Blocking vendor control transfer. Mirrors libusb/pyusb semantics:
     * bmRequestType 0xC0 = IN, 0x40 = OUT. Returns bytes transferred,
     * or < 0 on timeout / not-connected / queue failure. */
    int ctrl(uint8_t bmRequestType, uint8_t bRequest,
             uint16_t wValue, uint16_t wIndex,
             uint8_t* buf, uint16_t wLength, uint32_t timeout_ms) {
        Device_t* dev = *(Device_t* volatile*)&device;
        if (!dev) return -1;
        if (wLength > sizeof(m_buf)) return -1;

        const bool in = (bmRequestType & 0x80u) != 0;
        if (!in && wLength) memcpy(m_buf, buf, wLength);

        /* Invalidate our DMA buffer before an IN transfer so a dirty
         * cache line can't clobber what the controller writes back.
         * init_qTD (ehci.cpp) does no cache maintenance on the data
         * buffer — that's the caller's job on the cached M7. */
        if (in && wLength) arm_dcache_delete(m_buf, sizeof(m_buf));

        mk_setup(m_setup, bmRequestType, bRequest, wValue, wIndex, wLength);
        m_done   = false;
        m_result = -1;
        m_expect = wLength;
        if (!queue_Control_Transfer(dev, &m_setup,
                                    wLength ? m_buf : nullptr, this)) {
            return -1;
        }

        uint32_t start = millis();
        while (!m_done && (millis() - start) < timeout_ms) {
            /* completion arrives via control() from the USB ISR */
        }
        if (!m_done) return -1;                 /* timed out / STALL */

        if (in && m_result > 0 && buf) {
            arm_dcache_delete(m_buf, sizeof(m_buf));
            memcpy(buf, m_buf, (size_t)m_result);
        }
        return m_result;
    }

protected:
    bool claim(Device_t* dev, int type,
               const uint8_t* descriptors, uint32_t len) override {
        (void)descriptors; (void)len;
        if (type != 0) return false;            /* device-level only */
        if (!g_vendor_mode) return false;        /* let CDC have it */
        if (dev->idVendor != MAESTRO_USB_VID) return false;
        const uint16_t pid = dev->idProduct;
        if (pid != MAESTRO_PID_MICRO6 && pid != MAESTRO_PID_MINI12 &&
            pid != MAESTRO_PID_MINI18 && pid != MAESTRO_PID_MINI24) {
            return false;
        }
        device = dev;                            /* bind; operator bool() true */
        return true;
    }

    void control(const Transfer_t* transfer) override {
        /* Our queued transfer completed. We can't cheaply read the
         * actual transferred count out of the status qTD here, so
         * report the requested length on success — adequate because
         * the Maestro never short-replies a GET_PARAMETER. */
        (void)transfer;
        m_result = (int)m_expect;
        m_done   = true;
    }

    void disconnect() override { device = nullptr; }

private:
    void init() {
        contribute_Transfers(m_transfers,
                             sizeof(m_transfers) / sizeof(m_transfers[0]));
        driver_ready_for_device(this);
    }

    setup_t           m_setup;
    volatile bool     m_done   = false;
    volatile int      m_result = -1;
    uint16_t          m_expect = 0;
    /* 64-byte, 32-aligned: large enough for any Maestro vendor reply we
     * issue (GET_PARAMETER ≤ 2 bytes, GET_VARIABLES probe ≤ 2 bytes),
     * aligned + sized for whole-cache-line dcache ops. */
    uint8_t           m_buf[64] __attribute__((aligned(32)));
    Transfer_t        m_transfers[8] __attribute__((aligned(32)));
};

/* Declared BEFORE maestroSerial so it is offered the device first at
 * claim type 0 (see arbitration note above). */
static MaestroVendorDriver maestroVendor(myusb);

/* USBSerial_BigBuffer(host, min_rxtx=1) — `1` is the MIN_RXTX hint
 * (lets it also handle devices that report small max packet sizes,
 * which the Maestro does on its CDC ACM endpoints). It is NOT a port
 * index; the driver attaches to the first CDC ACM interface it finds.
 * On the Mini Maestro in USB Dual Port mode that's the Command Port
 * (the TTL Port comes second), so commands like Set Target reach the
 * Maestro's command interpreter. */
static USBSerial_BigBuffer maestroSerial(myusb, 1);

/* Response stash for the emulated byte-stream read() path. The shared
 * driver's query commands (Get Errors 0xA1, Get Position 0x90) are
 * "write opcode, then read N bytes"; on the vendor transport write()
 * fills this synchronously and read() drains it. */
static uint8_t g_resp[2];
static size_t  g_resp_len = 0;
static size_t  g_resp_pos = 0;

#endif  /* !SIMULATION */

/* Tracks whether maestroSerial.begin() has been called since the
 * Maestro last enumerated. CDC ACM devices typically silently drop
 * incoming bytes until the host sends SET_CONTROL_LINE_STATE with
 * DTR=1 — and on USBHost_t36 the *only* place that message gets
 * queued is inside USBSerial.begin(). We have to call it AFTER the
 * device enumerates (so `(bool)maestroSerial` is already true),
 * which means we can't do it inside usb_open at boot. Track the
 * transition here and call begin() once per connect. Symptom of
 * missing this: green LED blinks on USB activity, but the Maestro
 * never processes commands — bytes hit the CDC RX endpoint and
 * sit there because DTR is still 0. */
static bool g_cdc_began = false;

#ifndef SIMULATION
/* Bring the single USBHost stack up once. Shared by both USB
 * transports — whichever opens first wins; the second is a no-op. */
static void ensure_host_begun(void)
{
    if (!g_initialized) {
        myusb.begin();
        g_initialized = true;
    }
}
#endif

static bool usb_open(const flash_storage_data_t* storage)
{
    (void)storage;
#ifndef SIMULATION
    /* CDC mode: make the vendor driver decline the device at claim
     * time so maestroSerial binds the CDC interface as before. */
    g_vendor_mode = false;
    ensure_host_begun();
    return true;
#else
    return false;
#endif
}

static void usb_update(void)
{
#ifndef SIMULATION
    if (!g_initialized) return;
    myusb.Task();

    /* Connect / disconnect edge handling for the CDC line-state
     * setup. On enumeration we have to call maestroSerial.begin()
     * exactly once so USBHost_t36 queues SET_LINE_CODING +
     * SET_CONTROL_LINE_STATE (DTR=1, RTS=1) — without those the
     * Maestro's command interpreter never wakes up. On disconnect
     * we clear the flag so the next reconnect re-runs begin(). */
    bool connected_now = (bool)maestroSerial;
    if (connected_now && !g_cdc_began) {
        /* MAESTRO_DEFAULT_UART_BAUD matches the Maestro factory
         * setting; for USB CDC the baud value is informational
         * (no physical UART) but USBHost_t36 still needs a value. */
        maestroSerial.begin(MAESTRO_DEFAULT_UART_BAUD);
        g_cdc_began = true;
    } else if (!connected_now && g_cdc_began) {
        g_cdc_began = false;
    }
#endif
}

static bool usb_is_connected(void)
{
#ifndef SIMULATION
    /* Both the USB layer must be up AND we must have run begin() to
     * assert DTR. Without begin(), the device is enumerated but
     * the Maestro silently ignores incoming bytes; reporting
     * is_connected=true in that window would let the shared
     * driver's connect-hook fire prematurely (stop-script + apply-
     * home would all silently fail). */
    return (bool)maestroSerial && g_cdc_began;
#else
    return false;
#endif
}

static bool usb_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!usb_is_connected()) return false;
    return maestroSerial.write(data, len) == len;
#else
    (void)data; (void)len;
    return false;
#endif
}

static size_t usb_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
#ifndef SIMULATION
    if (!usb_is_connected()) return 0;
    size_t count = 0;
    uint32_t start = millis();
    while (count < len && (millis() - start) < timeout_ms) {
        if (maestroSerial.available()) {
            data[count++] = maestroSerial.read();
        }
    }
    return count;
#else
    (void)data; (void)len; (void)timeout_ms;
    return 0;
#endif
}

static bool usb_supports_hotplug(void)
{
    return true;
}

static const maestro_transport_ops_t usb_cdc_ops = {
    .name             = "usb_cdc",
    .open             = usb_open,
    .update           = usb_update,
    .is_connected     = usb_is_connected,
    .write            = usb_write,
    .read             = usb_read,
    .supports_hotplug = usb_supports_hotplug,
    .ctrl_xfer        = NULL,   /* CDC transport can't issue EP0 control transfers */
};

extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_cdc(void)
{
    return &usb_cdc_ops;
}

/* ── UART transport (HardwareSerial1..Serial8) ──────────────────── */

#ifndef SIMULATION
/* Map serial_port (1..8) → HardwareSerial& via runtime lookup.
 * Returning a pointer rather than a reference so the open() path can
 * cleanly signal "invalid serial_port" by leaving g_uart NULL. */
static HardwareSerial* hw_serial_for(uint8_t serial_port)
{
    switch (serial_port) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        case 8: return &Serial8;
        default: return nullptr;
    }
}

static HardwareSerial* g_uart = nullptr;
static bool g_uart_initialized = false;
#endif  // !SIMULATION

static bool uart_open(const flash_storage_data_t* storage)
{
#ifndef SIMULATION
    /* serial_port: 1..8 (Teensy hardware serial index). 0 is a
     * sentinel for "unset" — bail rather than guess. */
    uint8_t port = 0;
    if (storage) port = storage->maestro_config.serial_port;
    if (port == 0) {
        return false;
    }
    HardwareSerial* hs = hw_serial_for(port);
    if (!hs) {
        return false;   /* invalid port number */
    }
    /* MAESTRO_DEFAULT_UART_BAUD matches both the Maestro factory
     * default and what MCC sets when configuring a fresh device.
     * Compact-protocol auto-detect mode (the Maestro's default
     * factory setting) locks onto the first 0xAA byte we send and
     * works at any common baud, so 9600 here is fine even if the
     * operator changed the Maestro's fixed baud to something else
     * later. */
    if (!g_uart_initialized || g_uart != hs) {
        if (g_uart && g_uart_initialized && g_uart != hs) {
            g_uart->end();
        }
        hs->begin(MAESTRO_DEFAULT_UART_BAUD);
        g_uart = hs;
        g_uart_initialized = true;
    }
    return true;
#else
    (void)storage;
    return false;
#endif
}

static void uart_update(void) { /* HardwareSerial doesn't need pumping. */ }

static bool uart_is_connected(void)
{
#ifndef SIMULATION
    /* HardwareSerial has no link-down signal — once begin() has been
     * called and the pins are driving, we consider the link "up".
     * The Maestro's actual presence is observable only via the data
     * we get back from Get-Position queries; the shared driver core
     * already detects that path via maestro_get_position(). */
    return g_uart_initialized && g_uart != nullptr;
#else
    return false;
#endif
}

static bool uart_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!uart_is_connected()) return false;
    /* HardwareSerial::write returns total bytes written. On Teensy
     * the TX FIFO is large (~512 bytes) but for safety we check the
     * full count rather than fire-and-forget. */
    return g_uart->write(data, len) == len;
#else
    (void)data; (void)len;
    return false;
#endif
}

static size_t uart_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
#ifndef SIMULATION
    if (!uart_is_connected()) return 0;
    size_t count = 0;
    uint32_t start = millis();
    while (count < len && (millis() - start) < timeout_ms) {
        if (g_uart->available()) {
            data[count++] = g_uart->read();
        }
    }
    return count;
#else
    (void)data; (void)len; (void)timeout_ms;
    return 0;
#endif
}

static bool uart_supports_hotplug(void) { return false; }

static const maestro_transport_ops_t uart_ops = {
    .name             = "uart",
    .open             = uart_open,
    .update           = uart_update,
    .is_connected     = uart_is_connected,
    .write            = uart_write,
    .read             = uart_read,
    .supports_hotplug = uart_supports_hotplug,
    .ctrl_xfer        = NULL,   /* UART has no EP0 — never */
};

/* ── USB vendor (EP0 control transfer) transport ─────────────────────
 *
 * Drives the Maestro through Pololu vendor requests on endpoint 0 via
 * the MaestroVendorDriver above. Works in any Maestro Serial Mode and
 * is the only transport that can read EEPROM back (ctrl_xfer →
 * GET_PARAMETER), which is what maestro_read_channel_config_from_device
 * needs. The write/read ops translate the shared driver's Compact
 * Protocol byte stream into vendor requests — mirror of the Pi-side
 * _PyUsbVendorBackend in
 * firmware/raspberrypi/saint_node/peripherals/maestro.py. */

#ifndef SIMULATION

static bool vendor_open(const flash_storage_data_t* storage)
{
    (void)storage;
    /* Claim the device for vendor (EP0) use rather than CDC. Takes
     * effect on the next enumeration — see g_vendor_mode note. */
    g_vendor_mode = true;
    ensure_host_begun();
    return true;
}

static void vendor_update(void)
{
    if (!g_initialized) return;
    myusb.Task();
}

static bool vendor_is_connected(void)
{
    return maestroVendor.connected();
}

/* Decode one Compact Protocol command and issue the equivalent vendor
 * request. Returns the number of bytes consumed from `p`, or 0 if the
 * buffer doesn't yet hold a full command (caller should wait for more).
 * Query commands (Get Errors / Get Position) stash their reply for the
 * subsequent read(). */
static size_t vendor_dispatch(const uint8_t* p, size_t avail)
{
    switch (p[0]) {
        case MAESTRO_COMPACT_SET_TARGET: {
            if (avail < 4) return 0;
            uint16_t target = (uint16_t)(p[2] | (p[3] << 7));
            maestroVendor.ctrl(0x40, MAESTRO_REQ_SET_TARGET, target, p[1], nullptr, 0, 50);
            return 4;
        }
        case MAESTRO_COMPACT_SET_SPEED: {
            if (avail < 4) return 0;
            uint16_t speed = (uint16_t)(p[2] | (p[3] << 7));
            maestroVendor.ctrl(0x40, MAESTRO_REQ_SET_SERVO_VARIABLE, speed, p[1], nullptr, 0, 50);
            return 4;
        }
        case MAESTRO_COMPACT_SET_ACCEL: {
            if (avail < 4) return 0;
            uint16_t accel = (uint16_t)(p[2] | (p[3] << 7));
            /* SET_SERVO_VARIABLE distinguishes accel from speed by the
             * 0x80 bit on the channel index (Pololu protocol.h). */
            maestroVendor.ctrl(0x40, MAESTRO_REQ_SET_SERVO_VARIABLE, accel,
                               (uint16_t)(p[1] | 0x80), nullptr, 0, 50);
            return 4;
        }
        case MAESTRO_COMPACT_STOP_SCRIPT: {
            /* Compact 0xA4 → vendor SET_SCRIPT_DONE with wValue=1 (stop). */
            maestroVendor.ctrl(0x40, MAESTRO_REQ_SET_SCRIPT_DONE, 1, 0, nullptr, 0, 50);
            return 1;
        }
        case MAESTRO_COMPACT_GO_HOME: {
            /* No direct vendor "go home"; REINITIALIZE re-applies each
             * channel's EEPROM HOME, which is the estop intent. */
            maestroVendor.ctrl(0x40, MAESTRO_REQ_REINITIALIZE, 0, 0, nullptr, 0, 50);
            return 1;
        }
        case MAESTRO_COMPACT_GET_ERRORS: {
            /* Used by the shared driver's connect probe to decide
             * whether the command path is live. Over vendor the Serial
             * Mode is irrelevant, so we just confirm the device answers
             * EP0: read the first 2 bytes of GET_VARIABLES. On success
             * stash them (the low word is the error register); on
             * failure stash 0x0000 so the probe still reports "live"
             * rather than the CDC-only "wrong Serial Mode" warning. */
            uint8_t r[2] = {0, 0};
            int n = maestroVendor.ctrl(0xC0, MAESTRO_REQ_GET_VARIABLES, 0, 0, r, 2, 100);
            g_resp[0] = (n == 2) ? r[0] : 0;
            g_resp[1] = (n == 2) ? r[1] : 0;
            g_resp_len = 2;
            g_resp_pos = 0;
            return 1;
        }
        case MAESTRO_COMPACT_GET_POSITION: {
            if (avail < 2) return 0;
            /* Per-channel position over vendor requires parsing the
             * model-specific GET_VARIABLES servo-status array (the
             * layout differs between Micro and Mini Maestro). Not
             * implemented for v1 — stash 0 so maestro_get_position
             * returns 0 and drv_get_value falls back to the commanded
             * value. Tracked in docs/MAESTRO_BRINGUP.md. */
            g_resp[0] = 0;
            g_resp[1] = 0;
            g_resp_len = 2;
            g_resp_pos = 0;
            return 2;
        }
        case MAESTRO_COMPACT_GET_MOVING_STATE: {
            /* 0xA6 — used by the shared driver's status poll for the
             * Live Readings card's "Moving" indicator. Like
             * GET_POSITION, computing this over vendor needs the
             * model-specific GET_VARIABLES servo-status walk to
             * see whether any channel's actual ≠ target. Not
             * implemented v1 — stash a 1-byte 0 so the shared
             * driver's maestro_get_moving_state() returns 0
             * ("not moving"), and the UI's connected predicate
             * (errors-only) isn't fooled by a 0xFF sentinel into
             * thinking we're offline. */
            g_resp[0] = 0;
            g_resp_len = 1;
            g_resp_pos = 0;
            return 1;
        }
        default:
            return 1;   /* unknown opcode — skip one byte to resync */
    }
}

static bool vendor_write(const uint8_t* data, size_t len)
{
    if (!vendor_is_connected()) return false;
    size_t i = 0;
    while (i < len) {
        size_t consumed = vendor_dispatch(data + i, len - i);
        if (consumed == 0) break;   /* partial command — drop remainder */
        i += consumed;
    }
    return true;
}

static size_t vendor_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;   /* replies are stashed synchronously by write() */
    size_t n = 0;
    while (n < len && g_resp_pos < g_resp_len) {
        data[n++] = g_resp[g_resp_pos++];
    }
    return n;
}

static bool vendor_supports_hotplug(void) { return true; }

/* EEPROM readback path used by maestro_read_channel_config_from_device.
 * Direct passthrough to the blocking vendor control transfer. */
static int vendor_ctrl_xfer(uint8_t bmRequestType, uint8_t bRequest,
                            uint16_t wValue, uint16_t wIndex,
                            uint8_t* buf, uint16_t wLength,
                            uint32_t timeout_ms)
{
    if (!vendor_is_connected()) return -1;
    return maestroVendor.ctrl(bmRequestType, bRequest, wValue, wIndex,
                              buf, wLength, timeout_ms);
}

static const maestro_transport_ops_t usb_vendor_ops = {
    .name             = "usb_vendor",
    .open             = vendor_open,
    .update           = vendor_update,
    .is_connected     = vendor_is_connected,
    .write            = vendor_write,
    .read             = vendor_read,
    .supports_hotplug = vendor_supports_hotplug,
    .ctrl_xfer        = vendor_ctrl_xfer,
};

extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void)
{
    return &usb_vendor_ops;
}

#else  /* SIMULATION */

extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void)
{
    return NULL;   /* no USB host stack under Renode */
}

#endif

/* Legacy alias — main.c on the Teensy currently references
 * maestro_get_transport_usb_host(); keep it forwarding to usb_cdc so
 * existing code paths compile unchanged. */
extern "C" const maestro_transport_ops_t* maestro_get_transport_usb_host(void)
{
    return maestro_get_transport_usb_cdc();
}

extern "C" const maestro_transport_ops_t* maestro_get_transport_uart(void)
{
    return &uart_ops;
}
