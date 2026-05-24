# SAINT.OS RP2040 OTA Bootloader

A small bootloader that lives at the start of the RP2040 flash and supports
over-the-air firmware updates via HTTP over the W5500 Ethernet FeatherWing.
Forked from [picowota](https://github.com/usedbytes/picowota) by Brian
Starkey (BSD-3-Clause; original license preserved as `LICENSE.picowota`).

## What this gives us

- One-time BOOTSEL flash per board to install the bootloader, after which
  every app update is over ethernet.
- A/B-ish swap: the operator's new image is staged to flash, validated by
  CRC32, then swapped into the app slot in one shot. A power loss during
  the swap window is the only brick risk; everything before that is safe.
- Same HTTP protocol the Teensy 4.1 OTA path uses (via FlasherX), so the
  server speaks one wire format regardless of which MCU it's flashing.

## What we vendored from upstream picowota

| File | Status | Notes |
|------|--------|-------|
| `bootloader_shell.ld` | unchanged | linker script for the bootloader itself (lives at 0x10000000) |
| `standalone.ld` | unchanged | linker script for app binaries that link against the bootloader (start at 0x10004000) |
| `gen_imghdr.py` | unchanged | post-build tool that appends image header (size + CRC32) to the app .bin |
| `mkasm.py` | unchanged | helper that wraps the imghdr .S generation |
| `picowota_reboot/` | unchanged | tiny static lib the *app* links against to trigger OTA mode (`picowota_reboot_to_bootloader()`) |
| `main.c.upstream-reference` | reference only | the original cyw43+lwIP+tcp_comm bootloader main loop; we read this to understand the flash-swap mechanics, then write our own `main.c` |
| `LICENSE.picowota` | unchanged | upstream BSD-3-Clause license — preserved verbatim |

## What we are NOT vendoring

| Upstream file | Why dropped |
|--------------|-------------|
| `tcp_comm.c` / `tcp_comm.h` | Custom TCP protocol on port 4242 — replaced with plain HTTP GET so the server speaks one protocol for both RP2040 and Teensy. |
| `dhcpserver/` | Upstream's WiFi-AP-mode DHCP server. We connect to the Pi's dnsmasq as a DHCP client, so we don't need this. |
| `lwipopts.h` | Was tuned for cyw43+lwIP. We use the W5500 ioLibrary's hardware TCP sockets instead of an lwIP soft stack. |
| `CMakeLists.txt` | Upstream build pulls in cyw43 and lwIP — easier to write our own CMakeLists that brings in just what we want. |

## Architecture

```
0x10000000          ┌─────────────────────────┐
                    │  Bootloader (~16 KB)    │   bootloader_shell.ld
0x10004000          ├─────────────────────────┤
                    │  App slot               │   standalone.ld
                    │  (active)               │
                    │  ~1 MB                  │
0x10104000          ├─────────────────────────┤
                    │  Staging slot           │   downloaded firmware
                    │  (~1 MB)                │   lands here first
0x101FE000          ├─────────────────────────┤
                    │  OTA control (4 KB)     │   magic / size / CRC32 /
                    │                         │   "pending swap" flag
0x101FF000          ├─────────────────────────┤
                    │  Persistent storage     │   peripheral config
                    │  (4 KB) — UNCHANGED     │   (existing flash_storage)
0x10200000          └─────────────────────────┘
```

(Exact offsets TBD when the linker scripts are adapted.)

## Boot flow

1. Bootloader runs first on every reset.
2. Reads OTA control sector. If "pending swap" magic is set:
   - Validates staging CRC32 against the value recorded in the control sector.
   - If valid: erases app slot, copies staging → app slot, clears control sector, watchdog-reboots into the new app.
   - If invalid: clears control sector to abandon the bad image, then boots the existing app slot.
3. If the watchdog-scratch register matches the "enter OTA mode" magic (set by `picowota_reboot_to_bootloader()` from the app), enters OTA download mode:
   - Brings up W5500, DHCP client.
   - Performs HTTP GET to the URL the app handed off in scratch memory.
   - Streams response body into the staging slot.
   - On success, sets the "pending swap" magic + CRC, reboots.
4. Otherwise: validates app slot header, jumps to app.

## Status

Vendoring + design only — no build integration or working code yet. Next
commits will:
1. Replace `main.c.upstream-reference` with our W5500 + HTTP version.
2. Add a CMakeLists.txt that builds the bootloader as a standalone .uf2.
3. Adapt the linker scripts for our flash layout if we need slot sizes
   different from upstream's defaults.
4. Wire the app side: link `picowota_reboot` into the SAINT.OS firmware
   and add a `handle_firmware_update` path that calls into it.
5. Add the shared HTTP client (used by both this bootloader and the
   Teensy FlasherX integration).
