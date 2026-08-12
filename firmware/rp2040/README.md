# SAINT.OS RP2040 Node Firmware

micro-ROS node firmware for the **Adafruit Feather RP2040 + Ethernet
FeatherWing (W5500)** — the recommended SAINT.OS node. Written in C against the
Pico SDK; talks to the server via **micro-ROS over UDP/Ethernet**.

## Overview

The RP2040 is a peripheral-first node: it drives RoboClaw motor controllers,
SyRen drivers, servos/PWM, NeoPixels, BMS and IMU sensors, each addressed as
`(peripheral, channel)`. It announces itself to the server, is *adopted* from
the web UI, and receives its peripheral configuration and live setpoints over
ROS 2 topics. The same source builds a **hardware** `.uf2` and a **Renode
simulation** target.

## Install

The firmware ships **inside the server dist** — you normally never build it.

- **Over the air (recommended).** Once a node is adopted in the web UI, push the
  RP2040 firmware from the server's firmware store. See the
  [operator guide, §3](../../server/docs/SERVER_GUIDE.md#3-adopt-nodes-and-apply-ota-updates).
- **First bring-up (UF2).** A blank board can't OTA yet — flash the bundled
  `.uf2` once by BOOTSEL drag-and-drop.

Flashing methods (UF2, picotool, OpenOCD), network/pin configuration, and
hardware assembly are all in [`docs/INSTALL.md`](docs/INSTALL.md).

## Building from source

```bash
cd firmware/rp2040
./build.sh hw     # hardware .uf2
./build.sh sim    # Renode simulation build
```

Toolchain prerequisites and the full build walkthrough are in
[`docs/INSTALL.md`](docs/INSTALL.md); the repo-wide build index is
[`BUILD.md`](../../docs/BUILD.md).

> **Before you "clean up" any driver:** read
> [`docs/MAINTENANCE_NOTES.md`](docs/MAINTENANCE_NOTES.md). Several
> non-obvious design choices (RoboClaw CRC, PIO/GPIO muxing, duty keepalive)
> are load-bearing and each guards against a known, hard-won regression.

## Documentation

- [`docs/INSTALL.md`](docs/INSTALL.md) — build, flash, network/pin config, server setup
- [`docs/SIMULATION.md`](docs/SIMULATION.md) — Renode simulation build & run
- [`docs/MAINTENANCE_NOTES.md`](docs/MAINTENANCE_NOTES.md) — do-not-undo notes (read before refactoring drivers)

## Troubleshooting

See [`docs/INSTALL.md`](docs/INSTALL.md) (Troubleshooting) and the LED status
table there. For RoboClaw link issues specifically, the "known external causes"
list at the end of [`docs/MAINTENANCE_NOTES.md`](docs/MAINTENANCE_NOTES.md)
(BEC brown-out, S3 latching E-stop, Motion Studio modes) is the fastest triage.
