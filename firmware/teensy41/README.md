# SAINT.OS Teensy 4.1 Node Firmware

micro-ROS node firmware for the **Teensy 4.1**. Written in C++ on the Arduino
framework (PlatformIO under the hood); talks to the server via **micro-ROS over
UDP** on the Teensy's native Ethernet.

## Overview

The Teensy 4.1 is a peripheral-first node with the same model as the other node
types: it drives motors (RoboClaw), Pololu Maestro servo controllers (over
USBHost), servos/PWM, NeoPixels, and sensors, each addressed as
`(peripheral, channel)`. It announces itself to the server, is *adopted* from
the web UI, and receives peripheral configuration and live setpoints over ROS 2
topics. The same source builds a **hardware** `.hex` and a **Renode simulation**
target.

## Install

The firmware ships inside the server dist — push it **over the air** once the
node is adopted, or flash `firmware.hex` with the Teensy Loader for a blank
board's first bring-up. Steps: [`docs/INSTALL.md`](docs/INSTALL.md).

## Building from source

```bash
cd firmware/teensy41
./build.sh hw     # or: sim
```

Full build + flash detail: [`docs/INSTALL.md`](docs/INSTALL.md). Repo-wide build
index: [`../../docs/BUILD.md`](../../docs/BUILD.md).

## Documentation

- [`docs/INSTALL.md`](docs/INSTALL.md) — flash & build the Teensy firmware
- [`../../server/docs/SERVER_GUIDE.md`](../../server/docs/SERVER_GUIDE.md) — flashing, adoption, OTA, peripherals
- [`../../docs/MAESTRO_BRINGUP.md`](../../docs/MAESTRO_BRINGUP.md) — Pololu Maestro bring-up (driven from this node over USBHost)
