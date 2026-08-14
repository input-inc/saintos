# SAINT.OS

**System for Articulated Intelligence and Navigation Tasks**

SAINT.OS is a ROS 2 control platform for multi-node animatronic and mobile
robots. A central server (Raspberry Pi or Linux compatible) coordinates a fleet of
microcontroller and single-board-computer **nodes** over ROS 2, exposes a
web UI and a WebSocket API, routes live input (gamepad, Unreal LiveLink,
animation timelines) to actuators through operator-authored **routing
sheets**, and ships over-the-air firmware updates to every node.

SAINT.OS is robot-agnostic. A one-file **robot manifest**
(`server/config/robots/<id>.yaml`) describes the platform you're building —
its name, homepage, and the node roles it uses. The reference robot,
**OpenSAINT**, is a replica of the robot from *Short Circuit*; drop in your
own manifest to run the same software on a different machine.

## Features

- **Distributed architecture** — one server, many peripheral nodes, all on
  ROS 2. Nodes announce themselves and are *adopted* from the web UI.
- **Peripheral-first control** — RoboClaw motor controllers, Pololu Maestro
  servo controllers, native RP2040/Teensy servo & PWM, NeoPixels, battery
  monitors, and audio, each addressed as `(peripheral, channel)`.
- **Routing sheets** — a visual node-graph binds inputs (gamepad axes,
  LiveLink, ROS topics, URDF joints) to peripheral channels with operators
  (mixing, clamping, curves) in between.
- **Animations & poses** — keyframe timelines with easing curves, plus a
  per-node soundboard.
- **Multiple input sources** — the Steam Deck controller app, the web UI,
  Unreal Engine LiveLink, and face tracking.
- **OTA updates** — the server stages and pushes firmware to RP2040, Teensy,
  and Raspberry Pi nodes, and can update itself.
- **Offline install** — the server dist bundles every runtime dependency, so
  a robot with no internet installs and updates cleanly.

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                      SAINT.OS Server (Raspberry Pi)             │
│   Web UI · WebSocket API · LiveLink receiver                    │
│   Routing evaluator · Animation player · OTA firmware store     │
└────────────────────────────────────────────────────────────────┘
          │ ROS 2 (rclpy)              │ micro-ROS over UDP / Ethernet
   ┌──────┴───────┐            ┌───────┴────────┬────────────────┐
   ▼              ▼            ▼                ▼                ▼
┌────────┐   ┌────────┐   ┌────────┐      ┌────────┐      ┌────────┐
│  Pi    │   │  Pi    │   │ RP2040 │      │ Teensy │      │ RP2040 │
│ node   │   │ node   │   │ (W5500)│      │  4.1   │      │ (W5500)│
│(Python)│   │(Python)│   │ node   │      │  node  │      │ node   │
└────────┘   └────────┘   └────────┘      └────────┘      └────────┘
```

Microcontroller nodes talk to the server via **micro-ROS** over UDP
(Ethernet on the RP2040 Feather's W5500 FeatherWing); Raspberry Pi nodes run
a Python ROS 2 node directly.

## Repository layout

| Path | What it is |
|------|------------|
| `server/` | ROS 2 package `saint_os` — server node, WebSocket + web UI (`server/web`, Vue 3), routing evaluator, animation engine, OTA store. |
| `controller/` | Steam Deck / desktop controller app (Tauri 2 + Vue 3 frontend, Rust backend). |
| `firmware/` | Node firmware: `rp2040/` (C / Pico SDK / micro-ROS), `teensy41/` (C++ / PlatformIO), `raspberrypi/` (Python), `shared/` (platform-agnostic drivers), `simulation/` (Renode e2e). |
| `configs/` | Peripheral configuration presets (e.g. RoboClaw). |
| `scripts/` | Build & release tooling — notably `build-local-dist.sh`. |
| `packaging/` | `install.sh` and the systemd unit that run on the robot. |
| `docs/` | Architecture, hardware, and subsystem guides. |

## Getting started

SAINT.OS installs from a **prebuilt release** — you don't have to build
anything to stand up a robot. The server dist bundles the node firmware and
the controller app, so one download gets you everything. Three steps:
**install the server → adopt and flash nodes from it → install the
controller.** (To build from source instead, see [BUILD.md](docs/BUILD.md).)

### 1. Install the server (Raspberry Pi)

Download the latest server dist tarball —
`saint-os_<version>_arm64_kilted.tar.zst` — from the
[**Releases**](https://github.com/input-inc/saintos/releases) page, copy it
to the Pi, and install:

```bash
# On the Pi
tar --zstd -xf saint-os_<version>_arm64_kilted.tar.zst
cd saint-os_<version>_arm64_kilted
sudo ./install.sh            # installs deps offline, enables + starts the service
```

The server runs as the `saint-os` systemd service and serves the web UI:

```
http://opensaint.local/      # or http://<pi-ip>/
```

`install.sh --help` covers options (`--no-wifi`, `--no-start`, `--dry-run`).
For the full walkthrough — Pi OS image, networking, and verification — see
[INSTALL.md](docs/INSTALL.md) and the [operator guide](server/docs/SERVER_GUIDE.md).

### 2. Adopt nodes and flash firmware — from the server

Everything below happens in the server's web UI:

1. **Adopt nodes.** Powered-on nodes appear as *unadopted*. Adopt each one by
   giving it a **name** (its identity everywhere in the app), picking its
   **board**, and optionally a **role** from the active robot manifest.
2. **Flash firmware over the air.** The dist ships firmware for every node
   type into the server's firmware store — push it from the UI. For an
   RP2040's first bring-up, flash the bundled `.uf2` in BOOTSEL mode.
   Raspberry Pi nodes install the bundled `saint_firmware_raspberrypi` bundle.
3. **Add peripherals.** Configure each node's peripherals (motors, servos,
   Maestro channels, NeoPixels, audio) and sync them to the node.
4. **Build routing sheets.** Wire controller / LiveLink / animation inputs to
   peripheral channels.

The [operator guide](server/docs/SERVER_GUIDE.md) walks through all four in
detail.

### 3. Install the controller app

Get the controller onto a Steam Deck two ways — no toolchain either way:

- **OTA from the server** — the controller's **Settings** tab polls the
  SAINT.OS server and self-updates the running AppImage in place.
- **Bundled AppImage** — the server ships
  `saint_firmware_controller_*.AppImage`; drop it on the Deck and add it to
  Steam as a Non-Steam Game.

See [controller/README.md](controller/README.md) for first-time Steam Deck
setup.

### Run a different robot

SAINT.OS isn't tied to OpenSAINT — drop in a one-file robot manifest and select
it under **Settings → Robot**. See
[docs/INSTALL.md §6](docs/INSTALL.md#6-run-a-different-robot) for the manifest
format.

### Build from source

To build the server, firmware, or controller yourself — on a dev machine, for
development or to cut your own release — see [BUILD.md](docs/BUILD.md).

## Hardware support

### Server

| Platform | Status |
|----------|--------|
| Raspberry Pi 5 | Recommended |
| Raspberry Pi 4 | Supported |
| x86-64 Linux | Development |
| macOS (Apple Silicon) | Development |

### Nodes

| Platform | Status |
|----------|--------|
| Adafruit Feather RP2040 + Ethernet FeatherWing (W5500) | Recommended |
| Teensy 4.1 | Supported |
| Raspberry Pi 3 / 4 / 5 (Python node) | Supported |

### Controller app

| Platform | Status |
|----------|--------|
| Steam Deck (SteamOS) | Primary target |
| Linux / macOS / Windows | Supported (dev) |

### Software

- **ROS 2 Kilted** (bundled in the server dist; nodes use micro-ROS).
- **Python 3.11** on the server.

## Documentation

**Install & operate**

- [INSTALL.md](docs/INSTALL.md) — In-depth install guide: Pi OS image, download, install, first-run setup
- [server/docs/SERVER_GUIDE.md](server/docs/SERVER_GUIDE.md) — Operator guide: install, flash nodes, apply OTA updates, add peripherals, author routing sheets
- [controller/README.md](controller/README.md) — Controller install on a Steam Deck (OTA + AppImage)

**Build from source**

- [BUILD.md](docs/BUILD.md) — Build the server dist, node firmware, and controller from source

**Component landing pages**

- [server/README.md](server/README.md) — Server (ROS 2 `saint_os` package)
- [firmware/rp2040/README.md](firmware/rp2040/README.md) · [firmware/teensy41/README.md](firmware/teensy41/README.md) · [firmware/raspberrypi/README.md](firmware/raspberrypi/README.md) — Node firmware

**Reference**

- [docs/SAINT_OS_SPEC.md](docs/SAINT_OS_SPEC.md) — Full system specification
- [docs/HARDWARE.md](docs/HARDWARE.md) — Hardware requirements and supported platforms
- [docs/MAESTRO_BRINGUP.md](docs/MAESTRO_BRINGUP.md) — Pololu Maestro servo controller bring-up
- [docs/SOUNDBOARD.md](docs/SOUNDBOARD.md) — Per-node audio: register and trigger clips
- [controller/docs/SHEETS_BINDINGS.md](controller/docs/SHEETS_BINDINGS.md) — Binding controller inputs to routing-sheet WebSocket inputs
- [controller/docs/BINDINGS_SYSTEM.md](controller/docs/BINDINGS_SYSTEM.md) — Bindings data model (input sources, action types, preset panels)

## Project

Homepage: <https://github.com/input-inc/saintos>

## Sponsor

SAINT.OS is built by [Input Inc.](https://www.patreon.com/inputinc) If this
project is useful to you, please consider supporting development on
**[Patreon](https://www.patreon.com/inputinc)**.

## License

Licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike
4.0 International** license (CC BY-NC-SA 4.0) — see [LICENSE.MD](LICENSE.MD).

You are free to use, share, and adapt SAINT.OS **for non-commercial purposes**,
with attribution, provided you distribute your changes under the same license.
**Commercial use requires a separate license** — contact
[Input Inc.](https://github.com/input-inc) to arrange one.

## Contributing

Contributions are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md). The one
rule to internalize first: **SAINT.OS is robot-agnostic, and OpenSAINT is
just the first robot built on it.** Anything specific to a single robot
belongs in a robot manifest, board definition, peripheral config, or routing
sheet — not in the platform code.
