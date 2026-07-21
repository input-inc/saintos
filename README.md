# SAINT.OS

**System for Articulated Intelligence and Navigation Tasks**

SAINT.OS is a ROS 2 control platform for multi-node animatronic and mobile
robots. A central server (Raspberry Pi) coordinates a fleet of
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

There are two paths: **deploy to a robot** (the normal path) and
**development** (build from source on a dev machine).

### 1. Deploy to a robot (Raspberry Pi)

The server ships as a self-contained dist tarball that installs offline. On
a dev machine (macOS or Linux) with Docker + Node:

```bash
# Build the dist tarball (bundles ROS 2, all deps, node firmware, and the
# controller app). Output: dist/saint-os_<version>_arm64_kilted.tar.zst
scripts/build-local-dist.sh
```

Copy the tarball to the Raspberry Pi and install:

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

### 2. First-run setup (from the web UI)

1. **Adopt nodes.** Powered-on nodes appear as *unadopted*. Adopt each one
   by giving it a **name** (its identity everywhere in the app), picking its
   **board**, and optionally a **role** from the active robot manifest.
2. **Flash node firmware.** Push firmware over the air from the UI, or flash
   an RP2040 in BOOTSEL mode with the built `.uf2`.
3. **Add peripherals.** Configure each node's peripherals (motors, servos,
   Maestro channels, NeoPixels, audio) and sync them to the node.
4. **Build routing sheets.** Wire controller / LiveLink / animation inputs to
   peripheral channels. See the operator guide below.

### 3. Controller app

```bash
cd controller
npm install
npm run tauri dev            # native dev run
npm run tauri build          # production bundle (Steam Deck: see controller/README.md)
```

### 4. Node firmware

```bash
cd firmware/rp2040  && ./build.sh hw     # or: sim  (Renode simulation)
cd firmware/teensy41 && ./build.sh hw    # PlatformIO under the hood
```

Raspberry Pi nodes run the Python firmware in `firmware/raspberrypi/`.

### 5. Run a different robot

SAINT.OS isn't tied to OpenSAINT. Add your own manifest at
`server/config/robots/<id>.yaml` (or, on an installed system,
`/etc/saint-os/robots/`):

```yaml
id: myrobot
name: My Robot
description: What it is.
homepage: https://example.com/myrobot
roles:            # node role labels offered in the adoption dropdown
  - Base
  - Head
```

Select it under **Settings → Robot** (the setting persists across restarts).

### Development (build from source)

For running the server directly on a dev machine (no dist tarball), see
[INSTALL.md](INSTALL.md) — it covers ROS 2 setup on macOS/Linux/Windows and
the `colcon build` workflow.

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

- [docs/SAINT_OS_SPEC.md](docs/SAINT_OS_SPEC.md) — Full system specification
- [docs/HARDWARE.md](docs/HARDWARE.md) — Hardware requirements and supported platforms
- [docs/MAESTRO_BRINGUP.md](docs/MAESTRO_BRINGUP.md) — Pololu Maestro servo controller bring-up
- [docs/SOUNDBOARD.md](docs/SOUNDBOARD.md) — Per-node audio: register and trigger clips
- [INSTALL.md](INSTALL.md) — Source-build installation guide (dev machines)
- [server/docs/SERVER_GUIDE.md](server/docs/SERVER_GUIDE.md) — Operator guide: install, flash nodes, apply OTA updates, add peripherals, author routing sheets
- [controller/README.md](controller/README.md) — Controller build & deployment (Steam Deck + native dev)
- [controller/docs/SHEETS_BINDINGS.md](controller/docs/SHEETS_BINDINGS.md) — Binding controller inputs to routing-sheet WebSocket inputs
- [controller/docs/BINDINGS_SYSTEM.md](controller/docs/BINDINGS_SYSTEM.md) — Bindings data model (input sources, action types, preset panels)

## Project

Homepage: <https://github.com/input-inc/OpenSaint>

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
