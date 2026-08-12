# Building SAINT.OS from source

Most people never need this. SAINT.OS installs from a **prebuilt release** —
see [INSTALL.md](INSTALL.md) for the install-first path (download the server
dist, flash nodes from the UI, install the controller). Build from source only
when you're **developing** SAINT.OS or **cutting your own release**.

This is the single umbrella for every from-source build. Each section links to
the component's own deep-dive doc.

## What you can build

| Component | From | Command | Output |
|-----------|------|---------|--------|
| **Server dist** (the release artifact) | repo root | `scripts/build-local-dist.sh` | `dist/saint-os_<version>_arm64_kilted.tar.zst` |
| **Server dev workspace** | `server/` | `colcon build --symlink-install` | `install/` (run in place) |
| **RP2040 firmware** | `firmware/rp2040/` | `./build.sh hw` \| `sim` | `.uf2` |
| **Teensy 4.1 firmware** | `firmware/teensy41/` | `./build.sh hw` \| `sim` | `.hex` |
| **Raspberry Pi node firmware** | repo root | `scripts/build-local-dist.sh` | `firmware/raspberrypi/dist/saint_firmware_raspberrypi_<v>.tar.zst` |
| **Pimoroni Servo 2040 image** | `firmware/pimoroni_servo2040/` | `cmake` | `.uf2` |
| **Controller AppImage** | repo root | `controller/appimage/build-docker.sh` | `saint_firmware_controller_<v>.AppImage` |

The server dist **bundles** the node firmware and the controller AppImage, so
`build-local-dist.sh` is the one command that produces the shippable release.

## Prerequisites

- **Docker** — required for `build-local-dist.sh` and the controller AppImage
  (both build in Linux containers so output is correct regardless of host).
- **Node.js + npm** — the web UI and controller frontend.
- **Python 3.11** — server package + build tooling.
- **ROS 2 Kilted** — only for a native server **dev** workspace (the dist build
  supplies its own bundled ROS 2). See
  [server/docs/DEVELOPMENT.md](../server/docs/DEVELOPMENT.md) for per-platform ROS 2
  setup.
- **PlatformIO** (`pip install platformio`) — Teensy / RP2040 firmware.
- **ARM GCC toolchain + Pico SDK** — RP2040 firmware (the `build.sh` scripts
  fetch/point at these; see `firmware/rp2040/docs/INSTALL.md`).

## Server

### The dist tarball (release artifact)

The dist is self-contained: it bundles ROS 2 Kilted, the micro-ROS agent, the
`saint_os` package, node firmware, the controller AppImage, and the apt deps for
the target Pi — so it installs **offline** on a robot with no internet. Docker is
required on the build host.

```bash
cd SaintOS/source
scripts/build-local-dist.sh
# → dist/saint-os_<version>_arm64_kilted.tar.zst (prints SHA-256 + scp/install hint)
```

Install the result per [INSTALL.md](INSTALL.md) / the
[operator guide](../server/docs/SERVER_GUIDE.md).

### Native dev workspace

For an inner-loop dev build (no tarball), build the ROS 2 package directly:

```bash
cd server
colcon build --symlink-install
source install/setup.bash        # or setup.zsh on macOS
ros2 launch saint_os saint_server.launch.py
```

Per-platform ROS 2 Kilted setup (Linux apt, macOS Conda/RoboStack, Docker),
the `dev.sh` watch-rebuild loop, and running the micro-ROS agent are covered in
[server/docs/DEVELOPMENT.md](../server/docs/DEVELOPMENT.md).

## Node firmware

### RP2040 (Adafruit Feather + W5500)

```bash
cd firmware/rp2040
./build.sh hw     # hardware .uf2
./build.sh sim    # Renode simulation build
```

Toolchain setup, flashing methods, and pin/network config:
[firmware/rp2040/docs/INSTALL.md](../firmware/rp2040/docs/INSTALL.md). Renode
simulation: [firmware/rp2040/docs/SIMULATION.md](../firmware/rp2040/docs/SIMULATION.md).
Before "cleaning up" any RP2040 driver, read
[firmware/rp2040/docs/MAINTENANCE_NOTES.md](../firmware/rp2040/docs/MAINTENANCE_NOTES.md).

### Teensy 4.1

```bash
cd firmware/teensy41
./build.sh hw     # hardware .hex (PlatformIO under the hood)
./build.sh sim    # Renode simulation build
```

See [firmware/teensy41/README.md](../firmware/teensy41/README.md).

### Raspberry Pi node

The Pi node is a Python package, not compiled firmware. Its offline install
bundle is produced by the dist build and can also be packaged on its own — see
[firmware/raspberrypi/docs/INSTALL.md](../firmware/raspberrypi/docs/INSTALL.md)
(§3, "Build the offline bundle").

### Pimoroni Servo 2040

A one-time hand-provisioned I2C peripheral image (not part of the OTA pipeline).
Build + flash instructions:
[firmware/pimoroni_servo2040/README.md](../firmware/pimoroni_servo2040/README.md).

## Controller app

Built as a single self-contained `.AppImage` in a linux/amd64 Docker container:

```bash
controller/appimage/build-docker.sh          # incremental build
controller/appimage/build-docker.sh --clean  # from scratch
```

For a native dev inner loop (`npm run tauri dev`) and the full Steam Deck story,
see [controller/README.md](../controller/README.md).

## Continuous integration

`.github/workflows/dist.yml` builds every component above and, on a `v*` tag,
publishes the server dist tarball to the
[Releases](https://github.com/input-inc/saintos/releases) page. The CI and local
builds share the same scripts (`scripts/build-local-dist.sh`,
`controller/appimage/build-bundle.sh`), so local and released artifacts stay in
lockstep.

## Related docs

- [INSTALL.md](INSTALL.md) — install the prebuilt release (the normal path)
- [server/docs/DEVELOPMENT.md](../server/docs/DEVELOPMENT.md) — server dev environment
- [CONTRIBUTING.md](../CONTRIBUTING.md) — contribution workflow + the robot-agnostic rule
