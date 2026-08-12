# SAINT.OS Server

The central server — a ROS 2 package (`saint_os`) that coordinates the node
fleet, evaluates routing sheets, plays animations, serves the web UI + WebSocket
API, and stores/pushes OTA firmware. It runs on the robot's Raspberry Pi as the
`saint-os` systemd service.

> The ROS package is named `saint_os` even though the directory is `server/`.

## Overview

- **Node coordination** — nodes announce over ROS 2; operators *adopt* them from
  the web UI and push peripheral configuration.
- **Routing evaluator** — maps live inputs (gamepad, Unreal LiveLink, ROS
  topics, URDF joints) to peripheral channels through operator-authored routing
  sheets.
- **Animation player + soundboard** — keyframe timelines and per-node audio.
- **Web UI + WebSocket API** — `server/web/` (Vue 3); the API drives the
  controller app and any external client.
- **OTA firmware store** — hosts node firmware + the controller AppImage and
  pushes updates; can also update the server itself.

## Install

The server installs from a **prebuilt release** — no build required. Download
`saint-os_<version>_arm64_kilted.tar.zst` from the
[Releases](https://github.com/input-inc/saintos/releases) page and run
`install.sh` on the Pi.

- **In-depth install reference:** [`../docs/INSTALL.md`](../docs/INSTALL.md)
- **Guided operator walkthrough** (install → flash nodes → peripherals → routing
  sheets): [`docs/SERVER_GUIDE.md`](docs/SERVER_GUIDE.md)

## Building from source

```bash
cd server
colcon build --symlink-install
source install/setup.bash          # or setup.zsh on macOS
ros2 launch saint_os saint_server.launch.py
```

Per-platform ROS 2 Kilted setup, the `dev.sh` watch loop, and running the
micro-ROS agent are in [`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md). The
repo-wide build index (including the release dist tarball) is
[`../docs/BUILD.md`](../docs/BUILD.md).

## Documentation

- [`docs/SERVER_GUIDE.md`](docs/SERVER_GUIDE.md) — operator guide: install, flash nodes, OTA, peripherals, routing sheets
- [`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md) — development environment and dev loop
- [`docs/MAESTRO_PROTOCOL.md`](docs/MAESTRO_PROTOCOL.md) — Pololu Maestro wire protocol reference

## Troubleshooting

```bash
systemctl status saint-os
journalctl -u saint-os -f          # tail the logs
```

More install- and dev-time troubleshooting is in
[`../docs/INSTALL.md`](../docs/INSTALL.md#7-troubleshooting) and
[`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md#troubleshooting).
