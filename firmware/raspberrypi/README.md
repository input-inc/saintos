# SAINT.OS Raspberry Pi Node Firmware

ROS 2 node firmware for **Raspberry Pi 3 / 4 / 5**. Runs as a standard
ROS 2 process on Linux (no micro-ROS agent) and communicates with the
SAINT.OS server over native DDS. Pi model is auto-detected at startup
— the same firmware build runs on every supported generation.

> **Looking to install on a Pi?** See [`docs/INSTALL.md`](docs/INSTALL.md)
> for the full step-by-step walkthrough including offline-bundle build,
> supported hardware matrix, OTA flow, and troubleshooting.

## Overview

Unlike the RP2040 / Teensy 4.1 firmware (micro-ROS, flashed firmware),
this is a Python package running under `systemd`:

- Distributed as `saint_firmware_raspberrypi_<v>.tar.zst` (multi-target —
  carries both Bookworm and Trixie payloads; `install.sh` picks the
  right half from `/etc/os-release` at install time)
- Service `saint-node.service` runs as `root`, sourcing ROS 2 Kilted
  before exec'ing `python3 -m saint_node.node`
- Peripherals persist to `/etc/saint-node/config.yaml` so they replay
  on reboot without needing a server re-push

## Features

- **Hardware abstraction**: Pi 3 / 4 / 5 detected at boot — GPIO chip
  (`gpiochip0` vs `gpiochip4`), audio output (3.5 mm jack vs HDMI/USB
  DAC), `hw` announcement field all picked automatically.
- **Peripheral-first addressing**: Maestro servos, RoboClaw motor
  drivers, JBD/Pathfinder BMS over BLE, Tic stepper drivers, FAS-100
  IMUs, on-board audio, console kiosk display. Same peripheral model
  the RP2040 / Teensy firmware uses.
- **OTA update**: server pushes a new zip URL; the node downloads,
  verifies, swaps `/opt/saint-node/`, and restarts via `systemctl`.
- **Local activity log**: emits structured frames to
  `/saint/nodes/<id>/log` so the server-side activity feed narrates
  every operator action (restart, firmware update, config apply,
  estop) without needing journal SSH.

## Requirements

- **Raspberry Pi 3 / 4 / 5** — any 64-bit-capable Pi.
- **Pi OS Bookworm (Debian 12) or Trixie (Debian 13)**, 64-bit. The
  installer detects the release and resolves the matching `_ros2_install/`
  + `deps/` subtrees from the firmware bundle. Pi OS 32-bit isn't
  supported (ROS 2 Kilted has no armhf binaries).
- **ROS 2 Kilted** — bundled inside the firmware zip (source-built
  against each target release for ABI compatibility). No separate
  ROS install step needed if you're using the offline bundle.
- **Network** — DHCP-stable IP on the same subnet as the SAINT.OS
  server.

The bundled firmware zip is **fully self-contained**. The Pi does
**not** need internet access at install time when the zip carries
the `deps/` deb cache (the default; see §3.1 of `docs/INSTALL.md`).

## Install — short form

On the dev machine:

```bash
scripts/build-local-dist.sh
# Produces firmware/raspberrypi/dist/saint_firmware_raspberrypi_<v>.tar.zst
# (the multi-target bundle for both Bookworm and Trixie Pi targets)
```

scp the bundle to the Pi, then on the Pi:

```bash
cd /tmp
tar -xaf saint_firmware_raspberrypi_<v>.tar.zst   # -xaf auto-detects .zst
cd saint_firmware_raspberrypi_<v>/scripts
sudo ./install.sh
```

`install.sh` is **idempotent** — re-running it on an already-installed
Pi just updates in place. Service is enabled and started automatically;
no manual `systemctl enable` step required.

See [`docs/INSTALL.md`](docs/INSTALL.md) for the long form (build the
offline bundle, OTA flow, rollback, uninstall, troubleshooting).

## Layout on disk (after install)

| Path                                | Purpose                                  |
|-------------------------------------|------------------------------------------|
| `/opt/saint-node/saint_node/`       | Python package (symlinked into site-packages) |
| `/etc/saint-node/config.yaml`       | Node identity + persisted peripherals    |
| `/etc/saint-node/console-kiosk/`    | Kiosk launcher script + URL state file (console_display peripheral) |
| `/etc/xdg/autostart/saint-console-kiosk.desktop` | System-wide kiosk autostart entry |
| `/var/lib/saint-os/audio/`          | Audio library for `audio_player` peripheral |
| `/etc/systemd/system/saint-node.service` | systemd unit, sources ROS 2 then execs python3 |
| `/opt/ros/<distro>/install/`        | Bundled ROS 2 install tree              |

## ROS 2 topics

### Published by the node

| Topic                                  | Type             | Description                          |
|----------------------------------------|------------------|--------------------------------------|
| `/saint/nodes/announce`                | `std_msgs/String`| Node announcements (1 Hz)            |
| `/saint/nodes/<id>/capabilities`       | `std_msgs/String`| Pin/peripheral capabilities          |
| `/saint/nodes/<id>/state`              | `std_msgs/String`| Channel states (10 Hz)               |
| `/saint/nodes/<id>/log`                | `std_msgs/String`| Structured activity log              |
| `/saint/nodes/<id>/update_progress`    | `std_msgs/String`| OTA progress frames (stage + percent) |
| `/saint/nodes/<id>/ble_scan_results`   | `std_msgs/String`| Async BLE scan results               |

### Subscribed

| Topic                       | Type             | Description                              |
|-----------------------------|------------------|------------------------------------------|
| `/saint/nodes/<id>/config`  | `std_msgs/String`| `configure` / `adopt` (RELIABLE)         |
| `/saint/nodes/<id>/control` | `std_msgs/String`| Streaming setpoints + one-shot actions   |
| `/saint/nodes/<id>/command` | `std_msgs/String`| RELIABLE one-shots (firmware_update, etc.) |

## Configuration file shape

```yaml
node_id: raspberrypi_62e22359e3fc
role: head
display_name: "Head Controller"
adopted: true
pins: {}                # Legacy GPIO pin configs (pre-peripheral-first)
peripherals:            # Persisted from the server's last config push
  - id: console-1
    type: console_display
    pins: {}
    params:
      view: batteries
      server_url: http://opensaint.local:8080
      autostart: true
      _kiosk_token: <secret>
network:
  agent_host: 192.168.1.1
  agent_port: 8888
```

`peripherals` is the source of truth on the Pi — it replays on every
boot so the node comes up with the same configuration without waiting
for the server to re-push.

## Development

To run the node out of the source tree without installing:

```bash
source /opt/ros/kilted/install/setup.bash
cd firmware/raspberrypi
python3 -m saint_node.node
```

The GPIO controller falls back to a mock mode when `libgpiod` isn't
available, so dev hosts (Mac, x86 Linux) can boot the node for
protocol testing.

## Troubleshooting

See `docs/INSTALL.md` §6 for the full troubleshooting playbook. Quick
checks:

```bash
# Service status + recent logs
sudo systemctl status saint-node
journalctl -u saint-node -n 100

# What the kiosk would launch (if console_display is configured)
cat /etc/saint-node/console-kiosk/url

# Peripheral state on disk
cat /etc/saint-node/config.yaml
```

## License

Part of the SAINT.OS project.
