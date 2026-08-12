# SAINT.OS Installation Guide

This is the in-depth guide to **installing** SAINT.OS from a prebuilt release.
You do not need to build anything — the server ships as a self-contained dist
tarball that installs **offline** and bundles the node firmware and the
controller app.

The install story is three steps:

1. **Install the server** on a Raspberry Pi (this guide).
2. **Adopt and flash nodes** from the server's web UI.
3. **Install the controller** on a Steam Deck.

> Building from source instead? See **[BUILD.md](BUILD.md)**.
> Want the guided operator walkthrough end to end (install → flash → peripherals
> → routing sheets)? See **[server/docs/SERVER_GUIDE.md](../server/docs/SERVER_GUIDE.md)**.

## Table of Contents

- [What gets installed](#what-gets-installed)
- [1. Prerequisites](#1-prerequisites)
- [2. Download the release](#2-download-the-release)
- [3. Install the server](#3-install-the-server)
- [4. Flash firmware to nodes](#4-flash-firmware-to-nodes)
- [5. Install the controller](#5-install-the-controller)
- [6. Run a different robot](#6-run-a-different-robot)
- [7. Troubleshooting](#7-troubleshooting)
- [Next steps](#next-steps)

---

## What gets installed

The dist tarball is fully self-contained. `install.sh` lays down:

- **ROS 2 Kilted** + the micro-ROS agent, under `/opt/ros/kilted/` — no public
  apt repo, so it installs on a Pi with no internet.
- The **`saint_os` server package**, under `/opt/saint-os/`.
- A dedicated **`saint` service user** owning `/var/lib/saint-os/` (state) and
  `/var/log/saint-os/` (logs).
- The **`saint-os.service`** systemd unit plus the privileged OTA wrappers.
- **WiFi access point**, **mDNS**, and **internal-bus DHCP** configuration.
- The **node firmware store** and **controller AppImage**, served to nodes and
  the controller over the air.

---

## 1. Prerequisites

### Server hardware

| | Platform | Notes |
|---|----------|-------|
| **Recommended** | Raspberry Pi 5 (4 GB+) | USB-C 5V/5A supply, NVMe HAT + SSD — the routing evaluator and ROS bridge are I/O-heavy |
| **Supported** | Raspberry Pi 4 (4 GB+) | USB-C 5V/3A supply, A2-class microSD |

Also: built-in **Ethernet** for the internal peripheral bus, built-in **WiFi**
for the operator-facing access point.

### OS image

Either works — both 64-bit arm64:

- **Raspberry Pi OS Bookworm (64-bit, Lite)** — minimal, no desktop. Recommended.
- **Ubuntu 24.04 Server (arm64)**.

Flash with the Raspberry Pi Imager. In the imager's advanced settings:

- **Hostname:** `opensaint` (the installer expects this; override with
  `SAINT_HOSTNAME=…`).
- **Username:** `pi` (any account works — the installer creates the `saint`
  service user separately).
- **SSH:** enable, with your public key authorized.
- **WiFi:** leave **unset** — the installer takes over WiFi and turns the radio
  into an access point. Preconfigured WiFi here will be undone by the installer.

Boot the Pi and find it: `ssh pi@opensaint.local` (if mDNS works) or
`ssh pi@<router-assigned-ip>`.

### Network

The Pi becomes the robot's network hub: it hands out `192.168.10.10–254` on
`eth0` to peripheral nodes and hosts the **OpenSAINT** WiFi AP for operator
devices (laptop, Steam Deck).

---

## 2. Download the release

Grab the latest server dist tarball from the
[**Releases**](https://github.com/input-inc/saintos/releases) page —
`saint-os_<version>_arm64_kilted.tar.zst` (~80 MB compressed).

With the GitHub CLI:

```bash
gh release download --repo input-inc/saintos \
    --pattern 'saint-os_*_arm64_kilted.tar.zst'
```

---

## 3. Install the server

### Copy to the Pi

```bash
scp saint-os_*_arm64_kilted.tar.zst pi@opensaint.local:/tmp/
ssh pi@opensaint.local
cd /tmp
tar --zstd -xf saint-os_*_arm64_kilted.tar.zst
```

### Run the installer

```bash
sudo saint-os_*_arm64_kilted/install.sh
```

`install.sh` is **idempotent** — re-running it updates in place. It:

| Step | What it does |
|---|---|
| Extract ROS 2 + micro-ROS agent | Lands under `/opt/ros/kilted/` — no public apt repo, works offline |
| Install apt runtime deps | Uses the bundled local apt repo (`nginx`, `python3-packaging`, …); cleans up after |
| Create the `saint` service user | Owns `/var/lib/saint-os/` (state) and `/var/log/saint-os/` (logs) |
| Install the `saint_os` package | Lands under `/opt/saint-os/` |
| Install systemd units | `saint-os.service` plus the `apply-update.sh` / `usb-helper.sh` OTA wrappers |
| Configure the WiFi access point | SSID `OpenSAINT`, passphrase `ifeelalive`, country `US`. Override with `SAINT_WIFI_SSID=… SAINT_WIFI_PASS=…` |
| Configure mDNS + internal-bus DHCP | Pi answers to `opensaint.local`; `eth0` serves `192.168.10.10–254` to peripherals |
| Enable and start the service | Skip with `--no-start` |

Useful flags:

```bash
sudo ./install.sh --no-wifi     # keep host WiFi management as-is
sudo ./install.sh --no-dhcp     # don't run the internal-bus DHCP server
sudo ./install.sh --no-start    # install but don't enable / start
sudo ./install.sh --dry-run     # show what would happen
sudo ./install.sh --help        # all options
```

### Verify

```bash
systemctl status saint-os
journalctl -u saint-os -f       # tail the logs
```

### Open the web UI

Connect your laptop / Deck to the **OpenSAINT** WiFi AP, then:

```
http://opensaint.local/         # or http://<pi-ip>/
```

The default WebSocket password is `12345`. Change it from the web UI
(**Settings → Security**) or by editing `/etc/saint-os/server_config.yaml` and
restarting the service:

```yaml
websocket:
  password: 'your-strong-password'
```

---

## 4. Flash firmware to nodes

Everything below happens in the web UI — the dist already shipped the firmware
into the server's store. Powered-on nodes appear as **unadopted**; adopt each
(name + board + optional role), then push firmware **over the air**. For an
RP2040's first bring-up, flash the bundled `.uf2` in BOOTSEL mode; Raspberry Pi
nodes install the bundled `saint_firmware_raspberrypi` tarball.

The full node-flashing walkthrough (UF2 locations, per-board steps, OTA flow) is
in the [operator guide, §2–§3](../server/docs/SERVER_GUIDE.md#2-flash-uf2-firmware-to-the-initial-nodes).
Per-board detail lives in each firmware landing page:
[RP2040](../firmware/rp2040/README.md) ·
[Teensy 4.1](../firmware/teensy41/README.md) ·
[Raspberry Pi](../firmware/raspberrypi/README.md).

---

## 5. Install the controller

The Steam Deck controller installs with **no toolchain**:

- **OTA from the server** — the controller's **Settings** tab polls the SAINT.OS
  server and self-updates the running AppImage in place.
- **Bundled AppImage** — the server ships `saint_firmware_controller_*.AppImage`;
  drop it on the Deck and add it to Steam as a Non-Steam Game.

First-time Steam Deck setup (get the AppImage on, add to Steam, artwork, Game
Mode launch): [controller/README.md](../controller/README.md).

---

## 6. Run a different robot

SAINT.OS is robot-agnostic — OpenSAINT is just the reference robot. Add your own
manifest at `/etc/saint-os/robots/<id>.yaml` on the installed Pi (or
`server/config/robots/<id>.yaml` in-tree):

```yaml
id: myrobot
name: My Robot
description: What it is.
homepage: https://example.com/myrobot
roles:            # node role labels offered in the adoption dropdown
  - Base
  - Head
```

Select it under **Settings → Robot** (persists across restarts).

---

## 7. Troubleshooting

### Web UI unreachable at `opensaint.local`

- Confirm you're on the **OpenSAINT** WiFi AP (or the same wired subnet).
- mDNS may not resolve on every network — try the Pi's IP directly.
- `systemctl status saint-os` — is the service running? `journalctl -u saint-os -e`
  for the tail.

### Installer exits early

- Re-run it — `install.sh` is idempotent. Add `--dry-run` first to see the plan.
- On a non-`opensaint` hostname, pass `SAINT_HOSTNAME=<your-host>`.

### Nodes don't appear as unadopted

- Verify the node is powered and wired to the internal Ethernet bus (or on the
  same subnet for a Pi node).
- Check the node got a `192.168.10.x` lease (unless you installed with
  `--no-dhcp` and run your own DHCP).
- See [server/docs/SERVER_GUIDE.md](../server/docs/SERVER_GUIDE.md) and each firmware
  README's troubleshooting section.

### OTA update stalls

- The apply wrapper detaches the extract/install; give it a minute and watch
  `journalctl -u saint-os -f`. For the OTA internals see
  [server/docs/SERVER_GUIDE.md#3-adopt-nodes-and-apply-ota-updates](../server/docs/SERVER_GUIDE.md#3-adopt-nodes-and-apply-ota-updates).

---

## Next steps

- **[server/docs/SERVER_GUIDE.md](../server/docs/SERVER_GUIDE.md)** — the full
  operator workflow: flash nodes, add peripherals, author routing sheets.
- **[docs/SAINT_OS_SPEC.md](SAINT_OS_SPEC.md)** — system architecture.
- **[docs/HARDWARE.md](HARDWARE.md)** — hardware requirements.
- **[BUILD.md](BUILD.md)** — build any component from source.
