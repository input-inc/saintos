# SAINT.OS Raspberry Pi Node — Install Guide

This guide installs the **saint-node** firmware on a Raspberry Pi
(3 / 4 / 5). The same firmware runs on every supported generation
— Pi model is detected at startup and the GPIO chip, audio
output device, and announcement `hw` field are all selected
automatically.

> The Pi-node firmware is **distinct from the SAINT.OS server**.
> The server is the orchestrator (web UI, ROS2 bridge, routing
> evaluator) and ships as a separate Debian/AppImage bundle. A Pi
> running the server *can* also run saint-node — they're independent
> systemd services and the node will adopt itself like any other Pi
> on the network.

---

## 1. Supported hardware

| Pi model         | SoC      | GPIO chip          | 3.5 mm jack | Tested |
|------------------|----------|--------------------|-------------|--------|
| Raspberry Pi 5   | BCM2712  | RP1 → `gpiochip4`  | no          | ✓      |
| Raspberry Pi 4 B | BCM2711  | `gpiochip0`        | yes         | ✓      |
| Raspberry Pi 3 B+/3 B | BCM2837 | `gpiochip0`     | yes         | best-effort |
| Compute Module 4 | BCM2711  | `gpiochip0`        | depends     | likely-works |

The 40-pin GPIO header is identical across all of the above; routing
peripherals to specific pins from the SAINT.OS server's
**Peripherals** tab works the same on each. Pin layouts live in
`server/config/boards/raspberrypi/global.yaml`.

> Pi 5 dropped the analog 3.5 mm headphone jack. The built-in
> **audio_player** peripheral still works — output goes through
> HDMI, a USB DAC, or an I²S DAC HAT, picked via the `alsa_device`
> peripheral param.

## 2. Prerequisites

On a fresh Raspberry Pi OS Bookworm (64-bit) install:

1. **ROS 2 Jazzy** (preferred) or **Humble**.
   - Jazzy: <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
   - Humble: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
   - The installer detects whichever is present under `/opt/ros/`.

2. **Static or DHCP-stable IP** on the network the SAINT.OS server
   is on. The server discovers nodes over UDP multicast; nodes need
   to reach the server's IP for OTA downloads. Multi-homed servers
   are fine — the OTA URL the node receives carries the IP of
   *whichever* interface the server adopted the node on.

3. **Audio (optional)** — if you'll use the built-in `audio_player`:
   - Pi 4 / 3: enable the 3.5 mm jack in `raspi-config` → Advanced →
     Audio, or set `dtparam=audio=on` in `/boot/firmware/config.txt`.
   - Pi 5: plug in a USB DAC, route HDMI to an amplifier, or attach
     an I²S DAC HAT. Verify with `aplay -l` (should list at least one
     output card before you adopt the node).

## 3. Install

```bash
git clone <your fork URL> SaintOS
cd SaintOS/firmware/raspberrypi/scripts
sudo ./install.sh
```

The script:

1. Verifies it's running on a Pi (warns otherwise but continues).
2. `apt install`s system dependencies: `python3-pip python3-venv
   python3-gpiod libgpiod2 vlc-bin libvlc-dev python3-vlc alsa-utils`.
3. Finds ROS 2 under `/opt/ros/jazzy` or `/opt/ros/humble`.
4. `pip install`s `pyyaml gpiod` (uses `--break-system-packages`
   per Bookworm's PEP 668 stance).
5. Copies `saint_node/` to `/opt/saint-node/` and symlinks it into
   Python's site-packages so `python3 -m saint_node.node` works.
6. Creates:
   - `/etc/saint-node/`           — node identity + per-node config
   - `/var/lib/saint-os/audio/`   — audio library folder (drop
     `.wav`/`.mp3`/`.flac` here for the audio_player peripheral)
7. Installs `saint-node.service` under `/etc/systemd/system/`,
   substitutes the ROS setup path it found in step 3, and reloads
   systemd.
8. Adds a udev rule (`/etc/udev/rules.d/99-saint-gpio.rules`) so the
   `gpio` group can drive `gpiochip*`.

## 4. Start the service

```bash
sudo systemctl enable saint-node     # boot on every reboot
sudo systemctl start  saint-node     # start now
sudo systemctl status saint-node     # confirm it's Running
```

Live logs:

```bash
journalctl -u saint-node -f
```

You should see, within a few seconds:

```
Starting SAINT Node: raspberrypi_<hash>
Node initialized in state: UNADOPTED
Published capabilities (8 peripheral types)
```

Open the SAINT.OS web UI (the server should be on the same LAN) —
the Pi will appear in the **Discover** tab. Adopt it and the
`onboard_audio` peripheral auto-attaches from the board YAML.

## 5. OTA firmware updates

The Pi-node firmware updates **over the air** via the
[`FirmwareUpdater`](../saint_node/updater.py). The server stages a
zipped package; the node downloads it over HTTP, verifies a SHA256,
copies it into place, and restarts via systemd.

### Operator flow

1. On the server host, build / drop a new firmware package into
   `server/resources/firmware/raspberrypi/`. The release script
   (`scripts/build-local-dist.sh`) does this for you and writes a
   matching `info.json` with `version`, `filename`, `checksum`,
   `build_date`.
2. In the web UI, open the **Settings** view → **Firmware** card →
   pick the adopted Pi node → click **Update**.
3. The server publishes a `firmware_update` control message on the
   node's RELIABLE command topic. The node downloads from the URL,
   verifies the SHA256, backs up the current install to
   `/opt/saint/firmware/raspberrypi.backup`, copies the new
   firmware to `/opt/saint/firmware/raspberrypi`, and runs
   `sudo systemctl restart saint-node`.
4. The node re-announces with the new version under `fw` in the
   announcement JSON.

### What lives where on the Pi

| Path                                 | Owner    | Purpose                                |
|--------------------------------------|----------|----------------------------------------|
| `/opt/saint-node/`                   | install  | Initial firmware copy (from `install.sh`) |
| `/opt/saint/firmware/raspberrypi/`   | OTA      | Current OTA-installed firmware         |
| `/opt/saint/firmware/raspberrypi.backup/` | OTA | Previous firmware (rollback target)    |
| `/tmp/saint_firmware_staging/`       | OTA      | Download + extract scratch (cleared)   |
| `/etc/saint-node/config.yaml`        | runtime  | Node identity (role, display name)     |
| `/var/lib/saint-os/audio/`           | operator | Audio library for `audio_player`       |

### Rollback

If an OTA update breaks the firmware and the service won't start
cleanly, the previous version is one command away:

```bash
sudo systemctl stop saint-node
sudo rm -rf /opt/saint/firmware/raspberrypi
sudo cp -r /opt/saint/firmware/raspberrypi.backup /opt/saint/firmware/raspberrypi
sudo systemctl start saint-node
```

The OTA pipeline auto-rolls back on installation-step failures, but
this is the manual path for after-the-fact rollback.

### Disabling OTA

If your environment forbids OTA (corporate network, controlled-
deployment policy), stop the saint-node service from honoring
`firmware_update` actions by setting the systemd drop-in
`Environment=SAINT_DISABLE_OTA=1`. (Future work — not implemented
today; for now the surface is to gate the operator in the web UI.)

## 6. Troubleshooting

### "Cannot detect device model"

You're on a non-Pi (Mac / x86 Linux). The installer keeps going but
the runtime falls back to a generic `"Raspberry Pi"` for the `hw`
field. Real install on a Pi resolves this automatically.

### `aplay -l` shows no cards

The audio_player peripheral will adopt but `play_file` will fail.
On Pi 4/3 enable on-board audio in `raspi-config`. On Pi 5 plug in
the output device and reboot so ALSA enumerates it.

### Service flapping after OTA

Check `journalctl -u saint-node -n 200` for the failure. If it's a
Python import error, the package is broken — manually roll back
(see above) and report the bad build.

### "No route to host" when fetching the OTA URL

The server picks the IP it think the node can reach back on, based
on the node's last-known IP. If the node moved networks since
adoption, force a re-announce by restarting the node service. The
server's next `firmware_update` will carry the fresh URL.

## 7. Uninstall

```bash
cd firmware/raspberrypi/scripts
sudo ./uninstall.sh
```

Removes the systemd unit, `/opt/saint-node/`, the site-packages
symlink, and the udev rule. `/etc/saint-node/` and
`/var/lib/saint-os/audio/` are preserved — the operator can `rm` them
manually if a clean wipe is desired.
