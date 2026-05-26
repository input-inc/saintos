# SAINT.OS Server — operator guide

Walks through everything you need to stand up a SAINT.OS robot from a
fresh Raspberry Pi:

1. [Install the server](#1-install-the-server)
2. [Flash UF2 firmware to the initial nodes](#2-flash-uf2-firmware-to-the-initial-nodes)
3. [Adopt nodes and apply OTA updates](#3-adopt-nodes-and-apply-ota-updates)
4. [Add peripherals to a node](#4-add-peripherals-to-a-node)
5. [Build routing sheets — inputs to outputs](#5-build-routing-sheets--inputs-to-outputs)

For developer-mode source builds (Ubuntu / macOS / Windows colcon
workspaces) see [`../../INSTALL.md`](../../INSTALL.md). This guide
covers the **packaged** deployment — a tarball-based install onto a
Pi that the robot actually ships with.

For a flat reference of every systemd unit and netplan path the
installer touches, see [`DEPLOYMENT.md`](DEPLOYMENT.md). This guide is
the workflow; that one is the inventory.

---

## 1. Install the server

### Hardware

- **Recommended:** Raspberry Pi 5 (4 GB+), USB-C 5V/5A supply, NVMe HAT
  + NVMe SSD (the routing evaluator + ROS bridge are I/O-heavy in
  practice).
- **Supported:** Raspberry Pi 4 (4 GB+), USB-C 5V/3A supply, A2-class
  microSD card.
- **Network:** built-in Ethernet for the internal peripheral bus,
  built-in WiFi for the operator-facing access point.

### OS image

Either of these works — both are 64-bit arm64:

- **Raspberry Pi OS Bookworm (64-bit, Lite)** — minimal install, no
  desktop. Recommended.
- **Ubuntu 24.04 Server (arm64)**.

Flash with the Raspberry Pi Imager. In the imager's advanced settings,
set:

- **Hostname:** `opensaint` (the installer expects this; override via
  `SAINT_HOSTNAME=…` if you need a different name).
- **Username:** `pi` (any account works; the installer creates a
  dedicated `saint` service user separately).
- **SSH:** enable, with your public key authorized.
- **WiFi:** leave **unset** — the installer takes over WiFi and turns
  the radio into an access point. If you preconfigure WiFi here the
  installer will undo it.

Boot the Pi. Find it on your network (`ssh pi@<router-assigned-ip>` or
`ssh pi@opensaint.local` if mDNS works on your network).

### Build the dist tarball

The installer is shipped as a self-contained tarball — it bundles ROS2
Jazzy, the micro-ROS agent, the saint_os package, plus apt-deps for
the target distro. From a Linux dev machine (Docker required):

```bash
cd SaintOS/source
scripts/build-local-dist.sh
```

This produces `dist/saint-os_<version>_arm64_jazzy.tar.zst`. The
script prints the SHA-256 and an scp/install command at the end.

Built artifact you should see:

```
dist/saint-os_0.5.1_arm64_jazzy.tar.zst       (~80 MB compressed)
```

### Copy to the Pi and install

```bash
scp dist/saint-os_*_arm64_jazzy.tar.zst pi@opensaint.local:/tmp/
ssh pi@opensaint.local
cd /tmp
tar -xaf saint-os_*_arm64_jazzy.tar.zst
sudo saint-os_*_arm64_jazzy/install.sh
```

`install.sh` is idempotent. It will:

| Step | What it does |
|---|---|
| Extract ROS2 + micro-ROS agent | Lands under `/opt/ros/jazzy/` — does **not** use a public apt repo, so this works on machines with no internet access |
| Install apt runtime deps | Uses the bundled local apt repo for `python3-packaging`, `nginx`, etc.; cleans up the temporary repo when done |
| Create the `saint` service user | Owns `/var/lib/saint-os/` (state) and `/var/log/saint-os/` (logs) |
| Install the saint_os ROS package | Lands under `/opt/saint-os/` |
| Install systemd units | `saint-os.service` (the server itself), plus `apply-update.sh` and `usb-helper.sh` privileged wrappers used by the OTA flow |
| Configure the WiFi access point | SSID `OpenSAINT`, passphrase `ifeelalive`, country `US`. Override at install time with `SAINT_WIFI_SSID=… SAINT_WIFI_PASS=…` |
| Configure mDNS + DHCP on the internal Ethernet bus | The Pi answers to `opensaint.local`; `eth0` hands out `192.168.10.10–254` to peripherals |
| Enable and start the service | Skip with `--no-start` if you want to inspect the install before launching |

Useful flags:

```bash
sudo ./install.sh --no-wifi          # keep host WiFi management as-is
sudo ./install.sh --no-dhcp          # don't run the internal-bus DHCP server
sudo ./install.sh --no-start         # install but don't enable / start
sudo ./install.sh --dry-run          # show what would happen
```

Verify the service is up:

```bash
systemctl status saint-os
journalctl -u saint-os -f            # tail the logs
```

Open the web UI from your laptop / Deck once it's connected to the
**OpenSAINT** WiFi AP:

```
http://opensaint.local/
```

Default WebSocket password is `12345`. Change it from the web UI
(**Settings → Security**) or by editing
`/etc/saint-os/server_config.yaml` and restarting the service:

```yaml
websocket:
  password: 'your-strong-password'
```

---

## 2. Flash UF2 firmware to the initial nodes

A fresh RP2040 or Teensy node has factory firmware on it — it knows
nothing about SAINT.OS until you write the saint_node firmware. After
that, OTA takes over for future updates.

### Where the UF2s live

The running server ships the latest UF2 over HTTP:

```
http://opensaint.local/api/firmware                   # list everything
http://opensaint.local/api/firmware/rp2040            # one type
http://opensaint.local/api/firmware/rp2040/saint_node.uf2
```

On the server filesystem they're at
`/opt/saint-os/install/share/saint_os/resources/firmware/<type>/`. The three
supported node types are:

| Type | Artifact | What it goes on |
|---|---|---|
| `rp2040` | `saint_node.uf2` (initial flash) and `saint_node_combined.uf2` (initial + OTA bootloader for the first install) | Raspberry Pi Pico W |
| `teensy41` | `firmware.hex` | Teensy 4.1 |
| `rpi5` | `saint_firmware_rpi5_<ver>.zip` | Raspberry Pi 5 acting as a peripheral node (not the server) |

The web UI surfaces them on the **Firmware** page with one-click
download buttons.

### Flashing an RP2040 (Pi Pico W)

1. Hold the **BOOTSEL** button on the Pico.
2. Plug it into your laptop via USB-C / micro-USB while still holding
   BOOTSEL.
3. Release BOOTSEL once the Pico mounts as a USB drive named
   `RPI-RP2`.
4. From your laptop:
   ```bash
   curl -OL http://opensaint.local/api/firmware/rp2040/saint_node_combined.uf2
   cp saint_node_combined.uf2 /Volumes/RPI-RP2/      # macOS
   # or: cp saint_node_combined.uf2 /media/$USER/RPI-RP2/   # Linux
   ```
5. The Pico reboots itself off USB mass storage and the volume
   unmounts — that's the signal the flash worked.
6. Plug the Pico into the SAINT.OS Ethernet bus. Within ~10 seconds
   the server's **Unadopted Nodes** list (web UI → **Nodes**) shows a
   new entry.

For the very first flash use `saint_node_combined.uf2` — it bundles
the OTA bootloader at the bottom of flash plus the application image
on top, so future OTA updates can replace only the application slot
without re-flashing the bootloader. For nodes that already have the
bootloader (i.e. anything you've adopted before), `saint_node.uf2` is
fine.

### Flashing a Teensy 4.1

1. Plug the Teensy into your laptop via USB.
2. Download `firmware.hex` from the Firmware page (or
   `http://opensaint.local/api/firmware/teensy41/firmware.hex`).
3. Open the **Teensy Loader** app (`teensy_loader_cli` works too):
   ```bash
   teensy_loader_cli --mcu=TEENSY41 -w -v firmware.hex
   ```
4. Press the program button on the Teensy when prompted (or pass `-s`
   to soft-reboot).
5. Plug into the internal Ethernet bus — the node appears in
   **Unadopted Nodes**.

### Bringing up an RPi 5 peripheral node

For a Pi-5 used as a peripheral (analog heartbeat + audio + display
sink rather than the server), download
`saint_firmware_rpi5_<ver>.zip`, unzip it on the target Pi, and run
the install script inside. It registers itself with the upstream
SAINT.OS server via mDNS. See `firmware/rpi5/` for details
on the script and systemd unit it installs.

---

## 3. Adopt nodes and apply OTA updates

### Adopt a freshly-flashed node

In the web UI → **Nodes** → **Unadopted**:

1. Click the row for the new node — the panel shows its `node_id`,
   chip family, and a few diagnostic counters.
2. Pick a **role** (e.g. `tracks`, `head`, `arm_left`) — roles come
   from the YAML files under
   `/opt/saint-os/install/share/saint_os/config/roles/` and are what the
   routing UI keys off when picking a sheet for the node.
3. Pick a **board** definition. Board YAMLs map physical pins on a
   chip to logical labels, so you don't have to refer to "GPIO 14"
   when you mean "M1 throttle". If your board isn't listed, click
   **+ Board → Paste YAML** to add one, or pick the closest match and
   tweak via **Edit Board YAML**.
4. (Optional) Set a **display name** — the human label that shows up
   everywhere ("Track Drive Right" beats `teensy41_A1B2C3D4`).
5. Click **Adopt**. The server pushes role + board down to the node,
   the node persists it to flash, and the node moves to the
   **Adopted Nodes** list.

The adopted node now appears as its own sheet on the **Routing**
page, with peripherals (once you add some) showing up as channel
sinks on that sheet.

### Apply an OTA update

When a new server is installed it brings new node firmware with it.
The web UI → **Updates** page handles:

- **Latest installed** — what's running on the server.
- **GitHub releases** — fetched from
  `github.com/input-inc/saintos` if internet is available.
- **USB-staged releases** — found by scanning `/media/*` and `/mnt/*`
  for `saint-os_*.tar.zst` tarballs. The page has a **Scan USB**
  button. If you're on an air-gapped robot, build the tarball on a
  dev machine and copy it onto a USB stick.

To apply:

1. Click **Download** (for a GitHub release) or **Stage from USB**
   (for a tarball on plugged-in media).
2. Click **Install**. The privileged `apply-update.sh` wrapper takes
   over: stop the service, swap `/opt/saint-os/` for the new tree,
   restart the service.
3. The server's WebSocket disconnects during the swap (~30 s);
   reconnect with the same password and the **Nodes** page surfaces
   any nodes that need their firmware re-pushed to match.

Once the server is up on the new release, each adopted node sees a
new firmware version available and the Nodes page shows an **Update
firmware** button. Click it and the OTA bootloader on the node fetches
the new image from `/api/firmware/<type>/saint_node.uf2`, writes it
into the application slot, and reboots. No physical access required —
that's what `saint_node_combined.uf2` was for during initial flash.

For Teensy nodes, OTA isn't available — the Teensy has no bootloader
slot model that supports it. Re-flash with the Teensy Loader using the
new `firmware.hex`.

---

## 4. Add peripherals to a node

A node is just a CPU + role until you tell it what's wired to its pins.
Peripherals are added in the web UI → **Nodes → \[node\] → Peripherals**
tab.

### Picking from the catalog

Click **+ Add Peripheral**. The catalog lists every peripheral type
the server knows about:

| Type | Example use |
|---|---|
| `servo` | RC servo on a PWM-capable pin |
| `roboclaw` | RoboClaw motor controller over serial (UART) |
| `neopixel` | WS2812 strip or individual pixel |
| `pwm_out` | Generic PWM channel |
| `analog_in` | Voltage / current sense |
| `gpio_in` / `gpio_out` | Bare digital pin |
| `i2c_*` | I2C-attached sensors / drivers (each one is a separate catalog entry) |
| `audio_out` | I2S / analog audio sink (Pi-5 nodes only) |

Pick a type. The form unfolds the parameters that type defines (e.g.
`roboclaw` wants a serial port, baud rate, and address; `servo` wants
a pin, min/max pulse, and centre).

### Channels vs peripherals

Each peripheral instance exposes one or more **channels**. A `servo`
has a single `position` channel; a `roboclaw` exposes `m1`, `m2`,
`m1_current`, `voltage_battery`, …; a `neopixel` strip exposes one
channel per logical group (per-pixel addressing isn't surfaced — you
make a group of pixels a single channel).

Channels are the unit the routing graph addresses, so when you wire a
WS-input to "RoboClaw → m1", the value flows into the m1 channel of
the named instance regardless of which pin/UART it's on.

### Saving and verifying

Click **Save**. The server pushes the peripheral config to the node;
the node configures the underlying hardware and acknowledges back. A
green dot next to the peripheral row means "node confirmed the config
is applied." A red one means the node rejected it — open
**Logs → \[node\]** to see why (typically pin conflicts or missing
hardware).

You can flip the **Log channel** toggle next to any channel to enable
file-backed history under
`/var/lib/saint-os/peripheral-logs/<node>/<peripheral>/<channel>.ndjson`.
That feeds the **History** view on the channel detail page — useful
for tuning servos and debugging actuator behaviour.

---

## 5. Build routing sheets — inputs to outputs

The **Routing** page is one sheet per adopted node. Each sheet is a
free-form graph: ROS topic inputs and WebSocket inputs feed operator
nodes which feed outputs and peripheral-channel sinks.

The full controller-side authoring flow is in
[`../../controller/docs/SHEETS_BINDINGS.md`](../../controller/docs/SHEETS_BINDINGS.md);
this section covers the server-side picture.

### Sheet anatomy

Five kinds of nodes appear on a sheet:

| Kind | Role | Source / sink? |
|---|---|---|
| **Input** | A ROS topic the server is subscribed to (e.g. `/joy`, `/cmd_vel`). One scalar field per node, picked from the topic's flattened field list | Source only |
| **WebSocket Input** | A slot the controller writes into. Two flavours — `command` (joystick → slot) and `state` (sensor data echoed for downstream wires) | Source only |
| **Operator** | A pure math transform — `deadzone`, `scale_to_range`, `mix`, `curve`, `clamp`, `add`, `multiply`, … | Has typed input pins and one output |
| **Output** | A ROS topic publisher AND a tap. A value wired into the output also publishes on the configured topic | Sink + source |
| **Peripheral Channel** | Sink that writes the value down to a node's peripheral. Surfaces automatically for every channel the node's peripherals declare | Sink only |
| **Widget** | A sink that surfaces the value on the controller's live-readings dashboard (gauge / meter / readout) | Sink only |

### Wiring it up

1. Open the sheet for the node whose peripheral you want to drive.
2. **Add → WebSocket Input** if a controller will drive it, or **Add
   → Input** if the value comes from a ROS topic the server is
   already subscribed to.
3. **Add → Operator** as many times as you need. Most common chain:
   `deadzone → curve (expo) → scale_to_range (min/max)`. Operators
   have literal-value defaults on every input pin — handy for
   wiring `min/max` into a scale node without dragging from a
   "constant" source.
4. Drag a wire from the source's output pin to the operator's input
   pin. Drag from the operator's output to the peripheral channel's
   input.
5. (Optional) Add an **Output** to also publish the value as a ROS
   topic — other ROS nodes can then subscribe, and the value is
   available as a source for downstream wires on the same sheet
   without re-deriving it.

### Live values

The evaluator pushes a value snapshot per tick to the UI; every node
on the sheet shows the current value on its card. Push a controller
axis bound to a WS-input and you should see the values light up:
WS-input → operator → operator → channel sink. If a node stays at 0
while upstream lights up, the wire isn't connected or an operator
has a missing default.

### Worked example — "right stick Y drives the pan servo"

1. **Node side:** server-side; the Head node has been adopted and a
   `servo` peripheral added on the pan pin. Its single channel is
   `pan/position`.
2. **Routing page, Head sheet:** **Add → WebSocket Input**, label
   "Pan", kind `command`. Save the sheet.
3. **Add → Operator**, pick `scale_to_range`. Set the input range
   defaults to `-1..1` (controller scale) and output range to
   `-0.6..0.6` (limit servo travel to ±60% so it doesn't bind).
4. Wire **Pan** → `scale_to_range` input. Wire `scale_to_range` output
   → **pan/position** channel sink.
5. **Controller side:** Bindings → Add → Analog → Direct Control,
   source `right_stick_y`, target picker → pick the Head sheet → pick
   the "Pan" WS-input. Tweak deadzone to `0.05`, save.
6. Push the right stick — the servo follows.

That's the full input-to-output story. Everything else (mixers, OTA
updates pushing new firmware, peripheral hot-add) reuses the same
five primitives.

---

## Where things live on disk

For reference when poking at a live server:

| Path | What it is |
|---|---|
| `/opt/saint-os/` | Installed code (overwritten on update) |
| `/etc/saint-os/server_config.yaml` | Operator-editable config |
| `/var/lib/saint-os/` | Persistent state — adopted-node DB, routing sheets, peripheral configs |
| `/var/lib/saint-os/updates/` | Staged update tarballs |
| `/var/log/saint-os/` | Service logs |
| `/opt/saint-os/install/share/saint_os/resources/firmware/` | Node firmware artifacts served at `/api/firmware/` |
| `/etc/systemd/system/saint-os.service` | systemd unit |

Useful commands:

```bash
sudo systemctl restart saint-os
sudo journalctl -u saint-os -f --since "10 minutes ago"
sudo -u saint /opt/saint-os/bin/saint-shell    # if installed — exec into the ROS env
```

## Related docs

- [`DEPLOYMENT.md`](DEPLOYMENT.md) — every systemd / netplan / firewall
  path the installer touches (reference, not workflow)
- [`DEVELOPMENT.md`](DEVELOPMENT.md) — running the server out of a
  colcon workspace for development
- [`../../INSTALL.md`](../../INSTALL.md) — building the server from
  source on macOS / Linux / Windows
- [`../../controller/README.md`](../../controller/README.md) — building
  and deploying the controller Flatpak
- [`../../controller/docs/SHEETS_BINDINGS.md`](../../controller/docs/SHEETS_BINDINGS.md)
  — controller-side bindings → routing-sheet WS-inputs walkthrough
