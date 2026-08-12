# SAINT.OS Teensy 4.1 Firmware — Install & Build

Flashing and building the Teensy 4.1 node firmware. The firmware ships **inside
the server dist**, so the normal path is OTA from the server — you only build or
hand-flash for first bring-up or development.

## Install / flash

### Over the air (recommended)

Once the node is adopted in the web UI, push the Teensy firmware from the
server's firmware store. See the
[operator guide, §3](../../../server/docs/SERVER_GUIDE.md#3-adopt-nodes-and-apply-ota-updates).

### First bring-up (Teensy Loader)

A blank board needs one USB flash before it can OTA:

1. Plug the Teensy into your computer via USB.
2. Download `firmware.hex` from the server
   (`http://opensaint.local/api/firmware/teensy41/firmware.hex`).
3. Flash it with the Teensy Loader app, or the CLI:
   ```bash
   teensy_loader_cli --mcu=TEENSY41 -w -v firmware.hex
   ```
4. Press the program button when prompted (or pass `-s` to soft-reboot), then
   plug into the internal Ethernet bus — the node appears under **Unadopted
   Nodes**.

## Build from source

```bash
cd firmware/teensy41
./build.sh hw     # hardware .hex (PlatformIO under the hood)
./build.sh sim    # Renode simulation build
```

Prerequisites (PlatformIO, a working ROS 2 environment for the micro-ROS
library) and the repo-wide build index are in
[`../../../docs/BUILD.md`](../../../docs/BUILD.md).

> The shared build tree is env-specific — `build.sh` cleans `build/shared/` per
> target, so prefer it over a bare `pio run` when switching between `hw` and
> `sim` (a stale sim object in a hardware build swaps `Serial`→`Serial1` and
> bricks boot).

## Troubleshooting

- **Node never appears after flashing** — confirm the program button was pressed
  (the Teensy Loader waits for it) and the board is on the internal Ethernet bus.
- **Maestro-driven servos don't move** — see
  [`../../../docs/MAESTRO_BRINGUP.md`](../../../docs/MAESTRO_BRINGUP.md); check
  the Maestro's amber LED (a 2-blink heartbeat means a user script is overriding
  targets).
