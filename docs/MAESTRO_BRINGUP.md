# Pololu Mini Maestro 24 — Software Bringup

## Status: Solved (2026-06-16)

Servos on ch1/ch2/ch3 move when a target is sent over USB vendor protocol
from the Pi.

## Root Cause

Every Mini Maestro channel has a **HomeMode** (Off / Ignore / Goto) and
an associated **HOME** value. The encoding in EEPROM:

- `HomeMode.Off` → HOME = 0 — **channel is disabled, no PWM output on restart**
- `HomeMode.Ignore` → HOME = 1 — channel keeps target across restarts
- `HomeMode.Goto` → HOME = <position in qus> — channel comes up at this position, PWM enabled

A fresh / factory-default Maestro has every channel in `HomeMode.Off` with
HOME=0. From that state, `SET_TARGET` is accepted (the target field reads
back correctly) but the channel never generates PWM — and Get Servo
Settings reports `position == 0` to mirror "no output."

**The fix:** for each channel that should output PWM, configure it in
Maestro Control Center with **Mode = Servo** and **On startup or error =
Go to <position>** (this writes EEPROM `HomeMode = Goto`, HOME = the
position you set). After that, runtime `SET_TARGET (0x85)` works as
expected and the position field becomes honest — it ramps at the
runtime speed limit and reaches target accurately.

## Verification Recipe

The cleanest "is PWM real" test, since the firmware will report
position == target instantly when speed=0 even on a dead channel:

```python
dev.ctrl_transfer(0x40, 0x84, 20, ch, 0)              # runtime speed = 20 qus/cycle
dev.ctrl_transfer(0x40, 0x84,  0, ch | 0x80, 0)       # accel unlimited
dev.ctrl_transfer(0x40, 0x85, 4000, ch, 0)            # target = 1000us
time.sleep(2)                                          # let it settle
dev.ctrl_transfer(0x40, 0x85, 8000, ch, 0)            # target = 2000us
# Sample Get Servo Settings (0x87): position should ramp 4000→8000
# at ~2000 qus/sec. If it stays exactly at 4000, the channel is
# in HomeMode.Off and PWM is dead.
```

`/tmp/maestro_verify_and_move.py` does this end-to-end.

## What We Tried That DIDN'T Work (Lessons)

- **SET_PARAMETER writes to per-channel HOME via the change-then-set
  pattern** appeared to persist through `REINITIALIZE (0x90)` but
  reverted to 0 across a real power cycle. SET_PARAMETER does **not**
  write EEPROM on this firmware revision — only MCC's setRawParameter
  sequence persists. (How MCC actually commits to EEPROM, we don't
  know — possibly a timing requirement or a specific opcode we missed.)
- **USB device reset (`dev.reset()`)** and `START_BOOTLOADER (0xFF)` /
  power cycle do not change the Off state of channels with HOME=0.
- **Writing `INITIALIZED = 0xFF`, `SERVOS_AVAILABLE = 24`,
  `SERVO_MULTIPLIER = 1` into RAM after REINIT** does not unfreeze
  PWM, because these aren't the gate.

## Misreadings to Avoid

The previous Claude (this session, before correction) mis-classified
several params as "broken." Here is what's actually correct on a
Mini Maestro 24:

| Param | Value | Meaning |
|---|---|---|
| `INITIALIZED` (0) | `0x00` | Normal — not a brick indicator |
| `SERVOS_AVAILABLE` (1) | `0` | Ignored on Mini Maestro (Micro-only param) |
| `SERVO_MULTIPLIER` (26) | `0` | Stored as `multiplier - 1`; 0 = effective 1× |

`Usc.cs` line 858: `multiplier = (byte)(settings.servoMultiplier - 1);`
so writes from MCC also store 0.

## When the Maestro IS Actually Broken — Diagnostic Path

If a different channel later doesn't move, **before** chasing firmware
theories:

1. **Test in MCC (Windows) first.** Plug Maestro in, open MCC, go to
   the Status tab, drag the slider for the channel in question.
   - Moves → Maestro is fine; bug is in our code or our test
     methodology. Re-check EEPROM HOME for that channel via
     `GET_PARAMETER (0x81)`; if HOME=0, it needs MCC `HomeMode=Goto`.
   - Doesn't move in MCC → it's wiring, servo power, or
     ground/common (note: servo power supply ground **must** be
     tied to the Maestro signal ground for the servos to see the
     PWM edges).
2. Only **after** MCC slider has been ruled out should you consider
   firmware reflash. Pololu's `.pgm` files are encrypted Intel HEX;
   reflashing requires MCC's "Upgrade Firmware" menu or Mono +
   Pololu Linux MCC. There is no safe path to flash from a custom
   Python script.

## SaintOS Driver Implications

Our runtime driver should only issue `SET_TARGET (0x85)` plus optional
`SET_SERVO_VARIABLE (0x84)` for speed/accel. Per-Maestro EEPROM setup
(`HomeMode = Goto`, MIN/MAX, NEUTRAL, RANGE) is a **one-time provisioning
step done in MCC** — there is no software-only equivalent we've found.

A first-boot check in `firmware/shared/src/maestro_driver.c` could read
per-channel HOME via vendor 0x81 and warn/refuse if HOME == 0 (channel
not provisioned in MCC), preventing the "I commanded a target but
nothing moved" silent failure.

## SaintOS-Side Per-Channel Config (Phase 1, landed 2026-06-17)

Each Maestro channel has its own config stored on the server in
`PeripheralInstance.params["channels"]` — a list of dicts whose length
matches `params["channel_count"]`. The fields mirror the firmware's
`maestro_channel_config_t` so no key translation is needed on the
server-to-firmware push:

| Field | Type | Meaning |
|---|---|---|
| `label` | string ≤ 32 | Operator-set channel name; falls back to `Ch N` |
| `min_pulse_us` | int 64–3200 | Lower clamp on outgoing target |
| `max_pulse_us` | int 64–3200 | Upper clamp on outgoing target |
| `neutral_us` | int (clamped to [min,max]) | Position commanded at center input |
| `home_us` | int (clamped to [min,max]) | Position sent via SET_TARGET on driver connect/init; 0 = "skip auto-home" |
| `speed` | int 0–65535 | Default Pose-transition speed (0 = unlimited). **Not pushed to chip on config save.** |
| `acceleration` | int 0–255 | Default Pose-transition accel. **Not pushed to chip on config save.** |

Distinct from the Maestro EEPROM's own MIN/MAX/HOME/HomeMode: those
are the chip-side gates that need MCC provisioning once
(`HomeMode = Goto`, HOME != 0) to enable PWM at all. After that, the
SaintOS-side per-channel config is the operator-facing source of
truth — they edit it from the channel-edit modal (click any channel
chip on a Maestro card on the Peripherals page).

Defaults for a new channel are sourced from the peripheral-level
**Advanced settings** (`min_pulse_us`, `max_pulse_us`, etc. on the
main edit modal). The Advanced section is collapsible and rarely
needs editing once the operator has set the per-channel values they
want.

### Server-side normalization

`server/saint_server/peripheral_model.maestro_normalize_channels(params)`
is called from `state_manager.upsert_node_peripheral` whenever a
Maestro is saved. It:

- Ensures `params["channels"]` is a list whose length matches
  `params["channel_count"]`
- Sanitizes each entry (clamps min/max range, swaps if reversed, clamps
  neutral/home into [min,max], bounds speed/accel)
- Backfills missing entries with `_maestro_default_channel(idx, params)`,
  which draws on the peripheral-level Advanced fields

### Animation vs Pose semantics

- **Animation / real-time control input** path: target is clamped to
  `[min_pulse_us, max_pulse_us]` and sent via `SET_TARGET` only.
  Speed/accel limits are kept at 0 (unlimited) so the channel snaps to
  the target.
- **Pose transitions** (TODO — Pose editor not yet built): a
  Pose-play path will send `SET_SERVO_VARIABLE` with the channel's
  `speed` and `acceleration` before `SET_TARGET`, then reset to 0
  after the transition settles. See TODO comments in
  `firmware/shared/src/maestro_driver.c::maestro_set_channel_config`
  and `firmware/raspberrypi/saint_node/peripherals/maestro.py::apply_config`.

### Home-on-init

Both firmware drivers (C and Pi-side Python) send `SET_TARGET = home_us`
per configured channel on the disconnected → connected transition.
Skips channels where `home_us == 0` (treated as "unconfigured"). This
is SaintOS-side home, not the Maestro EEPROM HomeMode — the chip-side
HomeMode=Goto is still a one-time MCC prerequisite for PWM to come up
at all.

### End-to-end wiring verification

The Teensy → USB host → Maestro path is fully wired:

1. **UI** (`Peripherals.vue`): clicking a channel chip opens the
   channel-edit modal; saving POSTs the patched `params.channels[i]`
   via `save_node_peripheral`.
2. **Server** (`state_manager.upsert_node_peripheral` →
   `peripheral_model.maestro_normalize_channels`): sanitizes the
   incoming entry, persists the full peripheral instance YAML.
3. **Server → firmware push**: the saved peripheral JSON arrives at
   the firmware via the existing peripheral sync path.
4. **Firmware JSON parser**
   (`firmware/shared/src/maestro_driver.c::drv_parse_json`): for each
   channel (called once per channel by `apply_one_peripheral`),
   reads peripheral-level defaults, then walks the `channels[]`
   array and applies the matching entry's overrides. Per-channel
   values populate `g_channel_configs[]`.
5. **Firmware runtime**: `maestro_set_target` clamps to the
   channel's `[min_pulse_us, max_pulse_us]`. On the disconnected →
   connected transition, `maestro_apply_home_positions` sends
   `SET_TARGET = home_us` per configured channel.

Unit-tested in `firmware/shared/tests/test_maestro_driver.c::case_parse_json_per_channel`.

### Tasks deferred for later (when re-engaging)

**Task #17 — Teensy `usb_vendor` transport.** Requires a custom
`USBDriver` subclass in firmware/teensy41/src/maestro_transport.cpp
that:
- Claims the Pololu Maestro by VID/PID (0x1FFB/0x008C) — likely
  needs to coordinate with `USBSerial_BigBuffer` (only one driver
  can own the device at a time, so transport selection at runtime
  has to instantiate one or the other, not both).
- Wraps `queue_Control_Transfer(dev, &setup, buf, this)` with a
  synchronous spin-wait on `myusb.Task()`, signaled by the driver's
  `control()` override fire on transfer completion.
- Implements the byte-stream `write` / `read` ops by translating
  outgoing Compact Protocol bytes into the equivalent vendor
  requests (0x84 Set Target Compact → 0x85 Set Target vendor; 0x87
  Set Speed → 0x84 Set Servo Variable; etc.) — mirror what the Pi
  `_PyUsbVendorBackend` does in `firmware/raspberrypi/saint_node/peripherals/maestro.py`.
- Populates `ctrl_xfer` so `maestro_read_channel_config_from_device`
  works on Teensy and the Phase-2 EEPROM-defaults readback feature
  becomes available on this transport.

Budget: see [[feedback-usbhost-t36-vendor-request-cost]] in memory —
~200-300 LOC + hardware-level testing. Not a quick afternoon job.

**Task #10 — firmware → server EEPROM defaults reporting.** Once
either Pi `usb_vendor` (already implemented) or Teensy `usb_vendor`
(task #17) is in operator use:
1. On peripheral activation, the driver calls
   `maestro_read_channel_config_from_device(channel, &out)` /
   `_PyUsbVendorBackend.read_parameter` for each channel.
2. Bundles the result into a one-shot "peripheral_announce" event
   (separate from the 10 Hz state heartbeat so the 1024-byte
   announce buffer isn't overrun).
3. Pi node: extend `node.py::_publish_announcement` to add an
   optional `peripheral_readback: {<peripheral_id>: {channels:
   [...]}}` field. Teensy: extend the announce message similarly
   via micro-ROS.
4. Server-side `state_manager` announce-handler merges the readback
   into `PeripheralInstance.params.channels` ONLY for fields not
   already set by the operator (operator-set values always win).
5. UI: the channel-edit modal shows "(from Maestro)" annotation on
   fields whose source is the readback rather than operator input.

Budget: ~100 LOC across server + Pi node, plus message-format
extension to micro-ROS for the Teensy half.

### Phase 2 (partially landed 2026-06-17)

Transport refactor and EEPROM-readback foundation. The shared
`maestro_transport_ops_t` now carries an optional `ctrl_xfer` op
populated only by transports that support EP0 control transfers
(usb_vendor on Teensy when its USBHost implementation lands; pyusb on
Pi). Three transport modes are now recognized:

- `uart` (FLASH_MAESTRO_TRANSPORT_UART = 1): Pololu Compact Protocol
  over TTL serial. Teensy uses HardwareSerial1..8; Pi uses pyserial.
  Runtime ops only.
- `usb_cdc` (FLASH_MAESTRO_TRANSPORT_USB_CDC = 0; was named USB_HOST):
  Compact Protocol over CDC ACM. Teensy uses `USBSerial_BigBuffer`;
  Pi could use pyserial against `/dev/ttyACM0` (not yet implemented).
  Requires the Maestro's Serial Mode = "USB Dual Port" in MCC.
  Runtime ops only.
- `usb_vendor` (FLASH_MAESTRO_TRANSPORT_USB_VENDOR = 2): EP0 vendor
  requests. Works in any Maestro Serial Mode. Supports
  `GET_PARAMETER (0x81)` for EEPROM readback. Pi: implemented via
  `_PyUsbVendorBackend` in `firmware/raspberrypi/saint_node/peripherals/maestro.py`
  — translates Pololu-protocol byte stream from MaestroDriver into
  vendor requests on EP0. Teensy: scaffold returns NULL; full
  implementation requires custom USBDriver subclass + sync wrapper
  for `queue_Control_Transfer` (see task #17 description, deferred).

`maestro_read_channel_config_from_device()` in
`firmware/shared/src/maestro_driver.c` reads HOME/MIN/MAX/NEUTRAL/
SPEED/ACCEL via the active transport's ctrl_xfer; returns false on
transports without it. UI catalog (peripheral_model.py) lists all
three transports plus the legacy `usb_host` alias.

**Still deferred:** firmware → server protocol to ship the EEPROM
readback up to the server (task #10), so the channel-edit modal can
show "(from Maestro)" annotated defaults instead of SaintOS-side
ones. Plumbing requires extending the ROS2 / micro-ROS peripheral
state report — non-trivial integration left for a focused session.
Until it lands, channel defaults are sourced from the peripheral-
level Advanced settings, which works fine and is fully operator-
editable.

## Reference Files

- Pololu authoritative protocol & param map:
  - `https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/protocol.h`
  - `https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/Usc/Usc_protocol.cs`
  - `https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/Usc/Usc.cs`
  - `https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/Usc/UscSettings.cs`

## Constraints

- Do not autocommit or rebuild dist (`feedback_no_autocommit_or_dist`).
- Diagnostic scripts live in `/tmp/maestro_*.py` on Mac and Pi; canonical
  copies on the Mac, scp to Pi as needed.
- Pi has no internet — pyusb wheel was pushed from the Mac
  (`/tmp/pyusb-wheel/pyusb-1.3.1-py3-none-any.whl`).
- ModemManager on the Pi grabs `/dev/ttyACM*`; `sudo systemctl stop
  ModemManager` is needed before USB vendor probes (not persistent).
