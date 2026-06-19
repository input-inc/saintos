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

**The fix:** each channel that should output PWM needs **Mode = Servo**
and **On startup or error = Go to <position>** (EEPROM `HomeMode =
Goto`, HOME = the position). After that, runtime `SET_TARGET (0x85)`
works as expected and the position field becomes honest — it ramps at
the runtime speed limit and reaches target accurately.

> **Update (2026-06-17):** this no longer has to be done in Maestro
> Control Center. SaintOS now writes these same EEPROM settings itself
> over the `usb_vendor` transport — see **Software provisioning (no
> MCC)** below. MCC remains the fallback for CDC/UART-only setups.

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
  reverted to 0 across a real power cycle, which led to the (now
  **retracted**) conclusion that "SET_PARAMETER doesn't write EEPROM —
  only MCC persists." **That conclusion was wrong.** See "Software
  provisioning" below: MCC writes every setting through the *same*
  `SET_PARAMETER (0x82)` vendor request with no separate commit step,
  and the likely reason our earlier writes didn't stick is a wire-
  format bug — the byte count must be packed into the **high byte of
  `wIndex`** (`wIndex = (bytes << 8) | param`), with the value in
  `wValue`. A SET_PARAMETER missing that byte-count is malformed.
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

## Software provisioning (no MCC) — implemented 2026-06-17

**Supersedes the earlier "MCC is the only way to persist EEPROM"
conclusion.** Per-Maestro EEPROM setup (channel `Mode = Servo`,
MIN/MAX, NEUTRAL, `HomeMode = Goto`) *can* be done from firmware over
the `usb_vendor` transport, because MCC itself does nothing special:

> Pololu's `Usc.cs` writes every configuration value through one path —
> `setRawParameterNoChecks(parameter, value, bytes)`, which issues
> `controlTransfer(0x40, REQUEST_SET_PARAMETER /*0x82*/, value,
> (bytes << 8) | parameter)`. **There is no separate "commit to
> EEPROM" / flash-write step.** SET_PARAMETER *is* the persistence; the
> parameters *are* the stored config.

So the prerequisites for a channel to emit PWM are all
`SET_PARAMETER`-writable from our driver:

| Setting | Param | Bytes | Encoding | Notes |
|---|---|---|---|---|
| Channel Mode | `CHANNEL_MODES_0_3..20_23` = 12 + ch/4 | 1 | 2 bits/ch at shift `(ch%4)*2`; Servo = 0 | read-modify-write (shared byte) |
| MIN | `SERVO0_MIN` (32) + ch·9 | 1 | qus÷64 = µs/16 | |
| MAX | `SERVO0_MAX` (33) + ch·9 | 1 | qus÷64 = µs/16 | |
| NEUTRAL | `SERVO0_NEUTRAL` (34) + ch·9 | 2 | qus (µs·4) | |
| HOME / HomeMode | `SERVO0_HOME` (30) + ch·9 | 2 | 0 = Off, 1 = Ignore, else Goto `<qus>` | non-zero HOME = `Goto` = "enabled at startup" |

Channel Mode and HOME are init parameters, so `REINITIALIZE (0x90)` is
issued after the writes for them to take effect (and to re-home each
channel) without a power cycle.

**Implementation (Teensy + RP2040, shared C):**
`maestro_provision_channel()` / `maestro_provision_all_channels()` in
`firmware/shared/src/maestro_driver.c`. Called from the connect hook
(`maestro_apply_home_positions`) whenever the active transport exposes
`ctrl_xfer` — i.e. Teensy `usb_vendor` (RP2040/UART has none, so it
no-ops there). Each parameter is
**diff-checked** — read via `GET_PARAMETER (0x81)` and written only if
it differs — so a steady-state connect does reads only and doesn't wear
the EEPROM. Only channels with `home_us > 0` are provisioned (explicit
"enable at startup" intent); channels left at `home_us == 0` are not
touched, so Input/Output channels aren't clobbered. CDC and UART leave
`ctrl_xfer` NULL — those carriers can't write parameters at all, so on
them provisioning still requires MCC.

**Pi equivalent — deferred.** The Pi runs a separate Python driver
(`MaestroDriver` + `_PyUsbVendorBackend` in
`firmware/raspberrypi/saint_node/peripherals/maestro.py`). That backend
already has `read_parameter` (GET_PARAMETER) but no provisioning write
path. Adding a `provision_channel` there — same SET_PARAMETER encoding,
diff-check, REINITIALIZE — mirrors the shared C functions and is the
outstanding Pi-side work.

⚠️ **Bench-verification still owed:** confirm a provisioned channel
survives a real power cycle (this is the exact behavior the retracted
2026-06-16 note got wrong; the fix is the corrected `wIndex` byte-count
encoding above). The runtime driver's hot path remains just
`SET_TARGET (0x85)` plus optional `SET_SERVO_VARIABLE (0x84)`.

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

### Wire-size architecture (preventing buffer overrun crashes)

Three caps stack between server and Teensy/RP2040:

| Layer | Limit | Source |
|---|---|---|
| XRCE-DDS UDP frame | **512 bytes** | `UXR_CONFIG_UDP_TRANSPORT_MTU` baked into libmicroros |
| XRCE-DDS reassembly | ~**2048 bytes** | MTU × `RMW_UXRCE_MAX_HISTORY` (4) |
| Firmware `config_buffer` | **4096 bytes** | `firmware/teensy41/src/main.cpp`, `firmware/rp2040/src/main.c` |

The smallest binding cap is the **XRCE reassembly limit ≈ 2 KB**.
Messages larger than this overrun the rmw_microxrcedds internal
buffer and crash the firmware — the original Maestro `channels[]`
regression. The firmware `config_buffer` was bumped to 4096 as a
safety net, but real survival depends on staying under the 2 KB
reassembly cap.

**Three preventatives keep us safe:**

1. **Server-side slim** —
   `peripheral_model.maestro_slim_channels_for_wire` emits only the
   per-channel fields that differ from defaults. Drops a fresh
   24-channel Maestro from ~3500 → ~440 bytes (single XRCE frame).
   Typical operator workflow with a few customized channels: ~500-1000
   bytes. See state_manager.get_firmware_config_json.
2. **Regression test in CI** —
   `server/test/test_maestro_wire_size_budget.py` asserts realistic
   configurations stay under the XRCE cap. If a future field addition
   blows the budget, the test fails before the change can ship.
3. **Runtime guard** — `get_firmware_config_json` logs a warning if
   any push exceeds the cap, so an operator-set unusual configuration
   surfaces immediately in the activity log instead of as an
   unexplained firmware crash.

If a real workflow ever legitimately needs payloads above 2 KB
(e.g., per-channel EEPROM readback echo, task #10), the right answer
is **chunked config protocol**: replace the single `configure`
message with a stream of small messages (`clear_config`,
`add_peripheral`, `set_channel`, `commit_config`). Each stays under
the XRCE single-frame MTU, total payload is unbounded. Documented as
deferred future work.

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

**Task #17 — Teensy `usb_vendor` transport. (landed 2026-06-17, awaiting
on-hardware verification.)** Implemented in
`firmware/teensy41/src/maestro_transport.cpp` as a custom `USBDriver`
subclass `MaestroVendorDriver`:
- Claims the Pololu Maestro by VID `0x1FFB` / PID in
  {0x0089 Micro6, 0x008A Mini12, 0x008B Mini18, 0x008C Mini24} at
  **claim type 0 (whole device)**. `USBSerial` also claims CDC ACM at
  type 0 (serial.cpp:92), and `claim_drivers` (enumeration.cpp:327)
  binds the *first* driver to return true at type 0. Resolved by (a)
  declaring `maestroVendor` *before* `maestroSerial` so it's offered
  first, and (b) gating `claim()` on a runtime `g_vendor_mode` flag:
  vendor mode → vendor claims, CDC never binds; CDC mode → vendor
  declines at type 0, enumeration falls through to `maestroSerial`'s
  interface claim. One image carries both transports, no library
  patch. Transport switch takes effect on next Maestro enumeration
  (re-plug / power cycle).
- Wraps `queue_Control_Transfer(dev, &setup, buf, this)` in a blocking
  `ctrl()` that spin-waits a volatile flag set by the `control()`
  override (fired from the USB ISR on completion), bounded by a
  timeout. SET_* requests are zero-data-stage (payload in
  wValue/wIndex) so they finish in microseconds; only GET_PARAMETER
  carries a data buffer (cache-invalidated for the M7 D-cache).
- `write`/`read` translate the shared core's Compact Protocol stream
  into vendor requests (0x84 Set Target → 0x85; 0x87 Set Speed → 0x84;
  0x89 Set Accel → 0x84 with channel|0x80; 0xA4 Stop Script → 0xA2
  wValue=1; 0xA2 Go Home → 0x90 Reinitialize) — mirror of the Pi
  `_PyUsbVendorBackend`. Get-Position readback over vendor is NOT
  implemented (stashes 0 → drv_get_value falls back to the commanded
  value); per-channel position requires parsing the model-specific
  GET_VARIABLES servo-status array.
- Populates `ctrl_xfer` so `maestro_read_channel_config_from_device`
  (GET_PARAMETER EEPROM readback) now works on Teensy.

Builds clean for the `hardware` target. **Still needs a bench pass on a
real Mini Maestro 24** to confirm the claim arbitration, the
control-transfer completion/timeout path, and D-cache coherency behave
as designed. Reminder: vendor `SET_PARAMETER` still does NOT persist
EEPROM (only MCC does), so a channel in `HomeMode.Off` won't emit PWM
even over vendor — provisioning remains an MCC step.

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
  vendor requests on EP0. Teensy: implemented 2026-06-17 via the
  `MaestroVendorDriver` USBDriver subclass (see task #17 above),
  awaiting on-hardware verification.

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

- Pololu authoritative protocol & param map (all verified 2026-06-17):
  - [`protocol.h`](https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/protocol.h)
    — `enum uscRequest`: SET_TARGET 0x85, SET_SERVO_VARIABLE 0x84,
    GET_PARAMETER 0x81, SET_PARAMETER 0x82, GET_VARIABLES 0x83,
    REINITIALIZE 0x90, SET_SCRIPT_DONE 0xA2.
  - [`Usc_protocol.cs`](https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/Usc/Usc_protocol.cs)
    — `uscParameter`: SERVO0_HOME 30 (stride 9: HOME +0/2B, MIN +2,
    MAX +3, NEUTRAL +4/2B, RANGE +6, SPEED +7, ACCEL +8);
    CHANNEL_MODES_0_3 = 12 (groups +0..+5 → channels 0–23).
  - [`Usc.cs`](https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/Usc/Usc.cs)
    — `setRawParameterNoChecks`: writes are `controlTransfer(0x40,
    SET_PARAMETER, value, (bytes<<8)|param)` with **no separate EEPROM
    commit**; channel mode packing `(byte[ch>>2] >> ((ch&3)<<1)) & 3`
    (2 bits/ch); HOME encodes Off=0 / Ignore=1 / Goto=position; PIDs
    0x0089/8A/8B/8C = Micro6 / Mini12 / Mini18 / Mini24, VID 0x1FFB.
  - [`UscSettings.cs`](https://raw.githubusercontent.com/pololu/pololu-usb-sdk/master/Maestro/Usc/UscSettings.cs)
    — settings model (HomeMode / ChannelMode enums).

## Constraints

- Do not autocommit or rebuild dist (`feedback_no_autocommit_or_dist`).
- Diagnostic scripts live in `/tmp/maestro_*.py` on Mac and Pi; canonical
  copies on the Mac, scp to Pi as needed.
- Pi has no internet — pyusb wheel was pushed from the Mac
  (`/tmp/pyusb-wheel/pyusb-1.3.1-py3-none-any.whl`).
- ModemManager on the Pi grabs `/dev/ttyACM*`; `sudo systemctl stop
  ModemManager` is needed before USB vendor probes (not persistent).
