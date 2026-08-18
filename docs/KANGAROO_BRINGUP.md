# Kangaroo x2 + SyRen 25 — Linear Actuator Bringup

## Status: Design verified, implementation in progress (2026-08-14)

Protocol facts below are verified against primary sources (see
[Reference Files](#reference-files)). The panel-driven teach tune described
in "SaintOS Design Decisions" is not yet built.

## The Rig

One DC **linear actuator** driven by a SyRen 25 power stage with a Kangaroo x2
riding on it. Feedback is a **potentiometer** — absolute, not incremental.
Packet serial, address 128, channel `1`, independent mode.

Because the pot is absolute, the Kangaroo knows its position the instant it
powers on. Nothing needs to be sought out, so **the actuator must not move at
startup.** That single requirement drives most of what follows.

## DIP Switches (verified)

**Kangaroo x2** — ON means the toggle is pushed toward the "ON" marking on the
switch body, OFF means toward the numbers.

| 1 | 2 | 3 | 4 |
|---|---|---|---|
| ON | OFF | ON | ON |

Packet serial input · analog (pot) feedback · position control · independent mode.

**SyRen 25**

| 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|
| OFF | OFF | ON | ON | ON | ON |

Packetized serial · lithium cutoff disabled · address 128.

Notes:

- The SyRen needs its own setup; the Kangaroo's DIP switches configure only the
  Kangaroo. Kangaroo manual p.20: *"Set the motor driver to packet serial mode
  at address 128 as shown."*
- **Set both boards before tuning.** Changing Kangaroo switch **2 or 4** after a
  tune disables the motors until you retune. Switches 1 and 3 can be changed
  freely.
- SyRen switch 3 (lithium cutoff) stays **ON/disabled** on this rig: the pack is
  8S LiFePO4 with its own BMSes, and SyRen's lithium mode assumes ~3.7 V/cell
  Li-ion, so its automatic cell-count detection miscounts LiFePO4 and lands on a
  wrong cutoff in either direction.
- Baud rate needs no setting — in packetized serial the SyRen auto-detects the
  rate from the first character.
- The Kangaroo mounts into the SyRen's terminal block. Since SyRen is
  single-channel, use only the Kangaroo side labeled **1**. The microcontroller's
  TX goes to Kangaroo **S1**, RX to **S2** — not to the SyRen.

## Why Mode 1 (Teach), Not Mode 2 (Limit Switch)

A Mode 2 tune **homes on startup, by design and with no opt-out.** Kangaroo
manual p.7: *"the device will automatically home to one of the limit switches
before operating."* That is disqualifying here.

Mode 1 Teach is also the better fit on the merits — manual p.28 states that
positioning-before-power-up and homing are *not* necessary with analog feedback,
and p.8 calls teach tune *"especially useful when you are using a potentiometer
for feedback."*

The trade-off, stated plainly (p.7): *"You must do a Limit Switch Tune (Mode 2)
in order to tell the device you are using limit switches."* **With a Teach tune,
L1/L2 are ignored entirely.** Travel limits come from the taught pot endpoints
plus soft limits in DEScribe.

## Position Sensors — IDC PSR-2 (measured 2026-08-14)

The end-of-travel sensors on the N2 electric cylinder are **PSR-2**: mechanical
reed, contact closure, **normally closed**, red LED, 2 leads + shield, 26 AWG,
3 m, rated 4-120 V AC or DC, 50 mA / 6 W max output.

Two corrections to earlier assumptions, both verified on the bench:

- **PSR-2 is normally CLOSED, not normally open.** PSR-**1** is the
  normally-open variant (green LED); PSR-**2** is normally closed (red LED).
  Same reed, same output type, opposite connection. A continuity check reading
  closed at rest and open with a magnet present is the switch working
  *correctly* — the expectation was inverted, not the hardware.
- **There are diodes in series with the contact, and they are anti-parallel.**
  Diode-test mode reads ≈1.9 V at rest — a red LED's forward drop, not contact
  resistance; a bare dry reed would read near 0 Ω. Per the IDC wiring diagram
  the sensor is a reed contact in series with an **anti-parallel diode pair**,
  which is why the part is rated for AC as well as DC. **The sensor is therefore
  NOT polarity-sensitive** — ~1.9 V reads the same in both lead orientations,
  and it can be wired either way.

### Interfacing: ADC pin, NOT a digital pin

**A digital input cannot read this sensor at 3.3 V.** The ~1.9 V diode drop
lands in the forbidden zone between the logic thresholds:

| | valid LOW below | valid HIGH above |
|---|---|---|
| RP2040 (0.35 / 0.65 × IOVDD) | 1.16 V | 2.15 V |
| Teensy 4.1 (i.MX RT1062) | 0.99 V | 2.31 V |

No pull-up value fixes this, because the level is set by the diode, not the
resistor — 50 kΩ gives ≈1.6 V, 1 kΩ gives ≈1.8 V, both still undefined. Driving
one GPIO high and reading the sensor on another fails for the same reason. This
is arithmetic, not margin.

**The interface in use is an analog read with a software threshold**, which
needs one resistor and no active parts:

```
node 3.3V ──[10k]──┬── ADC pin
                   │
              [Brown] PSR-2 [Blue] ── node GND
```

| State | Reed | ADC pin |
|---|---|---|
| At rest, mid-travel | closed | ≈1.7 V |
| Magnet present (at limit) | open | 3.3 V |
| Broken wire / unplugged | open | 3.3 V |

Threshold in firmware at **2.5 V** — a ~1.6 V separation, enormous for a 12-bit
ADC, and well clear of the diode's ≈−2 mV/°C drift. Note the polarity: the
sensor is normally closed, so **high = tripped**, and a severed cable also reads
tripped, which is the correct fail-safe direction.

Cable: **Brown** and **Blue** are interchangeable (anti-parallel diodes).
**Shield** to ground at the controller end only. Black is unused on every PSR
reed variant.

Mechanical reeds bounce for roughly 0.5-2 ms on closure, so the firmware
debounces before latching.

### Hardening path (not currently built)

The analog read has no isolation: 3 m of sensor cable runs alongside the motor
leads of a regenerative driver and lands directly on an MCU pin. If cable pickup
proves troublesome, or for anything beyond a bench rig, move to the
optocoupler topology — which is what the IDC wiring diagram itself shows on the
controller side, so it is the manufacturer's reference design rather than a
workaround:

```
24V ── R(2.2k) ── [Brown] PSR-2 [Blue] ── opto LED ── 24V GND
                                       opto transistor ── node GPIO (10k pull-up)
```

Budget: 24 V − 1.9 V (sensor) − 1.2 V (opto LED) ≈ 20.9 V across R; 2.2 kΩ ½ W
gives ≈9.5 mA, far under the sensor's 50 mA / 6 W rating and ample for a PC817.
Keep the 24 V return and node ground separate — that separation is the entire
point. This frees the ADC pin and turns the input back into a plain digital
read.

### Why they still don't go to the Kangaroo's L1/L2

Normally-closed is in fact what the Kangaroo wants — but it wants a contact that
opens at end-of-travel and *stays* open. Whether these do depends entirely on
sensor placement: mounted at true end of travel the piston magnet parks adjacent
and the contact holds open; mounted anywhere inboard, the magnet sweeps past and
the contact recloses behind it — a **pulse**, which the Kangaroo reads as "back
in range" and drives straight through. That is the original run-past symptom.

Independently, a Mode 1 Teach tune ignores L1/L2 entirely, and Mode 2 is ruled
out because it homes on startup. So the sensors go to **node GPIO** — see
SaintOS Design Decisions below. Firmware latches the input regardless of
placement, which makes the pulse-vs-state distinction moot.

## Packet Serial — Verified Protocol Reference

Verified against the *Kangaroo Packet Serial Reference Manual* (Dimension
Engineering, 2022), cross-checked against the Arduino library's
`KangarooSystemCommand` and `KangarooGetType` enums. Both agree.

### Packet layout (p.5)

`Address (1) · Command (1) · Data Length (1) · Data (n) · CRC (2)`

Address bytes have the high bit set. The CRC is 14-bit over address (excluding
its high bit), command, length, and data — lower 7 bits into the first CRC byte,
upper 7 into the second. Init `0x3fff`, poly `0x22f0` (0x21E8 Koopman), final
XOR `0x3fff`.

Bit-packed numbers: positive doubled; negative made absolute, doubled, then `|1`.
Six bits per byte, low bits first, bit 7 set when more bytes follow. Valid range
−(2²⁹−1) … 2²⁹−1.

### Commands

| # | Command |
|---|---|
| 32 | Start |
| 33 | Units |
| 34 | Home |
| 35 | Get |
| 36 | Move |
| 37 | System |
| 67 | Get/Status reply opcode |

### Get parameters (p.11)

| Param | Meaning |
|---|---|
| 1 | Current position |
| 2 | Current speed |
| 8 | Minimum position (DEScribe Nominal Travel min) |
| 9 | Maximum position (DEScribe Nominal Travel max) |

Add 64 for incremental. Request flags: 16 echo code, 32 raw units, 64 sequence
code. **Params 8 and 9 are read-only — there is no command to write them.**

### System sub-commands (pp.13-15)

Data layout is Channel Name (1) · Flags (1) · [Sequence Code] · **System Command
Number (1)** · parameters as bit-packed numbers.

| Sub | Name | Parameters |
|---|---|---|
| 0 | Power Down | none |
| 1 | Power Down All | none |
| 3 | Enter Mode | Tune Mode: 1 Teach, 2 Limit Switches, 3 Mechanical Stops |
| 4 | Go | none |
| 5 | Abort | none |
| 6 | Control Open Loop | Power, **−(2²⁸−1) … 2²⁸−1** |
| 8 | Set Disabled Channels | bitmask; **0 enables all** |
| 32 | Set Baud Rate | 0/1/2/3 = 9600/19200/38400/115200 |
| 33 | Set Serial Timeout | 1/16 s units; 0 = DEScribe setting, −1 disables |

### Get reply error codes (p.12)

| Code | Meaning |
|---|---|
| 0 | No error |
| 1 | Channel not started — send `Start` |
| 2 | Channel needs homing — send `Home` |
| 3 | Control error — send `Start` to clear |
| 4 | Wrong mode (e.g. DIPs say mixed, tune was independent) |
| 5 | Unknown parameter |
| 6 | Serial timeout, or TX disconnected — send `Start` to clear |

## Gotchas

Three things here are easy to get wrong and each produces a real failure:

- **Control Open Loop clamps at 2²⁸−1 (268435455), not the bit-packer's 2²⁹−1
  (536870911).** Open Loop has its own narrower range. Reusing the bit-pack
  maximum — the obvious move, since `KANGAROO_BITPACK_MAX` already exists in
  `firmware/shared/include/kangaroo_protocol.h` — lets you command **double** the
  intended jog power on the one operation that has no feedback and no limits.
- **Do not use sequence codes around tuning.** Manual p.12: *"Tuning commands may
  have unusual effects on sequence code… These effects are not necessarily
  limited to the channel being commanded."* Our driver sends `flags = 0`
  everywhere. Keep it that way.
- **Two different error numberings exist.** The Get reply codes above are *not*
  the LED blink codes in the main manual (`1` wiring, `2` range, `3` control,
  `4` wrong mode, `5` aborted, `6` limit switch, `7` index). The `error_status`
  channel carries the **Get** codes. Do not decode one with the other's table.

Also worth knowing:

- **Tuning has an automatic serial timeout.** Manual p.14: *"You must continually
  send packets or it will abort. Get commands in a loop will do the job."* On a
  slow linear actuator the tune cycle runs minutes.
- **Open loop means open loop.** No feedback, no limits, no protection — it
  drives into the hard ends if nothing stops it.
- **Jog sign vs. direction is unknown before the first tune.** Start with a small
  positive power for a fraction of a second and observe.
- In serial mode you can command **beyond** the taught travel. A teach tune does
  not hard-stop you at the ends the way a Mode 2 tune would. Set soft limits in
  DEScribe once the numbers are known.

## Teach Tune Over Packet Serial

The whole procedure without touching the Autotune button (p.13: *"useful for
systems where the Kangaroo is physically inaccessible"*). All are System (37):

1. **Enter Mode** (sub 3), mode **1** = Teach.
2. **Set Disabled Channels** (sub 8), bitmask **0**. Not optional — *"Initially
   all channels are disabled for safety reasons after entering a tune mode."*
3. **Control Open Loop** (sub 6) to jog. Drive to the retracted endpoint, then
   the extended endpoint, then back to center, sending power 0 to stop at each.
4. **Go** (sub 4) to begin the tune cycle. Keep sending packets throughout.
5. **Abort** (sub 5) is the software e-stop.

Then **power cycle** — the tune is not active until you do.

Verify with `Start`, then `Get` 1 / 8 / 9 and confirm the endpoints match what
was taught. Command small moves before large ones.

### Endpoints cannot be typed

There is no command to write min/max. Endpoints come from **where the actuator
was jogged during the teach**, and are read back with Get 8/9. Setting them
numerically is DEScribe's job. This is the inverse of the Maestro servo flow
(see `MAESTRO_BRINGUP.md`), where extents are typed in µs and the servo follows —
do not carry that mental model over.

### First power-up after a tune is the runaway window

Hand on the power. If the pot reads backward relative to motor direction, the
system runs away instead of holding. Recovery: cut power, **hold the tune button
while applying power** (this is what keeps it from running), swap the **5V and B**
wires on the pot, and retune.

## SaintOS Design Decisions

- **The firmware owns the tune, not the UI.** The keep-alive must survive for
  minutes; a browser across server → micro-ROS → firmware cannot be the thing
  holding it up. Same for the dead-man (jog power decays to zero without a fresh
  command) and the open-loop power cap.
- **Do not disable the serial timeout** with System 33 / −1. It is tempting since
  it removes the keep-alive requirement, but it applies to *all channels on the
  controller* and deletes the last dead-man that still works when our own
  firmware wedges.
- **Jog rides `/control`, not `/command`.** Discrete transitions (enter, mark, go,
  abort) are one-shots and belong on the RELIABLE command topic. A continuous jog
  stream through a reliable depth-8 queue backlogs it — the same stale-setpoint
  failure documented in the deadstick investigation. Jog goes over `set_channel`:
  BEST_EFFORT, depth 1, newest-wins.
- **Limit switches come into node GPIO, latched, through an optocoupler.** The
  Kangaroo cannot report them (Mode 1 ignores L1/L2, and no Get parameter
  exposes limit state). Firmware latches the input — a momentary trip is gone by
  the next poll, and latching also makes the sensor's placement irrelevant — then
  powers down the channel via System 0. Read through the existing
  `pin_control_read_digital()`. **The PSR-2 cannot drive a 3.3 V pin directly**;
  see Position Sensors above. Note the sensors are normally closed, so the
  firmware's asserted state is contact **open** — the polarity is inverted
  relative to a typical normally-open input, and a severed cable reads as
  tripped, which is the correct fail-safe direction.
- **Home is a recall position; power-on is opt-in and defaults off.** The
  Kangaroo has no matching concept, and the whole point of this configuration is
  that nothing moves at startup.

## SaintOS Configuration

Set **Motion type = Linear actuator** on the peripheral to reveal the tune
group. Rotational is the default, so existing configs are untouched.

| Param | Default | Notes |
|---|---|---|
| `motion_mode` | `rotational` | `linear` enables the teach-tune workflow |
| `jog_power_pct` | 10 | Open-loop power cap. Low on purpose |
| `home_position` | 0 | Recall target. Never commanded automatically |
| `power_on_enabled` | **false** | Leave off — the pot is absolute, nothing needs to move at power-up |
| `power_on_position` | 0 | Only used when the above is on |

Only `jog_power_pct` crosses the wire. The other four are acted on
server-side, and `kangaroo_slim_params_for_wire` strips them from the
config push — the power cap stays in firmware because a safety limit
should not depend on the server scaling correctly.

**The wire budget is tight.** `KANGAROO_MAX_UNITS` is 8, and eight
fully-customized units serialize to ~1.8 KB against the ~2048-byte XRCE
reassembly cap — about 25 bytes per unit of headroom, less than one
average param. Before adding another wire-bound field, extend
`kangaroo_slim_params_for_wire` to drop default-valued params too.
`server/test/test_kangaroo_wire_size_budget.py` fails CI if this
regresses. Over the cap the firmware crashes rather than rejecting the
push, so this is a hard ceiling.

## Running a Tune from the Dashboard

Set **Motion type = Linear actuator**, save, then press the **tune** icon on
the peripheral's row in the Peripherals tab. The modal walks the sequence:

1. **Enter tune mode** — sends Enter Mode 1 and clears the Kangaroo's
   post-entry safety interlock.
2. **Jog** — press-and-hold Retract / Extend. Releasing stops immediately;
   the firmware dead-man is the backstop for a crash or dropped link, not
   the normal stop path. Tap briefly first — direction is unknown until the
   first tune.
3. **Mark retract / extend / centre** — a checklist for the operator. The
   Kangaroo watches the potentiometer itself; these just make it obvious
   whether both ends were actually visited, which is the usual cause of a
   failed tune. Go stays disabled until all three are marked.
4. **Start tune** — stand clear. Minutes on a slow actuator.
5. **Power cycle**, then **Read taught travel**.

**Abort is available at every stage**, including while the Kangaroo is
running its own cycle.

The Live tab's Kangaroo card shows position, speed, decoded error codes and
the tune state while this runs.

### Driving it without the UI

Useful for scripting or bench debugging — the same commands the modal sends:

```json
{"type":"control","action":"peripheral_command",
 "node_id":"<node>","peripheral_id":"<your kangaroo id>",
 "command":"tune_enter","args":{}}
```

(`peripheral_command` is a **control** action — `_handle_control` in
`websocket_handler.py` — not a management one.)

Commands: `tune_enter`, `tune_jog` (`{"power": -1..1}`), `tune_go`,
`tune_abort`, `tune_read_extents`.

Jogging is better done on the **`jog` channel** than via `tune_jog` —
same effect, but it rides `/control` (best-effort, newest-wins) instead
of the reliable command queue. See the channel comment in
`kangaroo_protocol.h` for why that distinction matters to the dead-man.

Order matters and the firmware enforces it: `tune_enter` first (it also
clears the Kangaroo's post-entry safety interlock), jog to each end and
then centre, then `tune_go`. Watch `/log` — every transition, the
dead-man, and every refusal is logged with its reason.

After `tune_go` reports done: **power cycle**, then `tune_read_extents`.
The taught limits appear on the `taught_min` / `taught_max` channels.

## Reference Files — dashboard

- `server/web/src/components/peripherals/KangarooTuneModal.vue` — tune workflow
- `server/web/src/components/peripherals/LinearTravelControl.vue` — travel readout
- `server/web/src/components/peripherals/KangarooCard.vue` — Live tab card
- `server/saint_server/webserver/state_manager.py` — `_FIRMWARE_CHANNEL_MAP`
  maps virtual GPIOs to channel ids. **Its stride must equal
  `KANGAROO_CHANNELS_PER_UNIT`** — get it wrong and unit 1's channels
  silently decode as unit 0's. Guarded by
  `test_kangaroo_wire_size_budget.py::test_channel_stride_matches_firmware_header`.

## Reference Files

- `firmware/shared/include/kangaroo_protocol.h` — packet builders, CRC-14, bit-pack
- `firmware/shared/src/kangaroo_driver.c` — channel state, peripheral_driver glue
- `firmware/shared/include/kangaroo_transport.h` — per-platform UART adapter
- `firmware/rp2040/tests/test_kangaroo_driver.c` — driver unit tests
- `server/saint_server/peripheral_model.py` — `kangaroo` catalog entry
- [Kangaroo Packet Serial Reference](https://www.dimensionengineering.com/datasheets/KangarooPacketSerialReference.pdf)
- [Kangaroo Arduino Library — KangarooChannel](https://www.dimensionengineering.com/software/KangarooArduinoLibrary/html/class_kangaroo_channel.html)
- [SyRen 10/25 User's Guide](https://www.dimensionengineering.com/datasheets/SyRen10-25.pdf)
- [DEScribe](https://www.dimensionengineering.com/info/describe)

## Constraints

- Firmware changes need a **reflash**. Server changes need a dist build +
  install — no hot-patching the Pi.
- Config push reassembly is capped at ~2 KB (MTU 512 × MAX_HISTORY 4). New
  catalog params must stay slim on the wire; past the cap the firmware crashes.
- Bench-test with the actuator mechanically disconnected or in free travel
  before anything runs under load.
