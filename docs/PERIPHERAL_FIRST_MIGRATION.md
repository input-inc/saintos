# Peripheral-first addressing — slab GPIO retirement plan

The control / telemetry pipeline addresses peripherals by
`(peripheral_id, channel_id)` end-to-end at the operator layer. Inside
the firmware, drivers still reserve **contiguous virtual-GPIO slabs**
(MaestroDriver owns 200..223, SyRen 224..231, FAS100 232..235,
RoboClaw 236..275, JBD BMS 276+, Tic 300..347, TMC2208 348+) — a
holdover from the original pin-config-keyed model. This doc tracks
the plan to retire those slabs in favour of named channel records on
both directions of the wire.

Status as of 2026-06-15: **inbound migrated, outbound infrastructure
ready (Phase 1a), no driver migrated yet (Phase 1b queued).** Both
firmware platforms now publish a `channels[]` array alongside the
legacy `pins[]` array; the server ingests both sides through
`update_pin_actual(node_id, pins_data, channels_data)`. Drivers
remain on the GPIO-keyed path until each one opts in by implementing
`state_emit_channels` on its `peripheral_driver_t`. Held on Maestro
specifically until the parallel stable-baseline work lands.

## Why this matters

1. **Slab collisions are silent bugs.** Adding a channel to a driver
   (e.g. surfacing Maestro `connected` / `errors` diagnostics) means
   either growing its slab past the next driver's base — silently
   overlapping until something writes the wrong register — or hunting
   for a free range and updating five files in lockstep. The collision
   path is what almost slipped through during the Maestro USB-host
   bring-up (2026-06-15).
2. **The slab table is duplicated.** Firmware drivers declare
   `virtual_gpio_base` + `channel_count`; the server mirrors them in
   `_FIRMWARE_CHANNEL_MAP` (`server/saint_server/webserver/state_manager.py`).
   Any drift between the two means firmware values land at the wrong
   catalog channel id on the dashboard — typically with no error.
3. **Operator-visible channel ids are already authoritative on the
   inbound side.** Continuing to translate them back to integers on
   the outbound side is round-trip work that buys nothing operationally.
4. **New drivers shouldn't have to pick a range.** The slab model
   forces every new peripheral driver to find a free base GPIO and
   pre-commit to a max-instance count. Channel-addressed state lets a
   driver be added by declaring channels in the catalog and emitting
   `{peripheral_id, channel_id, value}` records.

## What's already done

* **Inbound (server → firmware control writes)** is peripheral-first.
  `set_channel` carries `{"peripheral": <id>, "channel": <id>, "value": <f>}`
  on the wire; firmware uses `pin_config_t.logical_name` to find the
  base GPIO for that peripheral instance, then translates
  `channel_id → offset` via a per-mode lookup
  (`channel_offset_for` on RP2040, the inline switch in
  `apply_set_channel` on Teensy). Slabs are still used as the dispatch
  arithmetic underneath, but never appear in the operator-visible
  wire format.
* **`channels_per_instance`** was added to `peripheral_driver_t` so a
  single peripheral instance claims only as many GPIOs from the slab
  as it actually uses (one RoboClaw → 5 channels, not 40). The slab
  size is now a per-driver upper bound, not a per-instance reservation.
* **Routing** (`saint_server/router/routing_evaluator.py`) speaks
  `(peripheral_id, channel_id)` natively. Wires, signals, widgets all
  index by id.

## What's still slab-keyed

* **Outbound (firmware → server state)** state JSON. Format from
  `pin_control_state_to_json` (both firmware targets):

  ```json
  {"node_id":"teensy41_A1B2C3D4","timestamp":12345,
   "pins":[
     {"gpio":200,"mode":"maestro_servo","value":1500.0,"name":"maestro-1"},
     {"gpio":201,"mode":"maestro_servo","value":1480.0,"name":"maestro-1"},
     ...
   ]}
  ```

  The server-side `_FIRMWARE_CHANNEL_MAP` (`state_manager.py`)
  translates `gpio - base` back into the catalog's channel id. This
  table is the entire reason the slab integers still leak through.
* **Per-mode channel range maps** in
  `_FIRMWARE_CHANNEL_MAP` — 1 entry per driver type, hand-maintained,
  drifts on every catalog change.
* **`virtual_gpio_base` and slab size** in every driver's `*_driver.h`
  / `*_driver.c`. Drivers still allocate pin_config entries at
  `base + channel_index`.

## The plan

### Phase 1 — outbound channel records (the main move)

Extend the firmware state JSON to carry channel-addressed records
alongside the legacy `pins` array. Each peripheral driver gains an
optional `state_to_channels(buf, len, peripheral_id)` callback that
emits its `dir: "in"` readings as `{peripheral_id, channel_id, value}`
records. The state JSON becomes:

```json
{"node_id":"...","timestamp":...,
 "pins":[ /* unchanged legacy entries — still GPIO-indexed */ ],
 "channels":[
   {"peripheral_id":"maestro-1","channel_id":"connected","value":1.0},
   {"peripheral_id":"maestro-1","channel_id":"errors","value":0.0},
   {"peripheral_id":"maestro-1","channel_id":"ch0","value":1500.0},
   ...
 ]}
```

Server ingest in `state_manager` keys `channels[]` entries directly
into `NodeRuntimeState.set_channel(peripheral_id, channel_id, value)`
— no slab math, no `_FIRMWARE_CHANNEL_MAP` lookup.

* Allowed during phase 1: a driver may emit *some* channels through
  `channels[]` and *some* through `pins[]` (the latter still feed
  the `_FIRMWARE_CHANNEL_MAP` path). This unblocks per-driver
  migration without a flag day.

### Phase 2 — migrate each driver

In dependency order (low-risk first):

| Driver         | Notes                                                                    |
|----------------|--------------------------------------------------------------------------|
| Maestro        | Migrate first — already the trigger for this work. Diagnostics first (`connected`/`errors`), then servo positions. Once all channels emit through `channels[]`, remove the `maestro_servo` entry from `_FIRMWARE_CHANNEL_MAP`. |
| SyRen          | Single channel per instance. Trivial.                                    |
| FAS100         | 4 input channels. No instance multiplexing. Easy.                        |
| RoboClaw       | 5 channels × 8 units. Most numerous; verify the per-unit logical_name lookup carries through cleanly. |
| Pathfinder BMS | Mixed dirs + many cell channels. Existing slab entry already skips the per-cell indices; this is the test that the channels[] path handles "publishes more channels than the catalog declares" gracefully. |
| Tic            | 6 channels × 8 units. Same pattern as RoboClaw.                          |
| TMC2208        | 4 channels × 4 axes. Last one to migrate.                                |

After each driver migrates, delete its entry from
`_FIRMWARE_CHANNEL_MAP`. The server falls back to a "publishes via
channels[]" assumption; any straggler `pins[]` entry with that mode
logs a warning so the regression is visible, not silent.

### Phase 3 — drop the slabs

Once every driver emits state via `channels[]`:

1. Delete `_FIRMWARE_CHANNEL_MAP` and `_firmware_channel_id()`
   (`state_manager.py`).
2. Stop emitting peripheral-channel entries through `pins[]` in
   firmware — keep `pins[]` only for *real* GPIOs (PWM, ADC,
   digital_out). The peripheral-channel records all flow through
   `channels[]`.
3. Drop `virtual_gpio_base` and the slab arithmetic in
   `peripheral_driver_t`. Each driver still tracks its own per-
   instance per-channel state internally; the framework no longer
   imposes a contiguous-GPIO model.
4. `peripheral_gpio_to_channel` (`peripheral_manager.cpp`) becomes
   dead code — delete it after auditing the inbound `set_channel`
   path replaces every caller.

This is the point where adding a 25th channel to Maestro (or 9th to
SyRen, etc.) becomes a one-file change in that driver's catalog +
emit list — no global range bookkeeping.

## Hazards (read before starting Phase 1)

* **`_FIRMWARE_CHANNEL_MAP` deletions are observable.** A driver that
  still emits to `pins[]` after its map entry is deleted will silently
  drop those readings on the server. Order each driver's migration:
  add to `channels[]` first, *verify on hardware*, then delete the
  map entry, then stop emitting to `pins[]`.
* **Catalog channel ids are the wire identifier.** Once a driver
  starts emitting `{channel_id: "foo"}`, renaming `"foo"` in the
  catalog breaks routing without a migration. Pin the names down
  before migrating; if you must rename later, accept both old and
  new for one release.
* **Slab math still survives in inbound dispatch.** Don't try to
  delete `virtual_gpio_base` until both directions are migrated. The
  inbound `set_channel` path uses `base + offset` to compute which
  pin_config entry to write — that arithmetic comes out in Phase 3.
* **`pin_runtime_value_t` is GPIO-keyed.** That's fine; it's an
  internal cache. Channel-addressed emission can still read from it
  via `find_runtime(gpio)` — the conversion happens at emit time, not
  at storage time.

## Tests

* **Server** — `server/test/test_channel_addressed_state.py` pins
  the new ingest path (`update_channels_from_firmware`,
  `update_pin_actual`'s `channels_data` kwarg, dual-ingest
  coexistence, malformed-input resilience).
* **Firmware** — `firmware/shared/tests/test_peripheral_state_emit.c`
  covers `peripheral_state_append_channel` (formatting, comma
  management, overflow) and `peripheral_state_emit_all_channels`
  (driver iteration, NULL-callback skip, multi-driver shared
  array).

Each driver migration should extend both files with the per-driver
emit assertions (a fixture that registers the driver and verifies
its expected channel records appear in the emitted JSON).

## Verification checklist (per driver)

When migrating an individual driver:

- [ ] Driver declares all `dir: "in"` channels in the server catalog
      with stable ids.
- [ ] Driver emits each of those channels through `channels[]` on
      every state tick (or whatever poll cadence the operator
      configured).
- [ ] Operator's Live Readings view shows the new channels under the
      peripheral on the node detail page.
- [ ] Routing graph can wire those channels as sources (verifies the
      server's `set_channel` ingest is keying them correctly).
- [ ] Remove the driver's entry from `_FIRMWARE_CHANNEL_MAP`.
- [ ] Confirm any straggler `pins[]` entry for that mode logs a
      warning (one tick of warnings is fine; sustained means a
      driver-side emit was missed).
- [ ] Update this doc's per-driver table with the commit that
      migrated it.

## Where this overlaps with other work

* **The peripheral_driver_t TODO comment**
  (`firmware/rp2040/src/pin_control.c`) flags that the per-driver
  `set_value(channel)` still takes an integer channel index. Phase 3
  is when that becomes a `set_value(channel_id_string)` and the
  per-mode `channel_offset_for` switches go away.
* **`channels_per_instance`** stays — it's still the per-instance
  channel count even after slabs disappear. Drivers use it to know
  how many of their declared catalog channels each instance owns.
* **The latency-reduction work** doesn't touch any of this. Latency
  fixes are in the dispatch hot path; this is about the addressing
  model. They can move independently.
