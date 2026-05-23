# RP2040 firmware — do-not-undo notes

Hard-won design choices that were each the result of a multi-hour
diagnostic session. Removing or "cleaning up" any of these without
understanding *why* will re-introduce a known regression. Every entry
names the commit so you can read the original investigation.

## 1. CRC is required on read requests (RoboClaw)
**File:** `src/roboclaw_driver.c` — `send_command()`
**Don't:** "match the BasicMicro Arduino library" by stripping CRC off
read commands like GETVERSION / GETTEMP / GETM1ENC.
**Why:** BasicMicro Solo 60A firmware v4.4.8 silently drops 2-byte read
requests. The User Manual prescribes `[Address][Command][CRC16]` for
reads; the Arduino library happens to work on older firmware that's
more permissive. Confirmed empirically: stripping CRC (commit
`ee9c035`) killed all read traffic; restoring it (commit `4e46602`)
brought it back.
**Test:** A successful read returns `resp=N ok` in the wire stats. A
silent regression looks like `resp=0 ok/0 short/0 crc_bad`.

## 2. probe_unit must validate CRC on the version response
**File:** `src/roboclaw_driver.c` — `probe_unit()`
**Don't:** "trust the null terminator" and skip CRC. The lenient
parser (read-until-null + 2 bytes) will accept any byte stream that
happens to contain a 0, including stale orphans from a previous broken
exchange.
**Why:** Without the CRC check, a corrupted link reports the unit as
"connected" with a plausible version string, hiding the real failure
behind a false-positive probe. Commit `4e46602`.

## 3. `peripheral_init_all` is BEFORE the operator has signalled which
peripherals exist — drivers must stay dormant unless configured
**Files:** `src/pathfinder_bms_driver.c` — `bms_drv_init()`,
`src/fas100_driver.c` — `fas100_drv_init()`.
**Don't:** Make `drv_init` unconditionally call the driver's hardware
`*_init()` function. The init sequences call `gpio_set_function(pin,
GPIO_FUNC_UART)` on their *default* pins, which on this hardware
overlap with another peripheral's pads:
- `BMS_DEFAULT_TX_PIN=0`, `BMS_DEFAULT_RX_PIN=1` — same as RoboClaw's PIO UART
- `FAS100_DEFAULT_TX_PIN=0`, `FAS100_DEFAULT_RX_PIN=1` — same
**Why:** `peripheral_init_all()` runs *after* `pin_config_load →
drv_load → roboclaw_init` has bound PIO on GP0/GP1 and probed
successfully. Any later unconditional `gpio_set_function(0/1,
GPIO_FUNC_UART)` re-muxes those pads off the PIO function and the
RoboClaw link goes silent for the rest of the boot. Commit `b64d115`.
**Pattern to follow:** `drv_init` is a no-op. `drv_load` calls the
hardware init only when `storage->{driver}_config.enabled == 1`.
`drv_apply_config` calls it on the fresh-JSON-sync path. FAS100 has
been this way since first landing; BMS was fixed in `b64d115`.

## 4. SyRen has the same bug class but is not fixed
**File:** `src/syren_driver.c` — `syren_drv_init()`
SyRen's defaults are `GP4/GP5` (not GP0/GP1), so it doesn't collide
with RoboClaw on our current PCB layout. But the unconditional
`syren_init()` call from `drv_init` follows the bad pattern. **If you
change SyRen's default pins or the PCB pin map, port the fix from item
3 first.** An attempt to do this in the same `b64d115` cycle was
reverted because `syren_init()` resets `channel_configs` to defaults
on every call, so it can't be idempotently called from
`drv_apply_config` without redesign. To fix properly: split
`syren_init` into "UART bind" and "channel defaults" so the channel
half doesn't run on subsequent calls.

## 5. `pin_config_apply_hardware` must not clobber PIO function on UART_TX/RX entries
**File:** `src/pin_config.c` — `pin_config_apply_hardware()`, the
`PIN_MODE_UART_TX/RX` case.
**Don't:** Remove the `gpio_get_function() != GPIO_FUNC_PIO0/PIO1`
guard. Legacy flash configs (pre peripheral-first JSON sync) can have
GP0/GP1 entries with `PIN_MODE_UART_TX/RX`. Apply-hardware runs after
`drv_load → roboclaw_init` has muxed those pads to PIO; setting them
back to `GPIO_FUNC_UART` here silently breaks the link until the next
sync. Commit `77ef433`.

## 6. `roboclaw_init` must verify the actual GPIO function on the fast-path
**File:** `src/roboclaw_driver.c` — `roboclaw_init()` idempotency check
**Don't:** Strip the `gpio_get_function() == expected_fn` cross-check
to "save a register read." The internal `active_*` tracking can lie:
something in the boot or config-sync path may have called
`gpio_set_function()` on our pads after we bound them, and there's no
hook to invalidate `active_*`. Without this self-heal, the user has to
trigger a `roboclaw_debug:reconfigure` to manually force a re-bind.
**Behaviour:** When the actual function doesn't match
(`GPIO_FUNC_PIO1` for swap-mode, `GPIO_FUNC_UART` for HW), it logs a
warn naming the unexpected value (useful for finding the real
clobberer), `pio_uart_deinit()`s the stale state machines, stamps
`active_* = 0xFF`, and falls through to the full re-bind. Commit
`debd0e7`.
**Trigger paths:** boot's per-channel `apply_hardware` sweep, every
dashboard config sync, and the `roboclaw_debug:reconfigure` op. All
three end up at `roboclaw_init`.

## 7. Duty keepalive does NOT gate on `connected`
**File:** `src/roboclaw_driver.c` — `maybe_send_duty_keepalive()`
**Don't:** "Optimize" by skipping the keepalive when the unit reports
disconnected.
**Why:** `connected` tracks ACK reliability — useful for diagnostics
but the controller's serial-timeout watchdog only cares about *inbound
bytes at its RX*, not whether we hear back. On a flaky link, ACKs go
silent first, then the controller chops the motor a second later
because no further duty packets arrived. The motor cut is the safety
hazard; the connection-state log is just an observation. Commit
`ad2a8f9` and the in-code comment above the function go into more
detail.

## 8. Default values, fast-paths, and `set_defaults` interactions
**File:** `src/pin_config.c` — `pin_config_set()`
Peripheral driver modes deliberately do NOT get a `drv->set_defaults`
call from `pin_config_set` (commit `a6fd9a0`). The defaults look
identical to a fresh JSON-sync payload to the apply-config path; the
boot-reload sequence then clobbers `units[]` that `drv_load` just
restored from flash. JSON sync calls `set_defaults` explicitly right
before `parse_json_params`, so the dashboard path is unaffected.

## 9. RoboClaw debug passthrough is diagnostic infrastructure — keep it
**Files:** `src/roboclaw_driver.c` — `roboclaw_debug_handle_json()`,
`src/main.c` — `roboclaw_debug` action handler, `scripts/roboclaw_debug.py`,
server `webserver/websocket_handler.py` + `server_node.py`
roboclaw_debug ops:
- `raw` — send arbitrary bytes, read response, log timing
- `sniff` — passive listen, prove whether bytes are arriving
- `reconfigure` — rebind PIO at runtime (no OTA)

This is how every regression in items 1–6 was actually located.
Removing it because "we don't need it anymore" sets up the next
multi-hour session. Cost is ~200 lines of firmware + a small CLI;
benefit is being able to drive the wire from the host any time the
link goes weird.

## Reference: known external causes of "RoboClaw stopped working"
Before adding more firmware code, check these first:
- **BEC brown-out** — external load on the controller's 5V BEC causes
  logic brown-out during motor activity (see
  `memory/project_roboclaw_bec_brownout.md`).
- **S3 latching E-stop** — when Packet Serial mode + s3mode default to
  "E-Stop Latching", the controller ACKs duty packets with 0xFF but
  silently drops them after S3 trips. Driver drives `estop_pin` LOW at
  config-apply time to deassert; if the operator hasn't wired
  `estop_pin`, S3 floats and may latch.
- **Motion Studio "Multi-Unit Mode"** — makes S2 open-drain. The
  internal RP2040 pull-up is too weak; needs an external 1k–4.7k.
  Symptom: marginal comms, intermittent reads.
- **Motion Studio "USB to TTL Relay Mode"** — disables packet serial
  on the TTL UART (USB takes over). Solo 60A v4.4.8 has both checkboxes
  but they should both be **OFF** for our normal setup.
