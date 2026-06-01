# Sync-config regression — firmware investigation handoff

## Symptom

`ros2 topic echo /saint/nodes/<id>/log` shows **nothing** when the operator
clicks "Sync" in the web UI, and the UI's sync-status pill never leaves
"pending". The server side is fine — it publishes the config JSON to
`/saint/nodes/<id>/config` (confirmed by external `ros2 topic echo` on
that topic). The firmware never logs `Config received`, so we know the
firmware-side `config_subscription_callback` at
`firmware/rp2040/src/main.c:353` is not being invoked, even though:

- `ros2 topic info -v /saint/nodes/<id>/config` shows both endpoints
  matched (publisher + subscriber, RELIABLE QoS).
- `/saint/nodes/<id>/command` works end-to-end — pressing **Identify**
  in the web UI makes the node blink. Both subscriptions are
  `rclc_subscription_init_default` (RELIABLE) and live in the same
  rclc executor.
- The firmware's `/log` publisher works at boot — `saint_log_drain_boot_queue()`
  empties the boot queue and the lines reach the DDS graph. So the
  outbound path is healthy.

Conclusion: only the `/config` subscription's executor dispatch is
broken. The callback never runs.

## Reproduction

The hardware-targeted test is:

```bash
python3 firmware/simulation/test_sync_recovery.py \
    --ws-url ws://opensaint.local/api/ws \
    --existing-node --node-id rp2040_585783812c33
```

Exits 1 in ~13 s with
`"firmware never logged 'Config received' within 10.0s"`.

A Renode-based reproduction is also being brought up (see the
docker e2e harness under `firmware/simulation/docker/`).

## Known-working baseline

Commit `6cf4c43` ("Streaming-control deadstick fix") was the last
commit where the operator confirmed sync worked. The diff from that
commit to HEAD is small — five candidate commits, all on the RP2040
firmware path:

| Commit  | Subject                                                        |
|---------|----------------------------------------------------------------|
| bc310cb | Adjustments to ROS posing, the node, and controller auto-update — **adds `/command` subscription, bumps executor handles 4→5** |
| a098df6 | Added more testing infra and removed the duplicate reset functionality |
| bb42a73 | Firmware size issue (announcement buffer 512 → 1024)           |
| b56487c | RP5 firmware stuff and bugfixes to web server                  |
| a6ecb8c | Added Tic and TMC2208 drivers (`peripheral_register` calls)    |
| 27c73da | **Refactor of the hardware drivers — shared sources, saint_log moved to shared module** |

Likeliest culprits in priority order:

1. **`bc310cb`** — adds `/command` sub; same QoS as `/config`. Before
   this, the executor had only `/config` (RELIABLE) and `/control`
   (BEST_EFFORT); after, it also has `/command` (RELIABLE). The
   executor handle count went from 4 to 5. Adding a second
   simultaneous RELIABLE subscription is the most plausible trigger
   for "RELIABLE sub stops dispatching" symptoms.
2. **`27c73da`** — driver consolidation. Big surface area; could
   have shifted memory layout / static-init ordering / interrupt
   priorities in a way that starves `/config` of executor cycles.
3. **`a6ecb8c`** — adds peripheral_register calls for Tic + TMC2208,
   which could change driver init timing.

## Proposed experiments (smallest change first)

### Experiment 1 — Drop `/command` subscription, route identify through `/control`

Reverts the architecture of `bc310cb` while keeping all other changes.
`/control` was the universal one-shot channel before; the firmware's
`control_subscription_callback` already had the action-dispatch
switch (factory_reset / firmware_update / identify / estop) that
`dispatch_action_buffer` now handles. The diff is roughly:

- Remove `command_sub`, `command_msg`, `command_buffer` globals at
  `firmware/rp2040/src/main.c:217`, `:241`, `:243`.
- Remove the `/command` subscription creation block at `main.c:1034`.
- Remove the third `rclc_executor_add_subscription` call at
  `main.c:1178`.
- Drop the executor capacity back to 4:
  `rclc_executor_init(..., 4, ...)` at `main.c:1131`.
- In `control_subscription_callback` at `main.c:744`, restore the
  call to `dispatch_action_buffer` (already exists at `:750`).
- Remove `rcl_subscription_fini(&command_sub, ...)` at `main.c:1199`.
- Update the server side to publish operator one-shots
  (identify / restart / factory_reset) to `/control` again instead
  of `/command`. See `server/saint_server/server_node.py:947`
  (`send_identify_command` uses `/command` — switch back to
  `/control`).

If `/config` now dispatches correctly with this single revert, we've
proven the regression is the "second RELIABLE sub" change. The fix
then becomes a real fix in micro-ROS executor/agent (or finding why
2 RELIABLE subs misbehave for this specific firmware build), not a
git revert.

### Experiment 2 — Keep `/command`, bump executor capacity above the published cap

Try `rclc_executor_init(..., 8, ...)` at `main.c:1131`. The
micro-XRCE-DDS limits in
`firmware/rp2040/lib/micro_ros_raspberrypi_pico_sdk/libmicroros/include/rmw_microxrcedds_c/config.h`
are `RMW_UXRCE_MAX_SUBSCRIPTIONS=5` and `RMW_UXRCE_MAX_PUBLISHERS=10`,
which we're not exceeding (3 subs, 3 pubs). But the rclc executor
itself has internal arrays sized at init; if there's an off-by-one
somewhere when handle count exactly equals the published cap,
bumping it would dodge it. Cheap experiment.

### Experiment 3 — Bisect across the 5 candidate commits

`git bisect` between `6cf4c43` (good) and `HEAD` (bad), running the
hardware test as the oracle at each step. Each bisect step needs a
firmware flash. ~3 iterations to pinpoint.

### Experiment 4 — Look for static-initializer / global ordering changes in `27c73da`

The driver consolidation moved `saint_log_publish` to
`firmware/shared/src/saint_log.c` and introduced platform-specific
`saint_log_emit_ros` / `saint_log_emit_local` hooks in
`firmware/rp2040/src/main.c:325-343`. If any global / static in
`saint_log.c` or in the shared driver sources ends up at an address
that collides with `config_msg.data.data`'s backing buffer
(`config_buffer[2048]` at `main.c:237`), the subscription's
message would be silently corrupted on every receive. Worth a
careful read of the link map (`saint_node.elf.map`) for adjacent
globals to `config_buffer` / `config_msg`.

## How to validate any fix

Run the harness — it must go from exit 1 → exit 0:

```bash
python3 firmware/simulation/test_sync_recovery.py \
    --ws-url ws://opensaint.local/api/ws \
    --existing-node --node-id rp2040_585783812c33
```

Expected green path output:

```
[phase 2] ✓ syncing initial config (publisher pre-create check)
[phase 2] ✓ firmware acknowledged Sync to Node
[phase 2] ✓ config persisted to flash
[phase 3] ✓ skipped (existing hardware — no reset)
=== all phases passed ===
```

## Architectural note (out of scope for the fix)

The sync-ack mechanism is currently
"firmware publishes `Config saved to flash` to `/log` → server's
`_on_node_log` text-scans the message → `mark_node_synced(success=True)`
→ `_broadcast_sync_status` → UI pill updates" — see
`server/saint_server/server_node.py:765-815` and the commit message
of `60f9137` for why it works this way (the RP2040 firmware doesn't
publish to `/capabilities` like the Pi 5 firmware does). Any fix
that restores `/config` callback dispatch will resurrect this whole
chain automatically; no separate work needed on the ack pathway.
