# Config sync — how the server and a node agree on state

_2026-09-23. Replaces blind config pushes with a reconciliation
loop: the node says what it holds, the server sends only the
difference._

## The problem it solves

The server used to push a node's **entire** peripheral config on every
sync, and had no way to know what the node was actually holding.
`/announce` carried `node_id`, `state`, `uptime` and a save timestamp —
nothing identifying the config itself. So all of these were invisible:

- a node rebooting onto a stale flash blob
- a node coming back empty after a reflash or a flash-version bump
  (both happened on 2026-09-23)
- a config push that never landed
- a node sitting happily `ACTIVE` while holding the wrong config

The only reconcile that existed fired when an adopted node announced
`UNADOPTED`. A node that was ACTIVE and wrong was never corrected.

This is also how the Head Node's Maestro calibration was destroyed that
morning: the node came back empty, and the server had no way to tell
"this node has nothing" from "this node is fine", so it adopted the
emptiness as authoritative.

## The loop

**The node says what it has.** Every `/announce` carries `cfg_tag` — the
tag it was given with its current config, restored from flash at boot so
it survives a reboot. ~20 bytes, and `/announce` has room (275 bytes
against a ~480-byte practical budget — that budget is real, see
`docs/MAESTRO_BRINGUP.md`).

**The server observes — it does not act.** On each announce it computes
the tag its *current* config would carry and compares:

| node's `cfg_tag` | meaning | `sync_status` |
|---|---|---|
| equals expected | node is running what the dashboard holds | `synced` |
| differs | node is running something else | `pending` |
| absent | firmware predating the field; no opinion | unchanged |

**Config moves only on an explicit Sync.** Nothing on the announce path
pushes. That is deliberate and load-bearing: an operator has to be able
to edit a channel, look at it, decide against it and revert, without the
server having shipped the intermediate state to live hardware in the
second between. An earlier version of this did auto-push on mismatch,
which made every dashboard edit go live within ~1 s and left no way to
stage or discard anything.

What the tag buys, then, is not automation — it is **honesty**.
`sync_status` used to be set to "synced" at the moment the server
published, which said only that a message left the server: a push that
never landed, or a node that later rebooted onto an older blob, still
read as synced. Now "synced" means the node stated which config it
holds and it is ours.

**The tradeoff, stated plainly:** a lost push is no longer repaired
automatically. The operator has to notice and press Sync again. That is
the cost of config never moving on its own, and the truthful status
indicator is the mitigation — the condition is now visible instead of
silent.

## What the tag is

A **CRC32 over the config payload**, computed by the server and stored
verbatim by the node. Two consequences worth understanding:

- **The server is stateless about it.** It recomputes what it expects
  rather than remembering what it issued, so a server restart causes no
  resync storm and there is no second record of "what should the node
  have" to drift out of step. A second source of truth for that is
  precisely the bug this exists to detect.
- **It proves receipt, not content.** The tag says "this node received
  and applied the config the server currently intends". It does *not*
  prove the node's parsed state is byte-identical to the server's —
  that would need a canonical serializer in C matching Python exactly,
  and a canonicalization mismatch there would cause a permanent resync
  loop. The receipt covers every divergence actually observed: reboot
  onto a stale blob, wipe, reflash, dropped message.

The tag covers the **configuration**, not the server's edit counter:
`version` is excluded before hashing. It increments on every save, so
including it meant an edit followed by a revert produced byte-identical
config with a different tag, and the node read as `pending` forever
despite running exactly what the dashboard held — clearable only by a
push the operator did not need. Reverting a change has to actually
return to the same state.

A server-side encoding change (a new slimming rule, say) changes the
tag without changing the meaning, costing one extra Sync. That is
correct-by-construction: it never misses a real difference.

## Three things that are deliberately NOT a mismatch

- **`cfg_tag` absent.** Firmware predating this field says nothing;
  treating silence as "wrong" would put every un-upgraded node into a
  permanent push loop. Absent means "no opinion" and the previous
  behaviour stands. This is what makes the rollout safe with mixed
  firmware in the rig.
- **Expected tag `None`.** We have no config for the node, or we
  refused to build one because it exceeds the wire budget. Pushing
  would be meaningless or would crash the node.
- **A malformed tag.** Ignored rather than guessed at.

## Where it lives

| piece | location |
|---|---|
| tag storage | `flash_storage_data_t.reserved[0..3]`, via `flash_cfg_tag_get/set` |
| node accessors | `pin_config_cfg_tag()` / `pin_config_set_cfg_tag()` |
| parse on apply | each platform's config subscription callback |
| emit | each platform's `/announce` builder |
| tag generation | `state_manager.get_firmware_config_json` |
| expected tag | `state_manager.expected_config_tag` |
| reconcile | `server_node._maybe_reconcile_config_tag` |
| tests | `server/test/test_config_sync_tag.py` |

The tag deliberately reuses existing `reserved[]` bytes, so
`sizeof(flash_storage_data_t)` and every field offset are unchanged and
**no `FLASH_STORAGE_VERSION` bump is required**. Growing that struct is
what shifted every peripheral config and bricked both Track Drive nodes
on 2026-09-23. A blob written before this shipped reads tag 0 →
"unknown" → one push to establish it.

## Per-channel deltas (step 2, shipped)

With the tag proving what the node holds, a change confined to Maestro
channels goes out as a patch against that exact base:

```json
{"action":"patch_config","from":833831565,"to":2197497345,
 "peripheral":"maestro-1",
 "channels":{"23":{"min_pulse_us":1000,"max_pulse_us":2000,
                   "idle_disengage_ms":1000}}}
```

Measured on the Head Node: a one-channel edit costs **126 bytes instead
of 1492**, and fits in a single XRCE frame, so it never touches the
reassembly path the full push has to fight.

**The node applies it only if its current tag equals `from`.** Otherwise
it ignores the patch and keeps announcing its real tag; the server sees
the mismatch and sends a full config. That makes lost, duplicated and
reordered patches all self-healing with no handshake — verified live,
where a patch built against a tag the node had already moved past was
refused with `Config patch ignored: expects tag X, we hold Y`.

Channels are sent **whole**, not field-by-field: a field the operator
resets to its default disappears from the slimmed wire form, and a
field-level diff would silently leave the node on the old value.

`plan_config_push` is the single decision point, reached only from an
operator Sync. It falls back to a full
push whenever a patch cannot be *proven* safe:

| situation | why a patch is unsafe |
|---|---|
| node's tag ≠ our recorded base | we'd be patching unknown state |
| node reports no tag | ditto |
| nothing pushed to this node yet | no base to diff against |
| a peripheral-level param changed | not expressible as channel deltas |
| a peripheral was added or removed | ditto |
| a non-Maestro peripheral changed | no patch handler for it |
| patch would exceed `_MAX_PATCH_BYTES` | it would fragment like a full push |

Only a **full** push updates the recorded base. A patch is expressed
relative to that base and does not replace it — recording one would
make the next delta diff against a fragment.

## What the operator sees

The Peripherals tab reflects the model directly:

- **A banner when there are unsynced changes**, saying plainly that the
  node is still running its previous configuration. Staged edits used to
  be indistinguishable from live ones.
- **A Revert button**, shown only when `sync_status` is `pending` and a
  confirmed snapshot exists. It discards server-side edits and restores
  what the node is running. Nothing is published — that is what makes it
  safe to offer as an undo.
- **A modal when a sync does not complete**, offering Retry or Revert.
  It is deliberately not auto-retried: config reaches a node when the
  operator asks and not otherwise. The message says the node is still on
  its previous config, because the useful thing to know first is that
  nothing is half-applied.

"Complete" means the **node** confirmed it, not that the request
returned. The UI watches `sync_status` for the node's confirmation and
declares failure after `SYNC_CONFIRM_TIMEOUT_MS` (12 s — announcements
run at ~1 Hz and apply-plus-flash-save takes a few hundred ms). Trusting
the request's own success is exactly the assumption that hid lost pushes
twice during this work.

### The revert snapshot

`<node_id>.yaml.synced`, written when the node **confirms** a config —
its reported tag matching ours — never when we merely publish one.
Snapshotting at publish time would record an optimistic state: a push
that never landed would leave a "confirmed" snapshot the node never
received, and Revert would restore fiction.

It is also written on any confirmation where no snapshot yet exists, not
only on a status transition. A node already sitting at `synced` when
this shipped never transitions, and would otherwise have Revert
unavailable forever — which is exactly what happened on the Head Node
the first time it was deployed.

## The one remaining automatic push

A node that announces `UNADOPTED` has lost its config entirely and is
not running anything the operator could revert to. Leaving it dead until
someone notices is worse than restoring it, so that path still pushes —
but it pushes **the config the node was last running**
(`last_pushed_config`), not what the dashboard currently holds. Staged
edits stay staged; recovery is not a back door for promoting unsynced
work to live hardware. It falls back to the current config only when
there is no record (server restarted since the last push).

## What remains

3. **Chunked full push.** The baseline recovery path still has to work
   at any size, and a 24-channel Maestro with every channel
   mechanically distinct serializes to ~3258 bytes — past the
   ~2048-byte XRCE-DDS reassembly cap. Fragmenting the same document
   into sub-MTU pieces and reassembling in the node's existing
   `config_buffer` needs no driver changes, unlike the schema-level
   chunking sketched in `MAESTRO_BRINGUP.md`.
