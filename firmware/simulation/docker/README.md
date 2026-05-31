# SAINT.OS end-to-end test in Docker

Runs the full `firmware/simulation/test_sync_recovery.py` flow against
a real micro-ROS agent + saint_server inside a container — useful when
the host machine doesn't have (or shouldn't have) ROS2 installed, and
for reproducible CI runs.

## What gets exercised

The same three phases the bare harness covers, but with all the
infrastructure containerised:

1. **Sim node bring-up** — Renode boots the firmware, micro-ROS session
   establishes against the agent.
2. **Publisher pre-create / Sync to Node** — driver clicks Sync, asserts
   the firmware actually receives `"Config received"` / `"Config saved
   to flash"`. Validates the DDS race fix at the layer where it matters.
3. **Auto-reconcile** — factory-resets the sim node mid-test to simulate
   a post-OTA flash wipe, watches the server detect the divergence and
   re-push the saved config, asserts the node returns to ACTIVE.

## Prerequisites

* Docker 24+ with the `compose` plugin
* About 3 GB of free disk (ROS2 jazzy base + Renode runtime + saint_os
  build cache)
* The RP2040 simulation firmware built on the host:
  ```bash
  cd firmware/rp2040 && ./build.sh sim
  cd build_sim && make install_sim
  ```
  The container reads `firmware/rp2040/install/simulation/saint_node.elf`
  via a bind mount, so you don't have to rebuild the image to pick up
  firmware changes.

## Running

From the repo root:

```bash
# Full e2e (default).
docker compose -f firmware/simulation/docker/docker-compose.e2e.yml up \
    --build --abort-on-container-exit --exit-code-from e2e

# Just the sim bring-up (no server, no DDS, no auto-reconcile assert).
# Fast iteration when you're debugging Renode itself.
MODE=smoke docker compose -f firmware/simulation/docker/docker-compose.e2e.yml up \
    --build --abort-on-container-exit --exit-code-from e2e

# Drop into a debug shell with agent + server running.
MODE=shell docker compose -f firmware/simulation/docker/docker-compose.e2e.yml run \
    --rm --service-ports e2e
```

Test logs land in `firmware/simulation/logs/` on the host (bind-mounted
out of the container) so you can post-mortem failed runs with the same
`<node_id>.uart.log` flow as the bare harness.

## Architecture

Single container, one network namespace. Renode, the micro-ROS agent,
and the saint_server all talk via `127.0.0.1`. The harness runs in the
same container and connects to `ws://localhost:9090`.

```
┌─────────────────── e2e container ────────────────────┐
│                                                       │
│   Renode (RP2040 sim)                                 │
│       │                                               │
│       │ UDP :8888                                     │
│       ▼                                               │
│   micro-ROS agent ──── DDS ──── saint_server          │
│                                       │               │
│                                       │ WS :9090      │
│                                       ▼               │
│                              test_sync_recovery.py    │
│                                                       │
└───────────────────────────────────────────────────────┘
```

### Why not a service-per-container compose?

Cross-container DDS discovery in Docker requires either
`network_mode: host` (Linux-only, won't work on Docker Desktop) or a
FastDDS / Cyclone DDS discovery-server profile. For a test fixture
that's spinning up + tearing down whole stacks per CI run, one
network namespace is the simplest correct answer. The compose file is
structured so a future split is just two changes:

1. Pull `ros-jazzy-micro-ros-agent` + Renode out into a `sim` service.
2. Configure DDS for inter-container discovery (set `ROS_DISCOVERY_SERVER`
   on both `sim` and `server` services to a third `discovery` service
   running `fastdds discovery`).

Worth doing if/when CI parallelism or per-component caching becomes a
bottleneck — premature otherwise.

## Renode version pin

`Dockerfile.e2e` pins Renode to **1.15.3** via the `RENODE_VERSION`
build arg. The host's Renode 1.16.0 was failing the RP2040 bootrom →
XIP flash handoff — the CPU executed bootrom code but never reached
`main()`, and `uart0` stayed silent. Pinning lower until 1.16+ is
shown to work with the `renode_rp2040` board files. Bump and re-test
when:

* The `firmware/rp2040/simulation/renode_rp2040/tests/` Robot Framework
  suite passes on 1.16+, or
* The `cores/rp2040_simple.repl` flash/XIP model is updated to match
  Renode 1.16's expected register layout.

To override at build time:

```bash
docker compose -f firmware/simulation/docker/docker-compose.e2e.yml build \
    --build-arg RENODE_VERSION=1.16.0
```

## Troubleshooting

### `agent died during startup`

ROS2 jazzy needs its setup script sourced before `ros2 run` works. The
entrypoint does this for you — if you see this error, the
`ros-jazzy-micro-ros-agent` package install probably failed during
image build. Check `docker build` output for apt errors.

### Sim node creates but firmware never prints

Most common cause: the host didn't run `make install_sim` after
`build.sh sim`. The bind mount expects `firmware/rp2040/install/simulation/saint_node.elf`;
if that file isn't there the resc loads an empty ELF and the firmware
just doesn't run. Symptom in the container log: the `node_manager`
output mentions the firmware path but `<node>.uart.log` stays at 0
bytes.

Second most common: Renode version + repl mismatch (see "Renode
version pin" above). If you see `xip_ssi.xip_flash: Unhandled
operation` warnings repeatedly without progress past bootrom, that's
the XIP-handoff issue.

### Harness times out on phase 1 / 2 / 3

The harness preserves the per-node uart.log on failure
(see `firmware/simulation/logs/` on the host). Look for one of:

* `Config received` — firmware got the sync push
* `Config applied OK` / `Config apply failed` — firmware tried to apply
* `Config saved to flash` — firmware persisted
* `Adopted node announced UNADOPTED — re-pushing peripheral config`
  — server detected divergence (phase 3)

Which lines are present + which are missing pinpoints exactly which
hop is breaking.
