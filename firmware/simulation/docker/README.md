# SAINT.OS end-to-end test in Docker

Runs the full `firmware/simulation/test_sync_recovery.py` flow against
a real micro-ROS agent + saint_server inside a container — useful when
the host machine doesn't have (or shouldn't have) ROS2 installed, and
for reproducible CI runs. Boots the real RP2040 firmware in Renode and
exercises the same XRCE-DDS / DDS chain production uses.

## TL;DR — run the e2e

```bash
# From the repo root:
# 1. Build the sim firmware on the host. The container reads the ELF
#    from a bind mount, so you only need to rebuild the image when
#    docker/agent/server changes — not when firmware changes.
(cd firmware/rp2040 && ./build.sh sim && cd build_sim && make install_sim)

# 2. Run.
MODE=full docker compose -f firmware/simulation/docker/docker-compose.e2e.yml up \
    --build --abort-on-container-exit --exit-code-from e2e
```

Logs land in `firmware/simulation/logs/` on the host (bind-mounted out
of the container):

| File | What it is |
|---|---|
| `rp2040_synctest.log` | Renode operational log (CPU init, peripheral warnings, log markers) |
| `rp2040_synctest.uart.log` | Firmware `printf` stream — booted-or-not is here |
| `agent.log` | micro-ROS agent stdout/stderr — XRCE session events, crashes |
| `server.log` | saint_server stdout/stderr — WS API hits, DDS topic traffic |

The agent/server log capture is wired through `entrypoint.sh`'s `EXIT`
trap so they survive the container shutdown.

## Modes

```bash
# Default. Renode + real firmware + agent + server + harness.
MODE=full   docker compose -f ... up --build --abort-on-container-exit --exit-code-from e2e

# Python fake firmware (no Renode). Sanity-checks the server/harness
# chain without depending on the sim. Useful when you've broken Renode
# and want to confirm the rest still works.
MODE=fake   docker compose -f ... up --build --abort-on-container-exit --exit-code-from e2e

# Just the sim bring-up (no server, no DDS, no auto-reconcile assert).
# Fast iteration when you're debugging Renode itself.
MODE=smoke  docker compose -f ... up --build --abort-on-container-exit --exit-code-from e2e

# Drop into a debug shell with agent + server already running.
MODE=shell  docker compose -f ... run --rm --service-ports e2e
```

## What the e2e exercises (MODE=full)

Three phases in `test_sync_recovery.py`:

1. **Sim node bring-up.** Renode boots the firmware, micro-ROS session
   establishes against the agent, firmware reaches `BOOT → UNADOPTED`
   and announces.
2. **Sync to Node.** Test adopts the node, pushes a config, asserts the
   firmware acks `Config received` / `Config saved to flash`. Exercises
   the DDS publisher pre-create path.
3. **Auto-reconcile.** Factory-resets the sim node mid-test to simulate
   a post-OTA flash wipe, watches the server detect the divergence and
   re-push the saved config, asserts the node returns to ACTIVE.

## Architecture

Single container, one network namespace. Renode, the micro-ROS agent,
and the saint_server all talk via `127.0.0.1`. The harness runs in the
same container.

```
┌─────────────────── e2e container ────────────────────┐
│                                                       │
│   Renode (RP2040 sim)                                 │
│       │                                               │
│       │ UDP 127.0.0.1:8888 ←→ 127.0.0.1:9999          │
│       ▼                                               │
│   micro-ROS agent ──── DDS ──── saint_server          │
│                                       │               │
│                                       │ WS :80        │
│                                       ▼               │
│                              test_sync_recovery.py    │
│                                                       │
└───────────────────────────────────────────────────────┘
```

### Why not a service-per-container compose?

Cross-container DDS discovery in Docker requires either
`network_mode: host` (Linux-only, won't work on Docker Desktop) or a
FastDDS / Cyclone DDS discovery-server profile. For a test fixture
that's spinning up + tearing down whole stacks per CI run, one network
namespace is the simplest correct answer. The compose file is
structured so a future split is two changes:

1. Pull the agent + Renode out into a `sim` service.
2. Configure DDS for inter-container discovery (`ROS_DISCOVERY_SERVER`
   on both `sim` and `server` services pointing to a third `discovery`
   service running `fastdds discovery`).

Worth doing if/when CI parallelism or per-component caching becomes a
bottleneck — premature otherwise.

## ROS distro must match the firmware's Pico SDK

`docker-compose.e2e.yml`'s `ROS_DISTRO` build arg controls the whole
stack: the base image, the agent image, and the rmw implementation.
The firmware's micro-ROS client is the static library shipped in
`firmware/rp2040/lib/micro_ros_raspberrypi_pico_sdk/libmicroros/libmicroros.a`
— it has a branch per distro (`humble`, `iron`, `jazzy`, `kilted`,
`rolling`). The XRCE-DDS protocol's `CREATE_SESSION_Payload` layout
shifted between humble and jazzy, so **the Pico SDK submodule branch
must match `ROS_DISTRO`** or the agent will receive malformed packets
and the firmware will time out on session-create.

Current pin: **jazzy**. To switch:

```bash
# 1. Branch the Pico SDK submodule
git -C firmware/rp2040/lib/micro_ros_raspberrypi_pico_sdk checkout <distro>

# 2. Rebuild the firmware
rm -rf firmware/rp2040/build_sim
(cd firmware/rp2040 && ./build.sh sim && cd build_sim && make install_sim)

# 3. Bump ROS_DISTRO in docker-compose.e2e.yml
```

## Renode invocation gotchas

These are baked into `firmware/simulation/node_manager.py` — don't
re-introduce:

- **Never pair `--console` with `stdin=subprocess.DEVNULL`.** Renode's
  monitor reads EOF on the first read and disposes the machine ~100 ms
  after `start`. `--disable-xwt` already implies `HideMonitor`, so the
  background path runs without `--console`.
- **Never set `cpu0 MaximumBlockSize 1` + `cpu0 LogFunctionNames true`
  together.** Block size 1 forces the JIT to retranslate every
  instruction; with per-instruction symbol logging on, ordinary
  bootrom busy-waits look like hangs because they take real wall time
  to execute. The bootrom completes in ms when these are stripped.
- **`logLevel -1 sysbus.<peripheral>` is fine and cheap.** Strip from
  the resc generator before merging, but use freely while debugging.

## micro-ROS agent ABI pinning (the stack-smashing gotcha)

`microros/micro-ros-agent:jazzy` (the image published 2025-09-11) was
linked against:

- `libfastrtps.so.2.14.5`
- `libfastcdr.so.2.2.5`

`ros:jazzy-ros-base`'s apt feeds now ship `2.14.6` / `2.2.7` of these
libs. Two failure modes if you try to run the agent against the newer
libs in the same container:

1. **`*** stack smashing detected ***`** on the first received packet —
   the SHM transport's stack-frame layout shifted in the .6 point
   release, the agent overruns its local buffer, GCC's stack canary
   fires.
2. **`undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj`** once a
   session establishes — `librmw_dds_common__rosidl_typesupport_fastrtps_cpp.so`
   from the .7 fastcdr requires a symbol that .5 doesn't export.

`Dockerfile.e2e` works around both by stashing the bare agent image's
entire `/opt/ros/jazzy/lib` to `/opt/uros_libs`, and `entrypoint.sh`
prepends that to `LD_LIBRARY_PATH` only when starting the agent. The
agent binary is invoked directly (not via `ros2 run`) so the ros2 CLI
doesn't drag in the apt-installed typesupport libs that demand the
newer fastcdr.

If you ever see the agent dying with stack smashing or undefined
symbols, first thing to check: did the bare agent image's lib
versions drift, or did the apt feeds drift, such that they diverged
again? Run:

```bash
docker run --rm --entrypoint bash microros/micro-ros-agent:jazzy \
    -c 'ls /opt/ros/jazzy/lib/libfastrtps.so.* /opt/ros/jazzy/lib/libfastcdr.so.*'
```

vs

```bash
docker compose -f firmware/simulation/docker/docker-compose.e2e.yml run \
    --rm --entrypoint bash e2e -c \
    'ls /opt/ros/jazzy/lib/libfastrtps.so.* /opt/ros/jazzy/lib/libfastcdr.so.*'
```

If the version numbers no longer match, either update the symlink
recreation in the `COPY --from=uros_agent` block of `Dockerfile.e2e`
or rebuild the agent from source against the current apt versions.

## Troubleshooting

### `agent died during startup` / agent log shows `stack smashing detected`

Almost always the Fast-DDS lib mismatch above. Verify
`/opt/uros_libs` exists in the image and contains the right
`libfastrtps.so.2.14.x` / `libfastcdr.so.2.2.x` files, and that
`entrypoint.sh` is launching the agent with `LD_LIBRARY_PATH` pointing
to it.

### Sim node creates but firmware never prints

Most common cause: the host didn't run `make install_sim` after
`build.sh sim`. The bind mount expects
`firmware/rp2040/install/simulation/saint_node.elf`; if that file
isn't there the resc loads an empty ELF and the firmware just doesn't
run. Symptom in the container log: the `node_manager` output mentions
the firmware path but `rp2040_synctest.uart.log` stays at 0 bytes.

Second most common: Renode died early because of a `--console`
regression — see "Renode invocation gotchas" above. Symptom: Renode
log ends with `Machine paused` + `Disposed` within ~100 ms of
`Machine started`, with no UART activity.

### Harness times out on phase 1 (`node never announced`)

Check `agent.log` for XRCE session events:

* `create_client → establish_session` — the firmware's XRCE handshake
  reached the agent. If this is missing, the issue is below the
  micro-ROS layer (UDP routing, firmware boot, etc.).
* `create_participant → create_publisher → create_datawriter` — the
  firmware finished session bring-up and is publishing DDS topics. If
  this is missing, the firmware's micro-ROS init aborted — check the
  UART log for `Failed to initialize support: <n>`.

If all of those are present and phase 1 still fails, the gap is on
the server side (it's not seeing the firmware's announcement). Check
`server.log` for the firmware's log topic (`/saint/nodes/<id>/log`)
showing up — if it's there, DDS is bridging fine and the bug is in
the unadopted-announcement bridging specifically.

### Harness times out on phase 2 or 3

The harness preserves all logs on failure. Look for one of:

* `Config received` — firmware got the sync push
* `Config applied OK` / `Config apply failed` — firmware tried to apply
* `Config saved to flash` — firmware persisted
* `Adopted node announced UNADOPTED — re-pushing peripheral config`
  — server detected divergence (phase 3)

Which lines are present + which are missing pinpoints exactly which
hop is breaking.

## See also

- `docs/RENODE_RP2040_BRINGUP.md` — history of the Renode bring-up,
  including diagnoses that were wrong before they were right.
- `firmware/simulation/README.md` — non-Docker (host-native) sim usage
  for individual `node_manager.py` commands.
