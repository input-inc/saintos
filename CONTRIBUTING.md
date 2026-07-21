# Contributing to SAINT.OS

Thanks for your interest in improving SAINT.OS!

Before anything else, please read the one principle that shapes every other
guideline here:

> **SAINT.OS is a robot-agnostic platform. OpenSAINT is just the first robot
> built on it.** Contributions should make the *platform* better for every
> robot — never hard-wire the assumptions of one machine into the code.

If you keep that in mind, most of the rest follows naturally.

## The agnostic rule

A robot's identity lives in **configuration**, not in code. The platform
already provides places for everything that varies between robots — use them
instead of special-casing:

| If it varies per robot… | …it belongs in |
|---|---|
| Platform name, homepage, node role labels | a **robot manifest** — `server/config/robots/<id>.yaml` |
| A board's pin layout / built-in peripherals | a **board definition** — `server/config/boards/**/<board>.yaml` |
| Which motor/servo/sensor a node drives | a **peripheral type** in the catalog + the operator's per-node peripheral config |
| How inputs map to actuators | an operator-authored **routing sheet** |
| Sounds, poses, animations, URDF meshes | per-deployment **data** (created in the UI / stored under `/etc/saint-os`) — not committed to the repo |

### The litmus test

Before opening a PR, ask: **"Would this change only make sense for
OpenSAINT (or any one specific robot)?"**

- **Yes** → it doesn't belong in the code. Express it as a manifest, board,
  peripheral, or routing-sheet configuration instead.
- **No — it helps any robot** → great, that's a platform contribution.

### Concretely, please avoid

- **Hard-coding role names.** A role is an optional, free-text label chosen
  from the active robot's manifest. Never branch on it
  (`if role == "head": …`). The peripheral, board, and routing layers carry
  behavior; roles are just labels. (An earlier per-role config system was
  removed precisely because behavior must not key off roles.)
- **Hard-coding a robot's name, node list, or topology.** Read it from the
  active manifest / adopted-node state.
- **Hard-coding pins or wiring.** New hardware = a new board YAML, not a
  special case in firmware or the server.
- **Baking one robot's control mapping into the server or controller.** That
  is what routing sheets are for.
- **Committing OpenSAINT-specific assets** (its sounds, animations, poses,
  URDF, tuned servo extents) into the repo. Ship them as deployment data, or
  as a clearly-optional example — not as core defaults.
- **Adding a peripheral for exactly one robot's part number** when a generic
  driver + config params would cover the family.

### Do add

- New **peripheral types** with generic, config-driven parameters
  (addressed as `(peripheral_id, channel_id)`).
- New **board definitions** for additional microcontrollers / carrier boards.
- New **routing operators**, input sources, animation/curve features, and
  transport/telemetry improvements that any robot benefits from.
- **Docs and examples** — including sample robot manifests for other builds.

## Repository layout

See the table in [README.md](README.md#repository-layout). In short:
`server/` (ROS 2 package + web UI), `controller/` (Tauri app),
`firmware/` (RP2040 / Teensy / Pi nodes + shared drivers),
`configs/`, `scripts/`, `packaging/`, `docs/`.

## Development workflow

1. **Branch** off `main`. Keep changes focused; one concern per PR.
2. **Match the surrounding code** — naming, comment density, and idioms.
   New code should read like the file it lives in.
3. **Preserve peripheral-first addressing.** Firmware and server address
   channels by `(peripheral_id, channel_id)`; don't reintroduce virtual-GPIO
   addressing.
4. **Build and test locally** (see below) before opening a PR.
5. **Describe the change** in the PR: what platform-level need it serves, and
   confirm it isn't robot-specific.

## Building & testing

Run the checks relevant to what you touched:

```bash
# Server (Python / ROS 2 package)
cd server && python3 -m pytest test/ -q

# Web UI (Vue) — from server/web
cd server/web && npm run test        # vitest
npm run build                        # must build clean

# Controller (from controller/)
npm run test

# Firmware driver unit tests (host-runnable, no hardware)
cd firmware/rp2040/tests && ./run_tests.sh

# Firmware builds
cd firmware/rp2040  && ./build.sh hw   # or: sim  (Renode)
cd firmware/teensy41 && ./build.sh hw

# Shell scripts — syntax-check anything you edit
bash -n scripts/<script>.sh
```

Add or update tests alongside behavior changes. Server and firmware changes
that alter wire formats or config schemas should include a regression test.

## Reporting issues

Open a GitHub issue with: what you expected, what happened, the affected
component (server / web / controller / firmware), and — for a robot-specific
symptom — enough detail to reproduce on a generic setup. If it turns out to
be configuration rather than a platform bug, we'll point you at the right
manifest / board / routing knob.

## License

By contributing, you agree that your contributions are licensed under the
project's [CC BY-NC-SA 4.0](LICENSE.MD) license.
