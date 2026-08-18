# Collision Avoidance & Motion Supervision

> **Status:** Draft / planning · **Started:** 2026-08-15
> Living document — tracks design decisions, open questions, and phased tasks.
> Update the checklists and Changelog as work lands.

## 1. Motivation

SAINT.OS blends **automated motion** (animations, poses) with **manual motion**
(operator sticks) onto the same joints. Nothing today prevents those from
driving physical parts into each other. The concrete failure case that prompts
this work: on the OpenSAINT head (`johnny5_head` URDF), the **eyebrows can hit
the eyes** in certain positions — especially combined with the **eye-pop**
(the `*EyePop` prismatic joints push the eye forward, changing what the brows
can clear).

We want the system to *know* the mechanism's geometry and refuse to command a
pose that collides — both while an operator is **authoring** (visual feedback in
the animation viewer) and at **runtime** (motion stops at the contact boundary
and resumes when clear).

## 2. Goal & scope

**This is reactive collision avoidance, not path planning.** The desired
runtime behavior — *drive toward the target, stop at contact, hold, resume when
the path clears* — is a per-tick safety supervisor, not a trajectory search
(RRT/PRM). Full planning is out of scope: it doesn't fit real-time stick +
animation blending, and reactive clamping is what the requirement describes.

**In scope**
- Collision **geometry** for the interacting bodies, carried in the URDF.
- **Authoring-time** collision display in the animation viewer (highlight as the
  operator scrubs/poses).
- **Runtime** collision governor: clamp/hold commanded motion at the collision
  boundary; resume toward the target when it clears, if still commanded.

**Non-goals (for now)**
- Full motion planning / path search around obstacles.
- Environment (world) collision — self-collision only.
- A complete self-collision matrix for all 51 links (see §5.2 — we whitelist the
  pairs that matter).
- Dynamics/physics simulation.

## 3. Current state (what exists / what's missing)

- **URDF has no collision or inertia.** `johnny5_head.urdf`: 51 links, 106
  `<visual>`, **0 `<collision>`, 0 `<inertial>`**. Visual meshes are STL at
  `scale="0.001"`, many color-split; several links are cosmetic sub-parts
  (`StepColors`, `lip_led_*`, vent fins, `CadNeck` balls/pistons).
- **Forward kinematics is free on the client.** `URDFViewer.vue` uses
  `urdf-loader`, which poses the real geometry in the three.js scene graph —
  **including the eye-pop translation**. So client-side FK for the posed state
  already exists.
- **The −1..+1 ↔ joint mapping exists (client).** As of 2026-08-15 the viewer
  maps the SaintOS −1..+1 control range onto each joint's `<limit>`,
  home-centered at θ=0 (convention C: `−1→lower, 0→home, +1→upper`; see
  `denormJoint`/`normJoint` in `URDFViewer.vue`). **This is the bridge** from
  channel/servo space into the kinematic space where collision lives.
  ⚠️ It currently lives **only in the JS viewer** — the runtime governor needs
  the same mapping (and the joint limits) **server-side** (see §6, Phase 2).
- **No server-side FK, no collision engine, no output supervisor.** The routing
  evaluator (`routing_evaluator.py`) emits per-channel −1..+1 setpoints with no
  awareness of kinematics.

## 4. Key design decisions

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | **Reactive avoidance, not planning.** | Matches the requirement (stop-at-contact / resume); fits real-time blended control. |
| D2 | **Collision geometry lives in the URDF** (`<collision>` convex hulls). | Keeps the model self-contained / URDF-only, consistent with the "no sidecar" preference. |
| D3 | **Whitelist collision *pairs* instead of a full ACM.** | We only care about specific interactions (brows ↔ eyes/eye-pop). Avoids needing an SRDF allowed-collision matrix (which URDF can't hold). Compact + URDF-friendly. |
| D4 | **Convex hulls (not raw visual meshes) for the collision shapes.** | Engine-friendly (convex-convex is cheap/robust) and fast enough for per-tick runtime checks. Authoring viz *may* test visual meshes directly for fidelity. |
| D5 | **Governor applies only to channels bound to URDF joints.** | Peripherals are otherwise kinematics-agnostic; non-joint channels pass through untouched. |
| D6 | **Phase the work: geometry + authoring viz first, runtime governor after a design doc.** | Phase 1 is low-risk and validates the geometry before we trust it on hardware; Phase 2 is safety-critical and needs deliberate design. |

### Architectural flag (needs explicit sign-off)
Today peripherals are deliberately decoupled from any kinematic model
(`(peripheral, channel)`, −1..+1). The runtime governor (Phase 2) **couples the
output path to a URDF collision model** for joint-bound channels — inserting a
kinematics/safety layer between routing output and the servos. This is a real,
intentional addition to the peripheral-first architecture, not an incidental
change. Decide deliberately.

## 5. Design notes

### 5.1 Collision bodies (candidate set — to finalize)
Only bodies that can actually touch need geometry. Candidates:
- **Brows:** `BrowLeftTopTilt`/`Open`, `BrowLeftBottomOpen`,
  `BrowRightTopTilt`/`Open`, `BrowRightBottomOpen` (+ their carrier links).
- **Eyes:** `Left/RightLens*`, `Left/RightEye*` (lens/gimbal/fixed), and the
  **eye-pop** prismatic (`Left/RightEyePop`) which translates the eye body.
- (Open) mouth / nose vs brows? Left vs right brow? — decide from the actual
  reachable envelope.

### 5.2 Pairs to check (whitelist — to finalize)
Start with same-side brow ↔ eye, e.g. `{BrowLeft* ↔ LeftEye*}`,
`{BrowRight* ↔ RightEye*}`, evaluated **with the eye-pop position live**.
Cross-side and brow↔brow pairs added only if the envelope shows they matter.

### 5.3 Collision engines
- **Client (authoring):** `three-mesh-bvh` — mesh↔mesh intersection on the posed
  three.js geometry. `three` is already a dependency; `three-mesh-bvh` is a new
  one.
- **Server (runtime):** `python-fcl` / `trimesh`+FCL (convex-convex). Requires
  **server-side FK** (link transforms from joint state) — either a light URDF FK
  (`yourdfpy`/roll-our-own from `<origin>`/`<axis>`) or reuse a lib.
  ⚠️ **Offline-dist concern:** these are native/C-extension wheels; they must be
  bundled into the arm64 offline dist (`scripts/build-local-dist.sh`) to keep the
  "installs offline" guarantee.

## 6. Phased plan

### Phase 0 — Collision geometry (mechanical, low risk)
> **Update 2026-08-15:** a collision-bearing URDF was provided
> (`urdfModel.zip`) — 111 `<collision>` across 44 links, `auto_collision_*`
> = **full-detail visual-mesh copies** (e.g. 72k-tri eye body) + 4 iris
> cylinders, **no pair/ACM metadata**. So geometry now *exists*; hull
> generation below becomes a **performance** step (for BVH/FCL), not a
> prerequisite for eyeballing.
- [ ] Finalize the collision **body set** (§5.1) and **pair whitelist** (§5.2).
- [ ] Generate **convex hulls** from the visual STLs for those bodies (apply
      `scale` + visual `<origin>`); emit `<collision>` blocks into the URDF.
- [ ] Decide where the hull-gen step lives (a repo script that rewrites the URDF
      / an upload-time step in `URDFStore`).
- [ ] Store the pair whitelist (URDF custom tag or small derived config — keep
      URDF-only per D2/D3).

### Phase 1 — Authoring-time collision display (self-contained, no control-loop risk)
- [x] **1a — Collision-geometry overlay toggle** in the viewer (`parseCollision`
      + translucent red overlay, toolbar button). Lets you eyeball the collision
      model. *Done 2026-08-15 (`URDFViewer.vue`).*
- [x] Add `three-mesh-bvh`; build BVHs for the collision bodies on URDF load.
      *Done 2026-08-15 — `utils/collision.js` (full collision meshes + BVH,
      AABB broadphase, home-pose baseline auto-ACM).*
- [x] Per-pose (on scrub) collision test against the **posed** scene (eye-pop
      included). *Live "⚠ Collision" badge over the viewer (`collisionsAtCurrent`).*
- [x] **Timeline collision markers** — `scanTimeline()` sweeps the animation,
      `AnimationEditorView` debounces the scan, `TimelineEditor` draws red bands
      on the scrub ruler at colliding timecodes. *This was the requested feature.*
- [x] Repo regression test: `utils/__tests__/collision.spec.js` (4 tests).
- [ ] Verify live in the running mock + web UI with the collision model
      (needs a human to scrub a brow-into-eye animation and see the band).
- [x] **3D highlight** of the offending bodies — colliding links' visual
      meshes turn solid red at the playhead (`highlightCollision`, driven from
      `driveUrdfFromPlayhead`). *Done 2026-08-15.*
- [~] **Convex-hull simplification — tried and reverted.** Benchmarks showed
      hulls are *not* reliably faster (over-approximation makes bodies overlap
      more → more narrowphase) and they flag false collisions at the tight
      clearances that matter (brow skimming eye). Kept full-detail meshes. The
      real perf lever is the **home-pose baseline** (suppresses adjacent-part
      overlaps) — which only works once the load-timing fix is in, so the scan
      narrowphases only genuine new hits. Measured (spread bodies): ~0.3 ms/check.
- [x] **Non-blocking scan** — the scan runs on an **invisible robot clone**
      (FK sandbox, shares geometry/BVHs) and is **time-sliced** across animation
      frames (6 samples/frame, then `requestAnimationFrame` yield), with
      generation-based cancellation. *Done 2026-08-15.*
- [x] **Idle-gated scan** — a scan only runs after the user has been idle for
      ~350 ms; **any** manipulation aborts an in-flight scan and resets the
      countdown. Detected via a document-wide capture-phase activity listener
      (pointerdown/up, drag-move, wheel, keydown, input, change) so it covers
      sidebar edits, Save, other controls, timeline scrub, and 3D orbit — not
      just the viewer. *Done 2026-08-15.*
- [ ] *(Deferred)* if a single slice is still too heavy on huge models, move
      narrowphase to a **web worker**; and/or the **runtime** path uses a ROS
      collision engine (MoveIt 2 / FCL) server-side — see §4/Phase 2.
- [ ] *(Deferred)* baseline **contact-depth threshold** (vs. binary ignore-at-home).

### Phase 2 — Runtime collision governor (safety-critical — DESIGN DOC FIRST)
- [ ] **Write the Phase 2 design doc** before code: where it sits in the loop,
      clamp semantics, source blending (manual + animation), failure/degradation
      modes, interaction with dead-man/estop, control-rate budget.
- [ ] Port the **−1..+1 ↔ joint mapping + joint limits to the server** (today
      client-only; see §3).
- [ ] Add **server-side FK** + collision engine (§5.3) + offline-dist bundling.
- [ ] Implement the governor: per tick, map proposed channel setpoints → joint
      state → FK → check whitelisted pairs → clamp offending joint(s) to the last
      safe position along the commanded direction; hold; resume when clear.
- [ ] Hardware verification on the actual head (brows + eye-pop).

## 7. Open questions / decisions needed
- [ ] **Purpose confirmed:** motion-planning-flavored *reactive avoidance* — OK? (assumed yes)
- [ ] Final collision **body set** and **pair whitelist** (§5.1/§5.2).
- [ ] Collision shape fidelity: convex hull everywhere, or boxes for simple parts?
- [ ] Where does the **pair whitelist** live so it stays URDF-only (custom tag vs derived)?
- [ ] Runtime governor **placement** in the server pipeline and its interaction
      with the peripheral-first decoupling (architectural sign-off — §4 flag).
- [ ] Clamp strategy: bisection-to-boundary vs velocity/step limit toward target.
- [ ] Offline-dist: acceptable to add `python-fcl`/`trimesh` to the arm64 bundle?

## 8. Risks
- **Safety-critical (Phase 2).** A wrong governor either *allows* a crash or
  *freezes* motion. Must fail safe and degrade predictably (no model → passthrough
  with a loud log; solver stuck → hold last safe).
- **Control-rate budget.** Per-tick FK + collision on the whitelisted pairs must
  fit the control loop; convex hulls + a small pair set keep this bounded.
- **Model drift.** Collision geometry is derived from the uploaded URDF; a new
  CAD export must re-derive (tie hull-gen to upload, per Phase 0).
- **Mapping duplication.** The −1..+1↔joint mapping now exists client-side; the
  server copy must stay in lockstep (single source of truth / shared spec).

## 9. References
- URDF: `johnny5_head.urdf` (uploaded via Settings → Robot Model).
- Viewer + FK + −1..+1 mapping: `server/web/src/components/animation/URDFViewer.vue`
  (`denormJoint`/`normJoint`), joint sliders in `.../PropsPanel.vue`.
- Routing output: `server/saint_server/router/routing_evaluator.py`
  (`set_urdf_joint_value`, `apply_animation_frame`, `_urdf_joint_values`).
- Animation player: `server/saint_server/animation/player.py`.
- URDF parsing/store: `server/saint_server/animation/urdf_store.py`
  (`list_joints` — currently name/type only; needs limits + collision awareness).
- Mock (dev): `server/web/dev/mock-http.js`, `mock-server.js`, `mock-state.js`.

## 11. Precomputed C-space collision tables (candidate strategy)

Idea: since self-collision is **deterministic in joint space**, precompute
(once per model, server-side) a compact per-pair lookup over the joints that
affect that pair, then the frontend/runtime do **O(1) lookups** instead of
live BVH scans. This would remove the clone + time-slicing + idle-gating and
also give the Phase 2 runtime governor an instant "is this safe?" check + the
collision-free **ranges**.

**Why per-pair:** the relative pose of two rigid links depends *only* on the
movable joints on the kinematic path between them — so each pair gets a small
table over a few joints, not one giant N-D table. Decomposition is exact
(modulo grid resolution).

**Finding (2026-08-18):** raw per-pair path dimensionality for this rig is
**5–8 DOF**, higher than hoped — the eyes are deep in a `nose → eye-pop →
lens → iris` chain. A dense grid over all path joints is infeasible at 8 DOF.
Viable only with **dimensionality reduction**:
- Prune **occluded pairs** (brow × pupil/iris/vent — the brow hits the eye
  housing / eye-pop first).
- Prune **non-influential joints** per pair (iris is internal; nose/gaze may
  not move the outer envelope) — determined by domain knowledge or an
  automatic **sensitivity analysis** (perturb each path joint, drop ones that
  never change the collision outcome).
- Target: the dominant case **brow-tilt/open × eye-pop ≈ 3 DOF** → 32³ ≈ 33 K
  cells. Encode as a bitset, or store collision-free **ranges/boundary** for
  higher-D pairs.

**Open decisions:** how to reduce dimensionality (auto sensitivity vs domain
pruning); grid resolution vs conservative dilation near boundaries; where the
precompute runs (server FCL at upload, cached + shipped to UI + reused at
runtime).

## 10. Changelog
- **2026-08-18** — Explored precomputed C-space collision tables. Analyzed
  per-pair dimensionality from the URDF kinematic paths: brow↔eye pairs are
  **5–8 DOF raw** (eyes sit behind nose+pop+lens+iris), so dense tables need
  dimensionality reduction (prune occluded pairs + non-influential joints →
  ~3 DOF for the dominant brow×eye-pop case). Recorded strategy + open
  decisions in §11.
- **2026-08-15** — Perf, round 4 (interaction sluggish while scan runs): made
  the scan **idle-gated** — runs only after ~350 ms of no input, and ANY
  manipulation aborts it and resets the countdown. Uses a document-wide,
  capture-phase activity detector (pointer/drag/wheel/keydown/input/change) so
  editing sidebar values, clicking Save, or any control counts as manipulation,
  not just the 3D view. Scan reruns automatically once the user pauses.
- **2026-08-15** — Perf, round 3 (UI fully frozen during scan): the scan was
  synchronous on the one JS thread, so nothing rendered/handled input until it
  finished. Made it **non-blocking** — runs on an invisible robot **clone**
  (`URDFRobot.copy` rewires joints; geometry/BVHs shared) and **time-slices**
  across frames with rAF yields + generation cancellation. Orbit/scrub now work
  during reprocessing. Architecture note: authoring stays client-side (off the
  main thread); the **runtime governor (Phase 2) uses ROS — MoveIt 2 / FCL**.
- **2026-08-15** — Perf, round 2 (UI blocked during scan): benchmarked — 107
  spread bodies = 0.3 ms/check, so O(n²) broadphase is cheap; cost is narrowphase
  on overlapping pairs. Tried convex hulls → reverted (unreliable + inaccurate at
  tight clearances). Real fix = the load-timing correction (baseline now captures
  home-pose overlaps and suppresses them). Reduced scan to ~15 samples/s and
  added ms-timing to the `[collision]` diagnostics (BVH build / baseline / scan)
  to pinpoint any residual cost. Web-worker narrowphase noted as the fallback.
- **2026-08-15** — Perf: the live collision check + 3D re-tint was running on
  every keyframe-drag mousemove (via the value-track watcher), tanking drag
  performance. Throttled it to ~10 Hz (trailing); posing stays immediate. Full
  timeline rescan stays debounced (250 ms). Deferred: async/worker scan if the
  post-drag rescan burst is noticeable on large animations.
- **2026-08-15** — Fixed a load-timing bug (colliders were built before the
  async collision-mesh geometry loaded → empty set → no markers); the viewer now
  waits for all meshes first. Added **3D highlight**: colliding links' visual
  meshes turn solid red at the playhead. Dev-only `[collision]` console
  diagnostics added.
- **2026-08-15** — Implemented **Phase 1b (detection)**: `utils/collision.js`
  engine (three-mesh-bvh, AABB broadphase, home-pose baseline auto-ACM — no
  manual pair list/ACM), viewer `collisionsAtCurrent()`/`scanTimeline()`,
  debounced full-animation scan in `AnimationEditorView`, and **red collision
  bands on the timeline ruler** (the requested feature) + a live "⚠ Collision"
  badge. Added `collision.spec.js` (4 tests; suite 51/51 green). Design calls
  made without re-asking: full collision meshes (not hulls) for now; auto
  baseline instead of a whitelist.
- **2026-08-15** — Collision URDF (`urdfModel.zip`) received + verified through
  the pipeline (uploads to mock; all 102 unique visual+collision mesh refs serve
  200). Geometry is `auto_collision_*` full-detail mesh copies + iris cylinders,
  no pair metadata. Implemented **Phase 1a**: collision-overlay toggle in
  `URDFViewer.vue` (`parseCollision`, translucent render, toolbar button).
- **2026-08-15** — Doc created. Captured motivation (brows↔eyes + eye-pop),
  scope (reactive avoidance, not planning), decisions D1–D6 + architectural flag,
  phased plan, open questions. Noted the −1..+1 home-centered joint mapping
  landed in the viewer the same day and is the channel→kinematics bridge.
