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

## 13. Direction: general live-checking + ACM (supersedes precompute as primary)

**Decision (2026-08-18):** the collision solution must work for **arbitrary
uploaded URDFs**, not just johnny5. That rules out precomputed C-space tables as
the *primary* mechanism (they only work for low-DOF pairs; a general robot can be
high-DOF → curse of dimensionality). Precompute is demoted to an **opportunistic
accelerator** for pairs whose relevant DOF ≤ ~3 (see §11), not the foundation.

**Primary mechanism (general, MoveIt-style but FCL-direct):**
1. **Auto ACM at upload** (any URDF): disable pairs that are tree-**adjacent**,
   collide **at rest**, or collide in **~all** random samples ("always" = by
   design). What survives = the real watch-list. Mirrors the MoveIt Setup
   Assistant. *(An enumeration over 1200 random poses flagged 103 "colliding"
   pairs — but most are parent/child design-adjacencies (fabco body/piston,
   pupil/inner, vent/fins, nose body/basket) or random-extreme artifacts; a
   proper ACM prunes them.)*
2. **Decimate / convex-hull the collision meshes** — full-detail all-pairs was
   **452 ms/pose** (the whole perf problem); decimation → ~ms, for any model.
   Keep full detail only on pairs flagged tight-clearance.
3. **Fast FK** (batched/compiled, not per-cell Python; FK was ~80% of cost).

This serves both authoring (the existing frontend checker + ACM + decimation —
likely removing the need for idle-gating) and the runtime governor, and
generalizes to any URDF. Precompute tables stay a future per-pair speedup.

## 11. Precomputed C-space collision tables (opportunistic accelerator only)

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

## 12. Backend engine evaluation — MoveIt 2 vs. FCL-direct (for Phase 2)

**MoveIt 2 — what it gives out of the box** (verified 2026-08-18):
- Self- + scene-collision checking (FCL) from URDF+SRDF.
- ACM precompute via the Setup Assistant (samples configs, disables
  always/never/adjacent-colliding pairs).
- `moveit_servo`: reactive avoidance that **scales joint velocity down and
  stops before contact** (`self_collision_proximity_threshold`,
  `scene_collision_proximity_threshold`, `collision_check_rate`,
  `check_collisions`) — essentially the runtime "advance-until-near-contact,
  hold, resume" governor, as smooth scaling.
- Available on **ROS 2 Kilted (MoveIt 2.14.0)**, binaries for Ubuntu 24.04
  amd64 + arm64.

**What MoveIt does NOT give:** a shippable precomputed lookup for the frontend
authoring UI — it checks on-demand server-side. Authoring feedback stays a
separate concern (client-side, or WS round-trips, or our own precompute
tables).

**Caveats for SAINT.OS:**
- **Bundling:** binaries target Ubuntu 24.04; our server is Debian
  (Bookworm/Trixie) with **source-built ROS in the offline dist** → MoveIt
  would be a source build into the bundle (heavy: build time, size, Pi RAM/CPU).
- **Topology:** `moveit_servo` targets a serial manipulator + end-effector +
  planning group; our head is ~50 independent 1-DOF servos. The **collision
  checker** fits; the **servo abstraction** doesn't — likely use MoveIt's
  `collision_detection` in our own loop, not servo wholesale.
- Needs an SRDF/config pass (Setup Assistant) for a 50-joint non-serial rig.

**Lighter alternative — FCL / hpp-fcl / python-fcl direct:** same engine MoveIt
uses underneath, in a small node; no planning/SRDF/servo weight; fits the
independent-servo topology; natural home for the precompute tables that serve
BOTH authoring (shippable lookup) and runtime. Trade-off: we build the governor
+ precompute ourselves (vs. MoveIt's OOTB servo scaling + Setup Assistant).

**Decision (2026-08-18): FCL-direct** — lighter, faster to deploy/test, fits the
topology, serves both authoring precompute + runtime.

### FCL spike — measured (local Mac, python-fcl + trimesh + yourdfpy)
- **Effective dimensionality** (sensitivity analysis, non-influential joints
  pruned): brow-top × **eye-pop = 5 DOF** (brow open+tilt, eye-pop, **nose
  basket+body**), brow-top × **eye_h = 4 DOF** (lens/gaze pruned, nose kept).
  → The **nose is influential** (eyes are mounted on the nose chain), so it does
  NOT collapse to 3 DOF unless the nose is treated as static (domain question).
- **Per-cell cost ≈ 0.9 ms**, DOMINATED by Python FK (`update_cfg` 0.7 ms);
  FCL narrowphase is only ~0.18 ms even on the full-detail meshes. → FK is the
  bottleneck and is very optimizable (analytic/batched/compiled → ~0.1 ms).
- **Real precompute:** one 3D pair table, 20³ = 8 000 cells → **7.1 s** on Mac
  (23% of cells collide).
- **Extrapolated dense-grid precompute** (one pair; Pi ≈ 5× / 10× slower):

  | DOF | grid | cells | Mac (0.9ms) | Pi 5 (~5×) | Pi 4 (~10×) |
  |----:|-----:|------:|------------:|-----------:|------------:|
  | 3 | 20 | 8 K | 7 s | ~35 s | ~70 s |
  | 4 | 16 | 65 K | ~1 min | ~5 min | ~10 min |
  | 5 | 12 | 249 K | ~3.7 min | ~19 min | ~37 min |
  | 5 | 16 | 1.05 M | ~16 min | ~78 min | ~2.6 h |

  (÷3 with FK optimization; × number of watched pairs.)

**Implications:** precompute is a one-time upload job, so seconds-to-a-few-min
is fine; **tens of minutes (naive 5D on a Pi) is not**. Levers, highest first:
1. **Is the nose animated in normal use?** If effectively static → brow×eye-pop
   5D→3D, brow×eye_h 4D→2D → precompute in **seconds** even on a Pi. *(domain
   question — pending)*
2. **Optimize FK** (analytic/batched, not yourdfpy per-cell) → ~3× overall.
3. **Prune to the ~4–8 pairs that matter** (occluded pupil/iris/vent dropped).
4. **Boundary/range encoding** or coarse/adaptive grids for any residual 4–5D.
Runtime lookups are O(1) regardless, so the only cost that matters is this
one-time precompute.

## 10. Changelog
- **2026-08-18** — Prototyped the general approach in the **frontend** engine
  (experiment, before committing to the system): `collision.js` gained convex-
  hull simplification (`buildColliders({simplify:'hull'})`) + `computeAdjacency`;
  `URDFViewer` now builds a real **ACM** = adjacency ∪ at-rest ∪ ~always-collide
  (250 random samples on the clone) and uses it everywhere instead of the bare
  home-baseline. Instrumented: the `[collision]` console line reports hull tri
  reduction, ACM size, and build/scan times. Pending: measure in-browser.
- **2026-08-18** — Generality requirement (arbitrary uploaded URDFs, not just
  johnny5) → **pivoted away from precompute as primary** to general
  **live-checking + auto-ACM + mesh decimation** (§13). Enumeration of 1200
  random poses found 103 "colliding" pairs, but mostly parent/child
  design-adjacencies + random-extreme artifacts → confirms an ACM (adjacency +
  at-rest + always-collide pruning) is the right general filter. Full-detail
  all-pairs check measured at 452 ms/pose (the core perf issue → decimate).
- **2026-08-18** — **Chose FCL-direct** + ran a local FCL spike (python-fcl +
  trimesh + yourdfpy). Measured: per-cell ≈0.9 ms (FK-bound, not FCL),
  effective DOF 4–5 for brow↔eye (nose is influential), one 3D 8 K-cell table =
  7.1 s on Mac. Extrapolated Pi 4/5 precompute times (§12): 3D trivial, 4D a few
  min, naive 5D too long on a Pi. Key open lever: whether the **nose is
  animated** (static → collapses to 3D → seconds). FK optimization + pair
  pruning + boundary encoding are the other levers.
- **2026-08-18** — Evaluated **MoveIt 2** for the backend (§12): confirmed it
  does self/scene collision + ACM precompute (Setup Assistant) + reactive
  velocity-scaled avoidance (`moveit_servo`) OOTB, and is on Kilted (2.14.0,
  arm64). Caveats for us: source-build into the Debian offline dist, servo's
  manipulator/planning-group model doesn't fit a 50-independent-servo head
  (checker fits, servo doesn't), and it doesn't provide the frontend lookup.
  Noted FCL-direct as the lighter, better-fitting alternative that also feeds
  the authoring precompute. Decision deferred to the Phase 2 design doc.
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
