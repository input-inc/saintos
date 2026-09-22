<script setup>
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'
import * as THREE from 'three'
import { useDisplayStore } from '@/stores/display'

const display = useDisplayStore()

// Read a CSS color variable from <html> and parse it into a THREE
// numeric (0xRRGGBB). Three.js can't consume CSS variables directly
// — we resolve to the live computed value at the moment we need it,
// then re-apply when the theme changes via the watch below.
function cssColor (varName, fallback) {
  if (typeof document === 'undefined') return fallback
  const val = getComputedStyle(document.documentElement).getPropertyValue(varName).trim()
  if (!val) return fallback
  const m = val.match(/^#([0-9a-f]{6})$/i)
  if (m) return parseInt(m[1], 16)
  // color-mix() / rgb() etc. — fall back to the THREE.Color parser.
  try { return new THREE.Color(val).getHex() }
  catch (_) { return fallback }
}
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import URDFLoader from 'urdf-loader'
import { resolveMeshUrl } from '@/utils/meshUrl'
import { buildColliders, computeAdjacency, collidingPairs, samplesToIntervals } from '@/utils/collision'
import {
  buildControlShape,
  disposeRigShapeCache,
  setShapeHighlight,
  worldDragAxis,
} from '@/composables/useRigShapes'

const props = defineProps({
  // Source URL for the URDF text. Null/empty disables loading.
  urdfUrl: { type: String, default: null },
  // Base URL prefix used to resolve mesh references. The server flattens
  // meshes into a single dir, so we only ever need the basename — the
  // loadMeshCb below strips any package://… or relative-path prefix.
  meshesBase: { type: String, default: '/api/robot/meshes/' },
  // Optional fixed height. Falls back to 100% of the parent.
  height: { type: String, default: '100%' },
  // Self-collision machinery (parse <collision> geometry, hulls, BVHs,
  // FK-sandbox clone, ACM sampling). Everything the animation editor
  // needs and nothing a plain preview does — the Settings → Robot Model
  // tab was paying the full cost (including downloading every collision
  // mesh) to render a model it never collision-checks. Off = the
  // viewer is a pure viewer: collisionsAtCurrent returns [] and
  // scanTimeline returns null.
  collision: { type: Boolean, default: true },
})

const emit = defineEmits([
  'loaded',              // (robot) — the URDF root object3d
  'load-error',          // (Error)
  'joints',              // (jointNames[]) — emitted once after load
  'joint-click',         // (jointName) — operator clicked a link/mesh in the scene
  'joint-rotate',        // (jointName, angle) — live during gizmo drag
  'joint-rotate-commit', // (jointName, angle) — gizmo drag ended; safe to write a keyframe
  'interact',            // () — user is manipulating the view (orbit/zoom/drag)
  // Control-rig shape interaction. The viewer reports a normalized DRAG
  // DELTA and the parent turns it into a control value, so the viewport
  // and the side panel both go through one evaluate path.
  'rig-control-press',   // (controlName)
  'rig-control-drag',    // ({ name, delta }) | ({ name, deltaX, deltaY })
  'rig-control-release', // (controlName)
])

const container = ref(null)
const loading = ref(false)
const loadError = ref('')

let scene = null
let camera = null
let renderer = null
let controls = null
let robot = null
let grid = null
let rafHandle = 0
let resizeObserver = null

// Viewer-chrome state surfaced to the template (top-right toolbar).
const showGrid = ref(true)
// Collision-geometry overlay. Off by default; parseCollision below loads
// the <collision> shapes so this toggle can reveal them as a translucent
// overlay for eyeballing that the collision model looks sane.
const showCollision = ref(false)
// Self-collision detection state (see utils/collision.js). `colliders` is
// built once per load; `acm` is the Allowed-Collision Matrix — the set of
// link-pairs we ignore (tree-adjacent + colliding at rest + ~always colliding
// by design). Everything else is a genuine collision to report.
let colliders = []
let acm = null
// Meshes recolored for the live collision highlight: [{ mesh, material }].
let highlighted = []
// An invisible clone of the robot used purely as an FK sandbox for the
// timeline scan, so scanning never disturbs the visible model (no flicker)
// and can be time-sliced without fighting the user's scrub/orbit. Shares
// geometry with the display robot, so the BVHs are reused (not rebuilt).
let robotClone = null
let cloneColliders = []
let scanGeneration = 0
// Load-scoped abort for the async ACM build. Deliberately NOT
// scanGeneration: that counter is bumped by cancelScan() on every orbit,
// scroll and drag, and an ACM build that died on the first mouse wheel
// would leave collision reporting disabled forever. Only a new load (or
// unmount) invalidates an ACM in progress.
let acmGeneration = 0
const viewMenuOpen = ref(false)

// Raycast click-to-select state. We can't naively attach a `click`
// handler — OrbitControls swallows + emits mouseup after every orbit
// drag, and would fire spurious selections. Compare mousedown vs
// mouseup positions and only treat as a click if the cursor barely
// moved (4 px slop).
const raycaster = new THREE.Raycaster()
const ndcPointer = new THREE.Vector2()
let pressDownPx = null

// Per-joint rotation gizmo: an arc spanning the URDF <limit lower upper>
// range with a draggable sphere handle at the joint's current angle.
// The arc + caps + handle ARE the control — no TransformControls ring;
// the operator drags the cyan handle along the arc and the joint
// follows, clamped to the URDF-declared limits.
const LIMIT_ARC_RADIUS = 0.18
const LIMIT_ARC_SEGS   = 64
let limitVisual = null         // { group, arc, dot, lowMark, hiMark, joint }
// One arc-and-dot per joint in the current selection cluster. When
// a click resolves a multi-DOF region (shoulder, hip, …) we render
// a gizmo for every co-located joint so each DOF is draggable in
// place. Keyed by joint name. `limitVisual` points at whichever
// one the drag-state machinery is currently working with.
const limitVisuals = new Map()
let gizmoJoint = null          // currently-attached joint (mirrors limitVisual.joint)

// Handle-drag state. While `handleDragging` is true we own the mouse:
// orbit is suppressed, joint angle updates each move, and the
// drag-end emits a `joint-rotate-commit` so the caller writes a
// keyframe.
let handleDragging = false
const dragPlane = new THREE.Plane()
const dragCenter = new THREE.Vector3()
const dragFrame = { axis: new THREE.Vector3(), tangent: new THREE.Vector3(), bitangent: new THREE.Vector3() }
let dragStartMouseAngle = 0
let dragStartJointAngle = 0

function disposeMaterial (m) {
  if (!m) return
  if (Array.isArray(m)) {
    for (const sub of m) disposeMaterial(sub)
    return
  }
  for (const key of ['map', 'normalMap', 'roughnessMap', 'metalnessMap']) {
    if (m[key]?.dispose) m[key].dispose()
  }
  m.dispose?.()
}

function disposeObject3D (obj) {
  if (!obj) return
  obj.traverse((o) => {
    if (o.geometry?.dispose) o.geometry.dispose()
    if (o.material) disposeMaterial(o.material)
  })
}

function setupScene () {
  scene = new THREE.Scene()
  scene.background = new THREE.Color(cssColor('--color-canvas', 0x0f172a))

  const { clientWidth: w, clientHeight: h } = container.value
  camera = new THREE.PerspectiveCamera(45, w / Math.max(h, 1), 0.01, 100)
  camera.position.set(1.2, 1.2, 1.2)

  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setSize(w, h)
  renderer.setPixelRatio(window.devicePixelRatio)
  container.value.appendChild(renderer.domElement)

  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.dampingFactor = 0.08
  controls.target.set(0, 0.3, 0)

  // Lighting — one directional + an ambient fill keeps mesh details
  // legible without going full PBR. The shadow map is intentionally
  // disabled; URDFs render fine without and shadows cost ~30% of the
  // frame budget on integrated GPUs.
  const dir = new THREE.DirectionalLight(0xffffff, 1.2)
  dir.position.set(2, 3, 2)
  scene.add(dir)
  scene.add(new THREE.AmbientLight(0xffffff, 0.45))

  // Subtle ground grid for spatial reference. Module-level so the
  // Show/Hide Grid toolbar button can toggle visibility. Grid colors
  // follow the theme: --color-surface for the major lines (slate-700
  // in dark, slate-200 in light) and --color-line-subtle for the
  // minor lines.
  grid = new THREE.GridHelper(
    2, 20,
    cssColor('--color-surface', 0x334155),
    cssColor('--color-line-subtle', 0x1e293b),
  )
  grid.material.opacity = 0.7
  grid.material.transparent = true
  grid.visible = showGrid.value
  scene.add(grid)

  // Canvas pointer plumbing. Three concerns share this:
  //   1) Limit-handle drag (cyan dot → joint rotation).
  //   2) Joint pick (click a link mesh to summon the gizmo).
  //   3) OrbitControls (camera drag).
  // pointerdown decides which one owns the gesture; pointermove +
  // pointerup are gated on `handleDragging` so we don't fight orbit
  // when the operator's not interacting with the handle.
  renderer.domElement.addEventListener('pointerdown', onPointerDown)
  renderer.domElement.addEventListener('pointermove', onPointerMove)
  renderer.domElement.addEventListener('pointerup', onPointerUp)
  renderer.domElement.addEventListener('pointerleave', onPointerUp)
  // Separate, non-interfering interaction signals to pause the collision scan
  // while the user manipulates the view.
  renderer.domElement.addEventListener('pointerdown', onInteractStart)
  renderer.domElement.addEventListener('pointermove', onInteractMove)
  renderer.domElement.addEventListener('pointerup', onInteractEnd)
  renderer.domElement.addEventListener('pointerleave', onInteractEnd)
  renderer.domElement.addEventListener('wheel', onInteractWheel, { passive: true })
}

// Project the current mouse position onto the joint's rotation plane
// and compute the angle it represents within that plane's (tangent,
// bitangent) basis. Used during a handle drag to translate cursor
// motion into a joint angle.
function mouseAngleOnDragPlane (e) {
  const rect = renderer.domElement.getBoundingClientRect()
  ndcPointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1
  ndcPointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1
  raycaster.setFromCamera(ndcPointer, camera)
  const hit = new THREE.Vector3()
  if (!raycaster.ray.intersectPlane(dragPlane, hit)) return null
  const dir = hit.sub(dragCenter)
  return Math.atan2(dir.dot(dragFrame.bitangent), dir.dot(dragFrame.tangent))
}

// ── Limit visual: arc + handle around the joint axis ───────────────
//
// Read-only overlay (not directly draggable — TransformControls' ring
// is the interactive element). Lives in world space so its
// orientation tracks the joint's parent links as they animate.

function disposeLimitVisual (vis) {
  if (!vis) return
  const { group, arc, dot, lowMark, hiMark } = vis
  for (const obj of [arc, dot, lowMark, hiMark]) {
    obj?.geometry?.dispose()
    obj?.material?.dispose()
  }
  if (group?.parent) group.parent.remove(group)
}

function clearLimitVisual () {
  // Tear down every per-joint visual. `limitVisuals` keys are joint
  // names so we can target a single one if needed (drag commits go
  // through here when the gizmo set is rebuilt for a new cluster).
  for (const vis of limitVisuals.values()) disposeLimitVisual(vis)
  limitVisuals.clear()
  // Keep `limitVisual` pointed at the active joint's visual for the
  // existing drag-state machinery; reset here so nothing dangles.
  limitVisual = null
}

function buildLimitVisual (joint) {
  if (!joint || joint.jointType !== 'revolute') return
  const lim = joint.limit
  if (!lim || !Number.isFinite(lim.lower) || !Number.isFinite(lim.upper)) return

  const positions = new Float32Array((LIMIT_ARC_SEGS + 1) * 3)
  const arcGeom = new THREE.BufferGeometry()
  arcGeom.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  const arc = new THREE.Line(
    arcGeom,
    new THREE.LineBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.85 }),
  )
  // Render the arc above other geometry — a thin overlay shouldn't
  // get hidden behind the joint mesh.
  arc.renderOrder = 999
  arc.material.depthTest = false

  // Current-angle handle — sized for a comfortable grab target. This
  // sphere IS the only drag interaction for the joint (no separate
  // ring), so it needs enough hit area to be easy to land on.
  const dot = new THREE.Mesh(
    new THREE.SphereGeometry(0.022, 24, 24),
    new THREE.MeshBasicMaterial({ color: 0x67e8f9, depthTest: false }),
  )
  dot.renderOrder = 1000

  // Short red end-caps at the limit positions so "you've hit a wall"
  // is visually unambiguous.
  function makeMark () {
    return new THREE.Mesh(
      new THREE.BoxGeometry(0.01, 0.04, 0.01),
      new THREE.MeshBasicMaterial({ color: 0xef4444, depthTest: false }),
    )
  }
  const lowMark = makeMark()
  const hiMark = makeMark()
  lowMark.renderOrder = 1000
  hiMark.renderOrder = 1000

  const group = new THREE.Group()
  group.add(arc); group.add(dot); group.add(lowMark); group.add(hiMark)
  scene.add(group)
  const vis = { group, arc, dot, lowMark, hiMark, joint }
  // Replace any prior visual for this joint (e.g. URDF reload).
  const prev = limitVisuals.get(joint.name)
  if (prev) disposeLimitVisual(prev)
  limitVisuals.set(joint.name, vis)
  limitVisual = vis
  updateLimitVisual(vis)
}

// Reused scratch vectors to avoid per-frame allocations.
const _v3a = new THREE.Vector3()
const _v3b = new THREE.Vector3()
const _v3c = new THREE.Vector3()
const _qa  = new THREE.Quaternion()
const _qb  = new THREE.Quaternion()

// Compute (axisWorld, tangentWorld, bitangentWorld) for the joint's
// current rest frame in world space. The axis is invariant under the
// joint's own rotation (rotation around an axis leaves the axis
// unchanged), so we derive it from parentWorld * origQuaternion.
function jointAxisFrame (joint, out) {
  if (joint.parent) joint.parent.updateMatrixWorld(true)
  if (joint.parent) joint.parent.getWorldQuaternion(_qa)
  else _qa.identity()
  // Joint's rest orientation in world = parentWorldQ * joint.origQuaternion.
  // urdf-loader exposes origQuaternion publicly; fall back to identity
  // if it isn't present (older loader builds).
  _qb.copy(joint.origQuaternion || _qa.clone().identity())
  const restWorldQ = _qa.multiply(_qb)            // a *= b
  const axisLocal = joint.axis
  // Build a perpendicular pair in joint-local; rotating to world via
  // restWorldQ gives the arc's drawing basis.
  const up = Math.abs(axisLocal.y) > 0.9
    ? _v3a.set(1, 0, 0) : _v3a.set(0, 1, 0)
  const tangentLocal = _v3b.crossVectors(axisLocal, up).normalize()
  const bitangentLocal = _v3c.crossVectors(axisLocal, tangentLocal).normalize()
  out.axis = axisLocal.clone().applyQuaternion(restWorldQ).normalize()
  out.tangent = tangentLocal.clone().applyQuaternion(restWorldQ).normalize()
  out.bitangent = bitangentLocal.clone().applyQuaternion(restWorldQ).normalize()
}

function updateLimitVisual (target) {
  // Update a single visual when one is passed (used right after
  // build), else loop the whole cluster. The render loop calls us
  // without args every frame so all gizmos in the current cluster
  // follow their joints if the URDF moves.
  if (target === undefined) {
    for (const vis of limitVisuals.values()) updateLimitVisual(vis)
    return
  }
  if (!target) return
  const { joint, arc, dot, lowMark, hiMark } = target
  if (!joint) return
  const lim = joint.limit
  if (!lim) return

  joint.updateMatrixWorld(true)
  const center = new THREE.Vector3().setFromMatrixPosition(joint.matrixWorld)
  const frame = {}
  jointAxisFrame(joint, frame)

  // Arc points spanning [lower, upper].
  const positions = arc.geometry.attributes.position.array
  const span = lim.upper - lim.lower
  for (let i = 0; i <= LIMIT_ARC_SEGS; i++) {
    const t = i / LIMIT_ARC_SEGS
    const a = lim.lower + t * span
    const c = Math.cos(a), s = Math.sin(a)
    positions[i * 3 + 0] = center.x + LIMIT_ARC_RADIUS * (c * frame.tangent.x + s * frame.bitangent.x)
    positions[i * 3 + 1] = center.y + LIMIT_ARC_RADIUS * (c * frame.tangent.y + s * frame.bitangent.y)
    positions[i * 3 + 2] = center.z + LIMIT_ARC_RADIUS * (c * frame.tangent.z + s * frame.bitangent.z)
  }
  arc.geometry.attributes.position.needsUpdate = true
  arc.geometry.computeBoundingSphere()

  // Place an end-cap at each limit and align it to the axis so the
  // tick is perpendicular to the arc.
  function placeMark (mark, angle) {
    const c = Math.cos(angle), s = Math.sin(angle)
    mark.position.set(
      center.x + LIMIT_ARC_RADIUS * (c * frame.tangent.x + s * frame.bitangent.x),
      center.y + LIMIT_ARC_RADIUS * (c * frame.tangent.y + s * frame.bitangent.y),
      center.z + LIMIT_ARC_RADIUS * (c * frame.tangent.z + s * frame.bitangent.z),
    )
    mark.quaternion.setFromUnitVectors(_v3a.set(0, 1, 0), frame.axis)
  }
  placeMark(lowMark, lim.lower)
  placeMark(hiMark, lim.upper)

  // Current-angle handle.
  const angle = joint.angle ?? 0
  const c = Math.cos(angle), s = Math.sin(angle)
  dot.position.set(
    center.x + LIMIT_ARC_RADIUS * (c * frame.tangent.x + s * frame.bitangent.x),
    center.y + LIMIT_ARC_RADIUS * (c * frame.tangent.y + s * frame.bitangent.y),
    center.z + LIMIT_ARC_RADIUS * (c * frame.tangent.z + s * frame.bitangent.z),
  )
}

// Attach gizmos for one or more URDF joints. `jointName` marks the
// "active" one (matches the props panel's active card / receives
// kinematics first), but every name in `cluster` also gets a draggable
// arc-and-dot so multi-DOF regions (shoulder, hip, …) expose every
// axis at once. Pass null/undefined to detach.
function selectJoint (jointName, cluster = null) {
  if (!robot?.joints) return
  if (!jointName) {
    gizmoJoint = null
    handleDragging = false
    if (controls) controls.enabled = true
    clearLimitVisual()
    return
  }
  const joint = robot.joints[jointName]
  if (!joint) return
  // Build the full set fresh — joints that were in the previous
  // cluster but not in this one get torn down.
  clearLimitVisual()
  const names = Array.isArray(cluster) && cluster.length ? cluster : [jointName]
  for (const name of names) {
    const j = robot.joints[name]
    if (j) buildLimitVisual(j)
  }
  gizmoJoint = joint
  // Point `limitVisual` (drag-state target) at the active joint so a
  // drag started without hitting a specific dot defaults to it.
  limitVisual = limitVisuals.get(jointName) || null
}

function onPointerDown (e) {
  if (e.button !== 0) { pressDownPx = null; return }
  // Rig shapes render on top of everything, so a click that lands on one
  // belongs to it — check before the joint gizmo or the orbit control
  // get a say.
  const rigHit = rigHitAt(e)
  if (rigHit && beginRigDrag(e, rigHit)) {
    pressDownPx = null
    e.preventDefault?.()
    return
  }
  // First: did the operator grab a gizmo handle? Raycast against
  // every cyan dot in the current cluster; if any was hit, take
  // ownership of the gesture and use THAT joint's frame for the
  // drag — orbit stays off until pointerup.
  if (limitVisuals.size && renderer && camera) {
    const rect = renderer.domElement.getBoundingClientRect()
    ndcPointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1
    ndcPointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1
    raycaster.setFromCamera(ndcPointer, camera)
    const dots = []
    for (const vis of limitVisuals.values()) if (vis.dot) dots.push(vis.dot)
    const hits = raycaster.intersectObjects(dots, false)
    if (hits.length) {
      // Find which visual owns the hit dot, and make that the
      // active drag target so onPointerMove / onPointerUp operate
      // on its joint.
      const hitDot = hits[0].object
      let target = null
      for (const vis of limitVisuals.values()) {
        if (vis.dot === hitDot) { target = vis; break }
      }
      if (target?.joint) {
        limitVisual = target
        const joint = target.joint
        joint.updateMatrixWorld(true)
        dragCenter.setFromMatrixPosition(joint.matrixWorld)
        jointAxisFrame(joint, dragFrame)
        dragPlane.setFromNormalAndCoplanarPoint(dragFrame.axis, dragCenter)
        const startAngle = mouseAngleOnDragPlane(e)
        if (startAngle == null) return
        dragStartMouseAngle = startAngle
        dragStartJointAngle = joint.angle ?? 0
        handleDragging = true
        if (controls) controls.enabled = false
        // Suppress page-wide text selection for the duration of the
        // gizmo drag — pointer events on a canvas don't trigger text
        // selection by themselves, but if the drag continues onto an
        // overlapping HTML layer (props panel, toolbar) the browser
        // would otherwise start selecting whatever it crosses.
        document.body.style.userSelect = 'none'
        document.body.style.webkitUserSelect = 'none'
        e.preventDefault()
        e.stopPropagation()
        return
      }
    }
  }
  // Otherwise: stash the press position so pointerup can decide
  // click-vs-orbit-drag.
  pressDownPx = { x: e.clientX, y: e.clientY }
}

function onPointerMove (e) {
  if (rigDrag) { updateRigDrag(e); return }
  updateRigHover(e)
  if (!handleDragging || !limitVisual?.joint) return
  const cur = mouseAngleOnDragPlane(e)
  if (cur == null) return
  let delta = cur - dragStartMouseAngle
  while (delta > Math.PI)  delta -= 2 * Math.PI
  while (delta < -Math.PI) delta += 2 * Math.PI
  let angle = dragStartJointAngle + delta
  const lim = limitVisual.joint.limit
  if (lim && Number.isFinite(lim.lower) && Number.isFinite(lim.upper)) {
    angle = Math.max(lim.lower, Math.min(lim.upper, angle))
  }
  limitVisual.joint.setJointValue(angle)
  // Emit in the −1..+1 control domain (the model applied radians above).
  emit('joint-rotate', limitVisual.joint.name, normJoint(limitVisual.joint, angle))
}

function onPointerUp (e) {
  if (rigDrag) { endRigDrag(); return }
  if (handleDragging) {
    handleDragging = false
    if (controls) controls.enabled = true
    document.body.style.userSelect = ''
    document.body.style.webkitUserSelect = ''
    if (limitVisual?.joint) {
      emit('joint-rotate-commit', limitVisual.joint.name,
        normJoint(limitVisual.joint, limitVisual.joint.angle ?? 0))
    }
    pressDownPx = null
    e.preventDefault?.()
    e.stopPropagation?.()
    return
  }
  // Click-to-pick path: 4 px slop separates an actual click from an
  // orbit drag, so the operator can rotate the camera without
  // spurious joint selections.
  if (e.button !== 0 || !pressDownPx) return
  const dx = e.clientX - pressDownPx.x
  const dy = e.clientY - pressDownPx.y
  pressDownPx = null
  if (Math.hypot(dx, dy) > 4) return
  if (!robot || !renderer || !camera) return
  const rect = renderer.domElement.getBoundingClientRect()
  ndcPointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1
  ndcPointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1
  raycaster.setFromCamera(ndcPointer, camera)
  const hits = raycaster.intersectObject(robot, true)
  if (!hits.length) return
  // Walk the ENTIRE ancestor chain from the picked mesh to the robot
  // root, collecting every non-fixed joint along the way (ordered
  // nearest → farthest). URDF shoulders / hips / wrists are typically
  // a stack of co-located revolute joints sharing the same origin —
  // e.g. NAO's RShoulderPitch + RShoulderRoll both attach at the same
  // 3D point — so a single click on the upper arm should surface all
  // of them, not just the immediate parent joint.
  const tmpV = new THREE.Vector3()
  const chain = []
  let cur = hits[0].object
  while (cur && cur !== robot) {
    if (cur.isURDFJoint && (cur.jointType || '').toLowerCase() !== 'fixed') {
      cur.updateMatrixWorld(true)
      chain.push({
        name: cur.name,
        pos: new THREE.Vector3().setFromMatrixPosition(cur.matrixWorld),
      })
    }
    cur = cur.parent
  }
  if (!chain.length) return
  // Keep joints within 1 cm of the nearest one — that's the threshold
  // for "stacked at the same physical location". URDFs that compose a
  // shoulder out of separate pitch/roll/yaw joints land them at the
  // same origin (< 1 mm); 1 cm gives slop for slightly offset models
  // without picking up the next link's joint up the chain.
  const anchor = chain[0].pos
  const colocated = chain.filter(c => c.pos.distanceTo(anchor) < 0.01).map(c => c.name)
  // Emit (primary, alternatives) so the props panel can render a
  // chooser when more than one shares the spot.
  emit('joint-click', colocated[0], colocated)
}

function startRenderLoop () {
  const tick = () => {
    rafHandle = requestAnimationFrame(tick)
    controls?.update()
    // Keep the limit arc + handle locked onto the joint's current
    // world transform. Cheap (~64 vertex updates per frame at most)
    // and avoids any watcher overhead.
    if (limitVisual) updateLimitVisual()
    renderer?.render(scene, camera)
  }
  rafHandle = requestAnimationFrame(tick)
}

function onResize () {
  if (!container.value || !renderer) return
  const { clientWidth: w, clientHeight: h } = container.value
  if (w === 0 || h === 0) return
  camera.aspect = w / h
  camera.updateProjectionMatrix()
  renderer.setSize(w, h)
}

function makeMeshLoader (hooks = {}) {
  const manager = new THREE.LoadingManager()
  return (path, _manager, onComplete) => {
    // Mesh geometry loads asynchronously; hooks let loadUrdf wait for all
    // of them before inspecting meshes for collision.
    hooks.onStart?.()
    const settle = (obj) => { onComplete(obj); hooks.onSettle?.() }
    // Path-preserving resolution (see utils/meshUrl.js for the
    // johnny5 same-basename story).
    const url = resolveMeshUrl(path, props.urdfUrl, props.meshesBase)
    const ext = url.split('.').pop().split('?')[0].toLowerCase()
    let loader
    if (ext === 'stl') loader = new STLLoader(manager)
    else if (ext === 'dae') loader = new ColladaLoader(manager)
    else if (ext === 'obj') loader = new OBJLoader(manager)
    else if (ext === 'gltf' || ext === 'glb') loader = new GLTFLoader(manager)
    else {
      settle(new THREE.Object3D())
      return
    }
    loader.load(
      url,
      (result) => {
        let object
        if (ext === 'stl') {
          // STLLoader returns a BufferGeometry, not an Object3D.
          const mat = new THREE.MeshStandardMaterial({
            color: 0x94a3b8, metalness: 0.1, roughness: 0.6,
          })
          object = new THREE.Mesh(result, mat)
        } else if (ext === 'gltf' || ext === 'glb') {
          object = result.scene || result.scenes?.[0]
        } else if (ext === 'dae') {
          object = result.scene
        } else {
          object = result
        }
        settle(object)
      },
      undefined,
      (err) => {
        // Don't blow up the whole model on a missing mesh — render the
        // joint without it and surface the error so the operator can
        // figure out which file is missing from their bundle.
        // eslint-disable-next-line no-console
        console.warn('Mesh load failed', url, err)
        settle(new THREE.Object3D())
      },
    )
  }
}

function clearRobot () {
  // Rig shapes are children of robot links, so they have to come off
  // before the tree is disposed — and their geometry is shared from a
  // cache, which disposeObject3D would free once per shape using it.
  clearRigShapes()
  // Before disposeObject3D: it walks the tree disposing geometry, and
  // edge geometry is cached across meshes, so a shared EdgesGeometry
  // would be disposed once per mesh referencing it.
  clearEdges()
  disposeEdgeCache()
  if (robot) {
    scene?.remove(robot)
    disposeObject3D(robot)
    robot = null
  }
  // Drop stale collision/highlight references tied to the old robot. Don't
  // disposeObject3D(robotClone) — it shares geometry with the (separately
  // disposed) display robot; just release the ref.
  colliders = []
  acm = null
  highlighted = []
  robotClone = null
  cloneColliders = []
  scanGeneration += 1 // cancel any in-flight scan
  acmGeneration += 1  // and any in-flight ACM build
}

async function loadUrdf () {
  if (!props.urdfUrl) {
    clearRobot()
    return
  }
  loading.value = true
  loadError.value = ''
  try {
    // Track async mesh loads: loadMeshCb fires onStart per mesh during parse
    // and onSettle when each geometry arrives. We must wait for ALL of them
    // before building colliders — otherwise the <collision> meshes have no
    // geometry yet and the collision set comes up empty.
    let pending = 0
    let parseDone = false
    let resolveAll
    const allMeshesLoaded = new Promise((r) => { resolveAll = r })
    const settleIfDone = () => { if (parseDone && pending === 0) resolveAll() }

    const loader = new URDFLoader()
    loader.loadMeshCb = makeMeshLoader({
      onStart: () => { pending += 1 },
      onSettle: () => { pending -= 1; settleIfDone() },
    })
    // Parse <collision> too so the overlay + collision detection have geometry.
    // (Heavier load — these models can ship full-detail meshes, each an
    // extra HTTP fetch + parse. Skipped entirely for preview-only embeds.)
    loader.parseCollision = props.collision
    const result = await new Promise((resolve, reject) => {
      loader.load(props.urdfUrl, resolve, undefined, reject)
    })
    clearRobot()
    robot = result
    // Most URDFs are authored Z-up; three.js is Y-up. Rotate the root
    // so the robot stands on the grid rather than lying flat.
    robot.rotation.x = -Math.PI / 2
    scene.add(robot)

    // All loadMeshCb calls were issued synchronously during parse. The
    // settle-wait exists for ONE reason: colliders built before every
    // <collision> mesh has arrived come up empty. Visual meshes need no
    // waiting — three renders each one the moment it lands — so a
    // preview-only embed skips this entirely and the joints/loaded
    // events fire as soon as the parse is done.
    parseDone = true
    settleIfDone()
    if (props.collision) {
      await Promise.race([allMeshesLoaded, new Promise((r) => setTimeout(r, 15000))])
    }

    robot.updateMatrixWorld(true)

    // The robot is usable NOW — tell the parent and drop the loading
    // overlay before any collision prep. The ACM takes seconds of
    // (time-sliced) sampling on a real model, and collision reporting is
    // the only thing that needs it; leaving the "Loading robot model…"
    // spinner up while a fully-rendered, poseable robot sat behind it
    // read as a hang.
    const jointNames = Object.keys(robot.joints || {})
    emit('joints', jointNames)
    emit('loaded', robot)
    loading.value = false

    if (props.collision) {
      applyColliders()
      // Build colliders (convex-hull-simplified) + the ACM at the
      // freshly-loaded home pose, before any parent posing.
      const cb = buildColliders(robot, { simplify: 'hull' })
      colliders = cb.colliders
      // Invisible FK sandbox for the async timeline scan. Cheap second pass:
      // clone(true) shares geometry, and collision.js caches hulls + BVHs
      // per source geometry, so nothing heavyweight is rebuilt here.
      robotClone = robot.clone(true)
      robotClone.updateMatrixWorld(true)
      cloneColliders = buildColliders(robotClone, { simplify: 'hull' }).colliders

      const _ta = performance.now()
      const builtAcm = await buildAcm(cloneColliders, computeAdjacency(robotClone))
      if (builtAcm === null) return // superseded by a newer load — its build owns state now
      acm = builtAcm
      const _acmMs = performance.now() - _ta
      if (import.meta.env?.DEV) {
        // eslint-disable-next-line no-console
        console.info(`[collision] ${colliders.length} collider meshes; ` +
          `tris ${cb.stats.origTris}→${cb.stats.bvhTris} (hull ${cb.stats.buildMs | 0}ms); ` +
          `ACM ${acm.size} pairs (built ${_acmMs | 0}ms, time-sliced)`)
      }
    }
    // Outline pass last: the clone above is made with clone(true), so
    // building edges earlier would give the invisible FK sandbox a
    // duplicate set of line segments for nothing.
    buildEdges()
  } catch (e) {
    loadError.value = e?.message || String(e)
    emit('load-error', e)
  } finally {
    loading.value = false
  }
}

// ── Control-value normalization (home-centered, convention C) ────────
//
// SaintOS drives joints in a −1..+1 control range (the same range the
// servos use). We map that range onto each joint's URDF <limit>, pinned
// so the URDF home (θ=0) is control 0:
//
//   −1 → lower limit      0 → home (θ=0)      +1 → upper limit
//
// Each side is scaled independently (θ = n·upper for n≥0, n·|lower| for
// n<0), so asymmetric joints keep 0 at home — at the cost of a different
// gain per side. One-sided joints (lower=0) have no negative travel, so
// negative control just holds at home. All derived from the URDF limits;
// no extra metadata. Joints without a finite limit pass through as-is.
function jointLimits (joint) {
  const lim = joint?.limit
  const lo = lim ? Number(lim.lower) : NaN
  const hi = lim ? Number(lim.upper) : NaN
  return Number.isFinite(lo) && Number.isFinite(hi) ? { lo, hi } : null
}

// control (−1..+1) → joint value (radians / metres)
function denormJoint (joint, n) {
  const L = jointLimits(joint)
  if (!L) return n
  const c = Math.max(-1, Math.min(1, Number(n) || 0))
  const theta = c >= 0 ? c * L.hi : c * Math.abs(L.lo)
  return Math.max(L.lo, Math.min(L.hi, theta))
}

// joint value (radians / metres) → control (−1..+1)
function normJoint (joint, theta) {
  const L = jointLimits(joint)
  if (!L) return theta
  const t = Number(theta) || 0
  if (t >= 0) return L.hi > 0 ? Math.min(1, t / L.hi) : 0
  return L.lo < 0 ? Math.max(-1, t / Math.abs(L.lo)) : 0
}

// Public-ish API: parent calls setJointValue from a ref with a −1..+1
// control value; we denormalize to the joint's native units for display.
function setJointValue (jointName, value) {
  const j = robot?.joints?.[jointName]
  if (!j) return false
  j.setJointValue(denormJoint(j, value))
  return true
}

// ── Control rig shapes ──────────────────────────────────────────────
//
// One shape per rig control, parented to the URDF link the control is
// anchored to, so it follows the pose with no per-frame bookkeeping.
// The Unreal Control Rig arrangement: a shape hung off a bone, grabbed
// in the viewport rather than driven from a panel.
//
// Dragging maps pointer motion onto the widget's declared `axis`
// (projected to screen) and reports a 0..1 fraction of that axis; the
// PARENT turns that into a control value and pushes it through the same
// evaluate call the side panel uses. Keeping the value math out of here
// is what stops the viewport and the panel from disagreeing.

const showRig = ref(true)
// Reactive shape count for the toolbar. `rigShapes` below is a plain Map
// on purpose — it's read on every pointer move and doesn't want proxy
// overhead — so the template watches this instead of its .size.
const rigShapeCount = ref(0)
// control name → THREE.Group. Kept so highlight/visibility/teardown can
// address a control without walking the scene.
const rigShapes = new Map()
let rigControlsById = new Map()      // control name → parsed control
let rigHovered = null
let rigDrag = null
// Controls the evaluator reported it can't handle. Their shapes are drawn
// as inert wireframes and kept OUT of the hit list, so a drag falls
// through to the orbit control instead of dead-ending on a handle that
// does nothing.
let rigInert = new Set()

function clearRigShapes () {
  for (const group of rigShapes.values()) {
    group.parent?.remove(group)
    group.userData?.mesh?.material?.dispose()
  }
  rigShapes.clear()
  rigShapeCount.value = 0
  rigHovered = null
  rigDrag = null
}

/**
 * Attach shapes for a rig.
 *
 * @param {object} rig      parsed rig ({ controls: [...] })
 * @param {object} anchors  control name → URDF link name
 */
function setRigControls (rig, anchors = {}, inertNames = []) {
  clearRigShapes()
  rigControlsById = new Map()
  rigInert = new Set(inertNames || [])
  if (!robot || !rig?.controls?.length) return

  for (const control of rig.controls) {
    rigControlsById.set(control.name, control)
    if (control.widget?.visible === false) continue
    const linkName = anchors[control.name] || control.anchor
    // No anchor means we have nowhere to put it. Skipping beats parking
    // it at the origin, where a cluster of unrelated handles piles up on
    // the robot's base and reads as a bug.
    if (!linkName) continue
    const link = robot.links?.[linkName] || robot.frames?.[linkName]
    if (!link) continue

    const group = buildControlShape(control, {
      inert: rigInert.has(control.name),
    })
    group.visible = showRig.value
    link.add(group)
    rigShapes.set(control.name, group)
  }
  rigShapeCount.value = rigShapes.size
  lastRigArgs = { rig, anchors }
  robot.updateMatrixWorld(true)
}

function setRigVisible (on) {
  showRig.value = !!on
  for (const group of rigShapes.values()) group.visible = showRig.value
  if (!showRig.value) {
    rigHovered = null
    rigDrag = null
  }
}

function toggleRig () { setRigVisible(!showRig.value) }

/** Mark one control's shape as selected, e.g. from the side panel. */
function highlightRigControl (name) {
  for (const [key, group] of rigShapes) {
    setShapeHighlight(group, key === name ? 'active' : null)
  }
}

function rigShapeMeshes () {
  const out = []
  for (const group of rigShapes.values()) {
    if (!group.visible || group.userData?.inert) continue
    if (group.userData?.mesh) out.push(group.userData.mesh)
  }
  return out
}

/**
 * Update which controls are inert without rebuilding everything.
 *
 * The inert set arrives from the first evaluate, which lands AFTER the
 * shapes are built, so rebuilding is the simplest correct answer — and
 * it only happens when the set actually changes, which is essentially
 * once per rig load.
 */
function setRigInert (names) {
  const next = new Set(names || [])
  if (next.size === rigInert.size && [...next].every(n => rigInert.has(n))) return
  rigInert = next
  if (lastRigArgs) {
    setRigControls(lastRigArgs.rig, lastRigArgs.anchors, [...rigInert])
  }
}

// Remembered so setRigInert can rebuild with the same rig + anchors.
let lastRigArgs = null

function rigHitAt (e) {
  if (!showRig.value || !rigShapes.size || !renderer || !camera) return null
  const rect = renderer.domElement.getBoundingClientRect()
  ndcPointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1
  ndcPointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1
  raycaster.setFromCamera(ndcPointer, camera)
  const hits = raycaster.intersectObjects(rigShapeMeshes(), false)
  if (!hits.length) return null
  const group = hits[0].object.parent
  const name = group?.userData?.rigControl
  return name ? { name, group } : null
}

/**
 * Begin a rig drag. Returns true if we took the gesture.
 *
 * The drag frame is computed once here: the widget's axis in world
 * space, projected to a screen-space direction. Pointer movement is then
 * a simple dot product against it, which behaves the same regardless of
 * how the camera is oriented — including when the axis points nearly at
 * the camera, where the projection shortens and the control just gets
 * less sensitive rather than inverting.
 */
function beginRigDrag (e, hit) {
  const control = rigControlsById.get(hit.name)
  if (!control) return false
  hit.group.updateMatrixWorld(true)

  const origin = new THREE.Vector3().setFromMatrixPosition(hit.group.matrixWorld)
  const axisWorld = worldDragAxis(hit.group, control)
  const tip = origin.clone().add(axisWorld)

  const rect = renderer.domElement.getBoundingClientRect()
  const toScreen = (v) => {
    const p = v.clone().project(camera)
    return new THREE.Vector2(
      (p.x * 0.5 + 0.5) * rect.width,
      (-p.y * 0.5 + 0.5) * rect.height)
  }
  const screenDir = toScreen(tip).sub(toScreen(origin))
  const len = screenDir.length()
  // Axis pointing straight at the camera: there's no screen direction to
  // drag along, so fall back to horizontal rather than dividing by ~0
  // and sending the value to infinity on the first pixel.
  if (len < 1e-3) screenDir.set(1, 0)
  else screenDir.divideScalar(len)

  rigDrag = {
    name: hit.name,
    group: hit.group,
    kind: control.kind,
    startX: e.clientX,
    startY: e.clientY,
    screenDir,
    invertY: !!control.widget?.invert_y,
  }
  setShapeHighlight(hit.group, 'active')
  if (controls) controls.enabled = false
  setBodySelectNoneForRig(true)
  emit('rig-control-press', hit.name)
  return true
}

// Pixels of drag for one full sweep of a control's range. Generous on
// purpose: a control shape is small on screen and a twitchy handle is
// worse than a slow one.
const RIG_DRAG_PX = 260

function updateRigDrag (e) {
  if (!rigDrag) return
  const dx = e.clientX - rigDrag.startX
  const dy = e.clientY - rigDrag.startY

  if (rigDrag.kind === 'pad') {
    // Two axes, screen-aligned: a pad is inherently a 2D screen gesture,
    // and forcing it through the widget's single axis would lose one.
    const fy = (rigDrag.invertY ? -dy : dy) / RIG_DRAG_PX
    emit('rig-control-drag', {
      name: rigDrag.name,
      deltaX: dx / RIG_DRAG_PX,
      deltaY: -fy,
    })
    return
  }
  // Channel / spatial: project onto the widget's declared axis.
  const along = (dx * rigDrag.screenDir.x + dy * rigDrag.screenDir.y) / RIG_DRAG_PX
  emit('rig-control-drag', { name: rigDrag.name, delta: along })
}

function endRigDrag () {
  if (!rigDrag) return
  setShapeHighlight(rigDrag.group, null)
  emit('rig-control-release', rigDrag.name)
  rigDrag = null
  if (controls) controls.enabled = true
  setBodySelectNoneForRig(false)
}

function setBodySelectNoneForRig (on) {
  if (typeof document === 'undefined') return
  document.body.style.userSelect = on ? 'none' : ''
}

function updateRigHover (e) {
  if (rigDrag) return
  const hit = rigHitAt(e)
  const name = hit?.name || null
  if (name === rigHovered) return
  if (rigHovered && rigShapes.has(rigHovered)) {
    setShapeHighlight(rigShapes.get(rigHovered), null)
  }
  rigHovered = name
  if (name) setShapeHighlight(rigShapes.get(name), 'hover')
  if (renderer) {
    renderer.domElement.style.cursor = name ? 'grab' : ''
  }
}

// ── Edge overlay ────────────────────────────────────────────────────
//
// Draws each mesh's hard edges as line segments on top of the shaded
// surface. Without it, adjacent links sharing one material read as a
// single blob — the demo head is a dozen boxes and cylinders in the same
// grey, and you genuinely cannot tell where the jaw stops and the skull
// starts. Edges make the articulation legible, which is the whole point
// of a rig preview.
//
// Implementation notes that matter:
//
//  * `thresholdAngle` is what keeps this from being a wireframe. At 24°
//    a box shows its 12 edges and a cylinder shows its two rims and
//    silhouette, but the ~15° facets of a smooth sphere stay quiet.
//  * EdgesGeometry is cached per SOURCE geometry. urdf-loader reuses one
//    geometry across repeated links, and a real model (johnny5) has
//    hundreds of meshes — rebuilding per instance is the difference
//    between instant and a visible stall.
//  * High-poly meshes are skipped. Edge extraction is O(tris) and a
//    200k-triangle scan mesh produces an unreadable hairball anyway.
//  * The lines are CHILDREN of their mesh, so they inherit joint
//    transforms for free and pose correctly with no per-frame work.

const showEdges = ref(true)

// Above this triangle count, skip edge extraction: too slow to build and
// too dense to read.
const EDGE_TRI_BUDGET = 60000
const EDGE_THRESHOLD_DEG = 24

let edgeLines = []                  // LineSegments we added, for disposal
let edgeMaterial = null
const edgeGeomCache = new Map()     // source geometry uuid → EdgesGeometry

// Black, deliberately, and the same in both themes. An inked outline
// reads as a seam between parts rather than as a glowing wireframe over
// them — which is what makes the articulation legible instead of just
// busy. It's theme-independent for the same reason: the line is standing
// in for a physical gap, not for UI chrome.
const EDGE_COLOR = 0x000000

function ensureEdgeMaterial () {
  if (edgeMaterial) return edgeMaterial
  // Opaque: a solid line is crisper than a blended one, and staying out
  // of the transparent pass avoids sort-order artifacts against the
  // surfaces it outlines. (polygonOffset is deliberately absent — it
  // only affects polygon rasterization, so it does nothing for
  // LineSegments; EdgesGeometry lines sit on creases and silhouettes
  // where z-fighting isn't a problem in practice.)
  edgeMaterial = new THREE.LineBasicMaterial({ color: EDGE_COLOR })
  return edgeMaterial
}

function edgeGeometryFor (geometry) {
  if (!geometry?.attributes?.position) return null
  const cached = edgeGeomCache.get(geometry.uuid)
  if (cached !== undefined) return cached

  const idx = geometry.index
  const tris = (idx ? idx.count : geometry.attributes.position.count) / 3
  if (tris > EDGE_TRI_BUDGET) {
    edgeGeomCache.set(geometry.uuid, null)   // cache the refusal too
    return null
  }
  let edges = null
  try {
    edges = new THREE.EdgesGeometry(geometry, EDGE_THRESHOLD_DEG)
  } catch (e) {
    if (import.meta.env?.DEV) console.warn('[viewer] EdgesGeometry failed:', e)
    edges = null
  }
  edgeGeomCache.set(geometry.uuid, edges)
  return edges
}

function buildEdges () {
  clearEdges()
  if (!robot) return

  // Collect first, THEN add. Object3D.traverse walks a live children
  // array, so adding children mid-traverse visits the new nodes as well.
  const meshes = []
  robot.traverse((o) => {
    if (o.isMesh && o.geometry && !o.userData.__isEdgeLine) meshes.push(o)
  })

  const mat = ensureEdgeMaterial()
  for (const mesh of meshes) {
    const geom = edgeGeometryFor(mesh.geometry)
    if (!geom) continue
    const line = new THREE.LineSegments(geom, mat)
    line.userData.__isEdgeLine = true
    // Never let the outline participate in raycasts — clicking an edge
    // must select the joint, exactly as clicking the surface does.
    line.raycast = () => {}
    line.visible = showEdges.value
    mesh.add(line)
    edgeLines.push(line)
  }
}

function clearEdges () {
  for (const line of edgeLines) line.parent?.remove(line)
  edgeLines = []
  // Geometry stays in the cache — it's keyed by source geometry and
  // survives a rebuild. clearRobot() disposes it.
}

function disposeEdgeCache () {
  for (const geom of edgeGeomCache.values()) geom?.dispose?.()
  edgeGeomCache.clear()
  edgeMaterial?.dispose()
  edgeMaterial = null
}

function toggleEdges () {
  showEdges.value = !showEdges.value
  for (const line of edgeLines) line.visible = showEdges.value
}

// ── Toolbar controls (Center / Grid / Edges / Collision / Views) ─────

function toggleGrid () {
  showGrid.value = !showGrid.value
  if (grid) grid.visible = showGrid.value
}

// Translucent overlay material for collision shapes so they read over the
// visual mesh (depthWrite off keeps them from z-fighting the surface).
const COLLISION_MAT = new THREE.MeshStandardMaterial({
  color: 0xff3b30, transparent: true, opacity: 0.35,
  depthWrite: false, metalness: 0, roughness: 1,
})

// urdf-loader tags collision subtrees with `isURDFCollider`. Recolor their
// meshes to the overlay material and gate visibility on the toggle.
function applyColliders () {
  if (!robot) return
  robot.traverse((o) => {
    if (!o.isURDFCollider) return
    o.visible = showCollision.value
    o.traverse((m) => { if (m.isMesh) m.material = COLLISION_MAT })
  })
}

function toggleCollision () {
  showCollision.value = !showCollision.value
  applyColliders()
}

// Build the Allowed-Collision Matrix on the clone: ignore tree-adjacent pairs,
// pairs colliding at rest, and pairs colliding in ~all random samples (design
// nesting).
//
// Async and frame-budgeted, NOT run to completion in one go: 250 random-pose
// passes over a full CAD model is seconds of work, and the original
// synchronous version froze the whole page right after URDF upload. Yielding
// is by elapsed time per slice rather than a fixed sample count, because
// per-sample cost varies wildly with the pose. A load that supersedes this
// build (scanGeneration bump) aborts it — returning null.
async function buildAcm (cols, adjacency, samples = 250, alwaysFrac = 0.9) {
  const out = new Set(adjacency)
  if (!robotClone || !cols.length) return out
  const gen = acmGeneration
  const FRAME_BUDGET_MS = 10
  const ranged = Object.keys(robotClone.joints).filter((n) => {
    const l = robotClone.joints[n]?.limit
    return l && Number.isFinite(l.lower) && Number.isFinite(l.upper)
  })
  robotClone.updateMatrixWorld(true)
  for (const p of collidingPairs(cols, null)) out.add(p) // at-rest
  const counts = new Map()
  let sliceStart = performance.now()
  for (let s = 0; s < samples; s++) {
    for (const n of ranged) {
      const l = robotClone.joints[n].limit
      robotClone.joints[n].setJointValue(l.lower + Math.random() * (l.upper - l.lower))
    }
    robotClone.updateMatrixWorld(true)
    // Pairs already allowed (adjacent or touching at rest) skip the
    // narrowphase entirely — they're the ones most likely to overlap in
    // every sample, so pruning them here is where the time goes.
    for (const p of collidingPairs(cols, out)) counts.set(p, (counts.get(p) || 0) + 1)
    if (performance.now() - sliceStart > FRAME_BUDGET_MS) {
      await rafYield()
      if (gen !== acmGeneration) return null // superseded by a new load
      sliceStart = performance.now()
    }
  }
  const thr = alwaysFrac * samples
  for (const [p, c] of counts) if (c >= thr) out.add(p) // ~always → by design
  for (const n of ranged) robotClone.joints[n].setJointValue(0) // restore home
  robotClone.updateMatrixWorld(true)
  return out
}

// Self-collision at the CURRENT pose → array of "linkA|linkB" pair keys
// (ACM pairs excluded). Cheap; call on pose change.
//
// Reports nothing until the ACM exists. Now that the ACM builds
// asynchronously after load, this IS reachable in that window — and
// running it un-pruned would light every adjacent pair red (false
// positives) at several times the cost.
function collisionsAtCurrent () {
  if (!robot || !colliders.length || !acm) return []
  robot.updateMatrixWorld(true)
  return [...collidingPairs(colliders, acm)]
}

// Scan an animation for collisions, WITHOUT blocking the UI. `sampleFn(t)`
// returns a map { jointName: controlValue(−1..1) } for time t. Runs on the
// invisible clone (so it never disturbs the visible model or fights the user's
// scrub/orbit) and time-slices across animation frames — a few samples per
// frame, then yields so rendering + input stay live. Returns a Promise of
// merged intervals, or null if a newer scan superseded this one.
function rafYield () {
  return new Promise((resolve) => {
    if (typeof requestAnimationFrame === 'function') requestAnimationFrame(() => resolve())
    else setTimeout(resolve, 0)
  })
}

async function scanTimeline (sampleFn, duration, steps = 90) {
  if (!robotClone || !cloneColliders.length || !(duration > 0)) return []
  // Null (not []) while the ACM is still building: [] would tell the
  // editor "scanned, clean" and it would never rescan, while null routes
  // through its aborted-scan path and retries after the next idle. This
  // also keeps the scan from posing the clone WHILE buildAcm is posing
  // it — they share the same FK sandbox.
  if (!acm) return null
  const gen = ++scanGeneration
  const n = Math.max(2, Math.min(600, Math.round(steps)))
  const SLICE = 6 // samples processed per frame before yielding
  const samples = []
  for (let k = 0; k <= n; k++) {
    const t = (duration * k) / n
    const vals = sampleFn(t) || {}
    for (const name in vals) {
      const j = robotClone.joints[name]
      if (j) j.setJointValue(denormJoint(j, vals[name]))
    }
    robotClone.updateMatrixWorld(true)
    samples.push({ t, pairs: collidingPairs(cloneColliders, acm) })
    if (k % SLICE === SLICE - 1) {
      await rafYield()
      if (gen !== scanGeneration) return null // a newer scan started; bail
    }
  }
  return samplesToIntervals(samples)
}

// Abort any in-flight scan immediately. Called by the parent on scrub/edit,
// and internally on view manipulation, so interaction never competes with the
// collision reprocess for the main thread.
function cancelScan () { scanGeneration += 1 }

// Canvas input signals (orbit / zoom / drag). Each aborts an in-flight scan
// and tells the parent to (re)start its idle countdown, so reprocessing only
// runs once the user pauses.
let _pointerDown = false
let _lastInteractAt = 0
function notifyInteract () {
  cancelScan()
  emit('interact')
}
function notifyInteractThrottled () {
  const now = (typeof performance !== 'undefined') ? performance.now() : 0
  if (now - _lastInteractAt < 60) return
  _lastInteractAt = now
  notifyInteract()
}
function onInteractStart () { _pointerDown = true; notifyInteract() }
function onInteractMove () { if (_pointerDown) notifyInteractThrottled() }
function onInteractEnd () { _pointerDown = false; notifyInteract() }
function onInteractWheel () { notifyInteractThrottled() }

// Solid red material for parts currently in collision (distinct from the
// translucent collision-geometry overlay).
const HIGHLIGHT_MAT = new THREE.MeshStandardMaterial({
  color: 0xff3b30, emissive: 0x4c0000, metalness: 0, roughness: 0.7,
})

function clearCollisionHighlight () {
  for (const h of highlighted) h.mesh.material = h.material
  highlighted = []
}

// Tint the VISUAL meshes of the links named in `pairs` (["linkA|linkB", …])
// red. Only each link's OWN visuals — not its colliders, not descendant
// links (those are separate URDFLink nodes, highlighted only if named).
function highlightCollision (pairs) {
  clearCollisionHighlight()
  if (!robot || !pairs || !pairs.length) return
  const links = new Set()
  for (const p of pairs) for (const n of String(p).split('|')) links.add(n)
  robot.traverse((o) => {
    if (!o.isURDFLink || !links.has(o.name)) return
    for (const child of o.children) {
      if (!child.isURDFVisual) continue
      child.traverse((m) => {
        if (!m.isMesh) return
        highlighted.push({ mesh: m, material: m.material })
        m.material = HIGHLIGHT_MAT
      })
    }
  })
}

// Frame the robot so it fits the viewport with a small margin. Keeps
// the camera's current direction (so "Center" doesn't also reorient)
// — preset views call this AFTER positioning to dial in the distance.
function centerOnRobot (opts = {}) {
  if (!camera || !controls) return
  const target = opts.target || new THREE.Vector3()
  let size = 1
  if (robot) {
    const box = new THREE.Box3().setFromObject(robot)
    if (!box.isEmpty()) {
      box.getCenter(target)
      const dims = new THREE.Vector3()
      box.getSize(dims)
      size = Math.max(dims.x, dims.y, dims.z)
    }
  }
  // Distance such that the bounding sphere fits inside the vertical
  // FOV with ~30% padding. fov is in degrees on PerspectiveCamera.
  const fitOffset = 1.3
  const halfFov = THREE.MathUtils.degToRad(camera.fov) / 2
  const distance = (size * 0.5) / Math.tan(halfFov) * fitOffset
  // If no direction was provided, preserve the current viewing dir.
  const dir = opts.dir
    ? opts.dir.clone().normalize()
    : camera.position.clone().sub(controls.target).normalize()
  camera.position.copy(target).addScaledVector(dir, distance)
  controls.target.copy(target)
  camera.near = Math.max(0.001, distance / 100)
  camera.far  = distance * 100
  camera.updateProjectionMatrix()
  controls.update()
}

// Canonical view directions in world space. Camera position = target
// + (dir * distance), so `dir` is the unit vector pointing FROM the
// target TOWARD the camera. World axes after the URDF Z-up → Y-up
// reorient: +X = robot's right, +Y = up, +Z = robot's front.
const VIEW_DIRS = {
  front:     new THREE.Vector3( 0,  0,  1),
  back:      new THREE.Vector3( 0,  0, -1),
  right:     new THREE.Vector3( 1,  0,  0),
  left:      new THREE.Vector3(-1,  0,  0),
  top:       new THREE.Vector3( 0,  1,  0.001),   // ε offset so OrbitControls' up vector doesn't go singular
  bottom:    new THREE.Vector3( 0, -1,  0.001),
  isometric: new THREE.Vector3( 1,  1,  1),
}
const VIEW_LABELS = [
  ['front',     'Front'],
  ['back',      'Back'],
  ['left',      'Left'],
  ['right',     'Right'],
  ['top',       'Top'],
  ['bottom',    'Bottom'],
  ['isometric', 'Isometric'],
]
function setView (key) {
  const dir = VIEW_DIRS[key]
  if (!dir) return
  centerOnRobot({ dir })
  viewMenuOpen.value = false
}

// Close the view dropdown when clicking anywhere outside it. The
// menu's own buttons stop propagation, so this only fires for
// clicks that didn't land on the menu.
function onDocClickForMenu () {
  viewMenuOpen.value = false
}
watch(viewMenuOpen, (open) => {
  if (open) {
    // mousedown (not click) so we close before the user's second
    // click reopens it via the toggle button.
    setTimeout(() => document.addEventListener('mousedown', onDocClickForMenu), 0)
  } else {
    document.removeEventListener('mousedown', onDocClickForMenu)
  }
})

defineExpose({
  setRigControls, setRigVisible, setRigInert, toggleRig,
  highlightRigControl, showRig,
  setJointValue, selectJoint, collisionsAtCurrent, scanTimeline, highlightCollision, cancelScan })

onMounted(() => {
  setupScene()
  startRenderLoop()
  if (typeof ResizeObserver !== 'undefined') {
    resizeObserver = new ResizeObserver(onResize)
    resizeObserver.observe(container.value)
  } else {
    window.addEventListener('resize', onResize)
  }
  loadUrdf()
})

onBeforeUnmount(() => {
  if (rafHandle) cancelAnimationFrame(rafHandle)
  clearRigShapes()
  disposeRigShapeCache()
  clearEdges()
  disposeEdgeCache()
  resizeObserver?.disconnect()
  window.removeEventListener('resize', onResize)
  if (renderer?.domElement) {
    renderer.domElement.removeEventListener('pointerdown', onPointerDown)
    renderer.domElement.removeEventListener('pointermove', onPointerMove)
    renderer.domElement.removeEventListener('pointerup', onPointerUp)
    renderer.domElement.removeEventListener('pointerleave', onPointerUp)
    renderer.domElement.removeEventListener('pointerdown', onInteractStart)
    renderer.domElement.removeEventListener('pointermove', onInteractMove)
    renderer.domElement.removeEventListener('pointerup', onInteractEnd)
    renderer.domElement.removeEventListener('pointerleave', onInteractEnd)
    renderer.domElement.removeEventListener('wheel', onInteractWheel)
  }
  gizmoJoint = null
  handleDragging = false
  clearLimitVisual()
  clearRobot()
  controls?.dispose()
  renderer?.dispose()
  if (renderer?.domElement?.parentNode) {
    renderer.domElement.parentNode.removeChild(renderer.domElement)
  }
  scene = null
  camera = null
  renderer = null
  controls = null
})

watch(() => props.urdfUrl, () => loadUrdf())

// Re-paint the scene + grid when the operator changes the theme.
// We replace the grid object outright since GridHelper materials
// don't expose a public re-color API.
watch(() => display.theme, () => {
  if (!scene) return
  scene.background = new THREE.Color(cssColor('--color-canvas', 0x0f172a))
  if (grid) {
    const oldVisible = grid.visible
    scene.remove(grid)
    grid.material?.dispose?.()
    grid.geometry?.dispose?.()
    grid = new THREE.GridHelper(
      2, 20,
      cssColor('--color-surface', 0x334155),
      cssColor('--color-line-subtle', 0x1e293b),
    )
    grid.material.opacity = 0.7
    grid.material.transparent = true
    grid.visible = oldVisible
    scene.add(grid)
  }
})
</script>

<template>
  <div class="relative w-full overflow-hidden rounded-lg bg-canvas" :style="{ height }">
    <div ref="container" class="absolute inset-0"></div>

    <!-- Top-right toolbar: Center · Grid · View dropdown. Sits above
         the canvas with pointer-events-none on the wrapper so it
         can't intercept orbit drags outside the buttons themselves. -->
    <div class="viewer-toolbar pointer-events-none absolute top-2 right-2 flex items-start gap-1"
         v-if="urdfUrl && !loading">
      <button class="viewer-btn pointer-events-auto"
              title="Fit model in view"
              @click="centerOnRobot()">
        <span class="material-icons icon-sm">center_focus_strong</span>
      </button>
      <button class="viewer-btn pointer-events-auto"
              :title="showGrid ? 'Hide grid' : 'Show grid'"
              @click="toggleGrid">
        <span class="material-icons icon-sm">{{ showGrid ? 'grid_on' : 'grid_off' }}</span>
      </button>
      <!-- Control rig overlay. Hidden entirely when no rig is loaded, so
           the button never offers to toggle nothing. -->
      <button v-if="rigShapeCount"
              class="viewer-btn pointer-events-auto"
              :class="{ 'viewer-btn-active': showRig }"
              :title="showRig
                ? 'Hide control rig'
                : `Show control rig (${rigShapeCount} controls)`"
              @click="toggleRig">
        <span class="material-icons icon-sm">
          {{ showRig ? 'gamepad' : 'radio_button_unchecked' }}
        </span>
      </button>
      <!-- Edge overlay. On by default: links sharing one material read as
           a single blob without it, and telling them apart is the point
           of a rig preview. -->
      <button class="viewer-btn pointer-events-auto"
              :class="{ 'viewer-btn-active': showEdges }"
              :title="showEdges ? 'Hide edges' : 'Show edges'"
              @click="toggleEdges">
        <span class="material-icons icon-sm">{{ showEdges ? 'deselect' : 'select_all' }}</span>
      </button>
      <!-- Hidden entirely on preview-only embeds: collision geometry was
           never parsed, so the toggle would flip an empty overlay. -->
      <button v-if="props.collision"
              class="viewer-btn pointer-events-auto"
              :class="{ 'viewer-btn-active': showCollision }"
              :title="showCollision ? 'Hide collision geometry' : 'Show collision geometry'"
              @click="toggleCollision">
        <span class="material-icons icon-sm">{{ showCollision ? 'deblur' : 'blur_on' }}</span>
      </button>
      <div class="relative pointer-events-auto">
        <button class="viewer-btn"
                title="Set camera view"
                @mousedown.stop
                @click.stop="viewMenuOpen = !viewMenuOpen">
          <span class="material-icons icon-sm">3d_rotation</span>
          <span class="material-icons icon-sm">arrow_drop_down</span>
        </button>
        <div v-if="viewMenuOpen" class="viewer-menu" @mousedown.stop>
          <button v-for="[key, label] in VIEW_LABELS" :key="key"
                  class="viewer-menu-item"
                  @click="setView(key)">
            {{ label }}
          </button>
        </div>
      </div>
    </div>

    <div v-if="loading" class="absolute inset-0 flex items-center justify-center bg-canvas/70 text-sm text-fg">
      <span class="material-icons icon-sm animate-spin mr-2">progress_activity</span>
      Loading robot model…
    </div>
    <div v-else-if="loadError" class="absolute inset-x-0 bottom-0 p-3 bg-red-500/20 border-t border-red-500/40 text-sm text-red-300">
      {{ loadError }}
    </div>
    <div v-else-if="!urdfUrl" class="absolute inset-0 flex items-center justify-center text-sm text-fg-faint">
      No robot model uploaded.
    </div>
  </div>
</template>

<style scoped>
.viewer-toolbar { z-index: 5; }
.viewer-btn {
  display: inline-flex; align-items: center; gap: 0.1rem;
  padding: 0.25rem 0.4rem; border-radius: 0.375rem;
  background: rgba(15, 23, 42, 0.85);
  border: 1px solid rgba(51, 65, 85, 0.8);
  color: var(--color-fg);
  cursor: pointer;
  transition: background 0.1s, color 0.1s, border-color 0.1s;
}
.viewer-btn:hover {
  background: rgba(6, 182, 212, 0.18);
  border-color: #06b6d4;
  color: #67e8f9;
}
.viewer-btn-active {
  background: rgba(239, 68, 68, 0.22);
  border-color: #ef4444;
  color: #fca5a5;
}
.viewer-menu {
  position: absolute; top: calc(100% + 4px); right: 0;
  min-width: 8rem;
  background: rgba(15, 23, 42, 0.95);
  border: 1px solid rgba(51, 65, 85, 0.9);
  border-radius: 0.375rem;
  box-shadow: 0 8px 16px rgba(0, 0, 0, 0.4);
  padding: 0.25rem 0;
  z-index: 10;
}
.viewer-menu-item {
  display: block; width: 100%; text-align: left;
  padding: 0.35rem 0.75rem;
  background: transparent; border: 0;
  color: var(--color-fg); font-size: 0.8rem;
  cursor: pointer;
}
.viewer-menu-item:hover { background: rgba(6, 182, 212, 0.18); color: #67e8f9; }
</style>
