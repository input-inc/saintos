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

const props = defineProps({
  // Source URL for the URDF text. Null/empty disables loading.
  urdfUrl: { type: String, default: null },
  // Base URL prefix used to resolve mesh references. The server flattens
  // meshes into a single dir, so we only ever need the basename — the
  // loadMeshCb below strips any package://… or relative-path prefix.
  meshesBase: { type: String, default: '/api/robot/meshes/' },
  // Optional fixed height. Falls back to 100% of the parent.
  height: { type: String, default: '100%' },
})

const emit = defineEmits([
  'loaded',              // (robot) — the URDF root object3d
  'load-error',          // (Error)
  'joints',              // (jointNames[]) — emitted once after load
  'joint-click',         // (jointName) — operator clicked a link/mesh in the scene
  'joint-rotate',        // (jointName, angle) — live during gizmo drag
  'joint-rotate-commit', // (jointName, angle) — gizmo drag ended; safe to write a keyframe
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
  emit('joint-rotate', limitVisual.joint.name, angle)
}

function onPointerUp (e) {
  if (handleDragging) {
    handleDragging = false
    if (controls) controls.enabled = true
    document.body.style.userSelect = ''
    document.body.style.webkitUserSelect = ''
    if (limitVisual?.joint) {
      emit('joint-rotate-commit', limitVisual.joint.name, limitVisual.joint.angle ?? 0)
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

// URDF references meshes with either `package://pkg/path/foo.stl` or
// plain relative paths. The server flattens everything to a single
// `meshes/` dir, so we strip back to the basename and concat with the
// base URL. If a URDF starts referencing the same filename across
// multiple packages we'll need a smarter mapping; until then the flat
// store is dramatically simpler than mirroring a `package://` layout.
function resolveMeshUrl (path) {
  const base = path.split('/').pop()
  return props.meshesBase + encodeURIComponent(base)
}

function makeMeshLoader () {
  const manager = new THREE.LoadingManager()
  return (path, _manager, onComplete) => {
    const url = resolveMeshUrl(path)
    const ext = url.split('.').pop().split('?')[0].toLowerCase()
    let loader
    if (ext === 'stl') loader = new STLLoader(manager)
    else if (ext === 'dae') loader = new ColladaLoader(manager)
    else if (ext === 'obj') loader = new OBJLoader(manager)
    else if (ext === 'gltf' || ext === 'glb') loader = new GLTFLoader(manager)
    else {
      onComplete(new THREE.Object3D())
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
        onComplete(object)
      },
      undefined,
      (err) => {
        // Don't blow up the whole model on a missing mesh — render the
        // joint without it and surface the error so the operator can
        // figure out which file is missing from their bundle.
        // eslint-disable-next-line no-console
        console.warn('Mesh load failed', url, err)
        onComplete(new THREE.Object3D())
      },
    )
  }
}

function clearRobot () {
  if (robot) {
    scene?.remove(robot)
    disposeObject3D(robot)
    robot = null
  }
}

async function loadUrdf () {
  if (!props.urdfUrl) {
    clearRobot()
    return
  }
  loading.value = true
  loadError.value = ''
  try {
    const loader = new URDFLoader()
    loader.loadMeshCb = makeMeshLoader()
    const result = await new Promise((resolve, reject) => {
      loader.load(props.urdfUrl, resolve, undefined, reject)
    })
    clearRobot()
    robot = result
    // Most URDFs are authored Z-up; three.js is Y-up. Rotate the root
    // so the robot stands on the grid rather than lying flat.
    robot.rotation.x = -Math.PI / 2
    scene.add(robot)
    const jointNames = Object.keys(robot.joints || {})
    emit('joints', jointNames)
    emit('loaded', robot)
  } catch (e) {
    loadError.value = e?.message || String(e)
    emit('load-error', e)
  } finally {
    loading.value = false
  }
}

// Public-ish API: parent can call setJointValue from a ref.
function setJointValue (jointName, value) {
  if (!robot?.joints || !robot.joints[jointName]) return false
  robot.joints[jointName].setJointValue(value)
  return true
}

// ── Toolbar controls (Center / Grid / Views) ────────────────────────

function toggleGrid () {
  showGrid.value = !showGrid.value
  if (grid) grid.visible = showGrid.value
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

defineExpose({ setJointValue, selectJoint })

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
  resizeObserver?.disconnect()
  window.removeEventListener('resize', onResize)
  if (renderer?.domElement) {
    renderer.domElement.removeEventListener('pointerdown', onPointerDown)
    renderer.domElement.removeEventListener('pointermove', onPointerMove)
    renderer.domElement.removeEventListener('pointerup', onPointerUp)
    renderer.domElement.removeEventListener('pointerleave', onPointerUp)
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
