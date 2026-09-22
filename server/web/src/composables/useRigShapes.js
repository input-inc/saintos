// Control-rig shapes for the 3D viewport.
//
// Builds one three.js object per rig control, parented to the URDF link
// the control is anchored to, so a shape follows the pose for free with
// no per-frame bookkeeping. This is the Unreal Control Rig arrangement:
// a shape hung off a bone by an offset transform, grabbed and dragged in
// the viewport rather than driven from a panel.
//
// The design thread that produced the rig schema argued against giving a
// scalar a 3D gizmo — "a scalar with no spatial meaning rendered as a
// draggable widget in 3D is worse than a slider". The resolution is the
// `axis` attribute: a channel's shape sits on the joint it moves and
// drags along a declared direction, which is what gives the scalar
// spatial meaning. A ring on the head that you sweep to nod is not the
// same thing as a slider floating in space.
//
// Geometry only. No evaluation, no value math — the caller maps a drag
// delta to a control value and pushes it through the normal evaluate
// path, so the viewport and the side panel drive exactly the same code.

import * as THREE from 'three'

// Shapes are authored in metres at scale 1 and then scaled by the
// widget's `scale`. Keeping the unit geometry small means a default
// scale of 0.05 lands at a sensible size on a human-scale robot.
const UNIT = 1

function ringGeometry () {
  // Flat annulus. Reads as "rotate about the ring's normal", which is
  // the default for a 1-DOF channel.
  return new THREE.RingGeometry(UNIT * 0.75, UNIT, 48)
}

function wedgeGeometry () {
  // A ring segment — a bounded sweep, for a control whose range is
  // visibly less than a full revolution.
  return new THREE.RingGeometry(UNIT * 0.75, UNIT, 32, 1, 0, Math.PI * 0.75)
}

function arrowGeometry () {
  // Shaft + head, merged into one buffer so a single mesh is one hit
  // target. Points along +Y, which is what `axis` is rotated onto.
  const shaft = new THREE.CylinderGeometry(UNIT * 0.06, UNIT * 0.06, UNIT * 1.4, 12)
  const head = new THREE.ConeGeometry(UNIT * 0.18, UNIT * 0.45, 14)
  head.translate(0, UNIT * 0.92, 0)
  const merged = mergeGeometries([shaft, head])
  shaft.dispose()
  head.dispose()
  return merged
}

function diamondGeometry () {
  return new THREE.OctahedronGeometry(UNIT, 0)
}

/**
 * Minimal geometry merge — enough for the two-part arrow, without
 * pulling in BufferGeometryUtils (which is an extra example-module
 * import for one call site).
 */
function mergeGeometries (geoms) {
  const positions = []
  const normals = []
  for (const g of geoms) {
    const nonIndexed = g.index ? g.toNonIndexed() : g
    positions.push(...nonIndexed.attributes.position.array)
    if (nonIndexed.attributes.normal) {
      normals.push(...nonIndexed.attributes.normal.array)
    }
    if (nonIndexed !== g) nonIndexed.dispose()
  }
  const out = new THREE.BufferGeometry()
  out.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
  if (normals.length === positions.length) {
    out.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3))
  } else {
    out.computeVertexNormals()
  }
  return out
}

const BUILDERS = {
  sphere: () => new THREE.SphereGeometry(UNIT, 20, 14),
  box: () => new THREE.BoxGeometry(UNIT * 2, UNIT * 2, UNIT * 2),
  circle: () => new THREE.CircleGeometry(UNIT, 40),
  ring: ringGeometry,
  cylinder: () => new THREE.CylinderGeometry(UNIT, UNIT, UNIT * 2, 24),
  cone: () => new THREE.ConeGeometry(UNIT, UNIT * 2, 20),
  arrow: arrowGeometry,
  diamond: diamondGeometry,
  torus: () => new THREE.TorusGeometry(UNIT, UNIT * 0.22, 12, 36),
  wedge: wedgeGeometry,
  plane: () => new THREE.PlaneGeometry(UNIT * 2, UNIT * 2),
}

// Shapes that read as flat sheets. They get double-sided materials, or
// they vanish when the camera swings behind them — which looks exactly
// like a bug.
const FLAT_SHAPES = new Set(['ring', 'circle', 'plane', 'wedge'])

// Geometry is cached per shape name: several controls commonly share a
// shape, and the meshes are tiny but the allocations aren't free.
const geometryCache = new Map()

function geometryFor (shape) {
  const name = BUILDERS[shape] ? shape : 'sphere'
  if (!geometryCache.has(name)) geometryCache.set(name, BUILDERS[name]())
  return geometryCache.get(name)
}

/** Release the shared geometry cache. Call on teardown. */
export function disposeRigShapeCache () {
  for (const g of geometryCache.values()) g.dispose()
  geometryCache.clear()
}

/**
 * Build the shape for one control.
 *
 * @param {object} control  parsed rig control (kind, min/max, widget, …)
 * @returns {THREE.Group}   group whose local transform is the widget's
 *                          offset+rotation within the anchor link
 */
export function buildControlShape (control, { inert = false } = {}) {
  const w = control.widget || {}
  const shape = w.shape || 'sphere'
  const color = new THREE.Color(
    (w.color?.[0] ?? 1), (w.color?.[1] ?? 0.7), (w.color?.[2] ?? 0))
  // An INERT control is one the evaluator reported it can't handle (today:
  // a gaze binding, which needs the IK solver). Drawn as a dim wireframe
  // and excluded from hit-testing by the caller, because a handle that
  // looks grabbable and does nothing is worse than no handle at all —
  // you drag it, the robot ignores you, and nothing says why.
  const opacity = inert ? 0.3 : (w.color?.[3] ?? 0.8)

  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    wireframe: inert,
    side: FLAT_SHAPES.has(shape) ? THREE.DoubleSide : THREE.FrontSide,
    // Always visible through the model. A control handle buried inside
    // the head is unusable, and depth-sorting it correctly against the
    // robot is not worth the trouble — Unreal draws its shapes on top
    // for the same reason.
    depthTest: false,
    depthWrite: false,
  })

  const mesh = new THREE.Mesh(geometryFor(shape), material)
  // Render after the robot so the always-on-top material actually lands
  // on top rather than depending on traversal order.
  mesh.renderOrder = 10

  const group = new THREE.Group()
  group.add(mesh)

  const [sx, sy, sz] = normalizeTriple(w.scale, 0.05)
  group.scale.set(sx || 0.05, sy || 0.05, sz || 0.05)

  const [ox, oy, oz] = normalizeTriple(w.offset, 0)
  group.position.set(ox, oy, oz)

  const [rr, rp, ry] = normalizeTriple(w.rotation, 0)
  group.rotation.set(rr, rp, ry)

  group.userData.rigControl = control.name
  group.userData.baseColor = color.clone()
  group.userData.baseOpacity = opacity
  group.userData.mesh = mesh
  group.userData.inert = inert
  return group
}

function normalizeTriple (v, fallback) {
  if (Array.isArray(v)) {
    if (v.length >= 3) return [Number(v[0]), Number(v[1]), Number(v[2])]
    if (v.length === 1) return [Number(v[0]), Number(v[0]), Number(v[0])]
  }
  if (typeof v === 'number') return [v, v, v]
  return [fallback, fallback, fallback]
}

/**
 * World-space drag axis for a control, in the widget's own frame after
 * its rotation — which is what makes `axis="0 1 0"` mean "the widget's
 * local Y", not "world Y".
 */
export function worldDragAxis (group, control) {
  const [ax, ay, az] = normalizeTriple(control.widget?.axis, 0)
  const local = new THREE.Vector3(ax || 0, ay || 0, az || 0)
  if (local.lengthSq() === 0) local.set(1, 0, 0)
  return local.normalize().transformDirection(group.matrixWorld).normalize()
}

/** Tint a shape to show hover / active state. */
export function setShapeHighlight (group, state) {
  const mesh = group?.userData?.mesh
  if (!mesh || group.userData.inert) return
  const base = group.userData.baseColor
  const baseOpacity = group.userData.baseOpacity ?? 0.8
  if (state === 'active') {
    mesh.material.color.set(0xffffff)
    mesh.material.opacity = Math.min(1, baseOpacity + 0.35)
  } else if (state === 'hover') {
    mesh.material.color.copy(base).lerp(new THREE.Color(0xffffff), 0.45)
    mesh.material.opacity = Math.min(1, baseOpacity + 0.2)
  } else {
    mesh.material.color.copy(base)
    mesh.material.opacity = baseOpacity
  }
}
