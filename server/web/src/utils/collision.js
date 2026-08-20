// Self-collision detection for the URDF viewer.
//
// Works on the <collision> geometry urdf-loader builds when
// `parseCollision` is on. We group collision meshes by their owning link,
// accelerate each with a BVH (built once), and test link-pairs with a cheap
// world-AABB broadphase before an exact BVH↔BVH narrowphase.
//
// No hand-authored allowed-collision matrix: pairs that already intersect at
// the HOME pose (adjacency + intentional modeling overlaps) are captured as a
// "baseline" and ignored forever, so we only ever report *new* intersections
// that appear as joints move.
import * as THREE from 'three'
import { ConvexGeometry } from 'three/examples/jsm/geometries/ConvexGeometry.js'
import { MeshBVH } from 'three-mesh-bvh'

const pairKey = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`)

// Hull cache, keyed by SOURCE geometry. The viewer builds colliders twice
// over the same geometry objects — once for the display robot and once
// for its FK-sandbox clone (three's clone() shares geometry) — and
// without this the QuickHull pass and the BVH build both ran twice per
// mesh. On full-detail CAD collision meshes that alone froze the page on
// URDF upload. WeakMap so hulls die with their source geometry.
const _hullCache = new WeakMap()

// Convex hull of a geometry, in the same local frame (fast collision proxy).
// Over-approximates concavities — acceptable here because the ACM prunes the
// design-adjacency overlaps that over-approximation would otherwise flag.
function toHull (geom) {
  const pos = geom?.attributes?.position
  if (!pos || pos.count < 4) return null
  if (_hullCache.has(geom)) return _hullCache.get(geom)
  const pts = new Array(pos.count)
  for (let i = 0; i < pos.count; i++) pts[i] = new THREE.Vector3().fromBufferAttribute(pos, i)
  let hull = null
  try {
    const h = new ConvexGeometry(pts)
    hull = h.attributes?.position?.count ? h : null
  } catch (_) { hull = null }
  // Cache failures too — retrying QuickHull on a degenerate mesh every
  // rebuild is the same cost as succeeding.
  _hullCache.set(geom, hull)
  return hull
}

// Collect one entry per collision mesh: { link, mesh, geom } with a BVH and a
// local bounding box precomputed. `robot` is a URDFRobot (urdf-loader with
// parseCollision=true).
//
// opts.simplify: 'hull' replaces each collision geometry with its convex hull
// for the BVH (huge narrowphase speedup on full-detail CAD meshes). Returns
// { colliders, stats } where stats has orig/bvh triangle totals + build time.
export function buildColliders (robot, opts = {}) {
  const colliders = []
  let origTris = 0; let bvhTris = 0
  const t0 = (typeof performance !== 'undefined') ? performance.now() : 0
  robot.traverse((o) => {
    if (!o.isURDFCollider) return
    let link = o
    while (link && !link.isURDFLink) link = link.parent
    const linkName = link?.name || o.name || 'link'
    o.traverse((m) => {
      if (!m.isMesh || !m.geometry) return
      const src = m.geometry
      origTris += (src.index ? src.index.count : src.attributes.position.count) / 3
      let geom = src
      if (opts.simplify === 'hull') geom = toHull(src) || src
      if (!geom.boundsTree) geom.boundsTree = new MeshBVH(geom)
      if (!geom.boundingBox) geom.computeBoundingBox()
      bvhTris += (geom.index ? geom.index.count : geom.attributes.position.count) / 3
      colliders.push({ link: linkName, mesh: m, geom })
    })
  })
  const buildMs = ((typeof performance !== 'undefined') ? performance.now() : 0) - t0
  return { colliders, stats: { origTris: origTris | 0, bvhTris: bvhTris | 0, buildMs } }
}

// Tree-adjacency: link-pairs directly connected by one joint (parent↔child).
// These are designed to touch, so the ACM ignores them. Returns a Set of
// pair keys. Walks each URDFLink up to its nearest ancestor URDFLink.
export function computeAdjacency (robot) {
  const adj = new Set()
  robot.traverse((o) => {
    if (!o.isURDFLink) return
    let p = o.parent
    while (p && !p.isURDFLink) p = p.parent
    if (p && p.name && o.name) adj.add(pairKey(o.name, p.name))
  })
  return adj
}

const _boxA = new THREE.Box3()
const _boxB = new THREE.Box3()
const _mat = new THREE.Matrix4()

function worldBox (c, target) {
  return target.copy(c.geom.boundingBox).applyMatrix4(c.mesh.matrixWorld)
}

function meshesIntersect (a, b) {
  // Transform b's geometry into a's local frame, then BVH↔BVH test.
  _mat.copy(a.mesh.matrixWorld).invert().multiply(b.mesh.matrixWorld)
  return a.geom.boundsTree.intersectsGeometry(b.geom, _mat)
}

// Returns a Set of colliding link-pair keys ("linkA|linkB", sorted) at the
// colliders' CURRENT world transforms. Caller must have updated world
// matrices. Same-link and `ignore`d pairs are skipped; once a link-pair is
// confirmed we skip its remaining sub-mesh pairs.
export function collidingPairs (colliders, ignore = null) {
  const n = colliders.length
  const boxes = new Array(n)
  for (let i = 0; i < n; i++) boxes[i] = worldBox(colliders[i], new THREE.Box3())
  const hits = new Set()
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const A = colliders[i]; const B = colliders[j]
      if (A.link === B.link) continue
      const key = pairKey(A.link, B.link)
      if (hits.has(key)) continue
      if (ignore && ignore.has(key)) continue
      if (!boxes[i].intersectsBox(boxes[j])) continue // broadphase
      if (meshesIntersect(A, B)) hits.add(key) // narrowphase
    }
  }
  return hits
}

// Pairs intersecting at the current (home) pose — the set to ignore forever.
export function computeBaseline (colliders) {
  return collidingPairs(colliders, null)
}

// Merge a time-ordered list of { t, pairs:Set } samples into collision
// intervals [{ start, end, pairs:[...] }], one run per contiguous stretch
// where *any* non-baseline pair collides.
export function samplesToIntervals (samples) {
  const intervals = []
  let cur = null
  for (const s of samples) {
    const colliding = s.pairs && s.pairs.size > 0
    if (colliding) {
      if (!cur) cur = { start: s.t, end: s.t, pairs: new Set() }
      cur.end = s.t
      for (const p of s.pairs) cur.pairs.add(p)
    } else if (cur) {
      intervals.push({ start: cur.start, end: cur.end, pairs: [...cur.pairs] })
      cur = null
    }
  }
  if (cur) intervals.push({ start: cur.start, end: cur.end, pairs: [...cur.pairs] })
  return intervals
}
