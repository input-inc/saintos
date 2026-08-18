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
import { MeshBVH } from 'three-mesh-bvh'

const pairKey = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`)

// Collect one entry per collision mesh: { link, mesh, geom } with a BVH and a
// local bounding box precomputed. `robot` is a URDFRobot (urdf-loader with
// parseCollision=true).
//
// We keep the FULL collision geometry (not a convex hull): hulls over-
// approximate and would flag false collisions right at the tight clearances we
// care about (a brow just skimming an eye), and they aren't reliably faster.
// The real cost is narrowphase on OVERLAPPING pairs — kept low by the AABB
// broadphase + the home-pose baseline, which suppresses the many adjacent
// parts that intersect at rest so the scan only narrowphases genuine new hits.
export function buildColliders (robot) {
  const colliders = []
  robot.traverse((o) => {
    if (!o.isURDFCollider) return
    let link = o
    while (link && !link.isURDFLink) link = link.parent
    const linkName = link?.name || o.name || 'link'
    o.traverse((m) => {
      if (!m.isMesh || !m.geometry) return
      const geom = m.geometry
      if (!geom.boundsTree) geom.boundsTree = new MeshBVH(geom)
      if (!geom.boundingBox) geom.computeBoundingBox()
      colliders.push({ link: linkName, mesh: m, geom })
    })
  })
  return colliders
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
