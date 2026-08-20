import { describe, it, expect } from 'vitest'
import * as THREE from 'three'
import { MeshBVH } from 'three-mesh-bvh'
import { buildColliders, collidingPairs, computeBaseline, samplesToIntervals } from '@/utils/collision'

// A unit box collider at `pos`, tagged with a link name.
function collider (link, pos) {
  const geom = new THREE.BoxGeometry(1, 1, 1)
  geom.boundsTree = new MeshBVH(geom)
  geom.computeBoundingBox()
  const mesh = new THREE.Mesh(geom)
  mesh.position.set(...pos)
  mesh.updateMatrixWorld(true)
  return { link, mesh, geom }
}
const place = (c, pos) => { c.mesh.position.set(...pos); c.mesh.updateMatrixWorld(true) }

describe('collision engine', () => {
  it('detects overlapping link-pairs and ignores far ones', () => {
    const A = collider('A', [0, 0, 0])
    const B = collider('B', [0.5, 0, 0]) // overlaps A
    const C = collider('C', [10, 0, 0]) // far
    const hits = collidingPairs([A, B, C])
    expect([...hits]).toEqual(['A|B'])
  })

  it('excludes baseline (home-pose) pairs, reports only new collisions', () => {
    const A = collider('A', [0, 0, 0])
    const B = collider('B', [0.5, 0, 0]) // touches A at "home"
    const C = collider('C', [10, 0, 0])
    const base = computeBaseline([A, B, C])
    expect([...base]).toEqual(['A|B'])

    place(C, [-0.5, 0, 0]) // C now overlaps A (new), B stays where it was
    const hits = collidingPairs([A, B, C], base)
    expect(hits.has('A|B')).toBe(false) // baseline suppressed
    expect(hits.has('A|C')).toBe(true) // new collision reported
  })

  it('never reports a link against itself', () => {
    const A1 = collider('A', [0, 0, 0])
    const A2 = collider('A', [0.1, 0, 0]) // same link, overlapping
    expect(collidingPairs([A1, A2]).size).toBe(0)
  })

  it('reuses hulls and BVHs when rebuilding over shared geometry', () => {
    // The viewer builds colliders twice over the SAME geometry objects —
    // display robot + its clone(true) FK sandbox. Before the hull cache,
    // QuickHull and the BVH build ran twice per mesh, which froze the
    // page on URDF upload with real CAD collision meshes.
    const makeRobot = (geom) => {
      const link = new THREE.Group()
      link.isURDFLink = true
      link.name = 'linkA'
      const col = new THREE.Group()
      col.isURDFCollider = true
      col.add(new THREE.Mesh(geom))
      link.add(col)
      const root = new THREE.Group()
      root.add(link)
      root.updateMatrixWorld(true)
      return root
    }
    const shared = new THREE.SphereGeometry(1, 16, 16)
    const a = buildColliders(makeRobot(shared), { simplify: 'hull' })
    const b = buildColliders(makeRobot(shared), { simplify: 'hull' })
    expect(a.colliders.length).toBe(1)
    // Same hull object AND same boundsTree instance — not equal, identical.
    expect(b.colliders[0].geom).toBe(a.colliders[0].geom)
    expect(b.colliders[0].geom.boundsTree).toBe(a.colliders[0].geom.boundsTree)
  })

  it('merges samples into contiguous intervals', () => {
    const mk = (t, on) => ({ t, pairs: new Set(on ? ['A|C'] : []) })
    const samples = [mk(0, false), mk(0.1, false), mk(0.2, true),
      mk(0.3, true), mk(0.4, true), mk(0.5, false), mk(0.6, true)]
    const iv = samplesToIntervals(samples)
    expect(iv.length).toBe(2)
    expect(iv[0]).toMatchObject({ start: 0.2, end: 0.4 })
    expect(iv[1]).toMatchObject({ start: 0.6, end: 0.6 })
    expect(iv[0].pairs).toContain('A|C')
  })
})
