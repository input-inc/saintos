// Control-rig shape construction.
//
// The bug that prompted these: a gaze control rendered as a big solid
// sphere that looked perfectly grabbable and did nothing, because the
// evaluator skips gaze bindings. A handle that offers a drag it can't
// honour is worse than no handle — you pull it, the robot ignores you,
// and nothing says why. So "inert" has to be visible AND un-grabbable.

import { describe, expect, it } from 'vitest'
import {
  buildControlShape,
  disposeRigShapeCache,
  setShapeHighlight,
  worldDragAxis,
} from '../useRigShapes'

const control = (over = {}) => ({
  name: 'c', kind: 'channel', min: -1, max: 1, default: 0,
  widget: {
    shape: 'ring', scale: [0.05, 0.05, 0.05], color: [1, 0.7, 0, 0.8],
    offset: [0, 0, 0], rotation: [0, 0, 0], axis: [1, 0, 0],
    dofs: 'move_3d', invert_y: false, visible: true,
  },
  ...over,
})

describe('buildControlShape', () => {
  it('applies scale, offset and rotation from the widget', () => {
    const g = buildControlShape(control({
      widget: { ...control().widget, scale: [0.1, 0.2, 0.3], offset: [1, 2, 3], rotation: [0.5, 0, 0] },
    }))
    expect([g.scale.x, g.scale.y, g.scale.z]).toEqual([0.1, 0.2, 0.3])
    expect([g.position.x, g.position.y, g.position.z]).toEqual([1, 2, 3])
    expect(g.rotation.x).toBeCloseTo(0.5)
  })

  it('accepts a uniform scale expressed as one number', () => {
    const g = buildControlShape(control({
      widget: { ...control().widget, scale: 0.2 },
    }))
    expect([g.scale.x, g.scale.y, g.scale.z]).toEqual([0.2, 0.2, 0.2])
  })

  it('tags the control name so a hit can be traced back', () => {
    expect(buildControlShape(control({ name: 'mood' })).userData.rigControl)
      .toBe('mood')
  })

  it('draws flat shapes double-sided', () => {
    // A ring rendered single-sided vanishes when the camera swings behind
    // it, which looks exactly like a bug.
    const ring = buildControlShape(control())
    const sphere = buildControlShape(control({
      widget: { ...control().widget, shape: 'sphere' },
    }))
    expect(ring.userData.mesh.material.side).not
      .toBe(sphere.userData.mesh.material.side)
  })

  it('draws on top of the model', () => {
    // A handle buried inside the head is unusable.
    const m = buildControlShape(control()).userData.mesh
    expect(m.material.depthTest).toBe(false)
    expect(m.renderOrder).toBeGreaterThan(0)
  })

  it('falls back to a sphere for an unknown shape rather than throwing', () => {
    // The parser rejects unknown shapes, so reaching here means a newer
    // file than this build — degrade, don't crash the viewport.
    expect(() => buildControlShape(control({
      widget: { ...control().widget, shape: 'dodecahedron' },
    }))).not.toThrow()
  })
})

describe('inert controls', () => {
  it('are marked, dimmed and wireframed', () => {
    const g = buildControlShape(control(), { inert: true })
    expect(g.userData.inert).toBe(true)
    expect(g.userData.mesh.material.wireframe).toBe(true)
    expect(g.userData.mesh.material.opacity).toBeLessThan(0.5)
  })

  it('are not marked when interactive', () => {
    const g = buildControlShape(control())
    expect(g.userData.inert).toBe(false)
    expect(g.userData.mesh.material.wireframe).toBe(false)
  })

  it('ignore highlight — a hover glow on something unusable is the same lie', () => {
    const g = buildControlShape(control(), { inert: true })
    const before = g.userData.mesh.material.opacity
    setShapeHighlight(g, 'hover')
    expect(g.userData.mesh.material.opacity).toBe(before)
  })

  it('still highlight when interactive', () => {
    const g = buildControlShape(control())
    const before = g.userData.mesh.material.opacity
    setShapeHighlight(g, 'active')
    expect(g.userData.mesh.material.opacity).toBeGreaterThan(before)
    setShapeHighlight(g, null)
    expect(g.userData.mesh.material.opacity).toBeCloseTo(before)
  })
})

describe('worldDragAxis', () => {
  it('is the widget axis in the widget frame, not world space', () => {
    // axis="0 1 0" with a 90° roll must come out along world Z, or
    // `rotation` would silently not affect dragging.
    const g = buildControlShape(control({
      widget: { ...control().widget, axis: [0, 1, 0], rotation: [Math.PI / 2, 0, 0] },
    }))
    g.updateMatrixWorld(true)
    const a = worldDragAxis(g, control({
      widget: { ...control().widget, axis: [0, 1, 0], rotation: [Math.PI / 2, 0, 0] },
    }))
    expect(a.y).toBeCloseTo(0)
    expect(Math.abs(a.z)).toBeCloseTo(1)
  })

  it('falls back to X for a zero axis instead of producing NaN', () => {
    const c = control({ widget: { ...control().widget, axis: [0, 0, 0] } })
    const g = buildControlShape(c)
    g.updateMatrixWorld(true)
    const a = worldDragAxis(g, c)
    expect(Number.isFinite(a.x) && Number.isFinite(a.y) && Number.isFinite(a.z)).toBe(true)
    expect(a.length()).toBeCloseTo(1)
  })
})

describe('teardown', () => {
  it('releases the shared geometry cache', () => {
    buildControlShape(control())
    expect(() => disposeRigShapeCache()).not.toThrow()
  })
})
