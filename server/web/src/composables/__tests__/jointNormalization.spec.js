// The −1..+1 control range ↔ URDF joint value mapping used by the 3D
// viewport.
//
// This is viewport-only: the robot is driven by the normalized value
// straight through the routing sheet and never touches this code. So a
// mistake here shows the operator motion the hardware will not make —
// which is exactly how it surfaced. An animation with positive
// keyframes played backwards in the editor and correctly on the robot.
//
// The cause: the mapping is home-centered (control 0 → θ=0), which is
// only meaningful when the joint's travel actually contains home. For a
// joint limited to `[-2.0, -0.5]` the old formula (θ = c·upper for
// c ≥ 0) sent every positive control value to −0.5·c, then clamped it —
// so the whole positive half of the range collapsed onto the nearest
// limit and the joint sat at a negative angle regardless.

import { describe, expect, it } from 'vitest'
import {
  denormJoint, jointLimits, normJoint,
} from '../useJointNormalization'

const J = (lower, upper) => ({ limit: { lower, upper } })

describe('jointLimits', () => {
  it('reads finite limits', () => {
    expect(jointLimits(J(-1, 2))).toEqual({ lo: -1, hi: 2 })
  })

  it('returns null without usable limits', () => {
    expect(jointLimits({})).toBeNull()
    expect(jointLimits(J(NaN, 1))).toBeNull()
    expect(jointLimits(undefined)).toBeNull()
  })

  it('swaps reversed limits instead of trusting them', () => {
    // Left as authored these made the clamp collapse to a constant and
    // the joint froze. An author who writes them backwards means the
    // range, not a locked joint.
    expect(jointLimits(J(1.57, -1.57))).toEqual({ lo: -1.57, hi: 1.57 })
  })
})

describe('denormJoint — travel that straddles home', () => {
  // The common case, and deliberately unchanged.
  it('pins control 0 to the URDF home', () => {
    expect(denormJoint(J(-1.57, 1.57), 0)).toBeCloseTo(0, 9)
  })

  it('maps the ends to the limits', () => {
    expect(denormJoint(J(-1.57, 1.57), -1)).toBeCloseTo(-1.57, 9)
    expect(denormJoint(J(-1.57, 1.57), 1)).toBeCloseTo(1.57, 9)
  })

  it('scales each side independently so home stays at 0', () => {
    const j = J(-0.5, 2.0)
    expect(denormJoint(j, 0)).toBeCloseTo(0, 9)
    expect(denormJoint(j, 0.5)).toBeCloseTo(1.0, 9)
    expect(denormJoint(j, -0.5)).toBeCloseTo(-0.25, 9)
  })

  it('holds a one-sided joint at home on its dead side', () => {
    // Documented, intentional, and left alone.
    expect(denormJoint(J(0, 1.57), -1)).toBeCloseTo(0, 9)
    expect(denormJoint(J(-1.57, 0), 1)).toBeCloseTo(0, 9)
  })
})

describe('denormJoint — travel that excludes home', () => {
  const NEG = J(-2.0, -0.5)     // the reported case
  const POS = J(0.5, 2.0)

  it('moves for positive control instead of collapsing', () => {
    // The bug: every one of these used to land on -0.5.
    const a = denormJoint(NEG, 0.0)
    const b = denormJoint(NEG, 0.5)
    const c = denormJoint(NEG, 1.0)
    expect(new Set([a, b, c]).size).toBe(3)
  })

  it('spans the full travel across the full control range', () => {
    expect(denormJoint(NEG, -1)).toBeCloseTo(-2.0, 9)
    expect(denormJoint(NEG, 0)).toBeCloseTo(-1.25, 9)
    expect(denormJoint(NEG, 1)).toBeCloseTo(-0.5, 9)
  })

  it('increases monotonically with the control value', () => {
    // "It only moved in the negative direction" — the mapping has to be
    // order-preserving or the viewport shows motion reversed.
    const seq = [-1, -0.5, 0, 0.5, 1].map(c => denormJoint(NEG, c))
    for (let i = 1; i < seq.length; i++) expect(seq[i]).toBeGreaterThan(seq[i - 1])
  })

  it('works the same for wholly-positive travel', () => {
    expect(denormJoint(POS, -1)).toBeCloseTo(0.5, 9)
    expect(denormJoint(POS, 0)).toBeCloseTo(1.25, 9)
    expect(denormJoint(POS, 1)).toBeCloseTo(2.0, 9)
  })

  it('never leaves the limits', () => {
    for (const c of [-5, -1, 0, 1, 5]) {
      const v = denormJoint(NEG, c)
      expect(v).toBeGreaterThanOrEqual(-2.0)
      expect(v).toBeLessThanOrEqual(-0.5)
    }
  })
})

describe('denormJoint — degenerate input', () => {
  it('passes the value through without limits', () => {
    expect(denormJoint({}, 0.42)).toBe(0.42)
  })

  it('clamps the control range', () => {
    expect(denormJoint(J(-1, 1), 9)).toBeCloseTo(1, 9)
    expect(denormJoint(J(-1, 1), -9)).toBeCloseTo(-1, 9)
  })

  it('treats a non-numeric control as 0', () => {
    expect(denormJoint(J(-1, 1), 'x')).toBeCloseTo(0, 9)
  })

  it('survives a zero-width range', () => {
    expect(denormJoint(J(0.5, 0.5), 1)).toBeCloseTo(0.5, 9)
    expect(normJoint(J(0.5, 0.5), 0.5)).toBe(0)
  })
})

describe('normJoint round-trips denormJoint', () => {
  const joints = [
    ['straddles home', J(-1.57, 1.57)],
    ['asymmetric', J(-0.5, 2.0)],
    ['wholly negative', J(-2.0, -0.5)],
    ['wholly positive', J(0.5, 2.0)],
  ]
  for (const [label, j] of joints) {
    it(label, () => {
      for (const c of [-1, -0.75, -0.25, 0, 0.25, 0.75, 1]) {
        expect(normJoint(j, denormJoint(j, c))).toBeCloseTo(c, 6)
      }
    })
  }

  it('round-trips the reachable side of a one-sided joint', () => {
    const j = J(0, 1.57)
    for (const c of [0, 0.5, 1]) {
      expect(normJoint(j, denormJoint(j, c))).toBeCloseTo(c, 6)
    }
  })
})
