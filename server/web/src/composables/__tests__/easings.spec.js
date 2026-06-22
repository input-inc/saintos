// Easing catalog drives every animation curve the dashboard plays back
// and renders. These lock the sampler's known values + boundary clamps,
// the code→entry lookup (incl. the linear fallback that keeps unknown
// stored codes from breaking playback), and the catalog's integrity
// (unique codes — old animations on disk reference them by number).
import { describe, it, expect } from 'vitest'
import {
  EASINGS, EASING_CATEGORIES, easingByCode, sampleEasing, segmentPath,
} from '../easings'

const approx = (a, b, eps = 1e-6) => expect(Math.abs(a - b)).toBeLessThan(eps)

describe('sampleEasing', () => {
  it('is the identity for linear (code 1)', () => {
    approx(sampleEasing(1, 0.25), 0.25)
    approx(sampleEasing(1, 0.5), 0.5)
  })

  it('matches known easings.net values', () => {
    approx(sampleEasing(13, 0.5), 0.25)   // easeInQuad  x^2
    approx(sampleEasing(14, 0.5), 0.75)   // easeOutQuad 1-(1-x)^2
    approx(sampleEasing(16, 0.5), 0.125)  // easeInCubic x^3
  })

  it('clamps t outside [0,1] to the segment ends', () => {
    for (const code of [0, 1, 13, 35]) {
      approx(sampleEasing(code, 0), 0)
      approx(sampleEasing(code, 1), 1)
      approx(sampleEasing(code, -0.5), 0)
      approx(sampleEasing(code, 2), 1)
    }
  })

  it('holds 0 across a constant (step) segment interior', () => {
    expect(sampleEasing(0, 0.01)).toBe(0)
    expect(sampleEasing(0, 0.99)).toBe(0)
    expect(sampleEasing(0, 1)).toBe(1) // the t>=1 guard wins at the very end
  })

  it('stays finite and within plausible bounds for every catalog entry', () => {
    for (const e of EASINGS) {
      for (const t of [0.1, 0.25, 0.5, 0.75, 0.9]) {
        const y = sampleEasing(e.code, t)
        expect(Number.isFinite(y)).toBe(true)
        // Back/Elastic overshoot a little; keep a generous envelope.
        expect(y).toBeGreaterThan(-0.6)
        expect(y).toBeLessThan(1.6)
      }
    }
  })
})

describe('easingByCode', () => {
  it('looks up known codes', () => {
    expect(easingByCode(1).name).toBe('linear')
    expect(easingByCode(13).name).toBe('easeInQuad')
  })

  it('falls back to linear for unknown codes (stored-code safety)', () => {
    expect(easingByCode(999).name).toBe('linear')
    expect(easingByCode(undefined).name).toBe('linear')
  })
})

describe('EASINGS catalog integrity', () => {
  it('has unique codes (old animations reference them by number)', () => {
    const codes = EASINGS.map(e => e.code)
    expect(new Set(codes).size).toBe(codes.length)
  })

  it('exposes non-empty categorized groups', () => {
    expect(EASING_CATEGORIES.length).toBeGreaterThan(0)
    for (const g of EASING_CATEGORIES) expect(g.items.length).toBeGreaterThan(0)
  })
})

describe('segmentPath', () => {
  it('emits a single cubic command for CSS bezier easings', () => {
    const p = segmentPath(3 /* ease */, 0, 0, 10, 10)
    expect(p.trim().startsWith('C')).toBe(true)
  })

  it('emits the step shape for a constant easing', () => {
    const p = segmentPath(0, 0, 0, 10, 10)
    expect(p).toContain('L 10 0') // jump across, then down at the segment end
  })

  it('polyline-samples a function easing', () => {
    const p = segmentPath(13 /* easeInQuad */, 0, 0, 10, 10)
    expect((p.match(/L /g) || []).length).toBeGreaterThan(5)
  })
})
