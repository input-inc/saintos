// Curve sampling is the playback math: given an animation's keyframes
// and a time, produce the interpolated value the robot is driven to.
// These lock the segment boundary handling, linear / constant / easing
// interpolation, the cubic-bezier solver endpoints, and the multi-track
// fan-out.
import { describe, it, expect } from 'vitest'
import {
  cubicBezierAtTime, sampleCurve, sampleAllTracks,
  INTERP_LINEAR, INTERP_CONSTANT,
} from '../useCurveSampling'

const approx = (a, b, eps = 1e-4) => expect(Math.abs(a - b)).toBeLessThan(eps)

describe('cubicBezierAtTime', () => {
  it('pins the endpoints', () => {
    expect(cubicBezierAtTime(0, 0.42, 0, 0.58, 1)).toBe(0)
    expect(cubicBezierAtTime(1, 0.42, 0, 0.58, 1)).toBe(1)
  })

  it('solves the linear control points to ~identity', () => {
    approx(cubicBezierAtTime(0.5, 0, 0, 1, 1), 0.5, 1e-3)
    approx(cubicBezierAtTime(0.25, 0, 0, 1, 1), 0.25, 1e-3)
  })

  it('stays within [0,1] and monotonic for ease-in-out', () => {
    let prev = -1
    for (let t = 0; t <= 1.0001; t += 0.1) {
      const y = cubicBezierAtTime(t, 0.42, 0, 0.58, 1)
      expect(y).toBeGreaterThanOrEqual(-1e-6)
      expect(y).toBeLessThanOrEqual(1 + 1e-6)
      expect(y).toBeGreaterThanOrEqual(prev - 1e-6)
      prev = y
    }
  })
})

describe('sampleCurve', () => {
  const linear = { keys: [
    { time: 0, value: 0, interp: INTERP_LINEAR },
    { time: 10, value: 100, interp: INTERP_LINEAR },
  ] }

  it('returns 0 for an empty curve', () => {
    expect(sampleCurve({ keys: [] }, 5)).toBe(0)
    expect(sampleCurve(undefined, 5)).toBe(0)
  })

  it('clamps to the first/last key outside the time range', () => {
    expect(sampleCurve(linear, -5)).toBe(0)
    expect(sampleCurve(linear, 999)).toBe(100)
  })

  it('linearly interpolates between keys', () => {
    approx(sampleCurve(linear, 5), 50)
    approx(sampleCurve(linear, 2.5), 25)
  })

  it('holds the previous value across a constant segment', () => {
    const step = { keys: [
      { time: 0, value: 10, interp: INTERP_CONSTANT },
      { time: 10, value: 20, interp: INTERP_LINEAR },
    ] }
    expect(sampleCurve(step, 5)).toBe(10)
    expect(sampleCurve(step, 9.999)).toBe(10)
    expect(sampleCurve(step, 10)).toBe(20) // last-key clamp
  })

  it('applies an easing function across a segment', () => {
    // easeInQuad (code 13): y(0.5)=0.25 → 0 + 0.25*100 = 25.
    const eased = { keys: [
      { time: 0, value: 0, interp: 13 },
      { time: 10, value: 100, interp: INTERP_LINEAR },
    ] }
    approx(sampleCurve(eased, 5), 25)
  })

  it('handles zero-width segments without NaN', () => {
    const dup = { keys: [
      { time: 5, value: 1, interp: INTERP_LINEAR },
      { time: 5, value: 9, interp: INTERP_LINEAR },
    ] }
    const v = sampleCurve(dup, 5)
    expect(Number.isFinite(v)).toBe(true)
  })
})

describe('sampleAllTracks', () => {
  it('samples every value track into an id→value map', () => {
    const anim = {
      value_tracks: [
        { id: 'pan', curve: { keys: [
          { time: 0, value: 0, interp: INTERP_LINEAR },
          { time: 10, value: 1, interp: INTERP_LINEAR },
        ] } },
        { id: 'tilt', curve: { keys: [
          { time: 0, value: -1, interp: INTERP_LINEAR },
          { time: 10, value: 1, interp: INTERP_LINEAR },
        ] } },
      ],
    }
    const out = sampleAllTracks(anim, 5)
    approx(out.pan, 0.5)
    approx(out.tilt, 0)
  })

  it('returns an empty map for an animation with no tracks', () => {
    expect(sampleAllTracks({}, 5)).toEqual({})
    expect(sampleAllTracks(undefined, 5)).toEqual({})
  })
})
