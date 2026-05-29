// Client-side curve sampling — full easings.net palette + CSS basics.
// The canonical catalog lives in ./easings.js; this module just
// dispatches to it via the integer `interp` code stored on each key.

export const INTERP_CONSTANT     = 0
export const INTERP_LINEAR       = 1
export const INTERP_CUBIC        = 2
export const INTERP_EASE         = 3
export const INTERP_EASE_IN      = 4
export const INTERP_EASE_OUT     = 5
export const INTERP_EASE_IN_OUT  = 6

export const INTERP_LABELS = {
  [INTERP_CONSTANT]:    'Constant',
  [INTERP_LINEAR]:      'Linear',
  [INTERP_CUBIC]:       'Cubic (custom)',
  [INTERP_EASE]:        'Ease',
  [INTERP_EASE_IN]:     'Ease In',
  [INTERP_EASE_OUT]:    'Ease Out',
  [INTERP_EASE_IN_OUT]: 'Ease In Out',
}

// Bezier control points per easing — matches the CSS spec values
// (https://www.w3.org/TR/css-easing-1/#valdef-easing-function-ease).
// Each entry is [c1x, c1y, c2x, c2y] in the normalized [0,1] segment
// space, ready to drop into SVG `C` commands or solve numerically.
export const EASING_BEZIERS = {
  [INTERP_LINEAR]:      [0,    0,    1,    1],
  [INTERP_EASE]:        [0.25, 0.1,  0.25, 1],
  [INTERP_EASE_IN]:     [0.42, 0,    1,    1],
  [INTERP_EASE_OUT]:    [0,    0,    0.58, 1],
  [INTERP_EASE_IN_OUT]: [0.42, 0,    0.58, 1],
}

// CSS bezier sampler. Given a normalized progress in [0,1] and the
// four control points, solve for the parametric `s` where x(s) = t
// (Newton-Raphson with 8 iterations is plenty for animation work),
// then return y(s). Matches the standard CSS easing math used by
// browser engines.
export function cubicBezierAtTime (t, c1x, c1y, c2x, c2y) {
  if (t <= 0) return 0
  if (t >= 1) return 1
  let s = t
  for (let i = 0; i < 8; i++) {
    const one_s = 1 - s
    const x = 3 * one_s * one_s * s * c1x
            + 3 * one_s * s * s * c2x
            + s * s * s
    const dx = 3 * one_s * one_s * c1x
             + 6 * one_s * s * (c2x - c1x)
             + 3 * s * s * (1 - c2x)
    if (Math.abs(x - t) < 1e-5 || Math.abs(dx) < 1e-6) break
    s -= (x - t) / dx
    if (s < 0) s = 0
    if (s > 1) s = 1
  }
  const one_s = 1 - s
  return 3 * one_s * one_s * s * c1y
       + 3 * one_s * s * s * c2y
       + s * s * s
}

import { easingByCode, sampleEasing } from './easings'

export function sampleCurve (curve, time) {
  const keys = curve?.keys || []
  if (!keys.length) return 0
  if (time <= keys[0].time) return keys[0].value
  if (time >= keys[keys.length - 1].time) return keys[keys.length - 1].value
  for (let i = 0; i < keys.length - 1; i++) {
    const k0 = keys[i]
    const k1 = keys[i + 1]
    if (k0.time <= time && time <= k1.time) {
      const dt = k1.time - k0.time
      const t = dt === 0 ? 0 : (time - k0.time) / dt
      const code = k0.interp ?? INTERP_LINEAR
      const e = easingByCode(code)

      // Step easing: hold the previous value for the entire segment.
      if (e?.step) return k0.value

      // Hermite tangents (legacy custom cubic).
      if (e?.hermite) {
        const t2 = t * t
        const t3 = t2 * t
        const h1 = 2 * t3 - 3 * t2 + 1
        const h2 = -2 * t3 + 3 * t2
        const h3 = t3 - 2 * t2 + t
        const h4 = t3 - t2
        return (h1 * k0.value + h2 * k1.value +
                h3 * (k0.leave_tangent || 0) * dt +
                h4 * (k1.arrive_tangent || 0) * dt)
      }

      // CSS easings — cubic-bezier solved numerically. (Cheaper than
      // the easings.net polyline path because the bezier closed-form
      // converges in <8 Newton iterations.)
      if (e?.bezier) {
        const y = cubicBezierAtTime(t, e.bezier[0], e.bezier[1],
                                      e.bezier[2], e.bezier[3])
        return k0.value + y * (k1.value - k0.value)
      }

      // easings.net functions + linear.
      const y = sampleEasing(code, t)
      return k0.value + y * (k1.value - k0.value)
    }
  }
  return keys[keys.length - 1].value
}

// Sample every value track at a given time. Returns a {trackId: value}
// map suitable for fanning out to URDFViewer.setJointValue calls.
export function sampleAllTracks (animation, time) {
  const out = {}
  for (const t of animation?.value_tracks || []) {
    out[t.id] = sampleCurve(t.curve, time)
  }
  return out
}
