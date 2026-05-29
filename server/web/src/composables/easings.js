// Animation easings catalog — covers the full easings.net palette
// (https://easings.net/) plus the CSS basics and our legacy types.
//
// Each entry is the canonical source of truth: the integer `code` is
// what gets stored on keyframes (`interp`); UI menus render from the
// `label` / `category`; the sampler calls `fn(t)`; the SVG renderer
// prefers `bezier` when present (faster, single SVG C command) and
// falls back to polyline sampling of `fn` for everything else.
//
// Formulas ported verbatim from https://easings.net/ which credits
// Robert Penner's original easing equations. The CSS values match
// the W3C css-easing-1 spec.

const c1 = 1.70158
const c2 = c1 * 1.525
const c3 = c1 + 1
const c4 = (2 * Math.PI) / 3
const c5 = (2 * Math.PI) / 4.5
const n1 = 7.5625
const d1 = 2.75

function bounceOut (x) {
  if (x < 1 / d1)        return n1 * x * x
  if (x < 2 / d1)        return n1 * (x -= 1.5 / d1)   * x + 0.75
  if (x < 2.5 / d1)      return n1 * (x -= 2.25 / d1)  * x + 0.9375
  return                       n1 * (x -= 2.625 / d1) * x + 0.984375
}

// Codes 0-6 are the legacy / CSS set already shipped on stored
// keyframes; 10-39 are the easings.net additions in the order shown
// on that site. Don't reuse codes — old animations on disk reference
// them.
export const EASINGS = [
  // ── legacy + CSS basics ─────────────────────────────────────────
  { code: 0,  name: 'constant',       label: 'Constant',     category: 'Step',   fn: x => 0, step: true },
  { code: 1,  name: 'linear',         label: 'Linear',       category: 'Linear', fn: x => x },
  { code: 2,  name: 'cubic',          label: 'Cubic (custom)', category: 'Custom', fn: x => x, hermite: true },
  { code: 3,  name: 'ease',           label: 'Ease',         category: 'CSS',
    bezier: [0.25, 0.1, 0.25, 1] },
  { code: 4,  name: 'easeIn',         label: 'Ease In',      category: 'CSS',
    bezier: [0.42, 0,   1,    1] },
  { code: 5,  name: 'easeOut',        label: 'Ease Out',     category: 'CSS',
    bezier: [0,    0,   0.58, 1] },
  { code: 6,  name: 'easeInOut',      label: 'Ease In Out',  category: 'CSS',
    bezier: [0.42, 0,   0.58, 1] },

  // ── easings.net — Sine ──────────────────────────────────────────
  { code: 10, name: 'easeInSine',     label: 'Ease In Sine',     category: 'Sine',
    fn: x => 1 - Math.cos((x * Math.PI) / 2) },
  { code: 11, name: 'easeOutSine',    label: 'Ease Out Sine',    category: 'Sine',
    fn: x => Math.sin((x * Math.PI) / 2) },
  { code: 12, name: 'easeInOutSine',  label: 'Ease In Out Sine', category: 'Sine',
    fn: x => -(Math.cos(Math.PI * x) - 1) / 2 },

  // ── Quad ────────────────────────────────────────────────────────
  { code: 13, name: 'easeInQuad',     label: 'Ease In Quad',     category: 'Quad',
    fn: x => x * x },
  { code: 14, name: 'easeOutQuad',    label: 'Ease Out Quad',    category: 'Quad',
    fn: x => 1 - (1 - x) * (1 - x) },
  { code: 15, name: 'easeInOutQuad',  label: 'Ease In Out Quad', category: 'Quad',
    fn: x => x < 0.5 ? 2 * x * x : 1 - Math.pow(-2 * x + 2, 2) / 2 },

  // ── Cubic (note: distinct from code 2 'cubic' which is Hermite) ─
  { code: 16, name: 'easeInCubic',    label: 'Ease In Cubic',    category: 'Cubic',
    fn: x => x * x * x },
  { code: 17, name: 'easeOutCubic',   label: 'Ease Out Cubic',   category: 'Cubic',
    fn: x => 1 - Math.pow(1 - x, 3) },
  { code: 18, name: 'easeInOutCubic', label: 'Ease In Out Cubic', category: 'Cubic',
    fn: x => x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2 },

  // ── Quart ───────────────────────────────────────────────────────
  { code: 19, name: 'easeInQuart',    label: 'Ease In Quart',    category: 'Quart',
    fn: x => x * x * x * x },
  { code: 20, name: 'easeOutQuart',   label: 'Ease Out Quart',   category: 'Quart',
    fn: x => 1 - Math.pow(1 - x, 4) },
  { code: 21, name: 'easeInOutQuart', label: 'Ease In Out Quart', category: 'Quart',
    fn: x => x < 0.5 ? 8 * x * x * x * x : 1 - Math.pow(-2 * x + 2, 4) / 2 },

  // ── Quint ───────────────────────────────────────────────────────
  { code: 22, name: 'easeInQuint',    label: 'Ease In Quint',    category: 'Quint',
    fn: x => x * x * x * x * x },
  { code: 23, name: 'easeOutQuint',   label: 'Ease Out Quint',   category: 'Quint',
    fn: x => 1 - Math.pow(1 - x, 5) },
  { code: 24, name: 'easeInOutQuint', label: 'Ease In Out Quint', category: 'Quint',
    fn: x => x < 0.5 ? 16 * x * x * x * x * x : 1 - Math.pow(-2 * x + 2, 5) / 2 },

  // ── Expo ────────────────────────────────────────────────────────
  { code: 25, name: 'easeInExpo',     label: 'Ease In Expo',     category: 'Expo',
    fn: x => x === 0 ? 0 : Math.pow(2, 10 * x - 10) },
  { code: 26, name: 'easeOutExpo',    label: 'Ease Out Expo',    category: 'Expo',
    fn: x => x === 1 ? 1 : 1 - Math.pow(2, -10 * x) },
  { code: 27, name: 'easeInOutExpo',  label: 'Ease In Out Expo', category: 'Expo',
    fn: x => x === 0 ? 0 : x === 1 ? 1
      : x < 0.5 ? Math.pow(2, 20 * x - 10) / 2
                : (2 - Math.pow(2, -20 * x + 10)) / 2 },

  // ── Circ ────────────────────────────────────────────────────────
  { code: 28, name: 'easeInCirc',     label: 'Ease In Circ',     category: 'Circ',
    fn: x => 1 - Math.sqrt(1 - Math.pow(x, 2)) },
  { code: 29, name: 'easeOutCirc',    label: 'Ease Out Circ',    category: 'Circ',
    fn: x => Math.sqrt(1 - Math.pow(x - 1, 2)) },
  { code: 30, name: 'easeInOutCirc',  label: 'Ease In Out Circ', category: 'Circ',
    fn: x => x < 0.5 ? (1 - Math.sqrt(1 - Math.pow(2 * x, 2))) / 2
                     : (Math.sqrt(1 - Math.pow(-2 * x + 2, 2)) + 1) / 2 },

  // ── Back (overshoots a little) ─────────────────────────────────
  { code: 31, name: 'easeInBack',     label: 'Ease In Back',     category: 'Back',
    fn: x => c3 * x * x * x - c1 * x * x },
  { code: 32, name: 'easeOutBack',    label: 'Ease Out Back',    category: 'Back',
    fn: x => 1 + c3 * Math.pow(x - 1, 3) + c1 * Math.pow(x - 1, 2) },
  { code: 33, name: 'easeInOutBack',  label: 'Ease In Out Back', category: 'Back',
    fn: x => x < 0.5
      ? (Math.pow(2 * x, 2) * ((c2 + 1) * 2 * x - c2)) / 2
      : (Math.pow(2 * x - 2, 2) * ((c2 + 1) * (x * 2 - 2) + c2) + 2) / 2 },

  // ── Elastic (rubber-band) ───────────────────────────────────────
  { code: 34, name: 'easeInElastic',  label: 'Ease In Elastic',  category: 'Elastic',
    fn: x => x === 0 ? 0 : x === 1 ? 1
      : -Math.pow(2, 10 * x - 10) * Math.sin((x * 10 - 10.75) * c4) },
  { code: 35, name: 'easeOutElastic', label: 'Ease Out Elastic', category: 'Elastic',
    fn: x => x === 0 ? 0 : x === 1 ? 1
      : Math.pow(2, -10 * x) * Math.sin((x * 10 - 0.75) * c4) + 1 },
  { code: 36, name: 'easeInOutElastic', label: 'Ease In Out Elastic', category: 'Elastic',
    fn: x => x === 0 ? 0 : x === 1 ? 1
      : x < 0.5
        ? -(Math.pow(2, 20 * x - 10) * Math.sin((20 * x - 11.125) * c5)) / 2
        : (Math.pow(2, -20 * x + 10) * Math.sin((20 * x - 11.125) * c5)) / 2 + 1 },

  // ── Bounce ──────────────────────────────────────────────────────
  { code: 37, name: 'easeInBounce',   label: 'Ease In Bounce',   category: 'Bounce',
    fn: x => 1 - bounceOut(1 - x) },
  { code: 38, name: 'easeOutBounce',  label: 'Ease Out Bounce',  category: 'Bounce',
    fn: x => bounceOut(x) },
  { code: 39, name: 'easeInOutBounce', label: 'Ease In Out Bounce', category: 'Bounce',
    fn: x => x < 0.5 ? (1 - bounceOut(1 - 2 * x)) / 2
                     : (1 + bounceOut(2 * x - 1)) / 2 },
]

// Fast lookup by code.
const _BY_CODE = Object.fromEntries(EASINGS.map(e => [e.code, e]))
export function easingByCode (code) { return _BY_CODE[code] || _BY_CODE[1] }

// Categorized list for the picker UI. Preserves catalog order within
// each category so menus look consistent.
export const EASING_CATEGORIES = (() => {
  const order = ['Step', 'Linear', 'CSS', 'Custom', 'Sine', 'Quad',
                 'Cubic', 'Quart', 'Quint', 'Expo', 'Circ', 'Back',
                 'Elastic', 'Bounce']
  const byCat = {}
  for (const cat of order) byCat[cat] = []
  for (const e of EASINGS) {
    if (!byCat[e.category]) byCat[e.category] = []
    byCat[e.category].push(e)
  }
  return order.map(cat => ({ category: cat, items: byCat[cat] }))
                  .filter(g => g.items.length)
})()

// Sample an easing function. Returns y(t) in [0,1] for t in [0,1].
// Falls back to linear if the easing has no fn (shouldn't happen for
// non-step entries).
export function sampleEasing (code, t) {
  if (t <= 0) return 0
  if (t >= 1) return 1
  const e = easingByCode(code)
  if (e?.step) return 0           // constant: floor for the whole segment
  if (e?.fn)   return e.fn(t)
  return t
}

// Optional: SVG path snippet for a single segment, given the easing
// code and segment bounding box (x0,y0)→(x1,y1) in any units. Used by
// the timeline's curve renderer.
//
// For cubic-bezier easings (CSS basics), emit a single SVG C command
// using the bezier control points scaled into segment space.
// For Hermite (code 2), emit a C derived from tangents (caller must
// pass tangentInfo).
// For everything else, sample `fn` at N points and emit L commands.
const POLYLINE_SAMPLES = 28

export function segmentPath (code, x0, y0, x1, y1, opts = {}) {
  const dx = x1 - x0
  const dy = y1 - y0
  const e = easingByCode(code)
  if (e?.step) {
    return ` L ${x1} ${y0} L ${x1} ${y1}`
  }
  if (e?.bezier) {
    const [c1x, c1y, c2x, c2y] = e.bezier
    const cp1x = x0 + dx * c1x
    const cp1y = y0 + dy * c1y
    const cp2x = x0 + dx * c2x
    const cp2y = y0 + dy * c2y
    return ` C ${cp1x},${cp1y} ${cp2x},${cp2y} ${x1},${y1}`
  }
  if (e?.hermite) {
    // Hermite tangents stored on the segment's endpoints; opts
    // provides them.
    const { leave = 0, arrive = 0, dt = 1 } = opts
    const cp1x = x0 + dx / 3
    const cp1y = y0 + (leave * dt) / 3 * (dy / Math.abs(dy || 1))
    const cp2x = x1 - dx / 3
    const cp2y = y1 - (arrive * dt) / 3 * (dy / Math.abs(dy || 1))
    return ` C ${cp1x},${cp1y} ${cp2x},${cp2y} ${x1},${y1}`
  }
  if (e?.fn) {
    let out = ''
    for (let i = 1; i <= POLYLINE_SAMPLES; i++) {
      const t = i / POLYLINE_SAMPLES
      const y = e.fn(t)
      const px = x0 + dx * t
      const py = y0 + dy * y
      out += ` L ${px} ${py}`
    }
    return out
  }
  return ` L ${x1} ${y1}`
}

// 14×10 SVG glyph showing the curve shape — derived directly from the
// easing function so the icon always matches the actual curve.
export function easingGlyph (code) {
  const e = easingByCode(code)
  if (e?.step) {
    return '<svg viewBox="0 0 14 10" width="14" height="10" stroke="currentColor" stroke-width="1.5" fill="none"><path d="M1 8 L6 8 L6 2 L13 2"/></svg>'
  }
  // Build a polyline glyph from sampled values. Looks consistent
  // across all categories.
  const W = 12, H = 6, OX = 1, OY = 2
  let d = `M${OX} ${OY + H}`
  const N = 24
  for (let i = 1; i <= N; i++) {
    const t = i / N
    let y = t
    if (e?.bezier) {
      // Quick cubic-bezier parametric approximation — exact y at t=s
      // isn't critical for a 14-px glyph.
      const cp = e.bezier
      const one = 1 - t
      y = 3 * one * one * t * cp[1]
        + 3 * one * t * t * cp[3]
        + t * t * t
    } else if (e?.fn) {
      y = e.fn(t)
    }
    const px = OX + W * t
    const py = (OY + H) - H * y
    d += ` L${px.toFixed(2)} ${py.toFixed(2)}`
  }
  return `<svg viewBox="0 0 14 10" width="14" height="10" stroke="currentColor" stroke-width="1.4" fill="none" stroke-linejoin="round" stroke-linecap="round"><path d="${d}"/></svg>`
}
