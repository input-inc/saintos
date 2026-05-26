<script setup>
import { computed } from 'vue'

// Inline SVG sparkline of the recent rolling window. Renders a thin
// slate baseline when there's no data so card layouts don't reflow
// once the first sample arrives.
const props = defineProps({
  samples:  { type: Array,  default: () => [] },   // [{ts, value}]
  width:    { type: Number, default: 140 },
  height:   { type: Number, default: 24 },
  windowMs: { type: Number, default: 30000 },
})

function fmt (v) {
  if (typeof v !== 'number') return String(v)
  if (Number.isInteger(v)) return v.toString()
  return v.toFixed(3)
}

const stats = computed(() => {
  const s = props.samples
  if (!s || s.length < 2) return null
  let lo = Infinity, hi = -Infinity
  for (const { value } of s) {
    if (value < lo) lo = value
    if (value > hi) hi = value
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return null
  return { lo, hi }
})

const polyline = computed(() => {
  const s = props.samples
  const st = stats.value
  if (!st) return null
  const W = props.width
  const H = props.height
  const now = Date.now()
  const t0 = now - props.windowMs

  // Pad the y-range ±5% so the line never grazes the SVG edge. Falls
  // back to a ±0.5 pad when the value is flat (lo === hi) so a steady
  // reading renders as a centred horizontal line rather than NaN.
  let { lo, hi } = st
  if (hi === lo) { lo -= 0.5; hi += 0.5 }
  else {
    const pad = (hi - lo) * 0.05
    lo -= pad
    hi += pad
  }
  const span = hi - lo
  const yPad = 1
  const innerH = H - 2 * yPad
  const points = []
  for (const { ts, value } of s) {
    const x = Math.max(0, Math.min(W, ((ts - t0) / props.windowMs) * W))
    const y = (H - yPad) - ((value - lo) / span) * innerH
    points.push(`${x.toFixed(1)},${y.toFixed(1)}`)
  }
  const last = s[s.length - 1]
  const lx = Math.max(0, Math.min(W, ((last.ts - t0) / props.windowMs) * W))
  const ly = (H - yPad) - ((last.value - lo) / span) * innerH
  return {
    points: points.join(' '),
    lx: lx.toFixed(1),
    ly: ly.toFixed(1),
    lo: st.lo, // tooltip shows raw min/max, not padded
    hi: st.hi,
  }
})

const title = computed(() => {
  if (!polyline.value) return 'no data yet'
  return `min: ${fmt(polyline.value.lo)} / max: ${fmt(polyline.value.hi)} (${(props.windowMs / 1000) | 0}s)`
})
</script>

<template>
  <svg
    :width="width"
    :height="height"
    :viewBox="`0 0 ${width} ${height}`"
    class="text-cyan-400 inline-block align-middle"
    :title="title"
    :aria-label="title"
  >
    <template v-if="polyline">
      <polyline
        fill="none"
        stroke="currentColor"
        stroke-width="1.25"
        stroke-linejoin="round"
        stroke-linecap="round"
        :points="polyline.points"
      />
      <circle
        :cx="polyline.lx"
        :cy="polyline.ly"
        r="1.5"
        fill="currentColor"
      />
    </template>
    <line
      v-else
      :x1="0" :y1="height / 2"
      :x2="width" :y2="height / 2"
      stroke="rgb(71 85 105 / 0.4)"
      stroke-width="1"
      stroke-dasharray="2 3"
    />
  </svg>
</template>
