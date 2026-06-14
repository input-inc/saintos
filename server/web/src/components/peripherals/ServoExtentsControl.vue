<script setup>
import { computed, ref } from 'vue'

// Radial servo-extent picker. Renders a 180° arc (the mechanical
// sweep of a typical hobby servo) and four draggable handles:
//
//   start  — pulse driven when the routed signal is −1
//   center — pulse driven when the routed signal is 0
//   end    — pulse driven when the routed signal is +1
//   home   — pulse driven at startup / safe-reset
//
// The component is presentation-only — it edits a v-model'd object of
// shape {start_us, end_us, center_us, home_us}. Numeric inputs sit
// under the dial so an operator can type exact pulse widths instead
// of dragging if they already know the values.

const props = defineProps({
  modelValue: {
    type: Object,
    required: true,    // { start_us, end_us, center_us, home_us }
  },
  // Operator-side pulse-width bounds. Same defaults as the catalog
  // params' min/max so the dial doesn't let an operator drag past
  // what the server will accept.
  minPulseUs: { type: Number, default: 500  },
  maxPulseUs: { type: Number, default: 2500 },
})
const emit = defineEmits(['update:modelValue'])

// SVG geometry. The arc spans 180° (from 180° on the left, through
// 270° at the top, to 360°/0° on the right) — i.e. the upper half
// of a circle. The handle x-axis runs left=min_pulse → right=max_pulse.
const W = 320          // viewBox width
const H = 180          // viewBox height (extra space for the readouts)
const CX = W / 2
const CY = 150         // arc center (handles sit on the upper half)
const R  = 120

function usToAngleDeg (us) {
  const lo = props.minPulseUs
  const hi = props.maxPulseUs
  const t = Math.max(0, Math.min(1, (us - lo) / (hi - lo)))
  // 180° at min → 0° at max, swept counter-clockwise through 90° at midpoint.
  return 180 - t * 180
}
function angleDegToUs (deg) {
  const lo = props.minPulseUs
  const hi = props.maxPulseUs
  const clamped = Math.max(0, Math.min(180, deg))
  const t = 1 - (clamped / 180)
  return Math.round(lo + t * (hi - lo))
}
function polar (us) {
  const rad = (usToAngleDeg(us) * Math.PI) / 180
  return { x: CX + R * Math.cos(rad), y: CY - R * Math.sin(rad) }
}

const HANDLES = [
  { key: 'start_us',  label: 'Start',  color: '#f43f5e' },  // rose
  { key: 'end_us',    label: 'End',    color: '#22d3ee' },  // cyan
  { key: 'center_us', label: 'Center', color: '#a78bfa' },  // violet
  { key: 'home_us',   label: 'Home',   color: '#f59e0b' },  // amber
]

const svgRef = ref(null)
const dragging = ref(null)   // 'start_us' | 'end_us' | 'center_us' | 'home_us'

function update (key, us) {
  emit('update:modelValue', { ...props.modelValue, [key]: us })
}

function pointerToUs (evt) {
  const svg = svgRef.value
  if (!svg) return null
  const pt = svg.createSVGPoint()
  pt.x = evt.clientX
  pt.y = evt.clientY
  const ctm = svg.getScreenCTM()
  if (!ctm) return null
  const local = pt.matrixTransform(ctm.inverse())
  const dx = local.x - CX
  const dy = CY - local.y    // SVG y grows down; flip so up is +
  // atan2 returns (-π, π]; clamp to upper half (0..π) for the 180° arc.
  let rad = Math.atan2(dy, dx)
  if (rad < 0) rad = 0
  if (rad > Math.PI) rad = Math.PI
  const deg = (rad * 180) / Math.PI
  return angleDegToUs(deg)
}

function onDown (key, evt) {
  dragging.value = key
  evt.target.setPointerCapture?.(evt.pointerId)
  evt.preventDefault()
}
function onMove (evt) {
  if (!dragging.value) return
  const us = pointerToUs(evt)
  if (us !== null) update(dragging.value, us)
}
function onUp (evt) {
  if (!dragging.value) return
  evt.target.releasePointerCapture?.(evt.pointerId)
  dragging.value = null
}

function onNumericInput (key, e) {
  const v = parseInt(e.target.value, 10)
  if (Number.isNaN(v)) return
  const clamped = Math.max(props.minPulseUs, Math.min(props.maxPulseUs, v))
  update(key, clamped)
}

const positions = computed(() => {
  const out = {}
  for (const h of HANDLES) {
    const us = props.modelValue?.[h.key]
    if (typeof us === 'number') out[h.key] = polar(us)
  }
  return out
})

// Highlight arc from start_us → end_us so the operator can see the
// active sweep at a glance. The arc respects whichever direction the
// operator has chosen (start can be left or right of end on the dial,
// since servos can mount reversed).
const sweepPath = computed(() => {
  const s = positions.value.start_us
  const e = positions.value.end_us
  if (!s || !e) return ''
  // Both points lie on the upper semicircle; the sweep is always less
  // than 180° so largeArcFlag is 0. Sweep direction (sweepFlag) depends
  // on which side of the dial start is on relative to end.
  const startAngle = usToAngleDeg(props.modelValue.start_us)
  const endAngle   = usToAngleDeg(props.modelValue.end_us)
  const sweepFlag  = startAngle > endAngle ? 1 : 0
  return `M ${s.x.toFixed(2)} ${s.y.toFixed(2)} A ${R} ${R} 0 0 ${sweepFlag} ${e.x.toFixed(2)} ${e.y.toFixed(2)}`
})
</script>

<template>
  <div class="space-y-2">
    <svg
      ref="svgRef"
      :viewBox="`0 0 ${W} ${H}`"
      class="w-full select-none touch-none"
      @pointermove="onMove"
      @pointerup="onUp"
      @pointercancel="onUp"
    >
      <!-- Backdrop arc: full 180° in faint stroke. -->
      <path
        :d="`M ${CX - R} ${CY} A ${R} ${R} 0 0 1 ${CX + R} ${CY}`"
        fill="none"
        stroke="rgba(148,163,184,0.25)"
        stroke-width="6"
        stroke-linecap="round"
      />
      <!-- Active sweep arc: from start to end. -->
      <path
        v-if="sweepPath"
        :d="sweepPath"
        fill="none"
        stroke="rgba(34,211,238,0.4)"
        stroke-width="6"
        stroke-linecap="round"
      />
      <!-- Tick at center bottom + dead-center line for visual symmetry. -->
      <line :x1="CX" :y1="CY - R - 4" :x2="CX" :y2="CY - R + 8"
            stroke="rgba(148,163,184,0.55)" stroke-width="1.5" />
      <text :x="CX" :y="CY - R - 8" text-anchor="middle"
            class="fill-fg-faint" font-size="10">midpoint</text>

      <!-- Handles -->
      <g v-for="h in HANDLES" :key="h.key">
        <line
          v-if="positions[h.key]"
          :x1="CX" :y1="CY"
          :x2="positions[h.key].x" :y2="positions[h.key].y"
          :stroke="h.color"
          stroke-width="1.5"
          stroke-dasharray="3 3"
          opacity="0.6"
        />
        <circle
          v-if="positions[h.key]"
          :cx="positions[h.key].x"
          :cy="positions[h.key].y"
          r="10"
          :fill="h.color"
          stroke="white"
          stroke-width="2"
          class="cursor-grab active:cursor-grabbing"
          @pointerdown="onDown(h.key, $event)"
        />
        <text
          v-if="positions[h.key]"
          :x="positions[h.key].x"
          :y="positions[h.key].y - 16"
          text-anchor="middle"
          class="fill-fg-strong pointer-events-none"
          font-size="11"
          font-weight="600"
        >{{ h.label }}</text>
      </g>
    </svg>

    <!-- Numeric readouts: typing edits the same model values the dial drags. -->
    <div class="grid grid-cols-2 gap-2">
      <div v-for="h in HANDLES" :key="h.key" class="flex items-center gap-2">
        <span class="inline-block w-3 h-3 rounded-full" :style="{ backgroundColor: h.color }" />
        <label class="text-xs text-fg-muted w-14">{{ h.label }}</label>
        <input
          type="number"
          :min="minPulseUs" :max="maxPulseUs" step="10"
          :value="modelValue?.[h.key] ?? 0"
          @input="onNumericInput(h.key, $event)"
          class="input-field w-full text-sm"
        />
        <span class="text-[10px] text-fg-faint">µs</span>
      </div>
    </div>
  </div>
</template>
