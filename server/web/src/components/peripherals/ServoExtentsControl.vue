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
// Each handle rides its OWN concentric ring so the dots never overlap,
// even when two values coincide (home defaults to neutral == center,
// so those two collided constantly on a single-ring layout). Labels
// sit at the end of a radial leader line drawn through each handle, so
// they read clearly instead of stacking on top of one another.
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
// 'preview' fires with the absolute µs of the handle being moved so a
// parent can jog the real servo there for live dial-in (see the
// channel-edit modal in views/node/Peripherals.vue). Harmless to ignore
// where no live channel is bound (the peripheral-level defaults editor).
const emit = defineEmits(['update:modelValue', 'preview'])

// SVG geometry. The arc spans 180° (from 180° on the left, through
// 90° at the top, to 0° on the right) — the upper half of a circle.
// The handle x-axis runs left=min_pulse → right=max_pulse.
const W = 320          // viewBox width
const H = 184          // viewBox height (room for the outer labels)
const CX = W / 2
const CY = 150         // arc center (handles sit on the upper half)
const R  = 104         // outer arc radius (start/end + backdrop)
const LABEL_GAP = 16   // radial distance from a handle out to its label

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
function polarAt (us, radius) {
  const rad = (usToAngleDeg(us) * Math.PI) / 180
  return { x: CX + radius * Math.cos(rad), y: CY - radius * Math.sin(rad) }
}

// `ring` is the radius each handle rides on. start/end stay on the
// outer arc (they bound the highlighted sweep); center and home drop
// to inner rings so they can never sit under start/end or each other.
const HANDLES = [
  { key: 'start_us',  label: 'Start',  color: '#f43f5e', ring: R      },  // rose
  { key: 'end_us',    label: 'End',    color: '#22d3ee', ring: R      },  // cyan
  { key: 'center_us', label: 'Center', color: '#a78bfa', ring: R - 24 },  // violet
  { key: 'home_us',   label: 'Home',   color: '#f59e0b', ring: R - 44 },  // amber
]

const svgRef = ref(null)
const dragging = ref(null)   // 'start_us' | 'end_us' | 'center_us' | 'home_us'

function update (key, us) {
  emit('update:modelValue', { ...props.modelValue, [key]: us })
  emit('preview', us)
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
  // The handles live on the upper 180° semicircle. atan2 returns
  // (-π, π]; on the upper half that's the [0, π] we want. When the
  // pointer drops BELOW the horizontal axis (dy < 0 → rad < 0), atan2
  // wraps toward the opposite end of the arc — which made a handle
  // "jump to the other side" chasing the mouse. Instead, clamp to
  // whichever end the pointer is nearest: left → 180° (min), right →
  // 0° (max), so the handle just parks at the edge it left.
  let rad = Math.atan2(dy, dx)
  if (rad < 0) rad = (dx < 0) ? Math.PI : 0
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

// Per-handle geometry: the dot (on its ring), the label point (one
// LABEL_GAP further out along the same ray), and a text-anchor chosen
// from which side of the dial the label lands on.
const handleViews = computed(() =>
  HANDLES.map(h => {
    const us = props.modelValue?.[h.key]
    if (typeof us !== 'number') return null
    const dot      = polarAt(us, h.ring)
    const labelPos = polarAt(us, h.ring + LABEL_GAP)
    const anchor = labelPos.x < CX - 6 ? 'end' : labelPos.x > CX + 6 ? 'start' : 'middle'
    return { ...h, dot, labelPos, anchor }
  }).filter(Boolean)
)

// Highlight arc from start_us → end_us so the operator can see the
// active sweep at a glance. Respects whichever direction the operator
// chose (start can be left or right of end — servos mount reversed).
const sweepPath = computed(() => {
  const s = props.modelValue?.start_us
  const e = props.modelValue?.end_us
  if (typeof s !== 'number' || typeof e !== 'number') return ''
  const sp = polarAt(s, R)
  const ep = polarAt(e, R)
  const sweepFlag = usToAngleDeg(s) > usToAngleDeg(e) ? 1 : 0
  return `M ${sp.x.toFixed(2)} ${sp.y.toFixed(2)} A ${R} ${R} 0 0 ${sweepFlag} ${ep.x.toFixed(2)} ${ep.y.toFixed(2)}`
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
      <!-- Tick at center top + label for visual symmetry. -->
      <line :x1="CX" :y1="CY - R - 4" :x2="CX" :y2="CY - R + 8"
            stroke="rgba(148,163,184,0.55)" stroke-width="1.5" />
      <text :x="CX" :y="CY - R - 8" text-anchor="middle"
            class="fill-fg-faint" font-size="10">midpoint</text>

      <!-- Handles: each on its own ring, with a radial leader line out
           to its label so labels never stack. -->
      <g v-for="h in handleViews" :key="h.key">
        <line
          :x1="CX" :y1="CY"
          :x2="h.labelPos.x" :y2="h.labelPos.y"
          :stroke="h.color"
          stroke-width="1.5"
          stroke-dasharray="3 3"
          opacity="0.55"
        />
        <circle
          :cx="h.dot.x" :cy="h.dot.y"
          r="9"
          :fill="h.color"
          stroke="white"
          stroke-width="2"
          class="cursor-grab active:cursor-grabbing"
          @pointerdown="onDown(h.key, $event)"
        />
        <text
          :x="h.labelPos.x" :y="h.labelPos.y - 2"
          :text-anchor="h.anchor"
          class="fill-fg-strong pointer-events-none"
          font-size="10"
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
