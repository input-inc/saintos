<script setup>
import { computed, ref, watch } from 'vue'

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

  // ── Live dial-in section (current monitor + auto-move + stop) ──────
  // Off by default so the peripheral-level defaults editor and the
  // Histoire story stay minimal (no live servo bound there). Any
  // servo-type editor that CAN jog a real channel (Maestro channel
  // modal, native servo edit modal, and the upcoming Pimoroni editor)
  // sets liveJog and feeds a live current reading in.
  liveJog: { type: Boolean, default: false },
  // Live current draw for the bound servo/channel, in AMPS, or null
  // when no current source is resolvable on the node. Resolved by the
  // parent from either the servo driver's own current channel (if it
  // reports one) or a separate current-monitor peripheral.
  current: { type: Number, default: null },
  // Human label for where the reading comes from, e.g.
  // "RoboClaw · current". Empty ⇒ "no current source on this node".
  currentSource: { type: String, default: '' },
  // Fixed full-scale for the bar in amps. 0 ⇒ auto-scale off the
  // observed peak (with a small floor so a tiny reading isn't slammed
  // to 100%).
  currentFullScale: { type: Number, default: 0 },
})
// 'preview' fires with the absolute µs of the handle being moved so a
// parent can jog the real servo there for live dial-in (see the
// channel-edit modal in views/node/Peripherals.vue). Harmless to ignore
// where no live channel is bound (the peripheral-level defaults editor).
// 'stop' fires when the operator hits Stop — the parent freezes the
// servo at its last commanded pulse.
const emit = defineEmits(['update:modelValue', 'preview', 'stop'])

// When auto-move is off, dragging/typing a handle still edits the
// extents but does NOT jog the real servo — so an operator can set up
// positions without the servo chasing every handle (and without loading
// it against a mechanical limit while they read the current). Default
// on: the whole point of the dial is live dial-in.
const autoMove = ref(true)

// Below this the servo is drawing essentially no holding current — it's
// balanced/back-driven and needs no power to stay put. That's exactly
// the "sweet spot" this bar exists to find.
const HOLD_FREE_A = 0.05

// Peak-hold so a transient spike (servo slamming into a hard stop as an
// extent is dialed past its mechanical limit) stays visible after the
// instantaneous reading falls back. Reset manually or when the section
// (re)mounts via a fresh key from the parent.
const peakCurrent = ref(0)
watch(() => props.current, (a) => {
  if (typeof a === 'number' && a > peakCurrent.value) peakCurrent.value = a
})
function resetPeak () { peakCurrent.value = typeof props.current === 'number' ? props.current : 0 }

const hasCurrent = computed(() => typeof props.current === 'number')
const holdingFree = computed(() => hasCurrent.value && props.current <= HOLD_FREE_A)

// Bar scale: fixed full-scale if given, else auto off the peak with a
// 0.5 A floor and ~15% headroom so the fill doesn't peg at the edge.
const barScale = computed(() => {
  if (props.currentFullScale > 0) return props.currentFullScale
  return Math.max(0.5, peakCurrent.value * 1.15)
})
const barFrac = computed(() =>
  hasCurrent.value ? Math.max(0, Math.min(1, props.current / barScale.value)) : 0)
const peakFrac = computed(() =>
  Math.max(0, Math.min(1, peakCurrent.value / barScale.value)))
// Green when holding-free, ramping cyan → amber → rose as draw climbs.
const barColor = computed(() => {
  if (!hasCurrent.value) return '#475569'          // slate — no reading
  if (holdingFree.value) return '#22c55e'          // green — free-holding
  const f = barFrac.value
  if (f < 0.5) return '#22d3ee'                    // cyan
  if (f < 0.8) return '#f59e0b'                    // amber
  return '#f43f5e'                                 // rose
})
const currentText = computed(() =>
  hasCurrent.value ? `${props.current.toFixed(2)} A` : '—')

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
  // Only jog the real servo when auto-move is enabled. The extents still
  // update either way — the toggle just decouples the physical servo
  // from the handle so positions can be set without moving hardware.
  if (autoMove.value) emit('preview', us)
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

    <!-- ── Live dial-in: current monitor, auto-move toggle, stop ──────
         Only shown when a real servo channel is bound (liveJog). Lets
         the operator watch holding current while dialing an extent so
         they can land on a position the servo holds with no power. -->
    <div v-if="liveJog" class="pt-3 mt-1 border-t border-line space-y-2">
      <div class="flex items-center justify-between gap-3">
        <div class="flex items-center gap-2">
          <label class="inline-flex items-center gap-1.5 cursor-pointer select-none">
            <input type="checkbox" v-model="autoMove" class="accent-cyan-500" />
            <span class="text-xs text-fg-muted">Move servo to handle</span>
          </label>
        </div>
        <button
          type="button"
          class="btn-secondary text-xs py-1 px-2 flex items-center gap-1"
          title="Freeze the servo at its last commanded pulse"
          @click="emit('stop')"
        >
          <span class="material-icons icon-sm">stop</span>
          Stop
        </button>
      </div>

      <!-- Current bar with peak-hold marker. -->
      <div class="space-y-1">
        <div class="flex items-baseline justify-between">
          <span class="text-xs text-fg-muted">Current draw</span>
          <span class="text-xs font-mono tabular-nums" :style="{ color: barColor }">
            {{ currentText }}
            <span v-if="holdingFree" class="ml-1 text-[10px] text-green-400 font-sans">holding-free</span>
          </span>
        </div>
        <div class="relative h-2.5 rounded-full bg-slate-700/50 overflow-hidden">
          <div
            class="absolute inset-y-0 left-0 rounded-full transition-[width] duration-150"
            :style="{ width: `${barFrac * 100}%`, backgroundColor: barColor }"
          />
          <!-- Peak-hold tick -->
          <div
            v-if="hasCurrent && peakCurrent > 0"
            class="absolute inset-y-0 w-0.5 bg-white/70"
            :style="{ left: `calc(${peakFrac * 100}% - 1px)` }"
          />
        </div>
        <div class="flex items-center justify-between text-[10px] text-fg-faint">
          <span>{{ currentSource || 'No current source on this node' }}</span>
          <span v-if="hasCurrent">
            peak {{ peakCurrent.toFixed(2) }} A
            <button type="button" class="ml-1 underline hover:text-fg-muted" @click="resetPeak">reset</button>
          </span>
        </div>
      </div>

      <p class="text-[11px] text-fg-faint leading-snug">
        Let the current reading settle before setting an extent — watch for
        spikes as the servo loads against a mechanical limit. A near-zero,
        steady draw (<span class="text-green-400">holding-free</span>) means the
        servo holds this position with no power.
      </p>
    </div>
  </div>
</template>
