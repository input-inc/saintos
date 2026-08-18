<script setup>
import { computed } from 'vue'

// Linear-travel visualisation for a Kangaroo-driven actuator.
//
// Deliberately NOT a reuse of ServoExtentsControl: that renders a 180°
// arc with four draggable handles, because a servo's extents are typed
// in microseconds and the servo follows. Here the mental model is the
// opposite — the Kangaroo's travel limits are read-only on the wire and
// come from where the actuator was physically jogged during the teach.
// So there is nothing to drag. This is a readout: a stroke bar showing
// live position, the endpoints the operator captured, and (after a tune
// and a power cycle) the limits the Kangaroo actually taught itself.
//
// Everything is optional — before the first tune there are no taught
// limits, and at the start of a teach there are no marks either. The
// component degrades to an empty track rather than inventing a scale.

const props = defineProps({
  position:   { type: Number, default: null },  // live, machine units
  retract:    { type: Number, default: null },  // operator-captured
  extend:     { type: Number, default: null },
  center:     { type: Number, default: null },
  taughtMin:  { type: Number, default: null },  // read back via Get 8/9
  taughtMax:  { type: Number, default: null },
})

const nums = (...vals) => vals.filter(v => typeof v === 'number' && isFinite(v))

// Scale spans whatever we actually know about, with a little padding so
// a marker sitting exactly at an end isn't clipped by the track edge.
// With a single known value the range would collapse to zero width, so
// fall back to a nominal span and centre it.
const domain = computed(() => {
  const known = nums(props.position, props.retract, props.extend,
                     props.center, props.taughtMin, props.taughtMax)
  if (!known.length) return null
  let lo = Math.min(...known)
  let hi = Math.max(...known)
  if (hi - lo < 1) { lo -= 500; hi += 500 }
  const pad = (hi - lo) * 0.06
  return { lo: lo - pad, hi: hi + pad }
})

function pct (v) {
  const d = domain.value
  if (!d || typeof v !== 'number' || !isFinite(v)) return null
  return Math.max(0, Math.min(100, ((v - d.lo) / (d.hi - d.lo)) * 100))
}

const positionPct = computed(() => pct(props.position))

// The span the operator captured this session, drawn as a filled band.
const markedBand = computed(() => {
  const a = pct(props.retract)
  const b = pct(props.extend)
  if (a == null || b == null) return null
  return { left: Math.min(a, b), width: Math.abs(b - a) }
})

// The span the Kangaroo reports it taught itself. Drawn separately and
// underneath — if these two disagree noticeably, something went wrong in
// the teach and the operator should see that rather than have one
// quietly drawn over the other.
const taughtBand = computed(() => {
  const a = pct(props.taughtMin)
  const b = pct(props.taughtMax)
  if (a == null || b == null) return null
  return { left: Math.min(a, b), width: Math.abs(b - a) }
})

const MARKERS = [
  { key: 'retract', label: 'Retract', color: '#f43f5e' },
  { key: 'center',  label: 'Centre',  color: '#a78bfa' },
  { key: 'extend',  label: 'Extend',  color: '#22d3ee' },
]
const markerViews = computed(() =>
  MARKERS.map(m => ({ ...m, pct: pct(props[m.key]), value: props[m.key] }))
         .filter(m => m.pct != null))
</script>

<template>
  <div class="space-y-2">
    <div class="flex items-baseline justify-between">
      <span class="text-xs text-fg-muted">Travel</span>
      <span class="text-xs font-mono text-fg-faint">
        {{ position ?? '—' }} units
      </span>
    </div>

    <!-- Track. Height leaves room for the marker flags above and the
         taught band below without them overlapping. -->
    <div class="relative h-14">
      <!-- Base rail -->
      <div class="absolute inset-x-0 top-6 h-2.5 rounded-full bg-slate-700/50 overflow-hidden">
        <!-- Operator-captured span -->
        <div v-if="markedBand"
             class="absolute inset-y-0 bg-cyan-500/30"
             :style="{ left: `${markedBand.left}%`, width: `${markedBand.width}%` }" />
      </div>

      <!-- Live position -->
      <div v-if="positionPct != null"
           class="absolute top-[18px] w-1 h-6 rounded-full bg-white shadow"
           :style="{ left: `calc(${positionPct}% - 2px)` }" />

      <!-- Captured endpoint flags -->
      <div v-for="m in markerViews" :key="m.key"
           class="absolute top-0 flex flex-col items-center pointer-events-none"
           :style="{ left: `${m.pct}%`, transform: 'translateX(-50%)' }">
        <span class="text-[9px] leading-none whitespace-nowrap" :style="{ color: m.color }">
          {{ m.label }}
        </span>
        <span class="w-0.5 h-4 mt-0.5" :style="{ backgroundColor: m.color }" />
      </div>

      <!-- Taught band, reported by the Kangaroo after a tune -->
      <div v-if="taughtBand"
           class="absolute top-10 h-1.5 rounded-full bg-emerald-500/50"
           :style="{ left: `${taughtBand.left}%`, width: `${taughtBand.width}%` }" />
      <div v-if="taughtBand"
           class="absolute top-[46px] text-[9px] text-emerald-400"
           :style="{ left: `${taughtBand.left}%` }">
        taught
      </div>
    </div>

    <p v-if="!domain" class="text-[11px] text-fg-faint">
      No position reading yet.
    </p>
  </div>
</template>
