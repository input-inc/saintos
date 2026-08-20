<script setup>
import { computed, ref } from 'vue'

// Linear-travel control for a Kangaroo-driven actuator.
//
// Same interaction model as ServoExtentsControl — draggable handles plus
// numeric entry — but a straight stroke bar instead of a 180° arc,
// because the thing being configured is linear travel and an arc would
// misrepresent it.
//
// Five draggable handles, matching the servo control:
//
//   retract / extend / centre   the operator's USABLE travel (soft_min,
//                               soft_max, soft_center). Firmware clamps
//                               commanded positions to them.
//   home / power-on             recall positions.
//
// The one thing that is NOT draggable is the taught travel band, and the
// distinction matters: the teach tune establishes the axis's HARDWARE
// range and the Kangaroo has no command to set it (Get 8/9 are
// read-only). What the handles edit is the working range INSIDE that —
// the same thing the Kangaroo manual calls soft limits.
//
// So the band is drawn as the backdrop the handles live within. A
// draggable band would move, the operator would believe they'd retaught
// the travel limits, and the Kangaroo would ignore it.
//
// Live position is read-only for obvious reasons.

const props = defineProps({
  position:   { type: Number, default: null },  // live, machine units
  // Usable travel — settable, so these drag.
  retract:    { type: Number, default: null },
  extend:     { type: Number, default: null },
  center:     { type: Number, default: null },
  // Taught hardware range, read back via Get 8/9. Read-only backdrop.
  taughtMin:  { type: Number, default: null },
  taughtMax:  { type: Number, default: null },
  // Recall positions. null hides the handle — power-on is only
  // meaningful once the operator has opted into it.
  home:       { type: Number, default: null },
  powerOn:    { type: Number, default: null },
  // Handles only drag when the caller can actually persist the result.
  editable:   { type: Boolean, default: false },
  // Fallback scale when nothing has been taught or read yet, so the bar
  // still has a sane domain to lay handles out on.
  rangeMin:   { type: Number, default: 0 },
  rangeMax:   { type: Number, default: 10000 },
})

const emit = defineEmits([
  'update:retract', 'update:extend', 'update:center',
  'update:home', 'update:powerOn',
])

const nums = (...vals) => vals.filter(v => typeof v === 'number' && isFinite(v))

// Scale spans everything known, padded so a handle sitting exactly at an
// end isn't clipped. Prefer the taught range when we have it — that's
// the axis's real travel and the most honest axis for the handles.
const domain = computed(() => {
  const taught = nums(props.taughtMin, props.taughtMax)
  let lo, hi
  if (taught.length === 2) {
    lo = Math.min(...taught); hi = Math.max(...taught)
  } else {
    const known = nums(props.position, props.retract, props.extend,
                       props.center, props.home, props.powerOn)
    if (known.length) {
      lo = Math.min(...known); hi = Math.max(...known)
    } else {
      lo = props.rangeMin; hi = props.rangeMax
    }
  }
  if (hi - lo < 1) { lo -= 500; hi += 500 }
  const pad = (hi - lo) * 0.08
  return { lo: lo - pad, hi: hi + pad }
})

function pct (v) {
  const d = domain.value
  if (typeof v !== 'number' || !isFinite(v)) return null
  return Math.max(0, Math.min(100, ((v - d.lo) / (d.hi - d.lo)) * 100))
}
function valueAtPct (p) {
  const d = domain.value
  const t = Math.max(0, Math.min(1, p / 100))
  return Math.round(d.lo + t * (d.hi - d.lo))
}

const positionPct = computed(() => pct(props.position))

// The span the Kangaroo reports it taught itself. Drawn separately from
// the usable band rather than over it — if the usable window sits
// outside the taught travel, that's a real misconfiguration and worth
// seeing rather than hiding.
const taughtBand = computed(() => {
  const a = pct(props.taughtMin)
  const b = pct(props.taughtMax)
  if (a == null || b == null) return null
  return { left: Math.min(a, b), width: Math.abs(b - a) }
})

// ── Draggable handles ───────────────────────────────────────────────
// `row` staggers them vertically so two handles at the same value stay
// individually grabbable — the servo dial solves the same problem with
// concentric rings. Home defaults to a travel end often enough that
// overlap is the normal case, not an edge case.
const HANDLES = [
  { key: 'retract', event: 'update:retract', label: 'Retract',  color: '#f43f5e', row: 0 },
  { key: 'center',  event: 'update:center',  label: 'Centre',   color: '#a78bfa', row: 0 },
  { key: 'extend',  event: 'update:extend',  label: 'Extend',   color: '#22d3ee', row: 0 },
  { key: 'home',    event: 'update:home',    label: 'Home',     color: '#f59e0b', row: 1 },
  { key: 'powerOn', event: 'update:powerOn', label: 'Power-on', color: '#34d399', row: 2 },
]
const ROW_TOP = [26, 52, 74]     // px offsets within the track

const handleViews = computed(() =>
  HANDLES.map(h => ({
    ...h,
    value: props[h.key],
    pct: pct(props[h.key]),
    top: ROW_TOP[h.row],
  })).filter(h => h.pct != null))

// Usable-travel band, drawn between the retract and extend handles so
// the operator can see the window they're editing.
const usableBand = computed(() => {
  const a = pct(props.retract)
  const b = pct(props.extend)
  if (a == null || b == null) return null
  return { left: Math.min(a, b), width: Math.abs(b - a) }
})

const trackRef = ref(null)
const dragging = ref(null)

function pctFromEvent (evt) {
  const el = trackRef.value
  if (!el) return null
  const r = el.getBoundingClientRect()
  if (r.width <= 0) return null
  return ((evt.clientX - r.left) / r.width) * 100
}

function onDown (h, evt) {
  if (!props.editable) return
  dragging.value = h.key
  evt.target.setPointerCapture?.(evt.pointerId)
  evt.preventDefault()
}
function onMove (evt) {
  if (!dragging.value) return
  const p = pctFromEvent(evt)
  if (p == null) return
  const h = HANDLES.find(x => x.key === dragging.value)
  if (!h) return
  let v = valueAtPct(p)
  // Keep the window coherent while dragging: retract can't cross extend.
  // Without this the pair inverts and the firmware silently rejects the
  // whole soft-limit set (it requires min < max), so the operator's edit
  // would just vanish on save.
  if (h.key === 'retract' && typeof props.extend === 'number') {
    v = Math.min(v, props.extend - 1)
  } else if (h.key === 'extend' && typeof props.retract === 'number') {
    v = Math.max(v, props.retract + 1)
  }
  emit(h.event, v)
}
function onUp (evt) {
  if (!dragging.value) return
  evt.target.releasePointerCapture?.(evt.pointerId)
  dragging.value = null
}

function onNumericInput (h, e) {
  const v = parseInt(e.target.value, 10)
  if (Number.isNaN(v)) return
  emit(h.event, v)
}

// Clamp warning: a home or power-on position outside the taught travel
// can't be reached. Worth flagging rather than letting the operator
// wonder why the axis stops short.
function outsideTaught (v) {
  if (typeof v !== 'number') return false
  const lo = props.taughtMin, hi = props.taughtMax
  if (typeof lo !== 'number' || typeof hi !== 'number') return false
  if (hi <= lo) return false
  return v < lo || v > hi
}
const anyOutside = computed(() =>
  handleViews.value.some(h => outsideTaught(h.value)))
</script>

<template>
  <div class="space-y-2">
    <div class="flex items-baseline justify-between">
      <span class="text-xs text-fg-muted">Travel</span>
      <span class="text-xs font-mono text-fg-faint">
        {{ position ?? '—' }} units
      </span>
    </div>

    <!-- Track. Extra height leaves room for mark flags above and the
         taught band below without them colliding. -->
    <div
      ref="trackRef"
      class="relative h-[104px] select-none touch-none"
      @pointermove="onMove"
      @pointerup="onUp"
      @pointercancel="onUp"
    >
      <!-- Taught hardware travel, drawn as the backdrop the usable
           window sits inside. Read-only — the Kangaroo has no command to
           set it. -->
      <div v-if="taughtBand"
           class="absolute top-2 h-1.5 rounded-full bg-emerald-500/40"
           :style="{ left: `${taughtBand.left}%`, width: `${taughtBand.width}%` }" />
      <div v-if="taughtBand"
           class="absolute top-0 text-[9px] text-emerald-400 -translate-y-0.5"
           :style="{ left: `${taughtBand.left}%` }">
        taught travel (read-only)
      </div>

      <!-- Base rail + the usable window between the retract/extend handles -->
      <div class="absolute inset-x-0 top-[34px] h-2.5 rounded-full bg-slate-700/50 overflow-hidden">
        <div v-if="usableBand"
             class="absolute inset-y-0 bg-cyan-500/25"
             :style="{ left: `${usableBand.left}%`, width: `${usableBand.width}%` }" />
      </div>

      <!-- Live position (read-only) -->
      <div v-if="positionPct != null"
           class="absolute top-[28px] w-1 h-[22px] rounded-full bg-white shadow pointer-events-none z-10"
           :style="{ left: `calc(${positionPct}% - 2px)` }" />

      <!-- Draggable handles, staggered by row so coincident values stay
           individually grabbable. -->
      <div
        v-for="h in handleViews"
        :key="h.key"
        class="absolute"
        :style="{ left: `calc(${h.pct}% - 9px)`, top: `${h.top}px` }"
      >
        <!-- Leader line down to the rail so a stacked handle still reads
             as pointing at a position. -->
        <span v-if="h.row > 0"
              class="absolute left-1/2 -translate-x-1/2 w-px bg-current opacity-40"
              :style="{ backgroundColor: h.color, bottom: '18px',
                        height: `${h.top - 36}px` }" />
        <div
          class="w-[18px] h-[18px] rounded-full border-2 border-white shadow"
          :class="editable ? 'cursor-grab active:cursor-grabbing' : 'opacity-60 cursor-not-allowed'"
          :style="{ backgroundColor: h.color }"
          :title="editable ? `${h.label}: ${h.value} — drag to change`
                           : `${h.label}: ${h.value}`"
          @pointerdown="onDown(h, $event)"
        />
        <span class="absolute left-1/2 -translate-x-1/2 top-[19px] text-[9px] whitespace-nowrap"
              :style="{ color: h.color }">
          {{ h.label }}
        </span>
      </div>
    </div>

    <!-- Numeric entry for the draggable values — typing beats dragging
         when the operator already knows the number. -->
    <div v-if="handleViews.length" class="grid grid-cols-2 gap-2">
      <div v-for="h in handleViews" :key="h.key" class="flex items-center gap-2">
        <span class="inline-block w-3 h-3 rounded-full shrink-0"
              :style="{ backgroundColor: h.color }" />
        <label class="text-xs text-fg-muted w-16">{{ h.label }}</label>
        <input
          type="number" step="10"
          :value="h.value"
          :disabled="!editable"
          @input="(e) => onNumericInput(h, e)"
          class="input-field w-full text-sm"
        />
      </div>
    </div>

    <p v-if="anyOutside" class="text-[11px] text-amber-300 leading-snug">
      A handle sits outside the taught travel — the axis will stop at the
      hardware limit instead of reaching it.
    </p>
    <p v-if="!taughtBand" class="text-[11px] text-fg-faint leading-snug">
      No taught travel read back yet. These handles set the <em>usable</em>
      range; the hardware range comes from a teach tune and can't be typed
      in. Run a tune, power-cycle, then read it back to see the band you're
      working inside.
    </p>
  </div>
</template>
