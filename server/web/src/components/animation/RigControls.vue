<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'

// On-screen control rig. Renders the controls a rig file declares —
// sliders for 1D channels, XY pads for eye look — and turns operator
// input into joint values.
//
// The rig evaluates on the SERVER. We send control values and get joint
// values back, then hand those to the 3D viewer. Porting the evaluator
// to JS would duplicate the blend math, the clamp policy, and the mimic
// round-trip, and a local round-trip costs a millisecond that a slider
// drag doesn't notice. `<widget>` hints are the only part of the rig this
// component reads for itself — everything else is the server's contract.

const emit = defineEmits([
  // (joints) — resolved { jointName: normalized } for the 3D viewport.
  'joints',
  // No payload: the operator asked to key the current rig pose, and the
  // parent already holds the joint values from the last `joints` event.
  'key-frame',
  // (bool) — show/hide the 3D control shapes.
  'update:overlay',
  // (rig, anchors) — the parsed rig, so the parent can hand it to the
  // viewer to build shapes from.
  'rig-loaded',
  // (controlName|null) — hovering a panel row highlights its 3D shape,
  // which is how you find a control's handle on a busy model.
  'hover-control',
  // (controlName[]) — controls the evaluator can't handle. Their shapes
  // are drawn inert and made un-grabbable.
  'inert-controls',
])

const props = defineProps({
  // When true, evaluated frames are also pushed into the routing graph,
  // so the physical robot follows the controls.
  live: { type: Boolean, default: false },
  // Whether the 3D shapes are drawn in the viewport. Owned by the parent
  // (the viewer holds the actual meshes), mirrored here so the toggle can
  // live in this panel — the controls area is where an operator looks for
  // it, per the request.
  overlay: { type: Boolean, default: true },
})

const ws = useWsStore()

const rig = ref(null)
const warnings = ref([])
const values = ref({})           // control name (or "name.axis") → number
const contributions = ref({})    // control name → { joint: delta }
const clamped = ref(false)
const scaleApplied = ref(1)
const skipped = ref([])
const loading = ref(true)
const error = ref('')

const groups = computed(() => {
  if (!rig.value) return []
  // Groups in first-seen file order (authorable intent), controls by
  // `order` then label within each — matching Rig.controls_by_group().
  const out = []
  const seen = new Map()
  for (const c of rig.value.controls || []) {
    const key = c.group || ''
    if (!seen.has(key)) {
      const bucket = { group: key, controls: [] }
      seen.set(key, bucket)
      out.push(bucket)
    }
    seen.get(key).controls.push(c)
  }
  for (const g of out) {
    g.controls.sort((a, b) =>
      (a.order - b.order) || (a.label || a.name).localeCompare(b.label || b.name))
  }
  return out
})

const defaults = ref({})
const anchors = ref({})
const hasControls = computed(() => (rig.value?.controls?.length || 0) > 0)
const isDirty = computed(() =>
  Object.entries(values.value).some(([k, v]) => v !== (defaults.value[k] ?? 0)))

async function load () {
  loading.value = true
  error.value = ''
  try {
    const r = await ws.management('get_rig', {})
    rig.value = r?.rig || null
    warnings.value = r?.warnings || []
    defaults.value = r?.defaults || {}
    anchors.value = r?.anchors || {}
    values.value = { ...defaults.value }
    // The viewer needs the parsed rig to build shapes, and the anchors to
    // know which link each one hangs off.
    emit('rig-loaded', rig.value, anchors.value)
    if (rig.value) await evaluate()
  } catch (e) {
    error.value = e.message || String(e)
    rig.value = null
  } finally {
    loading.value = false
  }
}

// Rate limiting for slider drags, which fire an `input` event on every
// pointermove — 30-plus a second.
//
// Two mechanisms, and BOTH are needed:
//
//  * A ~30 Hz floor between requests, matching the animation editor's
//    Live Preview cadence.
//  * At most ONE request in flight. Without this, a drag turns into a
//    queue of round trips: they can complete out of order (so the rig
//    settles on a stale frame rather than where the slider actually is),
//    and against any backend slower than the input rate the queue grows
//    without bound until every request blows the client's 30 s timeout.
//    That is exactly what "Request timeout" on releasing a slider was.
//
// Intermediate values are simply dropped — correct for a slider, where
// only the current position matters. `queued` guarantees a trailing call
// with whatever the value is when the in-flight one lands, so the rig
// can never be left a frame behind the control.
const MIN_INTERVAL_MS = 33

let lastSent = 0
let timer = null
let inFlight = false
let queued = false

function scheduleEvaluate () {
  if (inFlight) { queued = true; return }
  const wait = MIN_INTERVAL_MS - (Date.now() - lastSent)
  clearTimeout(timer)
  if (wait <= 0) evaluate()
  else timer = setTimeout(evaluate, wait)
}

async function evaluate () {
  clearTimeout(timer)
  inFlight = true
  queued = false
  lastSent = Date.now()
  try {
    // Read values.value HERE, not at schedule time, so a coalesced burst
    // sends the latest position rather than the one that triggered it.
    const r = await ws.management('evaluate_rig', {
      values: values.value,
      apply: props.live,
    })
    if (r?.success === false) {
      error.value = r.message || 'Evaluation failed'
    } else {
      error.value = ''
      contributions.value = r?.contributions || {}
      clamped.value = !!r?.clamped
      scaleApplied.value = r?.scale_applied ?? 1
      const nextSkipped = r?.skipped || []
      // Only re-emit when the SET changes — the viewer rebuilds its
      // shapes on this, and doing that 30x a second during a drag would
      // be absurd.
      const nextKeys = nextSkipped.map(x => x.control).sort().join('|')
      if (nextKeys !== skippedKeys) {
        skippedKeys = nextKeys
        emit('inert-controls', nextSkipped.map(x => x.control))
      }
      skipped.value = nextSkipped
      emit('joints', r?.joints || {})
    }
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    inFlight = false
    if (queued) scheduleEvaluate()
  }
}

function setValue (key, v) {
  values.value = { ...values.value, [key]: Number(v) }
  scheduleEvaluate()
}

// ── driven from the 3D viewport ─────────────────────────────────────
//
// The viewer reports a normalized drag DELTA (fraction of a full sweep)
// and the value math happens here, so a shape drag and a slider drag end
// up in exactly the same place. `dragBase` snapshots the values at press
// so the whole drag is relative to where it started rather than
// accumulating rounding per frame.

let dragBase = null
// Last emitted inert-control key, so the viewer isn't told to rebuild on
// every evaluate.
let skippedKeys = ''

function controlByName (name) {
  return (rig.value?.controls || []).find(c => c.name === name) || null
}

function beginOverlayDrag (name) {
  dragBase = { ...values.value }
  selected.value = name
}

function applyOverlayDrag ({ name, delta, deltaX, deltaY }) {
  const c = controlByName(name)
  if (!c || !dragBase) return
  const next = { ...values.value }

  if (c.kind === 'pad') {
    for (const axis of c.axes || []) {
      const d = axis.name === 'x' ? deltaX : deltaY
      if (d === undefined) continue
      const key = `${c.name}.${axis.name}`
      const span = (axis.max ?? 1) - (axis.min ?? -1)
      const base = dragBase[key] ?? axis.default ?? 0
      next[key] = clampTo(base + d * span, axis.min ?? -1, axis.max ?? 1)
    }
  } else {
    if (delta === undefined) return
    const span = (c.max ?? 1) - (c.min ?? -1)
    const base = dragBase[c.name] ?? c.default ?? 0
    next[c.name] = clampTo(base + delta * span, c.min ?? -1, c.max ?? 1)
  }
  values.value = next
  scheduleEvaluate()
}

function endOverlayDrag () {
  dragBase = null
}

function clampTo (v, lo, hi) {
  return Math.max(lo, Math.min(hi, v))
}

// Which control the operator last touched, in either surface. Drives the
// panel's highlight so grabbing a shape in the viewport shows you which
// slider it is.
const selected = ref(null)

function resetAll () {
  values.value = { ...defaults.value }
  evaluate()
}
function resetControl (c) {
  const next = { ...values.value }
  if (c.kind === 'pad') {
    for (const a of c.axes || []) next[`${c.name}.${a.name}`] = a.default
  } else {
    next[c.name] = c.default
  }
  values.value = next
  evaluate()
}

// ── XY pad ─────────────────────────────────────────────────────────
//
// A pad is two scalars, addressed as "<control>.x" / "<control>.y".
// `invert_y` flips the vertical axis for screen-space feel: dragging up
// should raise the eyes, and screen y grows downward.

function padPos (c) {
  const ax = c.axes?.find(a => a.name === 'x')
  const ay = c.axes?.find(a => a.name === 'y')
  const vx = values.value[`${c.name}.x`] ?? ax?.default ?? 0
  const vy = values.value[`${c.name}.y`] ?? ay?.default ?? 0
  const nx = norm(vx, ax)
  let ny = norm(vy, ay)
  if (c.widget?.invert_y) ny = 1 - ny
  return { left: `${nx * 100}%`, top: `${ny * 100}%` }
}
function norm (v, axis) {
  const lo = axis?.min ?? -1
  const hi = axis?.max ?? 1
  if (hi <= lo) return 0.5
  return Math.min(1, Math.max(0, (v - lo) / (hi - lo)))
}
function denorm (n, axis) {
  const lo = axis?.min ?? -1
  const hi = axis?.max ?? 1
  return lo + Math.min(1, Math.max(0, n)) * (hi - lo)
}

const padDragging = ref(null)
function onPadPointerDown (c, e) {
  padDragging.value = c.name
  e.currentTarget.setPointerCapture?.(e.pointerId)
  onPadMove(c, e)
}
function onPadMove (c, e) {
  if (padDragging.value !== c.name) return
  const rect = e.currentTarget.getBoundingClientRect()
  if (!rect.width || !rect.height) return
  const nx = (e.clientX - rect.left) / rect.width
  let ny = (e.clientY - rect.top) / rect.height
  if (c.widget?.invert_y) ny = 1 - ny
  const ax = c.axes?.find(a => a.name === 'x')
  const ay = c.axes?.find(a => a.name === 'y')
  const next = { ...values.value }
  if (ax) next[`${c.name}.x`] = denorm(nx, ax)
  if (ay) next[`${c.name}.y`] = denorm(ny, ay)
  values.value = next
  scheduleEvaluate()
}
function onPadPointerUp () { padDragging.value = null }

// Keyboard access — a pad reachable only by pointer is unusable for
// anyone who can't use one, and arrow-key nudging is genuinely more
// precise than dragging for small offsets.
function onPadKey (c, e) {
  const step = e.shiftKey ? 0.2 : 0.05
  const deltas = {
    ArrowLeft: ['x', -step], ArrowRight: ['x', step],
    ArrowUp: ['y', c.widget?.invert_y ? step : -step],
    ArrowDown: ['y', c.widget?.invert_y ? -step : step],
  }
  const hit = deltas[e.key]
  if (!hit) return
  e.preventDefault()
  const [axisName, delta] = hit
  const axis = c.axes?.find(a => a.name === axisName)
  if (!axis) return
  const key = `${c.name}.${axisName}`
  const current = values.value[key] ?? axis.default ?? 0
  const clampedValue = Math.min(axis.max, Math.max(axis.min, current + delta))
  setValue(key, clampedValue)
}

// ── driven-joint readout ───────────────────────────────────────────

function jointsFor (name) {
  const map = contributions.value[name] || {}
  return Object.entries(map)
    .filter(([, v]) => Math.abs(v) > 1e-6)
    .sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]))
}
function fmt (v) {
  const n = Number(v)
  return Number.isFinite(n) ? n.toFixed(2) : '—'
}

watch(() => props.live, () => evaluate())
onMounted(load)

defineExpose({
  reload: load, resetAll, values,
  beginOverlayDrag, applyOverlayDrag, endOverlayDrag,
  setSelected: (name) => { selected.value = name },
})
</script>

<template>
  <div class="space-y-3">
    <div class="flex items-center justify-between gap-2">
      <h4 class="text-xs font-semibold text-fg-strong uppercase tracking-wide">
        Control Rig
      </h4>
      <div class="flex items-center gap-1">
        <!-- Show/hide the 3D shapes. Lives here because the controls area
             is where an operator looks for it, even though the viewer owns
             the meshes. -->
        <button v-if="hasControls"
                :class="['btn-sm text-fg-strong',
                         overlay ? 'bg-cyan-600 hover:bg-cyan-500'
                                 : 'bg-surface hover:bg-surface-2']"
                :title="overlay
                  ? 'Hide the control rig in the 3D view'
                  : 'Show the control rig in the 3D view'"
                @click="emit('update:overlay', !overlay)">
          <span class="material-icons icon-sm">
            {{ overlay ? 'gamepad' : 'radio_button_unchecked' }}
          </span>
        </button>
        <button v-if="hasControls"
                class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                title="Key the joints these controls are driving, at the playhead"
                @click="emit('key-frame')">
          <span class="material-icons icon-sm">vpn_key</span>
        </button>
        <button v-if="hasControls"
                class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                :disabled="!isDirty"
                title="Return every control to its default"
                @click="resetAll">
          <span class="material-icons icon-sm">restart_alt</span>
        </button>
      </div>
    </div>

    <p v-if="loading" class="text-xs text-fg-faint">Loading rig…</p>

    <div v-else-if="!rig" class="text-xs text-fg-muted space-y-1">
      <p>No rig file installed.</p>
      <p class="text-fg-faint">
        A rig file declares on-screen controls — a slider that blends
        poses, an XY pad for eye look, a nod that drives several joints at
        different weights. Upload one in
        <RouterLink to="/settings" class="text-cyan-400 hover:text-cyan-300 underline">
          Settings → Robot Model</RouterLink>.
      </p>
    </div>

    <template v-else>
      <div v-if="error"
           class="rounded border border-red-500/40 bg-red-500/10 p-2 text-xs text-red-300">
        {{ error }}
      </div>

      <!-- Unresolved references. A control naming a renamed joint just -->
      <!-- silently does nothing, so it has to be said out loud.        -->
      <details v-if="warnings.length"
               class="rounded border border-amber-500/40 bg-amber-500/10 p-2">
        <summary class="text-xs text-amber-200 cursor-pointer">
          {{ warnings.length }} unresolved reference{{ warnings.length === 1 ? '' : 's' }}
        </summary>
        <ul class="mt-1 space-y-0.5 text-[11px] text-amber-100/90 font-mono">
          <li v-for="(w, i) in warnings" :key="i">{{ w }}</li>
        </ul>
      </details>

      <!-- Controls the evaluator can't handle (gaze needs the solver). -->
      <details v-if="skipped.length"
               class="rounded border border-line/50 bg-panel/30 p-2">
        <summary class="text-xs text-fg-muted cursor-pointer">
          {{ skipped.length }} control{{ skipped.length === 1 ? '' : 's' }} not evaluated
        </summary>
        <ul class="mt-1 space-y-0.5 text-[11px] text-fg-faint">
          <li v-for="(s, i) in skipped" :key="i">
            <span class="font-mono text-fg-muted">{{ s.control }}</span> — {{ s.reason }}
          </li>
        </ul>
      </details>

      <!-- The clamp policy pulled the frame back. Worth surfacing: -->
      <!-- otherwise a slider just stops having an effect.          -->
      <div v-if="clamped"
           class="flex items-center gap-1 rounded border border-amber-500/40 bg-amber-500/10 px-2 py-1 text-[11px] text-amber-200"
           :title="`Joint limits reached. Policy '${rig.settings.clamp}' — `
             + (rig.settings.clamp === 'scale_back'
                 ? `the whole frame scaled back to ${Math.round(scaleApplied * 100)}% so the pose keeps its shape.`
                 : 'each joint stopped independently at its limit.')">
        <span class="material-icons" style="font-size:13px">compress</span>
        At limit
        <span v-if="rig.settings.clamp === 'scale_back'" class="tabular-nums">
          · {{ Math.round(scaleApplied * 100) }}%
        </span>
      </div>

      <div v-for="g in groups" :key="g.group || '_ungrouped'" class="space-y-2">
        <div v-if="g.group"
             class="text-[10px] uppercase tracking-wide text-fg-faint border-b border-line/40 pb-1">
          {{ g.group }}
        </div>

        <div v-for="c in g.controls" :key="c.name"
             :class="['space-y-1 rounded px-1 -mx-1 transition-colors',
                      selected === c.name ? 'bg-cyan-500/10' : '']"
             @pointerenter="emit('hover-control', c.name)"
             @pointerleave="emit('hover-control', null)">

          <!-- 1D channel → slider. A scalar with no spatial meaning must
               never get a 3D gizmo; that's strictly worse than a slider. -->
          <template v-if="c.kind === 'channel'">
            <div class="flex items-baseline justify-between gap-2">
              <label class="text-xs text-fg-strong truncate" :for="`rig-${c.name}`">
                {{ c.label || c.name }}
              </label>
              <button class="text-[11px] text-fg-muted hover:text-fg-strong tabular-nums"
                      title="Reset to default"
                      @click="resetControl(c)">
                {{ fmt(values[c.name] ?? c.default) }}
              </button>
            </div>
            <input :id="`rig-${c.name}`"
                   type="range" class="w-full"
                   :min="c.min" :max="c.max" step="0.01"
                   :value="values[c.name] ?? c.default"
                   @input="setValue(c.name, $event.target.value)" />
            <!-- Pose targets, so a ±1 slider says what each end does. -->
            <div v-if="c.targets?.length"
                 class="flex justify-between text-[10px] text-fg-faint">
              <span>{{ c.targets.find(t => t.at <= c.default)?.pose || '' }}</span>
              <span>{{ c.targets.find(t => t.at > c.default)?.pose || '' }}</span>
            </div>
          </template>

          <!-- 2D pad → eye look. Spatial in feel, so it gets a spatial
               control, but still no 3D gizmo: it drives two scalars. -->
          <template v-else-if="c.kind === 'pad'">
            <div class="flex items-baseline justify-between gap-2">
              <span class="text-xs text-fg-strong truncate">{{ c.label || c.name }}</span>
              <button class="text-[11px] text-fg-muted hover:text-fg-strong tabular-nums"
                      title="Reset to default"
                      @click="resetControl(c)">
                {{ fmt(values[`${c.name}.x`] ?? 0) }},
                {{ fmt(values[`${c.name}.y`] ?? 0) }}
              </button>
            </div>
            <div class="rig-pad"
                 tabindex="0"
                 role="application"
                 :aria-label="`${c.label || c.name} — arrow keys to adjust`"
                 @pointerdown="onPadPointerDown(c, $event)"
                 @pointermove="onPadMove(c, $event)"
                 @pointerup="onPadPointerUp"
                 @pointercancel="onPadPointerUp"
                 @keydown="onPadKey(c, $event)">
              <div class="rig-pad-cross-h"></div>
              <div class="rig-pad-cross-v"></div>
              <div class="rig-pad-knob" :style="padPos(c)"></div>
            </div>
          </template>

          <!-- Spatial → declared, but needs the IK solver. Say so rather
               than rendering a gizmo that does nothing. -->
          <template v-else-if="c.kind === 'spatial'">
            <div class="rounded border border-line/40 bg-panel/30 p-2">
              <div class="flex items-center gap-1.5 text-xs text-fg-strong">
                <span class="material-icons icon-sm text-fg-faint">my_location</span>
                {{ c.label || c.name }}
              </div>
              <p class="text-[11px] text-fg-faint mt-1">
                Look-at target in
                <code>{{ c.anchor || '—' }}</code>. Declared in the rig file;
                the gaze solve isn't wired up yet, so this control has no
                effect. Use a pad control for pan/tilt eye look meanwhile.
              </p>
            </div>
          </template>

          <!-- Which joints this control is actually moving right now. -->
          <div v-if="jointsFor(c.name).length"
               class="flex flex-wrap gap-1 pt-0.5">
            <span v-for="[joint, delta] in jointsFor(c.name)" :key="joint"
                  class="rounded bg-surface/60 border border-line/40 px-1.5 py-0.5 text-[10px] font-mono"
                  :title="`${joint} ${delta > 0 ? '+' : ''}${fmt(delta)} from this control`">
              {{ joint }}
              <span class="text-fg-faint tabular-nums">{{ delta > 0 ? '+' : '' }}{{ fmt(delta) }}</span>
            </span>
          </div>
        </div>
      </div>
    </template>
  </div>
</template>

<style scoped>
.rig-pad {
  position: relative;
  width: 100%;
  aspect-ratio: 1 / 1;
  max-height: 140px;
  border: 1px solid var(--color-line);
  border-radius: 6px;
  background: var(--color-canvas);
  cursor: crosshair;
  touch-action: none;   /* the pad owns the gesture, not the scroller */
}
.rig-pad:focus-visible { outline: 2px solid #22d3ee; outline-offset: 1px; }

.rig-pad-cross-h, .rig-pad-cross-v {
  position: absolute;
  background: var(--color-line);
  pointer-events: none;
}
.rig-pad-cross-h { left: 0; right: 0; top: 50%; height: 1px; }
.rig-pad-cross-v { top: 0; bottom: 0; left: 50%; width: 1px; }

.rig-pad-knob {
  position: absolute;
  width: 12px; height: 12px;
  margin: -6px 0 0 -6px;
  border-radius: 50%;
  background: #22d3ee;
  box-shadow: 0 0 0 3px rgba(34, 211, 238, 0.2);
  pointer-events: none;
}
</style>
