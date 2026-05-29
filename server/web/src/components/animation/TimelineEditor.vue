<script setup>
import { computed, inject, ref } from 'vue'
import { INTERP_LINEAR, INTERP_LABELS } from '@/composables/useCurveSampling'
import {
  EASING_CATEGORIES, easingByCode, easingGlyph as easingGlyphFor,
  segmentPath,
} from '@/composables/easings'
import { useAnimationsStore } from '@/stores/animations'

const animations = useAnimationsStore()

// Dual-mode timeline:
//   • Dope sheet — compact rows, keyframes at row-mid, curve drawn
//     faint behind. Best for high-density retiming.
//   • Curve editor — tall rows, keyframes at (time, value), drag both
//     axes; SVG paths show the real interpolated curves segment by
//     segment, respecting each easing's CSS-spec control points.

const props = defineProps({
  animation:     { type: Object, required: true },
  playerPos:     { type: Number, default: 0 },
  selection:     { type: Object, default: () => ({ kind: null }) },
  // Transport state: when `true`, the server-side player is running
  // this animation. Drives the play / stop button rendered in our
  // own top bar (no separate transport above the timeline).
  playing:       { type: Boolean, default: false },
  // Joint names from the URDF that aren't yet bound to a value track
  // — surfaced in the + Joint dropdown in the bottom bar.
  unboundJoints: { type: Array, default: () => [] },
})

const emit = defineEmits([
  'update:player-pos', 'select', 'dirty',
  'rename-track', 'remove-track',
  'play', 'stop', 'add-joint',
  'rename-trigger-track', 'remove-trigger-track',
])

// Provided by AnimationEditorView so the bottom-bar "+ Trigger"
// button and the trigger lane's click-to-add can mutate the animation
// without owning store state here.
const addTriggerTrack       = inject('add-trigger-track',       () => null)
const addTriggerKeyframeAt  = inject('add-trigger-keyframe-at', () => null)

// Display mode. The internal value stays 'dope' for backwards compat
// in the keyframe-pos computation; the button label reads "Keys" now.
const mode = ref('dope')
const rowSize = computed(() => mode.value === 'curve' ? 132 : 40)

// Joint dropdown (bottom bar) — opens upward so it doesn't overflow
// the viewport.
const showJointMenu = ref(false)
function pickJoint (name) {
  showJointMenu.value = false
  emit('add-joint', name)
}

// Horizontal zoom + scroll. The whole timeline (top scrub bar + every
// track row) shares one scroll container. The label columns stay
// pinned to the left via `position: sticky`. Zoom scales the inner
// content width — at 1.0 it fits the container; at 4.0 it's four
// times as wide and scrolls horizontally.
const zoom = ref(1)
const ZOOM_MIN = 1
const ZOOM_MAX = 16
const zoomPct = computed(() => Math.round(zoom.value * 100))
function zoomToFit () { zoom.value = 1 }
function bumpZoom (dir) {
  zoom.value = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, zoom.value * (dir > 0 ? 1.5 : 1 / 1.5)))
}

const dragging = ref(null)
const scrubbing = ref(false)
// Suppress the click that follows mouseup after a keyframe drag —
// otherwise the row's click handler fires and drops a new keyframe.
let suppressNextRowClick = false
// Easing dropdown state. `openEasingMenu` is the segment key (or null);
// `openEasingMenuAnchor` is the screen-space (x, y) the popup positions
// itself against. The popup itself is Teleport'd to <body> so it
// escapes the row's overflow: hidden clipping.
const openEasingMenu = ref(null)
const openEasingMenuAnchor = ref({ x: 0, y: 0 })
const openEasingMenuTrack = ref(null)        // { track, fromIdx }

const duration = computed(() => Math.max(0.0001, Number(props.animation?.duration || 0)))

const TRACK_COLORS = [
  '#38bdf8', '#fbbf24', '#a78bfa', '#34d399',
  '#fb7185', '#22d3ee', '#f472b6', '#facc15',
  '#60a5fa', '#fb923c',
]
function trackColor (idx) { return TRACK_COLORS[idx % TRACK_COLORS.length] }

const tickTimes = computed(() => {
  const d = duration.value
  let step
  if (d <= 2)       step = 0.25
  else if (d <= 5)  step = 0.5
  else if (d <= 15) step = 1
  else if (d <= 45) step = 5
  else              step = 10
  const out = []
  for (let t = 0; t <= d + 1e-6; t += step) out.push(Number(t.toFixed(3)))
  return out
})

function timeToPct (t) { return Math.max(0, Math.min(100, (t / duration.value) * 100)) }

function pctFromMouseX (e, el) {
  const rect = el.getBoundingClientRect()
  return Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width))
}
function pctFromMouseY (e, el) {
  const rect = el.getBoundingClientRect()
  return Math.max(0, Math.min(1, (e.clientY - rect.top) / rect.height))
}

function trackRange (track) {
  const ys = (track.curve?.keys || []).map(k => Number(k.value) || 0)
  if (!ys.length) return { min: -1, max: 1 }
  let lo = Math.min(...ys), hi = Math.max(...ys)
  if (lo === hi) {
    const pad = Math.max(1, Math.abs(lo) * 0.2)
    return { min: lo - pad, max: hi + pad }
  }
  const pad = (hi - lo) * 0.15
  return { min: lo - pad, max: hi + pad }
}

function isKeyframeSelected (trackId, idx) {
  const s = props.selection
  return s?.kind === 'keyframe' && s.trackId === trackId && s.kfIdx === idx
}

// ── Curve path generation ───────────────────────────────────────────

function curvePath (track) {
  const keys = track.curve?.keys || []
  if (!keys.length) return ''
  const { min, max } = trackRange(track)
  const span = max - min || 1
  const xPx = (t) => t / duration.value
  const yPx = (v) => 1 - (v - min) / span

  if (keys.length === 1) {
    const y = yPx(keys[0].value)
    return `M 0 ${y} L 1 ${y}`
  }

  let d = `M ${xPx(keys[0].time)} ${yPx(keys[0].value)}`
  for (let i = 1; i < keys.length; i++) {
    const k0 = keys[i - 1], k1 = keys[i]
    const code = k0.interp ?? INTERP_LINEAR
    d += segmentPath(
      code,
      xPx(k0.time), yPx(k0.value),
      xPx(k1.time), yPx(k1.value),
      // Hermite tangents passed in unit space (dt = normalized
      // segment width); only used when the easing has `hermite: true`.
      {
        leave: k0.leave_tangent || 0,
        arrive: k1.arrive_tangent || 0,
        dt: (k1.time - k0.time) / duration.value,
      },
    )
  }
  return d
}

function keyframePctPos (track, kf) {
  const x = (kf.time / duration.value) * 100
  if (mode.value === 'dope') return { x, y: 50 }
  const { min, max } = trackRange(track)
  const span = max - min || 1
  const y = 100 - ((kf.value - min) / span) * 100
  return { x, y }
}

// ── Click-to-add ────────────────────────────────────────────────────

function onRowClick (track, e) {
  // Suppress synthetic clicks that follow keyframe drags.
  if (suppressNextRowClick) {
    suppressNextRowClick = false
    return
  }
  if (e.target?.dataset?.kfIdx !== undefined) return
  const row = e.currentTarget
  const ratioX = pctFromMouseX(e, row)
  const t = ratioX * duration.value
  let value = 0
  if (mode.value === 'curve') {
    const ratioY = pctFromMouseY(e, row)
    const { min, max } = trackRange(track)
    value = min + (1 - ratioY) * (max - min)
  } else {
    value = interpolateAt(track.curve.keys, t)
  }
  animations.snapshot({ force: true })
  const insertAt = track.curve.keys.findIndex(k => k.time > t)
  const key = {
    time: t, value: Number(value.toFixed(3)),
    interp: INTERP_LINEAR, arrive_tangent: 0, leave_tangent: 0,
  }
  if (insertAt === -1) track.curve.keys.push(key)
  else track.curve.keys.splice(insertAt, 0, key)
  emit('dirty')
  const newIdx = insertAt === -1 ? track.curve.keys.length - 1 : insertAt
  emit('select', { kind: 'keyframe', trackId: track.id, kfIdx: newIdx })
}

function interpolateAt (keys, t) {
  if (!keys.length) return 0
  if (t <= keys[0].time) return keys[0].value
  if (t >= keys[keys.length - 1].time) return keys[keys.length - 1].value
  for (let i = 0; i < keys.length - 1; i++) {
    if (keys[i].time <= t && t <= keys[i + 1].time) {
      const dt = keys[i + 1].time - keys[i].time
      const r = dt === 0 ? 0 : (t - keys[i].time) / dt
      return keys[i].value + r * (keys[i + 1].value - keys[i].value)
    }
  }
  return keys[keys.length - 1].value
}

// ── Drag-to-move keys ───────────────────────────────────────────────

function onKeyframeMouseDown (track, idx, e) {
  e.preventDefault()
  e.stopPropagation()
  // Lock body text-selection for the duration of the drag — see
  // setBodySelectNone for the rationale.
  setBodySelectNone(true)
  emit('select', { kind: 'keyframe', trackId: track.id, kfIdx: idx })
  // Snapshot once at drag start. The drag emits many incremental
  // updates but they all coalesce into this one undo step.
  animations.snapshot({ force: true })
  suppressNextRowClick = true
  dragging.value = {
    track, trackId: track.id, kfIdx: idx,
    row: e.currentTarget.parentElement,
    range: trackRange(track),
  }
  window.addEventListener('mousemove', onKeyframeMove)
  window.addEventListener('mouseup', onKeyframeUp)
}

function onKeyframeMove (e) {
  const d = dragging.value
  if (!d) return
  const ratioX = pctFromMouseX(e, d.row)
  const newTime = ratioX * duration.value
  d.track.curve.keys[d.kfIdx].time = newTime
  if (mode.value === 'curve') {
    const ratioY = pctFromMouseY(e, d.row)
    const span = d.range.max - d.range.min || 1
    d.track.curve.keys[d.kfIdx].value = d.range.min + (1 - ratioY) * span
  }
  const moved = d.track.curve.keys[d.kfIdx]
  d.track.curve.keys.sort((a, b) => a.time - b.time)
  const newIdx = d.track.curve.keys.indexOf(moved)
  if (newIdx !== d.kfIdx) {
    d.kfIdx = newIdx
    emit('select', { kind: 'keyframe', trackId: d.trackId, kfIdx: newIdx })
  }
}

function onKeyframeUp () {
  if (dragging.value) emit('dirty')
  dragging.value = null
  setBodySelectNone(false)
  window.removeEventListener('mousemove', onKeyframeMove)
  window.removeEventListener('mouseup', onKeyframeUp)
  // The browser will fire a synthetic click on the row after this
  // mouseup; the row click handler reads `suppressNextRowClick` and
  // bails. Reset on the next tick so a normal subsequent click works.
  setTimeout(() => { suppressNextRowClick = false }, 0)
}

// ── Scrubber ────────────────────────────────────────────────────────

function onScrubStart (e) {
  // Suppress the browser's text-selection-on-drag globally for the
  // duration of the scrub — without this, dragging the scrubber past
  // the timeline edges starts selecting labels / props-panel text on
  // adjacent components. `preventDefault` here keeps the drag from
  // entering text-selection mode in the first place.
  e.preventDefault()
  setBodySelectNone(true)
  scrubbing.value = true
  emitScrub(e, e.currentTarget)
  window.addEventListener('mousemove', onScrubMove)
  window.addEventListener('mouseup', onScrubEnd)
}
function onScrubMove (e) {
  if (!scrubbing.value) return
  const el = document.querySelector('.timeline-scrub-strip')
  if (el) emitScrub(e, el)
}
function onScrubEnd () {
  scrubbing.value = false
  setBodySelectNone(false)
  window.removeEventListener('mousemove', onScrubMove)
  window.removeEventListener('mouseup', onScrubEnd)
}

// Toggle a body-level user-select lock for the duration of a drag.
// Cheaper than applying `user-select: none` to every individual lane
// and works regardless of where the cursor strays while dragging.
function setBodySelectNone (on) {
  document.body.style.userSelect = on ? 'none' : ''
  document.body.style.webkitUserSelect = on ? 'none' : ''
}
function emitScrub (e, el) {
  emit('update:player-pos', pctFromMouseX(e, el) * duration.value)
}

function selectTrack (track) {
  emit('select', { kind: 'track', trackId: track.id })
}

function onRemoveTrack (idx) {
  animations.snapshot({ force: true })
  emit('remove-track', idx)
}

// ── Per-segment easing picker ───────────────────────────────────────
//
// One chip between every adjacent keyframe pair. Click opens a
// dropdown showing all easing options with mini SVG previews; pick
// one to apply it to the segment.

// Menu groups for the easing picker — mirrors the categories list
// from the catalog so the menu order matches what an operator
// migrating from easings.net would expect.
const EASING_CATS = EASING_CATEGORIES

function segments (track) {
  const out = []
  const keys = track.curve?.keys || []
  for (let i = 0; i < keys.length - 1; i++) {
    const k0 = keys[i], k1 = keys[i + 1]
    const pos0 = keyframePctPos(track, k0)
    const pos1 = keyframePctPos(track, k1)
    out.push({
      fromIdx: i,
      interp: k0.interp ?? INTERP_LINEAR,
      x: (pos0.x + pos1.x) / 2,
      y: (pos0.y + pos1.y) / 2,
      key: `${track.id}:${i}`,
    })
  }
  return out
}

function setSegmentEasing (track, idx, easing) {
  animations.snapshot({ force: true })
  track.curve.keys[idx].interp = easing
  openEasingMenu.value = null
  emit('dirty')
}

function toggleEasingMenu (track, seg, e) {
  e.stopPropagation()
  if (openEasingMenu.value === seg.key) {
    openEasingMenu.value = null
    openEasingMenuTrack.value = null
    return
  }
  const rect = e.currentTarget.getBoundingClientRect()
  // Anchor the menu wherever it fits. Prefer below the chip; flip to
  // above when the menu would overflow the viewport bottom. Final
  // fallback clamps to viewport edges so it always sits onscreen.
  const MENU_ESTIMATED_H = 460          // taller than any actual menu
  const MARGIN = 8
  const vh = window.innerHeight
  const spaceBelow = vh - rect.bottom - MARGIN
  const spaceAbove = rect.top - MARGIN
  let y
  let dir
  if (spaceBelow >= MENU_ESTIMATED_H || spaceBelow >= spaceAbove) {
    y = rect.bottom + 4
    dir = 'below'
  } else {
    y = rect.top - 4
    dir = 'above'
  }
  openEasingMenu.value = seg.key
  openEasingMenuTrack.value = { track, fromIdx: seg.fromIdx }
  openEasingMenuAnchor.value = {
    x: rect.left + rect.width / 2,
    y,
    dir,
    maxH: Math.max(180, Math.min(MENU_ESTIMATED_H, dir === 'below' ? spaceBelow : spaceAbove)),
  }
}

function applyEasingFromMenu (interp) {
  const t = openEasingMenuTrack.value
  if (t) setSegmentEasing(t.track, t.fromIdx, interp)
  openEasingMenuTrack.value = null
}

function onTrackNameInput (track, e) {
  animations.snapshot()
  emit('rename-track', track, e.target.value)
}

// ── Trigger-track helpers ───────────────────────────────────────────

const triggerTracks = computed(() => props.animation?.trigger_tracks || [])

function onAddTriggerTrack () {
  addTriggerTrack()
}
function onTriggerTrackNameInput (track, e) {
  animations.snapshot()
  emit('rename-trigger-track', track, e.target.value)
}
function onRemoveTriggerTrack (track) {
  animations.snapshot({ force: true })
  emit('remove-trigger-track', track.id)
}
function onTriggerLaneClick (track, e) {
  if (suppressNextRowClick) {
    suppressNextRowClick = false
    return
  }
  const row = e.currentTarget
  const t = pctFromMouseX(e, row) * duration.value
  addTriggerKeyframeAt(track.id, t)
  // Auto-select the new keyframe (it's the one whose time matches t,
  // inserted at the right sorted slot). Find by time.
  const idx = (track.keyframes || []).findIndex(k => Math.abs(k.time - t) < 0.001)
  if (idx >= 0) emit('select', { kind: 'trigger-keyframe', trackId: track.id, kfIdx: idx })
  emit('dirty')
}
function onTriggerKeyframeMouseDown (track, idx, e) {
  e.preventDefault()
  e.stopPropagation()
  setBodySelectNone(true)
  emit('select', { kind: 'trigger-keyframe', trackId: track.id, kfIdx: idx })
  animations.snapshot({ force: true })
  suppressNextRowClick = true
  triggerDragging.value = { track, trackId: track.id, kfIdx: idx, row: e.currentTarget.parentElement }
  window.addEventListener('mousemove', onTriggerKeyframeMove)
  window.addEventListener('mouseup', onTriggerKeyframeUp)
}
const triggerDragging = ref(null)
function onTriggerKeyframeMove (e) {
  const d = triggerDragging.value
  if (!d) return
  const ratioX = pctFromMouseX(e, d.row)
  const newTime = Math.max(0, ratioX * duration.value)
  d.track.keyframes[d.kfIdx].time = newTime
  const moved = d.track.keyframes[d.kfIdx]
  d.track.keyframes.sort((a, b) => a.time - b.time)
  const newIdx = d.track.keyframes.indexOf(moved)
  if (newIdx !== d.kfIdx) {
    d.kfIdx = newIdx
    emit('select', { kind: 'trigger-keyframe', trackId: d.trackId, kfIdx: newIdx })
  }
}
function onTriggerKeyframeUp () {
  if (triggerDragging.value) emit('dirty')
  triggerDragging.value = null
  setBodySelectNone(false)
  window.removeEventListener('mousemove', onTriggerKeyframeMove)
  window.removeEventListener('mouseup', onTriggerKeyframeUp)
  setTimeout(() => { suppressNextRowClick = false }, 0)
}
function isTriggerKeyframeSelected (trackId, idx) {
  const s = props.selection
  return s?.kind === 'trigger-keyframe' && s.trackId === trackId && s.kfIdx === idx
}

function easingGlyph (code) { return easingGlyphFor(code) }
function interpName (code) { return easingByCode(code)?.label || INTERP_LABELS[code] || 'Linear' }

// Close any open easing menu when clicking elsewhere.
function onDocClick () { openEasingMenu.value = null }
import { onBeforeUnmount, onMounted } from 'vue'
onMounted(() => document.addEventListener('click', onDocClick))
onBeforeUnmount(() => document.removeEventListener('click', onDocClick))
</script>

<template>
  <div class="timeline-editor flex flex-col h-full select-none">
    <!-- One scroll container for the top bar AND the tracks. Sticky
         left positioning on the label columns keeps them pinned while
         the rest scrolls horizontally; sticky top on the top bar
         keeps it visible during vertical scroll. -->
    <div class="flex-1 min-h-0 overflow-x-auto overflow-y-auto bg-canvas timeline-scroll">
      <div class="timeline-content"
           :style="{ width: zoom * 100 + '%', minWidth: '100%' }">

        <!-- Top bar: label column + transport + scrub ruler -->
        <div class="flex items-stretch border-b border-line/50 bg-canvas sticky top-0 z-30">
          <div class="w-56 shrink-0 flex items-center gap-2 px-3 py-1.5 border-r border-line/50 sticky left-0 bg-canvas z-10">
            <button v-if="!playing"
                    class="btn-sm bg-emerald-500/80 hover:bg-emerald-500 text-fg-strong shrink-0"
                    title="Play" @click="emit('play')">
              <span class="material-icons icon-sm">play_arrow</span>
            </button>
            <button v-else
                    class="btn-sm bg-red-500/80 hover:bg-red-500 text-fg-strong shrink-0"
                    title="Stop" @click="emit('stop')">
              <span class="material-icons icon-sm">stop</span>
            </button>
            <span class="ml-auto text-xs text-fg tabular-nums shrink-0">
              {{ Number(playerPos).toFixed(2) }} / {{ Number(animation.duration || 0).toFixed(2) }}s
            </span>
          </div>
          <div class="timeline-scrub-strip relative flex-1 h-8 cursor-ew-resize"
               @mousedown="onScrubStart">
            <div v-for="t in tickTimes" :key="t"
                 class="absolute top-0 bottom-0 border-l border-line/40 pl-1"
                 :style="{ left: timeToPct(t) + '%' }">
              <span class="text-[10px] text-fg-faint tabular-nums">{{ t.toFixed(t < 1 ? 2 : 1) }}s</span>
            </div>
            <div class="absolute top-0 bottom-0 w-px bg-cyan-400 pointer-events-none"
                 :style="{ left: timeToPct(playerPos) + '%' }">
              <div class="absolute -top-1 -translate-x-1/2 w-2 h-2 rotate-45 bg-cyan-400"></div>
            </div>
          </div>
        </div>

        <!-- Tracks -->
        <div v-if="!animation?.value_tracks?.length" class="p-6 text-center text-sm text-fg-faint">
          No value tracks yet. Click <span class="text-cyan-300">+ Joint</span> in the bar below to bind one to a URDF joint.
        </div>

        <div v-for="(track, ti) in animation.value_tracks" :key="track.id"
             class="flex items-stretch border-b border-line-subtle/60 group"
             :style="{ height: rowSize + 'px' }">

          <!-- Sticky label column — stays parked at the left edge
               while the lane scrolls horizontally. -->
          <div class="w-56 shrink-0 flex items-center gap-2 px-2 border-r border-line/40 bg-canvas/60 sticky left-0 z-10">
          <span class="w-2.5 h-2.5 rounded-sm shrink-0 cursor-pointer"
                :style="{ backgroundColor: trackColor(ti) }"
                title="Select track"
                @click.stop="selectTrack(track)"></span>
          <input class="input-field flex-1 text-xs py-1 min-w-0"
                 :value="track.name" :title="track.id"
                 @input="onTrackNameInput(track, $event)" />
          <button class="btn-sm bg-panel hover:bg-red-600 text-fg-muted hover:text-fg-strong opacity-0 group-hover:opacity-100 transition-opacity"
                  :title="`Remove ${track.name || track.id}`"
                  @click="onRemoveTrack(ti)">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </div>

        <!-- Row body: optional chip strip (curve mode) above the curve
             lane. Splitting them into separate elements means the
             chip's hit area can never overlap with a keyframe's. -->
        <div class="timeline-row relative flex-1 flex flex-col overflow-hidden">

          <!-- Chip strip (curve mode only) -->
          <div v-if="mode === 'curve'"
               class="relative shrink-0 h-7 border-b border-line-subtle/60 bg-canvas/40">
            <button v-for="seg in segments(track)" :key="seg.key"
                    class="seg-interp"
                    :style="{ left: seg.x + '%' }"
                    :title="`Easing: ${interpName(seg.interp)} — click to change`"
                    @click="toggleEasingMenu(track, seg, $event)"
                    @mousedown.stop>
              <span v-html="easingGlyph(seg.interp)"></span>
            </button>
          </div>

          <!-- Curve lane — keyframes + SVG path + click-to-add live
               here. onRowClick reads e.currentTarget so percentages
               are computed relative to this lane only. -->
          <div class="relative flex-1 cursor-crosshair"
               @click="onRowClick(track, $event)">

          <div v-for="t in tickTimes" :key="t"
               class="absolute top-0 bottom-0 border-l border-line-subtle/60 pointer-events-none"
               :style="{ left: timeToPct(t) + '%' }"></div>

          <template v-if="mode === 'curve'">
            <div class="absolute left-0 right-0 border-t border-line/40 pointer-events-none"
                 :style="{
                   top: (() => {
                     const r = trackRange(track)
                     const span = r.max - r.min || 1
                     return (100 - ((0 - r.min) / span) * 100) + '%'
                   })()
                 }"></div>
            <span class="absolute left-1 top-1 text-[10px] text-fg-faint tabular-nums pointer-events-none">
              {{ trackRange(track).max.toFixed(2) }}
            </span>
            <span class="absolute left-1 bottom-1 text-[10px] text-fg-faint tabular-nums pointer-events-none">
              {{ trackRange(track).min.toFixed(2) }}
            </span>
          </template>

          <!-- SVG curve — only drawn in curve mode. Dope mode is
               pure keyframe markers (no easing preview between them);
               curve previews would clutter a dense dope-sheet view.
               z-index: 1 puts it above the playhead, below keyframes.
               `transform: translateZ(0)` + `will-change: transform`
               promote the SVG to its own GPU compositor layer so it
               isn't re-rasterized when the playhead's `left` style
               changes during scrub. Without this hint the browser
               keeps the curve in the same paint layer as the playhead,
               and the THREE.js scene re-render triggered on every
               scrub event starves the SVG of paint cycles — leaving
               the curve looking erased in the playhead's wake. -->
          <svg v-if="mode === 'curve'"
               class="absolute inset-0 w-full h-full pointer-events-none"
               style="z-index: 1; transform: translateZ(0); will-change: transform;"
               preserveAspectRatio="none" viewBox="0 0 1 1">
            <path :d="curvePath(track)"
                  :stroke="trackColor(ti)"
                  stroke-width="2"
                  fill="none"
                  stroke-linejoin="round"
                  stroke-linecap="round"
                  vector-effect="non-scaling-stroke" />
          </svg>

          <!-- Keyframes -->
          <div v-for="(kf, ki) in track.curve.keys" :key="ki"
               :data-kf-idx="ki"
               :class="['keyframe', isKeyframeSelected(track.id, ki) ? 'is-selected' : '']"
               :style="{
                 left: keyframePctPos(track, kf).x + '%',
                 top:  keyframePctPos(track, kf).y + '%',
                 borderColor: trackColor(ti),
               }"
               :title="`t=${kf.time.toFixed(2)}  ·  v=${Number(kf.value).toFixed(2)}`"
               @mousedown.stop="onKeyframeMouseDown(track, ki, $event)"
               @click.stop></div>

          <!-- Track-level playhead. Sits BELOW the curve (z-0 vs the
               SVG's z-1) so the curve stroke is never overwritten by
               the playhead during fast scrubs — at the column where
               they cross, the cyan curve simply paints over the
               1-px-wide translucent playhead. Earlier we used
               `mix-blend-mode: difference` here as a backstop, but
               the blend forced a compositor layer that the browser
               only partially repainted on rapid position changes,
               making the curve look as if it was being erased
               column-by-column as the playhead swept across. -->
          <div class="absolute top-0 bottom-0 w-px bg-cyan-300/70 pointer-events-none"
               :style="{ left: timeToPct(playerPos) + '%', zIndex: 0 }"></div>
          </div>
        </div>
      </div>           <!-- /track v-for -->

      <!-- ── Trigger tracks ─────────────────────────────────────────
           Discrete-event lanes: each keyframe is a one-shot dispatch
           to a routing-sheet WS input or a ROS topic field when the
           playhead crosses its time. Compact 36-px rows (independent
           of Keys/Curves mode since triggers have no value curve). -->
      <div v-if="triggerTracks.length" class="border-t border-line/40">
        <div v-for="track in triggerTracks" :key="'trig-' + track.id"
             class="flex items-stretch border-b border-line-subtle/60 group"
             style="height: 36px">
          <div class="w-56 shrink-0 flex items-center gap-2 px-2 border-r border-line/40 bg-canvas/60 sticky left-0 z-10">
            <span class="material-icons icon-sm text-amber-400 shrink-0" title="Trigger track">bolt</span>
            <input class="input-field flex-1 text-xs py-1 min-w-0"
                   :value="track.name" :title="track.id"
                   @input="onTriggerTrackNameInput(track, $event)" />
            <button class="btn-sm bg-panel hover:bg-red-600 text-fg-muted hover:text-fg-strong opacity-0 group-hover:opacity-100 transition-opacity"
                    :title="`Remove ${track.name || track.id}`"
                    @click="onRemoveTriggerTrack(track)">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </div>
          <div class="relative flex-1 cursor-crosshair bg-canvas"
               @click="onTriggerLaneClick(track, $event)">
            <!-- tick guides -->
            <div v-for="t in tickTimes" :key="t"
                 class="absolute top-0 bottom-0 border-l border-line-subtle/60 pointer-events-none"
                 :style="{ left: timeToPct(t) + '%' }"></div>
            <!-- keyframes (diamonds — same shape as value keys but
                 amber-tinted so triggers read distinct from joints) -->
            <div v-for="(kf, ki) in track.keyframes" :key="ki"
                 :class="['trigger-keyframe',
                          isTriggerKeyframeSelected(track.id, ki) ? 'is-selected' : '']"
                 :style="{ left: timeToPct(kf.time) + '%' }"
                 :title="`t=${Number(kf.time).toFixed(2)} → ${kf.target_kind}`"
                 @mousedown.stop="onTriggerKeyframeMouseDown(track, ki, $event)"
                 @click.stop></div>
            <!-- playhead -->
            <div class="absolute top-0 bottom-0 w-px bg-cyan-300/70 pointer-events-none"
                 :style="{ left: timeToPct(playerPos) + '%', zIndex: 0 }"></div>
          </div>
        </div>
      </div>

      </div>           <!-- /timeline-content -->
    </div>             <!-- /scroll container -->

    <!-- Bottom bar: pinned to the timeline's lower edge. Holds the
         joint-add control, zoom controls, and the Keys / Curves toggle. -->
    <div class="flex items-center justify-between gap-3 px-3 py-2 border-t border-line/50 bg-canvas shrink-0">
      <div class="flex items-center gap-1 shrink-0">
        <div class="relative shrink-0">
          <button class="btn-sm bg-surface hover:bg-cyan-600 text-fg-strong hover:text-fg-strong"
                  :disabled="!unboundJoints.length"
                  title="Bind a URDF joint as a value track"
                  @click="showJointMenu = !showJointMenu">
            <span class="material-icons icon-sm">add</span>
            Joint
          </button>
          <div v-if="showJointMenu"
               class="absolute bottom-full mb-1 left-0 w-56 max-h-72 overflow-y-auto bg-panel border border-line rounded-lg shadow-xl z-20">
            <button v-for="j in unboundJoints" :key="j"
                    class="block w-full text-left px-3 py-1.5 text-xs text-fg-strong hover:bg-cyan-600 hover:text-fg-strong"
                    @click="pickJoint(j)">{{ j }}</button>
            <p v-if="!unboundJoints.length" class="px-3 py-1.5 text-xs text-fg-faint italic">All joints bound.</p>
          </div>
        </div>
        <button class="btn-sm bg-surface hover:bg-amber-600 text-fg-strong hover:text-fg-strong"
                title="Add a trigger track (discrete events: WS-input or topic dispatch)"
                @click="onAddTriggerTrack">
          <span class="material-icons icon-sm">add</span>
          Trigger
        </button>
      </div>

      <!-- Zoom controls — slider plus step buttons + a "fit" reset.
           At 1.0 the timeline fits the container; higher zooms scroll
           horizontally with the sticky label columns. -->
      <div class="flex items-center gap-2 flex-1 max-w-md">
        <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong shrink-0"
                title="Zoom out" @click="bumpZoom(-1)" :disabled="zoom <= ZOOM_MIN">
          <span class="material-icons icon-sm">remove</span>
        </button>
        <input type="range"
               :min="ZOOM_MIN" :max="ZOOM_MAX" step="0.1"
               v-model.number="zoom"
               class="flex-1 accent-cyan-400 min-w-0" />
        <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong shrink-0"
                title="Zoom in" @click="bumpZoom(1)" :disabled="zoom >= ZOOM_MAX">
          <span class="material-icons icon-sm">add</span>
        </button>
        <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong shrink-0 text-[10px] tabular-nums w-14 justify-center"
                title="Reset zoom to fit"
                @click="zoomToFit">{{ zoomPct }}%</button>
      </div>

      <div class="inline-flex bg-panel rounded overflow-hidden shrink-0">
        <button :class="['mode-btn', mode === 'dope' ? 'is-active' : '']"
                title="Keys — compact dope-sheet" @click="mode = 'dope'">Keys</button>
        <button :class="['mode-btn', mode === 'curve' ? 'is-active' : '']"
                title="Curves — drag values in 2D" @click="mode = 'curve'">Curves</button>
      </div>
    </div>

    <!-- Teleport'd easing picker. Categorized two-column grid covers
         the full easings.net palette + CSS basics without scrolling. -->
    <Teleport to="body">
      <div v-if="openEasingMenu && openEasingMenuTrack"
           class="seg-menu-floating"
           :style="{
             left: openEasingMenuAnchor.x + 'px',
             top: openEasingMenuAnchor.y + 'px',
             maxHeight: (openEasingMenuAnchor.maxH || 320) + 'px',
             transform: openEasingMenuAnchor.dir === 'above'
               ? 'translate(-50%, -100%)'
               : 'translateX(-50%)',
           }"
           @click.stop @mousedown.stop>
        <div v-for="group in EASING_CATS" :key="group.category" class="seg-menu-group">
          <div class="seg-menu-cat">{{ group.category }}</div>
          <div class="seg-menu-grid">
            <button v-for="opt in group.items" :key="opt.code"
                    :class="['seg-menu-item', (openEasingMenuTrack.track.curve.keys[openEasingMenuTrack.fromIdx].interp ?? 1) === opt.code ? 'is-active' : '']"
                    :title="opt.label"
                    @click="applyEasingFromMenu(opt.code)">
              <span v-html="easingGlyph(opt.code)" class="seg-menu-glyph"></span>
              <span class="seg-menu-label">{{ opt.label.replace(/^Ease (In Out|In|Out) /, '$1 ').trim() }}</span>
            </button>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<style scoped>
.timeline-row { background-image: linear-gradient(180deg, var(--color-canvas) 0%, var(--color-canvas) 100%); }

.mode-btn {
  font-size: 11px; padding: 2px 8px; color: var(--color-fg-muted); cursor: pointer;
  transition: background-color 0.1s;
}
.mode-btn:hover { background: var(--color-surface); color: var(--color-fg); }
.mode-btn.is-active { background: #0e7490; color: white; }

.keyframe {
  position: absolute;
  width: 12px; height: 12px;
  transform: translate(-50%, -50%) rotate(45deg);
  background-color: var(--color-canvas);
  border: 2px solid #38bdf8;
  cursor: grab;
  transition: transform 0.05s;
  z-index: 2;
}
.keyframe:hover { transform: translate(-50%, -50%) rotate(45deg) scale(1.18); }
.keyframe:active { cursor: grabbing; }

/* Trigger-track keyframe — amber-tinted diamond to read distinct from
   value keyframes (cyan-bordered). Vertically centered in the 36-px
   lane. */
.trigger-keyframe {
  position: absolute;
  top: 50%;
  width: 12px; height: 12px;
  transform: translate(-50%, -50%) rotate(45deg);
  background-color: #f59e0b;
  border: 2px solid #fde68a;
  cursor: grab;
  transition: transform 0.05s;
  z-index: 2;
}
.trigger-keyframe:hover { transform: translate(-50%, -50%) rotate(45deg) scale(1.18); }
.trigger-keyframe:active { cursor: grabbing; }
.trigger-keyframe.is-selected {
  background-color: #fbbf24 !important;
  border-color: #fcd34d !important;
  box-shadow: 0 0 0 2px rgba(251, 191, 36, 0.5);
}

.keyframe.is-selected {
  background-color: #fbbf24 !important;
  border-color: #fcd34d !important;
  box-shadow: 0 0 0 2px rgba(251, 191, 36, 0.35);
}

.seg-interp {
  position: absolute;
  top: 50%;                                /* centered in the chip strip */
  transform: translate(-50%, -50%);
  width: 20px; height: 20px;
  display: inline-flex; align-items: center; justify-content: center;
  border-radius: 50%;
  background: rgba(15, 23, 42, 0.92);
  border: 1px solid rgba(148, 163, 184, 0.4);
  color: var(--color-fg-muted);
  cursor: pointer;
  transition: background-color 0.1s, color 0.1s, opacity 0.1s, border-color 0.1s;
  opacity: 0.7;
}
.seg-interp:hover {
  background: #0e7490;
  border-color: #0891b2;
  color: white;
  opacity: 1;
}

.seg-menu-floating {
  position: fixed;
  /* transform is set inline so the menu can flip above the chip when
   * there's no room below; max-height also comes inline based on
   * available space. */
  background: var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 8px;
  box-shadow: 0 18px 50px -12px rgba(0,0,0,0.7);
  padding: 8px;
  z-index: 9999;
  width: 340px;
  overflow-y: auto;
}
.seg-menu-group { margin-bottom: 4px; }
.seg-menu-group:last-child { margin-bottom: 0; }
.seg-menu-cat {
  font-size: 10px; text-transform: uppercase; letter-spacing: 0.06em;
  color: var(--color-fg-faint);
  padding: 4px 6px 2px;
}
.seg-menu-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 2px;
}
.seg-menu-item {
  display: flex; flex-direction: column; align-items: center; gap: 2px;
  padding: 6px 4px;
  border-radius: 4px;
  font-size: 10px;
  color: var(--color-fg-muted);
  cursor: pointer;
  text-align: center;
  line-height: 1.1;
}
.seg-menu-item:hover { background: var(--color-surface); color: var(--color-fg-strong); }
.seg-menu-item.is-active {
  background: rgba(6, 182, 212, 0.18);
  color: var(--color-cyan-300);
}
.seg-menu-glyph { display: inline-flex; color: currentColor; }
.seg-menu-label { display: block; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; max-width: 100%; }
</style>
