<script setup>
import { computed, defineAsyncComponent, onBeforeUnmount, onMounted, provide, ref, shallowRef, watch } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { useRobotModelStore } from '@/stores/robotModel'
import { useAnimationsStore } from '@/stores/animations'
import { useWsStore } from '@/stores/ws'
import { sampleAllTracks, sampleCurve } from '@/composables/useCurveSampling'
import TimelineEditor from '@/components/animation/TimelineEditor.vue'

const props = defineProps({
  id: { type: String, default: '' },
})

const URDFViewer = defineAsyncComponent(
  () => import('@/components/animation/URDFViewer.vue')
)
const PropsPanel = defineAsyncComponent(
  () => import('@/components/animation/PropsPanel.vue')
)

const route = useRoute()
const router = useRouter()
const robot = useRobotModelStore()
const animations = useAnimationsStore()
const ws = useWsStore()

// ── Shared state (provided to descendants) ─────────────────────────

const viewerRef  = shallowRef(null)
const jointNames = ref([])
const selection  = ref({ kind: null, value: null })
const playerPos  = ref(0)
const liveJointAngle = ref({ name: null, angle: 0 })

provide('urdf-viewer', viewerRef)
provide('urdf-joints', jointNames)
provide('animation-selection', selection)
provide('player-pos', playerPos)
provide('live-joint-angle', liveJointAngle)

const anim = computed(() => animations.editing)
const editingId = computed(() => anim.value?.id || null)
const trackIds = computed(() => new Set((anim.value?.value_tracks || []).map(t => t.id)))
const unboundJoints = computed(() => jointNames.value.filter(n => !trackIds.value.has(n)))

const playingState = computed(() => {
  if (!anim.value) return null
  return animations.players.find(p => p.id === anim.value.id) || null
})

// ── Auto-key helpers (provided to PropsPanel's joint mode) ─────────

function ensureJointTrack (jointName) {
  if (!anim.value) return null
  const existing = anim.value.value_tracks?.find(t => t.id === jointName)
  if (existing) return existing.id
  animations.snapshot({ force: true })
  anim.value.value_tracks.push({
    id: jointName, name: jointName,
    curve: { name: jointName, keys: [] },
  })
  animations.markDirty()
  return jointName
}
function setKeyframeAtPlayhead (jointName, value) {
  if (!anim.value) return null
  const trackId = ensureJointTrack(jointName)
  const track = anim.value.value_tracks.find(t => t.id === trackId)
  if (!track) return null
  const t = Number(playerPos.value) || 0
  const existing = track.curve.keys.findIndex(k => Math.abs(k.time - t) < 0.001)
  const key = { time: t, value: Number(value), interp: 1,
                arrive_tangent: 0, leave_tangent: 0 }
  if (existing >= 0) {
    track.curve.keys[existing] = key
  } else {
    const insertAt = track.curve.keys.findIndex(k => k.time > t)
    if (insertAt === -1) track.curve.keys.push(key)
    else track.curve.keys.splice(insertAt, 0, key)
  }
  animations.markDirty()
  return trackId
}
provide('set-keyframe-at-playhead', setKeyframeAtPlayhead)
provide('ensure-joint-track', ensureJointTrack)

// ── Trigger track helpers ──────────────────────────────────────────
//
// Trigger tracks carry discrete events — a target (routing-sheet WS
// input or ROS topic field) + a value that fires once when the
// playhead crosses its keyframe time.

function addTriggerTrack (name = '') {
  if (!anim.value) return null
  if (!Array.isArray(anim.value.trigger_tracks)) anim.value.trigger_tracks = []
  animations.snapshot({ force: true })
  // Unique id within the animation. "trig1", "trig2", …
  const used = new Set(anim.value.trigger_tracks.map(t => t.id))
  let n = 1
  while (used.has(`trig${n}`)) n++
  const id = `trig${n}`
  anim.value.trigger_tracks.push({
    id, name: name || `Trigger ${n}`, keyframes: [],
  })
  animations.markDirty()
  return id
}
function addTriggerKeyframeAt (trackId, time) {
  if (!anim.value) return
  const track = anim.value.trigger_tracks?.find(t => t.id === trackId)
  if (!track) return
  animations.snapshot({ force: true })
  const t = Math.max(0, Math.min(Number(anim.value.duration) || 0, Number(time) || 0))
  const kf = {
    time: t, target_kind: 'ws_input', target: ['', ''], value: 1, label: '',
  }
  const insertAt = track.keyframes.findIndex(k => k.time > t)
  if (insertAt === -1) track.keyframes.push(kf)
  else track.keyframes.splice(insertAt, 0, kf)
  animations.markDirty()
}
provide('add-trigger-track', addTriggerTrack)
provide('add-trigger-keyframe-at', addTriggerKeyframeAt)

// WS-input + topic catalog for the trigger keyframe target picker.
// Lazily loaded the first time the props panel asks for them.
const wsInputCatalog = ref([])   // [{ sheet_id, sheet_label, input_id, label, kind }]
const topicCatalogForTriggers = ref([])  // [{ topic, channels: [{ field, label, type }] }]
async function loadTriggerTargets () {
  if (!wsInputCatalog.value.length) {
    try {
      // Pulls the live set of WS-input nodes across every routing
      // sheet. MUST use the router channel + `list_websocket_inputs`
      // action — the server's _handle_router has that handler
      // (websocket_handler.py around line 1813), and there is no
      // management-channel "list_ws_inputs" action at all. Using the
      // wrong name made every request return "Unknown action", which
      // the catch silently swallowed → wsInputCatalog stayed empty
      // → the trigger-target sheet picker showed "No WS inputs
      // available" even when the operator had plenty defined.
      // PoseLibrary.vue's reloadInputs() uses the same call and
      // works correctly — this brings the animation editor into
      // parity.
      const r = await ws.router('list_websocket_inputs', {})
      // State-only WS inputs (the migration-only echoes of state-
      // only ROS endpoints) aren't valid trigger targets — a trigger
      // by definition pushes a value, and you can't push into a
      // state echo. Filter to "command" kind, matching how
      // PoseLibrary scopes its setpoint picker.
      const raw = r?.ws_inputs || r?.inputs || []
      wsInputCatalog.value = raw.filter(x => x.kind !== 'state')
    } catch (e) { console.warn('list_websocket_inputs failed:', e) }
  }
  if (!topicCatalogForTriggers.value.length) {
    try {
      // Same source the Routes view uses for its inputs/outputs
      // modals — keeps the catalog consistent across views.
      const r = await ws.send('ros', 'list_topic_channels', {})
      topicCatalogForTriggers.value = r?.topics || []
    } catch (e) { console.warn('list_topic_channels failed:', e) }
  }
}
provide('trigger-targets', { wsInputs: wsInputCatalog, topics: topicCatalogForTriggers, load: loadTriggerTargets })

// ── URDF live driving from the curve ───────────────────────────────

function driveUrdfFromPlayhead () {
  const v = viewerRef?.value
  if (!v?.setJointValue || !anim.value) return
  const sampled = sampleAllTracks(anim.value, playerPos.value)
  for (const [jointName, val] of Object.entries(sampled)) {
    v.setJointValue(jointName, val)
  }
}
watch(playerPos, (now, prev) => {
  driveUrdfFromPlayhead()
  if (livePreviewActive()) sendLivePreview(now, crossedTriggers(prev ?? now, now))
})
watch(
  () => JSON.stringify(anim.value?.value_tracks || []),
  () => {
    driveUrdfFromPlayhead()
    // A value keyframe was edited; time didn't move, so re-push values
    // only (no trigger crossing).
    if (livePreviewActive()) sendLivePreview(playerPos.value, [])
  },
)
// Trigger keyframe edits: fire any trigger sitting AT the current
// playhead (within a frame) so adjusting a point at the cursor shows
// its effect immediately — matching the value-track behavior above.
watch(
  () => JSON.stringify(anim.value?.trigger_tracks || []),
  () => {
    if (!livePreviewActive()) return
    const t = playerPos.value
    const trg = []
    for (const tt of anim.value?.trigger_tracks || []) {
      for (const kf of tt.keyframes || []) {
        if (Math.abs((kf.time || 0) - t) <= 0.05) {
          trg.push({ target_kind: kf.target_kind, target: kf.target, value: kf.value })
        }
      }
    }
    if (trg.length) sendLivePreview(t, trg)
  },
)
// When the server confirms playback started, sync the playhead once.
// After that the RAF loop below takes over for smooth client-side
// motion at 60 Hz — polling every 100 ms gave a stair-stepped scrub
// that felt much slower than dragging.
watch(playingState, (s, prev) => {
  if (s && !prev) playerPos.value = s.t
})

// ── Live Preview ───────────────────────────────────────────────────
//
// When enabled, scrubbing the timeline or editing a keyframe at the
// playhead pushes the sampled value-track frame (and any crossed
// triggers) straight into the routing graph via the server's
// preview_animation_frame action — same dispatch path the player uses
// — so the operator sees real-time impact on the rig without starting
// playback. Gated to when the SERVER player isn't already running this
// animation (it drives the rig itself in that case).
const livePreview = ref(false)
function livePreviewActive () {
  return livePreview.value && !!anim.value && !playingState.value?.running
}

// Build the value-track frame at time t: ws_input-bound tracks address
// (sheet, input); everything else is a URDF joint keyed by track id.
function buildPreviewValues (t) {
  const out = []
  for (const tr of anim.value?.value_tracks || []) {
    const v = sampleCurve(tr.curve, t)
    if (tr.target_kind === 'ws_input' && tr.target?.length >= 2) {
      out.push({ target_kind: 'ws_input', target: tr.target, value: v })
    } else {
      out.push({ target_kind: 'urdf_joint', id: tr.id, value: v })
    }
  }
  return out
}
// Triggers whose time falls in the (prev, now] window — forward only,
// matching the player so a backward scrub doesn't re-fire events.
function crossedTriggers (prevT, nowT) {
  if (nowT <= prevT) return []
  const out = []
  for (const tt of anim.value?.trigger_tracks || []) {
    for (const kf of tt.keyframes || []) {
      if (kf.time > prevT && kf.time <= nowT) {
        out.push({ target_kind: kf.target_kind, target: kf.target, value: kf.value })
      }
    }
  }
  return out
}

// ~30 Hz leading+trailing throttle on the value frame so a 60 fps scrub
// doesn't flood the management channel; crossed triggers accumulate
// across the throttle window so none are dropped, and the trailing call
// always lands the final resting frame.
let _previewLast = 0
let _previewTimer = null
let _pendingTriggers = []
function sendLivePreview (t, triggers) {
  if (triggers && triggers.length) _pendingTriggers.push(...triggers)
  const flush = () => {
    _previewLast = Date.now()
    const trg = _pendingTriggers; _pendingTriggers = []
    animations.previewFrame(buildPreviewValues(t), trg)
  }
  const wait = 33 - (Date.now() - _previewLast)
  clearTimeout(_previewTimer)
  if (wait <= 0) flush()
  else _previewTimer = setTimeout(flush, wait)
}
// Relax the rig (all value targets → 0) when Live Preview turns off or
// the editor unmounts, mirroring the player's stop-settles-to-neutral
// behavior so the rig doesn't hold the last previewed pose.
function relaxLivePreview () {
  const zeros = (anim.value?.value_tracks || []).map(tr =>
    (tr.target_kind === 'ws_input' && tr.target?.length >= 2)
      ? { target_kind: 'ws_input', target: tr.target, value: 0 }
      : { target_kind: 'urdf_joint', id: tr.id, value: 0 })
  if (zeros.length) animations.previewFrame(zeros, [])
}
watch(livePreview, (on) => {
  if (on) sendLivePreview(playerPos.value, [])   // snap rig to current frame
  else relaxLivePreview()
})

// Local playback loop. requestAnimationFrame ticks at the display
// refresh rate (typically 60 Hz), so the playhead moves at the same
// cadence as the user's manual scrub. The server's own player keeps
// running independently for ROS / peripheral dispatch — we only
// compute the UI's time cursor here.
let _rafHandle = 0
let _rafStartMs = 0
let _rafStartPos = 0
function startPlayLoop () {
  if (_rafHandle) return
  _rafStartMs = Date.now()
  _rafStartPos = playerPos.value
  const tick = () => {
    if (!playingState.value?.running) { _rafHandle = 0; return }
    const dur = Number(anim.value?.duration || 0)
    const elapsed = (Date.now() - _rafStartMs) / 1000
    let t = _rafStartPos + elapsed
    if (dur > 0 && t >= dur) {
      if (anim.value?.loop) {
        _rafStartMs = Date.now()
        _rafStartPos = 0
        t = 0
      } else {
        playerPos.value = dur
        // Stop server-side too so the player registry clears and the
        // button flips back to ▶ via the playingState watcher.
        if (anim.value?.id) animations.stop(anim.value.id)
        _rafHandle = 0
        return
      }
    }
    playerPos.value = t
    _rafHandle = requestAnimationFrame(tick)
  }
  _rafHandle = requestAnimationFrame(tick)
}
function stopPlayLoop () {
  if (_rafHandle) { cancelAnimationFrame(_rafHandle); _rafHandle = 0 }
}
watch(() => !!playingState.value?.running, (running) => {
  if (running) startPlayLoop()
  else stopPlayLoop()
})

// ── Joint tracks ───────────────────────────────────────────────────

function ensureUniqueTrackId (base) {
  const ids = trackIds.value
  if (!ids.has(base)) return base
  let n = 2
  while (ids.has(`${base}_${n}`)) n++
  return `${base}_${n}`
}
function addJointTrack (jointName) {
  if (!anim.value) return
  animations.snapshot({ force: true })
  const id = ensureUniqueTrackId(jointName)
  anim.value.value_tracks.push({
    id, name: jointName,
    target_kind: 'urdf_joint', target: [],
    curve: { name: jointName, keys: [] },
  })
  animations.markDirty()
  selection.value = { kind: 'track', trackId: id }
}
// Bind a controller routing-sheet WS input as a value track — the path
// that lets animations be authored with no URDF. The sampled curve
// value is pushed via set_ws_input on the server (see ValueTrack
// target_kind="ws_input"), routed like any other controller input.
function addWsInputTrack (payload) {
  if (!anim.value || !payload?.sheet_id || !payload?.ws_input_id) return
  animations.snapshot({ force: true })
  const label = payload.label || payload.ws_input_id
  const id = ensureUniqueTrackId(`${payload.sheet_id}.${payload.ws_input_id}`)
  anim.value.value_tracks.push({
    id, name: label,
    target_kind: 'ws_input',
    target: [payload.sheet_id, payload.ws_input_id],
    curve: { name: label, keys: [] },
  })
  animations.markDirty()
  selection.value = { kind: 'track', trackId: id }
}
function renameTrack (track, newName) {
  track.name = newName
  animations.markDirty()
}
function removeTrack (idx) {
  const removed = anim.value.value_tracks.splice(idx, 1)[0]
  if (removed && selection.value?.trackId === removed.id) {
    selection.value = { kind: null }
  }
  animations.markDirty()
}
function renameTriggerTrack (track, newName) {
  track.name = newName
  animations.markDirty()
}
function removeTriggerTrack (trackId) {
  if (!anim.value?.trigger_tracks) return
  const idx = anim.value.trigger_tracks.findIndex(t => t.id === trackId)
  if (idx < 0) return
  anim.value.trigger_tracks.splice(idx, 1)
  if (selection.value?.trackId === trackId) selection.value = { kind: null }
  animations.markDirty()
}

// ── Selection / gizmo wiring ───────────────────────────────────────

function onTimelineSelect (s) { selection.value = s || { kind: null } }
function onPropsSelect (s) { selection.value = s || { kind: null } }
function onJointsChanged (names) { jointNames.value = names }
// `alternatives` arrives when the click landed on a region where
// multiple non-fixed joints share the same origin (e.g. NAO's
// shoulder = Pitch + Roll). We hand them to the props panel via
// inject so it can render a one-click chooser.
const jointAlternatives = ref([])
provide('joint-alternatives', jointAlternatives)
function onJointClicked (jointName, alternatives = []) {
  jointAlternatives.value = (alternatives && alternatives.length > 1) ? alternatives : []
  selection.value = { kind: 'joint', value: jointName }
}
// Selection drives both the 3D gizmo and the playhead:
//   • kind=joint     → attach gizmo to that joint
//   • kind=keyframe  → seek playerPos to the keyframe's time AND
//                      attach gizmo to its track's joint, so the
//                      operator can re-pose via the gizmo and the
//                      auto-key (drag-end) lands on this exact key
//   • otherwise      → detach
watch(selection, (s) => {
  // Drop the co-located-joint chooser whenever the selection moves
  // away from one of those alternatives. Keeps the chooser visible
  // only while the operator is still cycling between joints in the
  // cluster they just clicked.
  if (s?.kind !== 'joint' || !jointAlternatives.value.includes(s.value)) {
    jointAlternatives.value = []
  }
  const v = viewerRef?.value
  if (!v?.selectJoint) return
  if (s?.kind === 'joint' && s.value) {
    // Pass the full cluster so the viewer renders one draggable
    // gizmo per co-located joint. The first arg marks the active
    // (gizmo-default-target) joint.
    const cluster = jointAlternatives.value.length > 1
      ? jointAlternatives.value
      : [s.value]
    v.selectJoint(s.value, cluster)
  } else if (s?.kind === 'keyframe' && anim.value) {
    const track = anim.value.value_tracks?.find(t => t.id === s.trackId)
    const kf = track?.curve?.keys?.[s.kfIdx]
    if (kf) {
      playerPos.value = Number(kf.time) || 0
      // Only joint-bound tracks drive the 3D gizmo; ws_input tracks
      // have no URDF joint, so detach rather than hunt for a joint
      // named after the sheet binding (which would never match).
      v.selectJoint(track.target_kind === 'ws_input' ? null : track.id)
    } else {
      v.selectJoint(null)
    }
  } else {
    v.selectJoint(null)
  }
}, { deep: true, immediate: true })

function onGizmoRotate (jointName, angle) {
  liveJointAngle.value = { name: jointName, angle }
}
function onGizmoCommit (jointName, angle) {
  if (!anim.value) return
  animations.snapshot({ force: true })
  setKeyframeAtPlayhead(jointName, angle)
}

function onDeleteKeyframe (trackId, kfIdx) {
  const t = anim.value?.value_tracks?.find(x => x.id === trackId)
  if (!t) return
  t.curve.keys.splice(kfIdx, 1)
  animations.markDirty()
}

// ── Transport ──────────────────────────────────────────────────────

async function play () {
  if (!anim.value?.id) return
  await animations.start(anim.value.id, anim.value.loop)
}
async function stopPlayback () {
  if (!anim.value?.id) return
  await animations.stop(anim.value.id)
}
async function seek () {
  if (!anim.value?.id) return
  await animations.seek(anim.value.id, Number(playerPos.value))
}
function onTimelineScrub (t) {
  playerPos.value = Number(t) || 0
  if (playingState.value) seek()
}

// ── Toolbar actions + global shortcuts ─────────────────────────────

async function backToList () { router.push({ name: 'animations' }) }

async function deleteCurrent () {
  if (!anim.value?.id) return
  if (!confirm(`Delete animation "${anim.value.name || anim.value.id}"?`)) return
  await animations.remove(anim.value.id)
  router.push({ name: 'animations' })
}
async function saveCurrent () { await animations.save() }

function onKeyDown (e) {
  const mod = e.metaKey || e.ctrlKey
  if (!mod) return
  const key = e.key?.toLowerCase()
  if (key !== 'z' && key !== 'y') return
  const target = e.target
  const tag = target?.tagName
  const inField = tag === 'INPUT' || tag === 'TEXTAREA' || target?.isContentEditable
  if (inField) return
  e.preventDefault()
  if ((key === 'z' && e.shiftKey) || key === 'y') animations.redo()
  else animations.undo()
}
function onFocusIn (e) {
  if (!animations.editing) return
  const tag = e.target?.tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') {
    animations.snapshot()
  }
}

const isMac = typeof navigator !== 'undefined' && /Mac/.test(navigator.platform || '')
const undoHint = isMac ? '⌘Z' : 'Ctrl+Z'
const redoHint = isMac ? '⇧⌘Z' : 'Ctrl+Y'

// ── Route binding ──────────────────────────────────────────────────

async function loadFromRoute () {
  const id = props.id || route.params?.id
  if (!id) return
  if (anim.value?.id === id) return
  await animations.load(id)
  if (!anim.value) router.replace({ name: 'animations' })
}
watch(() => route.params?.id, loadFromRoute)
watch(editingId, () => {
  playerPos.value = 0
  driveUrdfFromPlayhead()
  selection.value = { kind: null }
})

// ── Lifecycle ──────────────────────────────────────────────────────

onMounted(async () => {
  window.addEventListener('keydown', onKeyDown)
  document.addEventListener('focusin', onFocusIn)
  await Promise.all([robot.refresh(), animations.reload()])
  // Load the WS-input catalog up front so the timeline's "+ Input"
  // dropdown is populated immediately — value tracks can bind a
  // controller sheet input even when no URDF is installed.
  loadTriggerTargets()
  await loadFromRoute()
  // Land at t=0 with the URDF reflecting any saved keyframes.
  playerPos.value = 0
  driveUrdfFromPlayhead()
})
onBeforeUnmount(() => {
  window.removeEventListener('keydown', onKeyDown)
  document.removeEventListener('focusin', onFocusIn)
  stopPlayLoop()
  // Relax the live rig so leaving the editor with Live Preview on
  // doesn't strand the robot in the previewed pose.
  if (livePreview.value) { clearTimeout(_previewTimer); relaxLivePreview() }
  // Reset URDF to neutral so leaving doesn't leave joints frozen.
  const v = viewerRef?.value
  if (v?.setJointValue) {
    for (const name of jointNames.value) v.setJointValue(name, 0)
  }
})
</script>

<template>
  <section class="page animations-editor-page">
    <div class="editor-shell">
      <!-- Toolbar — spans the full page width across the top. -->
      <div class="editor-toolbar">
        <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong shrink-0"
                title="Back to all animations" @click="backToList">
          <span class="material-icons icon-sm">arrow_back</span>
          All
        </button>
        <div class="flex-1 min-w-0 flex items-center gap-2">
          <span class="material-icons text-fg shrink-0">
            {{ anim?.icon || 'animation' }}
          </span>
          <div class="min-w-0">
            <h2 class="text-base font-semibold text-fg-strong truncate">
              {{ anim?.name || anim?.id || 'Loading…' }}
              <span v-if="animations.dirty" class="text-xs text-amber-300 font-normal ml-2">• Unsaved</span>
            </h2>
            <p class="text-xs text-fg-muted truncate">
              <template v-if="anim">
                <span v-if="anim.group" class="text-fg">{{ anim.group }} ·</span>
                {{ anim.value_tracks?.length || 0 }} value ·
                {{ anim.trigger_tracks?.length || 0 }} trigger ·
                {{ Number(anim.duration || 0).toFixed(2) }}s ·
                {{ anim.fps || 60 }} fps{{ anim.loop ? ' · loop' : '' }}
              </template>
            </p>
          </div>
        </div>
        <div class="flex items-center gap-1 shrink-0">
          <button :class="['btn-sm flex items-center gap-1',
                           livePreview
                             ? 'bg-emerald-500/90 hover:bg-emerald-500 text-fg-strong'
                             : 'bg-surface hover:bg-surface-2 text-fg-strong']"
                  title="Live Preview — push sampled positions + triggers to the rig in real time as you scrub or edit (disabled while playing)"
                  :disabled="!!playingState?.running"
                  @click="livePreview = !livePreview">
            <span class="material-icons icon-sm">{{ livePreview ? 'sensors' : 'sensors_off' }}</span>
            Live
          </button>
          <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                  :title="`Undo (${undoHint})`"
                  :disabled="!animations.canUndo" @click="animations.undo()">
            <span class="material-icons icon-sm">undo</span>
          </button>
          <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                  :title="`Redo (${redoHint})`"
                  :disabled="!animations.canRedo" @click="animations.redo()">
            <span class="material-icons icon-sm">redo</span>
          </button>
          <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                  title="Delete this animation"
                  :disabled="!anim?.id" @click="deleteCurrent">
            <span class="material-icons icon-sm">delete</span>
          </button>
          <button class="btn-sm bg-cyan-600 hover:bg-cyan-500 text-fg-strong"
                  title="Save"
                  :disabled="!animations.dirty" @click="saveCurrent">
            <span class="material-icons icon-sm">save</span>
            Save
          </button>
        </div>
      </div>

      <!-- Preview row: URDF on the left, Props panel as its sidebar. -->
      <div class="editor-preview-row">
        <div class="editor-preview-main">
          <template v-if="robot.installed">
            <URDFViewer ref="viewerRef"
                        :urdf-url="robot.urdfUrl"
                        :meshes-base="robot.meshesBase"
                        height="100%"
                        @joints="onJointsChanged"
                        @joint-click="onJointClicked"
                        @joint-rotate="onGizmoRotate"
                        @joint-rotate-commit="onGizmoCommit" />
          </template>
          <div v-else class="absolute inset-0 flex items-center justify-center text-sm text-fg-muted">
            <div class="text-center">
              <span class="material-icons text-fg-faint" style="font-size:3rem">view_in_ar</span>
              <p class="mt-2">No robot model uploaded.</p>
              <RouterLink to="/settings" class="text-cyan-400 hover:text-cyan-300 underline mt-1 inline-block">
                Upload one in Settings → Robot Model
              </RouterLink>
            </div>
          </div>
        </div>

        <aside v-if="anim" class="editor-props">
          <div class="editor-props-header">Properties</div>
          <div class="editor-props-body">
            <PropsPanel :animation="anim"
                        :selection="selection"
                        :animations="animations.list"
                        @dirty="animations.markDirty()"
                        @select="onPropsSelect"
                        @delete-keyframe="onDeleteKeyframe" />
          </div>
        </aside>
      </div>

      <!-- Timeline — full page width below the preview row. -->
      <div class="editor-timeline">
        <TimelineEditor v-if="anim"
                        :animation="anim"
                        :player-pos="playerPos"
                        :selection="selection"
                        :playing="!!playingState?.running"
                        :unbound-joints="unboundJoints"
                        :ws-inputs="wsInputCatalog"
                        @update:player-pos="onTimelineScrub"
                        @select="onTimelineSelect"
                        @dirty="animations.markDirty()"
                        @rename-track="renameTrack"
                        @remove-track="removeTrack"
                        @rename-trigger-track="renameTriggerTrack"
                        @remove-trigger-track="removeTriggerTrack"
                        @play="play"
                        @stop="stopPlayback"
                        @add-joint="addJointTrack"
                        @add-ws-input="addWsInputTrack" />
      </div>
    </div>
  </section>
</template>
