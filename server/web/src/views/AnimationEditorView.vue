<script setup>
import { computed, defineAsyncComponent, onBeforeUnmount, onMounted, provide, ref, shallowRef, watch } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { useRobotModelStore } from '@/stores/robotModel'
import { useAnimationsStore } from '@/stores/animations'
import { usePosesStore } from '@/stores/poses'
import { usePlaylistsStore } from '@/stores/playlists'
import { useSoundsStore } from '@/stores/sounds'
import { useWsStore } from '@/stores/ws'
import {
  frameToPreviewValues,
  referencedPoseIds,
  relaxedFrame,
  resolveFrame,
} from '@/composables/useFrameResolve'
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
const RigControls = defineAsyncComponent(
  () => import('@/components/animation/RigControls.vue')
)

const route = useRoute()
const router = useRouter()
const robot = useRobotModelStore()
const animations = useAnimationsStore()
const poses = usePosesStore()
const playlists = usePlaylistsStore()
// Board libraries for the + Board Item menu and the length bars it
// draws. Sounds carry a measured `duration`; animations carry their own.
const sounds = useSoundsStore()
const ws = useWsStore()

// ── Shared state (provided to descendants) ─────────────────────────

const viewerRef  = shallowRef(null)
const jointNames = ref([])
const selection  = ref({ kind: null, value: null })
const playerPos  = ref(0)
const liveJointAngle = ref({ name: null, angle: 0 })
// Self-collision results: timeline intervals from a full-animation scan, and
// the pairs colliding at the current playhead (for a live badge).
const collisionIntervals = ref([])
const liveCollisions = ref([])

provide('urdf-viewer', viewerRef)
provide('urdf-joints', jointNames)
provide('animation-selection', selection)
provide('player-pos', playerPos)
provide('live-joint-angle', liveJointAngle)

const anim = computed(() => animations.editing)
const editingId = computed(() => anim.value?.id || null)
const trackIds = computed(() => new Set((anim.value?.value_tracks || []).map(t => t.id)))
const unboundJoints = computed(() => jointNames.value.filter(n => !trackIds.value.has(n)))

// ── Pose resolution ────────────────────────────────────────────────
//
// Pose tracks need joint values, and the pose LIST only carries
// summaries — so each referenced pose is fetched once and cached. The
// cache is keyed by pose id and invalidated when the pose library
// reloads, which is what makes editing a pose show up in the viewport
// without a page refresh.
const poseJoints = ref({})          // pose id → { joint: normalized }

// The rig's neutral pose, resolved to joint values — the base a pose
// track blends UP FROM on joints nothing else has touched.
//
// This has to come from the server: the player resolves frames with
// `neutral_source=rig_neutral` (state_manager), so previewing without it
// blends from implicit zeros and every pose track at weight < 1 lands
// somewhere the played animation never goes. Empty is correct when
// there's no rig or no declared neutral_pose — then zero IS the base.
// playlist id → name, for the timeline's + Pose menu. Poses carry
// playlist ids; the names live on the playlists themselves.
// playlist id → name for every board kind. The + Board Item menu labels
// playlists across poses, sounds and animations from one map; ids are
// unique per kind on the server, so a single map is unambiguous.
const posePlaylistNames = computed(() => {
  const out = {}
  for (const p of playlists.list || []) out[p.id] = p.name
  return out
})

const rigNeutral = ref({})
async function loadRigNeutral () {
  try {
    const r = await ws.management('get_rig', {})
    rigNeutral.value = r?.neutral || {}
  } catch {
    rigNeutral.value = {}   // no rig → zeros, same as the server
  }
}

async function ensurePosesLoaded (ids) {
  const missing = ids.filter(id => !(id in poseJoints.value))
  if (!missing.length) return
  const fetched = {}
  const fetchedSetpoints = {}
  for (const id of missing) {
    try {
      const r = await ws.management('get_pose', { id })
      const setpoints = r?.pose?.setpoints || []
      // Joint setpoints only: this map feeds resolveFrame, and pose
      // layering is joint-space. A ws_input setpoint has no joint to
      // blend.
      const joints = {}
      for (const s of setpoints) {
        if (s.target_kind === 'joint' && s.joint) joints[s.joint] = s.value
      }
      // ALL setpoints, for the track's disclosure. Kept separate from
      // `joints` on purpose: a pose authored in the dashboard's pose
      // editor is made of ws_input setpoints, so filtering to joints
      // left those poses disclosing nothing at all — the pose worked,
      // the disclosure just looked broken.
      fetchedSetpoints[id] = r?.pose ? setpoints.map(toPoseTarget) : null
      // Cache the miss too (as null), so a track pointing at a deleted
      // pose doesn't re-request on every frame.
      fetched[id] = r?.pose ? joints : null
    } catch (e) {
      console.warn(`get_pose(${id}) failed:`, e)
      fetched[id] = null
      fetchedSetpoints[id] = null
    }
  }
  poseJoints.value = { ...poseJoints.value, ...fetched }
  poseSetpoints.value = { ...poseSetpoints.value, ...fetchedSetpoints }
}

// One disclosed row per pose setpoint, whatever address space it uses.
// `joint` is set only for joint setpoints — the timeline uses it to show
// the live resolved value, which only exists for joints.
function toPoseTarget (s) {
  if (s.target_kind === 'joint' && s.joint) {
    return { key: `joint:${s.joint}`, label: s.joint, joint: s.joint,
             kind: 'joint', value: Number(s.value) || 0 }
  }
  const sheet = s.sheet_id || ''
  const input = s.ws_input_id || ''
  return { key: `ws:${sheet}/${input}`, label: `${sheet}/${input}`,
           joint: '', kind: 'ws_input', value: Number(s.value) || 0 }
}

// pose id → [{key, label, joint, kind, value}] for the disclosure.
const poseSetpoints = ref({})

const poseLookup = (id) => poseJoints.value[id] || null

// Resolved joint values at the playhead, published to the timeline so a
// pose track's disclosed joint rows can show real numbers.
const resolvedJoints = ref({})

// Tracks referencing a pose that no longer exists. Surfaced rather than
// silently doing nothing — a track that quietly stopped contributing is
// the hardest kind of animation bug to find.
const missingPoses = computed(() =>
  referencedPoseIds(anim.value || {})
    .filter(id => id in poseJoints.value && poseJoints.value[id] === null))


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
  const keys = track.curve.keys
  const existing = keys.findIndex(k => Math.abs(k.time - t) < 0.001)
  if (existing >= 0) {
    // Editing an existing keyframe's value must NOT reset its easing.
    // The old code replaced the whole key with a fresh interp:1 (LINEAR)
    // object, so tweaking a value at the playhead (gizmo/slider) silently
    // dropped whatever curve the operator had set — "the curve setting
    // isn't retained." Update value in place; keep interp + tangents.
    keys[existing].value = Number(value)
    keys[existing].time = t
  } else {
    // A new keyframe inherits the interpolation of the segment it lands
    // in (the preceding key's interp), so adding a point to an eased
    // curve continues that curve instead of forcing a linear kink.
    const insertAt = keys.findIndex(k => k.time > t)
    const prevIdx = insertAt === -1 ? keys.length - 1 : insertAt - 1
    const inheritInterp = prevIdx >= 0 ? (keys[prevIdx].interp ?? 1) : 1
    const key = { time: t, value: Number(value), interp: inheritInterp,
                  arrive_tangent: 0, leave_tangent: 0 }
    if (insertAt === -1) keys.push(key)
    else keys.splice(insertAt, 0, key)
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
// A sound or animation chosen from the + Board Item menu. It becomes a
// trigger keyframe at the playhead, on its own track named after the
// item — one lane per board item, so their length bars can't overlap
// into an unreadable stack.
//
// `duration: 0` means "the item's own length"; the timeline fills the
// bar from the library. Only a looping item's bar can then be dragged,
// which is what writes a non-zero duration here.
//
// `clip_length` records that own length on the keyframe when the item
// is added, so the bar can be drawn even when the library loaded in this
// editor has no length for it (e.g. the editor was opened before the
// server measured the clip). A sound the library hasn't measured is
// measured on the server first.
async function addBoardItemTrack (payload) {
  if (!anim.value || !payload?.id) return
  const kind = payload.kind
  if (kind !== 'sound' && kind !== 'animation') return
  const trackId = addTriggerTrack(payload.label || payload.id)
  if (!trackId) return
  const track = anim.value.trigger_tracks.find(t => t.id === trackId)
  if (!track) return
  const kf = {
    time: Math.max(0, Number(playerPos.value) || 0),
    target_kind: kind,
    target: [payload.id],
    value: null,
    label: payload.label || payload.id,
    duration: 0,
    clip_length: 0,
  }
  track.keyframes.push(kf)
  animations.markDirty()
  selection.value = { kind: 'trigger-keyframe', trackId, kfIdx: 0 }

  const library = kind === 'sound' ? sounds.list : animations.list
  let length = Number(library.find(x => x.id === payload.id)?.duration) || 0
  if (length <= 0 && kind === 'sound') length = await sounds.measure(payload.id)
  // Write through the reactive track: `kf` is the raw object we pushed.
  const added = track.keyframes.find(k => k === kf || k.target?.[0] === payload.id)
  if (added && length > 0) added.clip_length = length
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
  if (!anim.value) return
  // Shared resolver — same layering rules the server player uses, so
  // the viewport can't disagree with the robot. Pose tracks layer in
  // list order over the joint tracks below them.
  const { joints } = resolveFrame(anim.value, playerPos.value,
                                  { poseLookup, neutral: rigNeutral.value })
  // Kept for the timeline: a joint disclosed under a pose track shows
  // its POST-layering value, so a track above the pose overriding it is
  // visible rather than confusing.
  resolvedJoints.value = joints
  const v = viewerRef?.value
  if (!v?.setJointValue) return
  for (const [jointName, val] of Object.entries(joints)) {
    v.setJointValue(jointName, val)
  }
  // Posing is cheap and must stay smooth; the collision check + 3D re-tint is
  // heavy (full-body pass + material swaps), so throttle it — a rapid keyframe
  // or scrub drag fires this watcher on every mousemove.
  scheduleLiveCollision()
}

let _liveCollisionTimer = null
function updateLiveCollision () {
  const v = viewerRef?.value
  if (!v?.collisionsAtCurrent) return
  liveCollisions.value = v.collisionsAtCurrent()
  v.highlightCollision?.(liveCollisions.value)
}
function scheduleLiveCollision () {
  if (_liveCollisionTimer) return // trailing throttle: coalesce a burst
  _liveCollisionTimer = setTimeout(() => {
    _liveCollisionTimer = null
    updateLiveCollision()
  }, 100)
}

// Self-collision timeline scan, gated on IDLE. The scan only runs after the
// user has paused; ANY manipulation — scrubbing, keyframe drags, sidebar edits,
// Save, orbiting the 3D view, keystrokes — aborts an in-flight scan and resets
// the idle countdown. So reprocessing never competes with interaction.
//   _needScan  : the animation data changed since the last completed scan
//   _scanning  : a scan is currently in flight
//   _idleTimer : fires runCollisionScan once input has stopped for IDLE_MS
const IDLE_MS = 350
let _needScan = false
let _scanning = false
let _idleTimer = null

// Animation data changed → a scan is needed; then wait for idle.
function requestCollisionScan () {
  _needScan = true
  deferCollisionScan()
}
// Any user activity → abort the in-flight scan and (re)start the idle timer.
function deferCollisionScan () {
  viewerRef?.value?.cancelScan?.()
  clearTimeout(_idleTimer)
  _idleTimer = setTimeout(() => {
    if (_needScan && !_scanning) runCollisionScan()
  }, IDLE_MS)
}

async function runCollisionScan () {
  const v = viewerRef?.value
  const a = anim.value
  const dur = Number(a?.duration) || 0
  if (!v?.scanTimeline || !a || !(dur > 0)) { collisionIntervals.value = []; _needScan = false; return }
  const steps = Math.min(150, Math.max(20, Math.round(dur * 15))) // ~15 samples/sec
  const _ts = performance.now()
  _scanning = true
  let res
  try {
    // Non-blocking: scanTimeline runs on the viewer's hidden clone and yields
    // between slices; interaction aborts it (returns null) via cancelScan.
    res = await v.scanTimeline(
      (t) => resolveFrame(a, t,
                          { poseLookup, neutral: rigNeutral.value }).joints,
      dur, steps)
  } catch (_) {
    res = []
  }
  _scanning = false
  if (res == null) { deferCollisionScan(); return } // aborted → retry once idle
  collisionIntervals.value = res
  _needScan = false
  if (import.meta.env?.DEV) {
    // eslint-disable-next-line no-console
    console.info(`[collision] scan: dur=${dur}s steps=${steps} ` +
      `→ ${res.length} interval(s) in ${(performance.now() - _ts) | 0}ms`)
  }
}

// Document-wide activity detector: any of these means the user is manipulating
// something (a control, a field, the timeline, the view) → defer the scan.
// pointermove only counts while a button is held (a drag), so passive cursor
// movement doesn't starve the scan.
let _pointerHeld = false
function onActivityDown () { _pointerHeld = true; deferCollisionScan() }
function onActivityUp () { _pointerHeld = false; deferCollisionScan() }
function onActivityMove () { if (_pointerHeld) deferCollisionScan() }
function onActivity () { deferCollisionScan() }
watch(playerPos, (now, prev) => {
  driveUrdfFromPlayhead()
  deferCollisionScan() // scrubbing is interaction — hold the scan until idle
  if (livePreviewActive()) sendLivePreview(now, crossedTriggers(prev ?? now, now))
})
watch(
  () => JSON.stringify(anim.value?.value_tracks || []),
  () => {
    driveUrdfFromPlayhead()
    requestCollisionScan()
    // A value keyframe was edited; time didn't move, so re-push values
    // only (no trigger crossing).
    if (livePreviewActive()) sendLivePreview(playerPos.value, [])
  },
)
// Duration changes remap every keyframe's time — rescan.
watch(() => anim.value?.duration, requestCollisionScan)
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

// Build the value-track frame at time t. Pose tracks are resolved to
// joint values HERE rather than on the server, so the preview path
// carries only the two concrete target kinds and there's exactly one
// place that knows how pose layering works on each side.
function buildPreviewValues (t) {
  return frameToPreviewValues(
    resolveFrame(anim.value || {}, t,
                 { poseLookup, neutral: rigNeutral.value }))
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
// Relax the rig when Live Preview turns off or the editor unmounts,
// mirroring the player's stop-settles-to-neutral behavior so the rig
// doesn't hold the last previewed pose. Note this is NOT "resolve the
// frame with zero weights" — a pose track at weight 0 contributes
// nothing, which would strand the joints it was moving. relaxedFrame
// collects them explicitly.
function relaxLivePreview () {
  const values = frameToPreviewValues(
    relaxedFrame(anim.value || {}, { poseLookup, neutral: rigNeutral.value }))
  if (values.length) animations.previewFrame(values, [])
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
// Default clip length for a new pose track, in seconds.
const POSE_CLIP_SECONDS = 1

// Layer a whole named pose as one track. The curve is the pose's WEIGHT
// (0..1), not a joint value.
//
// Creates a SPAN (two keys), not a single key: a pose track is a clip and
// only contributes between its first and last keyframe, so a lone key
// would be a zero-length clip that does essentially nothing. See
// useFrameResolve.js.
async function addPoseTrack (payload) {
  if (!anim.value || !payload?.pose_id) return
  animations.snapshot({ force: true })
  const label = payload.label || payload.pose_id
  const id = ensureUniqueTrackId(`pose.${payload.pose_id}`)
  const dur = Number(anim.value.duration) || 0
  let start = Math.max(0, Number(playerPos.value) || 0)
  let end = start + POSE_CLIP_SECONDS
  if (dur > 0) {
    end = Math.min(end, dur)
    // Playhead parked at the very end: lay the clip out backwards rather
    // than collapsing it to nothing.
    if (end - start < 1e-6) start = Math.max(0, end - POSE_CLIP_SECONDS)
  }
  const key = (time) => ({ time, value: 1, interp: 1,
                           arrive_tangent: 0, leave_tangent: 0 })
  anim.value.value_tracks.push({
    id, name: label,
    target_kind: 'pose',
    target: [payload.pose_id],
    curve: { name: label, keys: [key(start), key(end)] },
  })
  animations.markDirty()
  selection.value = { kind: 'track', trackId: id }
  await ensurePosesLoaded([payload.pose_id])
  driveUrdfFromPlayhead()
}

// ── Per-joint overrides on a pose track ────────────────────────────
//
// A pose track's disclosed joint rows are keyable: the anchors at the
// pose's own keyframe times are locked, and the operator's keys go
// between them. Stored as `joint_overrides[joint]` on the track; the
// anchors are derived at resolve time so retiming the pose carries them
// along instead of stranding a stale copy.

function findTrack (trackId) {
  return anim.value?.value_tracks?.find(t => t.id === trackId) || null
}

function overrideKeys (track, joint, { create = false } = {}) {
  if (!track) return null
  if (!track.joint_overrides) {
    if (!create) return null
    track.joint_overrides = {}
  }
  if (!track.joint_overrides[joint]) {
    if (!create) return null
    track.joint_overrides[joint] = { name: joint, keys: [] }
  }
  return track.joint_overrides[joint].keys
}

function addOverrideKey ({ trackId, joint, time, value }) {
  const track = findTrack(trackId)
  if (!track || !joint) return
  animations.snapshot({ force: true })
  const keys = overrideKeys(track, joint, { create: true })
  const insertAt = keys.findIndex(k => k.time > time)
  // Inherit the preceding key's easing so adding a point to an eased
  // stretch continues that curve instead of forcing a linear kink —
  // same rule as the main keyframe path.
  const prevIdx = insertAt === -1 ? keys.length - 1 : insertAt - 1
  const key = {
    time, value: Number(value) || 0,
    interp: prevIdx >= 0 ? (keys[prevIdx].interp ?? 1) : 1,
    arrive_tangent: 0, leave_tangent: 0,
  }
  if (insertAt === -1) keys.push(key)
  else keys.splice(insertAt, 0, key)
  animations.markDirty()
  selection.value = { kind: 'override-keyframe', trackId, joint, time }
  driveUrdfFromPlayhead()
}

// One handler for both axes of the drag: horizontal retimes, vertical
// changes value. `value` is optional so a caller that only wants to
// retime doesn't have to know the current value.
function moveOverrideKey ({ trackId, joint, fromTime, toTime, value }) {
  const keys = overrideKeys(findTrack(trackId), joint)
  if (!keys) return
  const k = keys.find(x => Math.abs(x.time - fromTime) < 1e-6)
  if (!k) return
  k.time = toTime
  if (value !== undefined && Number.isFinite(Number(value))) {
    k.value = Number(value)
  }
  keys.sort((a, b) => a.time - b.time)
  animations.markDirty()
  // Keep the selection pinned to the key as it moves, so the Properties
  // panel doesn't blink out mid-drag.
  selection.value = { kind: 'override-keyframe', trackId, joint, time: toTime }
  driveUrdfFromPlayhead()
}

function removeOverrideKey ({ trackId, joint, time }) {
  const track = findTrack(trackId)
  const keys = overrideKeys(track, joint)
  if (!keys) return
  const idx = keys.findIndex(x => Math.abs(x.time - time) < 1e-6)
  if (idx < 0) return
  animations.snapshot({ force: true })
  keys.splice(idx, 1)
  // Drop the whole override once its last key goes, so the joint returns
  // to being driven by the pose blend rather than by an empty curve that
  // still counts as "overridden".
  if (!keys.length) {
    delete track.joint_overrides[joint]
    if (!Object.keys(track.joint_overrides).length) delete track.joint_overrides
  }
  if (selection.value?.kind === 'override-keyframe') {
    selection.value = { kind: null }
  }
  animations.markDirty()
  driveUrdfFromPlayhead()
}

// Move a value track within the list. Order is semantic: tracks layer
// bottom-up, so this changes the resolved output, not just the display.
function reorderTracks ({ from, to }) {
  const tracks = anim.value?.value_tracks
  if (!tracks) return
  if (from === to || from < 0 || to < 0 ||
      from >= tracks.length || to >= tracks.length) return
  animations.snapshot({ force: true })
  const [moved] = tracks.splice(from, 1)
  tracks.splice(to, 0, moved)
  animations.markDirty()
  driveUrdfFromPlayhead()
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

// ── Control rig ────────────────────────────────────────────────────
//
// The rig panel evaluates server-side and hands back resolved joint
// values. Those go straight to the viewer so the operator sees the pose
// as they drag — but they are NOT keyframes until asked for, so a rig
// control can be used to explore without dirtying the animation.

const rigPanelOpen = ref(true)
const rigJoints = ref({})
const rigPanelRef = ref(null)
// The 3D shapes are the primary interface; the side panel is the precise
// one. Both drive the same evaluate call, so they can't disagree.
const rigOverlay = ref(true)

// The viewer owns the meshes, so it needs the parsed rig and the anchor
// map to build them. Deferred until the viewer exists — the panel can
// finish loading before the async URDFViewer chunk has mounted.
let pendingRig = null
function onRigLoaded (rig, anchors) {
  pendingRig = { rig, anchors }
  applyRigToViewer()
}
function applyRigToViewer () {
  const v = viewerRef?.value
  if (!v?.setRigControls || !pendingRig) return
  v.setRigControls(pendingRig.rig, pendingRig.anchors, rigInert.value)
  v.setRigVisible(rigOverlay.value)
}
watch(rigOverlay, (on) => viewerRef?.value?.setRigVisible?.(on))

// Grabbing a shape in the viewport routes through the panel so the value
// math lives in exactly one place.
function onRigPress (name) {
  rigPanelRef.value?.beginOverlayDrag?.(name)
  viewerRef?.value?.highlightRigControl?.(name)
}
function onRigDrag (payload) {
  rigPanelRef.value?.applyOverlayDrag?.(payload)
}
function onRigRelease () {
  rigPanelRef.value?.endOverlayDrag?.()
  viewerRef?.value?.highlightRigControl?.(null)
}
// Hovering a panel row lights up its shape — how you find one control's
// handle among a dozen on a busy model.
function onRigHover (name) {
  viewerRef?.value?.highlightRigControl?.(name || null)
}
// Controls the evaluator can't handle (today: gaze). Their shapes are
// drawn as dim wireframes and taken out of hit-testing, so a drag falls
// through to the camera instead of dead-ending on an inert handle.
const rigInert = ref([])
function onRigInert (names) {
  rigInert.value = names || []
  viewerRef?.value?.setRigInert?.(rigInert.value)
}

// Shapes are parented to URDF links, so a robot reload destroys them and
// they have to be rebuilt against the new tree.
function onViewerLoaded () {
  requestCollisionScan()
  applyRigToViewer()
}

function onRigJoints (joints) {
  rigJoints.value = joints || {}
  const v = viewerRef?.value
  if (!v?.setJointValue) return
  for (const [jointName, val] of Object.entries(rigJoints.value)) {
    v.setJointValue(jointName, val)
  }
  scheduleLiveCollision()
}

// Commit the rig's current pose as keyframes at the playhead — one per
// joint the controls are driving. This is the bridge between posing with
// a rig and authoring a timeline: the rig is the input device, and the
// tracks stay the animation's source of truth (a saved animation must
// play back without the rig file present).
function onRigKeyFrame () {
  if (!anim.value) return
  const joints = Object.entries(rigJoints.value)
  if (!joints.length) return
  animations.snapshot({ force: true })
  for (const [jointName, value] of joints) {
    setKeyframeAtPlayhead(jointName, value)
  }
}

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
  // Unsaved edits ride along so playback matches the timeline on screen
  // rather than the last save. The server plays the copy without saving.
  const draft = animations.dirty ? anim.value : null
  await animations.start(anim.value.id, anim.value.loop, draft)
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
watch(editingId, async () => {
  playerPos.value = 0
  selection.value = { kind: null }
  await ensurePosesLoaded(referencedPoseIds(anim.value || {}))
  driveUrdfFromPlayhead()
})
// A track pointing at a pose we haven't fetched yet contributes nothing,
// so any change to the referenced set triggers a fetch. Covers undo/redo
// reinstating a pose track as well as adding one.
watch(() => referencedPoseIds(anim.value || {}).join('|'), async (ids) => {
  if (!ids) return
  await ensurePosesLoaded(ids.split('|'))
  driveUrdfFromPlayhead()
})

// ── Lifecycle ──────────────────────────────────────────────────────

onMounted(async () => {
  window.addEventListener('keydown', onKeyDown)
  document.addEventListener('focusin', onFocusIn)
  // Defer the collision scan on ANY interaction anywhere in the editor —
  // controls, fields, Save, timeline, 3D view. Capture phase so we still see
  // events that child handlers stopPropagation on (e.g. the gizmo).
  document.addEventListener('pointerdown', onActivityDown, true)
  document.addEventListener('pointermove', onActivityMove, true)
  document.addEventListener('pointerup', onActivityUp, true)
  document.addEventListener('wheel', onActivity, { capture: true, passive: true })
  document.addEventListener('keydown', onActivity, true)
  document.addEventListener('input', onActivity, true)
  document.addEventListener('change', onActivity, true)
  await Promise.all([robot.refresh(), animations.reload(), poses.reload(),
                     playlists.reload(), sounds.reload(), loadRigNeutral()])
  // Load the WS-input catalog up front so the timeline's "+ Input"
  // dropdown is populated immediately — value tracks can bind a
  // controller sheet input even when no URDF is installed.
  loadTriggerTargets()
  await loadFromRoute()
  // Pose tracks can't drive anything until their joint values are in,
  // so fetch them before the first frame rather than showing a pose
  // track that visibly pops in a moment later.
  await ensurePosesLoaded(referencedPoseIds(anim.value || {}))
  // Land at t=0 with the URDF reflecting any saved keyframes.
  playerPos.value = 0
  driveUrdfFromPlayhead()
})
onBeforeUnmount(() => {
  window.removeEventListener('keydown', onKeyDown)
  document.removeEventListener('focusin', onFocusIn)
  document.removeEventListener('pointerdown', onActivityDown, true)
  document.removeEventListener('pointermove', onActivityMove, true)
  document.removeEventListener('pointerup', onActivityUp, true)
  document.removeEventListener('wheel', onActivity, { capture: true })
  document.removeEventListener('keydown', onActivity, true)
  document.removeEventListener('input', onActivity, true)
  document.removeEventListener('change', onActivity, true)
  clearTimeout(_liveCollisionTimer)
  clearTimeout(_idleTimer)
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
                        @loaded="onViewerLoaded"
                        @rig-control-press="onRigPress"
                        @rig-control-drag="onRigDrag"
                        @rig-control-release="onRigRelease"
                        @interact="deferCollisionScan"
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
          <!-- Live self-collision badge at the current playhead pose. -->
          <div v-if="robot.installed && liveCollisions.length"
               class="absolute top-2 left-2 z-10 flex items-center gap-1 rounded bg-red-600/90 text-white text-xs px-2 py-1 pointer-events-none"
               :title="liveCollisions.join(', ')">
            <span class="material-icons" style="font-size:14px">warning</span>
            Collision
          </div>
          <!-- A pose track whose pose was deleted contributes nothing.
               Silently doing nothing is the hardest animation bug to
               find, so say so. -->
          <div v-if="missingPoses.length"
               class="absolute bottom-2 left-2 z-10 flex items-center gap-1 rounded bg-amber-600/90 text-white text-xs px-2 py-1"
               :title="`Missing pose(s): ${missingPoses.join(', ')} — these tracks do nothing until the pose is restored or the track removed.`">
            <span class="material-icons" style="font-size:14px">help_outline</span>
            {{ missingPoses.length }} missing pose{{ missingPoses.length === 1 ? '' : 's' }}
          </div>
        </div>

        <!-- Rig controls: the input device for posing. Sits between the
             viewport and the properties panel because it's used WITH the
             3D view, not read like a form. Collapsible — a rig with a
             dozen controls would otherwise crowd out the viewport. -->
        <aside v-if="anim && robot.hasRig" class="editor-rig"
               :class="{ 'is-collapsed': !rigPanelOpen }">
          <button class="editor-rig-toggle"
                  :title="rigPanelOpen ? 'Collapse the control rig' : 'Expand the control rig'"
                  @click="rigPanelOpen = !rigPanelOpen">
            <span class="material-icons icon-sm">
              {{ rigPanelOpen ? 'chevron_right' : 'tune' }}
            </span>
          </button>
          <div v-if="rigPanelOpen" class="editor-rig-body">
            <RigControls ref="rigPanelRef"
                         :live="livePreview"
                         v-model:overlay="rigOverlay"
                         @joints="onRigJoints"
                         @key-frame="onRigKeyFrame"
                         @rig-loaded="onRigLoaded"
                         @hover-control="onRigHover"
                         @inert-controls="onRigInert" />
          </div>
        </aside>

        <aside v-if="anim" class="editor-props">
          <div class="editor-props-header">Properties</div>
          <div class="editor-props-body">
            <PropsPanel :animation="anim"
                        :selection="selection"
                        @dirty="animations.markDirty()"
                        @select="onPropsSelect"
                        @delete-keyframe="onDeleteKeyframe"
                        @delete-override-key="removeOverrideKey" />
          </div>
        </aside>
      </div>

      <!-- Timeline — full page width below the preview row. -->
      <div class="editor-timeline">
        <TimelineEditor v-if="anim"
                        :animation="anim"
                        :player-pos="playerPos"
                        :collision-intervals="collisionIntervals"
                        :selection="selection"
                        :playing="!!playingState?.running"
                        :unbound-joints="unboundJoints"
                        :ws-inputs="wsInputCatalog"
                        :poses="poses.list"
                        :sounds="sounds.list"
                        :animations="animations.list"
                        :pose-playlists="posePlaylistNames"
                        :pose-joints="poseJoints"
                        :pose-setpoints="poseSetpoints"
                        :resolved-joints="resolvedJoints"
                        @update:player-pos="onTimelineScrub"
                        @select="onTimelineSelect"
                        @dirty="animations.markDirty()"
                        @rename-track="renameTrack"
                        @remove-track="removeTrack"
                        @reorder-tracks="reorderTracks"
                        @rename-trigger-track="renameTriggerTrack"
                        @remove-trigger-track="removeTriggerTrack"
                        @play="play"
                        @stop="stopPlayback"
                        @add-joint="addJointTrack"
                        @add-ws-input="addWsInputTrack"
                        @add-pose="addPoseTrack"
                        @add-board-item="addBoardItemTrack"
                        @add-override-key="addOverrideKey"
                        @move-override-key="moveOverrideKey"
                        @remove-override-key="removeOverrideKey" />
      </div>
    </div>
  </section>
</template>
