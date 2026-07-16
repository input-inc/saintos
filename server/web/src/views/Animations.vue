<script setup>
import { computed, defineAsyncComponent, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRouter } from 'vue-router'
import { useAnimationsStore } from '@/stores/animations'
import { usePosesStore } from '@/stores/poses'
import { useSoundsStore } from '@/stores/sounds'
import { useWsStore } from '@/stores/ws'

// Full-screen "Boards" management. Sidebar lists animations, poses, and
// sounds (each with their groups) and a "New" button per kind opens a
// modal collecting base info; submitting routes into the editor (for
// animations) or reloads the list (poses/sounds).

const NewAnimationModal = defineAsyncComponent(
  () => import('@/components/animation/NewAnimationModal.vue'))
const NewPoseModal = defineAsyncComponent(
  () => import('@/components/animation/NewPoseModal.vue'))
const NewSoundModal = defineAsyncComponent(
  () => import('@/components/animation/NewSoundModal.vue'))
const AddSoundsFromFolderModal = defineAsyncComponent(
  () => import('@/components/animation/AddSoundsFromFolderModal.vue'))
const MaestroImportModal = defineAsyncComponent(
  () => import('@/components/animation/MaestroImportModal.vue'))

const router = useRouter()
const animations = useAnimationsStore()
const poses = usePosesStore()
const sounds = useSoundsStore()
const ws = useWsStore()

// Sidebar selection: which kind is highlighted, and which group
// within it. group=null is the "All" bucket for that kind.
const view = ref({ kind: 'animations', group: null })
function selectView (kind, group) { view.value = { kind, group } }

// Modals
const newAnimOpen = ref(false)
const newPoseOpen = ref(false)
const newSoundOpen = ref(false)
const newSoundFolderOpen = ref(false)
const importOpen = ref(false)

// Audio-capable nodes, for resolving a sound's node_id to a label in
// the list. Loaded on mount + refreshed when the sound modal opens.
const audioNodes = ref([])
function nodeName (id) {
  const n = audioNodes.value.find(x => x.node_id === id)
  return n ? n.name : (id || '—')
}

// Groups derived from the lists (sorted).
const animationGroups = computed(() => {
  const set = new Set()
  for (const a of animations.list || []) if (a.group) set.add(a.group)
  return [...set].sort()
})
const poseGroups = computed(() => {
  const set = new Set()
  for (const p of poses.list || []) if (p.group) set.add(p.group)
  return [...set].sort()
})
const soundGroups = computed(() => {
  const set = new Set()
  for (const s of sounds.list || []) if (s.group) set.add(s.group)
  return [...set].sort()
})

// Filtered lists for the main pane.
function inGroup (item, g) {
  if (g === null) return true                     // "All"
  if (g === '__ungrouped__') return !item.group
  return (item.group || '') === g
}
const visibleAnimations = computed(() =>
  (animations.list || []).filter(a => inGroup(a, view.value.group))
                          .sort((a, b) => (a.name || '').localeCompare(b.name || ''))
)
const visiblePoses = computed(() =>
  (poses.list || []).filter(p => inGroup(p, view.value.group))
                    .sort((a, b) => (a.name || '').localeCompare(b.name || ''))
)
// Sounds keep the operator's explicit order (position); the store
// already returns them sorted by (group, position, name).
const visibleSounds = computed(() =>
  (sounds.list || []).filter(s => inGroup(s, view.value.group))
                     .sort((a, b) => (a.position || 0) - (b.position || 0)
                                     || (a.name || '').localeCompare(b.name || ''))
)

// Title for the main toolbar.
const viewTitle = computed(() => {
  const kindLabel = view.value.kind === 'animations'
    ? 'Animations' : (view.value.kind === 'poses' ? 'Poses' : 'Sounds')
  const g = view.value.group
  if (g === null) return `All ${kindLabel}`
  if (g === '__ungrouped__') return `${kindLabel} / Ungrouped`
  return `${kindLabel} / ${g}`
})

// ── CRUD via the WS layer ───────────────────────────────────────────

async function onCreateAnimation (payload) {
  newAnimOpen.value = false
  const r = await ws.management('save_animation', {
    animation: { id: '', ...payload, value_tracks: [], trigger_tracks: [] },
  })
  await animations.reload()
  const id = r?.animation?.id
  if (id) router.push({ name: 'animation-editor', params: { id } })
}

async function onCreatePose (payload) {
  newPoseOpen.value = false
  const r = await ws.management('save_pose', {
    pose: { id: '', ...payload, setpoints: [] },
  })
  await poses.reload()
  if (r?.pose?.id) {
    // Load it for inline editing; switch sidebar to the right group.
    await poses.load(r.pose.id)
    selectView('poses', payload.group || '__ungrouped__')
    editingPoseId.value = r.pose.id
  }
}

async function onCreateSound (payload) {
  newSoundOpen.value = false
  await sounds.save({ id: '', position: 0, ...payload })
  selectView('sounds', payload.group || '__ungrouped__')
}

async function onCreateSoundFolder (payload) {
  newSoundFolderOpen.value = false
  soundError.value = ''
  soundInfo.value = 'Adding sounds from folder…'
  const r = await sounds.bulkAddFromFolder(payload.node_id, payload.folder, {
    output_device: payload.output_device,
    group: payload.group,
    volume: payload.volume,
    loop: payload.loop,
    loop_count: payload.loop_count,
  })
  selectView('sounds', payload.group || '__ungrouped__')
  if (r) {
    soundInfo.value = `Added ${r.added} sound${r.added === 1 ? '' : 's'}`
      + (r.skipped ? `, skipped ${r.skipped} already added` : '')
      + ` (${r.scanned} audio file${r.scanned === 1 ? '' : 's'} scanned).`
  } else {
    soundInfo.value = ''
    soundError.value = sounds.error || 'Failed to add sounds from folder'
  }
}

// Inline editing for the row name field — click name to flip to input.
const renamingId = ref(null)
async function patchAnimation (id, patch) {
  const r = await ws.management('get_animation', { id })
  const anim = r?.animation
  if (!anim) return
  Object.assign(anim, patch)
  await ws.management('save_animation', { animation: anim })
  await animations.reload()
}
async function patchPose (id, patch) {
  const r = await ws.management('get_pose', { id })
  const pose = r?.pose
  if (!pose) return
  Object.assign(pose, patch)
  await ws.management('save_pose', { pose })
  await poses.reload()
}
async function patchSound (id, patch) {
  const r = await ws.management('get_sound', { id })
  const sound = r?.sound
  if (!sound) return
  Object.assign(sound, patch)
  await sounds.save(sound)
}

async function deleteAnimation (a) {
  if (!confirm(`Delete "${a.name || a.id}"? This can't be undone.`)) return
  await animations.remove(a.id)
}
async function deletePose (p) {
  if (!confirm(`Delete "${p.name || p.id}"? This can't be undone.`)) return
  await poses.remove(p.id)
}
async function deleteSound (s) {
  if (!confirm(`Delete "${s.name || s.id}"? This can't be undone.`)) return
  await sounds.remove(s.id)
}

// Play/stop straight from the list. Playback is stop-and-replace per
// node, so there's only ever one voice per node — `playingId` is the
// last sound we asked to play (optimistic; there's no continuous
// playback telemetry from the node). The play command's ok/error ack
// returns async on soundboard_result but we surface only failures.
const playingId = ref(null)
const soundError = ref('')
const soundInfo = ref('')
async function playSound (s) {
  soundError.value = ''
  playingId.value = s.id
  const r = await sounds.play(s.id)
  if (r == null && sounds.error) soundError.value = sounds.error
}
async function stopSound (s) {
  if (playingId.value === s.id) playingId.value = null
  await sounds.stop(s.node_id)
}

// Reorder within a group via up/down. Sounds are contiguous per group in
// the store's (group, position, name) sort, so swapping adjacent
// same-group entries in the global list and re-sending every id keeps
// positions globally consistent while only moving within the group.
async function moveSound (s, dir) {
  const arr = [...(sounds.list || [])]
  const i = arr.findIndex(x => x.id === s.id)
  const j = dir === 'up' ? i - 1 : i + 1
  if (i < 0 || j < 0 || j >= arr.length) return
  if ((arr[j].group || '') !== (s.group || '')) return   // don't cross groups
  ;[arr[i], arr[j]] = [arr[j], arr[i]]
  await sounds.reorder(arr.map(x => x.id))
}

async function duplicateAnimation (a) {
  const r = await ws.management('get_animation', { id: a.id })
  const src = r?.animation
  if (!src) return
  const copy = JSON.parse(JSON.stringify(src))
  copy.id = ''
  copy.name = `${src.name || src.id} (copy)`
  copy.created = ''
  copy.modified = ''
  await ws.management('save_animation', { animation: copy })
  await animations.reload()
}

async function exportAnimation (a) {
  const r = await ws.management('get_animation', { id: a.id })
  const anim = r?.animation
  if (!anim) return
  const blob = new Blob([JSON.stringify(anim, null, 2)],
                        { type: 'application/json' })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.download = `${anim.id || 'animation'}.json`
  link.click()
  setTimeout(() => URL.revokeObjectURL(url), 500)
}

function openAnimation (a) {
  router.push({ name: 'animation-editor', params: { id: a.id } })
}

// Pose detail inline editor (expanded row). Loads/saves through the
// existing pose store + WS surface; setpoint editing stays in the
// dedicated pose-detail panel for now.
const editingPoseId = ref(null)
async function startEditPose (p) {
  if (editingPoseId.value === p.id) {
    editingPoseId.value = null
    return
  }
  await Promise.all([poses.load(p.id), loadWsInputs()])
  editingPoseId.value = p.id
}
async function saveEditedPose () {
  await poses.save()
  editingPoseId.value = null
}
async function applyPose (p) {
  await poses.apply(p.id)
}
// Push the in-progress edit live without saving — lets the operator
// A/B it against the saved pose (the row's play button) before saving.
const previewing = ref(false)
async function previewEditedPose () {
  previewing.value = true
  try {
    await poses.preview()
  } finally {
    previewing.value = false
  }
}

// ── Pose tracks (setpoints) ────────────────────────────────────────
// Each track binds one WS input on a controller node's routing sheet
// (sheet_id, ws_input_id). Applying the pose pushes the value into that
// sheet's routing graph, where it routes to the controller like any
// other input. Sourced from the SAME server endpoint
// PoseLibrary.vue / the controller's binding picker hit —
// `list_websocket_inputs` on the router channel. The earlier
// `ws.management('list_ws_inputs', …)` shape silently returned
// "Unknown action" (no management handler exists by that name); the
// catch below swallowed the error, wsInputs stayed empty forever, and
// the operator-visible result was "Add track" disabled + the "No
// controller inputs available" amber line even when there WERE WS-
// input nodes on a sheet. Same fix landed in AnimationEditorView's
// loadTriggerTargets.
const wsInputs = ref([])
async function loadWsInputs () {
  try {
    const r = await ws.router('list_websocket_inputs', {})
    wsInputs.value = (r?.ws_inputs || r?.inputs || []).filter(x => x.kind !== 'state')
  } catch (e) { console.warn('list_websocket_inputs failed:', e) }
}
const wsSheets = computed(() => {
  const seen = new Map()
  for (const w of wsInputs.value) {
    if (!seen.has(w.sheet_id)) seen.set(w.sheet_id, w.sheet_label || w.sheet_id)
  }
  return [...seen.entries()].map(([id, label]) => ({ id, label }))
})
function wsInputsForSheet (sheetId) {
  return wsInputs.value.filter(w => w.sheet_id === sheetId)
}
function addPoseTrack () {
  if (!poses.editing) return
  if (!Array.isArray(poses.editing.setpoints)) poses.editing.setpoints = []
  const first = wsInputs.value[0]
  poses.editing.setpoints.push({
    sheet_id: first?.sheet_id || '',
    ws_input_id: first?.input_id || '',
    value: 0,
  })
  poses.markDirty()
}
function removePoseTrack (idx) {
  poses.editing.setpoints.splice(idx, 1)
  poses.markDirty()
}

function fmtTimeAgo (iso) {
  if (!iso) return '—'
  const dt = (Date.now() - new Date(iso).getTime()) / 1000
  if (dt < 60) return 'just now'
  if (dt < 3600) return `${Math.floor(dt / 60)}m ago`
  if (dt < 86400) return `${Math.floor(dt / 3600)}h ago`
  return `${Math.floor(dt / 86400)}d ago`
}

// Play / stop straight from the list, no editor round-trip. `isPlaying`
// reads the live player set (refreshed on start/stop and by the poll
// below, so a non-loop animation that runs to its end flips the button
// back to ▶ on its own).
function animIsPlaying (id) {
  return animations.players.some(p => p.id === id && p.running)
}
async function togglePlayAnimation (a) {
  if (animIsPlaying(a.id)) await animations.stop(a.id)
  else await animations.start(a.id)
}

// Poll the player set while this screen is mounted so list play/stop
// buttons reflect playback started elsewhere AND auto-revert when a
// non-looping animation finishes. 1.5 s is responsive enough for a
// transport indicator without hammering the management channel.
let _playerPoll = null
onMounted(async () => {
  await Promise.all([animations.reload(), poses.reload(), sounds.reload(), loadWsInputs()])
  animations.refreshPlayers()
  _playerPoll = setInterval(() => animations.refreshPlayers(), 1500)
  sounds.listNodes().then(n => { audioNodes.value = n })
})
onBeforeUnmount(() => {
  if (_playerPoll) { clearInterval(_playerPoll); _playerPoll = null }
})

// If the sidebar selection lands on an empty bucket (e.g. the last
// item in that group was just deleted), bounce to the "All" view of
// the same kind so the main pane never looks broken-empty.
watch([animationGroups, poseGroups, soundGroups], () => {
  if (view.value.group === null) return
  const list = view.value.kind === 'animations'
    ? animationGroups.value
    : (view.value.kind === 'poses' ? poseGroups.value : soundGroups.value)
  if (view.value.group !== '__ungrouped__' && !list.includes(view.value.group)) {
    view.value = { kind: view.value.kind, group: null }
  }
})
</script>

<template>
  <section class="page animations-page">
    <div class="animations-shell">
      <!-- Sidebar -->
      <aside class="animations-sidebar">
        <div class="p-2 space-y-1 border-b border-line/40">
          <button class="btn-sm w-full bg-cyan-600 hover:bg-cyan-500 text-fg-strong justify-center"
                  @click="newAnimOpen = true">
            <span class="material-icons icon-sm">add</span>
            New Animation
          </button>
          <button class="btn-sm w-full bg-emerald-600/80 hover:bg-emerald-500 text-fg-strong justify-center"
                  @click="newPoseOpen = true">
            <span class="material-icons icon-sm">add</span>
            New Pose
          </button>
          <button class="btn-sm w-full bg-violet-600/80 hover:bg-violet-500 text-fg-strong justify-center"
                  @click="newSoundOpen = true">
            <span class="material-icons icon-sm">add</span>
            New Sound
          </button>
          <button class="btn-sm w-full bg-violet-600/60 hover:bg-violet-500 text-fg-strong justify-center"
                  @click="newSoundFolderOpen = true">
            <span class="material-icons icon-sm">create_new_folder</span>
            Sounds from Folder
          </button>
          <button class="btn-sm w-full bg-surface hover:bg-surface-2 text-fg-strong justify-center"
                  @click="importOpen = true">
            <span class="material-icons icon-sm">file_upload</span>
            Import Maestro
          </button>
        </div>

        <div class="animations-sidebar-list">
          <!-- Animations section -->
          <div class="px-3 pt-3 pb-1 text-[10px] uppercase tracking-wide text-fg-faint">Animations</div>
          <div :class="['animations-sidebar-item', view.kind === 'animations' && view.group === null ? 'active' : '']"
               @click="selectView('animations', null)">
            <span class="material-icons icon-sm">animation</span>
            <span class="flex-1 truncate">All Animations</span>
            <span class="text-[10px] text-fg-faint tabular-nums">{{ animations.list.length }}</span>
          </div>
          <div v-for="g in animationGroups" :key="`a-${g}`"
               :class="['animations-sidebar-item', view.kind === 'animations' && view.group === g ? 'active' : '']"
               @click="selectView('animations', g)">
            <span class="material-icons icon-sm">folder</span>
            <span class="flex-1 truncate">{{ g }}</span>
            <span class="text-[10px] text-fg-faint tabular-nums">
              {{ animations.list.filter(a => a.group === g).length }}
            </span>
          </div>
          <div v-if="(animations.list || []).some(a => !a.group)"
               :class="['animations-sidebar-item', view.kind === 'animations' && view.group === '__ungrouped__' ? 'active' : '']"
               @click="selectView('animations', '__ungrouped__')">
            <span class="material-icons icon-sm">folder_open</span>
            <span class="flex-1 truncate text-fg-muted italic">Ungrouped</span>
            <span class="text-[10px] text-fg-faint tabular-nums">
              {{ animations.list.filter(a => !a.group).length }}
            </span>
          </div>

          <!-- Poses section -->
          <div class="px-3 pt-4 pb-1 text-[10px] uppercase tracking-wide text-fg-faint border-t border-line/40 mt-2">
            Poses
          </div>
          <div :class="['animations-sidebar-item', view.kind === 'poses' && view.group === null ? 'active' : '']"
               @click="selectView('poses', null)">
            <span class="material-icons icon-sm">accessibility</span>
            <span class="flex-1 truncate">All Poses</span>
            <span class="text-[10px] text-fg-faint tabular-nums">{{ poses.list.length }}</span>
          </div>
          <div v-for="g in poseGroups" :key="`p-${g}`"
               :class="['animations-sidebar-item', view.kind === 'poses' && view.group === g ? 'active' : '']"
               @click="selectView('poses', g)">
            <span class="material-icons icon-sm">folder</span>
            <span class="flex-1 truncate">{{ g }}</span>
            <span class="text-[10px] text-fg-faint tabular-nums">
              {{ poses.list.filter(p => p.group === g).length }}
            </span>
          </div>
          <div v-if="(poses.list || []).some(p => !p.group)"
               :class="['animations-sidebar-item', view.kind === 'poses' && view.group === '__ungrouped__' ? 'active' : '']"
               @click="selectView('poses', '__ungrouped__')">
            <span class="material-icons icon-sm">folder_open</span>
            <span class="flex-1 truncate text-fg-muted italic">Ungrouped</span>
            <span class="text-[10px] text-fg-faint tabular-nums">
              {{ poses.list.filter(p => !p.group).length }}
            </span>
          </div>

          <!-- Sounds section -->
          <div class="px-3 pt-4 pb-1 text-[10px] uppercase tracking-wide text-fg-faint border-t border-line/40 mt-2">
            Sounds
          </div>
          <div :class="['animations-sidebar-item', view.kind === 'sounds' && view.group === null ? 'active' : '']"
               @click="selectView('sounds', null)">
            <span class="material-icons icon-sm">volume_up</span>
            <span class="flex-1 truncate">All Sounds</span>
            <span class="text-[10px] text-fg-faint tabular-nums">{{ sounds.list.length }}</span>
          </div>
          <div v-for="g in soundGroups" :key="`s-${g}`"
               :class="['animations-sidebar-item', view.kind === 'sounds' && view.group === g ? 'active' : '']"
               @click="selectView('sounds', g)">
            <span class="material-icons icon-sm">folder</span>
            <span class="flex-1 truncate">{{ g }}</span>
            <span class="text-[10px] text-fg-faint tabular-nums">
              {{ sounds.list.filter(s => s.group === g).length }}
            </span>
          </div>
          <div v-if="(sounds.list || []).some(s => !s.group)"
               :class="['animations-sidebar-item', view.kind === 'sounds' && view.group === '__ungrouped__' ? 'active' : '']"
               @click="selectView('sounds', '__ungrouped__')">
            <span class="material-icons icon-sm">folder_open</span>
            <span class="flex-1 truncate text-fg-muted italic">Ungrouped</span>
            <span class="text-[10px] text-fg-faint tabular-nums">
              {{ sounds.list.filter(s => !s.group).length }}
            </span>
          </div>
        </div>
      </aside>

      <!-- Main pane -->
      <div class="animations-main">
        <div class="animations-toolbar">
          <div class="flex-1 min-w-0">
            <h2 class="text-base font-semibold text-fg-strong truncate">{{ viewTitle }}</h2>
            <p class="text-xs text-fg-muted">
              {{ view.kind === 'animations' ? visibleAnimations.length
                 : (view.kind === 'poses' ? visiblePoses.length : visibleSounds.length) }}
              {{ view.kind === 'animations' ? 'animation'
                 : (view.kind === 'poses' ? 'pose' : 'sound') }}{{
                (view.kind === 'animations' ? visibleAnimations.length
                 : (view.kind === 'poses' ? visiblePoses.length : visibleSounds.length)) === 1 ? '' : 's' }}
            </p>
          </div>
        </div>

        <div class="flex-1 overflow-y-auto p-4 space-y-3">
          <!-- Animations table -->
          <template v-if="view.kind === 'animations'">
            <div v-if="!visibleAnimations.length" class="text-center text-sm text-fg-faint py-10">
              No animations in this group yet. Click <span class="text-cyan-300">New Animation</span> to start.
            </div>
            <div v-else class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40">
              <div v-for="a in visibleAnimations" :key="a.id"
                   class="anim-row flex items-center gap-3 px-3 py-2 hover:bg-panel/60 transition-colors">
                <button class="anim-icon-cell" title="Open in editor" @click="openAnimation(a)">
                  <span class="material-icons">{{ a.icon || 'animation' }}</span>
                </button>
                <div class="flex-1 min-w-0">
                  <input v-if="renamingId === a.id"
                         class="input-field w-full text-sm py-1"
                         :value="a.name"
                         autofocus
                         @blur="(e) => { patchAnimation(a.id, { name: e.target.value }); renamingId = null }"
                         @keydown.enter="(e) => e.target.blur()"
                         @keydown.escape="renamingId = null" />
                  <div v-else
                       class="text-sm font-medium text-fg-strong truncate cursor-text"
                       @click="renamingId = a.id">
                    {{ a.name || a.id }}
                  </div>
                  <div class="text-xs text-fg-faint truncate font-mono">{{ a.id }}</div>
                </div>
                <input class="input-field text-xs py-1 w-32"
                       list="anim-group-suggest"
                       :value="a.group || ''"
                       placeholder="(group)"
                       @change="(e) => patchAnimation(a.id, { group: e.target.value })" />
                <div class="text-xs text-fg-muted tabular-nums w-20 text-right shrink-0">
                  {{ Number(a.duration || 0).toFixed(2) }}s
                </div>
                <div class="text-xs text-fg-faint w-20 text-right tabular-nums shrink-0">
                  {{ a.value_tracks }}v · {{ a.trigger_tracks }}t
                </div>
                <div class="text-xs text-fg-faint w-20 text-right shrink-0">{{ fmtTimeAgo(a.modified) }}</div>
                <div class="flex items-center gap-1 shrink-0">
                  <button :class="['btn-sm text-fg-strong',
                                   animIsPlaying(a.id)
                                     ? 'bg-red-600/80 hover:bg-red-500'
                                     : 'bg-emerald-500/80 hover:bg-emerald-500']"
                          :title="animIsPlaying(a.id) ? 'Stop' : 'Play'"
                          @click="togglePlayAnimation(a)">
                    <span class="material-icons icon-sm">{{ animIsPlaying(a.id) ? 'stop' : 'play_arrow' }}</span>
                  </button>
                  <button class="btn-sm bg-surface hover:bg-cyan-600 text-fg-strong hover:text-fg-strong"
                          title="Open in editor" @click="openAnimation(a)">
                    <span class="material-icons icon-sm">edit</span>
                  </button>
                  <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                          title="Duplicate" @click="duplicateAnimation(a)">
                    <span class="material-icons icon-sm">content_copy</span>
                  </button>
                  <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                          title="Export JSON" @click="exportAnimation(a)">
                    <span class="material-icons icon-sm">download</span>
                  </button>
                  <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                          title="Delete" @click="deleteAnimation(a)">
                    <span class="material-icons icon-sm">delete</span>
                  </button>
                </div>
              </div>
            </div>
          </template>

          <!-- Poses table -->
          <template v-else-if="view.kind === 'poses'">
            <div v-if="!visiblePoses.length" class="text-center text-sm text-fg-faint py-10">
              No poses in this group yet. Click <span class="text-cyan-300">New Pose</span> to start.
            </div>
            <div v-else class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40">
              <template v-for="p in visiblePoses" :key="p.id">
                <div class="anim-row flex items-center gap-3 px-3 py-2 hover:bg-panel/60 transition-colors">
                  <div class="anim-icon-cell">
                    <span class="material-icons">{{ p.icon || 'accessibility' }}</span>
                  </div>
                  <div class="flex-1 min-w-0">
                    <input v-if="renamingId === p.id"
                           class="input-field w-full text-sm py-1"
                           :value="p.name"
                           autofocus
                           @blur="(e) => { patchPose(p.id, { name: e.target.value }); renamingId = null }"
                           @keydown.enter="(e) => e.target.blur()"
                           @keydown.escape="renamingId = null" />
                    <div v-else class="text-sm font-medium text-fg-strong truncate cursor-text"
                         @click="renamingId = p.id">{{ p.name || p.id }}</div>
                    <div class="text-xs text-fg-faint truncate">{{ p.description || p.id }}</div>
                  </div>
                  <input class="input-field text-xs py-1 w-32"
                         list="pose-group-suggest"
                         :value="p.group || ''"
                         placeholder="(group)"
                         @change="(e) => patchPose(p.id, { group: e.target.value })" />
                  <div class="text-xs text-fg-faint w-24 text-right tabular-nums shrink-0">
                    {{ p.setpoint_count }} track{{ p.setpoint_count === 1 ? '' : 's' }}
                  </div>
                  <div class="text-xs text-fg-faint w-20 text-right shrink-0">{{ fmtTimeAgo(p.modified) }}</div>
                  <div class="flex items-center gap-1 shrink-0">
                    <button class="btn-sm bg-emerald-500/80 hover:bg-emerald-500 text-fg-strong"
                            title="Apply pose" @click="applyPose(p)">
                      <span class="material-icons icon-sm">play_arrow</span>
                    </button>
                    <button class="btn-sm bg-surface hover:bg-cyan-600 text-fg-strong hover:text-fg-strong"
                            title="Edit setpoints" @click="startEditPose(p)">
                      <span class="material-icons icon-sm">edit</span>
                    </button>
                    <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                            title="Delete" @click="deletePose(p)">
                      <span class="material-icons icon-sm">delete</span>
                    </button>
                  </div>
                </div>
                <!-- Expanded inline editor for setpoints -->
                <div v-if="editingPoseId === p.id && poses.editing"
                     class="px-4 py-3 bg-canvas/60 border-t border-line/30 space-y-2">
                  <div class="flex items-center justify-between">
                    <h4 class="text-sm font-semibold text-fg-strong">Edit pose: {{ poses.editing.name || poses.editing.id }}</h4>
                    <div class="flex gap-2">
                      <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong" @click="editingPoseId = null">
                        Done
                      </button>
                      <button class="btn-sm bg-emerald-600 hover:bg-emerald-500 text-fg-strong"
                              :disabled="previewing || !poses.editing.setpoints?.length"
                              title="Push this pose live without saving — compare it against the saved version with the row's play button"
                              @click="previewEditedPose">
                        <span class="material-icons icon-sm">play_arrow</span>
                        Preview
                      </button>
                      <button class="btn-sm bg-cyan-600 hover:bg-cyan-500 text-fg-strong"
                              :disabled="!poses.dirty" @click="saveEditedPose">
                        <span class="material-icons icon-sm">save</span>
                        Save
                      </button>
                    </div>
                  </div>
                  <label class="block">
                    <span class="block text-fg-muted text-xs mb-1">Description</span>
                    <input class="input-field w-full" v-model="poses.editing.description"
                           @input="poses.markDirty()" />
                  </label>

                  <!-- Tracks: each binds a WS input on a controller's
                       sheet; the value is pushed into the routing graph
                       on apply, routed like any other controller input. -->
                  <div>
                    <div class="flex items-center justify-between mb-1">
                      <span class="block text-fg-muted text-xs">Tracks</span>
                      <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                              :disabled="!wsInputs.length" @click="addPoseTrack">
                        <span class="material-icons icon-sm">add</span>
                        Add track
                      </button>
                    </div>
                    <p v-if="!wsInputs.length" class="text-xs text-amber-300 italic mb-1">
                      No controller inputs available — add WS-input nodes to a sheet in Routes first.
                    </p>
                    <ul v-if="poses.editing.setpoints?.length" class="space-y-2">
                      <li v-for="(sp, idx) in poses.editing.setpoints" :key="idx"
                          class="flex items-center gap-2">
                        <select class="input-field text-xs w-36 shrink-0" v-model="sp.sheet_id"
                                @change="poses.markDirty()">
                          <option v-for="s in wsSheets" :key="s.id" :value="s.id">{{ s.label }}</option>
                        </select>
                        <select class="input-field text-xs w-40 shrink-0" v-model="sp.ws_input_id"
                                @change="poses.markDirty()">
                          <option v-for="w in wsInputsForSheet(sp.sheet_id)"
                                  :key="w.input_id" :value="w.input_id">
                            {{ w.label || w.input_id }}
                          </option>
                        </select>
                        <input type="range" min="-1" max="1" step="0.01"
                               class="flex-1 accent-cyan-500 cursor-pointer"
                               v-model.number="sp.value" @input="poses.markDirty()" />
                        <span class="text-xs font-mono text-cyan-400 w-12 text-right tabular-nums">
                          {{ Number(sp.value).toFixed(2) }}
                        </span>
                        <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                                title="Remove track" @click="removePoseTrack(idx)">
                          <span class="material-icons icon-sm">delete</span>
                        </button>
                      </li>
                    </ul>
                    <p v-else class="text-xs text-fg-faint italic">
                      No tracks yet — add one to bind a controller input.
                    </p>
                  </div>
                </div>
              </template>
            </div>
          </template>

          <!-- Sounds table -->
          <template v-else>
            <div v-if="!visibleSounds.length" class="text-center text-sm text-fg-faint py-10">
              No sounds in this group yet. Click <span class="text-violet-300">New Sound</span> to start.
            </div>
            <div v-else class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40">
              <div v-for="(s, idx) in visibleSounds" :key="s.id"
                   class="anim-row flex items-center gap-3 px-3 py-2 hover:bg-panel/60 transition-colors">
                <div class="anim-icon-cell">
                  <span class="material-icons">{{ s.icon || 'volume_up' }}</span>
                </div>
                <div class="flex-1 min-w-0">
                  <input v-if="renamingId === s.id"
                         class="input-field w-full text-sm py-1"
                         :value="s.name"
                         autofocus
                         @blur="(e) => { patchSound(s.id, { name: e.target.value }); renamingId = null }"
                         @keydown.enter="(e) => e.target.blur()"
                         @keydown.escape="renamingId = null" />
                  <div v-else class="text-sm font-medium text-fg-strong truncate cursor-text"
                       @click="renamingId = s.id">{{ s.name || s.id }}</div>
                  <div class="text-xs text-fg-faint truncate font-mono" :title="s.file_path">
                    {{ nodeName(s.node_id) }} · {{ s.file_path || '(no file)' }}
                  </div>
                </div>
                <input class="input-field text-xs py-1 w-32"
                       list="sound-group-suggest"
                       :value="s.group || ''"
                       placeholder="(group)"
                       @change="(e) => patchSound(s.id, { group: e.target.value })" />
                <div class="text-xs text-fg-faint w-16 text-right tabular-nums shrink-0">
                  {{ Math.round((s.volume ?? 1) * 100) }}%
                </div>
                <div class="text-xs w-16 text-right shrink-0"
                     :class="s.loop ? 'text-cyan-300' : 'text-fg-faint'">
                  <span class="material-icons icon-sm align-middle">{{ s.loop ? 'repeat' : 'repeat_one' }}</span>
                  <span v-if="s.loop && s.loop_count">×{{ s.loop_count }}</span>
                  <span v-else-if="s.loop">∞</span>
                </div>
                <div class="flex items-center gap-1 shrink-0">
                  <div class="flex flex-col">
                    <button class="reorder-btn" title="Move up"
                            :disabled="idx === 0" @click="moveSound(s, 'up')">
                      <span class="material-icons">arrow_drop_up</span>
                    </button>
                    <button class="reorder-btn" title="Move down"
                            :disabled="idx === visibleSounds.length - 1" @click="moveSound(s, 'down')">
                      <span class="material-icons">arrow_drop_down</span>
                    </button>
                  </div>
                  <button class="btn-sm bg-emerald-500/80 hover:bg-emerald-500 text-fg-strong"
                          title="Play" @click="playSound(s)">
                    <span class="material-icons icon-sm">play_arrow</span>
                  </button>
                  <button class="btn-sm bg-surface hover:bg-red-600/80 text-fg-strong"
                          title="Stop this node" @click="stopSound(s)">
                    <span class="material-icons icon-sm">stop</span>
                  </button>
                  <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                          title="Delete" @click="deleteSound(s)">
                    <span class="material-icons icon-sm">delete</span>
                  </button>
                </div>
              </div>
            </div>
            <p v-if="soundError" class="text-xs text-amber-300 italic mt-2">{{ soundError }}</p>
            <p v-if="soundInfo" class="text-xs text-emerald-300 italic mt-2">{{ soundInfo }}</p>
          </template>
        </div>
      </div>
    </div>

    <datalist id="anim-group-suggest">
      <option v-for="g in animationGroups" :key="g" :value="g" />
    </datalist>
    <datalist id="pose-group-suggest">
      <option v-for="g in poseGroups" :key="g" :value="g" />
    </datalist>
    <datalist id="sound-group-suggest">
      <option v-for="g in soundGroups" :key="g" :value="g" />
    </datalist>

    <NewAnimationModal v-if="newAnimOpen"
                       :groups="animationGroups"
                       :default-group="view.kind === 'animations' && view.group && view.group !== '__ungrouped__' ? view.group : ''"
                       @close="newAnimOpen = false"
                       @create="onCreateAnimation" />
    <NewPoseModal v-if="newPoseOpen"
                  :groups="poseGroups"
                  :default-group="view.kind === 'poses' && view.group && view.group !== '__ungrouped__' ? view.group : ''"
                  @close="newPoseOpen = false"
                  @create="onCreatePose" />
    <NewSoundModal v-if="newSoundOpen"
                   :groups="soundGroups"
                   :default-group="view.kind === 'sounds' && view.group && view.group !== '__ungrouped__' ? view.group : ''"
                   @close="newSoundOpen = false"
                   @create="onCreateSound" />
    <AddSoundsFromFolderModal v-if="newSoundFolderOpen"
                   :groups="soundGroups"
                   :default-group="view.kind === 'sounds' && view.group && view.group !== '__ungrouped__' ? view.group : ''"
                   @close="newSoundFolderOpen = false"
                   @create="onCreateSoundFolder" />
    <MaestroImportModal v-if="importOpen" @close="importOpen = false" />
  </section>
</template>

<style scoped>
.anim-icon-cell {
  display: inline-flex; align-items: center; justify-content: center;
  width: 36px; height: 36px;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid var(--color-surface);
  border-radius: 6px;
  color: var(--color-fg);
  cursor: pointer;
  transition: border-color 0.1s, color 0.1s;
  flex-shrink: 0;
}
.anim-icon-cell:hover { border-color: var(--color-cyan-500); color: var(--color-fg-strong); }
.anim-icon-cell .material-icons { font-size: 22px; }

.reorder-btn {
  display: inline-flex; align-items: center; justify-content: center;
  height: 14px; width: 20px;
  color: var(--color-fg-faint);
  cursor: pointer;
  transition: color 0.1s;
}
.reorder-btn:hover:not(:disabled) { color: var(--color-cyan-300); }
.reorder-btn:disabled { opacity: 0.3; cursor: default; }
.reorder-btn .material-icons { font-size: 18px; }
</style>
