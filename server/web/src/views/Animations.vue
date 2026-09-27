<script setup>
import { computed, defineAsyncComponent, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRouter } from 'vue-router'
import { useAnimationsStore } from '@/stores/animations'
import { usePosesStore } from '@/stores/poses'
import { useSoundsStore } from '@/stores/sounds'
import { usePlaylistsStore } from '@/stores/playlists'
import { useWsStore } from '@/stores/ws'
import IconPicker from '@/components/animation/IconPicker.vue'
import NumberField from '@/components/NumberField.vue'

// Full-screen "Boards" management. Sidebar lists animations, poses, and
// sounds, each with their playlists, and a "New" button per kind opens a
// modal collecting base info; submitting routes into the editor (for
// animations) or reloads the list (poses/sounds).
//
// Grouping is playlists, not a field on the item. An item can be in any
// number of them and sits at its own slot in each, so there is nowhere
// on the item to write it -- membership is assigned by dragging a row
// onto a playlist in the sidebar, and order by dragging rows inside a
// playlist view. The modals no longer ask for a group at all.

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
const playlists = usePlaylistsStore()
const ws = useWsStore()

// Sidebar selection: which kind is highlighted, and which playlist
// within it. playlist=null is the "All" bucket for that kind;
// UNGROUPED is "in no playlist at all".
const UNGROUPED = '__ungrouped__'
const view = ref({ kind: 'animations', playlist: null })
function selectView (kind, playlist) { view.value = { kind, playlist } }

/** The store list backing a kind. */
function storeFor (kind) {
  return kind === 'animations' ? animations : (kind === 'poses' ? poses : sounds)
}

// Playback gain ceiling, as a percentage for the row editor. Mirrors
// SOUND_VOLUME_MAX in server/saint_server/animation/models.py — the
// server clamps on save and the players clamp again before VLC, so this
// is the UI bound, not the enforcement.
const VOLUME_MAX_PCT = 200

// Set one clip's volume from the list. Without this the only way to fix
// a clip that was added too quiet was to delete and re-add it: the new-
// sound modal sets a volume once and nothing else could change it.
async function setSoundVolumePct (s, pct) {
  const clamped = Math.max(0, Math.min(VOLUME_MAX_PCT, Number(pct) || 0))
  const next = Number((clamped / 100).toFixed(2))
  if (next === Number(s.volume ?? 1)) return
  await patchSound(s.id, { volume: next })
}

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

// The sidebar's three sections. Collapsed into a descriptor because the
// markup is identical per kind and the drag-and-drop wiring on it is
// fiddly enough that three hand-copied versions would drift.
const SECTIONS = [
  { kind: 'animations', label: 'Animations', allLabel: 'All Animations', icon: 'animation' },
  { kind: 'poses', label: 'Poses', allLabel: 'All Poses', icon: 'accessibility' },
  { kind: 'sounds', label: 'Sounds', allLabel: 'All Sounds', icon: 'volume_up' },
]

// Playlists per section, already in sidebar order from the server.
const animationPlaylists = computed(() => playlists.byKind('animations'))
const posePlaylists = computed(() => playlists.byKind('poses'))
const soundPlaylists = computed(() => playlists.byKind('sounds'))
function playlistsFor (kind) {
  return kind === 'animations' ? animationPlaylists.value
       : (kind === 'poses' ? posePlaylists.value : soundPlaylists.value)
}

// How many items sit in a bucket -- the sidebar counts. A playlist can
// name an id that no longer resolves (deleted out from under it), so
// count what actually renders rather than items.length.
function countIn (kind, playlistId) {
  return visibleFor(kind, playlistId).length
}

/** Rows for one (kind, playlist) pair, in the order they should render.
 *
 *  Inside a playlist the ORDER IS THE PLAYLIST'S -- that is the whole
 *  point of the change, and it is why this can't just sort by name.
 *  Outside one, animations and poses are alphabetical and sounds keep
 *  their own `position` (the flat drag order of "All Sounds").
 */
function visibleFor (kind, playlistId) {
  const rows = storeFor(kind).list || []
  if (playlistId === null) {
    return [...rows].sort(sortFor(kind))
  }
  if (playlistId === UNGROUPED) {
    return rows.filter(r => !(r.playlists || []).length).sort(sortFor(kind))
  }
  const order = playlists.itemsOf(playlistId)
  const byId = new Map(rows.map(r => [r.id, r]))
  return order.map(id => byId.get(id)).filter(Boolean)
}

function sortFor (kind) {
  if (kind === 'sounds') {
    return (a, b) => (a.position || 0) - (b.position || 0)
                     || (a.name || '').localeCompare(b.name || '')
  }
  return (a, b) => (a.name || '').localeCompare(b.name || '')
}

const visibleAnimations = computed(() =>
  visibleFor('animations', view.value.kind === 'animations' ? view.value.playlist : null))
const visiblePoses = computed(() =>
  visibleFor('poses', view.value.kind === 'poses' ? view.value.playlist : null))
const visibleSounds = computed(() =>
  visibleFor('sounds', view.value.kind === 'sounds' ? view.value.playlist : null))

/** The playlist record currently being viewed, or null. */
const currentPlaylist = computed(() => {
  const p = view.value.playlist
  if (!p || p === UNGROUPED) return null
  return playlists.get(p)
})

// Title for the main toolbar.
const viewTitle = computed(() => {
  const kindLabel = view.value.kind === 'animations'
    ? 'Animations' : (view.value.kind === 'poses' ? 'Poses' : 'Sounds')
  const p = view.value.playlist
  if (p === null) return `All ${kindLabel}`
  if (p === UNGROUPED) return `${kindLabel} / Ungrouped`
  return `${kindLabel} / ${currentPlaylist.value?.name || p}`
})

// ── CRUD via the WS layer ───────────────────────────────────────────

// When a playlist is the current view, anything created from here
// joins it. That is context, not a form field -- the operator is
// looking at the list they want it in.
async function joinCurrentPlaylist (kind, itemId) {
  if (!itemId) return
  if (view.value.kind !== kind) return
  const p = view.value.playlist
  if (!p || p === UNGROUPED) return
  await playlists.addItem(p, itemId)
}

async function onCreateAnimation (payload) {
  newAnimOpen.value = false
  const r = await ws.management('save_animation', {
    animation: { id: '', ...payload, value_tracks: [], trigger_tracks: [] },
  })
  await animations.reload()
  const id = r?.animation?.id
  await joinCurrentPlaylist('animations', id)
  if (id) router.push({ name: 'animation-editor', params: { id } })
}

async function onCreatePose (payload) {
  newPoseOpen.value = false
  const r = await ws.management('save_pose', {
    pose: { id: '', ...payload, setpoints: [] },
  })
  await poses.reload()
  if (r?.pose?.id) {
    await joinCurrentPlaylist('poses', r.pose.id)
    // Load it for inline editing, staying on whatever list we're on.
    await poses.load(r.pose.id)
    if (view.value.kind !== 'poses') selectView('poses', null)
    editingPoseId.value = r.pose.id
  }
}

async function onCreateSound (payload) {
  newSoundOpen.value = false
  const saved = await sounds.save({ id: '', position: 0, ...payload })
  await joinCurrentPlaylist('sounds', saved?.id)
  if (view.value.kind !== 'sounds') selectView('sounds', null)
}

async function onCreateSoundFolder (payload) {
  newSoundFolderOpen.value = false
  soundError.value = ''
  soundInfo.value = 'Adding sounds from folder…'
  // A batch add lands in the playlist being viewed, in file order.
  const target = view.value.kind === 'sounds'
              && view.value.playlist && view.value.playlist !== UNGROUPED
    ? view.value.playlist : ''
  const r = await sounds.bulkAddFromFolder(payload.node_id, payload.folder, {
    output_device: payload.output_device,
    playlist_id: target,
    volume: payload.volume,
    loop: payload.loop,
    loop_count: payload.loop_count,
  })
  await playlists.reload()
  if (view.value.kind !== 'sounds') selectView('sounds', null)
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

// ── drag and drop ───────────────────────────────────────────────────
//
// Two gestures, one drag source:
//   * row → sidebar playlist   = add to that playlist (append)
//   * row → another row        = place it there
//
// The second means different things in different views, and the
// difference is the point of playlists. Inside a playlist we rewrite
// THAT playlist's order and nothing else, so an item's slot in one list
// is independent of its slot in every other. In the flat "All Sounds"
// view there is no playlist to reorder, so we fall back to the sound's
// own `position` -- the pre-playlist behaviour, kept because that view
// still needs an order.
//
// Handle-gated: a row is only `draggable` while the operator is pressing
// its drag handle, so clicking the name field and the action buttons
// still works normally.
const drag = ref({ kind: null, id: null })   // what is in flight
const dragOverId = ref(null)                 // row under the cursor
const dragOverPlaylist = ref(null)           // sidebar target under it
const dragHandleId = ref(null)               // gates :draggable

function onRowDragStart (kind, item, e) {
  drag.value = { kind, id: item.id }
  if (e.dataTransfer) {
    e.dataTransfer.effectAllowed = 'move'
    // Some browsers refuse to start a drag with no payload.
    try { e.dataTransfer.setData('text/plain', item.id) } catch { /* no-op */ }
  }
}

function onRowDragEnd () {
  drag.value = { kind: null, id: null }
  dragOverId.value = null
  dragOverPlaylist.value = null
  dragHandleId.value = null
}

function onRowDragOver (kind, item) {
  if (!drag.value.id || drag.value.kind !== kind) return
  if (item.id === drag.value.id) return
  dragOverId.value = item.id
}

/** Drop a row onto another row: place it at the target's index. */
async function onRowDrop (kind, target) {
  const from = drag.value.id
  const fromKind = drag.value.kind
  onRowDragEnd()
  if (!from || fromKind !== kind || from === target.id) return

  const pid = view.value.playlist
  if (pid && pid !== UNGROUPED) {
    // Reordering within a playlist -- rewrite just this playlist.
    const order = [...playlists.itemsOf(pid)]
    const fi = order.indexOf(from)
    const ti = order.indexOf(target.id)
    if (fi < 0 || ti < 0) return
    order.splice(fi, 1)
    order.splice(ti, 0, from)
    await playlists.reorderItems(pid, order)
    return
  }

  // Flat view. Only sounds carry an order of their own out here.
  if (kind !== 'sounds') return
  const vis = [...visibleSounds.value]
  const fi = vis.findIndex(x => x.id === from)
  const ti = vis.findIndex(x => x.id === target.id)
  if (fi < 0 || ti < 0) return
  const [moved] = vis.splice(fi, 1)
  vis.splice(ti, 0, moved)
  // Splice the reordered slice back into the full list in place so ids
  // outside the current view keep their positions.
  const visIds = new Set(vis.map(x => x.id))
  const queue = [...vis]
  const merged = (sounds.list || []).map(it => visIds.has(it.id) ? queue.shift() : it)
  await sounds.reorder(merged.map(x => x.id))
}

/** Drop a row onto a playlist in the sidebar: join it. */
function onPlaylistDragOver (kind, playlistId) {
  if (!drag.value.id || drag.value.kind !== kind) return
  dragOverPlaylist.value = playlistId
}

async function onPlaylistDrop (kind, playlistId) {
  const from = drag.value.id
  const fromKind = drag.value.kind
  onRowDragEnd()
  // Kinds are separate namespaces -- a sound cannot join an animations
  // playlist. The server enforces it too; this keeps the drop from even
  // looking like it worked.
  if (!from || fromKind !== kind) return
  await playlists.addItem(playlistId, from)
}

/** Drop onto "Ungrouped": leave every playlist of that kind. */
async function onUngroupedDrop (kind) {
  const from = drag.value.id
  const fromKind = drag.value.kind
  onRowDragEnd()
  if (!from || fromKind !== kind) return
  for (const pl of playlistsFor(kind)) {
    if ((pl.items || []).includes(from)) await playlists.removeItem(pl.id, from)
  }
}

// ── playlist management ─────────────────────────────────────────────

const renamingPlaylistId = ref(null)

async function newPlaylist (kind) {
  const label = kind === 'animations' ? 'animation'
              : (kind === 'poses' ? 'pose' : 'sound')
  const name = prompt(`Name for the new ${label} playlist:`, '')
  if (name === null) return
  const trimmed = name.trim()
  if (!trimmed) return
  const created = await playlists.create(trimmed, kind)
  if (created?.id) selectView(kind, created.id)
}

async function renamePlaylist (pl, name) {
  renamingPlaylistId.value = null
  const trimmed = (name || '').trim()
  if (!trimmed || trimmed === pl.name) return
  await playlists.patch(pl.id, { name: trimmed })
}

async function deletePlaylist (pl) {
  const n = (pl.items || []).length
  const tail = n
    ? ` The ${n} item${n === 1 ? '' : 's'} in it will not be deleted.`
    : ''
  if (!confirm(`Delete the playlist "${pl.name}"?${tail}`)) return
  await playlists.remove(pl.id)
  if (view.value.playlist === pl.id) selectView(view.value.kind, null)
}

/** Take a row out of the playlist currently being viewed. */
async function removeFromCurrentPlaylist (item) {
  const pid = view.value.playlist
  if (!pid || pid === UNGROUPED) return
  await playlists.removeItem(pid, item.id)
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
  await Promise.all([animations.reload(), poses.reload(), sounds.reload(),
                     playlists.reload(), loadWsInputs()])
  animations.refreshPlayers()
  _playerPoll = setInterval(() => animations.refreshPlayers(), 1500)
  sounds.listNodes().then(n => { audioNodes.value = n })
})
onBeforeUnmount(() => {
  if (_playerPoll) { clearInterval(_playerPoll); _playerPoll = null }
})

// If the selected playlist disappears (deleted here or from another
// session), bounce to the "All" view of the same kind so the main pane
// never looks broken-empty. An EMPTY playlist is left selected on
// purpose -- you need to be able to look at one you just made in order
// to drag things into it.
watch(() => playlists.list, () => {
  const p = view.value.playlist
  if (p === null || p === UNGROUPED) return
  if (!playlistsFor(view.value.kind).some(x => x.id === p)) {
    view.value = { kind: view.value.kind, playlist: null }
  }
}, { deep: true })
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
          <!-- One block per kind. Playlists are drop targets: dragging a
               row from the main list onto one adds it to that playlist,
               and onto "Ungrouped" takes it out of all of them. -->
          <template v-for="(sec, si) in SECTIONS" :key="sec.kind">
            <div :class="['px-3 pb-1 text-[10px] uppercase tracking-wide text-fg-faint flex items-center gap-1',
                          si === 0 ? 'pt-3' : 'pt-4 border-t border-line/40 mt-2']">
              <span class="flex-1">{{ sec.label }}</span>
              <button class="text-fg-faint hover:text-cyan-300"
                      :title="`New ${sec.label.toLowerCase()} playlist`"
                      @click="newPlaylist(sec.kind)">
                <span class="material-icons icon-sm">playlist_add</span>
              </button>
            </div>

            <div :class="['animations-sidebar-item',
                          view.kind === sec.kind && view.playlist === null ? 'active' : '']"
                 @click="selectView(sec.kind, null)">
              <span class="material-icons icon-sm">{{ sec.icon }}</span>
              <span class="flex-1 truncate">{{ sec.allLabel }}</span>
              <span class="text-[10px] text-fg-faint tabular-nums">
                {{ (storeFor(sec.kind).list || []).length }}
              </span>
            </div>

            <div v-for="pl in playlistsFor(sec.kind)" :key="pl.id"
                 :class="['animations-sidebar-item playlist-item',
                          view.kind === sec.kind && view.playlist === pl.id ? 'active' : '',
                          dragOverPlaylist === pl.id ? 'playlist-drop-target' : '']"
                 @click="selectView(sec.kind, pl.id)"
                 @dragover.prevent="onPlaylistDragOver(sec.kind, pl.id)"
                 @dragleave="dragOverPlaylist = null"
                 @drop.prevent="onPlaylistDrop(sec.kind, pl.id)">
              <span class="material-icons icon-sm">queue_music</span>
              <input v-if="renamingPlaylistId === pl.id"
                     class="input-field flex-1 text-xs py-0.5"
                     :value="pl.name"
                     autofocus
                     @click.stop
                     @blur="e => renamePlaylist(pl, e.target.value)"
                     @keydown.enter="e => e.target.blur()"
                     @keydown.escape="renamingPlaylistId = null" />
              <span v-else class="flex-1 truncate"
                    @dblclick.stop="renamingPlaylistId = pl.id">{{ pl.name }}</span>
              <span class="text-[10px] text-fg-faint tabular-nums">
                {{ countIn(sec.kind, pl.id) }}
              </span>
              <button class="playlist-delete text-fg-faint hover:text-red-400"
                      title="Delete playlist (keeps its items)"
                      @click.stop="deletePlaylist(pl)">
                <span class="material-icons icon-sm">close</span>
              </button>
            </div>

            <div v-if="countIn(sec.kind, UNGROUPED) || drag.kind === sec.kind"
                 :class="['animations-sidebar-item',
                          view.kind === sec.kind && view.playlist === UNGROUPED ? 'active' : '',
                          dragOverPlaylist === `${sec.kind}:ungrouped` ? 'playlist-drop-target' : '']"
                 @click="selectView(sec.kind, UNGROUPED)"
                 @dragover.prevent="onPlaylistDragOver(sec.kind, `${sec.kind}:ungrouped`)"
                 @dragleave="dragOverPlaylist = null"
                 @drop.prevent="onUngroupedDrop(sec.kind)">
              <span class="material-icons icon-sm">folder_open</span>
              <span class="flex-1 truncate text-fg-muted italic">Ungrouped</span>
              <span class="text-[10px] text-fg-faint tabular-nums">
                {{ countIn(sec.kind, UNGROUPED) }}
              </span>
            </div>
          </template>
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
              <template v-if="view.playlist && view.playlist !== UNGROUPED">
                This playlist is empty. Drag animations here from
                <span class="text-cyan-300 cursor-pointer"
                      @click="selectView('animations', null)">All Animations</span>,
                or create one with <span class="text-cyan-300">New Animation</span>.
              </template>
              <template v-else>
                No animations yet. Click <span class="text-cyan-300">New Animation</span> to start.
              </template>
            </div>
            <div v-else class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40">
              <div v-for="a in visibleAnimations" :key="a.id"
                   class="anim-row flex items-center gap-3 px-3 py-2 hover:bg-panel/60 transition-colors"
                   :class="{ 'row-dragging': drag.id === a.id,
                             'row-drop-target': dragOverId === a.id }"
                   :draggable="dragHandleId === a.id"
                   @dragstart="onRowDragStart('animations', a, $event)"
                   @dragover.prevent="onRowDragOver('animations', a)"
                   @drop.prevent="onRowDrop('animations', a)"
                   @dragend="onRowDragEnd">
                <button class="drag-handle"
                        title="Drag onto a playlist to add it, or onto another row to reorder"
                        @mousedown="dragHandleId = a.id"
                        @mouseup="dragHandleId = null">
                  <span class="material-icons">drag_indicator</span>
                </button>
                <IconPicker :model-value="a.icon || ''" fallback="animation"
                            @update:model-value="(v) => patchAnimation(a.id, { icon: v })" />
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
                <div class="text-xs text-fg-muted tabular-nums w-20 text-right shrink-0">
                  {{ Number(a.duration || 0).toFixed(2) }}s
                </div>
                <div class="text-xs text-fg-faint w-20 text-right tabular-nums shrink-0">
                  {{ a.value_tracks }}v · {{ a.trigger_tracks }}t
                </div>
                <div class="text-xs text-fg-faint w-20 text-right shrink-0">{{ fmtTimeAgo(a.modified) }}</div>
                <div class="flex items-center gap-1 shrink-0">
                  <button v-if="currentPlaylist"
                          class="btn-sm bg-surface hover:bg-amber-600 text-fg hover:text-fg-strong"
                          :title="`Remove from ${currentPlaylist.name} (keeps the item)`"
                          @click="removeFromCurrentPlaylist(a)">
                    <span class="material-icons icon-sm">playlist_remove</span>
                  </button>
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
              <template v-if="view.playlist && view.playlist !== UNGROUPED">
                This playlist is empty. Drag poses here from
                <span class="text-cyan-300 cursor-pointer"
                      @click="selectView('poses', null)">All Poses</span>,
                or create one with <span class="text-cyan-300">New Pose</span>.
              </template>
              <template v-else>
                No poses yet. Click <span class="text-cyan-300">New Pose</span> to start.
              </template>
            </div>
            <div v-else class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40">
              <template v-for="p in visiblePoses" :key="p.id">
                <div class="anim-row flex items-center gap-3 px-3 py-2 hover:bg-panel/60 transition-colors"
                     :class="{ 'row-dragging': drag.id === p.id,
                               'row-drop-target': dragOverId === p.id }"
                     :draggable="dragHandleId === p.id"
                     @dragstart="onRowDragStart('poses', p, $event)"
                     @dragover.prevent="onRowDragOver('poses', p)"
                     @drop.prevent="onRowDrop('poses', p)"
                     @dragend="onRowDragEnd">
                  <button class="drag-handle"
                          title="Drag onto a playlist to add it, or onto another row to reorder"
                          @mousedown="dragHandleId = p.id"
                          @mouseup="dragHandleId = null">
                    <span class="material-icons">drag_indicator</span>
                  </button>
                  <IconPicker :model-value="p.icon || ''" fallback="accessibility"
                              @update:model-value="(v) => patchPose(p.id, { icon: v })" />
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
                  <div class="text-xs text-fg-faint w-24 text-right tabular-nums shrink-0">
                    {{ p.setpoint_count }} track{{ p.setpoint_count === 1 ? '' : 's' }}
                  </div>
                  <div class="text-xs text-fg-faint w-20 text-right shrink-0">{{ fmtTimeAgo(p.modified) }}</div>
                  <div class="flex items-center gap-1 shrink-0">
                    <button v-if="currentPlaylist"
                          class="btn-sm bg-surface hover:bg-amber-600 text-fg hover:text-fg-strong"
                          :title="`Remove from ${currentPlaylist.name} (keeps the item)`"
                          @click="removeFromCurrentPlaylist(p)">
                      <span class="material-icons icon-sm">playlist_remove</span>
                    </button>
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
              <template v-if="view.playlist && view.playlist !== UNGROUPED">
                This playlist is empty. Drag sounds here from
                <span class="text-cyan-300 cursor-pointer"
                      @click="selectView('sounds', null)">All Sounds</span>,
                or create one with <span class="text-violet-300">New Sound</span>.
              </template>
              <template v-else>
                No sounds yet. Click <span class="text-violet-300">New Sound</span> to start.
              </template>
            </div>
            <div v-else class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40">
              <div v-for="s in visibleSounds" :key="s.id"
                   class="anim-row flex items-center gap-3 px-3 py-2 hover:bg-panel/60 transition-colors"
                   :class="{ 'row-dragging': drag.id === s.id,
                             'row-drop-target': dragOverId === s.id }"
                   :draggable="dragHandleId === s.id"
                   @dragstart="onRowDragStart('sounds', s, $event)"
                   @dragover.prevent="onRowDragOver('sounds', s)"
                   @drop.prevent="onRowDrop('sounds', s)"
                   @dragend="onRowDragEnd">
                <button class="drag-handle"
                        title="Drag onto a playlist to add it, or onto another row to reorder"
                        @mousedown="dragHandleId = s.id"
                        @mouseup="dragHandleId = null">
                  <span class="material-icons">drag_indicator</span>
                </button>
                <IconPicker :model-value="s.icon || ''" fallback="volume_up"
                            @update:model-value="(v) => patchSound(s.id, { icon: v })" />
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
                <div class="flex items-center gap-0.5 w-24 shrink-0"
                     :title="(s.volume ?? 1) > 1
                       ? 'Boosted above the clip\'s own level (software gain)'
                       : 'Playback volume'">
                  <NumberField class="input-field text-xs py-1 w-14 text-right"
                               step="5" :decimals="0" :min="0" :max="VOLUME_MAX_PCT"
                               :model-value="Math.round((s.volume ?? 1) * 100)"
                               @commit="v => setSoundVolumePct(s, v)" />
                  <span class="text-xs"
                        :class="(s.volume ?? 1) > 1 ? 'text-amber-300' : 'text-fg-faint'">%</span>
                </div>
                <div class="text-xs w-16 text-right shrink-0"
                     :class="s.loop ? 'text-cyan-300' : 'text-fg-faint'">
                  <span class="material-icons icon-sm align-middle">{{ s.loop ? 'repeat' : 'repeat_one' }}</span>
                  <span v-if="s.loop && s.loop_count">×{{ s.loop_count }}</span>
                  <span v-else-if="s.loop">∞</span>
                </div>
                <div class="flex items-center gap-1 shrink-0">
                  <button v-if="currentPlaylist"
                          class="btn-sm bg-surface hover:bg-amber-600 text-fg hover:text-fg-strong"
                          :title="`Remove from ${currentPlaylist.name} (keeps the sound)`"
                          @click="removeFromCurrentPlaylist(s)">
                    <span class="material-icons icon-sm">playlist_remove</span>
                  </button>
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

    <!-- The group-name datalists went with the group fields: there is no
         single group to type any more. Membership is a drag onto a
         playlist in the sidebar; a new item joins whichever playlist is
         being viewed when it is created. -->
    <NewAnimationModal v-if="newAnimOpen"
                       @close="newAnimOpen = false"
                       @create="onCreateAnimation" />
    <NewPoseModal v-if="newPoseOpen"
                  @close="newPoseOpen = false"
                  @create="onCreatePose" />
    <NewSoundModal v-if="newSoundOpen"
                   @close="newSoundOpen = false"
                   @create="onCreateSound" />
    <AddSoundsFromFolderModal v-if="newSoundFolderOpen"
                   @close="newSoundFolderOpen = false"
                   @create="onCreateSoundFolder" />
    <MaestroImportModal v-if="importOpen" @close="importOpen = false" />
  </section>
</template>

<style scoped>

/* Drag-to-reorder: a grip handle gates row dragging so the inline
   inputs and action buttons still respond to normal clicks. */
.drag-handle {
  display: inline-flex; align-items: center; justify-content: center;
  color: var(--color-fg-faint);
  cursor: grab;
  transition: color 0.1s;
  flex-shrink: 0;
  touch-action: none;
}
.drag-handle:hover { color: var(--color-cyan-300); }
.drag-handle:active { cursor: grabbing; }
.drag-handle .material-icons { font-size: 20px; }

.anim-row { transition: background-color 0.1s, box-shadow 0.1s, opacity 0.1s; }
/* The row being dragged fades; the row under the cursor shows a drop line. */
.anim-row.row-dragging { opacity: 0.4; }
.anim-row.row-drop-target { box-shadow: inset 0 2px 0 0 var(--color-cyan-400); }

/* Sidebar playlists double as drop targets. The outline has to read as
   "let go here" against the `active` background, so it's a ring rather
   than a fill. */
.animations-sidebar-item.playlist-drop-target {
  box-shadow: inset 0 0 0 2px var(--color-cyan-400);
}
/* The delete affordance stays out of the way until the row is hovered --
   a playlist is cheap to remake, but not mid-drag by accident. */
.animations-sidebar-item .playlist-delete { opacity: 0; transition: opacity 0.12s; }
.animations-sidebar-item:hover .playlist-delete,
.animations-sidebar-item.active .playlist-delete { opacity: 1; }
</style>
