import { defineStore } from 'pinia'
import { computed, ref } from 'vue'
import { useWsStore } from './ws'

// Animation library: load summaries + fetch / save / delete individual
// Animation records. The active "edit" target is held alongside the
// library so the timeline editor can mutate it without round-tripping
// through the library list on every keyframe drag.
export const useAnimationsStore = defineStore('animations', () => {
  const ws = useWsStore()
  const list = ref([])
  const loading = ref(false)
  const editing = ref(null)        // full Animation object or null
  const dirty = ref(false)
  const error = ref('')
  const players = ref([])          // live playback state from animation_state

  // Undo/redo. `history` holds snapshots of `editing` BEFORE each
  // mutation; `future` holds snapshots that were popped via undo and
  // can be reapplied via redo. New edits clear future (standard undo
  // semantics — once you've undone something and then start editing,
  // the redo stack is invalidated).
  const SNAPSHOT_COALESCE_MS = 500
  const SNAPSHOT_LIMIT = 50
  const history = ref([])
  const future = ref([])
  let _lastSnapshotAt = 0

  // JSON clone is safer than structuredClone here — Pinia stores wrap
  // the value in a Vue reactive proxy, and structuredClone on certain
  // proxy shapes throws DataCloneError. Animation payloads are pure
  // JSON-compatible (numbers, strings, arrays, plain objects), so the
  // stringify/parse round-trip is both safe and ~50KB-per-second on
  // realistic data — fast enough for interactive editing.
  function _clone (obj) {
    return JSON.parse(JSON.stringify(obj))
  }

  // Capture the current state into history. Coalesces snapshots that
  // happen <500 ms apart so a typing burst or a drag doesn't pile up
  // one history entry per character / frame; the first snapshot in
  // the burst is what `undo` restores. Pass `force: true` to bypass
  // coalescing for actions that should always create a new entry
  // (e.g. clicking to add a keyframe).
  function snapshot ({ force = false } = {}) {
    if (!editing.value) return
    const now = Date.now()
    if (!force && now - _lastSnapshotAt < SNAPSHOT_COALESCE_MS) {
      _lastSnapshotAt = now
      return
    }
    history.value.push(_clone(editing.value))
    if (history.value.length > SNAPSHOT_LIMIT) history.value.shift()
    future.value = []
    _lastSnapshotAt = now
  }

  function undo () {
    if (!history.value.length || !editing.value) return false
    future.value.push(_clone(editing.value))
    editing.value = history.value.pop()
    dirty.value = true
    _lastSnapshotAt = 0      // next mutation makes a fresh snapshot
    return true
  }

  function redo () {
    if (!future.value.length || !editing.value) return false
    history.value.push(_clone(editing.value))
    editing.value = future.value.pop()
    dirty.value = true
    _lastSnapshotAt = 0
    return true
  }

  function _resetHistory () {
    history.value = []
    future.value = []
    _lastSnapshotAt = 0
  }

  const canUndo = computed(() => history.value.length > 0)
  const canRedo = computed(() => future.value.length > 0)

  async function reload () {
    loading.value = true
    error.value = ''
    try {
      const r = await ws.management('list_animations', {})
      list.value = r?.animations || []
    } catch (e) {
      error.value = e.message || String(e)
    } finally {
      loading.value = false
    }
  }

  async function load (id) {
    error.value = ''
    try {
      const r = await ws.management('get_animation', { id })
      editing.value = r?.animation || null
      dirty.value = false
      _resetHistory()      // history is per-record; switching loses it
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  function startNew () {
    editing.value = {
      id: '', name: '',
      duration: 5.0,
      fps: 60,
      loop: false,
      icon: '',
      group: '',
      value_tracks: [],
      trigger_tracks: [],
      created: '',
      modified: '',
    }
    dirty.value = true
    _resetHistory()
  }

  function markDirty () { dirty.value = true }

  async function save () {
    if (!editing.value) return
    error.value = ''
    try {
      const r = await ws.management('save_animation', { animation: editing.value })
      const ok = r?.success !== false
      if (!ok) {
        error.value = r?.message || 'Save failed'
        return
      }
      editing.value = r.animation
      dirty.value = false
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  async function remove (id) {
    error.value = ''
    try {
      await ws.management('delete_animation', { id })
      if (editing.value?.id === id) editing.value = null
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  async function refreshPlayers () {
    try {
      const r = await ws.management('animation_state', {})
      players.value = r?.players || []
    } catch { /* non-fatal */ }
  }

  async function start (id, loop = null) {
    error.value = ''
    try {
      const payload = { id }
      if (loop !== null) payload.loop = loop
      await ws.management('start_animation', payload)
      await refreshPlayers()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  async function stop (id) {
    try {
      await ws.management('stop_animation', { id })
    } finally {
      await refreshPlayers()
    }
  }

  async function pause (id) {
    try { await ws.management('pause_animation', { id }) }
    finally { await refreshPlayers() }
  }

  async function resume (id) {
    try { await ws.management('resume_animation', { id }) }
    finally { await refreshPlayers() }
  }

  async function seek (id, t) {
    try { await ws.management('seek_animation', { id, t }) }
    finally { await refreshPlayers() }
  }

  const isPlaying = computed(() => (id) =>
    players.value.some(p => p.id === id && p.running)
  )

  return {
    list, loading, editing, dirty, error, players,
    reload, load, startNew, markDirty, save, remove,
    start, stop, pause, resume, seek, refreshPlayers,
    isPlaying,
    snapshot, undo, redo, canUndo, canRedo,
  }
})
