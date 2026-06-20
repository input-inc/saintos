import { defineStore } from 'pinia'
import { ref } from 'vue'
import { useWsStore } from './ws'

// Pose library: snapshots of WS-input values that can be reapplied
// to the routing graph. A pose is much lighter than an animation —
// no curves, no trigger tracks, just (sheet_id, ws_input_id, value)
// triples plus a name + description.
export const usePosesStore = defineStore('poses', () => {
  const ws = useWsStore()
  const list = ref([])
  const loading = ref(false)
  const editing = ref(null)
  const dirty = ref(false)
  const error = ref('')
  const applyResult = ref(null)    // last apply_pose response (for toasts)

  async function reload () {
    loading.value = true
    error.value = ''
    try {
      const r = await ws.management('list_poses', {})
      list.value = r?.poses || []
    } catch (e) {
      error.value = e.message || String(e)
    } finally {
      loading.value = false
    }
  }

  async function load (id) {
    error.value = ''
    try {
      const r = await ws.management('get_pose', { id })
      editing.value = r?.pose || null
      dirty.value = false
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  function startNew () {
    editing.value = {
      id: '', name: '',
      icon: '',
      group: '',
      description: '',
      setpoints: [],
      created: '',
      modified: '',
    }
    dirty.value = true
  }

  function markDirty () { dirty.value = true }

  async function save () {
    if (!editing.value) return
    error.value = ''
    try {
      const r = await ws.management('save_pose', { pose: editing.value })
      const ok = r?.success !== false
      if (!ok) {
        error.value = r?.message || 'Save failed'
        return
      }
      editing.value = r.pose
      dirty.value = false
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  async function remove (id) {
    try {
      await ws.management('delete_pose', { id })
      if (editing.value?.id === id) editing.value = null
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  async function apply (id) {
    error.value = ''
    try {
      const r = await ws.management('apply_pose', { id })
      applyResult.value = r
      return r
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  // Push the currently-edited (possibly unsaved) setpoints live without
  // persisting, so the operator can compare the in-progress pose against
  // the saved version (the row's play button) before committing a save.
  async function preview () {
    if (!editing.value) return null
    error.value = ''
    try {
      const r = await ws.management('preview_pose', {
        setpoints: editing.value.setpoints || [],
      })
      applyResult.value = r
      return r
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  return {
    list, loading, editing, dirty, error, applyResult,
    reload, load, startNew, markDirty, save, remove, apply, preview,
  }
})
