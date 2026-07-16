import { defineStore } from 'pinia'
import { ref } from 'vue'
import { useWsStore } from './ws'

// Soundboard library: named audio files that a specific node plays out
// one of its own ALSA outputs. Unlike poses/animations, entries carry an
// explicit `position` (the operator drag-orders them) and are node-scoped
// (file_path lives on that node; playback is stop-and-replace per node).
//
// Two flavours of server call:
//   - persistence (list/save/delete/reorder/play/stop) is plain
//     request/response over `management`,
//   - filesystem-browse + audio-device enumeration are async round-trips
//     to the node: we subscribe to the reply topic, fire the request with
//     a fresh request_id, and resolve on the matching frame (same pattern
//     the BLE-scan picker uses in Peripherals.vue).
export const useSoundsStore = defineStore('sounds', () => {
  const ws = useWsStore()
  const list = ref([])
  const loading = ref(false)
  const error = ref('')

  async function reload () {
    loading.value = true
    error.value = ''
    try {
      const r = await ws.management('list_sounds', {})
      list.value = r?.sounds || []
    } catch (e) {
      error.value = e.message || String(e)
    } finally {
      loading.value = false
    }
  }

  async function save (sound) {
    error.value = ''
    try {
      const r = await ws.management('save_sound', { sound })
      if (r?.success === false) {
        error.value = r.message || 'Save failed'
        return null
      }
      await reload()
      return r?.sound || null
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  async function remove (id) {
    try {
      await ws.management('delete_sound', { id })
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  async function reorder (orderedIds) {
    try {
      const r = await ws.management('reorder_sounds', { ordered_ids: orderedIds })
      list.value = r?.sounds || list.value
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  // Add one sound per file in a folder, skipping already-added ones.
  // Enumerates the folder here (reusing the browse round-trip) so it
  // works for both the host and firmware nodes, then hands the file list
  // to the server, which dedupes by (node, path) and assigns ids.
  // Returns { added, skipped } or null on error.
  async function bulkAddFromFolder (nodeId, folder, opts = {}) {
    error.value = ''
    try {
      const data = await browseDir(nodeId, folder)
      if (data.status !== 'ok') {
        error.value = data.message || 'Could not read folder'
        return null
      }
      const base = String(data.path || folder).replace(/\/$/, '')
      const files = (data.entries || [])
        .filter(e => !e.is_dir && (opts.includeAll || e.is_audio))
        .map(e => `${base}/${e.name}`)
      const r = await ws.management('add_sounds_from_folder', {
        node_id: nodeId,
        files,
        output_device: opts.output_device || 'default',
        group: opts.group || '',
        volume: opts.volume ?? 1.0,
        loop: !!opts.loop,
        loop_count: opts.loop_count || 0,
      })
      list.value = r?.sounds || list.value
      return { added: r?.added || 0, skipped: r?.skipped || 0,
               scanned: files.length }
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  async function listNodes () {
    try {
      const r = await ws.management('sound_list_nodes', {})
      return r?.nodes || []
    } catch (e) {
      error.value = e.message || String(e)
      return []
    }
  }

  // Subscribe → request → resolve-on-matching-frame → cleanup. Rejects on
  // a 15 s silence so the picker/spinner can't hang forever if the node
  // is offline or the topic never matches.
  function _roundTrip (topicPrefix, action, nodeId, extraParams = {}) {
    return new Promise((resolve, reject) => {
      const topic = `${topicPrefix}/${nodeId}`
      const reqId = `sb-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`
      let done = false
      const onFrame = (msg) => {
        if (msg?.node !== topic) return
        const data = msg.data
        if (!data || typeof data !== 'object') return
        if (data.request_id && data.request_id !== reqId) return
        finish(() => resolve(data))
      }
      const timer = setTimeout(
        () => finish(() => reject(new Error('Node did not respond'))), 15000)
      function finish (cb) {
        if (done) return
        done = true
        clearTimeout(timer)
        ws.off('state', onFrame)
        ws.unsubscribe([topic]).catch(() => {})
        cb()
      }
      ws.on('state', onFrame)
      ws.subscribe([topic], 5)
        .then(() => ws.management(action,
          { node_id: nodeId, request_id: reqId, ...extraParams }))
        .catch(e => finish(() => reject(e)))
    })
  }

  const browseDir = (nodeId, path) =>
    _roundTrip('soundboard_fs', 'sound_list_dir', nodeId, { path })
  const listDevices = (nodeId) =>
    _roundTrip('soundboard_devices', 'sound_list_devices', nodeId)

  async function play (id) {
    error.value = ''
    try {
      return await ws.management('play_sound', { id })
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  async function stop (nodeId) {
    try {
      return await ws.management('stop_sound', { node_id: nodeId })
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  return {
    list, loading, error,
    reload, save, remove, reorder, listNodes, bulkAddFromFolder,
    browseDir, listDevices, play, stop,
  }
})
