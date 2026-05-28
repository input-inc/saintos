import { defineStore } from 'pinia'
import { ref } from 'vue'
import { useWsStore } from './ws'

const MAX = 500

// Activity log buffer for the "Server" source on the Logs page.
//
// Storage order is newest-first to match the legacy `activityLog`
// ring (vanilla `app.js`: `this.activityLog.unshift(entry)`). The
// Logs view reads `entries` directly and renders top-to-bottom, so
// a freshly arrived line appears at the top.
//
// Inbound shapes vary:
//   - `activity` frames from `broadcast_activity` have `{text, level,
//     timestamp}`.
//   - `get_logs` history entries (loaded by the Logs view) have
//     `{time, text, level, peripheral?}`.
// We normalize everything onto a single `{time, text, level,
// peripheral?}` shape so the rest of the UI doesn't have to care.
export const useActivityStore = defineStore('activity', () => {
  const ws = useWsStore()
  const entries = ref([])

  function _normalize (e) {
    if (!e) return null
    const time = e.time ?? e.timestamp ?? (Date.now() / 1000)
    return {
      time,
      text: e.text || e.message || '',
      level: e.level || 'info',
      ...(e.peripheral ? { peripheral: e.peripheral } : {}),
    }
  }

  function _prepend (e) {
    const norm = _normalize(e)
    if (!norm) return
    entries.value.unshift(norm)
    if (entries.value.length > MAX) entries.value.length = MAX
  }

  // The server broadcasts activity on a typed frame, but some
  // legacy paths still ride a generic `state` frame with
  // `msg.node === 'activity'` — handle both.
  ws.on('state', (msg) => {
    if (msg?.node !== 'activity') return
    _prepend(msg.data)
  })
  ws.on('activity', (msg) => _prepend(msg))

  function add (text, level = 'info') {
    _prepend({ time: Date.now() / 1000, text, level })
  }

  function clear () { entries.value = [] }

  // Replace the buffer wholesale (used on first open of the Logs
  // page to backfill from `get_logs`). Caller is responsible for
  // passing newest-first.
  function setAll (list) {
    const norm = (list || []).map(_normalize).filter(Boolean)
    entries.value = norm.slice(0, MAX)
  }

  // Merge a history fetch with whatever has streamed in already.
  // Newest-first ordering preserved.
  function mergeHistory (list) {
    const incoming = (list || []).map(_normalize).filter(Boolean)
    const existingTimes = new Set(entries.value.map(e => e.time))
    for (const e of incoming) {
      if (!existingTimes.has(e.time)) entries.value.push(e)
    }
    entries.value.sort((a, b) => b.time - a.time)
    if (entries.value.length > MAX) entries.value.length = MAX
  }

  return { entries, add, clear, setAll, mergeHistory }
})
