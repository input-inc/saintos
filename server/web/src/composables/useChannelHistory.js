import { reactive, ref } from 'vue'
import { useWsStore } from '@/stores/ws'

// Channel history (sparkline data) shared across components.
//
// Keyed by "<node_id>/<peripheral_id>/<channel_id>", each entry is a
// reactive array of {ts, value} where `ts` is a browser wall-clock
// millisecond timestamp. Entries are trimmed to the last WINDOW_MS on
// every push.
//
// The Pi's `time.time()`-stamped backend samples are normalised to
// browser-clock ms on ingest (see fetchHistory) so the in-RAM ring is
// internally consistent — even when the Pi has no RTC. Mixing Pi and
// browser clocks in the same array would cause every fresh live sample
// to land "older than 30s" and get trimmed away immediately.

export const WINDOW_MS = 30000

// Module-scoped reactive Map. Lives for the page lifetime so multiple
// components (Live.vue, future widgets) can observe the same ring.
const store = reactive(new Map())

function trim (arr, now = Date.now()) {
  const cutoff = now - WINDOW_MS
  while (arr.length && arr[0].ts < cutoff) arr.shift()
}

function ensure (key) {
  let arr = store.get(key)
  if (!arr) {
    arr = reactive([])
    store.set(key, arr)
  }
  return arr
}

function keyOf (node_id, peripheral_id, channel_id) {
  return `${node_id}/${peripheral_id}/${channel_id}`
}

export function useChannelHistory () {
  const ws = useWsStore()

  // Returns the live reactive array. Components binding to it will
  // update as pushSample / fetchHistory mutate the entry.
  function getHistory (key) {
    return ensure(key)
  }

  function pushSample (key, value, ts = Date.now()) {
    if (typeof value !== 'number' || !Number.isFinite(value)) return
    const arr = ensure(key)
    arr.push({ ts, value })
    trim(arr, ts)
  }

  async function fetchHistory (node_id, peripheral_id, channel_id) {
    const key = keyOf(node_id, peripheral_id, channel_id)
    const arr = ensure(key)
    try {
      const r = await ws.management('get_peripheral_history', {
        node_id,
        peripheral_id,
        channel_id,
        window: '30s',
      })
      const samples = Array.isArray(r?.samples) ? r.samples : []
      if (!samples.length) return arr
      // Server samples are stamped with the Pi's time.time() — possibly
      // many seconds (or hours, on an RTC-less Pi) off from the browser
      // clock. Rebase the whole batch so the *newest* sample lands
      // "now" in browser time; older samples keep their relative
      // spacing. Live samples added later via pushSample will use
      // Date.now() directly and slot in naturally.
      const lastTs = samples[samples.length - 1].ts
      const offsetMs = Date.now() - lastTs * 1000
      arr.splice(0, arr.length)
      for (const s of samples) {
        if (typeof s.v !== 'number' || !Number.isFinite(s.v)) continue
        arr.push({ ts: Math.round(s.ts * 1000 + offsetMs), value: s.v })
      }
      trim(arr)
      return arr
    } catch (e) {
      // Sparkline just stays empty until live samples land.
      return arr
    }
  }

  function clearHistory (node_id, peripheral_id) {
    const prefix = `${node_id}/${peripheral_id}/`
    for (const k of [...store.keys()]) {
      if (k.startsWith(prefix)) store.delete(k)
    }
  }

  return { getHistory, pushSample, fetchHistory, clearHistory, keyOf }
}
