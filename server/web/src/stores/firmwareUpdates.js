import { defineStore } from 'pinia'
import { reactive, computed } from 'vue'
import { useWsStore } from './ws'

// Per-node firmware-update progress, surfaced to any view that wants to
// render it (the Overview status card, the Nodes-view per-node card,
// etc.). Decoupled from the FirmwareUpdateModal — the modal initiates
// an update, then closes; this store keeps tracking the OTA across
// subsequent navigation.
//
// Data flow:
//   1. Modal (or future automation) calls start(nodeId) to begin tracking.
//   2. We WS-subscribe to `update_progress/<node_id>` and listen for
//      frames the server forwards from the node's ROS publication on
//      /saint/nodes/<id>/update_progress.
//   3. Each frame updates entries[nodeId] = { stage, percent, status,
//      message, startedAt, lastFrameAt }.
//   4. Consumers read via byId(nodeId) — a reactive view that returns
//      null when no update is active for that node.
//
// Stages the Pi-side updater emits (firmware/raspberrypi/saint_node/
// updater.py): downloading, verifying, extracting, validating, backup,
// installing, restarting. Only `downloading` carries a meaningful
// streaming percent on the Pi case — others are quick discrete bumps,
// so consumers should render those as an indeterminate bar with the
// stage label.

const TERMINAL_STATUSES = new Set(['complete', 'failed'])
// How long after a terminal status to keep the entry visible before
// purging it. Gives the operator a moment to see "Update complete!"
// before the badge disappears.
const TERMINAL_LINGER_MS = 8_000
// Drop stale in-progress entries that haven't received a frame in this
// long. A Pi mid-restart goes silent for a few seconds; allow a
// generous window before we assume the update died.
const STALE_THRESHOLD_MS = 5 * 60_000

export const STAGE_LABELS = {
  downloading: 'Downloading',
  verifying:   'Verifying',
  extracting:  'Extracting',
  validating:  'Validating',
  backup:      'Backing up',
  installing:  'Installing',
  restarting:  'Restarting',
}

export const useFirmwareUpdatesStore = defineStore('firmwareUpdates', () => {
  const ws = useWsStore()
  // node_id -> { stage, percent, status, message, startedAt, lastFrameAt }
  const entries = reactive({})

  function _topic (nodeId) { return `update_progress/${nodeId}` }

  function onStateFrame (msg) {
    const topic = msg?.node
    if (typeof topic !== 'string') return
    if (!topic.startsWith('update_progress/')) return
    const nodeId = topic.slice('update_progress/'.length)
    const d = msg.data
    if (!d || typeof d !== 'object') return
    const prev = entries[nodeId] || { startedAt: Date.now() }
    entries[nodeId] = {
      stage:        d.stage   || prev.stage   || '',
      percent:      typeof d.percent === 'number' ? d.percent : prev.percent,
      status:       d.status  || 'in_progress',
      message:      d.message || prev.message || '',
      startedAt:    prev.startedAt || Date.now(),
      lastFrameAt:  Date.now(),
    }
    if (TERMINAL_STATUSES.has(entries[nodeId].status)) {
      // Schedule purge so the UI flashes the terminal state then clears.
      const tid = nodeId
      setTimeout(() => {
        const e = entries[tid]
        if (e && TERMINAL_STATUSES.has(e.status)) {
          delete entries[tid]
        }
      }, TERMINAL_LINGER_MS)
    }
  }

  // Periodically prune stale in-progress entries. A Pi that hard-fails
  // or loses ROS connectivity mid-OTA stops publishing; without this
  // sweep the badge would stick around forever.
  function pruneStale () {
    const now = Date.now()
    for (const nodeId of Object.keys(entries)) {
      const e = entries[nodeId]
      if (!TERMINAL_STATUSES.has(e.status) &&
          (now - (e.lastFrameAt || e.startedAt || 0)) > STALE_THRESHOLD_MS) {
        delete entries[nodeId]
      }
    }
  }
  const _pruneTimer = setInterval(pruneStale, 30_000)
  void _pruneTimer

  // Tracked subscriptions so a reconnect re-subscribes them. The
  // server's subscription set is keyed per-WS-client, so a reconnect
  // (after Pi reboot / network blip) drops them — we must re-add.
  const subscribed = new Set()
  async function _ensureSubscribed (nodeId) {
    const t = _topic(nodeId)
    if (subscribed.has(t)) return
    try { await ws.subscribe([t]) } catch (_) {}
    subscribed.add(t)
  }

  async function start (nodeId) {
    if (!nodeId) return
    // Seed an entry so the UI shows "starting…" instantly, before the
    // first ROS frame arrives. Without this seed the operator clicks
    // Update, the modal closes, and they see nothing for the ~1-2s
    // it takes the control message to round-trip.
    if (!entries[nodeId] || TERMINAL_STATUSES.has(entries[nodeId].status)) {
      entries[nodeId] = {
        stage:   'downloading',
        percent: 0,
        status:  'in_progress',
        message: '',
        startedAt: Date.now(),
        lastFrameAt: Date.now(),
      }
    }
    await _ensureSubscribed(nodeId)
  }

  function cancel (nodeId) {
    delete entries[nodeId]
  }

  // Re-establish all subscriptions on WS (re)connect. Without this the
  // first connect right after the page loads, plus any subsequent
  // reconnect, both lose every active subscription.
  ws.on('ready', () => {
    for (const t of subscribed) {
      ws.subscribe([t]).catch(() => {})
    }
  })
  ws.on('state', onStateFrame)

  const byId = (nodeId) => computed(() => entries[nodeId] || null)

  const isActive = (nodeId) => computed(() => {
    const e = entries[nodeId]
    return !!e && !TERMINAL_STATUSES.has(e.status)
  })

  return { entries, start, cancel, byId, isActive }
})
