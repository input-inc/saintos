import { defineStore } from 'pinia'
import { reactive, computed, watch } from 'vue'
import { useWsStore } from './ws'
import { useNodesStore } from './nodes'

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
  // RP2040 / Teensy path — they hand off to a bootloader / FlashTxx
  // that the application can't publish from, so we only see "starting"
  // and then later the node re-announces under the new firmware.
  // Render indeterminate the whole time.
  starting:    'Starting update',
  // Pi path — granular stages emitted by saint_node.updater.
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
  const nodesStore = useNodesStore()
  // node_id -> { stage, percent, status, message, startedAt, lastFrameAt,
  //              startingVersion }
  // startingVersion captures the node's firmware_version at start() time
  // so we can flip to 'complete' when it changes (the new firmware
  // re-announced under a new build). This is how RP2040 / Teensy OTAs
  // resolve — those nodes can't publish per-percent progress because
  // their flashing path lives in the bootloader, but they DO re-announce
  // with the new version on the other side of the reboot.
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
    // Seed an entry so the UI shows activity instantly, before any
    // ROS frame arrives. Stage = 'starting' (not 'downloading') so
    // FirmwareUpdateProgress renders indeterminate — important for
    // RP2040 / Teensy nodes, which hand off to their bootloader and
    // can't publish per-percent progress at all. A determinate 0 %
    // bar would just freeze visibly until the chip rebooted. Pi
    // nodes overwrite this seed with a real 'downloading' frame on
    // first progress tick (~hundreds of ms), so the bar still
    // becomes determinate quickly when the data exists.
    //
    // We also snapshot the node's current firmware_version so the
    // version-change watcher (below) can flip status → 'complete'
    // when the new firmware re-announces. That's the only completion
    // signal we get from bootloader-driven targets.
    if (!entries[nodeId] || TERMINAL_STATUSES.has(entries[nodeId].status)) {
      const node = (nodesStore.all || []).find(n => n.node_id === nodeId)
      entries[nodeId] = {
        stage:   'starting',
        percent: null,
        status:  'in_progress',
        message: '',
        startedAt: Date.now(),
        lastFrameAt: Date.now(),
        startingVersion: node?.firmware_version || null,
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

  // Auto-complete the entry when a node re-announces under a new
  // firmware_version. This is the only completion signal we get
  // from RP2040 / Teensy OTAs — those nodes hand off to their
  // bootloader (which lives outside the saint_node application,
  // can't reach the ROS stack, and definitely can't publish to
  // /update_progress). What WE see is the node going offline mid-
  // OTA, then coming back online some seconds later with a new
  // firmware_version in its /announce. nodes.js maintains that
  // in nodesStore.all, so we just watch for the version flip.
  //
  // Pi nodes ALSO benefit: their saint_node service restarts at
  // the end of perform_update(), which means the last few
  // 'restarting' progress frames get dropped (publisher dies
  // mid-flight). The version-change watcher closes the loop so the
  // bar still clears.
  watch(
    () => (nodesStore.all || []).map(n => ({ id: n.node_id, fw: n.firmware_version })),
    (after, before) => {
      if (!before) return
      const prevByid = new Map(before.map(x => [x.id, x.fw]))
      for (const cur of after) {
        const entry = entries[cur.id]
        if (!entry || TERMINAL_STATUSES.has(entry.status)) continue
        const prevFw = prevByid.get(cur.id)
        // Need a real prior value to detect a change — the first
        // tick after page load has prevFw === undefined for every
        // node and we'd false-positive complete every adopted node.
        if (!prevFw) continue
        if (cur.fw && cur.fw !== prevFw) {
          // Compare to the version captured at start(): if it
          // changed away from there, we definitely flipped over a
          // reboot, not just a flapping cached value.
          if (!entry.startingVersion || cur.fw !== entry.startingVersion) {
            entries[cur.id] = {
              ...entry,
              stage: 'restarting',
              percent: 100,
              status: 'complete',
              message: `Updated to ${cur.fw}`,
              lastFrameAt: Date.now(),
            }
            setTimeout(() => {
              const e = entries[cur.id]
              if (e && TERMINAL_STATUSES.has(e.status)) {
                delete entries[cur.id]
              }
            }, TERMINAL_LINGER_MS)
          }
        }
      }
    },
    { deep: true }
  )

  const byId = (nodeId) => computed(() => entries[nodeId] || null)

  const isActive = (nodeId) => computed(() => {
    const e = entries[nodeId]
    return !!e && !TERMINAL_STATUSES.has(e.status)
  })

  return { entries, start, cancel, byId, isActive }
})
