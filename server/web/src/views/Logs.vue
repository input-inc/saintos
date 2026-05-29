<script setup>
/* eslint-disable vue/no-mutating-props */
// Logs page — master-detail layout matching the legacy vanilla UI.
//
//   Left rail: log sources. "Server" (the in-process activity ring) is
//   always present and selected by default. Each adopted node is
//   listed below as its own clickable entry.
//
//   Right pane: a toolbar (global server log-level dropdown, rank-based
//   row filter, search, clear button) and a list of log rows. Each row
//   is click-to-expand and shows the raw JSON for the entry in an
//   inline detail panel.
//
//   Per-source plumbing:
//     - Server: backed by `useActivityStore`, hydrated once via
//       `get_logs`, kept live by the existing `activity` event listener
//       in the store.
//     - Node:   backed by an in-component `Map<nodeId, entries[]>`,
//       hydrated via `get_node_logs`, live-tailed by subscribing to
//       `node_log/<id>` while that node is selected. Only one node is
//       subscribed at a time — switching sources unsubscribes the old
//       topic so a busy fleet doesn't flood quiet sources.
//
//   Scroll behavior matches vanilla: entries are stored newest-first
//   and rendered top-to-bottom, so new lines "prepend" visually. If
//   the user has scrolled down to read older entries, we offset
//   `scrollTop` by the height of the newly inserted row so the read
//   position stays anchored — same trick as `app.js:_prependLogRow`.

import { computed, nextTick, onMounted, onUnmounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useActivityStore } from '@/stores/activity'
import { useNodesStore } from '@/stores/nodes'

const ws = useWsStore()
const activity = useActivityStore()
const nodes = useNodesStore()

// ── Source selection ─────────────────────────────────────────────────
// Selected source key: 'server' or `node:<id>`.
const selectedSource = ref('server')

// Per-node ring buffer (newest-first, capped). Populated by
// `get_node_logs` on selection + the `node_log/<id>` topic listener
// while that node is active.
const nodeRings = ref(new Map())
const MAX_PER_NODE = 500

// Currently-subscribed node-log topic, if any. Tracked separately
// from `selectedSource` so we can tear it down even after the
// selection has moved on.
let liveNodeId = null

const sourceItems = computed(() => {
  const items = [{ key: 'server', label: 'Server', icon: 'dns', meta: '' }]
  for (const n of (nodes.all || [])) {
    if (!n.node_id) continue
    items.push({
      key: `node:${n.node_id}`,
      label: n.display_name || n.node_id,
      icon: 'memory',
      meta: n.node_id.length > 14 ? n.node_id.slice(0, 14) + '…' : n.node_id,
    })
  }
  return items
})

const sourceTitle = computed(() =>
  selectedSource.value === 'server' ? 'Server' : selectedSource.value.slice(5))

const sourceSubtitle = computed(() =>
  selectedSource.value === 'server'
    ? 'Activity events from the server process.'
    : 'Per-node events streamed from this node.')

// ── Toolbar state ────────────────────────────────────────────────────
const search = ref('')
// Rank-based "show this level and worse". Matches vanilla's
// `logs-row-filter`. 'all' shows everything; 'error' shows only
// error/fatal (the vanilla rule).
const rowFilter = ref('all')
const serverLogLevel = ref('WARNING')

const LEVEL_RANK = { debug: 10, info: 20, warn: 30, warning: 30, error: 40, fatal: 50 }

function entryPasses (entry) {
  const rank = LEVEL_RANK[(entry.level || 'info').toLowerCase()] || 20
  if (rowFilter.value !== 'all') {
    let threshold = 0
    if (rowFilter.value === 'debug') threshold = LEVEL_RANK.debug
    else if (rowFilter.value === 'info') threshold = LEVEL_RANK.info
    else if (rowFilter.value === 'warn') threshold = LEVEL_RANK.warn
    else if (rowFilter.value === 'error') threshold = LEVEL_RANK.error
    if (rowFilter.value === 'error') {
      if (rank < LEVEL_RANK.error) return false
    } else if (rank < threshold) {
      return false
    }
  }
  const q = search.value.trim().toLowerCase()
  if (q) {
    const hay = (entry.text || '').toLowerCase()
      + ' ' + (entry.peripheral || '').toLowerCase()
    if (!hay.includes(q)) return false
  }
  return true
}

// Source-backed buffer. Reactive so the row list updates as the
// store / per-node ring change.
const sourceEntries = computed(() => {
  if (selectedSource.value === 'server') return activity.entries
  if (selectedSource.value.startsWith('node:')) {
    const id = selectedSource.value.slice(5)
    return nodeRings.value.get(id) || []
  }
  return []
})

const visibleEntries = computed(() => sourceEntries.value.filter(entryPasses))

const fallbackSource = computed(() =>
  selectedSource.value === 'server'
    ? 'saint_server'
    : selectedSource.value.slice(5))

// ── Row expansion ────────────────────────────────────────────────────
const expandedKey = ref(null)
function rowKey (entry, idx) { return `${entry.time || 'x'}:${idx}` }
function toggleRow (entry, idx) {
  const key = rowKey(entry, idx)
  expandedKey.value = (expandedKey.value === key) ? null : key
}

// ── Source switching ─────────────────────────────────────────────────
async function selectSource (key) {
  if (!key || key === selectedSource.value) return
  selectedSource.value = key
  expandedKey.value = null
  // Suppress one pass of the prepend-shift watcher — the source
  // change itself replaces the buffer wholesale, which would
  // otherwise be mistaken for a giant prepend.
  suppressShift = true
  await syncLiveSubscription()
  if (key.startsWith('node:')) {
    await loadNodeHistory(key.slice(5))
  }
  // Scroll back to the top so the freshest row is visible on a
  // source switch — vanilla does the same after a full re-render.
  await nextTick()
  if (rowsEl.value) rowsEl.value.scrollTop = 0
  // Reset tracking so the next mutation is treated as a fresh
  // prepend rather than a delta against the old source's buffer.
  lastTopKey = visibleEntries.value.length ? rowKey(visibleEntries.value[0], 0) : null
  lastTopCount = visibleEntries.value.length
  suppressShift = false
}

async function syncLiveSubscription () {
  const wantNodeId = selectedSource.value.startsWith('node:')
    ? selectedSource.value.slice(5)
    : null
  if (liveNodeId && liveNodeId !== wantNodeId) {
    try { await ws.unsubscribe([`node_log/${liveNodeId}`]) } catch (_) {}
    liveNodeId = null
  }
  if (wantNodeId && liveNodeId !== wantNodeId) {
    try { await ws.subscribe([`node_log/${wantNodeId}`]) } catch (e) {
      console.warn('node_log subscribe failed:', e)
    }
    liveNodeId = wantNodeId
  }
}

async function loadNodeHistory (nodeId) {
  try {
    const r = await ws.management('get_node_logs', { node_id: nodeId })
    const list = (r && r.entries) || []
    // Server returns oldest-first; store newest-first to match the
    // activity buffer layout.
    nodeRings.value.set(nodeId, [...list].reverse().slice(0, MAX_PER_NODE))
  } catch (e) {
    console.warn('get_node_logs failed:', e)
    nodeRings.value.set(nodeId, [])
  }
}

// Live node_log/<id> entries arrive via the generic `state` frame.
// We catch them here (rather than `useWsTopic`) so the buffer keeps
// accumulating instead of clobbering on every new value.
function onStateFrame (msg) {
  if (typeof msg?.node !== 'string') return
  if (!msg.node.startsWith('node_log/')) return
  const nodeId = msg.node.slice('node_log/'.length)
  const entry = msg.data
  if (!entry) return
  const ring = nodeRings.value.get(nodeId) || []
  ring.unshift(entry)
  if (ring.length > MAX_PER_NODE) ring.length = MAX_PER_NODE
  // Re-set so the computed picks it up (Map.set on the same key
  // doesn't re-trigger reactivity on the ref).
  nodeRings.value.set(nodeId, [...ring])
}
ws.on('state', onStateFrame)
onUnmounted(() => {
  ws.off('state', onStateFrame)
  // Tear down the active subscription if we're leaving the page.
  if (liveNodeId) {
    ws.unsubscribe([`node_log/${liveNodeId}`]).catch(() => {})
    liveNodeId = null
  }
})

// ── Toolbar actions ──────────────────────────────────────────────────
async function applyServerLogLevel () {
  const level = serverLogLevel.value
  if (!level) return
  try {
    await ws.management('set_settings', { settings: { logging: { level } } })
    activity.add(`Log level set to ${level}`, 'info')
  } catch (err) {
    console.error('Failed to set log level:', err)
    alert(`Failed to set log level: ${err.message || err}`)
  }
}

async function clearCurrent () {
  if (selectedSource.value === 'server') {
    activity.clear()
    return
  }
  if (!selectedSource.value.startsWith('node:')) return
  const nodeId = selectedSource.value.slice(5)
  nodeRings.value.set(nodeId, [])
  try { await ws.management('clear_node_logs', { node_id: nodeId }) }
  catch (e) { console.warn('clear_node_logs failed:', e) }
}

// ── Initial load ─────────────────────────────────────────────────────
async function init () {
  // Backfill the server-source ring from history so the page isn't
  // empty on first open.
  try {
    const r = await ws.management('get_logs', { limit: 200 })
    activity.mergeHistory((r && r.logs) || [])
  } catch (e) { console.warn('get_logs failed:', e) }

  // Sync the level dropdown with the server's current value.
  try {
    const settings = await ws.management('get_settings', {})
    const lvl = settings?.logging?.level
    if (lvl) serverLogLevel.value = lvl
  } catch (_) { /* non-fatal */ }

  // Adopted-node list — if it hasn't loaded yet (e.g. deep-link to
  // /#logs without visiting /nodes first) the sidebar would be empty.
  if (!nodes.loaded && ws.connected) {
    nodes.fetchAll().catch(() => {})
  }
}
onMounted(init)

// ── Scroll preservation on prepend ───────────────────────────────────
// New entries land at the top of the list. If the user has scrolled
// down to read older entries, bumping scrollTop by the prepended
// row's height keeps their view anchored on whatever they were
// reading — same trick as vanilla's `_prependLogRow`.
const rowsEl = ref(null)
let lastTopKey = null
let lastTopCount = 0
let suppressShift = false
watch(visibleEntries, async (curr) => {
  // Capture scroll position BEFORE the DOM updates so we can adjust
  // after the new rows are in. Without this we'd measure the
  // already-shifted scrollTop.
  const el = rowsEl.value
  const prevScrollTop = el ? el.scrollTop : 0
  await nextTick()
  if (!el) return
  if (!curr.length) { lastTopKey = null; lastTopCount = 0; return }
  const newTopKey = rowKey(curr[0], 0)
  const prevTopKey = lastTopKey
  const added = Math.max(0, curr.length - lastTopCount)
  lastTopKey = newTopKey
  lastTopCount = curr.length
  if (suppressShift) return
  if (prevTopKey && newTopKey !== prevTopKey && added > 0) {
    const wasAtTop = prevScrollTop <= 4
    if (!wasAtTop) {
      let shift = 0
      const children = el.children
      for (let i = 0; i < Math.min(added, children.length); i++) {
        shift += children[i].offsetHeight
      }
      el.scrollTop = prevScrollTop + shift
    }
  }
})

// ── Formatting helpers ───────────────────────────────────────────────
function fmtTime (ts) {
  if (!ts) return ''
  const t = new Date(ts * 1000)
  return t.toLocaleTimeString([], { hour12: false })
    + '.' + String(t.getMilliseconds()).padStart(3, '0')
}
function levelClass (level) {
  return (level || 'info').toLowerCase()
}
function detailJson (entry) { return JSON.stringify(entry, null, 2) }
</script>

<template>
  <section class="logs-page">
    <div class="logs-shell">
      <!-- Left rail -->
      <aside class="logs-sidebar">
        <div class="logs-sidebar-header">
          <span class="text-xs uppercase tracking-wide text-fg-muted">Sources</span>
        </div>
        <div class="logs-source-list">
          <div
            v-for="item in sourceItems"
            :key="item.key"
            :class="['logs-source-item', { active: item.key === selectedSource }]"
            @click="selectSource(item.key)"
          >
            <span class="material-icons">{{ item.icon }}</span>
            <span class="truncate">{{ item.label }}</span>
            <span v-if="item.meta" class="logs-source-meta">{{ item.meta }}</span>
          </div>
        </div>
      </aside>

      <!-- Main area -->
      <div class="logs-main">
        <div class="logs-toolbar">
          <div class="min-w-0 flex-1">
            <h2 class="text-lg font-semibold text-fg-strong truncate">{{ sourceTitle }}</h2>
            <p class="text-xs text-fg-muted truncate">{{ sourceSubtitle }}</p>
          </div>
          <div class="flex gap-2 items-center flex-wrap">
            <label class="text-xs text-fg-muted">Level</label>
            <select
              v-model="serverLogLevel"
              class="input-field text-sm py-1"
              @change="applyServerLogLevel"
            >
              <option value="DEBUG">Debug</option>
              <option value="INFO">Info</option>
              <option value="WARNING">Warning</option>
              <option value="ERROR">Error</option>
            </select>
            <select
              v-model="rowFilter"
              class="input-field text-sm py-1"
              title="Filter the rows currently shown"
            >
              <option value="all">Show all</option>
              <option value="debug">Debug+</option>
              <option value="info">Info+</option>
              <option value="warn">Warn+</option>
              <option value="error">Error only</option>
            </select>
            <input
              v-model="search"
              type="text"
              placeholder="Search logs…"
              class="input-field text-sm py-1 w-48"
            />
            <button
              class="btn-secondary text-sm"
              title="Clear the rows shown for the current source"
              @click="clearCurrent"
            >
              <span class="material-icons icon-sm">delete_sweep</span>
              Clear
            </button>
          </div>
        </div>

        <div ref="rowsEl" class="logs-rows">
          <p v-if="!visibleEntries.length" class="logs-empty">
            No logs match the current filters.
          </p>
          <div
            v-for="(entry, idx) in visibleEntries"
            v-else
            :key="rowKey(entry, idx)"
            :class="[
              'logs-row',
              levelClass(entry.level),
              { expanded: expandedKey === rowKey(entry, idx) },
            ]"
            @click="toggleRow(entry, idx)"
          >
            <span class="logs-time">{{ fmtTime(entry.time) }}</span>
            <span class="logs-source">{{ entry.peripheral || fallbackSource }}</span>
            <span class="logs-summary">{{ entry.text || '' }}</span>
            <pre
              v-if="expandedKey === rowKey(entry, idx)"
              class="logs-detail"
              @click.stop
            >{{ detailJson(entry) }}</pre>
          </div>
        </div>
      </div>
    </div>
  </section>
</template>
