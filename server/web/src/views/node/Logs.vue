<script setup>
import { computed, nextTick, onMounted, onUnmounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const history = ref([])           // accumulating buffer (initial fetch + live)
const listEl = ref(null)
const MAX_ROWS = 500
// Monotonic seq for stable v-for keys. Using the array index would
// cause Vue to re-use DOM nodes with new content whenever the buffer
// shifts (every push past MAX_ROWS), producing visible flicker.
let nextSeq = 0
function withSeq (entry) {
  return { ...entry, _seq: ++nextSeq }
}

const entries = computed(() => history.value)

async function loadHistory () {
  history.value = []
  try {
    const r = await ws.management('get_node_logs', { node_id: props.nodeId })
    history.value = ((r && r.entries) || []).map(withSeq)
  } catch (e) {
    console.warn('Failed to load logs:', e)
  }
}

// Subscribe directly to the topic + catch live entries off the 'state'
// frame so each one APPENDS instead of clobbering the previous value.
// Same pattern as views/Logs.vue — see the comment there for why
// useWsTopic isn't sufficient (it only stores the latest payload, so
// every new log line replaces the previous one and the dashboard
// either shows just the newest line or briefly overlaps during the
// reactive re-render).
let liveTopic = null
async function attachLive (nodeId) {
  if (liveTopic) {
    try { await ws.unsubscribe([liveTopic]) } catch (_) {}
    liveTopic = null
  }
  if (!nodeId) return
  liveTopic = `node_log/${nodeId}`
  try {
    await ws.subscribe([liveTopic])
  } catch (e) {
    console.warn(`node_log subscribe failed: ${e}`)
  }
}

function onStateFrame (msg) {
  if (typeof msg?.node !== 'string') return
  if (msg.node !== liveTopic) return
  const entry = msg.data
  if (!entry || typeof entry !== 'object' || Array.isArray(entry)) return
  history.value.push(withSeq(entry))
  if (history.value.length > MAX_ROWS) {
    history.value.splice(0, history.value.length - MAX_ROWS)
  }
}

onMounted(async () => {
  ws.on('state', onStateFrame)
  await loadHistory()
  await attachLive(props.nodeId)
})

watch(() => props.nodeId, async (newId) => {
  await loadHistory()
  await attachLive(newId)
})

onUnmounted(async () => {
  ws.off('state', onStateFrame)
  if (liveTopic) {
    try { await ws.unsubscribe([liveTopic]) } catch (_) {}
    liveTopic = null
  }
})

// Stick to bottom as new lines arrive.
watch(entries, async () => {
  await nextTick()
  if (listEl.value) listEl.value.scrollTop = listEl.value.scrollHeight
})

async function clear () {
  history.value = []
  try { await ws.management('clear_node_logs', { node_id: props.nodeId }) }
  catch (e) { console.warn('clear_node_logs failed:', e) }
}

function fmtTime (ts) {
  if (!ts) return ''
  const t = new Date(ts * 1000)
  return t.toLocaleTimeString([], { hour12: false }) + '.' + String(t.getMilliseconds()).padStart(3, '0')
}
function classFor (level) {
  return ({ error: 'error', warn: 'warn', info: 'info', debug: 'debug' }[(level || 'info').toLowerCase()]) || 'info'
}
</script>

<template>
  <div class="card">
    <div class="flex items-center justify-between mb-4">
      <h3 class="text-lg font-semibold text-fg-strong">Logs</h3>
      <button class="btn-secondary text-sm" @click="clear">
        <span class="material-icons icon-sm">clear</span>
        Clear
      </button>
    </div>

    <div
      v-if="!entries.length"
      class="text-fg-faint italic text-sm py-6 text-center"
    >No logs yet.</div>

    <div
      v-else
      ref="listEl"
      class="space-y-1 font-mono text-xs max-h-[60vh] overflow-y-auto"
    >
      <div
        v-for="e in entries"
        :key="e._seq"
        :class="['log-entry', classFor(e.level)]"
      >
        <span class="text-fg-faint mr-2">{{ fmtTime(e.time) }}</span>
        {{ e.text || '' }}
      </div>
    </div>
  </div>
</template>
