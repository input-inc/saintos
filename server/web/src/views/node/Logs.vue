<script setup>
import { computed, nextTick, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useWsTopic } from '@/composables/useWsTopic'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const history = ref([])           // entries fetched on mount
const live = useWsTopic(() => `node_log/${props.nodeId}`)
const listEl = ref(null)
const MAX_ROWS = 500

const entries = computed(() => {
  const out = [...history.value]
  // node_log broadcasts arrive as single entries via the topic feed.
  if (live.value && typeof live.value === 'object' && !Array.isArray(live.value)) {
    out.push(live.value)
  }
  return out.slice(-MAX_ROWS)
})

async function loadHistory () {
  history.value = []
  try {
    const r = await ws.management('get_node_logs', { node_id: props.nodeId })
    history.value = (r && r.entries) || []
  } catch (e) {
    console.warn('Failed to load logs:', e)
  }
}

onMounted(loadHistory)
watch(() => props.nodeId, loadHistory)

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
        v-for="(e, i) in entries"
        :key="i"
        :class="['log-entry', classFor(e.level)]"
      >
        <span class="text-fg-faint mr-2">{{ fmtTime(e.time) }}</span>
        {{ e.text || '' }}
      </div>
    </div>
  </div>
</template>
