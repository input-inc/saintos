<script setup>
import { computed, nextTick, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useActivityStore } from '@/stores/activity'

const ws = useWsStore()
const activity = useActivityStore()
const history = ref([])
const listEl = ref(null)
const filter = ref('')
const levels = ref({ info: true, warn: true, error: true, debug: false })

const entries = computed(() => {
  const merged = [...history.value, ...activity.entries]
  const f = filter.value.toLowerCase()
  return merged
    .filter(e => levels.value[(e.level || 'info').toLowerCase()] !== false)
    .filter(e => !f || (e.text || '').toLowerCase().includes(f))
    .slice(-500)
})

async function loadHistory () {
  try {
    const r = await ws.management('get_logs', { limit: 200 })
    history.value = (r && r.entries) || []
  } catch (e) { console.warn('get_logs failed:', e) }
}
onMounted(loadHistory)

watch(entries, async () => {
  await nextTick()
  if (listEl.value) listEl.value.scrollTop = listEl.value.scrollHeight
})

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
  <section>
    <div class="flex items-center justify-between gap-3 mb-4 flex-wrap">
      <h2 class="text-2xl font-bold text-white">System Logs</h2>
      <div class="flex items-center gap-3 flex-wrap">
        <div class="flex items-center gap-2 text-xs">
          <label class="flex items-center gap-1"><input v-model="levels.info"  type="checkbox" class="rounded bg-slate-700 border-slate-600" />info</label>
          <label class="flex items-center gap-1"><input v-model="levels.warn"  type="checkbox" class="rounded bg-slate-700 border-slate-600" />warn</label>
          <label class="flex items-center gap-1"><input v-model="levels.error" type="checkbox" class="rounded bg-slate-700 border-slate-600" />error</label>
          <label class="flex items-center gap-1"><input v-model="levels.debug" type="checkbox" class="rounded bg-slate-700 border-slate-600" />debug</label>
        </div>
        <input v-model="filter" type="text" class="input-field" placeholder="Filter…" />
      </div>
    </div>

    <div class="card">
      <div
        v-if="!entries.length"
        class="text-slate-500 italic text-sm py-6 text-center"
      >No log entries{{ filter ? ' match the filter' : '' }}.</div>

      <div v-else ref="listEl" class="space-y-1 font-mono text-xs max-h-[70vh] overflow-y-auto">
        <div v-for="(e, i) in entries" :key="i" :class="['log-entry', classFor(e.level)]">
          <span class="text-slate-500 mr-2">{{ fmtTime(e.time) }}</span>
          {{ e.text || '' }}
        </div>
      </div>
    </div>
  </section>
</template>
