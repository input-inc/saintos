<script setup>
import { computed, onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useNodesStore } from '@/stores/nodes'

defineProps({ embedded: { type: Boolean, default: false } })

const ws = useWsStore()
const nodes = useNodesStore()
const moods = ref([])
const selected = ref(null)
const parsed = ref(null)
const loading = ref(false)
const error = ref('')

const headNode = computed(() =>
  nodes.all.find(n => (n.role || '').toLowerCase().includes('head') ||
                       (n.display_name || '').toLowerCase().includes('head'))
)

async function refresh () {
  loading.value = true
  error.value = ''
  try {
    const r = await ws.send('file', 'list', { pattern: 'DA_*.uasset' })
    moods.value = r?.files || []
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    loading.value = false
  }
}

const uploading = ref(false)
const uploadStatus = ref('')

function readBase64 (file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader()
    reader.onload = () => resolve(reader.result.split(',', 2)[1] || '')
    reader.onerror = () => reject(reader.error || new Error('read failed'))
    reader.readAsDataURL(file)
  })
}

async function onFile (evt) {
  const file = evt.target.files?.[0]
  if (!file) return
  uploading.value = true
  uploadStatus.value = `Uploading ${file.name}…`
  try {
    const content = await readBase64(file)
    const r = await ws.send('file', 'parse', {
      filename: file.name, content, category: 'moods',
    })
    uploadStatus.value = `Uploaded ${file.name}`
    await refresh()
    if (r?.path) selectMood({ path: r.path, name: file.name })
  } catch (e) {
    uploadStatus.value = `Failed: ${e.message || e}`
    error.value = e.message || String(e)
  } finally {
    uploading.value = false
    evt.target.value = ''
  }
}

async function selectMood (m) {
  selected.value = m.path
  parsed.value = null
  try {
    const r = await ws.send('file', 'parse', { path: m.path })
    parsed.value = r || null
  } catch (e) {
    error.value = e.message || String(e)
  }
}

async function applyMood () {
  if (!selected.value || !headNode.value) return
  try {
    await ws.send('ros', 'publish', {
      endpoint: `/saint/${headNode.value.node_id}/mood`,
      data: { path: selected.value },
    })
  } catch (e) {
    error.value = e.message || String(e)
  }
}

function prettyName (filename) {
  return (filename || '').replace(/^DA_/, '').replace(/\.uasset$/, '')
}

onMounted(() => { nodes.fetchAll().catch(() => {}); refresh() })
</script>

<template>
  <section>
    <div v-if="!embedded" class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-white">Moods</h2>
      <button class="btn-secondary" @click="refresh"><span class="material-icons icon-sm">refresh</span>Refresh</button>
    </div>
    <div v-else class="flex items-center justify-end mb-3">
      <button class="btn-secondary text-sm" @click="refresh"><span class="material-icons icon-sm">refresh</span>Refresh</button>
    </div>

    <div v-if="error" class="mb-4 p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">{{ error }}</div>

    <div class="card mb-4 flex items-center gap-3">
      <input type="file" accept=".uasset" :disabled="uploading" class="hidden" id="mood-file-input" @change="onFile" />
      <label for="mood-file-input" class="btn-primary cursor-pointer">
        <span class="material-icons icon-sm">upload</span>
        Upload .uasset
      </label>
      <span class="text-sm text-slate-400">{{ uploadStatus || 'Drag a DA_*.uasset file or click upload.' }}</span>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-3 gap-6">
      <div class="card lg:col-span-1">
        <h3 class="text-lg font-semibold text-white mb-3">Available moods</h3>
        <div v-if="loading" class="text-sm text-slate-400">Loading…</div>
        <div v-else-if="!moods.length" class="text-center py-6">
          <span class="material-icons text-slate-600" style="font-size: 32px;">sentiment_neutral</span>
          <p class="text-slate-500 text-sm mt-2">No mood files found.</p>
        </div>
        <ul v-else class="space-y-1">
          <li
            v-for="m in moods"
            :key="m.path"
            :class="['flex items-center justify-between px-2 py-1.5 rounded text-sm cursor-pointer',
                     selected === m.path ? 'bg-cyan-500/20 text-cyan-300' : 'hover:bg-slate-700/50 text-slate-300']"
            @click="selectMood(m)"
          >
            <span>{{ prettyName(m.name) }}</span>
            <span class="text-xs text-slate-500">{{ (m.size / 1024).toFixed(1) }}k</span>
          </li>
        </ul>
      </div>

      <div class="card lg:col-span-2">
        <div class="flex items-center justify-between mb-3">
          <h3 class="text-lg font-semibold text-white">{{ selected ? prettyName(selected.split('/').pop()) : 'Mood detail' }}</h3>
          <button class="btn-primary" :disabled="!selected || !headNode" @click="applyMood">
            <span class="material-icons icon-sm">play_arrow</span>
            Apply to {{ headNode?.display_name || 'head' }}
          </button>
        </div>
        <div v-if="!selected" class="text-sm text-slate-400 italic">Select a mood to view its parsed properties.</div>
        <pre v-else-if="parsed" class="text-xs font-mono text-slate-300 bg-slate-900 rounded p-3 overflow-x-auto max-h-96">{{ JSON.stringify(parsed, null, 2) }}</pre>
        <div v-else class="text-sm text-slate-400">Parsing…</div>
      </div>
    </div>
  </section>
</template>
