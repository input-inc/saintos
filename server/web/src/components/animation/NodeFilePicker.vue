<script setup>
import { onMounted, ref } from 'vue'
import AppModal from '@/components/AppModal.vue'
import { useSoundsStore } from '@/stores/sounds'

// Browse a node's own filesystem to pick an audio file. The listing is a
// round-trip to the node (see soundboard.py `list_directory`); we start
// at `/` and let the operator navigate anywhere. Audio files are
// selectable; other files are shown greyed unless "show all" is on.

const props = defineProps({
  nodeId: { type: String, required: true },
  start: { type: String, default: '/' },
  // Folder mode: navigate into directories and pick the current one
  // ("Use this folder"); files are shown for context but not selectable.
  pickFolder: { type: Boolean, default: false },
})
const emit = defineEmits(['close', 'select'])

const sounds = useSoundsStore()

const path = ref(props.start || '/')
const parent = ref('/')
const entries = ref([])
const loading = ref(false)
const error = ref('')
const showAll = ref(false)

async function browse (target) {
  loading.value = true
  error.value = ''
  try {
    const data = await sounds.browseDir(props.nodeId, target)
    if (data.status === 'ok') {
      path.value = data.path || target
      parent.value = data.parent || '/'
      entries.value = Array.isArray(data.entries) ? data.entries : []
    } else {
      error.value = data.message || 'Could not read directory'
    }
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    loading.value = false
  }
}

function open (entry) {
  if (entry.is_dir) {
    browse(path.value.replace(/\/$/, '') + '/' + entry.name)
  } else if (!props.pickFolder && (entry.is_audio || showAll.value)) {
    emit('select', {
      path: path.value.replace(/\/$/, '') + '/' + entry.name,
      name: entry.name,
    })
  }
}

function useThisFolder () {
  const p = path.value.replace(/\/$/, '') || '/'
  emit('select', { path: p, name: p.split('/').pop() || p, is_folder: true })
}

function fmtSize (bytes) {
  if (!bytes) return ''
  const u = ['B', 'KB', 'MB', 'GB']
  let i = 0; let n = bytes
  while (n >= 1024 && i < u.length - 1) { n /= 1024; i++ }
  return `${n.toFixed(i ? 1 : 0)} ${u[i]}`
}

onMounted(() => browse(props.start || '/'))
</script>

<template>
  <AppModal :title="pickFolder ? 'Pick a folder on the node' : 'Pick a file on the node'"
            width="max-w-lg" @close="emit('close')">
    <div class="space-y-2">
      <div class="flex items-center gap-2">
        <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                title="Up one level"
                :disabled="loading || path === '/'"
                @click="browse(parent)">
          <span class="material-icons icon-sm">arrow_upward</span>
        </button>
        <div class="flex-1 min-w-0 text-xs font-mono text-fg-muted truncate" :title="path">
          {{ path }}
        </div>
        <label v-if="!pickFolder" class="flex items-center gap-1 text-xs text-fg-muted shrink-0 cursor-pointer">
          <input type="checkbox" v-model="showAll" class="accent-cyan-500" />
          Show all files
        </label>
      </div>

      <p v-if="error" class="text-xs text-amber-300 italic">{{ error }}</p>
      <p v-if="loading" class="text-xs text-fg-faint italic">Loading…</p>

      <div v-if="!loading" class="rounded-lg border border-line/50 bg-panel/30 divide-y divide-line/40 max-h-[50vh] overflow-y-auto">
        <button v-for="e in entries" :key="e.name"
                type="button"
                :disabled="pickFolder ? !e.is_dir : (!e.is_dir && !e.is_audio && !showAll)"
                :class="['w-full flex items-center gap-2 px-3 py-1.5 text-left text-sm transition-colors',
                         (e.is_dir || (!pickFolder && (e.is_audio || showAll)))
                           ? 'hover:bg-panel/60 text-fg-strong'
                           : 'text-fg-faint cursor-not-allowed']"
                @click="open(e)">
          <span class="material-icons icon-sm">
            {{ e.is_dir ? 'folder' : (e.is_audio ? 'music_note' : 'draft') }}
          </span>
          <span class="flex-1 truncate">{{ e.name }}</span>
          <span v-if="!e.is_dir" class="text-xs text-fg-faint tabular-nums shrink-0">
            {{ fmtSize(e.size) }}
          </span>
        </button>
        <div v-if="!entries.length" class="text-center text-xs text-fg-faint py-6">
          Empty directory.
        </div>
      </div>
    </div>
    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button v-if="pickFolder" class="btn-primary" :disabled="loading"
              @click="useThisFolder">
        Use this folder
      </button>
    </template>
  </AppModal>
</template>
