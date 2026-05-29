<script setup>
import { ref } from 'vue'
import AppModal from '@/components/AppModal.vue'
import { useAnimationsStore } from '@/stores/animations'

const emit = defineEmits(['close'])

const animations = useAnimationsStore()

const fileInput = ref(null)
const fileBytes = ref(null)            // ArrayBuffer of last picked file
const fileName = ref('')
const sequences = ref([])              // names available in the file
const chosenSequence = ref('')
const parsedAnim = ref(null)           // Animation JSON returned by server
const channels = ref([])               // [{index, min_value, max_value, keyframe_count}]
const error = ref('')
const loading = ref(false)

function pickFile () {
  fileInput.value?.click()
}

async function onFile (e) {
  const f = e.target.files?.[0]
  e.target.value = ''
  if (!f) return
  error.value = ''
  fileName.value = f.name
  fileBytes.value = await f.arrayBuffer()
  // First call: discover sequences only (no chosenSequence).
  await runImport(null)
}

async function runImport (seqName) {
  if (!fileBytes.value) return
  loading.value = true
  error.value = ''
  try {
    const fd = new FormData()
    fd.append('file', new Blob([fileBytes.value]), fileName.value)
    if (seqName) fd.append('sequence', seqName)
    const r = await fetch('/api/animations/import/maestro', { method: 'POST', body: fd })
    if (!r.ok) {
      const body = await r.json().catch(() => ({}))
      throw new Error(body?.error || `HTTP ${r.status}`)
    }
    const data = await r.json()
    sequences.value = data.sequences || sequences.value
    parsedAnim.value = data.animation
    channels.value = data.channels || []
    chosenSequence.value = data.sequence_name
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    loading.value = false
  }
}

async function chooseSequence (name) {
  chosenSequence.value = name
  await runImport(name)
}

function renameTrack (idx, value) {
  if (parsedAnim.value) parsedAnim.value.value_tracks[idx].name = value
}
function renameTrackId (idx, value) {
  if (parsedAnim.value) parsedAnim.value.value_tracks[idx].id = value
}

async function commit () {
  if (!parsedAnim.value) return
  // Land it in the editor — operator can wire it up and save manually,
  // which gives them a chance to adjust before persisting.
  animations.editing = parsedAnim.value
  animations.dirty = true
  emit('close')
}
</script>

<template>
  <AppModal title="Import Maestro animation" width="max-w-3xl" @close="emit('close')">
    <div class="space-y-4">
      <div>
        <p class="text-sm text-fg">
          Upload a Pololu Maestro `.xml` save file. Each Maestro sequence becomes a
          new animation with one track per servo channel; pulse widths (in
          microseconds) become keyframe values. Rename tracks before saving — the
          imported channel numbers are just placeholders.
        </p>
      </div>

      <div class="flex items-center gap-3">
        <button class="btn-primary" :disabled="loading" @click="pickFile">
          <span class="material-icons icon-sm">upload_file</span>
          {{ fileName ? 'Replace file' : 'Choose file' }}
        </button>
        <span v-if="fileName" class="text-sm text-fg-muted">{{ fileName }}</span>
        <input ref="fileInput" type="file" accept=".xml" class="hidden" @change="onFile" />
      </div>

      <div v-if="error" class="p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">
        {{ error }}
      </div>

      <div v-if="sequences.length > 1">
        <label class="text-sm block">
          <span class="block text-fg-muted mb-1">Sequence</span>
          <select class="input-field" v-model="chosenSequence" @change="chooseSequence(chosenSequence)">
            <option v-for="s in sequences" :key="s" :value="s">{{ s }}</option>
          </select>
        </label>
      </div>

      <div v-if="parsedAnim" class="space-y-3">
        <div class="rounded-lg border border-line/50 p-3 bg-panel/30">
          <div class="text-sm text-fg-strong font-semibold mb-1">{{ parsedAnim.name }}</div>
          <div class="text-xs text-fg-muted">
            {{ parsedAnim.duration.toFixed(2) }}s ·
            {{ parsedAnim.value_tracks.length }} track{{ parsedAnim.value_tracks.length === 1 ? '' : 's' }} ·
            {{ parsedAnim.fps }} fps
          </div>
        </div>

        <div>
          <h4 class="text-sm font-semibold text-fg-strong mb-2">Tracks</h4>
          <ul class="space-y-2">
            <li v-for="(t, idx) in parsedAnim.value_tracks" :key="t.id"
                class="flex items-center gap-2 p-2 rounded bg-canvas border border-line/40">
              <input class="input-field w-28 text-xs" :value="t.id"
                     @input="renameTrackId(idx, $event.target.value)" placeholder="id" />
              <input class="input-field flex-1" :value="t.name"
                     @input="renameTrack(idx, $event.target.value)" placeholder="Track name" />
              <span class="text-xs text-fg-muted w-32 text-right tabular-nums">
                {{ channels[idx]?.min_value?.toFixed(0) }}–{{ channels[idx]?.max_value?.toFixed(0) }} µs
                · {{ t.curve.keys.length }} keys
              </span>
            </li>
          </ul>
        </div>
      </div>
    </div>

    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="!parsedAnim" @click="commit">
        Open in editor
      </button>
    </template>
  </AppModal>
</template>
