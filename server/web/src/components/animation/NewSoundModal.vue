<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import AppModal from '@/components/AppModal.vue'
import IconPicker from './IconPicker.vue'
import NodeFilePicker from './NodeFilePicker.vue'
import { useSoundsStore } from '@/stores/sounds'

// Create a soundboard entry. A sound is bound to one node: pick the node,
// browse that node's filesystem for the file, then pick that node's audio
// output. Volume / start-time / loop options mirror the play command.

const props = defineProps({
  groups: { type: Array, default: () => [] },
  defaultGroup: { type: String, default: '' },
})
const emit = defineEmits(['close', 'create'])

const sounds = useSoundsStore()

const name = ref('')
const group = ref(props.defaultGroup)
const icon = ref('volume_up')
const nodeId = ref('')
const filePath = ref('')
const outputDevice = ref('default')
const volume = ref(1.0)
const startTime = ref(0.0)
const loop = ref(false)
const infinite = ref(true)
const loopCount = ref(2)

const nodes = ref([])
const devices = ref([{ id: 'default', name: 'Node default output', is_default: true }])
const devicesLoading = ref(false)
const devicesError = ref('')
const pickerOpen = ref(false)

const canSubmit = computed(() =>
  name.value.trim().length > 0 && nodeId.value && filePath.value)

const fileName = computed(() => {
  if (!filePath.value) return ''
  const parts = filePath.value.split('/')
  return parts[parts.length - 1] || filePath.value
})

async function loadDevices () {
  if (!nodeId.value) return
  devicesLoading.value = true
  devicesError.value = ''
  try {
    const data = await sounds.listDevices(nodeId.value)
    if (data.status === 'ok' && Array.isArray(data.devices)) {
      devices.value = data.devices
      // Keep the current selection if still valid, else default.
      if (!devices.value.some(d => d.id === outputDevice.value)) {
        outputDevice.value = 'default'
      }
    } else {
      devicesError.value = data.message || 'Could not list audio devices'
    }
  } catch (e) {
    devicesError.value = e.message || String(e)
  } finally {
    devicesLoading.value = false
  }
}

// Node changed → the file path and device list belong to the old node,
// so clear/refresh them.
watch(nodeId, () => {
  filePath.value = ''
  outputDevice.value = 'default'
  devices.value = [{ id: 'default', name: 'Node default output', is_default: true }]
  loadDevices()
})

function onFileSelected (sel) {
  filePath.value = sel.path
  if (!name.value.trim()) {
    // Seed the name from the filename (sans extension) as a convenience.
    name.value = (sel.name || '').replace(/\.[^.]+$/, '')
  }
  pickerOpen.value = false
}

function submit () {
  if (!canSubmit.value) return
  emit('create', {
    name: name.value.trim(),
    group: group.value.trim(),
    icon: icon.value,
    node_id: nodeId.value,
    file_path: filePath.value,
    output_device: outputDevice.value,
    volume: Number(volume.value),
    start_time: Number(startTime.value),
    loop: loop.value,
    loop_count: loop.value && !infinite.value ? Math.max(1, Number(loopCount.value)) : 0,
  })
}

onMounted(async () => {
  nodes.value = await sounds.listNodes()
  if (nodes.value.length === 1) nodeId.value = nodes.value[0].node_id
})
</script>

<template>
  <AppModal title="New sound" width="max-w-md" @close="emit('close')">
    <div class="space-y-3">
      <div class="flex items-center gap-2">
        <IconPicker v-model="icon" fallback="volume_up" />
        <label class="flex-1">
          <span class="block text-fg-muted text-xs mb-1">Name</span>
          <input class="input-field w-full" v-model="name"
                 placeholder="Fanfare" autofocus @keydown.enter="submit" />
        </label>
      </div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Node</span>
        <select class="input-field w-full" v-model="nodeId">
          <option value="" disabled>Select a node…</option>
          <option v-for="n in nodes" :key="n.node_id" :value="n.node_id">
            {{ n.name }}{{ n.online ? '' : ' (offline)' }}
          </option>
        </select>
        <p v-if="!nodes.length" class="text-xs text-amber-300 italic mt-1">
          No audio-capable (Raspberry&nbsp;Pi) nodes are adopted.
        </p>
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">File (on the node)</span>
        <div class="flex items-center gap-2">
          <input class="input-field flex-1 text-xs font-mono" :value="filePath"
                 placeholder="No file selected" readonly />
          <button class="btn-sm bg-surface hover:bg-cyan-600 text-fg-strong shrink-0"
                  type="button" :disabled="!nodeId" @click="pickerOpen = true">
            <span class="material-icons icon-sm">folder_open</span>
            Browse
          </button>
        </div>
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Output device</span>
        <select class="input-field w-full" v-model="outputDevice" :disabled="!nodeId">
          <option v-for="d in devices" :key="d.id" :value="d.id">{{ d.name }}</option>
        </select>
        <p v-if="devicesLoading" class="text-xs text-fg-faint italic mt-1">Loading devices…</p>
        <p v-else-if="devicesError" class="text-xs text-amber-300 italic mt-1">{{ devicesError }}</p>
      </label>

      <div class="grid grid-cols-2 gap-3">
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Volume: {{ Math.round(volume * 100) }}%</span>
          <input type="range" min="0" max="1" step="0.01" v-model.number="volume"
                 class="w-full accent-cyan-500 cursor-pointer" />
        </label>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Start time (s)</span>
          <input class="input-field w-full" type="number" min="0" step="0.1"
                 v-model.number="startTime" />
        </label>
      </div>

      <div class="space-y-2">
        <label class="flex items-center gap-2 text-sm text-fg-strong cursor-pointer">
          <input type="checkbox" v-model="loop" class="accent-cyan-500" />
          Loop
        </label>
        <div v-if="loop" class="pl-6 space-y-2">
          <label class="flex items-center gap-2 text-sm text-fg-muted cursor-pointer">
            <input type="checkbox" v-model="infinite" class="accent-cyan-500" />
            Loop forever
          </label>
          <label v-if="!infinite" class="flex items-center gap-2 text-sm text-fg-muted">
            Play
            <input class="input-field w-20" type="number" min="1" step="1"
                   v-model.number="loopCount" />
            times
          </label>
        </div>
      </div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Group (optional)</span>
        <input class="input-field w-full" list="new-sound-groups" v-model="group"
               placeholder="Leave empty for Ungrouped" />
        <datalist id="new-sound-groups">
          <option v-for="g in groups" :key="g" :value="g" />
        </datalist>
      </label>
    </div>
    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="!canSubmit" @click="submit">Create</button>
    </template>

    <NodeFilePicker v-if="pickerOpen" :node-id="nodeId"
                    @close="pickerOpen = false" @select="onFileSelected" />
  </AppModal>
</template>
