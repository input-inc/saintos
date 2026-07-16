<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import AppModal from '@/components/AppModal.vue'
import NodeFilePicker from './NodeFilePicker.vue'
import { useSoundsStore } from '@/stores/sounds'

// Bulk-add one soundboard entry per audio file in a folder. Asks for the
// node + output device (like adding a single sound), then a folder on
// that node; the server creates a sound per file and skips any already
// added (dedupe by node + file path).

const props = defineProps({
  groups: { type: Array, default: () => [] },
  defaultGroup: { type: String, default: '' },
})
const emit = defineEmits(['close', 'create'])

const sounds = useSoundsStore()

const group = ref(props.defaultGroup)
const nodeId = ref('')
const folder = ref('')
const outputDevice = ref('default')
const volume = ref(1.0)
const loop = ref(false)
const infinite = ref(true)
const loopCount = ref(2)

const nodes = ref([])
const devices = ref([{ id: 'default', name: 'Node default output', is_default: true }])
const devicesLoading = ref(false)
const devicesError = ref('')
const pickerOpen = ref(false)

const canSubmit = computed(() => !!nodeId.value && !!folder.value)

async function loadDevices () {
  if (!nodeId.value) return
  devicesLoading.value = true
  devicesError.value = ''
  try {
    const data = await sounds.listDevices(nodeId.value)
    if (data.status === 'ok' && Array.isArray(data.devices)) {
      devices.value = data.devices
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

// Node changed → folder + device list belong to the old node.
watch(nodeId, () => {
  folder.value = ''
  outputDevice.value = 'default'
  devices.value = [{ id: 'default', name: 'Node default output', is_default: true }]
  loadDevices()
})

function onFolderSelected (sel) {
  folder.value = sel.path
  pickerOpen.value = false
}

function submit () {
  if (!canSubmit.value) return
  emit('create', {
    node_id: nodeId.value,
    folder: folder.value,
    output_device: outputDevice.value,
    group: group.value.trim(),
    volume: Number(volume.value),
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
  <AppModal title="Add sounds from folder" width="max-w-md" @close="emit('close')">
    <div class="space-y-3">
      <p class="text-xs text-fg-muted">
        Adds one sound per audio file in the folder, skipping any already
        added. Files stay on the node — nothing is copied.
      </p>

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
        <span class="block text-fg-muted text-xs mb-1">Folder (on the node)</span>
        <div class="flex items-center gap-2">
          <input class="input-field flex-1 text-xs font-mono" :value="folder"
                 placeholder="No folder selected" readonly />
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

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Volume: {{ Math.round(volume * 100) }}%</span>
        <input type="range" min="0" max="1" step="0.01" v-model.number="volume"
               class="w-full accent-cyan-500 cursor-pointer" />
      </label>

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
        <input class="input-field w-full" list="add-folder-groups" v-model="group"
               placeholder="Leave empty for Ungrouped" />
        <datalist id="add-folder-groups">
          <option v-for="g in groups" :key="g" :value="g" />
        </datalist>
      </label>
    </div>
    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="!canSubmit" @click="submit">Add all</button>
    </template>

    <NodeFilePicker v-if="pickerOpen" :node-id="nodeId" pick-folder
                    @close="pickerOpen = false" @select="onFolderSelected" />
  </AppModal>
</template>
