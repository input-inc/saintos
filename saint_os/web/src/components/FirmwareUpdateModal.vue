<script setup>
import { onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import AppModal from './AppModal.vue'

const props = defineProps({
  nodeId:         { type: String, required: true },
  currentVersion: { type: String, default: '' },
})
const emit = defineEmits(['close', 'updated'])

const ws = useWsStore()
const builds = ref([])
const selected = ref(null)
const force = ref(false)
const sending = ref(false)
const error = ref('')

async function load () {
  try {
    const r = await ws.management('get_firmware_builds', {})
    builds.value = r?.builds || []
  } catch (e) { error.value = e.message || String(e) }
}
onMounted(load)

async function send () {
  if (sending.value) return
  sending.value = true
  error.value = ''
  try {
    if (force.value && selected.value) {
      await ws.management('force_firmware_update', {
        node_id: props.nodeId,
        build_type: selected.value,
      })
    } else {
      await ws.management('update_firmware', { node_id: props.nodeId })
    }
    emit('updated')
    emit('close')
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    sending.value = false
  }
}
</script>

<template>
  <AppModal title="Update firmware" width="max-w-xl" @close="emit('close')">
    <div v-if="error" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">{{ error }}</div>

    <p class="text-sm text-slate-400 mb-4">
      Sends a firmware update to <code class="text-cyan-300 font-mono">{{ nodeId }}</code>.
      Current version: <span class="font-mono text-slate-300">{{ currentVersion || '—' }}</span>.
    </p>

    <div class="space-y-3">
      <div class="card bg-slate-900/60 border-slate-700">
        <h4 class="text-sm font-semibold text-white mb-2">Standard update</h4>
        <p class="text-xs text-slate-400">Server picks the matching build for this node's chip family and pushes it. Use this for routine updates.</p>
      </div>

      <div class="card bg-slate-900/60 border-slate-700">
        <h4 class="text-sm font-semibold text-white mb-2">Force a specific build</h4>
        <p class="text-xs text-slate-400 mb-3">Useful when reflashing to a different build type (sim/hw/etc).</p>
        <label class="flex items-center gap-2 mb-3 text-sm text-slate-300">
          <input v-model="force" type="checkbox" class="rounded bg-slate-700 border-slate-600" />
          Force build
        </label>
        <select v-model="selected" :disabled="!force" class="input-field w-full">
          <option :value="null">-- Available builds --</option>
          <option v-for="b in builds" :key="b.type" :value="b.type">
            {{ b.type }} · {{ b.version || '—' }}
          </option>
        </select>
      </div>
    </div>

    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="sending || (force && !selected)" @click="send">
        <span class="material-icons icon-sm">system_update</span>
        {{ sending ? 'Sending…' : (force ? 'Force update' : 'Update') }}
      </button>
    </template>
  </AppModal>
</template>
