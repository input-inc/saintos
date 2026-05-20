<script setup>
import { computed, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useRouter } from 'vue-router'
import FirmwareUpdateModal from './FirmwareUpdateModal.vue'

const props = defineProps({
  node: { type: Object, default: null },
})
const emit = defineEmits(['changed'])

const ws = useWsStore()
const router = useRouter()
const firmwareModalOpen = ref(false)
const message = ref('')

const nodeId = computed(() => props.node?.node_id)

async function restart () {
  if (!confirm('Restart this node?')) return
  try { await ws.management('restart_node', { node_id: nodeId.value }); message.value = 'Restart sent' }
  catch (e) { message.value = e.message }
}
async function identify () {
  try { await ws.management('identify_node', { node_id: nodeId.value }); message.value = 'Identifying…' }
  catch (e) { message.value = e.message }
}
async function estop () {
  try { await ws.command(nodeId.value, 'estop', {}); message.value = 'E-stop sent' }
  catch (e) { message.value = e.message }
}
async function factoryReset () {
  if (!confirm(`Factory reset ${nodeId.value}?\n\nThis wipes the node's saved configuration.`)) return
  try { await ws.management('factory_reset_node', { node_id: nodeId.value }); message.value = 'Factory reset issued' }
  catch (e) { message.value = e.message }
}
async function unadopt () {
  if (!confirm(`Unadopt ${nodeId.value}?\n\nThe node returns to the unadopted pool.`)) return
  try {
    await ws.management('reset_node', { node_id: nodeId.value, factory_reset: false })
    emit('changed')
    router.push('/nodes')
  } catch (e) { message.value = e.message }
}
async function remove () {
  if (!confirm(`Remove ${nodeId.value} from the server?`)) return
  try {
    await ws.management('remove_node', { node_id: nodeId.value })
    emit('changed')
    router.push('/nodes')
  } catch (e) { message.value = e.message }
}
</script>

<template>
  <div class="card">
    <h3 class="text-lg font-semibold text-white mb-3 flex items-center gap-2">
      <span class="material-icons text-cyan-400">build</span>
      Actions
    </h3>
    <div class="grid grid-cols-1 gap-2">
      <button class="btn-secondary justify-start" @click="identify">
        <span class="material-icons icon-sm">lightbulb</span> Identify
      </button>
      <button class="btn-secondary justify-start" @click="restart">
        <span class="material-icons icon-sm">restart_alt</span> Restart
      </button>
      <button class="btn-danger justify-start" @click="estop">
        <span class="material-icons icon-sm">warning</span> E-Stop
      </button>
      <button class="btn-primary justify-start" @click="firmwareModalOpen = true">
        <span class="material-icons icon-sm">system_update</span> Update firmware
      </button>
      <hr class="border-slate-700/50 my-1" />
      <button class="btn-secondary justify-start" @click="factoryReset">
        <span class="material-icons icon-sm">settings_backup_restore</span> Factory reset
      </button>
      <button class="btn-secondary justify-start" @click="unadopt">
        <span class="material-icons icon-sm">logout</span> Unadopt
      </button>
      <button class="btn-danger justify-start" @click="remove">
        <span class="material-icons icon-sm">delete</span> Remove
      </button>
    </div>
    <p v-if="message" class="mt-3 text-xs text-slate-400">{{ message }}</p>

    <FirmwareUpdateModal
      v-if="firmwareModalOpen"
      :node-id="nodeId"
      :current-version="node?.firmware_version"
      @close="firmwareModalOpen = false"
      @updated="emit('changed')"
    />
  </div>
</template>
