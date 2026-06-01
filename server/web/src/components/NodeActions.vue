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
  if (!confirm(
    `Factory reset ${nodeId.value}?\n\n` +
    'The node will erase its saved configuration and reboot, ' +
    'and the server will drop all record of it. ' +
    'It will reappear in the Unadopted list on its next announcement.'
  )) return
  try {
    await ws.management('remove_node', { node_id: nodeId.value })
    message.value = 'Factory reset issued'
    emit('changed')
    router.push('/nodes')
  } catch (e) { message.value = e.message }
}
</script>

<template>
  <div class="card">
    <h3 class="text-lg font-semibold text-fg-strong mb-3 flex items-center gap-2">
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
      <hr class="border-line/50 my-1" />
      <button class="btn-danger justify-start" @click="factoryReset">
        <span class="material-icons icon-sm">delete_forever</span> Factory reset
      </button>
    </div>
    <p v-if="message" class="mt-3 text-xs text-fg-muted">{{ message }}</p>

    <FirmwareUpdateModal
      v-if="firmwareModalOpen"
      :node-id="nodeId"
      :current-version="node?.firmware_version"
      @close="firmwareModalOpen = false"
      @updated="emit('changed')"
    />
  </div>
</template>
