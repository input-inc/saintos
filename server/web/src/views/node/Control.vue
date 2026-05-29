<script setup>
import { computed, ref } from 'vue'
import { useRouter } from 'vue-router'
import { useWsStore } from '@/stores/ws'
import FirmwareUpdateModal from '@/components/FirmwareUpdateModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const router = useRouter()
const message = ref('')
const firmwareModalOpen = ref(false)

// Cyan "Update Firmware" button only shows when the server has a newer
// build than what's installed on the node — matches vanilla's
// updateFirmwareUpdateUI() that toggles #btn-firmware-update visibility.
const fwUpdateAvailable = computed(() =>
  !!(props.node?.firmware_update_available && props.node?.server_firmware_version)
)
const fwAvailableVersion = computed(() => props.node?.server_firmware_version || '')

async function restartNode () {
  if (!confirm('Restart this node?')) return
  try {
    await ws.management('restart_node', { node_id: props.nodeId })
    message.value = 'Restarting…'
  } catch (e) { message.value = e.message || String(e) }
}

async function identifyNode () {
  try {
    await ws.management('identify_node', { node_id: props.nodeId })
    message.value = 'Identifying…'
  } catch (e) { message.value = e.message || String(e) }
}

async function estopNode () {
  try {
    await ws.command(props.nodeId, 'estop', {})
    message.value = 'E-Stop sent'
  } catch (e) { message.value = e.message || String(e) }
}

async function updateFirmware () {
  try {
    await ws.management('update_firmware', { node_id: props.nodeId })
    message.value = 'Firmware update started'
  } catch (e) {
    // Fall back to the force-firmware modal if the server doesn't
    // support a one-shot "update to latest available" action.
    firmwareModalOpen.value = true
  }
}

async function factoryResetNode () {
  if (!confirm('Factory reset this node? All configuration will be lost.')) return
  try {
    // Mirrors vanilla app.js factoryResetNode(): calls the
    // factory_reset_node management action (no-op in current server
    // but kept for parity — vanilla behaviour is identical).
    await ws.management('factory_reset_node', { node_id: props.nodeId })
    message.value = 'Factory reset issued'
  } catch (e) { message.value = e.message || String(e) }
}

async function unadoptNode () {
  if (!confirm('Unadopt this node? It will need to be re-adopted.')) return
  try {
    await ws.management('reset_node', { node_id: props.nodeId, factory_reset: false })
    message.value = 'Unadopting…'
    router.push({ name: 'nodes' })
  } catch (e) { message.value = e.message || String(e) }
}
</script>

<template>
  <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">
    <!-- Node Commands card -->
    <div class="card">
      <h3 class="text-lg font-semibold text-fg-strong mb-4">Node Commands</h3>
      <div class="space-y-3">
        <button
          v-if="fwUpdateAvailable"
          class="btn-primary w-full justify-center"
          @click="updateFirmware"
        >
          <span class="material-icons icon-sm">system_update</span>
          <span class="update-text">Update Firmware</span>
          <span v-if="fwAvailableVersion" class="text-xs opacity-75 ml-1">{{ fwAvailableVersion }}</span>
        </button>
        <button class="btn-secondary w-full justify-center" @click="restartNode">
          <span class="material-icons icon-sm">restart_alt</span>
          Restart Node
        </button>
        <button class="btn-secondary w-full justify-center" @click="identifyNode">
          <span class="material-icons icon-sm">lightbulb</span>
          Identify (Blink LED)
        </button>
        <button class="btn-danger w-full justify-center" @click="estopNode">
          <span class="material-icons icon-sm">warning</span>
          Emergency Stop
        </button>
      </div>
    </div>

    <!-- Danger Zone card -->
    <div class="card">
      <h3 class="text-lg font-semibold text-fg-strong mb-4">Danger Zone</h3>
      <div class="space-y-3">
        <button
          class="btn-secondary w-full justify-center text-cyan-400 border-cyan-500/50 hover:bg-cyan-500/20"
          @click="firmwareModalOpen = true"
        >
          <span class="material-icons icon-sm">system_update</span>
          Force Firmware Update
        </button>
        <button
          class="btn-secondary w-full justify-center text-amber-400 border-amber-500/50 hover:bg-amber-500/20"
          @click="factoryResetNode"
        >
          <span class="material-icons icon-sm">delete_forever</span>
          Factory Reset
        </button>
        <button
          class="btn-secondary w-full justify-center text-red-400 border-red-500/50 hover:bg-red-500/20"
          @click="unadoptNode"
        >
          <span class="material-icons icon-sm">person_remove</span>
          Unadopt Node
        </button>
      </div>
    </div>

    <p v-if="message" class="lg:col-span-2 text-xs text-fg-muted">{{ message }}</p>

    <FirmwareUpdateModal
      v-if="firmwareModalOpen"
      :node-id="nodeId"
      :current-version="node?.firmware_version"
      :node="node"
      @close="firmwareModalOpen = false"
    />
  </div>
</template>
