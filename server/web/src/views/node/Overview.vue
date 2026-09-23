<script setup>
import { computed, ref } from 'vue'
import { useRouter } from 'vue-router'
import { useNodesStore } from '@/stores/nodes'
import { useWsStore } from '@/stores/ws'
import FirmwareUpdateModal from '@/components/FirmwareUpdateModal.vue'
import FirmwareUpdateProgress from '@/components/FirmwareUpdateProgress.vue'
import { useFirmwareUpdatesStore } from '@/stores/firmwareUpdates'
import NodeEditModal from '@/components/NodeEditModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const nodes = useNodesStore()
const ws = useWsStore()
const firmwareUpdates = useFirmwareUpdatesStore()
const router = useRouter()

// Node actions used to live on their own Control tab. They're here now:
// a whole tab for six buttons wasn't paying for itself, and the
// destructive ones sat a tab away from the identity they act on.
// CPU/state/last-seen moved the other way — up into the page header, so
// they follow you across tabs (see NodeDetail.vue).
const message = ref('')

const fwUpdateAvailable = computed(() =>
  !!(props.node?.firmware_update_available && props.node?.server_firmware_version)
)
// Same store the progress strip reads, so the version block and the
// progress it shows can never disagree about whether this node is
// updating.
const updateInFlight = computed(
  () => firmwareUpdates.isUpdating(props.nodeId))

const fwTooltip = computed(() =>
  `Installed: ${props.node?.firmware_version || '—'} → Available: ${props.node?.server_firmware_version || '—'}`
)

const firmwareModalOpen = ref(false)
const editModalOpen = ref(false)

// Matches vanilla formatUptime() in js/app.js.
function formatUptime (seconds) {
  if (!seconds) return '--'
  const days = Math.floor(seconds / 86400)
  const hours = Math.floor((seconds % 86400) / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)
  if (days > 0) return `${days}d ${hours}h ${minutes}m`
  if (hours > 0) return `${hours}h ${minutes}m`
  return `${minutes}m`
}

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

async function factoryResetNode () {
  if (!confirm(
    'Factory reset this node?\n\n' +
    'The node will erase its saved configuration and reboot, ' +
    'and the server will drop all record of it. ' +
    'It will reappear in the Unadopted list on its next announcement.'
  )) return
  try {
    await ws.management('remove_node', { node_id: props.nodeId })
    message.value = 'Factory reset issued'
    router.push({ name: 'nodes' })
  } catch (e) { message.value = e.message || String(e) }
}
</script>

<template>
  <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">
    <!-- Node Information card -->
    <div class="card">
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-fg-strong">Node Information</h3>
        <button class="btn-secondary text-sm" @click="editModalOpen = true">
          <span class="material-icons icon-sm">edit</span>
          Edit
        </button>
      </div>
      <div class="grid grid-cols-1 sm:grid-cols-2 gap-4">
        <div class="stat-item min-w-0">
          <span class="stat-label">Node ID</span>
          <span class="stat-value text-sm font-mono break-all">{{ nodeId }}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">Role</span>
          <span class="stat-value">{{ node?.role || '--' }}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">Hardware</span>
          <span class="stat-value text-sm">{{ node?.hardware_model || 'Unknown' }}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">Firmware</span>
          <div class="flex items-center gap-2">
            <span class="stat-value text-sm">{{ node?.firmware_version || '--' }}</span>
            <!-- Hidden while an update is in flight: the progress
                 strip directly below is the live state, and offering
                 "Update Available" beside it invites starting a second
                 update on a node already mid-flight. -->
            <button
              v-if="fwUpdateAvailable && !updateInFlight"
              type="button"
              class="px-2 py-0.5 text-xs font-medium rounded-full bg-cyan-500/20 text-cyan-400 border border-cyan-500/30 cursor-pointer hover:bg-cyan-500/30"
              :title="fwTooltip"
              @click="firmwareModalOpen = true"
            >
              Update Available
            </button>
          </div>
          <span class="text-xs text-fg-faint block">{{ node?.firmware_build ? `Built: ${node.firmware_build}` : '' }}</span>
          <!-- Firmware lives HERE and only here. The Actions card used
               to carry a second "Update Firmware" button, so the same
               operation appeared twice on one screen with two different
               behaviours. Keep the version, the update affordance and
               the progress for that update in one block — "Force
               Firmware Update" stays in the danger zone because it is a
               different, deliberate action (pick a build, override the
               version check), not a duplicate of this one.

               OTA progress strip — renders only while an update is in
               flight for this node. Driven by the firmwareUpdates store
               (subscribes to update_progress/<node_id> on the WS
               broadcast that bridges the node's ROS publication). -->
          <FirmwareUpdateProgress :node-id="nodeId" variant="panel" />
        </div>
        <div class="stat-item">
          <span class="stat-label">Bootloader</span>
          <span class="stat-value text-sm">{{ node?.bootloader_version || 'unknown' }}</span>
          <span class="text-xs text-fg-faint block">Not OTA-updatable</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">IP Address</span>
          <span class="stat-value text-sm font-mono">{{ node?.ip_address || '--' }}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">Uptime</span>
          <span class="stat-value text-sm">{{ formatUptime(node?.uptime_seconds) }}</span>
        </div>
      </div>
    </div>

    <!-- Node actions. Moved here from the old Control tab. -->
    <div class="card">
      <h3 class="text-lg font-semibold text-fg-strong mb-4">Actions</h3>
      <div class="space-y-3">
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

      <!-- Irreversible actions, kept visually separate from the routine
           ones above so a mis-click can't wander into a factory reset. -->
      <div class="mt-5 pt-4 border-t border-line">
        <h4 class="text-xs uppercase tracking-wide text-fg-faint mb-3">Danger zone</h4>
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
        </div>
      </div>

      <p v-if="message" class="mt-3 text-xs text-fg-muted">{{ message }}</p>
    </div>

    <FirmwareUpdateModal
      v-if="firmwareModalOpen"
      :node-id="nodeId"
      :current-version="node?.firmware_version"
      :node="node"
      @close="firmwareModalOpen = false"
    />

    <NodeEditModal
      v-if="editModalOpen && node"
      :node="node"
      @close="editModalOpen = false"
      @updated="nodes.fetchAll()"
    />
  </div>
</template>
