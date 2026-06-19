<script setup>
import { computed, ref } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import { useDisplayStore } from '@/stores/display'
import FirmwareUpdateModal from '@/components/FirmwareUpdateModal.vue'
import FirmwareUpdateProgress from '@/components/FirmwareUpdateProgress.vue'
import NodeEditModal from '@/components/NodeEditModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const display = useDisplayStore()
const nodes = useNodesStore()

const fwUpdateAvailable = computed(() =>
  !!(props.node?.firmware_update_available && props.node?.server_firmware_version)
)
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

function formatLastSeen (ts) {
  if (!ts) return '--'
  try { return new Date(ts * 1000).toLocaleString() } catch (_) { return '--' }
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
      <div class="grid grid-cols-2 gap-4">
        <div class="stat-item">
          <span class="stat-label">Node ID</span>
          <span class="stat-value text-sm font-mono">{{ nodeId }}</span>
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
            <button
              v-if="fwUpdateAvailable"
              type="button"
              class="px-2 py-0.5 text-xs font-medium rounded-full bg-cyan-500/20 text-cyan-400 border border-cyan-500/30 cursor-pointer hover:bg-cyan-500/30"
              :title="fwTooltip"
              @click="firmwareModalOpen = true"
            >
              Update Available
            </button>
          </div>
          <span class="text-xs text-fg-faint block">{{ node?.firmware_build ? `Built: ${node.firmware_build}` : '' }}</span>
          <!-- OTA progress strip — renders only while an update is
               in flight for this node. Driven by the firmwareUpdates
               store (subscribes to update_progress/<node_id> on the
               WS broadcast that bridges the node's ROS publication). -->
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

    <!-- Status card -->
    <div class="card">
      <h3 class="text-lg font-semibold text-fg-strong mb-4">Status</h3>
      <div class="space-y-4">
        <div class="stat-item">
          <span class="stat-label">CPU Temperature</span>
          <span class="stat-value">{{ display.formatTemperature(node?.cpu_temp) }}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">State</span>
          <span class="stat-value">{{ node?.state || 'Unknown' }}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">Last Seen</span>
          <span class="stat-value text-sm">{{ formatLastSeen(node?.last_seen) }}</span>
        </div>
      </div>
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
