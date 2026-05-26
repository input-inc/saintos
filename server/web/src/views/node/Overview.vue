<script setup>
import { computed, ref } from 'vue'
import { useDisplayStore } from '@/stores/display'
import FirmwareUpdateModal from '@/components/FirmwareUpdateModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const display = useDisplayStore()

const fwUpdateAvailable = computed(() =>
  !!(props.node?.firmware_update_available && props.node?.server_firmware_version)
)
const fwTooltip = computed(() =>
  `Installed: ${props.node?.firmware_version || '—'} → Available: ${props.node?.server_firmware_version || '—'}`
)

const firmwareModalOpen = ref(false)
</script>

<template>
  <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">
    <div class="card">
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-white">Identity</h3>
        <span class="material-icons text-slate-500">badge</span>
      </div>
      <div class="grid grid-cols-2 gap-4">
        <div class="stat-item"><span class="stat-label">Node ID</span><span class="stat-value text-sm font-mono">{{ nodeId }}</span></div>
        <div class="stat-item"><span class="stat-label">Display name</span><span class="stat-value text-sm">{{ node?.display_name || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">Role</span><span class="stat-value text-sm">{{ node?.role || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">Hardware</span><span class="stat-value text-sm">{{ node?.hardware_model || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">Board</span><span class="stat-value text-sm">{{ node?.board_id || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">Chip</span><span class="stat-value text-sm">{{ node?.chip_family || '—' }}</span></div>
      </div>
    </div>

    <div class="card">
      <div class="flex items-center justify-between mb-4">
        <h3 class="text-lg font-semibold text-white">Connection</h3>
        <span :class="['px-2 py-1 text-xs font-medium rounded-full',
                       node?.online ? 'bg-emerald-500/20 text-emerald-400' : 'bg-slate-700 text-slate-400']">
          {{ node?.online ? 'Online' : 'Offline' }}
        </span>
      </div>
      <div class="grid grid-cols-2 gap-4">
        <div class="stat-item"><span class="stat-label">IP</span><span class="stat-value text-sm font-mono">{{ node?.ip_address || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">MAC</span><span class="stat-value text-sm font-mono">{{ node?.mac_address || '—' }}</span></div>
        <div class="stat-item">
          <span class="stat-label">Firmware</span>
          <span class="stat-value text-sm font-mono flex items-center gap-2">
            <span>{{ node?.firmware_version || '—' }}</span>
            <button
              v-if="fwUpdateAvailable"
              type="button"
              class="inline-flex items-center px-2 py-0.5 rounded-full text-xs font-medium bg-cyan-500/20 text-cyan-400 border border-cyan-500/30 hover:bg-cyan-500/30 transition-colors cursor-pointer"
              :title="fwTooltip"
              @click="firmwareModalOpen = true"
            >
              Update available
            </button>
          </span>
        </div>
        <div class="stat-item"><span class="stat-label">Bootloader</span><span class="stat-value text-sm font-mono">{{ node?.bootloader_version || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">CPU temp</span><span class="stat-value text-sm">{{ display.formatTemperature(node?.cpu_temp) }}</span></div>
        <div class="stat-item"><span class="stat-label">Uptime (s)</span><span class="stat-value text-sm">{{ node?.uptime_seconds ?? '—' }}</span></div>
      </div>
    </div>

    <FirmwareUpdateModal
      v-if="firmwareModalOpen"
      :node-id="nodeId"
      :current-version="node?.firmware_version"
      :node="node"
      @close="firmwareModalOpen = false"
    />
  </div>
</template>
