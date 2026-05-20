<script setup>
import { useDisplayStore } from '@/stores/display'

defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const display = useDisplayStore()
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
        <div class="stat-item"><span class="stat-label">Firmware</span><span class="stat-value text-sm font-mono">{{ node?.firmware_version || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">Bootloader</span><span class="stat-value text-sm font-mono">{{ node?.bootloader_version || '—' }}</span></div>
        <div class="stat-item"><span class="stat-label">CPU temp</span><span class="stat-value text-sm">{{ display.formatTemperature(node?.cpu_temp) }}</span></div>
        <div class="stat-item"><span class="stat-label">Uptime (s)</span><span class="stat-value text-sm">{{ node?.uptime_seconds ?? '—' }}</span></div>
      </div>
    </div>
  </div>
</template>
