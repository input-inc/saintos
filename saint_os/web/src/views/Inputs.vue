<script setup>
import { computed } from 'vue'
import { useWsTopic } from '@/composables/useWsTopic'

const livelink = useWsTopic(() => 'livelink')
const clients = useWsTopic(() => 'clients')
const status = computed(() => livelink.value || {})
const receiver = computed(() => status.value.receiver || {})
const router = computed(() => status.value.router || {})
const clientList = computed(() => clients.value?.clients || [])
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-white">Inputs</h2>
      <RouterLink to="/livelink" class="btn-secondary">
        <span class="material-icons icon-sm">open_in_new</span>
        LiveLink detail
      </RouterLink>
    </div>

    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      <div class="card">
        <div class="flex items-center justify-between mb-3">
          <h3 class="text-lg font-semibold text-white flex items-center gap-2">
            <span class="material-icons text-cyan-400 icon-md">face</span>
            LiveLink Face
          </h3>
          <span :class="['px-2 py-1 text-xs font-medium rounded-full', receiver.connected ? 'bg-emerald-500/20 text-emerald-400' : 'bg-slate-500/20 text-slate-400']">
            {{ receiver.connected ? 'Connected' : 'Disconnected' }}
          </span>
        </div>
        <div class="space-y-2 text-sm">
          <div class="stat-item">
            <span class="stat-label">Source</span>
            <span class="stat-value text-sm font-mono">{{ receiver.source_address || '—' }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Frames received</span>
            <span class="stat-value text-sm">{{ receiver.frames_received ?? 0 }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Routes</span>
            <span class="stat-value text-sm">{{ router.routes_count ?? 0 }}</span>
          </div>
        </div>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <span class="material-icons text-cyan-400 icon-md">cable</span>
          WebSocket clients
        </h3>
        <div v-if="!clientList.length" class="text-sm text-slate-400">No clients connected.</div>
        <ul v-else class="space-y-1 text-sm max-h-48 overflow-y-auto">
          <li v-for="c in clientList" :key="c.client_id" class="flex items-center justify-between font-mono text-xs">
            <span class="text-slate-300 truncate">{{ c.address || c.client_id }}</span>
            <span class="text-slate-500">{{ c.connected_at ? 'live' : '' }}</span>
          </li>
        </ul>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <span class="material-icons text-cyan-400 icon-md">settings_remote</span>
          RC Receiver
        </h3>
        <p class="text-sm text-slate-400">Disabled. Configure on a node's Peripherals tab.</p>
      </div>
    </div>
  </section>
</template>
