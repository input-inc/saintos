<script setup>
import { computed } from 'vue'
import { useRouter } from 'vue-router'
import { useWsTopic } from '@/composables/useWsTopic'

const livelink = useWsTopic(() => 'livelink')
const clients = useWsTopic(() => 'clients')
const router = useRouter()
const status = computed(() => livelink.value || {})
const receiver = computed(() => status.value.receiver || {})
const clientList = computed(() => clients.value?.clients || [])

// Mirrors vanilla updateInputsSummary(): badge text changes between
// Connected / Listening / Disconnected based on receiver state.
const badgeText = computed(() => {
  if (receiver.value.connected) return 'Connected'
  if (receiver.value.running)   return 'Listening'
  return 'Disconnected'
})
const badgeClass = computed(() => {
  if (receiver.value.connected) return 'bg-emerald-500/20 text-emerald-400'
  if (receiver.value.running)   return 'bg-amber-500/20 text-amber-400'
  return 'bg-slate-500/20 text-fg-muted'
})
const statusText = computed(() => {
  if (receiver.value.connected) return 'Receiving data'
  if (receiver.value.running)   return 'Waiting for connection'
  return 'Not running'
})

function viewDetails () { router.push('/livelink') }
</script>

<template>
  <section>
    <h2 class="text-2xl font-bold text-fg-strong mb-6">Inputs</h2>

    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      <div class="card">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-semibold text-fg-strong">LiveLink Face</h3>
          <span :class="['px-2 py-1 text-xs font-medium rounded-full', badgeClass]">
            {{ badgeText }}
          </span>
        </div>
        <div class="space-y-3">
          <div class="stat-item">
            <span class="stat-label">Status</span>
            <span class="stat-value text-sm">{{ statusText }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Port</span>
            <span class="stat-value text-sm font-mono">{{ receiver.port ?? 11111 }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">Packets</span>
            <span class="stat-value text-sm">{{ (receiver.packet_count ?? 0).toLocaleString() }}</span>
          </div>
        </div>
        <div class="mt-4 pt-4 border-t border-line">
          <button class="btn-secondary w-full justify-center" @click="viewDetails">
            <span class="material-icons icon-sm">visibility</span>
            View Details
          </button>
        </div>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-3 flex items-center gap-2">
          <span class="material-icons text-cyan-400 icon-md">cable</span>
          WebSocket clients
        </h3>
        <div v-if="!clientList.length" class="text-sm text-fg-muted">No clients connected.</div>
        <ul v-else class="space-y-1 text-sm max-h-48 overflow-y-auto">
          <li v-for="c in clientList" :key="c.client_id" class="flex items-center justify-between font-mono text-xs">
            <span class="text-fg truncate">{{ c.address || c.client_id }}</span>
            <span class="text-fg-faint">{{ c.connected_at ? 'live' : '' }}</span>
          </li>
        </ul>
      </div>

      <div class="card">
        <h3 class="text-lg font-semibold text-fg-strong mb-3 flex items-center gap-2">
          <span class="material-icons text-cyan-400 icon-md">settings_remote</span>
          RC Receiver
        </h3>
        <p class="text-sm text-fg-muted">Disabled. Configure on a node's Peripherals tab.</p>
      </div>
    </div>
  </section>
</template>
