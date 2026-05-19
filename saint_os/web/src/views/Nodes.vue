<script setup>
import { onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'

const nodes = useNodesStore()
onMounted(() => nodes.fetchAll().catch(() => {}))
</script>

<template>
  <div class="space-y-4">
    <div class="flex items-center justify-between">
      <h2 class="text-2xl font-semibold tracking-tight">Nodes</h2>
      <span class="text-xs text-slate-500">{{ nodes.all.length }} adopted</span>
    </div>

    <div v-if="nodes.loading" class="text-sm text-slate-400">Loading…</div>
    <div v-else-if="!nodes.all.length" class="card text-sm text-slate-400">
      No nodes adopted yet.
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
      <RouterLink
        v-for="n in nodes.all"
        :key="n.node_id"
        :to="{ name: 'node-overview', params: { id: n.node_id } }"
        class="card hover:border-slate-700 transition-colors"
      >
        <div class="flex items-center gap-2 mb-2">
          <span :class="['w-2 h-2 rounded-full', n.online ? 'bg-emerald-500' : 'bg-slate-500']" />
          <h3 class="font-medium">{{ n.display_name || n.node_id }}</h3>
        </div>
        <div class="text-xs text-slate-400 space-y-0.5 font-mono">
          <div>{{ n.hardware_model || '—' }}</div>
          <div>{{ n.ip_address || '—' }}</div>
          <div>FW {{ n.firmware_version || '—' }}</div>
        </div>
      </RouterLink>
    </div>
  </div>
</template>
