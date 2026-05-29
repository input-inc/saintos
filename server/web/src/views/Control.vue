<script setup>
import { onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'

const nodes = useNodesStore()

onMounted(() => nodes.fetchAll().catch(() => {}))
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-fg-strong">Control Panel</h2>
    </div>

    <p class="text-sm text-fg-muted mb-4">
      Per-channel control lives on each node's State tab. This page is a quick jump-off.
      For animations and pose presets, see the
      <RouterLink to="/animations" class="text-cyan-400 hover:text-cyan-300">Animation Builder</RouterLink>.
    </p>

    <div v-if="!nodes.all.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-fg-faint">dns</span>
      <p class="text-fg-muted text-sm mt-3">No adopted nodes.</p>
    </div>

    <div v-else class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <RouterLink
        v-for="n in nodes.all"
        :key="n.node_id"
        :to="{ name: 'node-state', params: { id: n.node_id } }"
        class="node-card flex items-start gap-3"
      >
        <span :class="['mt-1 w-2.5 h-2.5 rounded-full flex-shrink-0', n.online ? 'bg-emerald-500 animate-pulse-dot' : 'bg-slate-500']" />
        <div class="flex-1 min-w-0">
          <div class="font-semibold text-fg-strong truncate">{{ n.display_name || n.node_id }}</div>
          <div class="text-xs text-fg-muted font-mono">{{ n.role || 'unassigned' }} · {{ n.ip_address || '—' }}</div>
        </div>
        <span class="material-icons icon-sm text-fg-faint">chevron_right</span>
      </RouterLink>
    </div>
  </section>
</template>
