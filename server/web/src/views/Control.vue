<script setup>
import { onMounted, ref } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import Moods from '@/views/Moods.vue'

const nodes = useNodesStore()
const tab = ref('nodes')

onMounted(() => nodes.fetchAll().catch(() => {}))
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-white">Control Panel</h2>
    </div>

    <div class="flex items-center gap-1 mb-6 border-b border-slate-700/50">
      <button :class="['node-tab', tab === 'nodes' ? 'active' : '']" @click="tab = 'nodes'">Nodes</button>
      <button :class="['node-tab', tab === 'moods' ? 'active' : '']" @click="tab = 'moods'">Moods</button>
    </div>

    <div v-show="tab === 'nodes'">
      <p class="text-sm text-slate-400 mb-4">
        Per-channel control lives on each node's State tab. This page is a quick jump-off.
      </p>

      <div v-if="!nodes.all.length" class="card text-center py-10">
        <span class="material-icons icon-lg text-slate-600">dns</span>
        <p class="text-slate-400 text-sm mt-3">No adopted nodes.</p>
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
            <div class="font-semibold text-white truncate">{{ n.display_name || n.node_id }}</div>
            <div class="text-xs text-slate-400 font-mono">{{ n.role || 'unassigned' }} · {{ n.ip_address || '—' }}</div>
          </div>
          <span class="material-icons icon-sm text-slate-500">chevron_right</span>
        </RouterLink>
      </div>
    </div>

    <div v-show="tab === 'moods'">
      <Moods embedded />
    </div>
  </section>
</template>
