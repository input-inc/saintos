<script setup>
import { computed, onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'

const props = defineProps({ id: { type: String, required: true } })
const nodes = useNodesStore()
const node = nodes.byId(props.id)

onMounted(() => nodes.fetchAll().catch(() => {}))

const tabs = [
  { name: 'node-overview',    label: 'Overview' },
  { name: 'node-peripherals', label: 'Peripherals' },
  { name: 'node-live',        label: 'Live readings' },
  { name: 'node-state',       label: 'State' },
  { name: 'node-logs',        label: 'Logs' },
  { name: 'node-control',     label: 'Control' },
]

const title = computed(() => node.value?.display_name || props.id)
const role  = computed(() => node.value?.role || 'unassigned')
const online = computed(() => node.value?.online !== false)
</script>

<template>
  <section>
    <div class="flex items-center justify-between mb-4">
      <div class="flex items-center gap-3">
        <span :class="['w-3 h-3 rounded-full', online ? 'bg-emerald-500 animate-pulse-dot' : 'bg-slate-500']" />
        <div>
          <h2 class="text-2xl font-bold text-white leading-tight">{{ title }}</h2>
          <p class="text-xs text-slate-400 font-mono">{{ role }} · {{ id }}</p>
        </div>
      </div>
      <RouterLink :to="{ name: 'nodes' }" class="btn-secondary">
        <span class="material-icons icon-sm">arrow_back</span>
        Nodes
      </RouterLink>
    </div>

    <div class="flex gap-1 border-b border-slate-700/50 mb-6 overflow-x-auto">
      <RouterLink
        v-for="t in tabs"
        :key="t.name"
        :to="{ name: t.name, params: { id } }"
        class="node-tab"
      >
        {{ t.label }}
      </RouterLink>
    </div>

    <RouterView :node-id="id" :node="node" />
  </section>
</template>
