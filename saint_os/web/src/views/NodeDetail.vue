<script setup>
import { computed, onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'
import NodeActions from '@/components/NodeActions.vue'

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
  { name: 'node-boards',      label: 'Boards' },
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

    <div class="grid grid-cols-1 lg:grid-cols-4 gap-6">
      <div class="lg:col-span-3">
        <RouterView :node-id="id" :node="node" />
      </div>
      <div class="lg:col-span-1">
        <NodeActions :node="node" @changed="nodes.fetchAll()" />
      </div>
    </div>
  </section>
</template>
