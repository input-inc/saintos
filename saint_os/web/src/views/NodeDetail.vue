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
  { name: 'node-state',       label: 'State' },
  { name: 'node-live',        label: 'Live' },
  { name: 'node-logs',        label: 'Logs' },
  { name: 'node-boards',      label: 'Boards' },
]

const title = computed(() => node.value?.display_name || props.id)
const subtitle = computed(() => node.value
  ? `${node.value.role || '—'} · ${node.value.node_id}`
  : props.id)
const online = computed(() => node.value?.online !== false)
</script>

<template>
  <div class="space-y-4">
    <div class="flex items-center justify-between">
      <div>
        <h2 class="text-2xl font-semibold tracking-tight flex items-center gap-2">
          <span :class="['w-2.5 h-2.5 rounded-full', online ? 'bg-emerald-500' : 'bg-slate-500']" />
          {{ title }}
        </h2>
        <p class="text-xs text-slate-500 font-mono">{{ subtitle }}</p>
      </div>
      <RouterLink :to="{ name: 'nodes' }" class="btn">← Nodes</RouterLink>
    </div>

    <div class="flex gap-1 border-b border-slate-800">
      <RouterLink
        v-for="t in tabs"
        :key="t.name"
        :to="{ name: t.name, params: { id } }"
        active-class="text-cyan-300 border-cyan-400"
        class="px-3 py-2 text-sm text-slate-400 hover:text-white border-b-2 border-transparent"
      >
        {{ t.label }}
      </RouterLink>
    </div>

    <RouterView :node-id="id" :node="node" />
  </div>
</template>
