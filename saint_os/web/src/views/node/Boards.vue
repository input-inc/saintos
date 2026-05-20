<script setup>
import { onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'

const props = defineProps({
  nodeId: { type: String, required: true },
  node:   { type: Object, default: null },
})

const ws = useWsStore()
const boards = ref([])
const loading = ref(true)

async function load () {
  loading.value = true
  try {
    const r = await ws.management('list_boards', {})
    boards.value = r?.boards || []
  } catch (e) {
    console.warn('list_boards failed:', e)
  } finally {
    loading.value = false
  }
}
onMounted(load)
</script>

<template>
  <div class="card">
    <div class="flex items-center justify-between mb-4">
      <h3 class="text-lg font-semibold text-white">Board: {{ node?.board_id || '—' }}</h3>
      <span class="text-xs text-slate-500">{{ node?.chip_family || 'unknown chip' }}</span>
    </div>

    <p class="text-sm text-slate-400 mb-3">
      Boards are defined as YAML files on the server (see the
      <RouterLink to="/settings" class="text-cyan-400 underline">Settings → Boards</RouterLink>
      panel to view or edit). The current node was adopted against:
    </p>

    <div v-if="loading" class="text-sm text-slate-400">Loading…</div>
    <div v-else-if="!boards.length" class="text-sm text-slate-400">No boards registered on the server.</div>
    <ul v-else class="divide-y divide-slate-700/50">
      <li
        v-for="b in boards"
        :key="b.id"
        :class="['flex items-center justify-between py-2 text-sm',
                 b.id === node?.board_id ? 'text-cyan-300' : 'text-slate-300']"
      >
        <span class="font-mono">{{ b.id }}</span>
        <span class="text-xs text-slate-500">{{ b.chip_family || '—' }} · {{ b.builtin ? 'built-in' : 'operator' }}</span>
      </li>
    </ul>
  </div>
</template>
