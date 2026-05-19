<script setup>
import { computed } from 'vue'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()
const label = computed(() => {
  if (!ws.connected) return 'Disconnected'
  if (ws.authRequired && !ws.authenticated) return 'Auth required'
  return ws.serverName || 'Connected'
})
const dotClass = computed(() => {
  if (!ws.connected) return 'bg-rose-500'
  if (ws.authRequired && !ws.authenticated) return 'bg-amber-500'
  return 'bg-emerald-500'
})
</script>

<template>
  <div class="flex items-center gap-2 text-xs text-slate-300">
    <span :class="['w-2 h-2 rounded-full', dotClass]" />
    <span class="font-mono">{{ label }}</span>
  </div>
</template>
