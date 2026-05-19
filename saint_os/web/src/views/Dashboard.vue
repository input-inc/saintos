<script setup>
import { onMounted } from 'vue'
import { useNodesStore } from '@/stores/nodes'

const nodes = useNodesStore()
onMounted(() => nodes.fetchAll().catch(() => {}))
</script>

<template>
  <div class="space-y-6">
    <h2 class="text-2xl font-semibold tracking-tight">Dashboard</h2>

    <div class="grid grid-cols-1 md:grid-cols-3 gap-4">
      <div class="card">
        <div class="stat-label mb-1">Adopted nodes</div>
        <div class="text-3xl font-semibold">{{ nodes.all.length }}</div>
      </div>
      <div class="card">
        <div class="stat-label mb-1">Online</div>
        <div class="text-3xl font-semibold">{{ nodes.all.filter(n => n.online).length }}</div>
      </div>
      <div class="card">
        <div class="stat-label mb-1">Offline</div>
        <div class="text-3xl font-semibold">{{ nodes.all.filter(n => !n.online).length }}</div>
      </div>
    </div>
  </div>
</template>
