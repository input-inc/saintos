<script setup>
import { computed } from 'vue'
import { useRoute } from 'vue-router'
import ConsoleBatteriesOverview from '@/components/console/ConsoleBatteriesOverview.vue'

// Multi-pack console wrapper. Reads the `packs` query string as
// JSON; the console_display peripheral on the Pi builds this URL
// from operator-supplied pack list. Falls back to an empty array
// (the overview then renders a friendly empty state).
const route = useRoute()
const packs = computed(() => {
  const raw = route.query.packs
  if (typeof raw !== 'string' || !raw.length) return []
  try {
    const parsed = JSON.parse(raw)
    return Array.isArray(parsed) ? parsed : []
  } catch (e) {
    console.warn('console-batteries: invalid packs query param:', e)
    return []
  }
})
</script>

<template>
  <div class="h-full w-full">
    <ConsoleBatteriesOverview :packs="packs" />
  </div>
</template>
