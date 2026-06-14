<script setup>
import ConsoleBatteryView from '@/components/console/ConsoleBatteryView.vue'

// Thin router wrapper. The route is chromeless (meta.chromeless),
// so this view fills the whole viewport without AppHeader / NavBar.
// Eventually the Pi console kiosk will land here directly.
defineProps({
  nodeId: { type: String, required: true },
  peripheralId: { type: String, required: true },
})
</script>

<template>
  <div class="h-full w-full">
    <ConsoleBatteryView :node-id="nodeId" :peripheral-id="peripheralId" />
  </div>
</template>
