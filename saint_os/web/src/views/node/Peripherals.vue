<script setup>
import PendingPort from '@/components/PendingPort.vue'
defineProps({ nodeId: String, node: Object })
</script>
<template><PendingPort name="Node — Peripherals" legacy-src="web/js/peripherals.js" /></template>
