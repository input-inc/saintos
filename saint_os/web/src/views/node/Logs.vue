<script setup>
import PendingPort from '@/components/PendingPort.vue'
defineProps({ nodeId: String, node: Object })
</script>
<template><PendingPort name="Node — Logs" legacy-src="web/js/nodelogs.js" /></template>
