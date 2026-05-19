<script setup>
import PendingPort from '@/components/PendingPort.vue'
defineProps({ nodeId: String, node: Object })
</script>
<template><PendingPort name="Node — Live readings" legacy-src="web/js/nodelive.js" /></template>
