<script setup>
import PendingPort from '@/components/PendingPort.vue'
defineProps({ nodeId: String, node: Object })
</script>
<template><PendingPort name="Node — Boards" legacy-src="web/js/boards.js" /></template>
