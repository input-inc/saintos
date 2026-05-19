<script setup>
defineProps({
  nodeId: { type: String, required: true },
  node: { type: Object, default: null },
})
</script>

<template>
  <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
    <div class="card">
      <h3 class="text-sm font-semibold mb-3 text-slate-200">Identity</h3>
      <dl class="space-y-2 text-sm">
        <div class="flex justify-between"><dt class="stat-label">Node ID</dt><dd class="font-mono">{{ nodeId }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">Display name</dt><dd>{{ node?.display_name || '—' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">Role</dt><dd>{{ node?.role || '—' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">Hardware</dt><dd>{{ node?.hardware_model || '—' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">Board</dt><dd>{{ node?.board_id || '—' }}</dd></div>
      </dl>
    </div>
    <div class="card">
      <h3 class="text-sm font-semibold mb-3 text-slate-200">Connection</h3>
      <dl class="space-y-2 text-sm">
        <div class="flex justify-between"><dt class="stat-label">Online</dt><dd>{{ node?.online ? 'Yes' : 'No' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">IP</dt><dd class="font-mono">{{ node?.ip_address || '—' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">MAC</dt><dd class="font-mono">{{ node?.mac_address || '—' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">Firmware</dt><dd class="font-mono">{{ node?.firmware_version || '—' }}</dd></div>
        <div class="flex justify-between"><dt class="stat-label">Bootloader</dt><dd class="font-mono">{{ node?.bootloader_version || '—' }}</dd></div>
      </dl>
    </div>
  </div>
</template>
