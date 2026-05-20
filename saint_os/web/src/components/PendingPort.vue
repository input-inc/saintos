<script setup>
defineProps({
  name:      { type: String, required: true },
  legacySrc: { type: String, default: null },
})
</script>

<template>
  <div class="card">
    <div class="flex items-start gap-4">
      <div class="w-12 h-12 rounded-xl bg-amber-500/20 flex items-center justify-center flex-shrink-0">
        <span class="material-icons text-amber-400 icon-md">construction</span>
      </div>
      <div class="flex-1">
        <h3 class="text-lg font-semibold text-white mb-1">{{ name }} — port pending</h3>
        <p class="text-sm text-slate-400">
          This view hasn't been migrated to Vue yet.
          <span v-if="legacySrc">Legacy source: <code class="font-mono text-slate-300">{{ legacySrc }}</code></span>
        </p>
        <p class="text-sm text-slate-400 mt-2">
          The old UI is still available at
          <a href="/legacy.html" class="text-cyan-400 hover:text-cyan-300 underline">/legacy.html</a>.
        </p>
      </div>
    </div>
  </div>
</template>
