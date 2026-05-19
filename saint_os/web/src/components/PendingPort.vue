<script setup>
defineProps({
  name: { type: String, required: true },
  legacySrc: { type: String, default: null },
})
</script>

<template>
  <div class="card">
    <div class="flex items-start gap-3">
      <span class="mt-0.5 inline-flex w-8 h-8 items-center justify-center rounded-full bg-amber-900/30 text-amber-300">
        🚧
      </span>
      <div class="flex-1">
        <h3 class="text-base font-semibold mb-1">{{ name }} — port pending</h3>
        <p class="text-sm text-slate-400">
          This view hasn't been migrated to Vue yet.
          <span v-if="legacySrc">The legacy implementation lives in <code class="font-mono text-slate-300">{{ legacySrc }}</code>.</span>
        </p>
        <p class="text-sm text-slate-400 mt-2">
          Until then, the old UI is still available at
          <a href="/legacy.html" class="text-cyan-400 underline">/legacy.html</a>.
        </p>
      </div>
    </div>
  </div>
</template>
