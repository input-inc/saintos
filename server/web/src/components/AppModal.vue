<script setup>
import { onMounted, onUnmounted } from 'vue'

const props = defineProps({
  title:    { type: String, required: true },
  width:    { type: String, default: 'max-w-xl' },
})
const emit = defineEmits(['close'])

function onKey (e) { if (e.key === 'Escape') emit('close') }
onMounted(() => window.addEventListener('keydown', onKey))
onUnmounted(() => window.removeEventListener('keydown', onKey))
</script>

<template>
  <div class="fixed inset-0 z-50 flex items-center justify-center p-4 bg-canvas/70 backdrop-blur-sm">
    <div :class="['w-full bg-panel rounded-xl border border-line shadow-xl', width]">
      <div class="flex items-center justify-between px-5 py-3 border-b border-line">
        <h3 class="text-base font-semibold text-fg-strong">{{ title }}</h3>
        <button class="p-1 rounded hover:bg-surface text-fg-muted hover:text-fg-strong transition-colors" @click="emit('close')">
          <span class="material-icons icon-sm">close</span>
        </button>
      </div>
      <div class="px-5 py-4 max-h-[75vh] overflow-y-auto">
        <slot />
      </div>
      <div v-if="$slots.actions" class="px-5 py-3 border-t border-line flex items-center justify-end gap-2">
        <slot name="actions" />
      </div>
    </div>
  </div>
</template>
