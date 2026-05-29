<script setup>
import { ref, watch } from 'vue'

const props = defineProps({
  label:      { type: String, required: true },
  spec:       { type: Object, required: true },
  modelValue: { type: Number, default: 0 },
})
const emit = defineEmits(['update:modelValue', 'commit'])

const hex = ref('var(--color-panel)')
watch(() => props.modelValue, (v) => {
  if (typeof v !== 'number') return
  hex.value = '#' + (v >>> 0).toString(16).padStart(6, '0').slice(-6)
})

function onChange (e) {
  hex.value = e.target.value
  const packed = parseInt(hex.value.replace('#', ''), 16)
  emit('update:modelValue', packed)
  emit('commit', packed)
}
</script>

<template>
  <div>
    <div class="flex items-center justify-between">
      <span class="text-sm font-medium text-fg-strong">{{ label }}</span>
      <input
        type="color"
        :value="hex"
        class="h-8 w-16 cursor-pointer rounded border border-line bg-canvas"
        @change="onChange"
      />
    </div>
    <div v-if="spec.unsupported" class="text-xs text-amber-400/80 mt-1 flex items-center gap-1">
      <span class="material-icons icon-sm">construction</span>{{ spec.note || 'Not yet supported.' }}
    </div>
  </div>
</template>
