<script setup>
import { computed } from 'vue'

const props = defineProps({
  label:      { type: String,  required: true },
  modelValue: { type: Number,  default: 0 },
})
const emit = defineEmits(['update:modelValue', 'commit'])

const isOn = computed(() => (props.modelValue ?? 0) >= 0.5)

function toggle () {
  const next = isOn.value ? 0 : 1
  emit('update:modelValue', next)
  emit('commit', next)
}
</script>

<template>
  <div class="flex items-center justify-between">
    <span class="text-sm font-medium">{{ label }}</span>
    <button
      :class="[
        'relative inline-flex h-6 w-11 items-center rounded-full transition-colors',
        isOn ? 'bg-emerald-500' : 'bg-slate-700'
      ]"
      @click="toggle"
    >
      <span :class="[
        'inline-block h-4 w-4 transform rounded-full transition-transform',
        isOn ? 'translate-x-6 bg-white' : 'translate-x-1 bg-slate-400'
      ]" />
    </button>
  </div>
</template>
