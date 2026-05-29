<script setup>
import { ref, watch } from 'vue'
import { useThrottledSend } from '@/composables/useThrottledSend'

const props = defineProps({
  label:    { type: String, required: true },
  spec:     { type: Object, required: true },
  modelValue: { type: Number, default: null },
})
const emit = defineEmits(['update:modelValue', 'commit'])

const local = ref(props.modelValue ?? props.spec.neutral ?? props.spec.min)
const dragging = ref(false)

watch(() => props.modelValue, (v) => {
  if (v === null || v === undefined) return
  if (!dragging.value) local.value = Number(v)
})

const send = useThrottledSend(v => emit('commit', v))

function onInput (e) {
  dragging.value = true
  local.value = parseFloat(e.target.value)
  emit('update:modelValue', local.value)
  send(local.value)
}
function onChange () {
  dragging.value = false
  emit('commit', local.value)
}
function reset () {
  local.value = props.spec.neutral
  emit('update:modelValue', local.value)
  emit('commit', local.value)
}
const fmt = (v) => props.spec.format ? props.spec.format(v) : Number(v).toFixed(2)
</script>

<template>
  <div>
    <div class="flex items-center justify-between mb-2">
      <span class="text-sm font-medium text-fg-strong">{{ label }}</span>
      <div class="flex items-center gap-2">
        <span class="text-sm text-cyan-400 font-mono w-20 text-right">{{ fmt(local) }}</span>
        <button class="text-xs text-fg-muted hover:text-fg-strong transition-colors" @click="reset">
          <span class="material-icons icon-sm align-middle">restart_alt</span>
        </button>
      </div>
    </div>
    <input
      type="range"
      :min="spec.min" :max="spec.max" :step="spec.step" :value="local"
      class="w-full accent-cyan-500 cursor-pointer"
      @input="onInput"
      @change="onChange"
    />
    <div class="flex justify-between mt-1 text-xs text-fg-faint">
      <span>{{ fmt(spec.min) }}</span>
      <span>{{ fmt(spec.neutral) }}</span>
      <span>{{ fmt(spec.max) }}</span>
    </div>
    <div v-if="spec.hint" class="text-xs text-fg-faint mt-1">{{ spec.hint }}</div>
    <div v-if="spec.unsupported" class="text-xs text-amber-400/80 mt-1 flex items-center gap-1">
      <span class="material-icons icon-sm">construction</span>{{ spec.note || 'Not yet supported.' }}
    </div>
  </div>
</template>
