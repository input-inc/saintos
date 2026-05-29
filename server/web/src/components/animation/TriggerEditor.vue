<script setup>
import { computed } from 'vue'
import AppModal from '@/components/AppModal.vue'
import { useAnimationsStore } from '@/stores/animations'

const props = defineProps({
  anim:     { type: Object, required: true },
  trackId:  { type: String, required: true },
  kfIdx:    { type: Number, required: true },
  wsInputs: { type: Array,  required: true }, // [{sheet_id, sheet_label, input_id, label, kind}]
})

const emit = defineEmits(['close', 'delete'])

const animations = useAnimationsStore()

const track = computed(() =>
  props.anim.trigger_tracks.find(t => t.id === props.trackId)
)
const kf = computed(() => track.value?.keyframes[props.kfIdx])

const sheets = computed(() => {
  const ids = new Set(props.wsInputs.map(w => w.sheet_id))
  return [...ids].map(id => ({
    id,
    label: props.wsInputs.find(w => w.sheet_id === id)?.sheet_label || id,
  }))
})

const inputsForSheet = computed(() => {
  if (!kf.value || kf.value.target_kind !== 'ws_input') return []
  const sheetId = kf.value.target?.[0]
  return props.wsInputs.filter(w => w.sheet_id === sheetId)
})

function onTargetKindChange () {
  // Reset target shape when switching dispatch kind so we don't carry
  // ws_input parts into a topic dispatch or vice versa.
  if (!kf.value) return
  kf.value.target = ['', '']
  animations.markDirty()
}

function dismiss () { emit('close') }
function destroy () { emit('delete', track.value, props.kfIdx) }
</script>

<template>
  <AppModal v-if="kf" @close="dismiss" title="Trigger keyframe">
    <div class="space-y-3">
      <div class="grid grid-cols-2 gap-3">
        <label class="text-sm">
          <span class="block text-fg-muted mb-1">Time (s)</span>
          <input type="number" step="0.01" min="0" :max="anim.duration"
                 class="input-field" v-model.number="kf.time"
                 @input="animations.markDirty()" />
        </label>
        <label class="text-sm">
          <span class="block text-fg-muted mb-1">Label</span>
          <input class="input-field" v-model="kf.label" @input="animations.markDirty()" />
        </label>
      </div>

      <label class="text-sm block">
        <span class="block text-fg-muted mb-1">Target kind</span>
        <select class="input-field" v-model="kf.target_kind" @change="onTargetKindChange">
          <option value="ws_input">Node sheet input</option>
          <option value="topic">ROS topic field</option>
        </select>
      </label>

      <template v-if="kf.target_kind === 'ws_input'">
        <div class="grid grid-cols-2 gap-3">
          <label class="text-sm">
            <span class="block text-fg-muted mb-1">Sheet</span>
            <select class="input-field" v-model="kf.target[0]" @change="animations.markDirty(); kf.target[1] = ''">
              <option value="">— Pick a sheet —</option>
              <option v-for="s in sheets" :key="s.id" :value="s.id">{{ s.label }}</option>
            </select>
          </label>
          <label class="text-sm">
            <span class="block text-fg-muted mb-1">Input</span>
            <select class="input-field" v-model="kf.target[1]" @change="animations.markDirty()"
                    :disabled="!inputsForSheet.length">
              <option value="">— Pick an input —</option>
              <option v-for="w in inputsForSheet" :key="w.input_id" :value="w.input_id">
                {{ w.label || w.input_id }}
              </option>
            </select>
          </label>
        </div>
        <p v-if="!sheets.length" class="text-xs text-amber-300">
          No WS inputs available — add some on a routing sheet first.
        </p>
      </template>

      <template v-else>
        <div class="grid grid-cols-2 gap-3">
          <label class="text-sm">
            <span class="block text-fg-muted mb-1">Topic endpoint</span>
            <input class="input-field" placeholder="/saint/head" v-model="kf.target[0]"
                   @input="animations.markDirty()" />
          </label>
          <label class="text-sm">
            <span class="block text-fg-muted mb-1">Field</span>
            <input class="input-field" placeholder="pan" v-model="kf.target[1]"
                   @input="animations.markDirty()" />
          </label>
        </div>
      </template>

      <label class="text-sm block">
        <span class="block text-fg-muted mb-1">Value</span>
        <input type="number" step="0.01" class="input-field" v-model.number="kf.value"
               @input="animations.markDirty()" />
      </label>
    </div>

    <template #actions>
      <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong" @click="destroy">
        <span class="material-icons icon-sm">delete</span>
        Delete
      </button>
      <button class="btn-primary" @click="dismiss">Done</button>
    </template>
  </AppModal>
</template>
