<script setup>
import { computed, ref } from 'vue'
import AppModal from '@/components/AppModal.vue'
import IconPicker from './IconPicker.vue'

const props = defineProps({
  groups: { type: Array, default: () => [] },     // suggested group names
  defaultGroup: { type: String, default: '' },
})
const emit = defineEmits(['close', 'create'])

const name = ref('')
const group = ref(props.defaultGroup)
const icon = ref('animation')
const duration = ref(5.0)
const fps = ref(60)
const loop = ref(false)

const canSubmit = computed(() => name.value.trim().length > 0)

function submit () {
  if (!canSubmit.value) return
  emit('create', {
    name: name.value.trim(),
    group: group.value.trim(),
    icon: icon.value,
    duration: Math.max(0.1, Number(duration.value) || 5),
    fps: Math.max(1, Math.min(240, Number(fps.value) || 60)),
    loop: !!loop.value,
  })
}
</script>

<template>
  <AppModal title="New animation" width="max-w-md" @close="emit('close')">
    <div class="space-y-3">
      <p class="text-xs text-fg-muted">
        Set the basics here — you'll add joints, keyframes, and triggers in the editor.
      </p>

      <div class="flex items-center gap-2">
        <IconPicker v-model="icon" fallback="animation" />
        <label class="flex-1">
          <span class="block text-fg-muted text-xs mb-1">Name</span>
          <input class="input-field w-full"
                 v-model="name"
                 placeholder="Wave Hello"
                 autofocus
                 @keydown.enter="submit" />
        </label>
      </div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Group (optional)</span>
        <input class="input-field w-full"
               list="new-anim-groups"
               v-model="group"
               placeholder="Leave empty for Ungrouped" />
        <datalist id="new-anim-groups">
          <option v-for="g in groups" :key="g" :value="g" />
        </datalist>
      </label>

      <div class="grid grid-cols-2 gap-2">
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Duration (s)</span>
          <input type="number" step="0.1" min="0.1" class="input-field w-full"
                 v-model.number="duration" />
        </label>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">FPS</span>
          <input type="number" step="1" min="1" max="240" class="input-field w-full"
                 v-model.number="fps" />
        </label>
      </div>

      <label class="flex items-center gap-2 text-sm">
        <input type="checkbox" v-model="loop" />
        <span class="text-fg">Loop playback</span>
      </label>
    </div>
    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="!canSubmit" @click="submit">
        Create &amp; open editor
      </button>
    </template>
  </AppModal>
</template>
