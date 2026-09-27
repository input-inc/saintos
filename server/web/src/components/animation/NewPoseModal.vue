<script setup>
import { computed, ref } from 'vue'
import AppModal from '@/components/AppModal.vue'
import IconPicker from './IconPicker.vue'

// No grouping field: an item can be in many playlists now, and
// membership is assigned by dragging the row onto one in the board
// sidebar. See views/Animations.vue.
const emit = defineEmits(['close', 'create'])

const name = ref('')
const icon = ref('accessibility')
const description = ref('')

const canSubmit = computed(() => name.value.trim().length > 0)

function submit () {
  if (!canSubmit.value) return
  emit('create', {
    name: name.value.trim(),
    icon: icon.value,
    description: description.value.trim(),
  })
}
</script>

<template>
  <AppModal title="New pose" width="max-w-md" @close="emit('close')">
    <div class="space-y-3">
      <p class="text-xs text-fg-muted">
        Name the pose — you'll add WS-input setpoints in the editor, and drag it onto a playlist in the sidebar to group it.
      </p>

      <div class="flex items-center gap-2">
        <IconPicker v-model="icon" fallback="accessibility" />
        <label class="flex-1">
          <span class="block text-fg-muted text-xs mb-1">Name</span>
          <input class="input-field w-full"
                 v-model="name"
                 placeholder="Rest"
                 autofocus
                 @keydown.enter="submit" />
        </label>
      </div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Description (optional)</span>
        <textarea class="input-field w-full" rows="2"
                  v-model="description"
                  placeholder="e.g. neutral standing pose"></textarea>
      </label>
    </div>
    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" :disabled="!canSubmit" @click="submit">
        Create &amp; edit
      </button>
    </template>
  </AppModal>
</template>
