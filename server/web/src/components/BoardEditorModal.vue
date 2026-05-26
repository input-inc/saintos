<script setup>
import { onMounted, ref } from 'vue'
import { useWsStore } from '@/stores/ws'
import AppModal from './AppModal.vue'

const props = defineProps({
  boardId: { type: String, default: null },        // null = new board
})
const emit = defineEmits(['close', 'saved'])

const ws = useWsStore()
const yamlText = ref('')
const editable = ref(true)
const error = ref('')
const saving = ref(false)

async function load () {
  if (!props.boardId) {
    yamlText.value = [
      'board_id: my_board',
      'display_name: My Board',
      'chip_family: rp2040',
      'available_pins: []',
      'reserved_pins: []',
      'builtin_peripherals: []',
      '',
    ].join('\n')
    return
  }
  try {
    const r = await ws.management('get_board_yaml', { board_id: props.boardId })
    yamlText.value = r?.yaml || ''
    editable.value = !r?.builtin
  } catch (e) { error.value = e.message || String(e) }
}
onMounted(load)

async function save () {
  if (saving.value || !editable.value) return
  saving.value = true
  error.value = ''
  try {
    await ws.management('save_board', { yaml: yamlText.value })
    emit('saved')
    emit('close')
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    saving.value = false
  }
}
</script>

<template>
  <AppModal :title="boardId ? `Board: ${boardId}` : 'New board'" width="max-w-3xl" @close="emit('close')">
    <p class="text-xs text-slate-500 mb-2">
      Edit the YAML directly. Required keys: <code class="text-cyan-300">board_id</code>,
      <code class="text-cyan-300">display_name</code>, <code class="text-cyan-300">chip_family</code>.
      Lists: <code class="text-cyan-300">available_pins</code>, <code class="text-cyan-300">reserved_pins</code>,
      <code class="text-cyan-300">builtin_peripherals</code>.
    </p>
    <textarea
      v-model="yamlText"
      :readonly="!editable"
      class="input-field w-full font-mono text-xs"
      style="min-height: 50vh;"
      spellcheck="false"
    />
    <div v-if="!editable" class="mt-2 text-xs text-amber-400">
      <span class="material-icons icon-sm align-middle">lock</span>
      Built-in boards are read-only.
    </div>
    <div v-if="error" class="mt-2 p-2 rounded bg-red-500/20 border border-red-500/40 text-xs text-red-300">{{ error }}</div>

    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Close</button>
      <button v-if="editable" class="btn-primary" :disabled="saving" @click="save">
        {{ saving ? 'Saving…' : 'Save' }}
      </button>
    </template>
  </AppModal>
</template>
