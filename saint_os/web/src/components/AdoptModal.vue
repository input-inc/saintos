<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import AppModal from './AppModal.vue'

const props = defineProps({
  nodeId: { type: String, required: true },
  announcedChip: { type: String, default: '' },
})
const emit = defineEmits(['close', 'adopted'])

const ws = useWsStore()
const chips = ref([])
const boards = ref([])
const roles = ref([])

const chipFamily = ref('')
const boardId = ref('')
const role = ref('')
const displayName = ref('')
const error = ref('')

async function loadChips () {
  try {
    const r = await ws.management('list_chips', {})
    chips.value = r?.chips || []
    const announced = chips.value.find(c => c.chip_family === props.announcedChip)
    chipFamily.value = announced ? props.announcedChip : ''
  } catch (e) { console.warn('list_chips failed:', e) }
}
async function loadBoards () {
  try {
    const r = await ws.management('list_boards', chipFamily.value ? { chip_family: chipFamily.value } : {})
    boards.value = r?.boards || []
    const builtins = boards.value.filter(b => b.builtin)
    if (builtins.length === 1) boardId.value = builtins[0].board_id
    else if (!boards.value.find(b => b.board_id === boardId.value)) boardId.value = ''
  } catch (e) { console.warn('list_boards failed:', e) }
}
async function loadRoles () {
  try {
    const r = await ws.management('get_roles')
    roles.value = r?.roles || []
  } catch (e) {
    roles.value = ['head', 'arms', 'tracks', 'console'].map(name => ({ role: name, display_name: name }))
  }
}

watch(chipFamily, loadBoards)

onMounted(() => { loadChips().then(loadBoards); loadRoles() })

const chipHint = computed(() => {
  if (!props.announcedChip) return 'Firmware did not report a chip — please choose.'
  const known = chips.value.some(c => c.chip_family === props.announcedChip)
  return known
    ? `Detected from firmware: ${props.announcedChip}`
    : `Firmware reported '${props.announcedChip}' (not a known chip — please choose).`
})

const roleDescription = computed(() => roles.value.find(r => r.role === role.value)?.description || '')

async function submit () {
  error.value = ''
  if (!role.value)        { error.value = 'Pick a role'; return }
  if (!chipFamily.value)  { error.value = 'Pick a chip'; return }
  if (!boardId.value)     { error.value = 'Pick a board'; return }
  const params = { node_id: props.nodeId, role: role.value, chip_family: chipFamily.value, board_id: boardId.value }
  if (displayName.value) params.display_name = displayName.value
  try {
    const r = await ws.management('adopt_node', params)
    if (r?.success === false) { error.value = r.message || 'Adoption failed'; return }
    emit('adopted', { nodeId: props.nodeId, role: role.value, boardId: boardId.value })
    emit('close')
  } catch (e) {
    error.value = e.message || String(e)
  }
}
</script>

<template>
  <AppModal title="Adopt node" width="max-w-xl" @close="emit('close')">
    <div v-if="error" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">{{ error }}</div>

    <div class="space-y-4">
      <div>
        <label class="block text-sm font-medium text-slate-300 mb-1">Node ID</label>
        <div class="input-field w-full bg-slate-900/50 text-slate-400 font-mono">{{ nodeId }}</div>
      </div>

      <div>
        <label class="block text-sm font-medium text-slate-300 mb-1">Chip family</label>
        <select v-model="chipFamily" class="input-field w-full">
          <option value="">-- Select Chip --</option>
          <option v-for="c in chips" :key="c.chip_family" :value="c.chip_family">
            {{ c.display_name }} ({{ c.chip_family }})
          </option>
        </select>
        <p :class="['text-xs mt-1', props.announcedChip && !chips.find(c => c.chip_family === props.announcedChip) ? 'text-amber-400' : 'text-slate-500']">
          {{ chipHint }}
        </p>
      </div>

      <div>
        <label class="block text-sm font-medium text-slate-300 mb-1">Board</label>
        <select v-model="boardId" class="input-field w-full" :disabled="!chipFamily">
          <option value="">-- Select Board --</option>
          <option v-for="b in boards" :key="b.board_id" :value="b.board_id">
            {{ b.display_name }}{{ b.builtin ? '' : '  (custom)' }}
          </option>
        </select>
      </div>

      <div>
        <label class="block text-sm font-medium text-slate-300 mb-1">Role</label>
        <select v-model="role" class="input-field w-full">
          <option value="">-- Select Role --</option>
          <option v-for="r in roles" :key="r.role" :value="r.role">
            {{ r.display_name || r.role }}
          </option>
        </select>
        <p v-if="roleDescription" class="text-xs text-slate-500 mt-1">{{ roleDescription }}</p>
      </div>

      <div>
        <label class="block text-sm font-medium text-slate-300 mb-1">Display name (optional)</label>
        <input v-model="displayName" type="text" class="input-field w-full" placeholder="e.g., Left Arm Controller" />
      </div>
    </div>

    <template #actions>
      <button class="btn-secondary" @click="emit('close')">Cancel</button>
      <button class="btn-primary" @click="submit">Adopt</button>
    </template>
  </AppModal>
</template>
