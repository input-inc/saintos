<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import AppModal from './AppModal.vue'

// Mirrors vanilla app.js openEditNodeModal()/submitNodeEdit(): chip →
// board cascade, role dropdown with description, optional display name.
// Submits via the `update_node` management action.
const props = defineProps({
  node: { type: Object, required: true },
})
const emit = defineEmits(['close', 'updated'])

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
  } catch (e) {
    console.warn('list_chips failed:', e)
  }
}
async function loadBoards (preselect) {
  try {
    const r = await ws.management(
      'list_boards', chipFamily.value ? { chip_family: chipFamily.value } : {}
    )
    boards.value = r?.boards || []
    if (preselect && boards.value.some(b => b.board_id === preselect)) {
      boardId.value = preselect
    } else if (!boards.value.find(b => b.board_id === boardId.value)) {
      boardId.value = ''
    }
  } catch (e) {
    console.warn('list_boards failed:', e)
  }
}
async function loadRoles () {
  try {
    const r = await ws.management('get_roles')
    roles.value = r?.roles || []
  } catch (e) {
    roles.value = ['head', 'arms', 'tracks', 'console'].map(name => ({ role: name, display_name: name }))
  }
}

const roleDescription = computed(() => roles.value.find(r => r.role === role.value)?.description || '')

// Cascade — board list re-loads on chip change, mirroring vanilla.
let initialBoardId = null
watch(chipFamily, (newChip, oldChip) => {
  if (oldChip === undefined) return
  loadBoards(null)
})

onMounted(async () => {
  // Seed initial values from current node info.
  chipFamily.value  = props.node.chip_family  || ''
  initialBoardId    = props.node.board_id     || ''
  role.value        = props.node.role         || ''
  displayName.value = props.node.display_name || ''
  await Promise.all([loadChips(), loadRoles()])
  await loadBoards(initialBoardId)
})

async function submit () {
  error.value = ''
  if (!role.value)       { error.value = 'Please select a role'; return }
  if (!chipFamily.value) { error.value = 'Please select a chip'; return }
  if (!boardId.value)    { error.value = 'Please select a board'; return }

  const params = {
    node_id: props.node.node_id,
    role: role.value,
    chip_family: chipFamily.value,
    board_id: boardId.value,
    // Always send display_name even when empty so the operator can
    // clear a previously-set name — matches vanilla submitNodeEdit().
    display_name: displayName.value.trim(),
  }

  try {
    const r = await ws.management('update_node', params)
    if (r && r.success === false) {
      error.value = r.message || 'Update failed'
      return
    }
    emit('updated')
    emit('close')
  } catch (e) {
    error.value = e.message || String(e)
  }
}
</script>

<template>
  <AppModal title="Edit Node" width="max-w-md" @close="emit('close')">
    <div v-if="error" class="mb-3 p-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">{{ error }}</div>

    <div class="space-y-4">
      <div>
        <label class="block text-sm font-medium text-fg mb-2">Node ID</label>
        <div class="input-field w-full bg-canvas/50 text-fg-muted font-mono">{{ node.node_id }}</div>
      </div>

      <div>
        <label class="block text-sm font-medium text-fg mb-2">Chip</label>
        <select v-model="chipFamily" class="input-field w-full">
          <option value="">-- Select Chip --</option>
          <option v-for="c in chips" :key="c.chip_family" :value="c.chip_family">
            {{ c.display_name }} ({{ c.chip_family }})
          </option>
        </select>
      </div>

      <div>
        <label class="block text-sm font-medium text-fg mb-2">Board</label>
        <select v-model="boardId" class="input-field w-full">
          <option value="">-- Select Board --</option>
          <option v-for="b in boards" :key="b.board_id" :value="b.board_id">
            {{ b.display_name }}{{ b.builtin ? '' : '  (custom)' }}
          </option>
        </select>
        <p class="text-xs text-fg-faint mt-1">
          Changing the board re-derives pin layout and re-seeds built-in peripherals.
        </p>
      </div>

      <div>
        <label class="block text-sm font-medium text-fg mb-2">Role</label>
        <select v-model="role" class="input-field w-full">
          <option value="">-- Select Role --</option>
          <option v-for="r in roles" :key="r.role" :value="r.role">
            {{ r.display_name || r.role }}
          </option>
        </select>
        <p v-if="roleDescription" class="text-xs text-fg-faint mt-1">{{ roleDescription }}</p>
      </div>

      <div>
        <label class="block text-sm font-medium text-fg mb-2">
          Display Name <span class="text-fg-faint">(optional)</span>
        </label>
        <input v-model="displayName" type="text" class="input-field w-full" placeholder="e.g., Left Arm Controller" />
      </div>
    </div>

    <template #actions>
      <button class="btn-secondary flex-1" @click="emit('close')">Cancel</button>
      <button class="btn-primary flex-1" @click="submit">
        <span class="material-icons icon-sm">check</span>
        Save
      </button>
    </template>
  </AppModal>
</template>
