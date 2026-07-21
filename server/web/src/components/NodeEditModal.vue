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
const robotRoles = ref([])
const robotName = ref('')

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
async function loadRobot () {
  try {
    const r = await ws.management('get_robot')
    robotRoles.value = r?.roles || []
    robotName.value = r?.name || ''
  } catch (e) {
    robotRoles.value = []
  }
}
// If the node's current role isn't in the active robot's list (e.g. a
// manifest change or a legacy slug), still show it as a selectable
// option so editing other fields doesn't silently drop it.
const roleOptions = computed(() => {
  const opts = [...robotRoles.value]
  if (role.value && !opts.includes(role.value)) opts.unshift(role.value)
  return opts
})

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
  await Promise.all([loadChips(), loadRobot()])
  await loadBoards(initialBoardId)
})

/* Refresh built-in peripherals from the node's current board YAML
 * without changing role/board/chip. Useful when the operator edits a
 * board definition to add new built-ins (e.g. the Teensy onboard LED)
 * and wants existing adopted nodes to pick those up without
 * re-adoption.
 *
 * The result panel shows what got added so the operator knows
 * something happened — silent "OK" buttons are a dark pattern for
 * idempotent actions like this. Empty `added` is treated as a
 * success too: "nothing new" is a valid outcome. */
const refreshing = ref(false)
const refreshResult = ref(null)

async function refreshBuiltins () {
  refreshing.value = true
  refreshResult.value = null
  error.value = ''
  try {
    const r = await ws.management('refresh_node_builtins', {
      node_id: props.node.node_id,
    })
    if (r && r.success === false) {
      error.value = r.message || 'Refresh failed'
      return
    }
    refreshResult.value = r
    emit('updated')  // so the parent reloads peripheral list
  } catch (e) {
    error.value = e.message || String(e)
  } finally {
    refreshing.value = false
  }
}

async function submit () {
  error.value = ''
  if (!displayName.value.trim()) { error.value = 'Give the node a name'; return }
  if (!chipFamily.value) { error.value = 'Please select a chip'; return }
  if (!boardId.value)    { error.value = 'Please select a board'; return }

  const params = {
    node_id: props.node.node_id,
    // role is an optional label; always send it (may be empty) so the
    // operator can clear it, mirroring the display_name behavior.
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

        <!-- Same-board refresh, for when the operator edits a board YAML
             to add a new built-in peripheral (e.g. the Teensy onboard LED)
             and wants existing adopted nodes assigned to that board to
             pick it up without a destructive re-adoption. -->
        <div class="mt-2 flex items-center gap-2">
          <button
            type="button"
            class="btn-secondary text-xs"
            :disabled="refreshing"
            @click="refreshBuiltins">
            <span class="material-icons icon-sm">refresh</span>
            {{ refreshing ? 'Refreshing…' : 'Refresh built-ins from board' }}
          </button>
          <span v-if="refreshResult && refreshResult.added && refreshResult.added.length"
                class="text-xs text-green-400">
            Added: {{ refreshResult.added.join(', ') }}
          </span>
          <span v-else-if="refreshResult"
                class="text-xs text-fg-faint">
            No new built-ins — already up to date.
          </span>
        </div>
      </div>

      <div>
        <label class="block text-sm font-medium text-fg mb-2">Name</label>
        <input v-model="displayName" type="text" class="input-field w-full" placeholder="e.g., Left Arm Controller" />
      </div>

      <div>
        <label class="block text-sm font-medium text-fg mb-2">Role <span class="text-fg-faint">(optional)</span></label>
        <select v-model="role" class="input-field w-full">
          <option value="">-- No role --</option>
          <option v-for="r in roleOptions" :key="r" :value="r">{{ r }}</option>
        </select>
        <p class="text-xs text-fg-faint mt-1">
          {{ robotRoles.length
            ? `Node categories defined by ${robotName || 'the active robot'}.`
            : 'No roles defined by the active robot.' }}
        </p>
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
