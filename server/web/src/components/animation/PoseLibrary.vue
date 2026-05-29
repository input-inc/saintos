<script setup>
import { onMounted, ref } from 'vue'
import { usePosesStore } from '@/stores/poses'
import { useWsStore } from '@/stores/ws'

const poses = usePosesStore()
const ws = useWsStore()

const wsInputs = ref([])      // [{sheet_id, sheet_label, input_id, label, kind}]
const showEditor = ref(false)
const lastApply = ref(null)

async function reloadInputs () {
  try {
    const r = await ws.router('list_websocket_inputs', {})
    wsInputs.value = (r?.ws_inputs || []).filter(x => x.kind !== 'state')
  } catch { /* non-fatal */ }
}

function startNew () {
  poses.startNew()
  showEditor.value = true
}

async function editPose (id) {
  await poses.load(id)
  showEditor.value = true
}

async function apply (id) {
  const r = await poses.apply(id)
  lastApply.value = r
}

function addSetpoint () {
  if (!poses.editing) return
  const first = wsInputs.value[0]
  poses.editing.setpoints.push({
    sheet_id: first?.sheet_id || '',
    ws_input_id: first?.input_id || '',
    value: 0,
  })
  poses.markDirty()
}

function removeSetpoint (idx) {
  poses.editing.setpoints.splice(idx, 1)
  poses.markDirty()
}

async function save () {
  await poses.save()
  if (!poses.error) showEditor.value = false
}

function cancel () {
  showEditor.value = false
}

onMounted(async () => {
  await poses.reload()
  await reloadInputs()
})
</script>

<template>
  <div class="space-y-4">
    <div v-if="!showEditor">
      <div class="flex items-center justify-between mb-4">
        <div>
          <h3 class="text-lg font-semibold text-fg-strong">Pose Library</h3>
          <p class="text-sm text-fg-muted mt-1">
            A pose is a snapshot of WS-input values. Applying one pokes every setpoint
            into the routing graph at once — useful for "go to neutral" or
            "wave hello" presets.
          </p>
        </div>
        <button class="btn-primary" @click="startNew">
          <span class="material-icons icon-sm">add</span>
          New pose
        </button>
      </div>

      <div v-if="poses.error" class="mb-3 p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">
        {{ poses.error }}
      </div>
      <div v-if="lastApply" class="mb-3 p-3 bg-emerald-500/10 border border-emerald-500/30 rounded-lg text-sm text-emerald-300">
        Applied {{ lastApply.applied }} setpoint{{ lastApply.applied === 1 ? '' : 's' }}.
        <span v-if="lastApply.skipped?.length"> Skipped: {{ lastApply.skipped.join(', ') }}</span>
      </div>

      <ul v-if="poses.list.length" class="divide-y divide-line/50 rounded-lg border border-line/50">
        <li v-for="p in poses.list" :key="p.id" class="flex items-center gap-3 p-3">
          <div class="flex-1 min-w-0">
            <div class="text-sm font-medium text-fg-strong truncate">{{ p.name || p.id }}</div>
            <div class="text-xs text-fg-muted">
              {{ p.setpoint_count }} setpoint{{ p.setpoint_count === 1 ? '' : 's' }}
              <span v-if="p.description"> · {{ p.description }}</span>
            </div>
          </div>
          <button class="btn-sm bg-emerald-500/80 hover:bg-emerald-500 text-fg-strong" @click="apply(p.id)">
            <span class="material-icons icon-sm">play_arrow</span>
            Apply
          </button>
          <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong" @click="editPose(p.id)">
            <span class="material-icons icon-sm">edit</span>
          </button>
          <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                  @click="poses.remove(p.id)">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </li>
      </ul>
      <p v-else-if="!poses.loading" class="text-sm text-fg-muted italic">No poses saved yet.</p>
    </div>

    <div v-else class="space-y-4">
      <div class="flex items-center justify-between">
        <h3 class="text-lg font-semibold text-fg-strong">
          {{ poses.editing?.id ? 'Edit pose' : 'New pose' }}
        </h3>
        <div class="flex gap-2">
          <button class="btn-secondary" @click="cancel">Cancel</button>
          <button class="btn-primary" :disabled="!poses.dirty" @click="save">
            <span class="material-icons icon-sm">save</span>
            Save
          </button>
        </div>
      </div>

      <div v-if="poses.error" class="p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">
        {{ poses.error }}
      </div>

      <div class="grid grid-cols-2 gap-4">
        <label class="text-sm">
          <span class="block text-fg-muted mb-1">Name</span>
          <input class="input-field" v-model="poses.editing.name" @input="poses.markDirty()" />
        </label>
        <label class="text-sm">
          <span class="block text-fg-muted mb-1">Description</span>
          <input class="input-field" v-model="poses.editing.description" @input="poses.markDirty()" />
        </label>
      </div>

      <div>
        <div class="flex items-center justify-between mb-2">
          <h4 class="text-sm font-semibold text-fg-strong">Setpoints</h4>
          <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                  :disabled="!wsInputs.length" @click="addSetpoint">
            <span class="material-icons icon-sm">add</span>
            Add setpoint
          </button>
        </div>
        <p v-if="!wsInputs.length" class="text-xs text-amber-300 italic mb-2">
          No WS inputs on any sheet yet — add them in Routes first.
        </p>
        <ul v-if="poses.editing.setpoints.length" class="space-y-2">
          <li v-for="(sp, idx) in poses.editing.setpoints" :key="idx"
              class="flex items-center gap-2 p-2 rounded bg-panel/40 border border-line/40">
            <select class="input-field flex-1" v-model="sp.sheet_id" @change="poses.markDirty()">
              <option v-for="opt in [...new Set(wsInputs.map(w => w.sheet_id))]" :key="opt" :value="opt">
                {{ wsInputs.find(w => w.sheet_id === opt)?.sheet_label || opt }}
              </option>
            </select>
            <select class="input-field flex-1" v-model="sp.ws_input_id" @change="poses.markDirty()">
              <option v-for="w in wsInputs.filter(x => x.sheet_id === sp.sheet_id)"
                      :key="w.input_id" :value="w.input_id">
                {{ w.label || w.input_id }}
              </option>
            </select>
            <input type="number" step="0.01" class="input-field w-28"
                   v-model.number="sp.value" @input="poses.markDirty()" />
            <button class="btn-sm bg-surface hover:bg-red-600 text-fg hover:text-fg-strong"
                    @click="removeSetpoint(idx)">
              <span class="material-icons icon-sm">delete</span>
            </button>
          </li>
        </ul>
        <p v-else class="text-sm text-fg-muted italic">No setpoints — add one to make this pose useful.</p>
      </div>
    </div>
  </div>
</template>
