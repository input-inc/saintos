<script setup>
import { computed, ref, watch } from 'vue'
import AppModal from '@/components/AppModal.vue'
import IconPicker from './IconPicker.vue'

// Offers the SRDF's <group_state> definitions as poses. Shown right
// after an SRDF upload (and reachable again from Settings), because a
// group_state IS a named pose — that's the whole reason the pose library
// gets a standard encoding instead of a bespoke one.
//
// Two things this screen has to get right:
//
//  * Show BOTH unit systems. The SRDF authors joint values in URDF-native
//    units (radians / metres); poses store the normalized −1..+1 that
//    every sink downstream of the routing graph speaks. Showing only one
//    makes the other look wrong, and an operator comparing this against
//    their SRDF in a text editor needs to see the authored number.
//  * Never silently clobber. A pose imported once and then hand-tuned
//    must not be overwritten by a re-uploaded SRDF unless asked.

const props = defineProps({
  // [{ name, group, joint_values, normalized, unresolved, joint_count,
  //    pose_id, exists, locally_edited }]
  states: { type: Array, default: () => [] },
  // Existing pose group names, for the datalist.
  poseGroups: { type: Array, default: () => [] },
  busy: { type: Boolean, default: false },
})
const emit = defineEmits(['close', 'import'])

const selected = ref(new Set())
const group = ref('')
const icon = ref('accessibility')
const overwrite = ref(false)
const showValues = ref(null)      // name of the row whose joints are expanded

// Anything already present starts unchecked — the default action should
// never be the destructive one.
watch(() => props.states, (states) => {
  selected.value = new Set(
    (states || []).filter(s => !s.exists && s.joint_count > 0).map(s => s.name))
  // Prefill the group from the SRDF's own grouping when it's unanimous,
  // so the common single-group case needs no typing.
  const groups = [...new Set((states || []).map(s => s.group).filter(Boolean))]
  if (groups.length === 1) group.value = groups[0]
}, { immediate: true })

const importable = computed(() =>
  props.states.filter(s => s.joint_count > 0))
const unimportable = computed(() =>
  props.states.filter(s => s.joint_count === 0))

// A row is blocked when it exists and we're not overwriting.
function blocked (s) {
  return s.exists && !overwrite.value
}
const selectable = computed(() =>
  importable.value.filter(s => !blocked(s)))

const chosen = computed(() =>
  selectable.value.filter(s => selected.value.has(s.name)))

const anyExisting = computed(() => importable.value.some(s => s.exists))
const anyEdited = computed(() =>
  importable.value.some(s => s.exists && s.locally_edited))

function toggle (name) {
  const next = new Set(selected.value)
  if (next.has(name)) next.delete(name)
  else next.add(name)
  selected.value = next
}
function selectAll () {
  selected.value = new Set(selectable.value.map(s => s.name))
}
function selectNone () {
  selected.value = new Set()
}

function submit () {
  if (!chosen.value.length) return
  emit('import', {
    names: chosen.value.map(s => s.name),
    group: group.value.trim(),
    icon: icon.value,
    overwrite: overwrite.value,
  })
}

function fmt (v) {
  const n = Number(v)
  if (!Number.isFinite(n)) return '—'
  return n.toFixed(3).replace(/\.?0+$/, '') || '0'
}
function jointRows (s) {
  return Object.keys(s.normalized || {}).sort().map(joint => ({
    joint,
    native: s.joint_values?.[joint],
    normalized: s.normalized?.[joint],
  }))
}
</script>

<template>
  <AppModal title="Import poses from SRDF" width="max-w-3xl" @close="emit('close')">
    <div class="space-y-4">
      <p class="text-xs text-fg-muted">
        This SRDF defines
        <strong class="text-fg-strong">{{ states.length }}</strong>
        <code class="text-cyan-300 mx-1">&lt;group_state&gt;</code>
        definition{{ states.length === 1 ? '' : 's' }} — named joint
        configurations, which is exactly what a pose is. Import the ones you
        want and they'll appear in the pose library, usable on the soundboard,
        in animations, and as rig control targets.
      </p>

      <!-- Nothing importable at all. -->
      <div v-if="!importable.length"
           class="rounded-lg border border-amber-500/40 bg-amber-500/10 p-3 text-sm text-amber-200">
        None of these group_states name joints that exist in the installed
        URDF, so there's nothing to import. Check that the SRDF and URDF are
        for the same robot.
      </div>

      <template v-else>
        <!-- Bulk selection + shared metadata. -->
        <div class="flex flex-wrap items-end gap-3">
          <div class="flex items-center gap-2">
            <IconPicker v-model="icon" fallback="accessibility" />
            <label class="block">
              <span class="block text-fg-muted text-xs mb-1">Pose group</span>
              <input class="input-field w-48"
                     list="import-pose-groups"
                     v-model="group"
                     placeholder="Leave empty for Ungrouped" />
              <datalist id="import-pose-groups">
                <option v-for="g in poseGroups" :key="g" :value="g" />
              </datalist>
            </label>
          </div>
          <div class="flex items-center gap-1 ml-auto">
            <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                    @click="selectAll">All</button>
            <button class="btn-sm bg-surface hover:bg-surface-2 text-fg-strong"
                    @click="selectNone">None</button>
          </div>
        </div>

        <!-- The overwrite gate. Off by default, and it says what it -->
        <!-- would cost when any target has local edits.             -->
        <label v-if="anyExisting"
               class="flex items-start gap-2 rounded-lg border border-line/50 bg-panel/30 p-3 cursor-pointer">
          <input type="checkbox" class="mt-0.5" v-model="overwrite" />
          <span class="text-xs">
            <span class="text-fg-strong">Overwrite poses that already exist</span>
            <span v-if="anyEdited" class="block text-amber-300 mt-0.5">
              Some of these have been edited since they were imported —
              overwriting discards those changes and re-derives the values
              from the SRDF.
            </span>
            <span v-else class="block text-fg-muted mt-0.5">
              Off by default, so a re-uploaded SRDF can't quietly replace
              work done in the editor.
            </span>
          </span>
        </label>

        <!-- The candidate table. -->
        <div class="rounded-lg border border-line/50 overflow-hidden">
          <table class="w-full text-sm">
            <thead class="bg-surface/60 text-xs text-fg-muted">
              <tr>
                <th class="w-8 p-2"></th>
                <th class="text-left p-2 font-medium">group_state</th>
                <th class="text-left p-2 font-medium">SRDF group</th>
                <th class="text-right p-2 font-medium">Joints</th>
                <th class="text-left p-2 font-medium">Status</th>
                <th class="w-8 p-2"></th>
              </tr>
            </thead>
            <tbody>
              <template v-for="s in importable" :key="s.name">
                <tr :class="['border-t border-line/30',
                             blocked(s) ? 'opacity-50' : 'hover:bg-surface/40']">
                  <td class="p-2 text-center">
                    <input type="checkbox"
                           :disabled="blocked(s)"
                           :checked="selected.has(s.name)"
                           @change="toggle(s.name)" />
                  </td>
                  <td class="p-2 text-fg-strong font-medium">{{ s.name }}</td>
                  <td class="p-2 text-fg-muted">{{ s.group || '—' }}</td>
                  <td class="p-2 text-right tabular-nums">{{ s.joint_count }}</td>
                  <td class="p-2">
                    <span v-if="s.exists && s.locally_edited"
                          class="text-amber-300 text-xs">Exists · edited</span>
                    <span v-else-if="s.exists" class="text-fg-muted text-xs">Exists</span>
                    <span v-else class="text-emerald-400 text-xs">New</span>
                    <span v-if="s.unresolved?.length"
                          class="block text-amber-300 text-xs"
                          :title="s.unresolved.join(', ')">
                      {{ s.unresolved.length }} joint(s) not in URDF — will be dropped
                    </span>
                  </td>
                  <td class="p-2 text-center">
                    <button class="text-fg-muted hover:text-fg-strong"
                            :title="showValues === s.name ? 'Hide joint values' : 'Show joint values'"
                            @click="showValues = showValues === s.name ? null : s.name">
                      <span class="material-icons icon-sm">
                        {{ showValues === s.name ? 'expand_less' : 'expand_more' }}
                      </span>
                    </button>
                  </td>
                </tr>
                <!-- Both unit systems side by side: the authored value -->
                <!-- and what actually gets stored.                     -->
                <tr v-if="showValues === s.name" class="border-t border-line/30 bg-surface/30">
                  <td></td>
                  <td colspan="5" class="p-2">
                    <table class="text-xs">
                      <thead class="text-fg-muted">
                        <tr>
                          <th class="text-left pr-4 font-medium">Joint</th>
                          <th class="text-right pr-4 font-medium">SRDF (rad/m)</th>
                          <th class="text-right font-medium">Stored (−1…+1)</th>
                        </tr>
                      </thead>
                      <tbody class="tabular-nums">
                        <tr v-for="row in jointRows(s)" :key="row.joint">
                          <td class="pr-4 text-fg font-mono">{{ row.joint }}</td>
                          <td class="pr-4 text-right text-fg-muted">{{ fmt(row.native) }}</td>
                          <td class="text-right text-fg-strong">{{ fmt(row.normalized) }}</td>
                        </tr>
                      </tbody>
                    </table>
                    <p class="text-fg-faint mt-2" style="font-size:11px">
                      Converted once, here, against each joint's URDF
                      <code>&lt;limit&gt;</code>. Stored 0 is the midpoint of a
                      joint's travel, so an asymmetric joint's native 0 is not 0.
                    </p>
                  </td>
                </tr>
              </template>
            </tbody>
          </table>
        </div>

        <div v-if="unimportable.length"
             class="text-xs text-fg-muted">
          Skipping {{ unimportable.length }} group_state(s) with no joints
          resolvable against the URDF:
          <span class="text-fg">{{ unimportable.map(s => s.name).join(', ') }}</span>
        </div>
      </template>
    </div>

    <template #actions>
      <button class="btn-secondary" @click="emit('close')">
        {{ importable.length ? 'Not now' : 'Close' }}
      </button>
      <button v-if="importable.length"
              class="btn-primary"
              :disabled="!chosen.length || busy"
              @click="submit">
        <span class="material-icons icon-sm">library_add</span>
        {{ busy ? 'Importing…'
                : chosen.length
                  ? `Import ${chosen.length} pose${chosen.length === 1 ? '' : 's'}`
                  : 'Import' }}
      </button>
    </template>
  </AppModal>
</template>
