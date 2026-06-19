<script setup>
import { computed } from 'vue'
import { useFirmwareUpdatesStore, STAGE_LABELS } from '@/stores/firmwareUpdates'

// Inline OTA progress strip for a single node. Renders nothing when
// no update is in flight for nodeId, so it's safe to drop into any
// node-scoped card.
//
// Designed to be generic across chip families. As long as the node's
// firmware-update flow publishes frames on /saint/nodes/<id>/
// update_progress (the Pi does this via saint_node.updater; RP2040 /
// Teensy can adopt the same topic from their bootloaders), this
// component renders them with zero per-family branching.
//
// Two visual modes:
//   - Determinate: `downloading` stage + percent < 100 → fills the
//     bar proportionally. The slow Pi OTA spends nearly all its time
//     here, so the percent is the key signal.
//   - Indeterminate: every other stage → full-width pulsing bar with
//     the stage label, because the percent jumps in big discrete
//     steps that don't read naturally as a bar.
//
// Variant prop tunes density: "card" suits a small Nodes-view tile
// (label + thin bar), "panel" suits the bigger Overview status card
// (label + bar + message line).

const props = defineProps({
  nodeId:  { type: String, required: true },
  variant: { type: String, default: 'panel' },  // 'panel' | 'card'
})

const store = useFirmwareUpdatesStore()
const entry = store.byId(props.nodeId)

const visible = computed(() => entry.value != null)

const isFailed   = computed(() => entry.value?.status === 'failed')
const isComplete = computed(() => entry.value?.status === 'complete')

const stageLabel = computed(() => {
  if (!entry.value) return ''
  if (isFailed.value)   return 'Update failed'
  if (isComplete.value) return 'Update complete'
  return STAGE_LABELS[entry.value.stage] || entry.value.stage || 'Updating'
})

const isDeterminate = computed(() =>
  !isFailed.value &&
  !isComplete.value &&
  entry.value?.stage === 'downloading' &&
  typeof entry.value?.percent === 'number' &&
  entry.value.percent < 100
)

const percentDisplay = computed(() => {
  const p = entry.value?.percent
  return (typeof p === 'number' && p >= 0 && p <= 100) ? Math.round(p) : 0
})

const barClass = computed(() => {
  if (isFailed.value)   return 'bg-red-500'
  if (isComplete.value) return 'bg-emerald-500'
  return 'bg-cyan-500'
})
</script>

<template>
  <div v-if="visible" :class="variant === 'card' ? 'mt-2' : 'mt-3'">
    <div class="flex items-center justify-between text-xs text-fg-muted mb-1">
      <span class="flex items-center gap-1.5">
        <span class="material-icons icon-xs"
              :class="isFailed ? 'text-red-400' : (isComplete ? 'text-emerald-400' : 'text-cyan-400')">
          {{ isFailed ? 'error' : (isComplete ? 'check_circle' : 'cloud_download') }}
        </span>
        <span>{{ stageLabel }}</span>
      </span>
      <span v-if="isDeterminate" class="tabular-nums">{{ percentDisplay }}%</span>
    </div>
    <!-- Bar height differs by variant so the card variant doesn't
         dominate a small tile while the panel variant feels solid. -->
    <div class="w-full bg-surface rounded-full overflow-hidden"
         :class="variant === 'card' ? 'h-1' : 'h-1.5'">
      <div
        v-if="isComplete || isFailed"
        :class="['h-full', barClass]"
        style="width: 100%"
      ></div>
      <div
        v-else-if="isDeterminate"
        :class="['h-full', barClass, 'transition-[width] duration-300 ease-out']"
        :style="{ width: percentDisplay + '%' }"
      ></div>
      <div
        v-else
        :class="['h-full', barClass, 'animate-pulse']"
        style="width: 100%"
      ></div>
    </div>
    <p v-if="variant === 'panel' && entry?.message" class="text-xs text-fg-muted mt-1">
      {{ entry.message }}
    </p>
  </div>
</template>
