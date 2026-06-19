<script setup>
import { computed } from 'vue'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { specFor } from '@/components/channel/ChannelSpec'
import ChannelSlider from '@/components/channel/ChannelSlider.vue'
import ChannelToggle from '@/components/channel/ChannelToggle.vue'
import ChannelColor  from '@/components/channel/ChannelColor.vue'

const props = defineProps({
  peripheral:    { type: Object, required: true },
  connected:     { type: [Boolean, undefined], default: undefined },
  channelValues: { type: Object, default: () => ({}) },
})
const emit = defineEmits(['commit'])

const catalog = usePeripheralCatalog()
const type = computed(() => catalog.byType(props.peripheral.type))
const writable = computed(() => (type.value?.channels || []).filter(c => c.dir === 'out'))
const readable = computed(() => (type.value?.channels || []).filter(c => c.dir === 'in'))

const statusBadge = computed(() => {
  if (props.connected === false) return { label: 'Disconnected', cls: 'bg-red-500/20 text-red-400' }
  if (props.connected === true)  return { label: 'Connected',    cls: 'bg-emerald-500/20 text-emerald-400' }
  return null
})

function commit (channelId, value) { emit('commit', { channelId, value }) }

// Show the operator's per-channel name (set in the channel-edit modal,
// stored in params.channels[idx].label) instead of the generic catalog
// id. Mirrors channelDisplayLabel in views/node/Peripherals.vue. Falls
// back to the catalog display/id for channels with no custom label.
function channelLabel (ch) {
  const m = /^ch(\d+)$/.exec(ch.id)
  if (m) {
    const entry = props.peripheral.params?.channels?.[Number(m[1])]
    const label = entry?.label
    if (label && String(label).trim().length) return label
  }
  return ch.display || ch.id
}
</script>

<template>
  <div class="card">
    <header class="flex items-start justify-between mb-4">
      <div>
        <h4 class="text-base font-semibold text-fg-strong flex items-center gap-2 flex-wrap">
          {{ peripheral.label || peripheral.id }}
          <span v-if="peripheral.builtin" class="px-2 py-0.5 text-xs rounded-full bg-surface text-fg">Built-in</span>
          <span v-if="statusBadge" :class="['px-2 py-0.5 text-xs font-medium rounded-full', statusBadge.cls]">{{ statusBadge.label }}</span>
        </h4>
        <p class="text-xs text-fg-faint mt-0.5">
          {{ type?.label || peripheral.type }} · <span class="font-mono">{{ peripheral.id }}</span>
        </p>
      </div>
    </header>

    <div v-if="!writable.length" class="text-xs text-fg-faint italic">
      No writable channels — this peripheral only emits readings.
    </div>

    <div v-else class="space-y-5">
      <template v-for="ch in writable" :key="ch.id">
        <component
          :is="{ slider: ChannelSlider, toggle: ChannelToggle, color: ChannelColor }[specFor(peripheral.type, ch).kind] || 'div'"
          :label="channelLabel(ch)"
          :spec="specFor(peripheral.type, ch)"
          :model-value="channelValues[ch.id]"
          @commit="v => commit(ch.id, v)"
        />
      </template>
    </div>

    <footer v-if="readable.length" class="mt-4 pt-4 border-t border-line/50 text-xs text-fg-faint">
      <span class="font-medium uppercase tracking-wide">Readings</span>
      <span class="ml-2">{{ readable.map(c => c.display || c.id).join(', ') }}</span>
    </footer>
  </div>
</template>
