<script setup>
import { computed } from 'vue'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { specFor } from '@/components/channel/ChannelSpec'
import ChannelSlider from '@/components/channel/ChannelSlider.vue'
import ChannelToggle from '@/components/channel/ChannelToggle.vue'
import ChannelColor  from '@/components/channel/ChannelColor.vue'

const props = defineProps({
  peripheral:  { type: Object, required: true },
  connected:   { type: [Boolean, undefined], default: undefined },
  channelValues: { type: Object, default: () => ({}) },  // { [channel_id]: number }
})
const emit = defineEmits(['commit'])     // ({ channelId, value })

const catalog = usePeripheralCatalog()
const type = computed(() => catalog.byType(props.peripheral.type))
const writable = computed(() => (type.value?.channels || []).filter(c => c.dir === 'out'))
const readable = computed(() => (type.value?.channels || []).filter(c => c.dir === 'in'))

const statusBadge = computed(() => {
  if (props.connected === false) return { label: 'Disconnected', cls: 'bg-rose-900/40 text-rose-300' }
  if (props.connected === true)  return { label: 'Connected',    cls: 'bg-emerald-900/40 text-emerald-300' }
  return null
})

function commit (channelId, value) {
  emit('commit', { channelId, value })
}
</script>

<template>
  <div class="card">
    <header class="mb-3">
      <h4 class="text-base font-semibold flex items-center gap-2 flex-wrap">
        {{ peripheral.label || peripheral.id }}
        <span v-if="peripheral.builtin" class="px-2 py-0.5 rounded-full text-xs bg-slate-700 text-slate-300">Built-in</span>
        <span v-if="statusBadge" :class="['px-2 py-0.5 rounded-full text-xs', statusBadge.cls]">{{ statusBadge.label }}</span>
      </h4>
      <p class="text-xs text-slate-500 mt-0.5">
        {{ type?.label || peripheral.type }} ·
        <span class="font-mono">{{ peripheral.id }}</span>
      </p>
    </header>

    <div v-if="!writable.length" class="text-xs text-slate-500 italic">
      No writable channels — this peripheral only emits readings.
    </div>

    <div v-else class="space-y-4">
      <template v-for="ch in writable" :key="ch.id">
        <template v-if="specFor(peripheral.type, ch).kind === 'slider'">
          <ChannelSlider
            :label="ch.display || ch.id"
            :spec="specFor(peripheral.type, ch)"
            :model-value="channelValues[ch.id]"
            @commit="v => commit(ch.id, v)"
          />
        </template>
        <template v-else-if="specFor(peripheral.type, ch).kind === 'toggle'">
          <ChannelToggle
            :label="ch.display || ch.id"
            :model-value="channelValues[ch.id]"
            @commit="v => commit(ch.id, v)"
          />
        </template>
        <template v-else-if="specFor(peripheral.type, ch).kind === 'color'">
          <ChannelColor
            :label="ch.display || ch.id"
            :spec="specFor(peripheral.type, ch)"
            :model-value="channelValues[ch.id]"
            @commit="v => commit(ch.id, v)"
          />
        </template>
      </template>
    </div>

    <footer v-if="readable.length" class="mt-3 pt-3 border-t border-slate-800 text-xs text-slate-500">
      Readings: {{ readable.map(c => c.display || c.id).join(', ') }}
    </footer>
  </div>
</template>
