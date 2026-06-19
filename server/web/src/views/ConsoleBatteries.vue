<script setup>
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { useRoute } from 'vue-router'
import ConsoleBatteriesOverview from '@/components/console/ConsoleBatteriesOverview.vue'
import { useNodesStore } from '@/stores/nodes'
import { useWsStore } from '@/stores/ws'

// Multi-pack console wrapper. Two modes:
//
//   1. Explicit list: ?packs=JSON  — the console_display peripheral
//      builds this URL when the operator picked a curated subset.
//
//   2. Auto-discover (default): no `packs` query  — walk every
//      adopted node, ask the server for its peripherals, and surface
//      anything that looks like a BMS. This is what the default
//      console_display config produces: "show me all the batteries".
//
// Re-discovery: when adopted_nodes changes (a new BMS comes online),
// we refresh. The view also subscribes to per-node pin_state in the
// overview component itself, so values stream in regardless of which
// mode we're in.

const route = useRoute()
const ws = useWsStore()
const nodesStore = useNodesStore()

const BMS_TYPES = new Set(['pathfinder_bms', 'jbd_bms'])

const explicitPacks = computed(() => {
  const raw = route.query.packs
  if (typeof raw !== 'string' || !raw.length) return null
  try {
    const parsed = JSON.parse(raw)
    return Array.isArray(parsed) ? parsed : null
  } catch (e) {
    console.warn('console-batteries: invalid packs query param:', e)
    return null
  }
})

const discoveredPacks = ref([])

async function discover () {
  // Fan out to every adopted node and pick out BMS-typed peripherals.
  // get_node_peripherals returns the persisted peripheral_config —
  // adopted-but-empty nodes simply yield no entries.
  const nodes = nodesStore.all || []
  const acc = []
  await Promise.all(nodes.map(async n => {
    try {
      const r = await ws.management('get_node_peripherals', { node_id: n.node_id })
      for (const p of (r?.peripherals || [])) {
        if (BMS_TYPES.has(p.type)) {
          acc.push({
            nodeId: n.node_id,
            peripheralId: p.id,
            label: p.label || p.id,
          })
        }
      }
    } catch (_) {
      // One node failing shouldn't kill the whole overview; just skip.
    }
  }))
  discoveredPacks.value = acc
}

// Active list = explicit if provided, else auto-discovered.
const packs = computed(() =>
  explicitPacks.value !== null ? explicitPacks.value : discoveredPacks.value
)

// Re-discover when the set of adopted nodes changes (so a BMS coming
// online after kiosk launch shows up without a reload).
watch(() => (nodesStore.all || []).map(n => n.node_id).join(','),
      () => { if (explicitPacks.value === null) discover() })

onMounted(() => {
  if (explicitPacks.value === null) discover()
})
onUnmounted(() => {})
</script>

<template>
  <div class="h-full w-full">
    <ConsoleBatteriesOverview :packs="packs" />
  </div>
</template>
