<script setup>
import { computed, onMounted, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { usePeripheralCatalog } from '@/stores/peripheralCatalog'
import { useWsTopic } from '@/composables/useWsTopic'
import RoboClawMonitor from '@/components/widgets/RoboClawMonitor.vue'
import Fas100Monitor from '@/components/widgets/Fas100Monitor.vue'
import BMSMonitor from '@/components/widgets/BMSMonitor.vue'
import WidgetFrame from '@/components/widgets/WidgetFrame.vue'

defineProps({ embedded: { type: Boolean, default: false } })

const ws = useWsStore()
const catalog = usePeripheralCatalog()
const routing = ref({ sheets: {} })
const widgetCatalog = ref([])

const systemRouting = useWsTopic(() => 'system_routing')

async function load () {
  try {
    const r = await ws.management('get_system_routing', {})
    routing.value = r || { sheets: {} }
  } catch (e) {
    console.warn('widgets load failed:', e)
  }
}
onMounted(() => { catalog.ensureLoaded().then(() => widgetCatalog.value = catalog.widgetTypes); load() })

// Keep in sync with server broadcasts.
const live = computed(() => systemRouting.value || routing.value)

// Per-sheet routing model: flatten widgets and wires across all sheets so
// the Widgets page lists every widget regardless of which controller-node
// sheet (or _dashboard sheet) owns it.
//
// Sorted by the persisted dashboard_order. Array.sort is stable, so
// widgets saved before ordering existed (all tied at 0) keep exactly
// their previous order instead of shuffling on upgrade.
// `_sheetId` is stamped on while flattening: sheets are keyed by node id,
// so the owning sheet is what tells a widget which node to drill into.
// Flattening loses that, hence carrying it explicitly.
const serverWidgets = computed(() => {
  const all = []
  for (const [sheetId, s] of Object.entries(live.value.sheets || {})) {
    for (const w of (s.widgets || [])) all.push({ ...w, _sheetId: sheetId })
  }
  return all.sort((a, b) => (a.dashboard_order ?? 0) - (b.dashboard_order ?? 0))
})

// Local override applied the instant a drop lands, so the card visibly
// moves without waiting for the save + routing rebroadcast round trip.
// Cleared once the server's own order agrees, which is also what makes a
// rejected reorder snap back rather than silently diverge.
const pendingOrder = ref(null)
const widgets = computed(() => {
  const list = serverWidgets.value
  const order = pendingOrder.value
  if (!order) return list
  const byId = new Map(list.map(w => [w.id, w]))
  const out = []
  for (const id of order) {
    const w = byId.get(id)
    if (w) { out.push(w); byId.delete(id) }
  }
  // Anything the pending order doesn't mention (added server-side while
  // a drag was in flight) keeps its place at the end.
  for (const w of byId.values()) out.push(w)
  return out
})
watch(serverWidgets, (list) => {
  if (!pendingOrder.value) return
  const serverIds = list.map(w => w.id).join(',')
  if (serverIds === pendingOrder.value.join(',')) pendingOrder.value = null
})

// ── Drag-to-reorder ─────────────────────────────────────────────────
// HTML5 DnD rather than a library: the grid is a flat list of cards and
// this is the whole of the interaction. Dragging is armed only by a
// pointerdown on the handle — otherwise the entire card would be
// draggable and selecting text in a widget body would start a drag.
const dragId = ref(null)      // widget being dragged
const dragOverId = ref(null)  // widget currently hovered
const dragAfter = ref(false)  // insert after (vs before) the hovered card
const armed = ref(null)       // handle pressed; card is draggable until pointerup
const reorderError = ref('')

function onHandleDown (id) {
  armed.value = id
  // A pointerdown that never becomes a drag must not leave the card
  // draggable — otherwise the next text selection drags the card.
  const release = () => {
    armed.value = null
    window.removeEventListener('pointerup', release)
    window.removeEventListener('pointercancel', release)
  }
  window.addEventListener('pointerup', release)
  window.addEventListener('pointercancel', release)
}

function onDragStart (id, evt) {
  if (armed.value !== id) { evt.preventDefault(); return }
  dragId.value = id
  reorderError.value = ''
  try {
    evt.dataTransfer.effectAllowed = 'move'
    // Firefox ignores a drag with no payload.
    evt.dataTransfer.setData('text/plain', id)
  } catch (_) { /* older browsers */ }
}

function onDragOver (id, evt) {
  if (!dragId.value || id === dragId.value) return
  evt.preventDefault()
  // Which half of the card the pointer is over decides whether the
  // dragged card lands before or after it.
  const r = evt.currentTarget.getBoundingClientRect()
  dragAfter.value = (evt.clientX - r.left) > r.width / 2
  dragOverId.value = id
}

function onDragLeave (id) {
  if (dragOverId.value === id) dragOverId.value = null
}

function onDrop () {
  const from = dragId.value
  const to = dragOverId.value
  const after = dragAfter.value
  onDragEnd()
  if (!from || !to || from === to) return
  const ids = widgets.value.map(w => w.id)
  const fromIdx = ids.indexOf(from)
  if (fromIdx < 0) return
  ids.splice(fromIdx, 1)
  let toIdx = ids.indexOf(to)
  if (toIdx < 0) return
  if (after) toIdx += 1
  ids.splice(toIdx, 0, from)
  commitOrder(ids)
}

function onDragEnd () {
  dragId.value = null
  dragOverId.value = null
  armed.value = null
}

// Keyboard reorder from the focused handle — a drag-only control is
// unusable without a pointer.
function moveBy (id, delta) {
  const ids = widgets.value.map(w => w.id)
  const i = ids.indexOf(id)
  const j = i + delta
  if (i < 0 || j < 0 || j >= ids.length) return
  ids.splice(j, 0, ids.splice(i, 1)[0])
  commitOrder(ids)
}

async function commitOrder (ids) {
  pendingOrder.value = ids
  try {
    const r = await ws.management('reorder_widgets', { widget_ids: ids })
    if (r && r.success === false) {
      reorderError.value = r.message || 'Could not save the new order'
      pendingOrder.value = null   // snap back to the server's order
    }
  } catch (e) {
    reorderError.value = e?.message || String(e)
    pendingOrder.value = null
  }
}
const routes = computed(() => {
  const all = []
  for (const s of Object.values(live.value.sheets || {})) {
    for (const w of (s.wires || [])) all.push(w)
  }
  return all
})

function widgetType (id) { return widgetCatalog.value.find(t => t.id === id) }
function routesIntoWidget (widgetId, inputId) {
  return routes.value.filter(r =>
    r.sink?.kind === 'widget' && r.sink.parts?.[0] === widgetId && r.sink.parts?.[1] === inputId
  )
}
function sourceLabel (route) {
  const s = route.source || {}
  if (s.kind === 'peripheral') return `${s.parts[0]}/${s.parts[1]}/${s.parts[2]}`
  if (s.kind === 'signal') return s.parts.join('/')
  return s.kind + ':' + (s.parts || []).join('/')
}
</script>

<template>
  <section>
    <div v-if="!embedded" class="flex items-center justify-between mb-6">
      <h2 class="text-2xl font-bold text-fg-strong">Widgets</h2>
      <RouterLink to="/routes" class="btn-secondary">
        <span class="material-icons icon-sm">bolt</span>
        Manage routes
      </RouterLink>
    </div>

    <div v-if="!widgets.length" class="card text-center py-10">
      <span class="material-icons icon-lg text-fg-faint">dashboard</span>
      <p class="text-fg-muted text-sm mt-3">No widgets configured yet.</p>
      <p class="text-fg-faint text-xs mt-1">Add widgets and connect them to peripherals from the Routes page.</p>
    </div>

    <p v-if="reorderError" class="mb-3 px-3 py-2 rounded bg-red-500/20 border border-red-500/40 text-sm text-red-300">
      {{ reorderError }}
    </p>

    <div v-if="widgets.length" class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <!-- The drag wiring lives on this wrapper, not inside each widget:
           `draggable` has to sit on the element being dragged, and the
           wrapper is the only thing common to bespoke and generic cards.
           `armed` gates it so only a pointerdown on a handle can start a
           drag — otherwise selecting text in a card body drags it. -->
      <div
        v-for="w in widgets"
        :key="w.id"
        :draggable="armed === w.id"
        @dragstart="onDragStart(w.id, $event)"
        @dragover="onDragOver(w.id, $event)"
        @dragleave="onDragLeave(w.id)"
        @drop.prevent="onDrop"
        @dragend="onDragEnd"
      >
        <!-- Type-specific renderers come first so they can opt out of
             the generic card layout. The dispatcher is a simple switch
             on widget.type — small enough to keep here, will grow into
             a Map<typeId, Component> if we sprout more bespoke types. -->
        <RoboClawMonitor
          v-if="w.type === 'roboclaw_monitor'"
          :widget="w"
          :routes="routes"
          :sheet-id="w._sheetId"
          draggable
          :dragging="dragId === w.id"
          :drop-before="dragOverId === w.id && !dragAfter"
          :drop-after="dragOverId === w.id && dragAfter"
          @handle-down="onHandleDown(w.id)"
          @move-prev="moveBy(w.id, -1)"
          @move-next="moveBy(w.id, 1)"
        />
        <Fas100Monitor
          v-else-if="w.type === 'battery_monitor'"
          :widget="w"
          :routes="routes"
          :sheet-id="w._sheetId"
          draggable
          :dragging="dragId === w.id"
          :drop-before="dragOverId === w.id && !dragAfter"
          :drop-after="dragOverId === w.id && dragAfter"
          @handle-down="onHandleDown(w.id)"
          @move-prev="moveBy(w.id, -1)"
          @move-next="moveBy(w.id, 1)"
        />
        <BMSMonitor
          v-else-if="w.type === 'bms_monitor'"
          :widget="w"
          :routes="routes"
          :sheet-id="w._sheetId"
          draggable
          :dragging="dragId === w.id"
          :drop-before="dragOverId === w.id && !dragAfter"
          :drop-after="dragOverId === w.id && dragAfter"
          @handle-down="onHandleDown(w.id)"
          @move-prev="moveBy(w.id, -1)"
          @move-next="moveBy(w.id, 1)"
        />
        <!-- Generic fallback: same frame, so an unstyled widget type
             still gets the header and handle in the same places. -->
        <WidgetFrame
          v-else
          :widget="w"
          :routes="routes"
          :sheet-id="w._sheetId"
          :icon="widgetType(w.type)?.icon || 'widgets'"
          icon-class="text-fg-faint"
          rule-class="bg-line"
          :badge="widgetType(w.type)?.label || w.type"
          draggable
          :dragging="dragId === w.id"
          :drop-before="dragOverId === w.id && !dragAfter"
          :drop-after="dragOverId === w.id && dragAfter"
          @handle-down="onHandleDown(w.id)"
          @move-prev="moveBy(w.id, -1)"
          @move-next="moveBy(w.id, 1)"
        >
          <div v-if="!(widgetType(w.type)?.inputs?.length)" class="text-xs text-fg-faint italic">
            No declared inputs.
          </div>
          <div v-else class="space-y-1">
            <div
              v-for="inp in widgetType(w.type).inputs"
              :key="inp.id"
              class="flex items-center justify-between text-sm border-b border-line/40 py-1 font-mono"
            >
              <span class="text-fg-muted">{{ inp.display || inp.id }}</span>
              <span v-if="routesIntoWidget(w.id, inp.id).length" class="text-cyan-300 text-xs">
                ← {{ sourceLabel(routesIntoWidget(w.id, inp.id)[0]) }}
              </span>
              <span v-else class="text-fg-faint text-xs">unconnected</span>
            </div>
          </div>
        </WidgetFrame>
      </div>
    </div>
  </section>
</template>
