<script setup>
// Shared chrome for every dashboard widget: a distinct header strip with
// the widget's icon and name, a slot for its type-specific status chip,
// and the drag handle used to reorder cards.
//
// This exists because each widget type used to render its own card and
// its own header. They were similar but not identical, and there was
// nowhere to put a handle that would land in the same spot on all of
// them. Widgets now supply only their body plus an icon/accent identity.
//
// Accent classes are passed in as complete strings rather than built from
// a colour name (`bg-${accent}-500`). Tailwind can only see class names
// that appear literally in source, so an interpolated one would be
// purged from the production build and silently render unstyled.

import { computed } from 'vue'
import { RouterLink } from 'vue-router'

const props = defineProps({
  widget:    { type: Object, required: true },
  // Id of the routing sheet that OWNS this widget. Sheets are keyed by
  // node id, so for a controller sheet this is the drilldown target
  // directly — the authoritative answer, since the sheet is what the
  // widget actually belongs to.
  //
  // The exception is DASHBOARD_SHEET_ID ("_dashboard"), a pseudo-sheet
  // that is deliberately not node-scoped; those fall back to `routes`.
  sheetId:   { type: String, default: '' },
  // The widget's wires. Only consulted for `_dashboard`-owned widgets,
  // where the sheet can't name a node. Derived here rather than passed
  // in as a node id so the three bespoke widgets and the generic
  // fallback all share one implementation.
  routes:    { type: Array, default: () => [] },
  // Material icon name + the classes that give this widget type its
  // identity in the header.
  icon:      { type: String, default: 'widgets' },
  iconClass: { type: String, default: 'text-cyan-400' },
  // Thin rule under the header. Also the drag-affordance colour.
  ruleClass: { type: String, default: 'bg-cyan-500' },
  // Short name for the device behind this widget — "BMS", "FAS100",
  // "RoboClaw". Consistently the widget's IDENTITY on every card; health
  // goes in statusDot instead, so the chip never means two things
  // depending on which card you're reading.
  badge:      { type: String, default: '' },
  badgeClass: { type: String, default: 'bg-surface text-fg-muted' },
  // Optional health indicator, left of the badge. `statusDot` is the
  // full colour/animation class string; `statusLabel` is its meaning,
  // surfaced as a tooltip and to screen readers since colour alone
  // isn't a label.
  statusDot:   { type: String, default: '' },
  statusLabel: { type: String, default: '' },
  // Reorder affordances. Off by default so the frame stays usable in
  // contexts with no ordering (the Histoire stories, for one).
  draggable: { type: Boolean, default: false },
  dragging:  { type: Boolean, default: false },
  dropBefore: { type: Boolean, default: false },
  dropAfter:  { type: Boolean, default: false },
})

const emit = defineEmits([
  'handle-down',   // pointer went down on the handle — arms the drag
  'move-prev',     // keyboard reorder, one slot earlier
  'move-next',     // keyboard reorder, one slot later
])

const DASHBOARD_SHEET_ID = '_dashboard'

// Distinct nodes feeding this widget, from its peripheral-sourced wires.
// A fallback only — a wire tells you where a VALUE came from, which
// isn't the same as what the widget belongs to.
const sourceNodes = computed(() => {
  const ids = new Set()
  for (const r of props.routes) {
    if (r?.sink?.kind !== 'widget') continue
    if (r.sink.parts?.[0] !== props.widget.id) continue
    if (r?.source?.kind !== 'peripheral') continue
    const nodeId = r.source.parts?.[0]
    if (nodeId) ids.add(nodeId)
  }
  return [...ids]
})

// Where "more information" lives for this widget.
//
// The owning sheet first: it's keyed by node id and is what the widget
// actually belongs to. Only the non-node-scoped `_dashboard` sheet needs
// the wire fallback, and there we link solely when every peripheral wire
// agrees on one node — a widget fed from two nodes has no single target,
// and picking the first would send the operator somewhere misleading.
const drilldownNode = computed(() => {
  if (props.sheetId && props.sheetId !== DASHBOARD_SHEET_ID) return props.sheetId
  return sourceNodes.value.length === 1 ? sourceNodes.value[0] : null
})
</script>

<template>
  <div
    class="card relative transition-opacity"
    :class="[
      dragging ? 'opacity-40' : '',
      // Insertion marker. A ring on the whole card would compete with
      // the status chips; an edge bar reads as 'it lands here'.
      dropBefore ? 'before:absolute before:-left-2 before:inset-y-2 before:w-1 before:rounded-full before:bg-cyan-400' : '',
      dropAfter  ? 'after:absolute after:-right-2 after:inset-y-2 after:w-1 after:rounded-full after:bg-cyan-400' : '',
    ]"
    :data-widget-id="widget.id"
  >
    <header class="flex items-center gap-2 mb-3">
      <span class="material-icons icon-md shrink-0" :class="iconClass">{{ icon }}</span>

      <!-- The name drills down to the node's Live readings tab, which
           shows every channel with its live value rather than the
           handful this card summarises. Plain text when no single node
           owns the widget — see drilldownNode. -->
      <h4 class="text-base font-semibold truncate min-w-0">
        <RouterLink
          v-if="drilldownNode"
          :to="{ name: 'node-live', params: { id: drilldownNode } }"
          class="text-fg-strong hover:text-cyan-300 hover:underline decoration-cyan-400/50
                 underline-offset-2 transition-colors inline-flex items-center gap-1 group/link"
          :title="`Live readings for ${drilldownNode}`"
        >
          <span class="truncate">{{ widget.label || widget.id }}</span>
          <span class="material-icons text-[14px] leading-none text-fg-faint
                       opacity-0 group-hover/link:opacity-100 transition-opacity shrink-0">
            open_in_new
          </span>
        </RouterLink>
        <span v-else class="text-fg-strong">{{ widget.label || widget.id }}</span>
      </h4>

      <!-- Type-specific chip (OK / FAULT / device name). Pushed right,
           and sits left of the handle so the handle's position is
           identical on every card regardless of chip width. -->
      <div class="ml-auto flex items-center gap-1.5 shrink-0">
        <span
          v-if="statusDot"
          class="w-2 h-2 rounded-full"
          :class="statusDot"
          :title="statusLabel"
          role="img"
          :aria-label="statusLabel"
        />
        <span
          v-if="badge"
          class="px-2 py-0.5 text-xs font-medium rounded-full whitespace-nowrap"
          :class="badgeClass"
        >{{ badge }}</span>
        <slot name="status" />

        <button
          v-if="draggable"
          type="button"
          class="drag-handle -mr-1 p-1 rounded text-fg-faint hover:text-fg-strong hover:bg-surface/60
                 focus-visible:outline focus-visible:outline-2 focus-visible:outline-cyan-500
                 cursor-grab active:cursor-grabbing transition-colors"
          :aria-label="`Reorder ${widget.label || widget.id}. Drag, or use arrow keys.`"
          title="Drag to reorder — or focus and use ← →"
          @pointerdown="emit('handle-down', $event)"
          @keydown.left.prevent="emit('move-prev')"
          @keydown.up.prevent="emit('move-prev')"
          @keydown.right.prevent="emit('move-next')"
          @keydown.down.prevent="emit('move-next')"
          @click.prevent
        >
          <span class="material-icons icon-sm">drag_indicator</span>
        </button>
      </div>
    </header>

    <div class="h-0.5 rounded-full mb-3" :class="ruleClass"></div>

    <slot />
  </div>
</template>
