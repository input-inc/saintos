<!--
    Preset panel overlay. Shown when a digital binding fires a
    show_panel action; displays a grid of presets the operator can
    navigate with d-pad / sticks and activate with A. The active
    panel + selection comes from useBindings; this component just
    renders + dispatches click→activatePreset.
-->
<script setup lang="ts">
import { computed } from 'vue';
import { useBindings, type PanelItem } from '../composables/useBindings';
import { useConnection } from '../composables/useConnection';
import { useLibrary } from '../composables/useLibrary';

const bindings = useBindings();
const conn = useConnection();
const library = useLibrary();

// The panel's title/icon, group filter, and page indicator now live in
// the app's main header (see App.vue) — this component renders just the
// grid + footer hints.

// For server-backed panels (Animations/Poses/Sounds), explain WHY the
// grid is empty instead of the generic "no presets": not connected vs
// still loading vs genuinely empty on the server.
const emptyState = computed(() => {
    const source = bindings.activePanelSource.value;
    if (!source) return { icon: 'folder_open', text: 'No presets in this panel' };
    const kind = source;   // 'animations' | 'poses' | 'sounds'
    if (!conn.isConnected.value) {
        return { icon: 'cloud_off', text: `Connect to the robot to load ${kind}` };
    }
    const loaded = source === 'animations'
        ? library.animationsLoaded.value
        : (source === 'poses'
            ? library.posesLoaded.value
            : library.soundsLoaded.value);
    if (!loaded) return { icon: 'sync', text: `Loading ${kind}…` };
    return { icon: 'folder_open', text: `No ${kind} saved on the server` };
});

const panel = computed(() => bindings.activePanel.value);
const panelState = computed(() => bindings.activePanelState.value);
const currentPage = computed(() => panelState.value.currentPage);
const selectedIndex = computed(() => panelState.value.selectedIndex);

// Items come from useBindings: server-backed panels (Animations/Poses)
// stream live from the server library; static panels use their presets.
const items = computed<PanelItem[]>(() => bindings.activePanelItems.value);

const totalPages = computed(() => {
    const p = panel.value;
    if (!p) return 0;
    return Math.ceil(items.value.length / p.itemsPerPage);
});

const visiblePresets = computed(() => {
    const p = panel.value;
    if (!p) return [];
    const start = currentPage.value * p.itemsPerPage;
    return items.value.slice(start, start + p.itemsPerPage);
});

function isSelected(pageIndex: number): boolean {
    const p = panel.value;
    if (!p) return false;
    const globalIndex = currentPage.value * p.itemsPerPage + pageIndex;
    return globalIndex === selectedIndex.value;
}

// Animation playback state (only meaningful on the animations panel).
// `isPlaying` drives the "playing" highlight/badge; `isLooping` picks the
// loop badge so the operator knows tapping again will stop it.
function isPlaying(itemId: string): boolean {
    return bindings.activePanelSource.value === 'animations'
        && !!library.playing.value[itemId];
}

function isLooping(itemId: string): boolean {
    return isPlaying(itemId) && !!library.playing.value[itemId]?.loop;
}

function selectPreset(item: PanelItem): void {
    bindings.triggerActiveItem(item.id);
    // Sticky panels (keep_open) stay up so several presets can be fired
    // in a row; otherwise selecting dismisses the panel.
    if (!bindings.activePanelState.value.keepOpen) bindings.hidePanel();
}

function prevPage(): void { bindings.navigatePanel('prev_page'); }
function nextPage(): void { bindings.navigatePanel('next_page'); }
</script>

<template>
    <div v-if="panel" class="h-full flex flex-col"
         :style="{ '--panel-color': panel.color }">

        <!-- Grid Content (title/group/page live in the app header now) -->
        <div class="panel-content flex-1 bg-saint-background p-6 overflow-auto">
            <div class="grid gap-3"
                 :style="{ gridTemplateColumns: 'repeat(4, minmax(0, 1fr))' }">
                <button v-for="(preset, i) in visiblePresets" :key="preset.id"
                        class="preset-item group relative flex items-center gap-3 px-4 py-3 rounded-xl
                               transition-all duration-150 text-left"
                        :class="{ 'preset-selected': isSelected(i), 'preset-playing': isPlaying(preset.id) }"
                        :style="{ '--preset-color': preset.color || panel.color }"
                        @click="selectPreset(preset)">
                    <span class="material-symbols-outlined text-2xl shrink-0 transition-transform group-hover:scale-110"
                          :style="{ color: preset.color || panel.color }">
                        {{ preset.icon || 'radio_button_unchecked' }}
                    </span>
                    <span class="flex-1 min-w-0 text-base font-medium text-saint-text truncate">
                        {{ preset.name }}
                    </span>
                    <!-- Playing badge: pulsing equalizer for a playing
                         animation; loopers add a loop glyph ("tap to stop"). -->
                    <span v-if="isPlaying(preset.id)"
                          class="shrink-0 flex items-center gap-1 text-saint-success">
                        <span class="material-symbols-outlined text-lg animate-pulse">graphic_eq</span>
                        <span v-if="isLooping(preset.id)" class="material-symbols-outlined text-sm">loop</span>
                    </span>
                    <div v-if="isSelected(i)" class="absolute inset-0 rounded-xl border-2 pointer-events-none"
                         :style="{ borderColor: preset.color || panel.color }"></div>
                </button>
            </div>

            <div v-if="visiblePresets.length === 0" class="text-center py-12 text-saint-text-muted">
                <span class="material-symbols-outlined text-4xl mb-2"
                      :class="{ 'animate-spin': emptyState.icon === 'sync' }">{{ emptyState.icon }}</span>
                <p>{{ emptyState.text }}</p>
            </div>
        </div>

        <!-- Footer with controls hint -->
        <div class="panel-footer px-6 py-3 bg-saint-surface border-t border-saint-surface-light
                    flex items-center justify-between text-sm text-saint-text-muted">
            <div class="flex items-center gap-4">
                <span class="flex items-center gap-1">
                    <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-saint-surface-light text-xs font-bold">D</span>
                    Navigate
                </span>
                <span class="flex items-center gap-1">
                    <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-green-600 text-white text-xs font-bold">A</span>
                    Select
                </span>
                <span class="flex items-center gap-1">
                    <span class="inline-flex items-center justify-center w-6 h-6 rounded bg-red-500 text-white text-xs font-bold">B</span>
                    Close
                </span>
                <span v-if="totalPages > 1" class="flex items-center gap-1 ml-2 pl-2 border-l border-saint-surface-light">
                    <span class="inline-flex items-center justify-center px-2 h-6 rounded bg-saint-surface-light text-xs font-bold">LB</span>
                    <span class="inline-flex items-center justify-center px-2 h-6 rounded bg-saint-surface-light text-xs font-bold">RB</span>
                    Page
                </span>
            </div>
            <div class="flex items-center gap-2">
                <template v-if="totalPages > 1">
                    <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors"
                            :disabled="currentPage === 0"
                            :class="{ 'opacity-50': currentPage === 0 }"
                            @click="prevPage">
                        <span class="material-symbols-outlined text-lg">chevron_left</span>
                    </button>
                    <span class="text-saint-text">{{ currentPage + 1 }} / {{ totalPages }}</span>
                    <button class="p-1.5 rounded hover:bg-saint-surface-light transition-colors"
                            :disabled="currentPage >= totalPages - 1"
                            :class="{ 'opacity-50': currentPage >= totalPages - 1 }"
                            @click="nextPage">
                        <span class="material-symbols-outlined text-lg">chevron_right</span>
                    </button>
                </template>
            </div>
        </div>
    </div>
</template>

<style scoped>
.preset-item {
    background: var(--saint-surface, #1e293b);
    border: 1px solid var(--saint-surface-light, #334155);
}
.preset-item:hover {
    background: var(--saint-surface-light, #334155);
    border-color: var(--preset-color);
}
.preset-selected {
    background: color-mix(in srgb, var(--preset-color) 15%, var(--saint-surface, #1e293b));
}
.preset-playing {
    background: color-mix(in srgb, #22c55e 12%, var(--saint-surface, #1e293b));
    box-shadow: inset 0 0 0 2px color-mix(in srgb, #22c55e 55%, transparent);
}
</style>
