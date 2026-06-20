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

const bindings = useBindings();

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

function selectPreset(item: PanelItem): void {
    bindings.triggerActiveItem(item.id);
    bindings.hidePanel();
}

function close(): void { bindings.hidePanel(); }
function prevPage(): void { bindings.navigatePanel('prev_page'); }
function nextPage(): void { bindings.navigatePanel('next_page'); }
</script>

<template>
    <div v-if="panel" class="h-full flex flex-col"
         :style="{ '--panel-color': panel.color }">

        <!-- Header -->
        <div class="panel-header px-6 py-4 flex items-center justify-between"
             :style="{ background: panel.color }">
            <div class="flex items-center gap-3">
                <span class="material-symbols-outlined text-2xl text-white">{{ panel.icon }}</span>
                <h2 class="text-xl font-semibold text-white">{{ panel.name }}</h2>
            </div>
            <div class="flex items-center gap-4">
                <div class="text-white/80 text-sm">
                    Page {{ currentPage + 1 }}/{{ totalPages }}
                </div>
                <button class="p-1.5 rounded hover:bg-white/20 transition-colors text-white"
                        @click="close">
                    <span class="material-symbols-outlined">close</span>
                </button>
            </div>
        </div>

        <!-- Grid Content -->
        <div class="panel-content flex-1 bg-saint-background p-6 overflow-auto">
            <div class="grid gap-4 max-w-4xl mx-auto"
                 :style="{ gridTemplateColumns: `repeat(${panel.columns}, 1fr)` }">
                <button v-for="(preset, i) in visiblePresets" :key="preset.id"
                        class="preset-item group relative flex flex-col items-center justify-center
                               p-6 rounded-xl transition-all duration-150"
                        :class="{ 'preset-selected': isSelected(i) }"
                        :style="{ '--preset-color': preset.color || panel.color }"
                        @click="selectPreset(preset)">
                    <span class="material-symbols-outlined text-4xl mb-3 transition-transform
                                 group-hover:scale-110"
                          :style="{ color: preset.color || panel.color }">
                        {{ preset.icon || 'radio_button_unchecked' }}
                    </span>
                    <span class="text-base font-medium text-saint-text text-center">
                        {{ preset.name }}
                    </span>
                    <div v-if="isSelected(i)" class="absolute inset-0 rounded-xl border-2 pointer-events-none"
                         :style="{ borderColor: preset.color || panel.color }"></div>
                </button>
            </div>

            <div v-if="visiblePresets.length === 0" class="text-center py-12 text-saint-text-muted">
                <span class="material-symbols-outlined text-4xl mb-2">folder_open</span>
                <p>No presets in this panel</p>
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
</style>
