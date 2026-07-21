<!--
    Root shell. Header (nav OR active-panel chrome + connection
    dropdown + E-Stop), e-stop banner, main (router-outlet OR
    preset-panel overlay), and the virtual keyboard overlay.
    Battery/WiFi live on the Dashboard tab; on-screen controls were
    removed (physical gamepad only).
-->
<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from 'vue';
import { RouterLink, RouterView } from 'vue-router';
import PresetPanel from './components/PresetPanel.vue';
import VirtualKeyboard from './components/VirtualKeyboard.vue';
import SaintSelect from './components/SaintSelect.vue';
import { useConnection, ConnectionStatus } from './composables/useConnection';
import { useInput, type ButtonEvent } from './composables/useInput';
import { useBindings, type DigitalInput, type DigitalAction } from './composables/useBindings';
import { useKeyboard } from './composables/useKeyboard';
import { useLibrary } from './composables/useLibrary';

const conn = useConnection();
const input = useInput();
const bindings = useBindings();
const keyboard = useKeyboard();
const library = useLibrary();

// ─── Reactive state ──────────────────────────────────────────────────

// Written by the physical-button handlers; kept for hold-mode tracking.
const virtualPressed = ref<Record<string, boolean>>({});
let heldButton: DigitalInput | null = null;

// Hardware-button-name → DigitalInput key. Keeps the binding lookup
// keyed on canonical short names while the gamepad poll emits whatever
// the platform reports.
const buttonMap: Record<string, DigitalInput> = {
    A: 'a', B: 'b', X: 'x', Y: 'y',
    LB: 'lb', RB: 'rb',
    DPadUp: 'd_pad_up', DPadDown: 'd_pad_down',
    DPadLeft: 'd_pad_left', DPadRight: 'd_pad_right',
    Start: 'start', Select: 'select',
    LeftStick: 'left_stick', RightStick: 'right_stick',
    L4: 'l4', R4: 'r4', L5: 'l5', R5: 'r5', Steam: 'steam',
};

// ─── Connection dropdown + active-panel header state ─────────────────
const connMenuOpen = ref(false);

// The preset panel currently open drives the header's icon/name/color,
// group filter, and page indicator (the panel no longer has its own header).
const headerPanel = computed(() => bindings.activePanel.value);
const headerGroups = computed(() => bindings.activePanelGroups.value);
const headerHasGroups = computed(() => headerGroups.value.length > 0);
const headerGroupOptions = computed(() => [
    { value: 'All', label: 'All' },
    ...headerGroups.value.map(g => ({ value: g, label: g })),
]);
const headerSelectedGroup = computed<string>({
    get: () => bindings.activePanelSelectedGroup.value,
    set: (v) => bindings.setActiveGroup(v),
});

function connDotClass(): string {
    switch (conn.status.value) {
        case ConnectionStatus.Connected:      return 'text-saint-success';
        case ConnectionStatus.Connecting:
        case ConnectionStatus.Authenticating: return 'text-yellow-500';
        default:                              return 'text-red-500';
    }
}

async function onReconnect(): Promise<void> {
    connMenuOpen.value = false;
    try { await conn.reconnect(); }
    catch (err) { console.error('Reconnect failed:', err); }
}

const presetPanelActive = computed(() =>
    bindings.activePanelState.value.activePanelId !== null);

// ─── Status display ──────────────────────────────────────────────────

function getStatusText(): string {
    switch (conn.status.value) {
        case ConnectionStatus.Connected:     return 'Connected';
        case ConnectionStatus.Connecting:    return 'Connecting...';
        case ConnectionStatus.Authenticating: return 'Authenticating...';
        default:                             return 'Disconnected';
    }
}

async function emergencyStop(): Promise<void> {
    try { await conn.emergencyStop(); }
    catch (err) { console.error('Emergency stop failed:', err); }
}

// ─── Binding-action execution ────────────────────────────────────────

function executeDigitalAction(action: DigitalAction): void {
    switch (action.type) {
        case 'show_panel':       bindings.showPanel(action.panel_id, action.default_group, action.keep_open); break;
        case 'hide_panel':       bindings.hidePanel(); break;
        case 'navigate_panel':   bindings.navigatePanel(action.direction); break;
        case 'select_panel_item': bindings.selectCurrentItem(); break;
        case 'activate_preset':  void bindings.activatePreset(action.preset_id); break;
        case 'e_stop':           void emergencyStop(); break;
        case 'toggle_output':    console.log('Toggle output:', action.target_id); break;
        case 'cycle_output':     console.log('Cycle output:', action.target_id, action.values); break;
        case 'direct_control':   console.log('Direct control:', action.target, action.value); break;
        case 'none':             break;
    }
}

function onVirtualButtonDown(button: DigitalInput): void {
    virtualPressed.value = { ...virtualPressed.value, [button]: true };

    const profile = bindings.activeProfile.value;
    if (!profile) return;

    const pressBinding = profile.digitalBindings.find(
        b => b.input === button && b.trigger === 'press' && b.enabled);
    const holdBinding = profile.digitalBindings.find(
        b => b.input === button && b.trigger === 'hold' && b.enabled);

    const panelActivation = profile.settings.panelActivation || 'press';

    // show_panel actions: hold-mode shows on press + hides on release;
    // press-mode toggles.
    if (pressBinding?.action.type === 'show_panel' || holdBinding?.action.type === 'show_panel') {
        const binding = holdBinding || pressBinding;
        if (!binding || binding.action.type !== 'show_panel') return;
        const panelId = binding.action.panel_id;
        const defaultGroup = binding.action.default_group;
        const keepOpen = binding.action.keep_open;
        if (panelActivation === 'hold') {
            heldButton = button;
            bindings.showPanel(panelId, defaultGroup, keepOpen);
        } else {
            bindings.togglePanel(panelId, defaultGroup, keepOpen);
        }
        return;
    }

    if (pressBinding) {
        executeDigitalAction(pressBinding.action);
    }
}

function onVirtualButtonUp(button: DigitalInput): void {
    virtualPressed.value = { ...virtualPressed.value, [button]: false };

    if (heldButton === button) {
        const profile = bindings.activeProfile.value;
        const panelActivation = profile?.settings.panelActivation || 'press';
        if (panelActivation === 'hold') bindings.hidePanel();
        heldButton = null;
    }
}

function handleHardwareButton(event: ButtonEvent): void {
    const digitalInput = buttonMap[event.button];
    if (!digitalInput) return;

    // X is the Steam OSK trigger in Game Mode — when a text field is
    // focused, let Steam handle it instead of activating our binding.
    if (digitalInput === 'x' && keyboard.isTextFieldFocused()) return;

    if (event.pressed) onVirtualButtonDown(digitalInput);
    else onVirtualButtonUp(digitalInput);
}

// ─── Lifecycle ───────────────────────────────────────────────────────

let unsubButtons: (() => void) | null = null;

onMounted(() => {
    unsubButtons = input.onButtonEvent(handleHardwareButton);
});

onBeforeUnmount(() => {
    unsubButtons?.();
});
</script>

<template>
    <div class="h-screen flex flex-col">
        <!-- Header. When a preset panel is open the bar takes the panel's
             color and hosts the panel chrome (icon/name/group/page);
             otherwise it shows the app nav. -->
        <header class="border-b border-saint-surface-light px-4 py-3 flex items-center justify-between gap-4"
                :class="presetPanelActive ? 'text-white' : 'bg-saint-surface'"
                :style="presetPanelActive && headerPanel ? { background: headerPanel.color } : undefined">

            <!-- Left: nav (idle) or Back + preset identity (panel open) -->
            <div class="flex items-center gap-3 min-w-0">
                <template v-if="!presetPanelActive">
                    <nav class="flex gap-2">
                        <RouterLink to="/dashboard"
                                    class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                    active-class="bg-saint-primary">Dashboard</RouterLink>
                        <RouterLink to="/controller"
                                    class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                    active-class="bg-saint-primary">Controller</RouterLink>
                        <RouterLink to="/bindings"
                                    class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                    active-class="bg-saint-primary">Bindings</RouterLink>
                        <RouterLink to="/settings"
                                    class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                    active-class="bg-saint-primary">Settings</RouterLink>
                    </nav>
                </template>
                <template v-else>
                    <button class="px-3 py-1.5 rounded-lg text-sm hover:bg-black/15 transition-colors flex items-center gap-2 shrink-0"
                            @click="bindings.hidePanel">
                        <span class="material-icons text-sm">arrow_back</span>
                        Back
                    </button>
                    <span v-if="headerPanel" class="material-symbols-outlined text-2xl shrink-0">{{ headerPanel.icon }}</span>
                    <h1 v-if="headerPanel" class="text-lg font-semibold truncate">{{ headerPanel.name }}</h1>
                </template>
            </div>

            <!-- Center: group filter (panel open only). Page count lives
                 in the panel's footer nav. -->
            <div v-if="presetPanelActive && headerHasGroups" class="flex items-center gap-3 shrink-0">
                <SaintSelect v-model="headerSelectedGroup" :options="headerGroupOptions" />
            </div>

            <!-- Right: sync spinner, connection icon+dropdown, E-Stop -->
            <div class="flex items-center gap-4 shrink-0">
                <span v-if="library.syncing.value"
                      class="material-icons animate-spin text-lg leading-none opacity-70"
                      title="Syncing library…">sync</span>

                <!-- Connection: icon only; click for ping + reconnect. -->
                <div class="relative">
                    <button class="flex items-center p-1 rounded-lg hover:bg-black/15 transition-colors"
                            :title="getStatusText()"
                            @click="connMenuOpen = !connMenuOpen">
                        <span class="material-icons text-xl leading-none" :class="connDotClass()">
                            fiber_manual_record
                        </span>
                    </button>
                    <template v-if="connMenuOpen">
                        <!-- click-away backdrop -->
                        <div class="fixed inset-0 z-40" @click="connMenuOpen = false"></div>
                        <div class="absolute right-0 mt-2 w-52 z-50 bg-saint-surface text-saint-text border border-saint-surface-light rounded-lg shadow-lg p-3 space-y-2">
                            <div class="flex items-center justify-between text-sm">
                                <span class="text-saint-text-muted">Status</span>
                                <span :class="connDotClass()">{{ getStatusText() }}</span>
                            </div>
                            <div class="flex items-center justify-between text-sm">
                                <span class="text-saint-text-muted">Ping</span>
                                <span class="font-mono">{{ conn.pingMs.value === null ? '—' : conn.pingMs.value + ' ms' }}</span>
                            </div>
                            <button class="w-full mt-1 px-3 py-1.5 rounded-lg text-sm bg-saint-primary hover:bg-blue-600 text-white flex items-center justify-center gap-2 transition-colors"
                                    @click="onReconnect">
                                <span class="material-icons text-sm">refresh</span> Reconnect
                            </button>
                        </div>
                    </template>
                </div>

                <!-- E-Stop -->
                <button :class="conn.estopActive.value
                            ? 'flex items-center gap-2 px-4 py-2 bg-amber-500 hover:bg-amber-600 text-black font-semibold rounded-lg transition-colors uppercase tracking-wide ring-2 ring-amber-300'
                            : 'flex items-center gap-2 px-4 py-2 bg-red-600 hover:bg-red-700 text-white font-semibold rounded-lg transition-colors uppercase tracking-wide'"
                        :title="conn.estopActive.value ? 'E-Stop is ENGAGED — press to release' : 'Emergency Stop'"
                        @click="emergencyStop">
                    <span class="material-icons icon-sm">
                        {{ conn.estopActive.value ? 'check_circle' : 'front_hand' }}
                    </span>
                    <span>{{ conn.estopActive.value ? 'Release' : 'E-Stop' }}</span>
                </button>
            </div>
        </header>

        <!-- System-wide E-Stop banner. The Rust client and the
             server's routing evaluator both block control inputs
             while engaged — this is visual confirmation. -->
        <div v-if="conn.estopActive.value"
             class="bg-amber-500 text-black px-4 py-2 text-center text-sm font-semibold flex items-center justify-center gap-2">
            <span class="material-icons icon-sm">warning</span>
            EMERGENCY STOP ENGAGED — control inputs are suppressed system-wide.
            Press the E-Stop button to release.
        </div>

        <!-- Main: router-outlet or preset-panel overlay -->
        <main class="flex-1 overflow-hidden">
            <PresetPanel v-if="presetPanelActive" />
            <div v-else class="h-full overflow-auto touch-scroll">
                <RouterView />
            </div>
        </main>


        <!-- Virtual Keyboard overlay -->
        <VirtualKeyboard />
    </div>
</template>
