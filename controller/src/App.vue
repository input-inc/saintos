<!--
    Root shell. Header (nav + connection status + E-Stop button +
    e-stop banner), main (router-outlet OR preset-panel overlay),
    footer (Controls / Battery accordion tabs), and the virtual
    keyboard overlay component.

    Translates from controller/src/app/app.component.ts. The
    differential-drive / direct-control sending paths still need
    wiring through useConnection.sendWsInputValue — for the
    migration window those TODO console.logs match what the Angular
    version emits.
-->
<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from 'vue';
import { RouterLink, RouterView } from 'vue-router';
import VirtualJoystick, { type JoystickPosition } from './components/VirtualJoystick.vue';
import PresetPanel from './components/PresetPanel.vue';
import VirtualKeyboard from './components/VirtualKeyboard.vue';
import { useConnection, ConnectionStatus } from './composables/useConnection';
import { useInput, type ButtonEvent } from './composables/useInput';
import { useBindings, type DigitalInput, type DigitalAction } from './composables/useBindings';
import { useKeyboard } from './composables/useKeyboard';
import { useBatteries } from './composables/useBatteries';
import { useWifi, type SurveyChannel } from './composables/useWifi';

const conn = useConnection();
const input = useInput();
const bindings = useBindings();
const keyboard = useKeyboard();

// ─── Reactive state ──────────────────────────────────────────────────

const activePanel = ref<string | null>(null);
const leftStick = ref<JoystickPosition>({ x: 0, y: 0 });
const rightStick = ref<JoystickPosition>({ x: 0, y: 0 });
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

// Live BMS packs, sniffed from each adopted node's pin_state topic.
// Same field set as the dashboard's BMSMonitor widget.
const { batteries } = useBatteries();

// WiFi link health + channel management (port of the dashboard's WiFi
// card + channel-survey modal — see useWifi).
const {
    config: wifiConfig,
    live: wifiLive,
    survey: wifiSurvey,
    switching: wifiSwitching,
    bestPick: wifiBestPick,
    runSurvey: runWifiSurvey,
    applyChannel: applyWifiChannel,
} = useWifi();

// Panel-local survey selection + two-step apply confirm. A row click
// selects; the first Apply press arms the confirm, the second fires.
const wifiSelected = ref<SurveyChannel | null>(null);
const wifiApplyArmed = ref(false);

function selectWifiChannel(ch: SurveyChannel): void {
    wifiSelected.value = ch;
    wifiApplyArmed.value = false;
}

function isWifiSelected(ch: SurveyChannel): boolean {
    const s = wifiSelected.value;
    return !!s && s.band === ch.band && s.channel === ch.channel;
}

async function onWifiApply(): Promise<void> {
    const ch = wifiSelected.value;
    if (!ch || ch.is_current) return;
    if (!wifiApplyArmed.value) {
        wifiApplyArmed.value = true;
        return;
    }
    wifiApplyArmed.value = false;
    wifiSelected.value = null;
    try { await applyWifiChannel(ch); }
    catch (err) { console.error('WiFi channel switch failed:', err); }
}

function wifiBandLabel(band: string | null): string {
    if (band === 'a') return '5 GHz';
    if (band === 'bg') return '2.4 GHz';
    return '—';
}

// Signal quality thresholds (dBm): ≥ -55 solid, ≥ -70 usable, below
// that expect deadstick-grade latency.
function wifiSignalTextClass(dbm: number | null): string {
    if (dbm === null) return 'text-saint-text-muted';
    if (dbm >= -55) return 'text-saint-success';
    if (dbm >= -70) return 'text-yellow-500';
    return 'text-red-500';
}
function wifiSignalBarClass(dbm: number | null): string {
    if (dbm === null) return 'bg-saint-surface';
    if (dbm >= -55) return 'bg-saint-success';
    if (dbm >= -70) return 'bg-yellow-500';
    return 'bg-red-500';
}
// Map -90…-30 dBm onto 0…100 % for the bar.
function wifiSignalPct(dbm: number | null): number {
    if (dbm === null) return 0;
    return Math.max(0, Math.min(100, ((dbm + 90) / 60) * 100));
}
function getWifiStatusClass(): string {
    const dbm = wifiLive.value.signalDbm;
    if (dbm === null) return 'bg-saint-text-muted/20 text-saint-text-muted';
    if (dbm >= -55) return 'bg-saint-success/20 text-saint-success';
    if (dbm >= -70) return 'bg-yellow-500/20 text-yellow-500';
    return 'bg-red-500/20 text-red-500';
}
// Same congestion color bands as the dashboard's survey modal.
function apCountClass(count: number): string {
    if (count === 0) return 'bg-saint-success/20 text-saint-success';
    if (count <= 2) return 'bg-saint-surface text-saint-text-muted';
    if (count <= 5) return 'bg-yellow-500/20 text-yellow-500';
    return 'bg-red-500/20 text-red-500';
}

// Headline = average SOC across packs that are actually reporting.
const overallBatteryPercent = computed(() => {
    const withSoc = batteries.value.filter(b => typeof b.soc === 'number');
    if (withSoc.length === 0) return 0;
    return Math.round(withSoc.reduce((sum, b) => sum + (b.soc as number), 0) / withSoc.length);
});

// Temp shown in °C with one decimal; '—' when the pack isn't reporting.
function fmt(v: number | null, places = 2, unit = ''): string {
    if (v === null || v === undefined || !isFinite(v)) return '—';
    return `${v.toFixed(places)}${unit ? ' ' + unit : ''}`;
}
function socClass(soc: number | null): string {
    if (soc === null) return 'text-saint-text-muted';
    if (soc >= 50) return 'text-saint-success';
    if (soc >= 20) return 'text-yellow-500';
    return 'text-red-500';
}

const gamepadConnected = computed(() => input.isGamepadConnected.value);
const gamepadName       = computed(() => input.gamepad.value.name);
const physicalLeftStick = computed(() => input.gamepad.value.leftStick);
const physicalRightStick= computed(() => input.gamepad.value.rightStick);
const gamepadButtons    = computed(() => input.gamepad.value.buttons);
const leftTrigger       = computed(() => input.gamepad.value.leftTrigger);
const rightTrigger      = computed(() => input.gamepad.value.rightTrigger);

const presetPanelActive = computed(() =>
    bindings.activePanelState.value.activePanelId !== null);

// ─── Status display ──────────────────────────────────────────────────

function getStatusClass(): string {
    const base = 'status-indicator';
    switch (conn.status.value) {
        case ConnectionStatus.Connected:  return `${base} status-connected`;
        case ConnectionStatus.Connecting: return `${base} status-connecting`;
        default:                          return `${base} status-disconnected`;
    }
}

function getStatusText(): string {
    switch (conn.status.value) {
        case ConnectionStatus.Connected:     return 'Connected';
        case ConnectionStatus.Connecting:    return 'Connecting...';
        case ConnectionStatus.Authenticating: return 'Authenticating...';
        default:                             return 'Disconnected';
    }
}

function getTabClass(panel: string): string {
    const base = 'border-r border-saint-surface-light';
    if (activePanel.value === panel) {
        return `${base} bg-saint-primary text-white`;
    }
    return `${base} hover:bg-saint-surface-light text-saint-text-muted`;
}

function getBatteryStatusClass(): string {
    const percent = overallBatteryPercent.value;
    if (percent >= 50) return 'bg-green-500/20 text-green-500';
    if (percent >= 20) return 'bg-yellow-500/20 text-yellow-500';
    return 'bg-red-500/20 text-red-500';
}

function togglePanel(panel: string): void {
    activePanel.value = activePanel.value === panel ? null : panel;
}

async function emergencyStop(): Promise<void> {
    try { await conn.emergencyStop(); }
    catch (err) { console.error('Emergency stop failed:', err); }
}

// ─── Binding-action execution ────────────────────────────────────────

function executeDigitalAction(action: DigitalAction): void {
    switch (action.type) {
        case 'show_panel':       bindings.showPanel(action.panel_id); break;
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
        if (panelActivation === 'hold') {
            heldButton = button;
            bindings.showPanel(panelId);
        } else {
            bindings.togglePanel(panelId);
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

function onVirtualButtonPress(button: DigitalInput): void {
    onVirtualButtonDown(button);
    setTimeout(() => onVirtualButtonUp(button), 150);
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

function onLeftStickChange(position: JoystickPosition): void {
    leftStick.value = position;
    // TODO: route through useConnection.sendWsInputValue once the
    // joystick → ws_input plumbing is ported. For now the Rust side
    // still receives gamepad state via the input-state event path,
    // which the routing layer consumes.
}

function onRightStickChange(position: JoystickPosition): void {
    rightStick.value = position;
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
        <!-- Header -->
        <header class="bg-saint-surface border-b border-saint-surface-light px-4 py-3 flex items-center justify-between">
            <nav class="flex gap-2">
                <template v-if="!presetPanelActive">
                    <RouterLink to="/controller"
                                class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                active-class="bg-saint-primary">
                        Controller
                    </RouterLink>
                    <RouterLink to="/bindings"
                                class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                active-class="bg-saint-primary">
                        Bindings
                    </RouterLink>
                    <RouterLink to="/settings"
                                class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors"
                                active-class="bg-saint-primary">
                        Settings
                    </RouterLink>
                </template>
                <template v-else>
                    <button class="px-3 py-1.5 rounded-lg text-sm hover:bg-saint-surface-light transition-colors flex items-center gap-2"
                            @click="bindings.hidePanel">
                        <span class="material-icons text-sm">arrow_back</span>
                        Back
                    </button>
                </template>
            </nav>

            <div class="flex items-center gap-4">
                <!-- Connection Status -->
                <div class="flex items-center gap-2">
                    <div :class="getStatusClass()"></div>
                    <span class="text-sm text-saint-text-muted">{{ getStatusText() }}</span>
                </div>

                <!-- E-Stop Button — color + label tracks the latched
                     server state so the operator can tell engaged
                     from released without guessing. -->
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

        <!-- Footer Navigation Bar -->
        <footer class="bg-saint-surface border-t border-saint-surface-light">
            <!-- Tab Buttons -->
            <div class="flex border-b border-saint-surface-light">
                <button :class="getTabClass('controls')"
                        class="flex items-center gap-2 px-4 py-2 text-sm transition-colors"
                        @click="togglePanel('controls')">
                    <span class="material-icons icon-sm">sports_esports</span>
                    <span>Controls</span>
                    <span v-if="gamepadConnected"
                          class="px-2 py-0.5 bg-saint-success/20 text-saint-success text-xs rounded-full">
                        {{ gamepadName }}
                    </span>
                    <span v-else
                          class="px-2 py-0.5 bg-saint-text-muted/20 text-saint-text-muted text-xs rounded-full">
                        No Controller
                    </span>
                    <span class="material-icons icon-sm transition-transform"
                          :class="{ 'rotate-180': activePanel === 'controls' }">
                        expand_less
                    </span>
                </button>
                <button :class="getTabClass('battery')"
                        class="flex items-center gap-2 px-4 py-2 text-sm transition-colors"
                        @click="togglePanel('battery')">
                    <span class="material-icons icon-sm">battery_4_bar</span>
                    <span>Battery</span>
                    <span class="px-2 py-0.5 text-xs rounded-full" :class="getBatteryStatusClass()">
                        {{ overallBatteryPercent }}%
                    </span>
                    <span class="material-icons icon-sm transition-transform"
                          :class="{ 'rotate-180': activePanel === 'battery' }">
                        expand_less
                    </span>
                </button>
                <button :class="getTabClass('wifi')"
                        class="flex items-center gap-2 px-4 py-2 text-sm transition-colors"
                        @click="togglePanel('wifi')">
                    <span class="material-icons icon-sm">wifi</span>
                    <span>WiFi</span>
                    <span class="px-2 py-0.5 text-xs rounded-full font-mono" :class="getWifiStatusClass()">
                        {{ wifiLive.signalDbm === null ? '—' : Math.round(wifiLive.signalDbm) + ' dBm' }}
                    </span>
                    <span class="material-icons icon-sm transition-transform"
                          :class="{ 'rotate-180': activePanel === 'wifi' }">
                        expand_less
                    </span>
                </button>
            </div>

            <!-- Sliding Panel -->
            <div class="overflow-hidden transition-all duration-300 ease-in-out"
                 :style="{ height: activePanel ? '280px' : '0' }">

                <!-- Controls Panel -->
                <div v-if="activePanel === 'controls'" class="p-4 h-full">
                    <div class="flex justify-center items-center h-full">
                        <div class="relative w-full max-w-2xl h-56">

                            <!-- Left Side -->
                            <div class="absolute left-0 top-0 bottom-0 w-1/2 flex flex-col justify-between py-2">
                                <!-- Left Shoulder/Trigger -->
                                <div class="flex flex-col gap-1 ml-8">
                                    <button class="w-20 h-6 rounded-t-lg text-xs font-medium transition-colors relative overflow-hidden"
                                            :class="leftTrigger < 0.1 ? 'bg-saint-surface-light' : 'bg-saint-primary'">
                                        <div class="absolute inset-0 bg-saint-primary transition-all"
                                             :style="{ width: `${leftTrigger * 100}%` }"></div>
                                        <span class="relative z-10">LT</span>
                                    </button>
                                    <button class="w-20 h-5 rounded-b text-xs font-medium transition-colors active:scale-95"
                                            :class="(gamepadButtons.LB || virtualPressed.lb)
                                                ? 'bg-saint-primary' : 'bg-saint-surface-light'"
                                            @click="onVirtualButtonPress('lb')">
                                        LB
                                    </button>
                                </div>

                                <!-- Left Stick + D-Pad -->
                                <div class="flex items-center gap-6 ml-4">
                                    <VirtualJoystick :size="80" :knob-size="40"
                                                     :external-position="physicalLeftStick"
                                                     @position-change="onLeftStickChange" />

                                    <div class="relative w-20 h-20">
                                        <button class="absolute top-0 left-1/2 -translate-x-1/2 w-6 h-6 rounded-t transition-colors active:scale-95"
                                                :class="(gamepadButtons.DPadUp || virtualPressed.d_pad_up)
                                                    ? 'bg-saint-primary' : 'bg-saint-surface-light'"
                                                @click="onVirtualButtonPress('d_pad_up')">
                                            <span class="material-icons text-sm">keyboard_arrow_up</span>
                                        </button>
                                        <button class="absolute bottom-0 left-1/2 -translate-x-1/2 w-6 h-6 rounded-b transition-colors active:scale-95"
                                                :class="(gamepadButtons.DPadDown || virtualPressed.d_pad_down)
                                                    ? 'bg-saint-primary' : 'bg-saint-surface-light'"
                                                @click="onVirtualButtonPress('d_pad_down')">
                                            <span class="material-icons text-sm">keyboard_arrow_down</span>
                                        </button>
                                        <button class="absolute left-0 top-1/2 -translate-y-1/2 w-6 h-6 rounded-l transition-colors active:scale-95"
                                                :class="(gamepadButtons.DPadLeft || virtualPressed.d_pad_left)
                                                    ? 'bg-saint-primary' : 'bg-saint-surface-light'"
                                                @click="onVirtualButtonPress('d_pad_left')">
                                            <span class="material-icons text-sm">keyboard_arrow_left</span>
                                        </button>
                                        <button class="absolute right-0 top-1/2 -translate-y-1/2 w-6 h-6 rounded-r transition-colors active:scale-95"
                                                :class="(gamepadButtons.DPadRight || virtualPressed.d_pad_right)
                                                    ? 'bg-saint-primary' : 'bg-saint-surface-light'"
                                                @click="onVirtualButtonPress('d_pad_right')">
                                            <span class="material-icons text-sm">keyboard_arrow_right</span>
                                        </button>
                                        <div class="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-6 h-6 bg-saint-surface-light"></div>
                                    </div>
                                </div>

                                <div class="h-6"></div>
                            </div>

                            <!-- Right Side -->
                            <div class="absolute right-0 top-0 bottom-0 w-1/2 flex flex-col justify-between py-2">
                                <!-- Right Shoulder/Trigger -->
                                <div class="flex flex-col gap-1 ml-auto mr-8">
                                    <button class="w-20 h-6 rounded-t-lg text-xs font-medium transition-colors relative overflow-hidden"
                                            :class="rightTrigger < 0.1 ? 'bg-saint-surface-light' : 'bg-saint-primary'">
                                        <div class="absolute inset-0 bg-saint-primary transition-all origin-right"
                                             :style="{ width: `${rightTrigger * 100}%` }"></div>
                                        <span class="relative z-10">RT</span>
                                    </button>
                                    <button class="w-20 h-5 rounded-b text-xs font-medium transition-colors active:scale-95"
                                            :class="(gamepadButtons.RB || virtualPressed.rb)
                                                ? 'bg-saint-primary' : 'bg-saint-surface-light'"
                                            @click="onVirtualButtonPress('rb')">
                                        RB
                                    </button>
                                </div>

                                <div class="flex items-center gap-6 justify-end mr-4">
                                    <!-- Face Buttons (ABXY) -->
                                    <div class="relative w-20 h-20">
                                        <button class="absolute top-0 left-1/2 -translate-x-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                                :class="(gamepadButtons.Y || virtualPressed.y) ? 'bg-yellow-400 scale-95' : 'bg-yellow-600'"
                                                @mousedown="onVirtualButtonDown('y')"
                                                @mouseup="onVirtualButtonUp('y')"
                                                @mouseleave="onVirtualButtonUp('y')"
                                                @touchstart.prevent="onVirtualButtonDown('y')"
                                                @touchend="onVirtualButtonUp('y')">
                                            Y
                                        </button>
                                        <button class="absolute bottom-0 left-1/2 -translate-x-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                                :class="(gamepadButtons.A || virtualPressed.a) ? 'bg-green-400 scale-95' : 'bg-green-600'"
                                                @click="onVirtualButtonPress('a')">
                                            A
                                        </button>
                                        <button class="absolute left-0 top-1/2 -translate-y-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                                :class="(gamepadButtons.X || virtualPressed.x) ? 'bg-blue-400 scale-95' : 'bg-blue-600'"
                                                @mousedown="onVirtualButtonDown('x')"
                                                @mouseup="onVirtualButtonUp('x')"
                                                @mouseleave="onVirtualButtonUp('x')"
                                                @touchstart.prevent="onVirtualButtonDown('x')"
                                                @touchend="onVirtualButtonUp('x')">
                                            X
                                        </button>
                                        <button class="absolute right-0 top-1/2 -translate-y-1/2 w-8 h-8 rounded-full text-xs font-bold transition-colors shadow-md active:scale-90"
                                                :class="(gamepadButtons.B || virtualPressed.b) ? 'bg-red-400 scale-95' : 'bg-red-600'"
                                                @click="onVirtualButtonPress('b')">
                                            B
                                        </button>
                                    </div>

                                    <VirtualJoystick :size="80" :knob-size="40"
                                                     :external-position="physicalRightStick"
                                                     @position-change="onRightStickChange" />
                                </div>

                                <div class="h-6"></div>
                            </div>

                        </div>
                    </div>
                </div>

                <!-- Battery Panel — live BMS packs sniffed from each
                     node's pin_state topic (see useBatteries). Same
                     field set as the dashboard's BMSMonitor widget. -->
                <div v-if="activePanel === 'battery'" class="p-3 h-full overflow-auto">
                    <div v-if="batteries.length === 0"
                         class="h-full flex items-center justify-center text-saint-text-muted text-sm">
                        No batteries reporting.
                    </div>
                    <div v-else class="flex flex-wrap gap-3 h-full">
                        <div v-for="battery in batteries" :key="battery.key"
                             class="bg-saint-surface-light rounded-lg p-3 flex-1 min-w-[280px] flex flex-col">
                            <!-- Header: icon + name + SOC % -->
                            <div class="flex items-center justify-between mb-3">
                                <div class="flex items-center gap-2 min-w-0">
                                    <span class="material-icons text-2xl" :class="socClass(battery.soc)">
                                        {{
                                            battery.chargeOn ? 'battery_charging_full' :
                                            battery.soc === null ? 'battery_unknown' :
                                            battery.soc >= 80 ? 'battery_full' :
                                            battery.soc >= 50 ? 'battery_4_bar' :
                                            battery.soc >= 20 ? 'battery_2_bar' :
                                            'battery_1_bar'
                                        }}
                                    </span>
                                    <div class="font-medium truncate">{{ battery.name }}</div>
                                </div>
                                <span class="text-2xl font-bold" :class="socClass(battery.soc)">
                                    {{ battery.soc === null ? '—' : Math.round(battery.soc) + '%' }}
                                </span>
                            </div>

                            <!-- SOC bar -->
                            <div class="h-2 w-full rounded-full bg-saint-surface overflow-hidden mb-3">
                                <div class="h-2 transition-all"
                                     :class="battery.soc === null ? 'bg-saint-surface'
                                             : battery.soc >= 50 ? 'bg-saint-success'
                                             : battery.soc >= 20 ? 'bg-yellow-500' : 'bg-red-500'"
                                     :style="{ width: `${Math.max(0, Math.min(100, battery.soc ?? 0))}%` }" />
                            </div>

                            <!-- Pack metrics: voltage / current / temp -->
                            <div class="grid grid-cols-3 gap-2 text-center mb-3">
                                <div class="bg-saint-surface rounded p-2">
                                    <div class="text-[10px] uppercase text-saint-text-muted">Voltage</div>
                                    <div class="font-mono">{{ fmt(battery.voltage, 2, 'V') }}</div>
                                </div>
                                <div class="bg-saint-surface rounded p-2">
                                    <div class="text-[10px] uppercase text-saint-text-muted">Current</div>
                                    <div class="font-mono"
                                         :class="(battery.current ?? 0) >= 0 ? 'text-saint-success' : 'text-yellow-500'">
                                        {{ fmt(battery.current, 2, 'A') }}
                                    </div>
                                </div>
                                <div class="bg-saint-surface rounded p-2">
                                    <div class="text-[10px] uppercase text-saint-text-muted">Temp</div>
                                    <div class="font-mono">{{ battery.temp === null ? '—' : fmt(battery.temp, 1, '°C') }}</div>
                                </div>
                            </div>

                            <!-- FET state chips -->
                            <div class="flex items-center gap-2 text-xs mb-2">
                                <span class="px-2 py-0.5 rounded-full"
                                      :class="battery.chargeOn ? 'bg-green-500/20 text-green-500' : 'bg-saint-surface text-saint-text-muted'">
                                    CHG {{ battery.chargeOn ? 'ON' : 'OFF' }}
                                </span>
                                <span class="px-2 py-0.5 rounded-full"
                                      :class="battery.dischargeOn ? 'bg-green-500/20 text-green-500' : 'bg-saint-surface text-saint-text-muted'">
                                    DSG {{ battery.dischargeOn ? 'ON' : 'OFF' }}
                                </span>
                            </div>

                            <!-- Fault panel — only when something is asserted -->
                            <div v-if="battery.faults.length"
                                 class="rounded border border-red-500/40 bg-red-500/10 px-2 py-1.5 text-xs mt-auto">
                                <div class="flex items-center gap-1 text-red-400 font-medium mb-0.5">
                                    <span class="material-icons text-base">warning</span>
                                    BMS faults asserted
                                </div>
                                <ul class="list-disc list-inside text-red-300 ml-1">
                                    <li v-for="f in battery.faults" :key="f">{{ f }}</li>
                                </ul>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- WiFi Panel — link health to the server AP plus the
                     channel-survey / channel-change flow ported from
                     the dashboard's WifiChannelModal (see useWifi). -->
                <div v-if="activePanel === 'wifi'" class="p-3 h-full overflow-hidden">

                    <!-- AP restarting: the switch was commanded and the
                         link is expected to drop for ~5-10 s. -->
                    <div v-if="wifiSwitching.active"
                         class="h-full flex flex-col items-center justify-center gap-2 text-saint-text-muted">
                        <span class="material-icons text-4xl animate-spin">autorenew</span>
                        <div class="font-medium text-saint-text">Restarting WiFi AP…</div>
                        <div v-if="wifiSwitching.detail" class="text-sm">
                            Switching to {{ wifiSwitching.detail }} — reconnecting automatically.
                        </div>
                    </div>

                    <div v-else class="h-full flex gap-3">
                        <!-- Link status card -->
                        <div class="w-64 shrink-0 bg-saint-surface-light rounded-lg p-3 flex flex-col">
                            <div class="flex items-center justify-between mb-1">
                                <div class="font-medium truncate">{{ wifiConfig.ssid ?? 'No AP info' }}</div>
                                <span class="px-2 py-0.5 text-xs rounded-full bg-saint-surface text-saint-text-muted font-mono">
                                    {{ wifiBandLabel(wifiConfig.band) }} · ch {{ wifiConfig.channel ?? '—' }}
                                </span>
                            </div>
                            <div class="text-3xl font-bold font-mono mb-1"
                                 :class="wifiSignalTextClass(wifiLive.signalDbm)">
                                {{ wifiLive.signalDbm === null ? '—' : Math.round(wifiLive.signalDbm) + ' dBm' }}
                            </div>
                            <div class="h-2 w-full rounded-full bg-saint-surface overflow-hidden mb-3">
                                <div class="h-2 transition-all"
                                     :class="wifiSignalBarClass(wifiLive.signalDbm)"
                                     :style="{ width: `${wifiSignalPct(wifiLive.signalDbm)}%` }" />
                            </div>
                            <div class="grid grid-cols-3 gap-2 text-center">
                                <div class="bg-saint-surface rounded p-2">
                                    <div class="text-[10px] uppercase text-saint-text-muted">Retries</div>
                                    <div class="font-mono"
                                         :class="(wifiLive.retryPct ?? 0) > 10 ? 'text-yellow-500' : ''">
                                        {{ wifiLive.retryPct === null ? '—' : wifiLive.retryPct.toFixed(1) + '%' }}
                                    </div>
                                </div>
                                <div class="bg-saint-surface rounded p-2">
                                    <div class="text-[10px] uppercase text-saint-text-muted">Noise</div>
                                    <div class="font-mono">
                                        {{ wifiLive.noiseDbm === null ? '—' : Math.round(wifiLive.noiseDbm) + ' dBm' }}
                                    </div>
                                </div>
                                <div class="bg-saint-surface rounded p-2">
                                    <div class="text-[10px] uppercase text-saint-text-muted">Rate</div>
                                    <div class="font-mono">
                                        {{ wifiLive.bitrateMbps === null ? '—' : Math.round(wifiLive.bitrateMbps) + ' Mb/s' }}
                                    </div>
                                </div>
                            </div>
                            <div class="mt-auto text-[10px] text-saint-text-muted">
                                Signal is your Deck's link as the AP sees it, updated 1 Hz.
                            </div>
                        </div>

                        <!-- Channel survey + switch -->
                        <div class="flex-1 min-w-0 bg-saint-surface-light rounded-lg p-3 flex flex-col">
                            <div class="flex items-center justify-between mb-2">
                                <div class="font-medium">Channel survey</div>
                                <button class="px-3 py-1 rounded-lg text-xs font-medium transition-colors flex items-center gap-1"
                                        :class="wifiSurvey.loading
                                            ? 'bg-saint-surface text-saint-text-muted'
                                            : 'bg-saint-primary hover:bg-blue-600 text-white'"
                                        :disabled="wifiSurvey.loading"
                                        @click="runWifiSurvey">
                                    <span class="material-icons text-sm"
                                          :class="{ 'animate-spin': wifiSurvey.loading }">
                                        {{ wifiSurvey.loading ? 'autorenew' : 'radar' }}
                                    </span>
                                    {{ wifiSurvey.loading ? 'Scanning… ~10 s' : 'Scan' }}
                                </button>
                            </div>

                            <div v-if="wifiSurvey.error"
                                 class="text-xs text-red-400 mb-2">{{ wifiSurvey.error }}</div>

                            <div v-if="!wifiSurvey.channels.length && !wifiSurvey.loading"
                                 class="flex-1 flex items-center justify-center text-saint-text-muted text-sm text-center px-4">
                                Scan to see nearby AP congestion per channel and move the
                                robot's AP somewhere quieter.
                            </div>

                            <div v-else class="flex-1 overflow-auto touch-scroll">
                                <button v-for="ch in wifiSurvey.channels"
                                        :key="`${ch.band}-${ch.channel}`"
                                        class="w-full flex items-center gap-2 px-2 py-1.5 rounded text-xs text-left transition-colors"
                                        :class="[
                                            isWifiSelected(ch) ? 'ring-1 ring-saint-primary bg-saint-primary/10'
                                                               : 'hover:bg-saint-surface',
                                            ch.is_current ? 'opacity-70' : '',
                                        ]"
                                        @click="selectWifiChannel(ch)">
                                    <span class="font-mono w-20 shrink-0">{{ ch.band }} GHz · {{ ch.channel }}</span>
                                    <span class="font-mono w-16 shrink-0 text-saint-text-muted">{{ ch.freq_mhz }} MHz</span>
                                    <span class="px-2 py-0.5 rounded-full font-mono shrink-0" :class="apCountClass(ch.ap_count)">
                                        {{ ch.ap_count }} AP{{ ch.ap_count === 1 ? '' : 's' }}
                                    </span>
                                    <span class="font-mono text-saint-text-muted shrink-0">
                                        {{ ch.strongest_signal_dbm === null ? '' : Math.round(ch.strongest_signal_dbm) + ' dBm' }}
                                    </span>
                                    <span class="ml-auto flex items-center gap-1 shrink-0">
                                        <span v-if="ch.is_current"
                                              class="px-2 py-0.5 rounded-full bg-cyan-500/20 text-cyan-400">Current</span>
                                        <span v-if="wifiBestPick === ch"
                                              class="px-2 py-0.5 rounded-full bg-saint-success/20 text-saint-success">Best</span>
                                        <span v-if="ch.is_dfs"
                                              class="px-2 py-0.5 rounded-full bg-yellow-500/20 text-yellow-500"
                                              title="DFS: 60 s radar-listen before the AP can transmit">DFS</span>
                                    </span>
                                </button>
                            </div>

                            <div v-if="wifiSurvey.channels.length"
                                 class="pt-2 mt-1 border-t border-saint-surface flex items-center gap-2">
                                <div class="text-[11px] text-saint-text-muted flex-1 min-w-0 truncate">
                                    <template v-if="wifiApplyArmed && wifiSelected">
                                        All clients (including this Deck) drop and reconnect over ~5–10 s{{ wifiSelected.is_dfs ? ' — DFS adds a 60 s radar-listen' : '' }}.
                                    </template>
                                    <template v-else-if="wifiBestPick">
                                        Recommended: {{ wifiBestPick.band }} GHz ch {{ wifiBestPick.channel }}
                                        ({{ wifiBestPick.ap_count }} AP{{ wifiBestPick.ap_count === 1 ? '' : 's' }} nearby).
                                    </template>
                                </div>
                                <button class="px-3 py-1.5 rounded-lg text-xs font-semibold transition-colors shrink-0"
                                        :class="!wifiSelected || wifiSelected.is_current
                                            ? 'bg-saint-surface text-saint-text-muted'
                                            : wifiApplyArmed
                                                ? 'bg-amber-500 hover:bg-amber-600 text-black'
                                                : 'bg-saint-primary hover:bg-blue-600 text-white'"
                                        :disabled="!wifiSelected || wifiSelected.is_current"
                                        @click="onWifiApply">
                                    {{ wifiApplyArmed ? 'Confirm — restart AP' : 'Apply channel' }}
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </footer>

        <!-- Virtual Keyboard overlay -->
        <VirtualKeyboard />
    </div>
</template>
