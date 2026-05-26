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

interface BatteryCell {
    voltage: number;
    temperature: number;
    health: number;
}

interface BMSInfo {
    manufacturer: string;
    model: string;
    serialNumber: string;
    firmwareVersion: string;
    cycleCount: number;
    maxCapacity: number;
    currentCapacity: number;
}

interface TrackBattery {
    name: string;
    cells: BatteryCell[];
    chargePercent: number;
    isCharging: boolean;
    voltage: number;
    current: number;
    bms: BMSInfo;
}

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

// Mock battery data — unchanged from the Angular shell. A real battery
// feed would replace this; the structure is what the BMS UI renders.
const batteries = ref<TrackBattery[]>([
    {
        name: 'Left Track',
        cells: [
            { voltage: 3.92, temperature: 28, health: 98 },
            { voltage: 3.89, temperature: 29, health: 97 },
            { voltage: 3.91, temperature: 28, health: 99 },
            { voltage: 3.88, temperature: 30, health: 96 },
        ],
        chargePercent: 78, isCharging: false, voltage: 15.6, current: -1.2,
        bms: {
            manufacturer: 'SAINT Power', model: 'BMS-4S-30A',
            serialNumber: 'SP-L-2024-00142', firmwareVersion: '1.2.4',
            cycleCount: 127, maxCapacity: 5000, currentCapacity: 4850,
        },
    },
    {
        name: 'Right Track',
        cells: [
            { voltage: 3.95, temperature: 27, health: 99 },
            { voltage: 3.93, temperature: 28, health: 98 },
            { voltage: 3.94, temperature: 27, health: 99 },
            { voltage: 3.91, temperature: 29, health: 97 },
        ],
        chargePercent: 82, isCharging: false, voltage: 15.73, current: -1.1,
        bms: {
            manufacturer: 'SAINT Power', model: 'BMS-4S-30A',
            serialNumber: 'SP-R-2024-00143', firmwareVersion: '1.2.4',
            cycleCount: 124, maxCapacity: 5000, currentCapacity: 4900,
        },
    },
]);

const overallBatteryPercent = computed(() => {
    const bs = batteries.value;
    if (bs.length === 0) return 0;
    return Math.round(bs.reduce((sum, b) => sum + b.chargePercent, 0) / bs.length);
});

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

                <!-- Battery Panel -->
                <div v-if="activePanel === 'battery'" class="p-3 h-full overflow-auto">
                    <div class="flex flex-wrap gap-3 h-full">
                        <div v-for="battery in batteries" :key="battery.name"
                             class="bg-saint-surface-light rounded-lg p-3 flex-1 min-w-[280px] flex flex-col">
                            <div class="flex items-center justify-between mb-2">
                                <div class="flex items-center gap-2">
                                    <span class="material-icons text-2xl"
                                          :class="{
                                              'text-saint-success': battery.chargePercent >= 50,
                                              'text-yellow-500': battery.chargePercent >= 20 && battery.chargePercent < 50,
                                              'text-red-500': battery.chargePercent < 20,
                                          }">
                                        {{
                                            battery.isCharging ? 'battery_charging_full' :
                                            battery.chargePercent >= 80 ? 'battery_full' :
                                            battery.chargePercent >= 50 ? 'battery_4_bar' :
                                            battery.chargePercent >= 20 ? 'battery_2_bar' :
                                            'battery_1_bar'
                                        }}
                                    </span>
                                    <div>
                                        <div class="font-medium">{{ battery.name }}</div>
                                        <div class="flex gap-3 text-xs text-saint-text-muted">
                                            <span>{{ battery.voltage.toFixed(2) }}V</span>
                                            <span>{{ battery.current.toFixed(1) }}A</span>
                                            <span v-if="battery.isCharging" class="text-saint-success">Charging</span>
                                        </div>
                                    </div>
                                </div>
                                <span class="text-2xl font-bold"
                                      :class="{
                                          'text-saint-success': battery.chargePercent >= 50,
                                          'text-yellow-500': battery.chargePercent >= 20 && battery.chargePercent < 50,
                                          'text-red-500': battery.chargePercent < 20,
                                      }">
                                    {{ battery.chargePercent }}%
                                </span>
                            </div>

                            <div class="flex gap-4 flex-1">
                                <div class="flex-1 min-w-0 overflow-hidden">
                                    <div class="text-xs text-saint-text-muted mb-1">Cell Voltages</div>
                                    <div class="grid grid-cols-2 sm:grid-cols-4 gap-1.5">
                                        <div v-for="(cell, i) in battery.cells" :key="i"
                                             class="bg-saint-surface rounded p-1.5 text-center min-w-0">
                                            <div class="text-[10px] text-saint-text-muted truncate">C{{ i + 1 }}</div>
                                            <div class="text-xs sm:text-sm font-medium truncate"
                                                 :class="{
                                                     'text-saint-success': cell.voltage >= 3.7,
                                                     'text-yellow-500': cell.voltage >= 3.4 && cell.voltage < 3.7,
                                                     'text-red-500': cell.voltage < 3.4,
                                                 }">
                                                {{ cell.voltage.toFixed(2) }}V
                                            </div>
                                            <div class="text-[10px] text-saint-text-muted truncate">{{ cell.temperature }}°</div>
                                        </div>
                                    </div>
                                </div>

                                <div class="border-l border-saint-surface pl-3 min-w-0 flex-shrink-0 overflow-hidden">
                                    <div class="text-xs text-saint-text-muted mb-1">BMS Info</div>
                                    <div class="grid grid-cols-[auto_1fr] gap-x-2 gap-y-0.5 text-xs">
                                        <span class="text-saint-text-muted">Model</span>
                                        <span class="truncate">{{ battery.bms.model }}</span>
                                        <span class="text-saint-text-muted">Serial</span>
                                        <span class="font-mono text-[10px] truncate">{{ battery.bms.serialNumber }}</span>
                                        <span class="text-saint-text-muted">FW</span>
                                        <span class="truncate">v{{ battery.bms.firmwareVersion }}</span>
                                        <span class="text-saint-text-muted">Cycles</span>
                                        <span>{{ battery.bms.cycleCount }}</span>
                                        <span class="text-saint-text-muted">Cap</span>
                                        <span class="truncate">{{ battery.bms.currentCapacity }}/{{ battery.bms.maxCapacity }}</span>
                                    </div>
                                </div>
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
