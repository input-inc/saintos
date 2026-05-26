<!--
    Controller view — live gamepad / touchpad / gyro state monitor.
    Pure read-only display driven by useInput; no inputs to the
    server originate here. Connection state is shown in the Command
    Log section so the operator can confirm the bridge is up.
-->
<script setup lang="ts">
import { useInput } from '../composables/useInput';
import { useConnection } from '../composables/useConnection';

const input = useInput();
const conn = useConnection();

const buttonList = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'Select', 'Start', 'L3', 'R3', 'L4', 'R4', 'L5', 'R5', 'Steam', 'QAM'];

function formatAxis(value: number): string { return value.toFixed(2); }
function formatAngle(degrees: number): string { return `${degrees.toFixed(1)}°`; }

function getButtonClass(button: string): string {
    const pressed = input.gamepad.value.buttons[button];
    return pressed
        ? 'bg-saint-primary text-white'
        : 'bg-saint-surface-light text-saint-text-muted';
}
</script>

<template>
    <div class="p-6 space-y-6">
        <!-- Gamepad Status -->
        <div class="card">
            <div class="flex items-center justify-between mb-4">
                <h2 class="text-lg font-semibold">Gamepad</h2>
                <div class="flex items-center gap-2">
                    <div :class="input.gamepad.value.connected
                        ? 'status-indicator status-connected'
                        : 'status-indicator status-disconnected'"></div>
                    <span class="text-sm text-saint-text-muted">
                        {{ input.gamepad.value.connected ? input.gamepad.value.name : 'No gamepad detected' }}
                    </span>
                </div>
            </div>

            <div v-if="input.gamepad.value.connected" class="grid grid-cols-2 lg:grid-cols-4 gap-6">
                <!-- Left Stick -->
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Left Stick</span>
                    <div class="relative w-32 h-32 bg-saint-background rounded-full border-2 border-saint-surface-light">
                        <div class="absolute w-4 h-4 bg-saint-primary rounded-full transform -translate-x-1/2 -translate-y-1/2"
                             :style="{
                                 left: `${50 + input.gamepad.value.leftStick.x * 40}%`,
                                 top:  `${50 - input.gamepad.value.leftStick.y * 40}%`,
                             }"></div>
                        <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                            <div class="w-1 h-full bg-saint-surface-light opacity-30"></div>
                        </div>
                        <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                            <div class="w-full h-1 bg-saint-surface-light opacity-30"></div>
                        </div>
                    </div>
                    <span class="text-xs text-saint-text-muted mt-2">
                        {{ formatAxis(input.gamepad.value.leftStick.x) }},
                        {{ formatAxis(input.gamepad.value.leftStick.y) }}
                    </span>
                </div>

                <!-- Right Stick -->
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Right Stick</span>
                    <div class="relative w-32 h-32 bg-saint-background rounded-full border-2 border-saint-surface-light">
                        <div class="absolute w-4 h-4 bg-saint-primary rounded-full transform -translate-x-1/2 -translate-y-1/2"
                             :style="{
                                 left: `${50 + input.gamepad.value.rightStick.x * 40}%`,
                                 top:  `${50 - input.gamepad.value.rightStick.y * 40}%`,
                             }"></div>
                        <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                            <div class="w-1 h-full bg-saint-surface-light opacity-30"></div>
                        </div>
                        <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                            <div class="w-full h-1 bg-saint-surface-light opacity-30"></div>
                        </div>
                    </div>
                    <span class="text-xs text-saint-text-muted mt-2">
                        {{ formatAxis(input.gamepad.value.rightStick.x) }},
                        {{ formatAxis(input.gamepad.value.rightStick.y) }}
                    </span>
                </div>

                <!-- Triggers -->
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Triggers</span>
                    <div class="flex gap-4">
                        <div class="flex flex-col items-center">
                            <span class="text-xs text-saint-text-muted mb-1">LT</span>
                            <div class="w-8 h-32 bg-saint-background rounded border border-saint-surface-light relative overflow-hidden">
                                <div class="absolute bottom-0 left-0 right-0 bg-saint-primary transition-all"
                                     :style="{ height: `${input.gamepad.value.leftTrigger * 100}%` }"></div>
                            </div>
                            <span class="text-xs text-saint-text-muted mt-1">{{ formatAxis(input.gamepad.value.leftTrigger) }}</span>
                        </div>
                        <div class="flex flex-col items-center">
                            <span class="text-xs text-saint-text-muted mb-1">RT</span>
                            <div class="w-8 h-32 bg-saint-background rounded border border-saint-surface-light relative overflow-hidden">
                                <div class="absolute bottom-0 left-0 right-0 bg-saint-primary transition-all"
                                     :style="{ height: `${input.gamepad.value.rightTrigger * 100}%` }"></div>
                            </div>
                            <span class="text-xs text-saint-text-muted mt-1">{{ formatAxis(input.gamepad.value.rightTrigger) }}</span>
                        </div>
                    </div>
                </div>

                <!-- Buttons -->
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Buttons</span>
                    <div class="grid grid-cols-4 gap-2">
                        <div v-for="btn in buttonList" :key="btn"
                             :class="getButtonClass(btn)"
                             class="w-8 h-8 rounded flex items-center justify-center text-xs font-medium">
                            {{ btn }}
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Touchpads (Steam Deck) -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Touchpads</h2>
            <div class="grid grid-cols-2 gap-6">
                <!-- Left Touchpad -->
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Left Touchpad</span>
                    <div class="relative w-32 h-32 bg-saint-background rounded-lg border-2"
                         :class="input.leftTouchpad.value.touched ? 'border-saint-primary' : 'border-saint-surface-light'">
                        <div class="absolute w-3 h-3 rounded-full transform -translate-x-1/2 -translate-y-1/2"
                             :class="input.leftTouchpad.value.clicked ? 'bg-saint-primary' : 'bg-saint-accent'"
                             :style="{
                                 left: `${50 + input.leftTouchpad.value.x * 45}%`,
                                 top:  `${50 - input.leftTouchpad.value.y * 45}%`,
                                 opacity: input.leftTouchpad.value.touched ? 1 : 0.3,
                             }"></div>
                    </div>
                    <span class="text-xs text-saint-text-muted mt-2">
                        {{ formatAxis(input.leftTouchpad.value.x) }},
                        {{ formatAxis(input.leftTouchpad.value.y) }}
                        {{ input.leftTouchpad.value.touched ? '(touched)' : '' }}
                        {{ input.leftTouchpad.value.clicked ? '(clicked)' : '' }}
                    </span>
                </div>
                <!-- Right Touchpad -->
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Right Touchpad</span>
                    <div class="relative w-32 h-32 bg-saint-background rounded-lg border-2"
                         :class="input.rightTouchpad.value.touched ? 'border-saint-primary' : 'border-saint-surface-light'">
                        <div class="absolute w-3 h-3 rounded-full transform -translate-x-1/2 -translate-y-1/2"
                             :class="input.rightTouchpad.value.clicked ? 'bg-saint-primary' : 'bg-saint-accent'"
                             :style="{
                                 left: `${50 + input.rightTouchpad.value.x * 45}%`,
                                 top:  `${50 - input.rightTouchpad.value.y * 45}%`,
                                 opacity: input.rightTouchpad.value.touched ? 1 : 0.3,
                             }"></div>
                    </div>
                    <span class="text-xs text-saint-text-muted mt-2">
                        {{ formatAxis(input.rightTouchpad.value.x) }},
                        {{ formatAxis(input.rightTouchpad.value.y) }}
                        {{ input.rightTouchpad.value.touched ? '(touched)' : '' }}
                        {{ input.rightTouchpad.value.clicked ? '(clicked)' : '' }}
                    </span>
                </div>
            </div>
        </div>

        <!-- Gyroscope -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Gyroscope</h2>
            <div class="grid grid-cols-3 gap-4">
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Pitch</span>
                    <div class="text-2xl font-mono">{{ formatAngle(input.gyro.value.pitch) }}</div>
                </div>
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Roll</span>
                    <div class="text-2xl font-mono">{{ formatAngle(input.gyro.value.roll) }}</div>
                </div>
                <div class="flex flex-col items-center">
                    <span class="text-sm text-saint-text-muted mb-2">Yaw</span>
                    <div class="text-2xl font-mono">{{ formatAngle(input.gyro.value.yaw) }}</div>
                </div>
            </div>
        </div>

        <!-- Command Log -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Command Log</h2>
            <div class="bg-saint-background rounded-lg p-3 h-48 overflow-y-auto font-mono text-sm">
                <p v-if="!conn.isConnected.value" class="text-saint-text-muted">
                    Connect to server to see commands...
                </p>
                <p v-else class="text-saint-text-muted">Commands will appear here...</p>
            </div>
        </div>
    </div>
</template>
