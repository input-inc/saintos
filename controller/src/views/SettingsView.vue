<!--
    Settings view. Connection form + mDNS discovery list + UI scale +
    OTA flow + dev tools + quit. The OTA logic (download → SHA-256
    verify → install_controller_update) is identical to the Angular
    SettingsComponent; the comparison logic uses installed.json
    sidecar + currentVersion fallback the same way.

    Native <select> for the scale picker replaced with <SaintSelect>
    (Headless UI Listbox) — that was the original reason this whole
    migration started.
-->
<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { getVersion } from '@tauri-apps/api/app';
import SaintSelect from '../components/SaintSelect.vue';
import {
    useConnection,
    ConnectionStatus,
    type ConnectionConfig,
} from '../composables/useConnection';
import { useKeyboard } from '../composables/useKeyboard';

interface DiscoveredServer {
    instance_name: string;
    hostname: string;
    ipv4: string | null;
    port: number;
}

interface FirmwareInfo {
    type: string;
    latest_version?: string;
    latest_package?: string;
    latest_checksum?: string;
    updated?: string;
}

type UpdateState =
    | 'not-connected'
    | 'checking'
    | 'up-to-date'
    | 'available'
    | 'downloading'
    | 'verifying'
    | 'installing'
    | 'installed'
    | 'error';

const conn = useConnection();
const keyboard = useKeyboard();

const config = ref<ConnectionConfig>({ host: 'localhost', port: 80, password: '' });
const pollingRate = ref(16);
const throttleMs = ref(50);
const devtoolsOpen = ref(false);
const uiScale = ref(1.0);

const discoveredServers = ref<DiscoveredServer[]>([]);
let discoveryPollHandle: ReturnType<typeof setInterval> | null = null;

const currentVersion = ref<string | null>(null);
const installedInfo = ref<{ version?: string; checksum?: string; filename?: string } | null>(null);
const updateInfo = ref<FirmwareInfo | null>(null);
const updateState = ref<UpdateState>('not-connected');
const updateError = ref<string>('');
const updateProgress = ref<string>('');

const scaleOptions = [
    { value: 1.0, label: '100% (Default)' },
    { value: 1.25, label: '125%' },
    { value: 1.5, label: '150%' },
    { value: 1.75, label: '175%' },
    { value: 2.0, label: '200%' },
];

const pollingRateOptions = [
    { value: 16, label: '60 Hz (16ms)' },
    { value: 8,  label: '120 Hz (8ms)' },
    { value: 4,  label: '250 Hz (4ms)' },
];

const throttleOptions = [
    { value: 50,  label: '50ms (Default)' },
    { value: 33,  label: '33ms (30 Hz)' },
    { value: 16,  label: '16ms (60 Hz)' },
    { value: 100, label: '100ms (10 Hz)' },
];

// ─── Connection status ────────────────────────────────────────────────

function statusClass(): string {
    const base = 'status-indicator';
    switch (conn.status.value) {
        case ConnectionStatus.Connected:     return `${base} status-connected`;
        case ConnectionStatus.Connecting:
        case ConnectionStatus.Authenticating: return `${base} status-connecting`;
        default:                              return `${base} status-disconnected`;
    }
}

function statusText(): string {
    switch (conn.status.value) {
        case ConnectionStatus.Connected:     return 'Connected';
        case ConnectionStatus.Connecting:    return 'Connecting...';
        case ConnectionStatus.Authenticating: return 'Authenticating...';
        case ConnectionStatus.Error:         return 'Connection Error';
        default:                             return 'Disconnected';
    }
}

const isConnecting = computed(() =>
    conn.status.value === ConnectionStatus.Connecting ||
    conn.status.value === ConnectionStatus.Authenticating);

async function doConnect(): Promise<void> {
    localStorage.setItem('saint-controller-config', JSON.stringify({
        host: config.value.host,
        port: config.value.port,
        password: config.value.password,
    }));
    try {
        await conn.connect(config.value);
    } catch (err) {
        console.error('Connection failed:', err);
    }
}

async function doDisconnect(): Promise<void> {
    await conn.disconnect();
}

function selectDiscoveredServer(server: DiscoveredServer): void {
    const hostLabel = server.hostname ? `${server.hostname}.local` : (server.ipv4 ?? '');
    if (hostLabel) config.value.host = hostLabel;
    if (server.port > 0) config.value.port = server.port;
}

async function refreshDiscoveredServers(): Promise<void> {
    try {
        const servers = await invoke<DiscoveredServer[]>('discover_servers');
        discoveredServers.value = servers;
    } catch (err) {
        console.warn('discover_servers failed:', err);
    }
}

// ─── UI scale ─────────────────────────────────────────────────────────

async function applyScale(scale: number): Promise<void> {
    try {
        await invoke('set_zoom', { scale });
    } catch (err) {
        console.error('[Settings] set_zoom failed, falling back to CSS zoom:', err);
        document.documentElement.style.zoom = scale.toString();
    }
}

function onScaleChange(scale: number): void {
    const v = typeof scale === 'string' ? parseFloat(scale) : scale;
    uiScale.value = v;
    localStorage.setItem('saint-controller-ui-scale', v.toString());
    void applyScale(v);
}

async function initializeScale(): Promise<void> {
    const saved = localStorage.getItem('saint-controller-ui-scale');
    uiScale.value = saved ? parseFloat(saved) : 1.0;
    await applyScale(uiScale.value);
}

// ─── Version + OTA ────────────────────────────────────────────────────

async function initializeVersion(): Promise<void> {
    try { currentVersion.value = await getVersion(); }
    catch (err) {
        console.error('[Settings] getVersion failed:', err);
        currentVersion.value = 'unknown';
    }
    try {
        installedInfo.value = await invoke<typeof installedInfo.value>('get_installed_controller_info');
    } catch (err) {
        console.error('[Settings] get_installed_controller_info failed:', err);
        installedInfo.value = null;
    }
}

async function checkForUpdate(): Promise<void> {
    updateError.value = '';
    if (!conn.isConnected.value) {
        updateState.value = 'not-connected';
        updateInfo.value = null;
        return;
    }
    const cfg = conn.getSavedConfig();
    if (!cfg) { updateState.value = 'not-connected'; return; }
    updateState.value = 'checking';
    try {
        const resp = await fetch(`http://${cfg.host}:${cfg.port}/api/firmware/controller`);
        if (!resp.ok) {
            if (resp.status === 404) {
                updateInfo.value = null;
                updateState.value = 'up-to-date';
                return;
            }
            throw new Error(`HTTP ${resp.status}`);
        }
        const info: FirmwareInfo = await resp.json();
        updateInfo.value = info;

        if (!info.latest_version || !info.latest_package || !info.latest_checksum) {
            updateState.value = 'up-to-date';
        } else {
            const installed = installedInfo.value;
            const upToDate = installed?.version && installed.checksum
                ? installed.version === info.latest_version
                    && installed.checksum === info.latest_checksum
                : info.latest_version === currentVersion.value;
            updateState.value = upToDate ? 'up-to-date' : 'available';
        }
    } catch (err) {
        updateError.value = `Update check failed: ${err}`;
        updateState.value = 'error';
    }
}

async function installUpdate(): Promise<void> {
    const info = updateInfo.value;
    const cfg = conn.getSavedConfig();
    if (!info?.latest_package || !info.latest_checksum || !cfg) {
        updateError.value = 'Missing update info — try Check again.';
        updateState.value = 'error';
        return;
    }
    try {
        updateState.value = 'downloading';
        updateProgress.value = '';

        const url = `http://${cfg.host}:${cfg.port}/api/firmware/controller/${info.latest_package}`;
        const resp = await fetch(url);
        if (!resp.ok) throw new Error(`download HTTP ${resp.status}`);
        const buf = await resp.arrayBuffer();
        const megabytes = (buf.byteLength / 1024 / 1024).toFixed(1);
        updateProgress.value = `${megabytes} MB downloaded`;

        updateState.value = 'verifying';
        const hashBuf = await crypto.subtle.digest('SHA-256', buf);
        const hashHex = Array.from(new Uint8Array(hashBuf))
            .map(b => b.toString(16).padStart(2, '0')).join('');
        if (hashHex !== info.latest_checksum) {
            throw new Error(
                `Checksum mismatch (expected ${info.latest_checksum.slice(0, 12)}…, got ${hashHex.slice(0, 12)}…)`);
        }

        updateState.value = 'installing';
        const bytes = Array.from(new Uint8Array(buf));
        await invoke('install_controller_update', {
            bytes,
            version: info.latest_version,
            filename: info.latest_package,
            checksum: info.latest_checksum,
        });
        installedInfo.value = {
            version: info.latest_version,
            checksum: info.latest_checksum,
            filename: info.latest_package,
        };
        updateState.value = 'installed';
    } catch (err) {
        updateError.value = `Update failed: ${err}`;
        updateState.value = 'error';
    }
}

// ─── Devtools / quit ─────────────────────────────────────────────────

async function checkDevtoolsState(): Promise<void> {
    try { devtoolsOpen.value = await invoke<boolean>('is_devtools_open'); }
    catch (err) { console.error('Failed to check devtools state:', err); }
}

async function toggleDevtools(): Promise<void> {
    try {
        if (devtoolsOpen.value) {
            await invoke('close_devtools');
            devtoolsOpen.value = false;
        } else {
            await invoke('open_devtools');
            devtoolsOpen.value = true;
        }
    } catch (err) {
        console.error('Failed to toggle devtools:', err);
    }
}

async function quitApp(): Promise<void> {
    try {
        if (conn.isConnected.value) await conn.disconnect();
        await invoke('quit_app');
    } catch (err) {
        console.error('Failed to quit app:', err);
    }
}

// ─── Lifecycle ───────────────────────────────────────────────────────

// Restore saved config + scale + version on mount.
const saved = localStorage.getItem('saint-controller-config');
if (saved) {
    try { config.value = { ...config.value, ...JSON.parse(saved) }; }
    catch { /* ignore */ }
}
void checkDevtoolsState();
void initializeScale();

// Auto-check for updates when connection becomes Connected.
// Uses watch (not watchEffect) so it doesn't track the writes to
// updateState that checkForUpdate makes — otherwise we'd recurse.
watch(() => conn.isConnected.value, (connected) => {
    const state = updateState.value;
    const inFlight =
        state === 'downloading' ||
        state === 'verifying' ||
        state === 'installing';
    if (connected && !inFlight) {
        void checkForUpdate();
    } else if (!connected && !inFlight) {
        updateState.value = 'not-connected';
        updateInfo.value = null;
    }
});

onMounted(() => {
    void refreshDiscoveredServers();
    discoveryPollHandle = setInterval(() => {
        if (!conn.isConnected.value) void refreshDiscoveredServers();
    }, 3000);
    void initializeVersion();
});

onBeforeUnmount(() => {
    if (discoveryPollHandle !== null) {
        clearInterval(discoveryPollHandle);
        discoveryPollHandle = null;
    }
});
</script>

<template>
    <div class="p-6 space-y-6 h-full overflow-y-auto touch-scroll">
        <!-- Connection Settings -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Connection</h2>
            <div class="space-y-4">
                <!-- Detected servers (mDNS) -->
                <div v-if="discoveredServers.length > 0 && !conn.isConnected.value">
                    <label class="block text-sm text-saint-text-muted mb-1">
                        Detected servers
                        <span class="text-xs">(auto-discovered on this network)</span>
                    </label>
                    <div class="flex flex-col gap-1">
                        <button v-for="server in discoveredServers" :key="server.instance_name"
                                type="button"
                                class="btn btn-secondary text-left flex justify-between items-center"
                                @click="selectDiscoveredServer(server)">
                            <span class="font-mono">{{ server.hostname || 'unknown' }}.local</span>
                            <span class="text-xs text-saint-text-muted">
                                {{ server.ipv4 || '(resolving…)' }}:{{ server.port }}
                            </span>
                        </button>
                    </div>
                </div>

                <div class="grid grid-cols-2 gap-4">
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Host</label>
                        <input type="text" class="input w-full"
                               v-model="config.host"
                               :disabled="conn.isConnected.value"
                               placeholder="192.168.1.100 or opensaint.local">
                    </div>
                    <div>
                        <label class="block text-sm text-saint-text-muted mb-1">Port</label>
                        <input type="number" class="input w-full"
                               v-model.number="config.port"
                               :disabled="conn.isConnected.value"
                               placeholder="80">
                    </div>
                </div>

                <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Password</label>
                    <input type="password" class="input w-full"
                           v-model="config.password"
                           :disabled="conn.isConnected.value"
                           placeholder="Enter password">
                </div>

                <div v-if="conn.error.value"
                     class="bg-saint-error/20 border border-saint-error rounded-lg p-3 text-sm text-saint-error">
                    {{ conn.error.value }}
                </div>

                <div class="flex items-center justify-between pt-2">
                    <div class="flex items-center gap-2">
                        <div :class="statusClass()"></div>
                        <span class="text-sm">{{ statusText() }}</span>
                    </div>
                    <div class="flex gap-3">
                        <button v-if="!conn.isConnected.value"
                                class="btn btn-primary"
                                @click="doConnect"
                                :disabled="isConnecting">
                            {{ isConnecting ? 'Connecting...' : 'Connect' }}
                        </button>
                        <button v-else class="btn btn-danger" @click="doDisconnect">
                            Disconnect
                        </button>
                    </div>
                </div>
            </div>
        </div>

        <!-- Display Settings -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Display</h2>
            <div class="space-y-4">
                <div>
                    <label class="block text-sm text-saint-text-muted mb-1">
                        UI Scale (for high DPI displays)
                    </label>
                    <SaintSelect :model-value="uiScale"
                                 :options="scaleOptions"
                                 @update:model-value="onScaleChange" />
                    <p class="text-xs text-saint-text-muted mt-1">
                        Increase scale for better visibility on Steam Deck and other high DPI displays
                    </p>
                </div>

                <div>
                    <label class="flex items-start gap-3 cursor-pointer">
                        <input type="checkbox" class="mt-1 w-4 h-4 rounded"
                               :checked="keyboard.enabled.value"
                               @change="keyboard.setEnabled(($event.target as HTMLInputElement).checked)">
                        <span>
                            <span class="block text-sm">In-app virtual keyboard</span>
                            <span class="block text-xs text-saint-text-muted mt-0.5">
                                When off, focus a text field and press Steam + X (Game Mode)
                                or use your system keyboard (Desktop Mode) instead.
                            </span>
                        </span>
                    </label>
                </div>
            </div>
        </div>

        <!-- Input Settings -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Input Settings</h2>
            <div class="space-y-4">
                <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Polling Rate</label>
                    <SaintSelect v-model="pollingRate" :options="pollingRateOptions" />
                </div>
                <div>
                    <label class="block text-sm text-saint-text-muted mb-1">Command Throttle</label>
                    <SaintSelect v-model="throttleMs" :options="throttleOptions" />
                </div>
            </div>
        </div>

        <!-- About -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">About</h2>
            <div class="space-y-2 text-sm text-saint-text-muted">
                <p><span class="text-saint-text">SAINT Controller</span> v{{ currentVersion || '…' }}</p>
                <p>A Tauri-based controller application for SAINT.OS robots.</p>
                <p>Supports gamepad, gyroscope, and touch input.</p>
            </div>
        </div>

        <!-- Software Update -->
        <div class="card">
            <div class="flex items-start justify-between mb-4">
                <h2 class="text-lg font-semibold">Software Update</h2>
                <button class="btn btn-secondary btn-sm flex items-center gap-1"
                        @click="checkForUpdate"
                        :disabled="['checking','downloading','verifying','installing'].includes(updateState)">
                    <span class="material-icons icon-sm">refresh</span>
                    Check
                </button>
            </div>
            <div class="space-y-3 text-sm">
                <div class="flex items-center justify-between">
                    <span class="text-saint-text-muted">Installed</span>
                    <span class="font-mono">{{ currentVersion || '…' }}</span>
                </div>
                <div v-if="updateInfo?.latest_version" class="flex items-center justify-between">
                    <span class="text-saint-text-muted">Latest on server</span>
                    <span class="font-mono">{{ updateInfo.latest_version }}</span>
                </div>

                <template v-if="updateState === 'not-connected'">
                    <p class="text-saint-text-muted">
                        Connect to a SAINT.OS server (above) to check for updates.
                    </p>
                </template>
                <template v-else-if="updateState === 'checking'">
                    <p class="text-saint-text-muted">Checking for updates…</p>
                </template>
                <template v-else-if="updateState === 'up-to-date'">
                    <p class="text-saint-text-muted">You're on the latest version.</p>
                </template>
                <template v-else-if="updateState === 'available'">
                    <button class="btn btn-primary w-full" @click="installUpdate">
                        Install update
                    </button>
                </template>
                <template v-else-if="updateState === 'downloading'">
                    <p class="text-saint-text-muted">Downloading… {{ updateProgress }}</p>
                </template>
                <template v-else-if="updateState === 'verifying'">
                    <p class="text-saint-text-muted">Verifying checksum…</p>
                </template>
                <template v-else-if="updateState === 'installing'">
                    <p class="text-saint-text-muted">
                        Installing on host (this may take a minute)…
                    </p>
                </template>
                <template v-else-if="updateState === 'installed'">
                    <div class="bg-saint-success/20 border border-saint-success rounded-lg p-3">
                        Update installed.
                        <strong class="block mt-1">Please manually relaunch the SAINT Controller</strong>
                        from your Steam library so the new version takes effect.
                    </div>
                </template>
                <template v-else-if="updateState === 'error'">
                    <div class="bg-saint-error/20 border border-saint-error rounded-lg p-3 text-saint-error">
                        {{ updateError }}
                    </div>
                </template>
            </div>
        </div>

        <!-- Developer Tools -->
        <div class="card">
            <h2 class="text-lg font-semibold mb-4">Developer Tools</h2>
            <div class="space-y-3">
                <div class="flex items-center justify-between">
                    <div>
                        <p class="font-medium">Web Inspector</p>
                        <p class="text-sm text-saint-text-muted">
                            Open browser developer tools for debugging
                        </p>
                    </div>
                    <button class="btn btn-secondary flex items-center gap-2" @click="toggleDevtools">
                        <span class="material-icons icon-sm">code</span>
                        {{ devtoolsOpen ? 'Close' : 'Open' }} DevTools
                    </button>
                </div>
                <div class="flex items-center justify-between">
                    <div>
                        <p class="font-medium">Log Location</p>
                        <p class="text-sm text-saint-text-muted font-mono">
                            ~/.local/share/com.saintos.controller/logs/
                        </p>
                    </div>
                </div>
            </div>
        </div>

        <!-- Quit Application -->
        <div class="card">
            <div class="flex items-center justify-between">
                <div>
                    <h2 class="text-lg font-semibold">Quit Application</h2>
                    <p class="text-sm text-saint-text-muted">Exit the SAINT Controller</p>
                </div>
                <button class="btn btn-danger flex items-center gap-2" @click="quitApp">
                    <span class="material-icons icon-sm">power_settings_new</span>
                    Quit
                </button>
            </div>
        </div>
    </div>
</template>
