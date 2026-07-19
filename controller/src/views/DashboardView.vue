<!--
    Dashboard — the operator's at-a-glance status page (first tab).
    Holds the battery (BMS) packs and WiFi link/channel tools that used
    to live in the app's bottom accordion. Both useBatteries and useWifi
    are module-scoped singletons, so the live data is shared with the
    rest of the app.
-->
<script setup lang="ts">
import { ref } from 'vue';
import { useBatteries } from '../composables/useBatteries';
import { useWifi, type SurveyChannel } from '../composables/useWifi';

const { batteries } = useBatteries();

const {
    config: wifiConfig,
    live: wifiLive,
    survey: wifiSurvey,
    switching: wifiSwitching,
    bestPick: wifiBestPick,
    runSurvey: runWifiSurvey,
    applyChannel: applyWifiChannel,
} = useWifi();

// ─── Battery helpers ─────────────────────────────────────────────────
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

// ─── WiFi helpers ────────────────────────────────────────────────────
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
function wifiSignalPct(dbm: number | null): number {
    if (dbm === null) return 0;
    return Math.max(0, Math.min(100, ((dbm + 90) / 60) * 100));
}
function apCountClass(count: number): string {
    if (count === 0) return 'bg-saint-success/20 text-saint-success';
    if (count <= 2) return 'bg-saint-surface text-saint-text-muted';
    if (count <= 5) return 'bg-yellow-500/20 text-yellow-500';
    return 'bg-red-500/20 text-red-500';
}
</script>

<template>
    <div class="h-full overflow-auto touch-scroll p-4 space-y-6">

        <!-- ─── Batteries ─────────────────────────────────────────── -->
        <section>
            <h2 class="text-lg font-semibold mb-3 flex items-center gap-2">
                <span class="material-icons text-xl">battery_full</span> Batteries
            </h2>
            <div v-if="batteries.length === 0"
                 class="text-saint-text-muted text-sm py-6 text-center bg-saint-surface rounded-lg">
                No batteries reporting.
            </div>
            <div v-else class="flex flex-wrap gap-3">
                <div v-for="battery in batteries" :key="battery.key"
                     class="bg-saint-surface-light rounded-lg p-3 flex-1 min-w-[280px] flex flex-col">
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

                    <div class="h-2 w-full rounded-full bg-saint-surface overflow-hidden mb-3">
                        <div class="h-2 transition-all"
                             :class="battery.soc === null ? 'bg-saint-surface'
                                     : battery.soc >= 50 ? 'bg-saint-success'
                                     : battery.soc >= 20 ? 'bg-yellow-500' : 'bg-red-500'"
                             :style="{ width: `${Math.max(0, Math.min(100, battery.soc ?? 0))}%` }" />
                    </div>

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
        </section>

        <!-- ─── WiFi ──────────────────────────────────────────────── -->
        <section>
            <h2 class="text-lg font-semibold mb-3 flex items-center gap-2">
                <span class="material-icons text-xl">wifi</span> WiFi
            </h2>

            <div v-if="wifiSwitching.active"
                 class="flex flex-col items-center justify-center gap-2 text-saint-text-muted py-10 bg-saint-surface-light rounded-lg">
                <span class="material-icons text-4xl animate-spin">autorenew</span>
                <div class="font-medium text-saint-text">Restarting WiFi AP…</div>
                <div v-if="wifiSwitching.detail" class="text-sm">
                    Switching to {{ wifiSwitching.detail }} — reconnecting automatically.
                </div>
            </div>

            <div v-else class="flex flex-wrap gap-3">
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
                    <div class="mt-auto text-[10px] text-saint-text-muted pt-2">
                        Signal is your Deck's link as the AP sees it, updated 1 Hz.
                    </div>
                </div>

                <!-- Channel survey + switch -->
                <div class="flex-1 min-w-[320px] bg-saint-surface-light rounded-lg p-3 flex flex-col">
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
                         class="py-6 flex items-center justify-center text-saint-text-muted text-sm text-center px-4">
                        Scan to see nearby AP congestion per channel and move the
                        robot's AP somewhere quieter.
                    </div>

                    <div v-else class="max-h-72 overflow-auto touch-scroll">
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
        </section>
    </div>
</template>
