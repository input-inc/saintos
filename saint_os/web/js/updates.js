/**
 * SAINT.OS software update UI.
 *
 * Renders the update card on the Settings page from state broadcast on
 * the 'update' topic by the server's UpdateManager. Handles button
 * clicks for check / download / install / scan USB, shows the install
 * modal with release notes, and overlays an "installing" message that
 * survives the server restart that the install triggers.
 */

class UpdateManager {
    constructor() {
        this.state = null;
        this._installing = false;
        this._installPollHandle = null;
    }

    init() {
        const ws = window.saintWS;
        if (!ws) {
            console.warn('updates.js: WebSocket not ready');
            return;
        }

        // Subscribe to update state updates.
        ws.on('state', (msg) => {
            if (msg.node === 'update') {
                this.onState(msg.data);
            }
        });

        // Wire button clicks.
        document.getElementById('update-check-btn')?.addEventListener('click', () => this.checkNow());
        document.getElementById('update-download-btn')?.addEventListener('click', () => this.startDownload());
        document.getElementById('update-scan-usb-btn')?.addEventListener('click', () => this.scanUsb());

        // Subscribe to the topic once the WS is ready.
        const subscribe = async () => {
            try {
                await ws.subscribe(['update'], 5);
            } catch (err) {
                console.warn('updates.js subscribe failed:', err);
            }
        };
        if (ws.connected) {
            subscribe();
            this.fetchStatus();
        } else {
            ws.on('ready', () => {
                subscribe();
                this.fetchStatus();
            });
        }
    }

    async fetchStatus() {
        try {
            const result = await window.saintWS.management('update.get_status', {});
            if (result && result.data) this.onState(result.data);
        } catch (err) {
            console.warn('update.get_status failed:', err);
        }
    }

    async checkNow() {
        try {
            const result = await window.saintWS.management('update.check_now', {});
            if (result && result.data) this.onState(result.data);
        } catch (err) {
            console.warn('update.check_now failed:', err);
        }
    }

    async scanUsb() {
        try {
            const result = await window.saintWS.management('update.scan_usb', {});
            if (result && result.data) this.onState(result.data);
        } catch (err) {
            console.warn('update.scan_usb failed:', err);
        }
    }

    async startDownload() {
        try {
            await window.saintWS.management('update.download', {});
            // State will flip to 'downloading' via broadcast.
        } catch (err) {
            console.warn('update.download failed:', err);
        }
    }

    async stageUsb(version) {
        try {
            await window.saintWS.management('update.stage_usb', { version });
        } catch (err) {
            console.warn('update.stage_usb failed:', err);
        }
    }

    confirmInstall() {
        this.closeInstallModal();
        this._installing = true;
        document.getElementById('update-installing-overlay')?.classList.remove('hidden');
        window.saintWS.management('update.install', {}).catch((err) => {
            console.warn('update.install failed:', err);
            this._installing = false;
            document.getElementById('update-installing-overlay')?.classList.add('hidden');
        });
        // Server is about to restart — start polling for it to come back.
        this._startReconnectPoll();
    }

    closeInstallModal() {
        document.getElementById('update-install-modal')?.classList.add('hidden');
    }

    // ── State rendering ──────────────────────────────────────────────

    onState(state) {
        this.state = state;
        this.render();

        // If we were waiting for an install to start and a fresh state
        // arrives with the installed version updated, the server has
        // come back online with the new version. Reload to pick up new
        // assets.
        if (this._installing && state.status !== 'installing' && state.installed_version !== 'unknown') {
            // Reload after a short delay so the user sees the success.
            setTimeout(() => window.location.reload(), 1500);
        }
    }

    render() {
        const state = this.state;
        if (!state) return;

        document.getElementById('update-installed-version').textContent = state.installed_version || '--';

        const pill = document.getElementById('update-status-pill');
        const checkBtn = document.getElementById('update-check-btn');
        const downloadBtn = document.getElementById('update-download-btn');
        const availInfo = document.getElementById('update-available-info');
        const progress = document.getElementById('update-progress');
        const errorBox = document.getElementById('update-error');

        // Reset visibility.
        checkBtn.classList.add('hidden');
        downloadBtn.classList.add('hidden');
        availInfo.classList.add('hidden');
        progress.classList.add('hidden');
        errorBox.classList.add('hidden');

        switch (state.status) {
            case 'unknown':
                pill.textContent = 'Not checked';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-slate-700 text-slate-400';
                checkBtn.classList.remove('hidden');
                break;
            case 'checking':
                pill.textContent = 'Checking…';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-slate-700 text-slate-300';
                break;
            case 'up_to_date':
                pill.textContent = 'Up to date';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-300';
                checkBtn.classList.remove('hidden');
                break;
            case 'no_network':
                pill.textContent = 'Offline';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-amber-500/20 text-amber-300';
                checkBtn.classList.remove('hidden');
                break;
            case 'available': {
                pill.textContent = 'Update available';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-cyan-500/20 text-cyan-300';
                downloadBtn.classList.remove('hidden');
                checkBtn.classList.remove('hidden');
                availInfo.classList.remove('hidden');
                const rel = state.github_release || {};
                document.getElementById('update-available-version').textContent = rel.version || '--';
                const link = document.getElementById('update-release-link');
                if (rel.html_url) {
                    link.href = rel.html_url;
                    link.classList.remove('hidden');
                } else {
                    link.classList.add('hidden');
                }
                break;
            }
            case 'downloading':
                pill.textContent = 'Downloading';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-cyan-500/20 text-cyan-300';
                progress.classList.remove('hidden');
                {
                    const total = state.download_total || 0;
                    const got = state.download_received || 0;
                    const pct = total > 0 ? Math.min(100, (got / total) * 100) : 0;
                    document.getElementById('update-progress-bar').style.width = `${pct.toFixed(1)}%`;
                    document.getElementById('update-progress-pct').textContent =
                        total > 0 ? `${pct.toFixed(0)}%  (${this.formatBytes(got)} / ${this.formatBytes(total)})`
                                  : this.formatBytes(got);
                }
                break;
            case 'downloaded':
                pill.textContent = 'Ready to install';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-cyan-500/20 text-cyan-300';
                this.openInstallModalFromState();
                break;
            case 'installing':
                pill.textContent = 'Installing…';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-cyan-500/20 text-cyan-300';
                document.getElementById('update-installing-overlay')?.classList.remove('hidden');
                this._installing = true;
                this._startReconnectPoll();
                break;
            case 'error':
                pill.textContent = 'Error';
                pill.className = 'px-2 py-0.5 text-xs font-medium rounded-full bg-red-500/20 text-red-300';
                if (state.last_error) {
                    errorBox.textContent = state.last_error;
                    errorBox.classList.remove('hidden');
                }
                checkBtn.classList.remove('hidden');
                break;
            default:
                pill.textContent = state.status;
                break;
        }

        this.renderUsb();
    }

    renderUsb() {
        const container = document.getElementById('update-usb-results');
        if (!container) return;
        const state = this.state || {};
        const releases = state.usb_releases || [];

        if (state.last_usb_scan_at && releases.length === 0) {
            container.innerHTML = '<span class="text-slate-500">No newer SAINT.OS tarballs found on mounted USB drives.</span>';
            return;
        }
        if (!state.last_usb_scan_at) {
            container.innerHTML = 'No USB media scanned yet.';
            return;
        }

        container.innerHTML = releases.map(r => `
            <div class="flex items-center justify-between p-2 rounded bg-slate-900 border border-slate-700 mb-2">
                <div>
                    <div class="font-mono text-cyan-300">v${this.escape(r.version)}</div>
                    <div class="text-xs text-slate-500">${this.escape(r.asset_name)} · ${this.formatBytes(r.asset_size || 0)}</div>
                </div>
                <button class="btn-secondary text-xs" data-usb-version="${this.escape(r.version)}">
                    <span class="material-icons text-xs">install_desktop</span>
                    Install from USB
                </button>
            </div>
        `).join('');

        container.querySelectorAll('[data-usb-version]').forEach(btn => {
            btn.addEventListener('click', () => {
                this.stageUsb(btn.getAttribute('data-usb-version'));
            });
        });
    }

    openInstallModalFromState() {
        const state = this.state || {};
        const rel = state.github_release || {};
        document.getElementById('update-modal-version').textContent = rel.version || state.installed_version || '--';
        document.getElementById('update-modal-source').textContent =
            rel.source === 'usb' ? 'Source: USB media'
            : rel.source === 'github' ? 'Source: GitHub release'
            : '';
        const notesEl = document.getElementById('update-modal-notes');
        if (rel.notes && rel.notes.trim()) {
            notesEl.textContent = rel.notes;
        } else if (rel.source === 'usb') {
            notesEl.textContent = 'No release notes available for USB-supplied updates.';
        } else {
            notesEl.textContent = 'No release notes available.';
        }
        document.getElementById('update-install-modal').classList.remove('hidden');
    }

    // ── Reconnect polling after install ─────────────────────────────

    _startReconnectPoll() {
        if (this._installPollHandle) return;
        const detail = document.getElementById('update-installing-detail');
        let elapsed = 0;
        this._installPollHandle = setInterval(() => {
            elapsed += 2;
            if (detail) detail.textContent = `Waiting for server… ${elapsed}s`;
            // The ws layer auto-reconnects; if it does, the next state
            // broadcast will arrive and onState will tear this down.
        }, 2000);
    }

    // ── Helpers ─────────────────────────────────────────────────────

    formatBytes(n) {
        if (!n) return '0 B';
        const units = ['B', 'KB', 'MB', 'GB'];
        let i = 0;
        while (n >= 1024 && i < units.length - 1) { n /= 1024; i++; }
        return `${n.toFixed(i === 0 ? 0 : 1)} ${units[i]}`;
    }

    escape(s) {
        return String(s ?? '')
            .replace(/&/g, '&amp;').replace(/</g, '&lt;')
            .replace(/>/g, '&gt;').replace(/"/g, '&quot;');
    }
}

// Global singleton — wired by app.js after WS init.
window.updateManager = new UpdateManager();
