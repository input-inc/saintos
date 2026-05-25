import { Component, effect, OnDestroy, OnInit, signal } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { ConnectionService, ConnectionStatus, ConnectionConfig } from '../../core/services/connection.service';
import { KeyboardService } from '../../core/services/keyboard.service';
import { invoke } from '@tauri-apps/api/core';
import { getVersion } from '@tauri-apps/api/app';

/** Mirrors crate::discovery::DiscoveredServer. The Rust backend
 *  serializes this shape verbatim, so any rename on either side must
 *  be coordinated. */
interface DiscoveredServer {
  instance_name: string;
  hostname: string;
  ipv4: string | null;
  port: number;
}

/** Subset of the /api/firmware/<type> shape we care about. Mirrors
 *  the JSON produced by controller/appimage/build-bundle.sh's staging
 *  helper — keep the field names in sync with that script. */
interface FirmwareInfo {
  type: string;
  latest_version?: string;
  latest_package?: string;
  latest_checksum?: string;
  updated?: string;
}

/** UI states for the in-app OTA flow. 'not-connected' is the resting
 *  state when there's no server to ask; everything else is a step in
 *  the check → download → verify → install → done pipeline. */
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

@Component({
  selector: 'app-settings',
  standalone: true,
  imports: [CommonModule, FormsModule],
  template: `
    <div class="p-6 space-y-6 h-full overflow-y-auto touch-scroll">
      <!-- Connection Settings -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">Connection</h2>

        <div class="space-y-4">
          <!-- Detected servers (mDNS) -->
          @if (discoveredServers().length > 0 && !connectionService.isConnected()) {
            <div>
              <label class="block text-sm text-saint-text-muted mb-1">
                Detected servers
                <span class="text-xs">(auto-discovered on this network)</span>
              </label>
              <div class="flex flex-col gap-1">
                @for (server of discoveredServers(); track server.instance_name) {
                  <button type="button"
                          class="btn btn-secondary text-left flex justify-between items-center"
                          (click)="selectDiscoveredServer(server)">
                    <span class="font-mono">{{ server.hostname || 'unknown' }}.local</span>
                    <span class="text-xs text-saint-text-muted">
                      {{ server.ipv4 || '(resolving…)' }}:{{ server.port }}
                    </span>
                  </button>
                }
              </div>
            </div>
          }

          <div class="grid grid-cols-2 gap-4">
            <div>
              <label class="block text-sm text-saint-text-muted mb-1">Host</label>
              <input type="text" class="input w-full"
                     [(ngModel)]="config.host"
                     [disabled]="connectionService.isConnected()"
                     placeholder="192.168.1.100 or opensaint.local">
            </div>
            <div>
              <label class="block text-sm text-saint-text-muted mb-1">Port</label>
              <input type="number" class="input w-full"
                     [(ngModel)]="config.port"
                     [disabled]="connectionService.isConnected()"
                     placeholder="80">
            </div>
          </div>

          <div>
            <label class="block text-sm text-saint-text-muted mb-1">Password</label>
            <input type="password" class="input w-full"
                   [(ngModel)]="config.password"
                   [disabled]="connectionService.isConnected()"
                   placeholder="Enter password">
          </div>

          @if (connectionService.error()) {
            <div class="bg-saint-error/20 border border-saint-error rounded-lg p-3 text-sm text-saint-error">
              {{ connectionService.error() }}
            </div>
          }

          <div class="flex items-center justify-between pt-2">
            <div class="flex items-center gap-2">
              <div [class]="getStatusClass()"></div>
              <span class="text-sm">{{ getStatusText() }}</span>
            </div>

            <div class="flex gap-3">
              @if (!connectionService.isConnected()) {
                <button class="btn btn-primary"
                        (click)="connect()"
                        [disabled]="isConnecting()">
                  {{ isConnecting() ? 'Connecting...' : 'Connect' }}
                </button>
              } @else {
                <button class="btn btn-danger" (click)="disconnect()">
                  Disconnect
                </button>
              }
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
            <select class="input w-full" [(ngModel)]="uiScale" (ngModelChange)="onScaleChange($event)">
              @for (option of scaleOptions; track option.value) {
                <option [value]="option.value">{{ option.label }}</option>
              }
            </select>
            <p class="text-xs text-saint-text-muted mt-1">
              Increase scale for better visibility on Steam Deck and other high DPI displays
            </p>
          </div>

          <!-- In-app virtual keyboard toggle. Default off so the
               platform OSK (Steam OSK in Game Mode, KDE virtual
               keyboard in Desktop Mode) handles text entry. Turn on
               in environments without a working platform OSK. -->
          <div>
            <label class="flex items-start gap-3 cursor-pointer">
              <input type="checkbox" class="mt-1 w-4 h-4 rounded"
                     [checked]="keyboardEnabled()"
                     (change)="onKeyboardEnabledChange($any($event.target).checked)">
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
            <label class="block text-sm text-saint-text-muted mb-1">
              Polling Rate
            </label>
            <select class="input w-full" [(ngModel)]="pollingRate">
              <option [value]="16">60 Hz (16ms)</option>
              <option [value]="8">120 Hz (8ms)</option>
              <option [value]="4">250 Hz (4ms)</option>
            </select>
          </div>

          <div>
            <label class="block text-sm text-saint-text-muted mb-1">
              Command Throttle
            </label>
            <select class="input w-full" [(ngModel)]="throttleMs">
              <option [value]="50">50ms (Default)</option>
              <option [value]="33">33ms (30 Hz)</option>
              <option [value]="16">16ms (60 Hz)</option>
              <option [value]="100">100ms (10 Hz)</option>
            </select>
          </div>
        </div>
      </div>

      <!-- About -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">About</h2>
        <div class="space-y-2 text-sm text-saint-text-muted">
          <p><span class="text-saint-text">SAINT Controller</span> v{{ currentVersion() || '…' }}</p>
          <p>A Tauri-based controller application for SAINT.OS robots.</p>
          <p>Supports gamepad, gyroscope, and touch input.</p>
        </div>
      </div>

      <!-- Software Update -->
      <div class="card">
        <div class="flex items-start justify-between mb-4">
          <h2 class="text-lg font-semibold">Software Update</h2>
          <button class="btn btn-secondary btn-sm flex items-center gap-1"
                  (click)="checkForUpdate()"
                  [disabled]="updateState() === 'checking' || updateState() === 'downloading'
                             || updateState() === 'verifying' || updateState() === 'installing'">
            <span class="material-icons icon-sm">refresh</span>
            Check
          </button>
        </div>

        <div class="space-y-3 text-sm">
          <div class="flex items-center justify-between">
            <span class="text-saint-text-muted">Installed</span>
            <span class="font-mono">{{ currentVersion() || '…' }}</span>
          </div>
          @if (updateInfo()?.latest_version) {
            <div class="flex items-center justify-between">
              <span class="text-saint-text-muted">Latest on server</span>
              <span class="font-mono">{{ updateInfo()!.latest_version }}</span>
            </div>
          }

          @switch (updateState()) {
            @case ('not-connected') {
              <p class="text-saint-text-muted">
                Connect to a SAINT.OS server (above) to check for updates.
              </p>
            }
            @case ('checking') {
              <p class="text-saint-text-muted">Checking for updates…</p>
            }
            @case ('up-to-date') {
              <p class="text-saint-text-muted">You're on the latest version.</p>
            }
            @case ('available') {
              <button class="btn btn-primary w-full"
                      (click)="installUpdate()">
                Install update
              </button>
            }
            @case ('downloading') {
              <p class="text-saint-text-muted">
                Downloading… {{ updateProgress() }}
              </p>
            }
            @case ('verifying') {
              <p class="text-saint-text-muted">Verifying checksum…</p>
            }
            @case ('installing') {
              <p class="text-saint-text-muted">
                Installing on host (this may take a minute)…
              </p>
            }
            @case ('installed') {
              <div class="bg-saint-success/20 border border-saint-success rounded-lg p-3">
                Update installed.
                <strong class="block mt-1">Please manually relaunch the SAINT
                Controller</strong>
                from your Steam library so the new version takes effect.
              </div>
            }
            @case ('error') {
              <div class="bg-saint-error/20 border border-saint-error rounded-lg p-3 text-saint-error">
                {{ updateError() }}
              </div>
            }
          }
        </div>
      </div>

      <!-- Developer Tools -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">Developer Tools</h2>
        <div class="space-y-3">
          <div class="flex items-center justify-between">
            <div>
              <p class="font-medium">Web Inspector</p>
              <p class="text-sm text-saint-text-muted">Open browser developer tools for debugging</p>
            </div>
            <button class="btn btn-secondary flex items-center gap-2" (click)="toggleDevtools()">
              <span class="material-icons icon-sm">code</span>
              {{ devtoolsOpen() ? 'Close' : 'Open' }} DevTools
            </button>
          </div>
          <div class="flex items-center justify-between">
            <div>
              <p class="font-medium">Log Location</p>
              <p class="text-sm text-saint-text-muted font-mono">~/.local/share/com.saintos.controller/logs/</p>
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
          <button class="btn btn-danger flex items-center gap-2" (click)="quitApp()">
            <span class="material-icons icon-sm">power_settings_new</span>
            Quit
          </button>
        </div>
      </div>
    </div>
  `
})
export class SettingsComponent implements OnInit, OnDestroy {
  config: ConnectionConfig = {
    host: 'localhost',
    port: 80,
    password: ''
  };

  pollingRate = 16;
  throttleMs = 50;
  devtoolsOpen = signal(false);
  uiScale = 1.0;

  /** mDNS-discovered SAINT.OS servers on the LAN. Populated by a
   *  background poll while the controller is NOT connected — once
   *  connected we don't need to keep looking, and stopping the poll
   *  keeps the LAN multicast group quieter. */
  discoveredServers = signal<DiscoveredServer[]>([]);
  private discoveryPollHandle: ReturnType<typeof setInterval> | null = null;

  /** Controller version reported by Tauri (from tauri.conf.json, which
   *  is kept in sync with controller/VERSION by scripts/sync-version.js).
   *  Read once on init; doesn't change at runtime. This is the bare
   *  semver ("0.5.0") with no build metadata. */
  currentVersion = signal<string | null>(null);
  /** What the last successful `install_controller_update` call wrote to
   *  the installed.json sidecar — `{ version, checksum, filename, ... }`.
   *  Authoritative for update detection; falls back to currentVersion()
   *  only when no install has run on this Deck (hand-downloaded
   *  AppImage). Null until fetched, or stays null if the sidecar
   *  doesn't exist yet. */
  installedInfo = signal<{ version?: string; checksum?: string; filename?: string } | null>(null);
  /** Latest controller info from `/api/firmware/controller` on the
   *  connected server. Null until we've successfully fetched it once. */
  updateInfo = signal<FirmwareInfo | null>(null);
  /** Step in the OTA check/install pipeline — drives the UI states. */
  updateState = signal<UpdateState>('not-connected');
  /** Human-readable error surfaced under the Software Update card when
   *  any step in the OTA pipeline fails. */
  updateError = signal<string>('');
  /** Progress hint shown during the 'downloading' state (e.g. byte count). */
  updateProgress = signal<string>('');

  readonly scaleOptions = [
    { value: 1.0, label: '100% (Default)' },
    { value: 1.25, label: '125%' },
    { value: 1.5, label: '150%' },
    { value: 1.75, label: '175%' },
    { value: 2.0, label: '200%' },
  ];

  /** Mirror of KeyboardService.enabled exposed for the template's
   *  checkbox. Read via the signal so toggles elsewhere (or in a
   *  future hotkey) re-render the checkbox without manual sync. */
  keyboardEnabled = () => this.keyboardService.enabled();

  onKeyboardEnabledChange(value: boolean): void {
    this.keyboardService.setEnabled(value);
  }

  constructor(
    public connectionService: ConnectionService,
    private keyboardService: KeyboardService,
  ) {
    // Check initial devtools state
    this.checkDevtoolsState();

    // Load saved UI scale and apply it via Tauri
    this.initializeScale();

    // Load saved config from localStorage
    const saved = localStorage.getItem('saint-controller-config');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        this.config = { ...this.config, ...parsed };
      } catch {
        // Ignore parse errors
      }
    }

    // Re-check for updates whenever the connection becomes Connected.
    // Skips the check while a download/install is in flight so we don't
    // clobber the operator's in-progress action with a fresh poll.
    effect(() => {
      const connected = this.connectionService.isConnected();
      const state = this.updateState();
      const inFlight =
        state === 'downloading' ||
        state === 'verifying' ||
        state === 'installing';
      if (connected && !inFlight) {
        void this.checkForUpdate();
      } else if (!connected && !inFlight) {
        this.updateState.set('not-connected');
        this.updateInfo.set(null);
      }
    });
  }

  private async initializeScale(): Promise<void> {
    // Load saved scale from localStorage and apply it
    // Note: Tauri 2.x doesn't have a zoom getter, so localStorage is our source of truth
    const savedScale = localStorage.getItem('saint-controller-ui-scale');
    if (savedScale) {
      this.uiScale = parseFloat(savedScale);
    } else {
      // No saved preference - use default (1.0 for most platforms)
      // Could detect Steam Deck here and use 1.25 as default if needed
      this.uiScale = 1.0;
    }
    await this.applyScale(this.uiScale);
  }

  isConnecting(): boolean {
    const status = this.connectionService.status();
    return status === ConnectionStatus.Connecting ||
           status === ConnectionStatus.Authenticating;
  }

  getStatusClass(): string {
    const base = 'status-indicator';
    switch (this.connectionService.status()) {
      case ConnectionStatus.Connected:
        return `${base} status-connected`;
      case ConnectionStatus.Connecting:
      case ConnectionStatus.Authenticating:
        return `${base} status-connecting`;
      default:
        return `${base} status-disconnected`;
    }
  }

  getStatusText(): string {
    switch (this.connectionService.status()) {
      case ConnectionStatus.Connected:
        return 'Connected';
      case ConnectionStatus.Connecting:
        return 'Connecting...';
      case ConnectionStatus.Authenticating:
        return 'Authenticating...';
      case ConnectionStatus.Error:
        return 'Connection Error';
      default:
        return 'Disconnected';
    }
  }

  async connect(): Promise<void> {
    // Save config (including password for auto-connect)
    localStorage.setItem('saint-controller-config', JSON.stringify({
      host: this.config.host,
      port: this.config.port,
      password: this.config.password
    }));

    try {
      await this.connectionService.connect(this.config);
    } catch (err) {
      console.error('Connection failed:', err);
    }
  }

  async disconnect(): Promise<void> {
    await this.connectionService.disconnect();
  }

  ngOnInit(): void {
    // Initial pull is cheap (snapshot of whatever the embedded mDNS
    // browser has already cached) and lets the dropdown show
    // immediately when the page mounts. The interval keeps it fresh
    // as servers come and go on the LAN.
    void this.refreshDiscoveredServers();
    this.discoveryPollHandle = setInterval(() => {
      // Skip the poll while connected — once we've joined a server,
      // refreshing the list would only add noise.
      if (!this.connectionService.isConnected()) {
        void this.refreshDiscoveredServers();
      }
    }, 3000);

    // Resolve our own version once. The effect() in the constructor
    // handles the first update check as soon as the connection becomes
    // ready, so we don't need to schedule one here.
    void this.initializeVersion();
  }

  private async initializeVersion(): Promise<void> {
    try {
      this.currentVersion.set(await getVersion());
    } catch (err) {
      // getVersion() reads from the bundled tauri.conf.json — failures
      // are highly unusual but we still don't want them to break the
      // settings tab. Fall back to a placeholder and log.
      console.error('[Settings] getVersion failed:', err);
      this.currentVersion.set('unknown');
    }
    // installed.json records what the last successful in-app OTA wrote.
    // Pulling it once on init lets the update check compare the exact
    // version+checksum against the server, catching same-version-
    // different-content rebuilds that bare-version comparison misses.
    try {
      const info = await invoke<{ version?: string; checksum?: string; filename?: string } | null>(
        'get_installed_controller_info'
      );
      this.installedInfo.set(info);
    } catch (err) {
      console.error('[Settings] get_installed_controller_info failed:', err);
      this.installedInfo.set(null);
    }
  }

  /** Poll /api/firmware/controller on the connected server and decide
   *  whether an Install button should appear.
   *
   *  Comparison uses both `latest_version` AND `latest_checksum` so a
   *  rebuild that produces a different binary with the same version
   *  string still surfaces as "available". The local side comes from
   *  the installed.json sidecar written by the last successful OTA;
   *  when that sidecar doesn't exist (fresh manual install, no in-app
   *  OTA has run yet), we fall back to comparing the server's
   *  `latest_version` against Tauri's bare `getVersion()` — which is
   *  loose (no SHA suffix) but better than nothing. */
  async checkForUpdate(): Promise<void> {
    this.updateError.set('');
    if (!this.connectionService.isConnected()) {
      this.updateState.set('not-connected');
      this.updateInfo.set(null);
      return;
    }
    const config = this.connectionService.getSavedConfig();
    if (!config) {
      this.updateState.set('not-connected');
      return;
    }
    this.updateState.set('checking');
    try {
      const resp = await fetch(`http://${config.host}:${config.port}/api/firmware/controller`);
      if (!resp.ok) {
        // 404 means the server doesn't have any controller bundles
        // staged yet — treat that as "no updates available" rather
        // than an error so the UI doesn't shout at the operator.
        if (resp.status === 404) {
          this.updateInfo.set(null);
          this.updateState.set('up-to-date');
          return;
        }
        throw new Error(`HTTP ${resp.status}`);
      }
      const info: FirmwareInfo = await resp.json();
      this.updateInfo.set(info);

      if (!info.latest_version || !info.latest_package || !info.latest_checksum) {
        this.updateState.set('up-to-date');
      } else {
        const installed = this.installedInfo();
        const upToDate = installed?.version && installed.checksum
          ? installed.version === info.latest_version
            && installed.checksum === info.latest_checksum
          : info.latest_version === this.currentVersion();
        this.updateState.set(upToDate ? 'up-to-date' : 'available');
      }
    } catch (err) {
      this.updateError.set(`Update check failed: ${err}`);
      this.updateState.set('error');
    }
  }

  /** Download the .AppImage, verify SHA-256, then hand the bytes to
   *  the Rust `install_controller_update` command which writes the file
   *  to its canonical install path (~/.local/share/saint-controller/),
   *  marks it executable, and writes a .desktop entry. The operator
   *  must relaunch manually after a successful install. */
  async installUpdate(): Promise<void> {
    const info = this.updateInfo();
    const config = this.connectionService.getSavedConfig();
    if (!info?.latest_package || !info.latest_checksum || !config) {
      this.updateError.set('Missing update info — try Check again.');
      this.updateState.set('error');
      return;
    }
    try {
      this.updateState.set('downloading');
      this.updateProgress.set('');

      const url = `http://${config.host}:${config.port}/api/firmware/controller/${info.latest_package}`;
      const resp = await fetch(url);
      if (!resp.ok) throw new Error(`download HTTP ${resp.status}`);
      const buf = await resp.arrayBuffer();
      const megabytes = (buf.byteLength / 1024 / 1024).toFixed(1);
      this.updateProgress.set(`${megabytes} MB downloaded`);

      this.updateState.set('verifying');
      const hashBuf = await crypto.subtle.digest('SHA-256', buf);
      const hashHex = Array.from(new Uint8Array(hashBuf))
        .map((b) => b.toString(16).padStart(2, '0'))
        .join('');
      if (hashHex !== info.latest_checksum) {
        throw new Error(
          `Checksum mismatch (expected ${info.latest_checksum.slice(0, 12)}…, got ${hashHex.slice(0, 12)}…)`,
        );
      }

      this.updateState.set('installing');
      // Tauri's IPC serializes number arrays as Vec<u8> on the Rust
      // side. Multi-MB payloads work but allocate twice (here +
      // serialization); acceptable for a one-shot operator action.
      const bytes = Array.from(new Uint8Array(buf));
      await invoke('install_controller_update', {
        bytes,
        version: info.latest_version,
        filename: info.latest_package,
        checksum: info.latest_checksum,
      });
      // Update local cache of installed.json so a re-check immediately
      // after install reports "up to date" without re-reading the file.
      this.installedInfo.set({
        version: info.latest_version,
        checksum: info.latest_checksum,
        filename: info.latest_package,
      });

      this.updateState.set('installed');
    } catch (err) {
      this.updateError.set(`Update failed: ${err}`);
      this.updateState.set('error');
    }
  }

  ngOnDestroy(): void {
    if (this.discoveryPollHandle !== null) {
      clearInterval(this.discoveryPollHandle);
      this.discoveryPollHandle = null;
    }
  }

  private async refreshDiscoveredServers(): Promise<void> {
    try {
      const servers: DiscoveredServer[] = await invoke('discover_servers');
      this.discoveredServers.set(servers);
    } catch (err) {
      // Discovery being disabled (e.g. mDNS daemon couldn't bind its
      // socket) is reported as an empty list, not an error, so an
      // actual error here means something unexpected — log and keep
      // showing whatever we had before.
      console.warn('discover_servers failed:', err);
    }
  }

  /** Operator clicked a row in the discovered-servers list — copy its
   *  hostname/port into the form so the next Connect press uses it.
   *  The embedded mDNS resolver kicks in inside the connect command,
   *  so even on Steam Deck the `.local` name resolves correctly. */
  selectDiscoveredServer(server: DiscoveredServer): void {
    const hostLabel = server.hostname ? `${server.hostname}.local` : (server.ipv4 ?? '');
    if (hostLabel) {
      this.config.host = hostLabel;
    }
    if (server.port > 0) {
      this.config.port = server.port;
    }
  }

  async checkDevtoolsState(): Promise<void> {
    try {
      const isOpen = await invoke<boolean>('is_devtools_open');
      this.devtoolsOpen.set(isOpen);
    } catch (err) {
      console.error('Failed to check devtools state:', err);
    }
  }

  async toggleDevtools(): Promise<void> {
    try {
      if (this.devtoolsOpen()) {
        await invoke('close_devtools');
        this.devtoolsOpen.set(false);
      } else {
        await invoke('open_devtools');
        this.devtoolsOpen.set(true);
      }
    } catch (err) {
      console.error('Failed to toggle devtools:', err);
    }
  }

  async quitApp(): Promise<void> {
    try {
      // Disconnect first if connected
      if (this.connectionService.isConnected()) {
        await this.connectionService.disconnect();
      }
      // Quit the application via Tauri command
      await invoke('quit_app');
    } catch (err) {
      console.error('Failed to quit app:', err);
    }
  }

  onScaleChange(scale: number): void {
    // Ensure scale is a number (ngModel might pass string)
    const scaleValue = typeof scale === 'string' ? parseFloat(scale) : scale;
    this.uiScale = scaleValue;
    localStorage.setItem('saint-controller-ui-scale', scaleValue.toString());
    this.applyScale(scaleValue);
  }

  private async applyScale(scale: number): Promise<void> {
    // Use Tauri's webview zoom to scale the entire UI including native controls
    try {
      await invoke('set_zoom', { scale });
      console.log(`[Settings] Zoom set to ${scale * 100}%`);
    } catch (err) {
      console.error('[Settings] Failed to set zoom via Tauri:', err);
      // Fallback to CSS zoom if Tauri command fails
      document.documentElement.style.zoom = scale.toString();
    }
  }
}
