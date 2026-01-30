import { Component, signal } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { ConnectionService, ConnectionStatus, ConnectionConfig } from '../../core/services/connection.service';
import { invoke } from '@tauri-apps/api/core';

@Component({
  selector: 'app-settings',
  standalone: true,
  imports: [CommonModule, FormsModule],
  template: `
    <div class="p-6 space-y-6">
      <!-- Connection Settings -->
      <div class="card">
        <h2 class="text-lg font-semibold mb-4">Connection</h2>

        <div class="space-y-4">
          <div class="grid grid-cols-2 gap-4">
            <div>
              <label class="block text-sm text-saint-text-muted mb-1">Host</label>
              <input type="text" class="input w-full"
                     [(ngModel)]="config.host"
                     [disabled]="connectionService.isConnected()"
                     placeholder="192.168.1.100">
            </div>
            <div>
              <label class="block text-sm text-saint-text-muted mb-1">Port</label>
              <input type="number" class="input w-full"
                     [(ngModel)]="config.port"
                     [disabled]="connectionService.isConnected()"
                     placeholder="9090">
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
          <p><span class="text-saint-text">SAINT Controller</span> v0.1.0</p>
          <p>A Tauri-based controller application for SAINT.OS robots.</p>
          <p>Supports gamepad, gyroscope, and touch input.</p>
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
export class SettingsComponent {
  config: ConnectionConfig = {
    host: 'localhost',
    port: 9090,
    password: ''
  };

  pollingRate = 16;
  throttleMs = 50;
  devtoolsOpen = signal(false);
  uiScale = 1.0;

  readonly scaleOptions = [
    { value: 1.0, label: '100% (Default)' },
    { value: 1.25, label: '125%' },
    { value: 1.5, label: '150%' },
    { value: 1.75, label: '175%' },
    { value: 2.0, label: '200%' },
  ];

  constructor(public connectionService: ConnectionService) {
    // Check initial devtools state
    this.checkDevtoolsState();

    // Load saved UI scale
    const savedScale = localStorage.getItem('saint-controller-ui-scale');
    if (savedScale) {
      this.uiScale = parseFloat(savedScale);
      this.applyScale(this.uiScale);
    }
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
    // Save config (without password)
    localStorage.setItem('saint-controller-config', JSON.stringify({
      host: this.config.host,
      port: this.config.port
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

  private applyScale(scale: number): void {
    // Apply scale to the root HTML element using CSS zoom
    // This scales the entire UI uniformly
    document.documentElement.style.zoom = scale.toString();
  }
}
