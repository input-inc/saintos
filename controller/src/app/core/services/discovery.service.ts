import { Injectable, signal, computed } from '@angular/core';
import { TauriService } from './tauri.service';
import { ConnectionService } from './connection.service';

/**
 * Represents a controllable function on a node
 */
export interface ControllableFunction {
  function: string;
  mode: string;  // 'pwm', 'servo', 'digital_out'
  gpio: number;  // For debugging only
}

/**
 * Represents a node with controllable functions
 */
export interface ControllableNode {
  role: string;
  node_id: string;
  display_name: string;
  online: boolean;
  functions: ControllableFunction[];
}

/**
 * Role definition from the server
 */
export interface RoleDefinition {
  name: string;
  display_name: string;
  description?: string;
  pins: Record<string, unknown>;
}

@Injectable({
  providedIn: 'root'
})
export class DiscoveryService {
  private _controllable = signal<ControllableNode[]>([]);
  private _roles = signal<RoleDefinition[]>([]);
  private _loading = signal(false);
  private _lastFetched = signal<Date | null>(null);

  /** All controllable nodes with their functions */
  readonly controllable = this._controllable.asReadonly();

  /** All role definitions */
  readonly roles = this._roles.asReadonly();

  /** Loading state */
  readonly loading = this._loading.asReadonly();

  /** Unique list of active roles (roles that have nodes assigned) */
  readonly activeRoles = computed(() => {
    const roles = new Set<string>();
    for (const node of this._controllable()) {
      roles.add(node.role);
    }
    return Array.from(roles).sort();
  });

  /** Get functions available for a specific role */
  getFunctionsForRole(role: string): string[] {
    const functions = new Set<string>();
    for (const node of this._controllable()) {
      if (node.role === role) {
        for (const fn of node.functions) {
          functions.add(fn.function);
        }
      }
    }
    return Array.from(functions).sort();
  }

  /** Get all unique functions across all roles */
  readonly allFunctions = computed(() => {
    const functions = new Set<string>();
    for (const node of this._controllable()) {
      for (const fn of node.functions) {
        functions.add(fn.function);
      }
    }
    return Array.from(functions).sort();
  });

  constructor(
    private tauri: TauriService,
    private connection: ConnectionService
  ) {
    console.log('[DiscoveryService] Initialized');

    // Listen for discovery responses from the server (forwarded by Rust backend)
    this.tauri.listen<{ controllable: ControllableNode[] }>('discovery-controllable').subscribe(data => {
      console.log('[DiscoveryService] Received discovery-controllable event:', data);
      if (data && data.controllable) {
        console.log('[DiscoveryService] Setting controllable data from server:',
          data.controllable.map(n => `${n.role}: ${n.functions?.map(f => f.function).join(', ') || 'no functions'}`));
        this._controllable.set(data.controllable);
        this._lastFetched.set(new Date());
      }
    });

    this.tauri.listen<{ roles: RoleDefinition[] }>('discovery-roles').subscribe(data => {
      console.log('[DiscoveryService] Received discovery-roles event:', data);
      if (data && data.roles) {
        this._roles.set(data.roles);
      }
    });

    // Auto-refresh when connection status changes to connected
    this.tauri.listen<{ status: string }>('connection-status').subscribe(state => {
      console.log('[DiscoveryService] Connection status event received:', state.status);
      if (state.status === 'connected') {
        console.log('[DiscoveryService] Connected! Will refresh discovery in 500ms...');
        // Small delay to let the connection stabilize
        setTimeout(() => this.refresh(), 500);
      }
    });
  }

  /**
   * Fetch discovery data from the server
   */
  async refresh(): Promise<void> {
    console.log('[DiscoveryService] refresh() called, isConnected:', this.connection.isConnected());

    if (!this.connection.isConnected()) {
      console.log('[DiscoveryService] Not connected, skipping refresh');
      return;
    }

    this._loading.set(true);
    console.log('[DiscoveryService] Starting discovery refresh...');

    try {
      // Request controllable functions from server
      // The response will come via the 'discovery-controllable' event (handled in constructor)
      console.log('[DiscoveryService] Calling discoverControllable...');
      await this.connection.discoverControllable();
      console.log('[DiscoveryService] discoverControllable request sent, waiting for server response via event...');

      // Also request roles
      console.log('[DiscoveryService] Calling discoverRoles...');
      await this.connection.discoverRoles();
      console.log('[DiscoveryService] discoverRoles request sent');

      // The actual data will arrive via Tauri events and be handled by the listeners in constructor
      // Set a timeout to set default data if server doesn't respond
      setTimeout(() => {
        if (this._controllable().length === 0) {
          console.warn('[DiscoveryService] No data received from server after 3s, using fallback defaults');
          this.setFallbackData();
        }
      }, 3000);

    } catch (err) {
      console.error('[DiscoveryService] Failed to refresh discovery data:', err);
      // Use fallback data on error
      this.setFallbackData();
    } finally {
      this._loading.set(false);
    }
  }

  /**
   * Set fallback data when server doesn't respond
   */
  private setFallbackData(): void {
    const fallbackData: ControllableNode[] = [
      {
        role: 'head',
        node_id: 'head-node',
        display_name: 'Head Controller',
        online: true,
        functions: [
          { function: 'pan', mode: 'servo', gpio: 0 },
          { function: 'tilt', mode: 'servo', gpio: 1 },
          { function: 'eye_left_lr', mode: 'servo', gpio: 2 },
          { function: 'eye_left_ud', mode: 'servo', gpio: 3 },
          { function: 'eye_right_lr', mode: 'servo', gpio: 4 },
          { function: 'eye_right_ud', mode: 'servo', gpio: 5 },
          { function: 'eyelid_left', mode: 'servo', gpio: 6 },
          { function: 'eyelid_right', mode: 'servo', gpio: 7 },
        ]
      },
      {
        role: 'tracks',
        node_id: 'tracks-node',
        display_name: 'Track Drive',
        online: true,
        functions: [
          { function: 'linear_velocity', mode: 'pwm', gpio: 0 },
          { function: 'angular_velocity', mode: 'pwm', gpio: 1 },
        ]
      },
      {
        role: 'sound',
        node_id: 'sound-node',
        display_name: 'Sound System',
        online: true,
        functions: [
          { function: 'play', mode: 'digital_out', gpio: 0 },
          { function: 'volume', mode: 'pwm', gpio: 1 },
        ]
      }
    ];

    console.log('[DiscoveryService] Setting fallback data:', fallbackData.map(n => `${n.role}: ${n.functions.map(f => f.function).join(', ')}`));
    this._controllable.set(fallbackData);
    this._lastFetched.set(new Date());
  }

  /**
   * Get display name for a role
   */
  getRoleDisplayName(role: string): string {
    const roleDef = this._roles().find(r => r.name === role);
    return roleDef?.display_name || role;
  }

  /**
   * Check if a role+function combination is valid
   */
  isValidTarget(role: string, functionName: string): boolean {
    for (const node of this._controllable()) {
      if (node.role === role) {
        for (const fn of node.functions) {
          if (fn.function === functionName) {
            return true;
          }
        }
      }
    }
    return false;
  }
}
