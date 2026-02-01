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
      // Fetch controllable functions
      console.log('[DiscoveryService] Calling discoverControllable...');
      await this.connection.discoverControllable();
      console.log('[DiscoveryService] discoverControllable request completed');

      // TODO: The server sends a response but we're not receiving it yet
      // For now, we set default test data
      // In production, we need to either:
      // 1. Have the Tauri command return the data directly
      // 2. Listen for a Tauri event with the discovery response

      // Temporary: Set some default controllable items for testing
      // These should match what's actually configured on your server
      const defaultData: ControllableNode[] = [
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

      console.log('[DiscoveryService] Setting default discovery data:', defaultData.map(n => `${n.role}: ${n.functions.map(f => f.function).join(', ')}`));
      this._controllable.set(defaultData);

      this._lastFetched.set(new Date());
      console.log('[DiscoveryService] Discovery data refreshed successfully');
      console.log('[DiscoveryService] Active roles:', this.activeRoles());
    } catch (err) {
      console.error('[DiscoveryService] Failed to refresh discovery data:', err);
    } finally {
      this._loading.set(false);
    }
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
