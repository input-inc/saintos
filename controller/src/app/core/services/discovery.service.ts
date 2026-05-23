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

/**
 * A single scalar channel inside a ROS topic message (flattened from
 * the message's field tree by the server's introspection pass).
 */
export interface TopicChannel {
  field: string;
  label: string;
  type?: string;
}

export interface TopicChannelTopic {
  topic: string;
  state_type?: string;
  channels: TopicChannel[];
}

/**
 * A WebSocket-input slot on a routing sheet. The new bindings picker
 * lists these (per-sheet) instead of raw ROS topic channels — a
 * gamepad axis writes into the slot, and the server-side routing
 * graph evaluator picks the value up and fans it onward through math
 * nodes / ROS outputs / peripheral channels.
 *
 * `sheet_label` is the owning node's display_name (e.g. "Track Drive
 * Right") so the picker doesn't have to surface raw node IDs.
 */
export interface WsInputSlot {
  sheet_id: string;
  sheet_label: string;
  input_id: string;
  label: string;
}

@Injectable({
  providedIn: 'root'
})
export class DiscoveryService {
  private _controllable = signal<ControllableNode[]>([]);
  private _roles = signal<RoleDefinition[]>([]);
  private _topics = signal<TopicChannelTopic[]>([]);
  private _wsInputs = signal<WsInputSlot[]>([]);
  private _loading = signal(false);
  private _lastFetched = signal<Date | null>(null);

  /** All controllable nodes with their functions */
  readonly controllable = this._controllable.asReadonly();

  /** All role definitions */
  readonly roles = this._roles.asReadonly();

  /** ROS topics with their scalar channels — drives the bindings
   *  Topic/Channel picker. */
  readonly topics = this._topics.asReadonly();

  /** Just the topic names, sorted, for the Topic dropdown. */
  readonly availableTopics = computed(() =>
    this._topics().map(t => t.topic).sort()
  );

  /** Channels available on a given topic. */
  getChannelsForTopic(topic: string): TopicChannel[] {
    return this._topics().find(t => t.topic === topic)?.channels ?? [];
  }

  /** WS-input slots declared across all routing sheets — the new
   *  bindings picker enumerates these in place of topic/channel. */
  readonly wsInputs = this._wsInputs.asReadonly();

  /** Unique sheets (with at least one WS input) as {id, label} pairs,
   *  sorted by label — that's what the binding picker presents. */
  readonly wsInputSheets = computed(() => {
    const byId = new Map<string, string>();
    for (const s of this._wsInputs()) {
      // Last-write-wins on label — server should be consistent across
      // all rows of the same sheet, so this is just a defensive merge.
      byId.set(s.sheet_id, s.sheet_label || s.sheet_id);
    }
    return Array.from(byId, ([id, label]) => ({ id, label }))
      .sort((a, b) => a.label.localeCompare(b.label));
  });

  /** WS inputs declared on a given sheet. */
  getWsInputsForSheet(sheetId: string): WsInputSlot[] {
    return this._wsInputs().filter(s => s.sheet_id === sheetId);
  }

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

    this.tauri.listen<{ topics: TopicChannelTopic[] }>('discovery-topic-channels').subscribe(data => {
      console.log('[DiscoveryService] Received discovery-topic-channels event:', data);
      if (data && Array.isArray(data.topics)) {
        this._topics.set(data.topics);
      }
    });

    this.tauri.listen<{ ws_inputs: WsInputSlot[] }>('discovery-ws-inputs').subscribe(data => {
      console.log('[DiscoveryService] Received discovery-ws-inputs event:', data);
      if (data && Array.isArray(data.ws_inputs)) {
        this._wsInputs.set(data.ws_inputs);
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

      // Topic/channel catalog — kept for legacy bindings authored
      // against the old picker; new bindings should use WS inputs.
      console.log('[DiscoveryService] Calling discoverTopicChannels...');
      try {
        await this.connection.discoverTopicChannels();
      } catch (err) {
        console.warn('[DiscoveryService] discoverTopicChannels failed:', err);
      }

      // WS-input slots from routing sheets — the new picker for bindings.
      console.log('[DiscoveryService] Calling discoverWsInputs...');
      try {
        await this.connection.discoverWsInputs();
      } catch (err) {
        console.warn('[DiscoveryService] discoverWsInputs failed:', err);
      }

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
