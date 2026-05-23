"""
SAINT.OS WebSocket Handler

Manages WebSocket client connections and message handling.
"""

import asyncio
import base64
import json
import os
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Set, Any, Optional, Callable, Awaitable

from aiohttp import web, WSMsgType

from saint_server.webserver.state_manager import StateManager
from saint_server.roles import RoleManager

# Type checking imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from saint_server.ros_bridge import ROSBridge


# Throttle settings
CONTROL_THROTTLE_MS = 50  # Minimum ms between control commands per node
NEUTRAL_EPSILON = 0.02    # Values within this range of 0 are considered neutral/stop


def is_neutral_value(value: float) -> bool:
    """
    Check if a value is at or near neutral (zero).

    Neutral/stop values should always bypass throttling to ensure
    motors stop immediately when the joystick is released.
    """
    return abs(value) <= NEUTRAL_EPSILON


def translate_normalized_value(value: float, mode: str, input_range: str = 'bipolar') -> float:
    """
    Translate a normalized value to native range based on pin mode.

    Controllers send normalized values:
    - Joysticks/bipolar: -1.0 to 1.0 (centered at 0)
    - Triggers/unipolar: 0.0 to 1.0

    Firmware expects:
    - PWM: 0-100 (percent)
    - Servo: 0-180 (degrees)
    - Digital: 0 or 1

    Args:
        value: Input value (-1.0 to 1.0 or 0.0 to 1.0)
        mode: Pin mode ('pwm', 'servo', 'digital_out', etc.)
        input_range: 'bipolar' (-1 to 1, default), 'unipolar' (0 to 1)

    Returns:
        Translated value in native range
    """
    # If value is already in native range (> 1 or < -1), pass through unchanged
    if value > 1.0 or value < -1.0:
        return value

    mode_lower = mode.lower() if mode else ''

    # Default to bipolar (joystick-style) for all normalized values
    # This ensures symmetric behavior: -1→min, 0→center, 1→max
    use_bipolar = (input_range != 'unipolar')

    if mode_lower == 'pwm':
        if use_bipolar:
            # Bipolar/joystick: -1→0, 0→50, 1→100
            return (value + 1.0) * 50.0
        else:
            # Unipolar/trigger: 0→0, 1→100
            return value * 100.0

    elif mode_lower == 'servo':
        if use_bipolar:
            # Bipolar/joystick: -1→0, 0→90, 1→180
            return (value + 1.0) * 90.0
        else:
            # Unipolar/trigger: 0→0, 1→180
            return value * 180.0

    elif mode_lower == 'maestro_servo':
        if use_bipolar:
            # Bipolar/joystick: -1→0, 0→90, 1→180
            return (value + 1.0) * 90.0
        else:
            # Unipolar/trigger: 0→0, 1→180
            return value * 180.0

    elif mode_lower == 'syren_motor':
        if use_bipolar:
            # Bipolar/joystick: -1→-100, 0→0, 1→100
            return value * 100.0
        else:
            # Unipolar/trigger: 0→-100, 0.5→0, 1→100
            return value * 200.0 - 100.0

    elif mode_lower == 'pathfinder_bms_sensor':
        return value

    elif mode_lower == 'digital_out':
        # Threshold at 0.5
        return 1.0 if value >= 0.5 else 0.0

    # Unknown mode - pass through
    return value


@dataclass
class WebSocketClient:
    """Represents a connected WebSocket client."""
    id: str
    ws: web.WebSocketResponse
    subscriptions: Set[str] = field(default_factory=set)
    connected_at: float = field(default_factory=time.time)
    authenticated: bool = False  # True if auth succeeded or auth not required
    ip_address: str = ""  # Client's IP address
    user_agent: str = ""  # Client's User-Agent header
    client_name: str = ""  # Optional client-provided name
    terminal_session: Optional[Any] = None  # TerminalSession when one is open


class WebSocketHandler:
    """Handles WebSocket connections and messages."""

    def __init__(self, state_manager: StateManager, logger=None):
        self.state_manager = state_manager
        self.logger = logger
        self.clients: Dict[str, WebSocketClient] = {}
        self._lock = asyncio.Lock()
        self._broadcast_task: Optional[asyncio.Task] = None

        # Role manager for pin configuration
        self.role_manager = RoleManager(logger=logger)

        # Callback for syncing config to nodes (set by server_node)
        self._sync_config_callback: Optional[Callable[[str, str], None]] = None

        # Callback for sending control commands to nodes (set by server_node)
        self._send_control_callback: Optional[Callable[[str, int, float], None]] = None

        # Callback for sending channel-addressed control to nodes (set by server_node).
        # Signature: (node_id, peripheral_id, channel_id, value, peripheral_type) -> None.
        # peripheral_type is the catalog type (e.g. "neopixel", "roboclaw")
        # — empty string if unknown. Firmware uses it to route writes for
        # peripherals that don't live in pin_config (the built-in status
        # NeoPixel is the canonical case).
        self._send_channel_callback: Optional[
            Callable[[str, str, str, float, str], None]] = None

        # Callback for triggering firmware update (set by server_node)
        # Signature: (node_id: str, simulation: bool) -> None
        self._firmware_update_callback: Optional[Callable[[str, bool, bool], None]] = None

        # Callback for sending factory reset to node (set by server_node)
        self._factory_reset_callback: Optional[Callable[[str], None]] = None

        # Callback for sending restart command to node (set by server_node)
        self._restart_node_callback: Optional[Callable[[str], None]] = None

        # Callback for sending identify command to node (set by server_node)
        self._identify_node_callback: Optional[Callable[[str], None]] = None

        # Callback for sending estop command to node (set by server_node).
        # Signature: (node_id: str, action: str) -> None where action is
        # "engage" (assert e-stop / drive estop_pin HIGH / cut motor
        # outputs) or "release" (clear the latch).
        self._estop_node_callback: Optional[Callable[[str, str], None]] = None

        # Callback for sending a RoboClaw debug passthrough to a node
        # (set by server_node). Payload is a dict matching the firmware's
        # roboclaw_debug_handle_json format.
        self._roboclaw_debug_callback: Optional[Callable[[str, dict], None]] = None

        # System-wide E-Stop latch state. Mutated by 'estop' command
        # handlers; broadcast to subscribers on the 'estop' topic so
        # multiple dashboards stay in sync.
        self._estop_active: bool = False
        self._estop_changed_at: float = 0.0

        # Throttle tracking: (node_id, gpio) -> last_send_time
        # Per-gpio throttling allows controlling multiple pins simultaneously
        self._control_throttle: Dict[tuple, float] = {}

        # LiveLink callbacks (set by server_node)
        self._livelink_get_status: Optional[Callable[[], Dict[str, Any]]] = None
        self._livelink_get_routes: Optional[Callable[[], Any]] = None
        self._livelink_set_routes: Optional[Callable[[Any], None]] = None
        self._livelink_enable_route: Optional[Callable[[str, bool], None]] = None
        self._livelink_create_default_route: Optional[Callable[[str], Dict[str, Any]]] = None

        # ROS Bridge (set by server_node)
        self._ros_bridge: Optional['ROSBridge'] = None

        # Update manager (set by server_node)
        self._update_manager = None

        # Authentication settings (loaded from config)
        self._auth_password: Optional[str] = None
        self._auth_timeout: float = 10.0
        self._load_auth_config()

    def _load_auth_config(self):
        """Load authentication settings from server config."""
        try:
            from saint_server.config import get_config
            config = get_config()
            self._auth_password = config.websocket.password
            self._auth_timeout = config.websocket.auth_timeout
            if self._auth_password:
                self.log('info', 'WebSocket authentication enabled')
        except Exception as e:
            self.log('warn', f'Could not load auth config: {e}')

    def set_sync_config_callback(self, callback: Callable[[str, str], None]):
        """Set callback for syncing config to nodes. Callback takes (node_id, config_json)."""
        self._sync_config_callback = callback

    def set_request_capabilities_callback(self, callback: Callable[[str], None]):
        """Set callback for requesting capabilities from nodes. Callback takes (node_id)."""
        self._request_capabilities_callback = callback

    def set_send_control_callback(self, callback: Callable[[str, int, float], None]):
        """Set callback for sending control commands to nodes. Callback takes (node_id, gpio, value)."""
        self._send_control_callback = callback

    def set_send_channel_callback(
            self, callback: Callable[[str, str, str, float, str], None]):
        """Set callback for channel-addressed control.
        Takes (node_id, peripheral_id, channel_id, value, peripheral_type)."""
        self._send_channel_callback = callback

    def set_firmware_update_callback(self, callback: Callable[[str, bool, bool], None]):
        """Set callback for triggering firmware update. Callback takes (node_id, simulation, force)."""
        self._firmware_update_callback = callback

    def set_factory_reset_callback(self, callback: Callable[[str], None]):
        """Set callback for sending factory reset to node. Callback takes (node_id)."""
        self._factory_reset_callback = callback

    def set_restart_node_callback(self, callback: Callable[[str], None]):
        """Set callback for sending restart command to node. Callback takes (node_id)."""
        self._restart_node_callback = callback

    def set_identify_node_callback(self, callback: Callable[[str], None]):
        """Set callback for sending identify command to node. Callback takes (node_id)."""
        self._identify_node_callback = callback

    def set_estop_node_callback(self, callback: Callable[[str, str], None]):
        """Set callback for sending emergency stop to a node.
        Callback takes (node_id, action) where action is 'engage' or 'release'."""
        self._estop_node_callback = callback

    def set_roboclaw_debug_callback(self, callback: Callable[[str, dict], None]):
        """Set callback for forwarding a RoboClaw debug payload to a node.
        Callback takes (node_id, payload_dict). The payload is the full
        debug message (op + op-specific fields) which server_node will
        wrap in the control envelope and publish."""
        self._roboclaw_debug_callback = callback

    def set_update_manager(self, update_manager):
        """Wire the update manager and subscribe to state changes."""
        self._update_manager = update_manager
        # Broadcast state changes to subscribed clients on the 'update' topic.
        loop = asyncio.get_event_loop()

        def _on_change(state):
            payload = state.to_dict()
            try:
                loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(self.broadcast_state('update', payload))
                )
            except RuntimeError:
                # Event loop may not be running yet during early init.
                pass

        update_manager.add_listener(_on_change)

    def set_livelink_callbacks(
        self,
        get_status: Callable[[], Dict[str, Any]],
        get_routes: Callable[[], Any],
        set_routes: Callable[[Any], None],
        enable_route: Callable[[str, bool], None],
        create_default_route: Callable[[str], Dict[str, Any]],
    ):
        """Set callbacks for LiveLink management."""
        self._livelink_get_status = get_status
        self._livelink_get_routes = get_routes
        self._livelink_set_routes = set_routes
        self._livelink_enable_route = enable_route
        self._livelink_create_default_route = create_default_route

    def set_ros_bridge(self, bridge: 'ROSBridge'):
        """Set the ROS bridge for topic subscriptions/publications."""
        self._ros_bridge = bridge
        bridge.set_message_callback(self._on_ros_message)

    def _on_ros_message(self, topic: str, data: Dict[str, Any]):
        """Callback when ROS message received - broadcasts to WebSocket clients."""
        # This will be called from ROS thread, need to schedule in async loop
        # The actual broadcast is handled by the caller (server_node)
        pass

    def log(self, level: str, message: str):
        """Log a message if logger is available.

        Note: Each level must be called from a different line because ROS2's
        rcutils logger tracks calls by file/line and rejects severity changes
        from the same call site.
        """
        if not self.logger:
            return
        if level == 'debug':
            self.logger.debug(message)
        elif level == 'info':
            self.logger.info(message)
        elif level == 'warn' or level == 'warning':
            self.logger.warning(message)
        elif level == 'error':
            self.logger.error(message)
        elif level == 'fatal':
            self.logger.fatal(message)
        else:
            self.logger.info(message)

    async def handle_connection(self, request: web.Request) -> web.WebSocketResponse:
        """Handle a new WebSocket connection."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        client_id = str(uuid.uuid4())[:8]
        auth_required = self._auth_password is not None

        # Get client IP address (check X-Forwarded-For for reverse proxy setups)
        forwarded_for = request.headers.get('X-Forwarded-For')
        if forwarded_for:
            ip_address = forwarded_for.split(',')[0].strip()
        else:
            peername = request.transport.get_extra_info('peername')
            ip_address = peername[0] if peername else 'unknown'

        # Get user agent
        user_agent = request.headers.get('User-Agent', 'unknown')

        client = WebSocketClient(
            id=client_id,
            ws=ws,
            authenticated=not auth_required,  # Auto-auth if no password set
            ip_address=ip_address,
            user_agent=user_agent,
        )

        async with self._lock:
            self.clients[client_id] = client
            self.state_manager.set_client_count(len(self.clients))

        self.log('info', f'WebSocket client connected: {client_id} from {ip_address}')

        # Send connected confirmation with auth requirement
        await self._send_to_client(client, {
            "type": "connected",
            "client_id": client_id,
            "server_name": self.state_manager.state.server_name,
            "auth_required": auth_required,
        })

        # Start auth timeout if authentication is required
        auth_timeout_task = None
        if auth_required:
            auth_timeout_task = asyncio.create_task(
                self._auth_timeout_handler(client)
            )

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self._handle_message(client, msg.data)
                elif msg.type == WSMsgType.ERROR:
                    self.log('error', f'WebSocket error: {ws.exception()}')
                    break
        finally:
            # Cancel auth timeout task if still running
            if auth_timeout_task and not auth_timeout_task.done():
                auth_timeout_task.cancel()

            async with self._lock:
                self.clients.pop(client_id, None)
                self.state_manager.set_client_count(len(self.clients))

            # Clean up ROS subscriptions for this client
            if self._ros_bridge:
                self._ros_bridge.client_disconnected(client_id)

            # Tear down any open terminal session — never leak shells.
            if client.terminal_session is not None:
                try:
                    await client.terminal_session.close()
                except Exception:
                    self.log('warn', f'terminal cleanup error for {client_id}')
                client.terminal_session = None

            self.log('info', f'WebSocket client disconnected: {client_id}')

        return ws

    async def _auth_timeout_handler(self, client: WebSocketClient):
        """Disconnect client if not authenticated within timeout."""
        await asyncio.sleep(self._auth_timeout)
        if not client.authenticated:
            self.log('warn', f'Client {client.id} auth timeout - disconnecting')
            await self._send_to_client(client, {
                "type": "auth_failed",
                "message": "Authentication timeout",
            })
            await client.ws.close()

    async def _send_to_client(self, client: WebSocketClient, message: dict):
        """Send a message to a specific client."""
        try:
            await client.ws.send_json(message)
        except Exception as e:
            self.log('error', f'Failed to send to client {client.id}: {e}')

    async def _handle_auth(self, client: WebSocketClient, message: dict):
        """Handle authentication message from client."""
        action = message.get('action', 'login')

        if action == 'login':
            await self._handle_auth_login(client, message)
        elif action == 'change_password':
            await self._handle_auth_change_password(client, message)
        else:
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "error",
                "message": f"Unknown auth action: {action}",
            })

    async def _handle_auth_login(self, client: WebSocketClient, message: dict):
        """Handle login authentication."""
        if client.authenticated:
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "ok",
                "message": "Already authenticated",
            })
            return

        password = message.get('password')

        # Convert to string for comparison (handle int passwords from YAML)
        if self._auth_password is not None:
            auth_pass_str = str(self._auth_password)
        else:
            auth_pass_str = None

        if auth_pass_str is None or str(password) == auth_pass_str:
            client.authenticated = True
            self.log('info', f'Client {client.id} authenticated successfully')
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "ok",
                "message": "Authentication successful",
            })
        else:
            self.log('warn', f'Client {client.id} authentication failed')
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "error",
                "message": "Invalid password",
            })
            # Disconnect after failed auth
            await client.ws.close()

    async def _handle_auth_change_password(self, client: WebSocketClient, message: dict):
        """Handle password change request."""
        # Must be authenticated to change password
        if not client.authenticated:
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "error",
                "message": "Must be authenticated to change password",
            })
            return

        new_password = message.get('new_password')

        if new_password is None:
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "error",
                "message": "Missing new_password field",
            })
            return

        # Update in-memory password
        old_had_password = self._auth_password is not None
        self._auth_password = new_password if new_password != "" else None

        # Save to config file
        try:
            self._save_password_to_config(new_password if new_password != "" else None)
            self.log('info', f'Client {client.id} changed server password')

            # Notify about auth requirement change
            auth_now_required = self._auth_password is not None
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "ok",
                "message": "Password changed successfully",
                "data": {
                    "auth_required": auth_now_required,
                }
            })

            # Broadcast to all clients that auth requirement may have changed
            await self.broadcast_activity(
                f'Server password {"updated" if auth_now_required else "removed"}',
                'info'
            )

        except Exception as e:
            self.log('error', f'Failed to save password: {e}')
            # Revert in-memory change
            self._auth_password = self._auth_password if old_had_password else None
            await self._send_to_client(client, {
                "type": "auth_result",
                "status": "error",
                "message": f"Failed to save password: {str(e)}",
            })

    def _save_password_to_config(self, new_password: Optional[str]):
        """Save the new password to server_config.yaml."""
        import os
        import yaml

        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'server_config.yaml'
        )

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        # Read current config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f) or {}

        # Update password
        if 'websocket' not in config:
            config['websocket'] = {}
        config['websocket']['password'] = new_password

        # Write back
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    async def _handle_message(self, client: WebSocketClient, data: str):
        """Handle an incoming message from a client."""
        try:
            message = json.loads(data)
        except json.JSONDecodeError:
            await self._send_to_client(client, {
                "status": "error",
                "message": "Invalid JSON",
            })
            return

        # Log incoming messages (excluding high-frequency control messages at debug level)
        msg_type = message.get('type')
        msg_action = message.get('action')
        if msg_type == 'control':
            # Log control messages at debug level to avoid spam
            self.log('debug', f'[WS] Control: {msg_action} params={message.get("params")}')
        elif msg_type != 'auth':
            # Log other messages at info level
            self.log('info', f'[WS] Received: type={msg_type} action={msg_action}')

        # Handle authentication message
        if msg_type == 'auth':
            await self._handle_auth(client, message)
            return

        # Require authentication for all other messages
        if not client.authenticated:
            await self._send_to_client(client, {
                "id": message.get('id'),
                "status": "error",
                "message": "Authentication required. Send {\"type\": \"auth\", \"password\": \"...\"} first.",
            })
            return

        # Validate message format
        msg_id = message.get('id')
        msg_type = message.get('type')
        action = message.get('action')
        params = message.get('params', {})

        if not msg_id:
            await self._send_to_client(client, {
                "status": "error",
                "message": "Missing message id",
            })
            return

        if not msg_type:
            await self._send_to_client(client, {
                "id": msg_id,
                "status": "error",
                "message": "Missing message type",
            })
            return

        # Route to handler
        handler = {
            'command': self._handle_command,
            'management': self._handle_management,
            'subscribe': self._handle_subscribe,
            'router': self._handle_router,
            'control': self._handle_control,
            'discovery': self._handle_discovery,
            'livelink': self._handle_livelink,
            'ros': self._handle_ros,
            'file': self._handle_file,
        }.get(msg_type)

        if handler:
            try:
                response = await handler(client, action, params)
                response['id'] = msg_id
                await self._send_to_client(client, response)
            except Exception as e:
                self.log('error', f'Handler error for {msg_type}/{action}: {e}')
                await self._send_to_client(client, {
                    "id": msg_id,
                    "status": "error",
                    "message": f"Internal error: {str(e)}",
                })
        else:
            await self._send_to_client(client, {
                "id": msg_id,
                "status": "error",
                "message": f"Unknown message type: {msg_type}",
            })

    async def _handle_command(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle command messages."""
        if action == 'estop':
            # Latching toggle. Each press flips the system-wide state.
            # An optional explicit `state` parameter ("engage"/"release")
            # lets the client force a particular direction — useful for
            # bringing a fresh dashboard into sync without an extra
            # round-trip (clicking "release" on an already-released
            # latch is a no-op rather than re-engaging).
            requested = params.get('state')
            if requested == 'engage':
                new_state = True
            elif requested == 'release':
                new_state = False
            else:
                new_state = not self._estop_active

            target = params.get('target', 'all')
            verb = 'ENGAGED' if new_state else 'RELEASED'
            self.log('warn', f'E-STOP {verb} by client {client.id}, target: {target}')

            # Fan out to every adopted node. We do this before mutating
            # state and broadcasting so any partial failure surfaces
            # immediately in the activity log.
            engaged_count = 0
            errored = []
            if self._estop_node_callback:
                adopted = self.state_manager.get_adopted_nodes()
                wire_action = 'engage' if new_state else 'release'
                for node in adopted:
                    nid = node.get('node_id')
                    if not nid:
                        continue
                    try:
                        self._estop_node_callback(nid, wire_action)
                        engaged_count += 1
                    except Exception as e:
                        errored.append(f'{nid}: {e}')
            else:
                return {"status": "error",
                        "message": "Estop callback not configured"}

            self._estop_active = new_state
            self._estop_changed_at = time.time()
            await self.broadcast_state('estop', {
                'active': new_state,
                'changed_at': self._estop_changed_at,
                'node_count': engaged_count,
            })
            if errored:
                await self.broadcast_activity(
                    f'E-STOP {verb}: command failed on {len(errored)} node(s) — '
                    + '; '.join(errored), 'warning')
            else:
                await self.broadcast_activity(
                    f'E-STOP {verb} on {engaged_count} node(s)', 'warning')
            return {"status": "ok", "data": {
                "active": new_state,
                "node_count": engaged_count,
            }}
        else:
            return {"status": "error", "message": f"Unknown command action: {action}"}

    async def _handle_management(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle management messages."""
        if action == 'list_adopted':
            nodes = self.state_manager.get_adopted_nodes()
            return {"status": "ok", "data": {"nodes": nodes}}

        elif action == 'list_unadopted':
            nodes = self.state_manager.get_unadopted_nodes()
            return {"status": "ok", "data": {"nodes": nodes}}

        elif action == 'list_clients':
            # Return list of connected WebSocket clients
            clients_list = []
            async with self._lock:
                for c in self.clients.values():
                    clients_list.append({
                        "id": c.id,
                        "ip_address": c.ip_address,
                        "user_agent": c.user_agent,
                        "client_name": c.client_name,
                        "connected_at": c.connected_at,
                        "authenticated": c.authenticated,
                        "is_self": c.id == client.id,
                    })
            return {"status": "ok", "data": {"clients": clients_list}}

        elif action == 'disconnect_client':
            target_client_id = params.get('client_id')
            if not target_client_id:
                return {"status": "error", "message": "Missing client_id"}

            # Don't allow disconnecting yourself
            if target_client_id == client.id:
                return {"status": "error", "message": "Cannot disconnect yourself"}

            async with self._lock:
                target_client = self.clients.get(target_client_id)
                if not target_client:
                    return {"status": "error", "message": "Client not found"}

                # Send disconnect message to target client
                await self._send_to_client(target_client, {
                    "type": "disconnected",
                    "reason": "Disconnected by administrator",
                })
                # Close the connection
                await target_client.ws.close()

            self.log('info', f'Client {target_client_id} disconnected by {client.id}')
            await self.broadcast_activity(f'Client {target_client_id} disconnected by administrator', 'info')
            return {"status": "ok", "message": f"Client {target_client_id} disconnected"}

        elif action == 'adopt_node':
            node_id = params.get('node_id')
            role = params.get('role')
            display_name = params.get('display_name')
            board_id = params.get('board_id')   # new: operator picks board on adoption
            chip_family = params.get('chip_family') or None  # operator override

            if not node_id or not role:
                return {"status": "error", "message": "Missing node_id or role"}

            result = self.state_manager.adopt_node(
                node_id, role, display_name, board_id, chip_family=chip_family)
            if result['success']:
                board_note = f", board {result.get('board_id')}" if result.get('board_id') else ""
                await self.broadcast_activity(
                    f'Node {node_id} adopted as {role}{board_note}', 'info'
                )
            return {"status": "ok" if result['success'] else "error", "data": result}

        elif action == 'update_node':
            # Edit role / board / chip / display_name on an already-
            # adopted node. Mirror of adopt_node for post-adoption
            # edits. All fields optional; only the ones supplied get
            # changed. Same broadcast semantics so the activity log
            # records the change for everyone.
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            result = self.state_manager.update_node(
                node_id,
                role=params.get('role'),
                display_name=params.get('display_name'),
                board_id=params.get('board_id'),
                chip_family=params.get('chip_family'),
            )
            if result.get('success'):
                await self.broadcast_activity(
                    f'Node {node_id} updated', 'info'
                )
            return {"status": "ok" if result['success'] else "error", "data": result}

        elif action == 'list_chips':
            return {"status": "ok", "data": {"chips": self.state_manager.list_chips()}}

        elif action == 'list_boards':
            chip_family = params.get('chip_family')
            return {"status": "ok", "data": {
                "boards": self.state_manager.list_boards(chip_family)
            }}

        elif action == 'set_node_board':
            node_id = params.get('node_id')
            board_id = params.get('board_id')
            if not node_id or not board_id:
                return {"status": "error", "message": "Missing node_id or board_id"}
            result = self.state_manager.set_node_board(node_id, board_id)
            return {"status": "ok", "data": result}

        elif action == 'get_board_yaml':
            board_id = params.get('board_id')
            if not board_id:
                return {"status": "error", "message": "Missing board_id"}
            text = self.state_manager.get_board_yaml(board_id)
            if text is None:
                return {"status": "error", "message": f"Unknown board '{board_id}'"}
            return {"status": "ok", "data": {"board_id": board_id, "yaml": text}}

        elif action == 'save_board':
            yaml_text = params.get('yaml')
            if not yaml_text:
                return {"status": "error", "message": "Missing yaml"}
            result = self.state_manager.save_board_yaml(yaml_text)
            return {"status": "ok", "data": result}

        elif action == 'delete_board':
            board_id = params.get('board_id')
            if not board_id:
                return {"status": "error", "message": "Missing board_id"}
            result = self.state_manager.delete_board(board_id)
            return {"status": "ok", "data": result}

        elif action == 'reset_node':
            node_id = params.get('node_id')
            factory_reset = params.get('factory_reset', False)

            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            # Send factory reset command to node first (if doing factory reset)
            if factory_reset and self._factory_reset_callback:
                self._factory_reset_callback(node_id)

            result = self.state_manager.reset_node(node_id, factory_reset)
            if result['success']:
                await self.broadcast_activity(f'Node {node_id} reset', 'info')
            return {"status": "ok" if result['success'] else "error", "data": result}

        elif action == 'remove_node':
            node_id = params.get('node_id')

            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            # Send factory reset command to node first so it clears its config
            if self._factory_reset_callback:
                self._factory_reset_callback(node_id)

            result = self.state_manager.remove_node(node_id)
            if result['success']:
                await self.broadcast_activity(f'Node {node_id} removed', 'info')
            return {"status": "ok" if result['success'] else "error", "data": result}

        elif action == 'restart_node':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            if self._restart_node_callback:
                self._restart_node_callback(node_id)
                await self.broadcast_activity(f'Restart command sent to node {node_id}', 'info')
                return {"status": "ok", "data": {"message": "Restart command sent"}}
            else:
                return {"status": "error", "message": "Restart callback not configured"}

        elif action == 'identify_node':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            if self._identify_node_callback:
                self._identify_node_callback(node_id)
                await self.broadcast_activity(f'Identify command sent to node {node_id}', 'info')
                return {"status": "ok", "data": {"message": "Identify command sent"}}
            else:
                return {"status": "error", "message": "Identify callback not configured"}

        elif action == 'estop_node':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            # Per-node estop also uses engage/release. Default to engage
            # for backwards-compatible callers that just want to stop one
            # node without flipping the system-wide latch.
            wire_action = params.get('state', 'engage')
            if wire_action not in ('engage', 'release'):
                return {"status": "error",
                        "message": "state must be 'engage' or 'release'"}

            if self._estop_node_callback:
                self._estop_node_callback(node_id, wire_action)
                verb = 'engaged' if wire_action == 'engage' else 'released'
                await self.broadcast_activity(
                    f'Emergency stop {verb} on node {node_id}', 'warning')
                return {"status": "ok", "data": {"message": f"Emergency stop {verb}"}}
            else:
                return {"status": "error", "message": "Estop callback not configured"}

        elif action == 'get_estop_state':
            return {"status": "ok", "data": {
                "active": self._estop_active,
                "changed_at": self._estop_changed_at,
            }}

        elif action == 'roboclaw_debug':
            # Diagnostic passthrough to the RoboClaw driver on a node.
            # The payload (op + op-specific keys) is forwarded verbatim
            # to the firmware; the response comes back over the normal
            # log stream as a "roboclaw_dbg:" entry the caller can watch.
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            op = params.get('op')
            if op not in ('raw', 'sniff', 'reconfigure'):
                return {"status": "error",
                        "message": "op must be raw|sniff|reconfigure"}
            if not self._roboclaw_debug_callback:
                return {"status": "error",
                        "message": "RoboClaw debug callback not configured"}
            # Only forward whitelisted keys to keep the wire payload tight.
            payload = {'op': op}
            for k in ('tx_hex', 'read_len', 'timeout_ms',
                      'duration_ms', 'baud', 'swap'):
                if k in params:
                    payload[k] = params[k]
            self._roboclaw_debug_callback(node_id, payload)
            return {"status": "ok", "data": {"message": f"roboclaw_debug {op} dispatched"}}

        # Pin Configuration Actions
        elif action == 'get_roles':
            roles = self.role_manager.get_roles_summary()
            return {"status": "ok", "data": {"roles": roles}}

        elif action == 'get_role_definition':
            role_name = params.get('role')
            if not role_name:
                return {"status": "error", "message": "Missing role parameter"}

            role_def = self.role_manager.get_role(role_name)
            if not role_def:
                return {"status": "error", "message": f"Unknown role: {role_name}"}

            return {"status": "ok", "data": role_def.to_dict()}

        elif action == 'get_node_capabilities':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            capabilities = self.state_manager.get_node_capabilities(node_id)
            if capabilities:
                return {"status": "ok", "data": capabilities}
            else:
                return {"status": "ok", "data": None, "message": "Capabilities not available"}

        elif action == 'request_node_capabilities':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            if hasattr(self, '_request_capabilities_callback') and self._request_capabilities_callback:
                self._request_capabilities_callback(node_id)
                return {"status": "ok", "message": "Capabilities request sent"}
            else:
                return {"status": "error", "message": "Capabilities request not available"}

        # ─── Peripheral catalog (types of peripherals known to the system) ──
        elif action == 'get_peripheral_catalog':
            return {"status": "ok", "data": {
                "peripheral_types": self.state_manager.get_peripheral_catalog(),
                "widget_types": self.state_manager.get_widget_catalog(),
                "operator_types": self.state_manager.get_operator_catalog(),
            }}

        # ─── Per-node peripheral configuration ──────────────────────────────
        elif action == 'get_node_peripherals':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            data = self.state_manager.get_node_peripherals(node_id)
            if data is None:
                return {"status": "error", "message": f"Node {node_id} not found"}
            return {"status": "ok", "data": data}

        elif action == 'save_node_peripheral':
            node_id = params.get('node_id')
            peripheral = params.get('peripheral')
            if not node_id or not peripheral:
                return {"status": "error", "message": "Missing node_id or peripheral"}
            result = self.state_manager.upsert_node_peripheral(node_id, peripheral)
            return {"status": "ok", "data": result}

        elif action == 'remove_node_peripheral':
            node_id = params.get('node_id')
            peripheral_id = params.get('peripheral_id')
            if not node_id or not peripheral_id:
                return {"status": "error", "message": "Missing node_id or peripheral_id"}
            result = self.state_manager.remove_node_peripheral(node_id, peripheral_id)
            return {"status": "ok", "data": result}

        elif action == 'set_peripheral_log_enabled':
            node_id = params.get('node_id')
            peripheral_id = params.get('peripheral_id')
            enabled = params.get('enabled')
            if not node_id or not peripheral_id or enabled is None:
                return {"status": "error",
                        "message": "Missing node_id, peripheral_id, or enabled"}
            result = self.state_manager.set_peripheral_log_enabled(
                node_id, peripheral_id, bool(enabled))
            return {"status": "ok", "data": result}

        elif action == 'get_peripheral_history':
            node_id = params.get('node_id')
            peripheral_id = params.get('peripheral_id')
            channel_id = params.get('channel_id')
            window = params.get('window', '30s')
            if not node_id or not peripheral_id or not channel_id:
                return {"status": "error",
                        "message": "Missing node_id, peripheral_id, or channel_id"}
            if window not in ('30s', '60s'):
                return {"status": "error", "message": "window must be '30s' or '60s'"}
            plog = self.state_manager.peripheral_logger
            samples = (plog.history(node_id, peripheral_id, channel_id, window)
                       if plog is not None else [])
            return {"status": "ok", "data": {
                "node_id": node_id,
                "peripheral_id": peripheral_id,
                "channel_id": channel_id,
                "window": window,
                # Compact wire format — same shape as the NDJSON file lines.
                "samples": [{"ts": ts, "v": v} for ts, v in samples],
            }}

        elif action == 'get_node_logs':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            entries = self.state_manager.get_node_logs(node_id)
            return {"status": "ok", "data": {"entries": entries}}

        elif action == 'clear_node_logs':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            self.state_manager.clear_node_logs(node_id)
            return {"status": "ok", "data": {"success": True}}

        elif action == 'sync_node_peripherals':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            config_json = self.state_manager.get_firmware_config_json(node_id)
            if not config_json:
                return {"status": "error", "message": "No peripheral configuration to sync"}
            if self._sync_config_callback:
                self._sync_config_callback(node_id, config_json)
                await self.broadcast_activity(f'Syncing peripherals to node {node_id}', 'info')
                return {"status": "ok", "data": {"message": "Sync initiated", "success": True}}
            return {"status": "error", "message": "Config sync not available"}

        # ─── System routing (sheets + widgets) ──────────────────────────────
        elif action == 'get_system_routing':
            return {"status": "ok", "data": self.state_manager.get_system_routing()}

        elif action == 'get_routing_sheet':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            return {"status": "ok",
                    "data": self.state_manager.get_routing_sheet(node_id)}

        elif action == 'add_routing_input':
            node_id = params.get('node_id')
            topic = params.get('topic')
            if not node_id or not topic:
                return {"status": "error", "message": "Missing node_id or topic"}
            result = self.state_manager.add_routing_input(
                node_id=node_id,
                topic=topic,
                field=params.get('field', ''),
                label=params.get('label', ''),
                position=params.get('position'),
            )
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'add_routing_operator':
            node_id = params.get('node_id')
            op = params.get('op')
            if not node_id or not op:
                return {"status": "error", "message": "Missing node_id or op"}
            result = self.state_manager.add_routing_operator(
                node_id=node_id,
                op=op,
                label=params.get('label', ''),
                params=params.get('params'),
                defaults=params.get('defaults'),
                position=params.get('position'),
            )
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'add_routing_wire':
            node_id = params.get('node_id')
            source = params.get('source')
            sink = params.get('sink')
            if not node_id or not source or not sink:
                return {"status": "error", "message": "Missing node_id, source, or sink"}
            result = self.state_manager.add_routing_wire(node_id, source, sink)
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'update_sheet_node':
            node_id = params.get('node_id')
            sheet_node_id = params.get('sheet_node_id')
            if not node_id or not sheet_node_id:
                return {"status": "error", "message": "Missing node_id or sheet_node_id"}
            changes = {k: v for k, v in params.items()
                       if k in ('label', 'params', 'defaults', 'position',
                                'topic', 'field')}
            result = self.state_manager.update_sheet_node(node_id, sheet_node_id, **changes)
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'remove_sheet_node':
            node_id = params.get('node_id')
            sheet_node_id = params.get('sheet_node_id')
            if not node_id or not sheet_node_id:
                return {"status": "error", "message": "Missing node_id or sheet_node_id"}
            result = self.state_manager.remove_sheet_node(node_id, sheet_node_id)
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'remove_routing_wire':
            node_id = params.get('node_id')
            wire_id = params.get('wire_id')
            if not node_id or not wire_id:
                return {"status": "error", "message": "Missing node_id or wire_id"}
            result = self.state_manager.remove_routing_wire(node_id, wire_id)
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'add_routing_output':
            node_id = params.get('node_id')
            topic = params.get('topic')
            if not node_id or not topic:
                return {"status": "error", "message": "Missing node_id or topic"}
            result = self.state_manager.add_routing_output(
                node_id=node_id,
                topic=topic,
                field=params.get('field', ''),
                label=params.get('label', ''),
                position=params.get('position'),
            )
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'add_routing_ws_input':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            result = self.state_manager.add_routing_ws_input(
                node_id=node_id,
                label=params.get('label', ''),
                position=params.get('position'),
            )
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'add_widget':
            node_id = params.get('node_id')
            type_id = params.get('type')
            if not node_id or not type_id:
                return {"status": "error", "message": "Missing node_id or type"}
            result = self.state_manager.add_widget(
                node_id=node_id,
                type_id=type_id,
                label=params.get('label'),
                position=params.get('position'),
                params=params.get('params'),
            )
            await self._broadcast_routing()
            return {"status": "ok", "data": result}

        elif action == 'get_logs':
            limit = params.get('limit', 100)
            level = params.get('level')
            logs = self.state_manager.get_logs(limit=limit, level=level)
            return {"status": "ok", "data": {"logs": logs}}

        # Software Update Actions ---------------------------------------------
        elif action == 'update.get_status':
            if not self._update_manager:
                return {"status": "error", "message": "Update manager not available"}
            return {"status": "ok", "data": self._update_manager.state.to_dict()}

        elif action == 'update.check_now':
            if not self._update_manager:
                return {"status": "error", "message": "Update manager not available"}
            state = await self._update_manager.check_github()
            return {"status": "ok", "data": state.to_dict()}

        elif action == 'update.scan_usb':
            if not self._update_manager:
                return {"status": "error", "message": "Update manager not available"}
            state = await self._update_manager.scan_usb()
            return {"status": "ok", "data": state.to_dict()}

        elif action == 'update.download':
            if not self._update_manager:
                return {"status": "error", "message": "Update manager not available"}
            # Run as a background task so the response can return immediately
            # while download progress streams over the broadcast channel.
            asyncio.create_task(self._update_manager.download_github_release())
            return {"status": "ok", "data": self._update_manager.state.to_dict()}

        elif action == 'update.stage_usb':
            if not self._update_manager:
                return {"status": "error", "message": "Update manager not available"}
            version = params.get('version')
            if not version:
                return {"status": "error", "message": "Missing version"}
            asyncio.create_task(self._update_manager.stage_usb_release(version))
            return {"status": "ok", "data": self._update_manager.state.to_dict()}

        elif action == 'update.install':
            if not self._update_manager:
                return {"status": "error", "message": "Update manager not available"}
            await self.broadcast_activity('Installing software update — server will restart', 'info')
            asyncio.create_task(self._update_manager.install_staged())
            return {"status": "ok", "data": self._update_manager.state.to_dict()}

        # Web Terminal Actions -----------------------------------------------
        elif action == 'terminal.open':
            return await self._handle_terminal_open(client, params)
        elif action == 'terminal.input':
            return self._handle_terminal_input(client, params)
        elif action == 'terminal.resize':
            return self._handle_terminal_resize(client, params)
        elif action == 'terminal.close':
            return await self._handle_terminal_close(client)

        # Firmware Management Actions
        elif action == 'get_server_firmware':
            fw_info = self.state_manager.get_server_firmware_info()
            return {"status": "ok", "data": fw_info}

        elif action == 'check_firmware_update':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            update_info = self.state_manager.is_firmware_update_available(node_id)
            return {"status": "ok", "data": update_info}

        elif action == 'update_firmware':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            # Check if update is available
            update_info = self.state_manager.is_firmware_update_available(node_id)
            if not update_info["available"]:
                return {"status": "error", "message": update_info["message"]}

            # Trigger firmware update via callback (default to hardware update, not forced)
            if self._firmware_update_callback:
                self._firmware_update_callback(node_id, False, False)  # simulation=False, force=False
                await self.broadcast_activity(
                    f'Firmware update initiated for node {node_id}: '
                    f'{update_info["current_version"]} → {update_info["server_version"]}',
                    'info'
                )
                return {
                    "status": "ok",
                    "data": {
                        "message": "Firmware update initiated",
                        "from_version": update_info["current_version"],
                        "to_version": update_info["server_version"],
                    }
                }
            else:
                return {"status": "error", "message": "Firmware update not available"}

        elif action == 'get_firmware_builds':
            # Get info for all firmware builds (simulation and hardware)
            builds = self.state_manager.get_all_firmware_builds()
            return {"status": "ok", "data": builds}

        elif action == 'force_firmware_update':
            node_id = params.get('node_id')
            build_type = params.get('build_type')  # 'simulation' or 'hardware'

            if not node_id:
                return {"status": "error", "message": "Missing node_id"}
            if build_type not in ('simulation', 'hardware'):
                return {"status": "error", "message": "Invalid build_type (must be 'simulation' or 'hardware')"}

            # Get firmware info for the requested build type
            fw_info = self.state_manager.get_firmware_build_info(build_type)
            if not fw_info["available"]:
                return {"status": "error", "message": f"No {build_type} firmware build found"}

            # Trigger firmware update via callback (forced)
            if self._firmware_update_callback:
                is_simulation = (build_type == 'simulation')
                self._firmware_update_callback(node_id, is_simulation, True)  # force=True
                await self.broadcast_activity(
                    f'Force firmware update ({build_type}) initiated for node {node_id}: {fw_info.get("version_full") or fw_info.get("version")}',
                    'info'
                )
                return {
                    "status": "ok",
                    "data": {
                        "message": "Firmware update initiated",
                        "build_type": build_type,
                        "version": fw_info.get("version_full") or fw_info.get("version"),
                        "success": True,
                    }
                }
            else:
                return {"status": "error", "message": "Firmware update not available"}

        # Server Settings Actions
        elif action == 'get_settings':
            try:
                from saint_server.config import config_to_dict
                settings = config_to_dict()
                return {"status": "ok", "data": settings}
            except Exception as e:
                self.log('error', f'Failed to get settings: {e}')
                return {"status": "error", "message": f"Failed to get settings: {e}"}

        elif action == 'set_settings':
            try:
                from saint_server.config import update_config_from_dict, save_config, get_config

                settings = params.get('settings', {})
                if not settings:
                    return {"status": "error", "message": "Missing settings data"}

                # Update in-memory config
                update_config_from_dict(settings)

                # Save to file
                if save_config():
                    # Update local auth settings if changed
                    config = get_config()
                    self._auth_password = config.websocket.password
                    self._auth_timeout = config.websocket.auth_timeout

                    await self.broadcast_activity('Server settings updated (restart required for some changes)', 'info')
                    return {
                        "status": "ok",
                        "message": "Settings saved. Some changes require a server restart.",
                        "data": {"restart_required": True}
                    }
                else:
                    return {"status": "error", "message": "Failed to save settings to file"}
            except Exception as e:
                self.log('error', f'Failed to save settings: {e}')
                return {"status": "error", "message": f"Failed to save settings: {e}"}

        else:
            return {"status": "error", "message": f"Unknown management action: {action}"}

    async def _handle_subscribe(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle subscription messages."""
        if action == 'subscribe':
            topics = params.get('topics', [])
            client.subscriptions.update(topics)
            return {
                "status": "ok",
                "message": "Subscribed",
                "data": {"topics": list(client.subscriptions)},
            }

        elif action == 'unsubscribe':
            topics = params.get('topics', [])
            client.subscriptions.difference_update(topics)
            return {
                "status": "ok",
                "message": "Unsubscribed",
                "data": {"topics": list(client.subscriptions)},
            }

        else:
            return {"status": "error", "message": f"Unknown subscribe action: {action}"}

    async def _handle_router(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle routing-runtime messages.

        Used by the controller's binding system to discover WS-input
        nodes declared on routing sheets and to push axis values into
        them. Sheet authoring lives on the management channel — this
        handler is just the live data path.
        """
        if action == 'list_websocket_inputs':
            return {"status": "ok",
                    "data": {"ws_inputs": self.state_manager.list_ws_inputs()}}

        if action == 'set_input':
            sheet_id = params.get('sheet_id')
            input_id = params.get('input_id')
            value = params.get('value')
            if not sheet_id or not input_id or value is None:
                return {"status": "error",
                        "message": "Missing sheet_id, input_id, or value"}
            try:
                scalar = float(value)
            except (TypeError, ValueError):
                return {"status": "error", "message": "Invalid value type"}
            ok = self.state_manager.push_ws_input(sheet_id, input_id, scalar)
            if not ok:
                return {"status": "error",
                        "message": f"No WS input {sheet_id}/{input_id} (or evaluator offline)"}
            return {"status": "ok"}

        return {"status": "error", "message": f"Unknown router action: {action}"}

    async def _handle_control(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle pin control messages with throttling."""
        if action == 'set_pin_value':
            node_id = params.get('node_id')
            gpio = params.get('gpio')
            value = params.get('value')

            if not node_id or gpio is None or value is None:
                return {"status": "error", "message": "Missing node_id, gpio, or value"}

            try:
                gpio = int(gpio)
                value = float(value)
            except (ValueError, TypeError):
                return {"status": "error", "message": "Invalid gpio or value type"}

            # Direct pin write — the value is passed through verbatim. The
            # routing engine handles unit conversion; raw control endpoints
            # like this are for debug / power-user use, so callers are
            # responsible for sending native-range values.
            native_value = value

            throttle_key = (node_id, gpio)
            now = time.time() * 1000  # ms
            last_send = self._control_throttle.get(throttle_key, 0)
            is_neutral = is_neutral_value(value)

            if not is_neutral and now - last_send < CONTROL_THROTTLE_MS:
                self.state_manager.update_pin_desired(node_id, gpio, native_value)
                await self._broadcast_pin_state(node_id)
                self.log('debug', f'[Control] THROTTLED {node_id} GPIO {gpio}: {native_value:.1f}')
                return {"status": "ok", "message": "Throttled", "data": {"throttled": True}}

            self.state_manager.update_pin_desired(node_id, gpio, native_value)

            if self._send_control_callback:
                self._send_control_callback(node_id, gpio, native_value)
                self._control_throttle[throttle_key] = now
                bypass_note = ' [STOP]' if is_neutral else ''
                self.log('info', f'[Control] SENT{bypass_note} {node_id} GPIO {gpio}: {native_value:.1f}')

            await self._broadcast_pin_state(node_id)

            return {"status": "ok", "data": {"throttled": False, "native_value": native_value}}

        elif action == 'set_pin_values':
            # Batch update multiple pins
            node_id = params.get('node_id')
            pins = params.get('pins', [])  # List of {gpio, value}

            if not node_id or not pins:
                return {"status": "error", "message": "Missing node_id or pins"}

            now = time.time() * 1000
            sent_count = 0
            throttled_count = 0

            for pin_data in pins:
                try:
                    gpio = int(pin_data.get('gpio'))
                    value = float(pin_data.get('value'))

                    # Raw write — caller is responsible for native ranges.
                    native_value = value
                    self.state_manager.update_pin_desired(node_id, gpio, native_value)

                    # Check per-gpio throttle
                    # IMPORTANT: Neutral/stop values ALWAYS bypass throttle
                    throttle_key = (node_id, gpio)
                    last_send = self._control_throttle.get(throttle_key, 0)
                    is_neutral = is_neutral_value(value)

                    if is_neutral or now - last_send >= CONTROL_THROTTLE_MS:
                        if self._send_control_callback:
                            self._send_control_callback(node_id, gpio, native_value)
                            self._control_throttle[throttle_key] = now
                            sent_count += 1
                    else:
                        throttled_count += 1
                except (ValueError, TypeError, AttributeError):
                    continue

            return {"status": "ok", "data": {"sent": sent_count, "throttled": throttled_count, "count": len(pins)}}

        elif action == 'set_channel_value':
            node_id = params.get('node_id')
            peripheral_id = params.get('peripheral_id')
            channel_id = params.get('channel_id')
            value = params.get('value')

            if not node_id or not peripheral_id or not channel_id or value is None:
                return {"status": "error",
                        "message": "Missing node_id, peripheral_id, channel_id, or value"}

            try:
                value = float(value)
            except (ValueError, TypeError):
                return {"status": "error", "message": "Invalid value type"}

            target = self.state_manager.lookup_channel(
                node_id, peripheral_id, channel_id)
            if not target:
                return {"status": "error",
                        "message": f"Channel {peripheral_id}/{channel_id} not found on node"}
            if target["direction"] != "out":
                return {"status": "error",
                        "message": f"Channel {peripheral_id}/{channel_id} is not writable"}

            throttle_key = (node_id, peripheral_id, channel_id)
            now = time.time() * 1000
            last_send = self._control_throttle.get(throttle_key, 0)
            is_neutral = is_neutral_value(value)

            if not is_neutral and now - last_send < CONTROL_THROTTLE_MS:
                self.log('debug', f'[Control] THROTTLED {node_id} '
                                  f'{peripheral_id}/{channel_id}: {value:.3f}')
                return {"status": "ok", "data": {"throttled": True}}

            if self._send_channel_callback:
                # Pass the catalog type along so the firmware can route
                # type-specific writes (e.g. peripheral_type="neopixel"
                # for the status LED) without needing to know the
                # operator-chosen instance id.
                peripheral_type = target.get("peripheral_type", "")
                self._send_channel_callback(node_id, peripheral_id, channel_id,
                                            value, peripheral_type)
                self._control_throttle[throttle_key] = now
                bypass_note = ' [STOP]' if is_neutral else ''
                self.log('info', f'[Control] SENT{bypass_note} {node_id} '
                                 f'{peripheral_id}/{channel_id}: {value:.3f}')
            else:
                return {"status": "error",
                        "message": "Channel-addressed control not wired to firmware"}

            return {"status": "ok",
                    "data": {"throttled": False, "value": value}}

        elif action == 'get_runtime_state':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            state = self.state_manager.get_runtime_state(node_id)
            if state:
                return {"status": "ok", "data": state}
            else:
                return {"status": "ok", "data": None, "message": "No runtime state available"}

        else:
            return {"status": "error", "message": f"Unknown control action: {action}"}

    async def _handle_discovery(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """
        Handle discovery messages for roles and functions.

        This allows the controller to discover available roles and their
        logical functions without hardcoding node IDs or GPIO pins.
        """
        if action == 'get_roles':
            # Return all available role definitions
            roles = []
            for role_def in self.role_manager.get_all_roles():
                roles.append(role_def.to_dict())
            return {"status": "ok", "data": {"roles": roles}}

        elif action == 'get_role':
            # Return a specific role definition
            role_name = params.get('role')
            if not role_name:
                return {"status": "error", "message": "Missing role parameter"}

            role_def = self.role_manager.get_role(role_name)
            if not role_def:
                return {"status": "error", "message": f"Role not found: {role_name}"}

            return {"status": "ok", "data": role_def.to_dict()}

        elif action == 'get_active_roles':
            # Return roles that are currently assigned to adopted nodes
            active_roles = []
            for node in self.state_manager.state.adopted_nodes.values():
                if node.role:
                    role_def = self.role_manager.get_role(node.role)
                    if role_def:
                        active_roles.append({
                            "role": node.role,
                            "node_id": node.node_id,
                            "display_name": node.display_name or node.node_id,
                            "online": node.online,
                            "definition": role_def.to_dict(),
                        })
            return {"status": "ok", "data": {"active_roles": active_roles}}

        elif action == 'get_controllable_functions':
            # Return functions that are currently configured on adopted nodes
            # This is what the controller needs to know what it can actually control
            controllable = []
            for node in self.state_manager.state.adopted_nodes.values():
                if not node.role or not node.pin_config:
                    continue

                node_functions = []
                for gpio, config in node.pin_config.pins.items():
                    if config.logical_name and config.mode in ('pwm', 'servo', 'digital_out'):
                        node_functions.append({
                            "function": config.logical_name,
                            "mode": config.mode,
                            "gpio": gpio,  # Include for debugging, controller doesn't need this
                        })

                if node_functions:
                    controllable.append({
                        "role": node.role,
                        "node_id": node.node_id,
                        "display_name": node.display_name or node.node_id,
                        "online": node.online,
                        "functions": node_functions,
                    })

            return {"status": "ok", "data": {"controllable": controllable}}

        else:
            return {"status": "error", "message": f"Unknown discovery action: {action}"}

    async def _handle_livelink(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle LiveLink management messages."""
        if action == 'get_status':
            if self._livelink_get_status:
                status = self._livelink_get_status()
                return {"status": "ok", "data": status}
            else:
                return {"status": "error", "message": "LiveLink not available"}

        elif action == 'get_routes':
            if self._livelink_get_routes:
                routes = self._livelink_get_routes()
                return {"status": "ok", "data": {"routes": routes}}
            else:
                return {"status": "error", "message": "LiveLink not available"}

        elif action == 'set_routes':
            routes = params.get('routes', [])
            if self._livelink_set_routes:
                self._livelink_set_routes(routes)
                await self.broadcast_activity(f'LiveLink routes updated ({len(routes)} routes)', 'info')
                return {"status": "ok", "message": "Routes updated"}
            else:
                return {"status": "error", "message": "LiveLink not available"}

        elif action == 'enable_route':
            route_name = params.get('route_name')
            enabled = params.get('enabled', True)
            if not route_name:
                return {"status": "error", "message": "Missing route_name"}

            if self._livelink_enable_route:
                self._livelink_enable_route(route_name, enabled)
                return {"status": "ok", "message": f"Route '{route_name}' {'enabled' if enabled else 'disabled'}"}
            else:
                return {"status": "error", "message": "LiveLink not available"}

        elif action == 'create_default_route':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            if self._livelink_create_default_route:
                route = self._livelink_create_default_route(node_id)
                await self.broadcast_activity(f'Created default LiveLink route for {node_id}', 'info')
                return {"status": "ok", "data": route}
            else:
                return {"status": "error", "message": "LiveLink not available"}

        else:
            return {"status": "error", "message": f"Unknown livelink action: {action}"}

    async def _handle_ros(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """Handle ROS bridge messages (subscribe, unsubscribe, publish, list_endpoints).

        Uses unified endpoint paths - same path for subscribe and publish:
          /saint/head  <- subscribe gets state, publish sends commands
        """
        if not self._ros_bridge:
            return {"status": "error", "message": "ROS bridge not available"}

        if action == 'subscribe':
            # Support both 'endpoint' and 'topic' param names for flexibility
            endpoint = params.get('endpoint') or params.get('topic')
            if not endpoint:
                return {"status": "error", "message": "Missing endpoint parameter"}

            result = self._ros_bridge.subscribe(endpoint, client.id)
            if result.get('status') == 'ok':
                # Track subscription in client for WebSocket state management
                client.subscriptions.add(f'ros:{endpoint}')
            return result

        elif action == 'unsubscribe':
            endpoint = params.get('endpoint') or params.get('topic')
            if not endpoint:
                return {"status": "error", "message": "Missing endpoint parameter"}

            result = self._ros_bridge.unsubscribe(endpoint, client.id)
            client.subscriptions.discard(f'ros:{endpoint}')
            return result

        elif action == 'publish':
            endpoint = params.get('endpoint') or params.get('topic')
            data = params.get('data', {})
            if not endpoint:
                return {"status": "error", "message": "Missing endpoint parameter"}

            return self._ros_bridge.publish(endpoint, data, client.id)

        elif action in ('list_endpoints', 'list_topics'):
            return self._ros_bridge.list_endpoints()

        elif action == 'list_topic_channels':
            return self._ros_bridge.list_topic_channels()

        elif action == 'set_topic_channel':
            endpoint = params.get('endpoint') or params.get('topic')
            field = params.get('field') or params.get('channel')
            if endpoint is None or field is None or 'value' not in params:
                return {"status": "error",
                        "message": "Missing endpoint/topic, field/channel, or value"}
            return self._ros_bridge.set_topic_channel(
                endpoint, field, params.get('value'), client.id)

        else:
            return {"status": "error", "message": f"Unknown ROS action: {action}"}

    def _get_resources_dir(self) -> Path:
        """Get the resources directory path."""
        # Resources directory is at saint_os/resources
        return Path(__file__).parent.parent.parent / 'resources'

    def _validate_filename(self, filename: str) -> bool:
        """Validate filename to prevent path traversal attacks."""
        if not filename:
            return False
        # Reject paths with directory traversal
        if '..' in filename or filename.startswith('/') or filename.startswith('\\'):
            return False
        # Reject absolute paths on Windows
        if len(filename) > 1 and filename[1] == ':':
            return False
        # Only allow certain characters
        safe_chars = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-./!')
        if not all(c in safe_chars for c in filename):
            return False
        return True

    def _get_safe_path(self, filename: str, category: Optional[str] = None) -> Optional[Path]:
        """
        Get a safe file path within the resources directory.

        Returns None if the path would escape the resources directory.
        """
        resources_dir = self._get_resources_dir()

        # Build the path
        if category:
            if not self._validate_filename(category):
                return None
            file_path = resources_dir / category / filename
        else:
            file_path = resources_dir / filename

        # Resolve to absolute and verify it's still under resources
        try:
            resolved = file_path.resolve()
            resources_resolved = resources_dir.resolve()
            if not str(resolved).startswith(str(resources_resolved)):
                return None
            return resolved
        except (ValueError, OSError):
            return None

    async def _handle_file(self, client: WebSocketClient, action: str, params: dict) -> dict:
        """
        Handle file transfer messages.

        Actions:
          - upload: Upload a file to resources (base64 encoded)
          - list: List files in resources directory
          - download: Download a file (base64 encoded)
          - delete: Delete a file from resources
          - parse: Upload and parse a data asset file
        """
        if action == 'upload':
            return await self._handle_file_upload(client, params)
        elif action == 'list':
            return await self._handle_file_list(client, params)
        elif action == 'download':
            return await self._handle_file_download(client, params)
        elif action == 'delete':
            return await self._handle_file_delete(client, params)
        elif action == 'parse':
            return await self._handle_file_parse(client, params)
        else:
            return {"status": "error", "message": f"Unknown file action: {action}"}

    async def _handle_file_upload(self, client: WebSocketClient, params: dict) -> dict:
        """Handle file upload."""
        filename = params.get('filename')
        content_b64 = params.get('content')
        category = params.get('category')  # Optional subdirectory

        if not filename:
            return {"status": "error", "message": "Missing filename"}
        if not content_b64:
            return {"status": "error", "message": "Missing content"}

        # Validate filename
        if not self._validate_filename(filename):
            return {"status": "error", "message": "Invalid filename"}

        # Get safe path
        file_path = self._get_safe_path(filename, category)
        if not file_path:
            return {"status": "error", "message": "Invalid file path"}

        # Decode content
        try:
            content = base64.b64decode(content_b64)
        except Exception as e:
            return {"status": "error", "message": f"Invalid base64 content: {e}"}

        # Create directory if needed
        try:
            file_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            return {"status": "error", "message": f"Failed to create directory: {e}"}

        # Write file
        try:
            with open(file_path, 'wb') as f:
                f.write(content)
        except OSError as e:
            return {"status": "error", "message": f"Failed to write file: {e}"}

        # Calculate relative path for response
        resources_dir = self._get_resources_dir().resolve()
        relative_path = str(file_path.relative_to(resources_dir))

        self.log('info', f'Client {client.id} uploaded file: {relative_path} ({len(content)} bytes)')
        await self.broadcast_activity(f'File uploaded: {relative_path}', 'info')

        return {
            "status": "ok",
            "data": {
                "path": relative_path,
                "size": len(content),
                "filename": filename,
            }
        }

    async def _handle_file_list(self, client: WebSocketClient, params: dict) -> dict:
        """List files in resources directory."""
        category = params.get('category')  # Optional subdirectory filter
        pattern = params.get('pattern', '*')  # Optional glob pattern

        resources_dir = self._get_resources_dir()

        if category:
            if not self._validate_filename(category):
                return {"status": "error", "message": "Invalid category"}
            search_dir = resources_dir / category
        else:
            search_dir = resources_dir

        if not search_dir.exists():
            return {"status": "ok", "data": {"files": []}}

        try:
            files = []
            for path in search_dir.rglob(pattern):
                if path.is_file():
                    relative = path.relative_to(resources_dir)
                    files.append({
                        "path": str(relative),
                        "name": path.name,
                        "size": path.stat().st_size,
                        "modified": path.stat().st_mtime,
                    })

            # Sort by name
            files.sort(key=lambda f: f['path'])

            return {"status": "ok", "data": {"files": files}}
        except OSError as e:
            return {"status": "error", "message": f"Failed to list files: {e}"}

    async def _handle_file_download(self, client: WebSocketClient, params: dict) -> dict:
        """Download a file from resources."""
        filename = params.get('filename')
        category = params.get('category')

        if not filename:
            return {"status": "error", "message": "Missing filename"}

        if not self._validate_filename(filename):
            return {"status": "error", "message": "Invalid filename"}

        file_path = self._get_safe_path(filename, category)
        if not file_path:
            return {"status": "error", "message": "Invalid file path"}

        if not file_path.exists():
            return {"status": "error", "message": "File not found"}

        if not file_path.is_file():
            return {"status": "error", "message": "Path is not a file"}

        try:
            with open(file_path, 'rb') as f:
                content = f.read()

            content_b64 = base64.b64encode(content).decode('utf-8')

            return {
                "status": "ok",
                "data": {
                    "filename": filename,
                    "content": content_b64,
                    "size": len(content),
                }
            }
        except OSError as e:
            return {"status": "error", "message": f"Failed to read file: {e}"}

    async def _handle_file_delete(self, client: WebSocketClient, params: dict) -> dict:
        """Delete a file from resources."""
        filename = params.get('filename')
        category = params.get('category')

        if not filename:
            return {"status": "error", "message": "Missing filename"}

        if not self._validate_filename(filename):
            return {"status": "error", "message": "Invalid filename"}

        file_path = self._get_safe_path(filename, category)
        if not file_path:
            return {"status": "error", "message": "Invalid file path"}

        if not file_path.exists():
            return {"status": "error", "message": "File not found"}

        if not file_path.is_file():
            return {"status": "error", "message": "Path is not a file"}

        try:
            file_path.unlink()

            resources_dir = self._get_resources_dir().resolve()
            relative_path = str(file_path.relative_to(resources_dir))

            self.log('info', f'Client {client.id} deleted file: {relative_path}')
            await self.broadcast_activity(f'File deleted: {relative_path}', 'info')

            return {"status": "ok", "data": {"deleted": relative_path}}
        except OSError as e:
            return {"status": "error", "message": f"Failed to delete file: {e}"}

    async def _handle_file_parse(self, client: WebSocketClient, params: dict) -> dict:
        """
        Upload and parse a data asset file.

        This combines upload with parsing, returning both the file path
        and the parsed asset data.
        """
        filename = params.get('filename')
        content_b64 = params.get('content')
        category = params.get('category', 'moods')  # Default to moods

        if not filename:
            return {"status": "error", "message": "Missing filename"}
        if not content_b64:
            return {"status": "error", "message": "Missing content"}

        # Validate filename
        if not self._validate_filename(filename):
            return {"status": "error", "message": "Invalid filename"}

        # Decode content
        try:
            content = base64.b64decode(content_b64)
        except Exception as e:
            return {"status": "error", "message": f"Invalid base64 content: {e}"}

        # Try to parse as data asset
        parsed_data = None
        if filename.endswith('.uasset'):
            try:
                from saint_server.unreal.data_asset_reader import DataAssetReader
                reader = DataAssetReader()
                asset = reader.load_bytes(content, filename)
                parsed_data = asset.to_motion_dict()
            except Exception as e:
                self.log('warn', f'Failed to parse asset {filename}: {e}')
                # Continue with upload even if parsing fails

        # Get safe path
        file_path = self._get_safe_path(filename, category)
        if not file_path:
            return {"status": "error", "message": "Invalid file path"}

        # Create directory if needed
        try:
            file_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            return {"status": "error", "message": f"Failed to create directory: {e}"}

        # Write file
        try:
            with open(file_path, 'wb') as f:
                f.write(content)
        except OSError as e:
            return {"status": "error", "message": f"Failed to write file: {e}"}

        # Calculate relative path
        resources_dir = self._get_resources_dir().resolve()
        relative_path = str(file_path.relative_to(resources_dir))

        self.log('info', f'Client {client.id} uploaded asset: {relative_path} ({len(content)} bytes)')
        await self.broadcast_activity(f'Asset uploaded: {relative_path}', 'info')

        return {
            "status": "ok",
            "data": {
                "path": relative_path,
                "size": len(content),
                "filename": filename,
                "parsed": parsed_data,
            }
        }

    async def broadcast_ros_state(self, topic: str, data: dict):
        """Broadcast ROS state update to subscribed WebSocket clients."""
        message = {
            "type": "ros_state",
            "topic": topic,
            "data": data,
            "timestamp": time.time(),
        }

        subscription_key = f'ros:{topic}'

        async with self._lock:
            for client in self.clients.values():
                if subscription_key in client.subscriptions:
                    await self._send_to_client(client, message)

    async def _broadcast_pin_state(self, node_id: str):
        """Broadcast pin runtime state to subscribers."""
        state = self.state_manager.get_runtime_state(node_id)
        if state:
            await self.broadcast_state(f'pin_state/{node_id}', state)

    async def _broadcast_routing(self):
        """Broadcast the current system routing graph to subscribers."""
        await self.broadcast_state('system_routing', self.state_manager.get_system_routing())

    # ── Web terminal ─────────────────────────────────────────────────

    async def _handle_terminal_open(self, client: WebSocketClient, params: dict) -> dict:
        """Spawn a PTY-backed bash session for this client."""
        if client.terminal_session is not None:
            # One session per client. Close the old one first so resize/
            # reload always lands on a fresh shell.
            try:
                await client.terminal_session.close()
            except Exception:
                self.log('warn', f'terminal: prior close failed for {client.id}')
            client.terminal_session = None

        try:
            from saint_server.terminal_session import TerminalSession
        except ImportError as e:
            return {"status": "error", "message": f"Terminal module unavailable: {e}"}

        cols = int(params.get('cols') or 80)
        rows = int(params.get('rows') or 24)
        loop = asyncio.get_event_loop()

        async def on_output(chunk: bytes):
            try:
                # Forward as a binary WebSocket frame — xterm.js writes
                # bytes directly, so no UTF-8 split surprises.
                await client.ws.send_bytes(chunk)
            except Exception:
                self.log('warn', f'terminal: send_bytes failed for {client.id}')

        async def on_exit(code: int):
            self.log('info', f'terminal: shell exited code={code} for {client.id}')
            client.terminal_session = None
            try:
                await client.ws.send_json({
                    "type": "state",
                    "node": "terminal",
                    "data": {"event": "exit", "code": code},
                })
            except Exception:
                pass

        session = TerminalSession(
            on_output=on_output,
            on_exit=on_exit,
            client_id=client.id,
            logger=self.logger,
        )
        try:
            await session.start(cols=cols, rows=rows)
        except Exception as e:
            self.log('error', f'terminal: spawn failed: {e}')
            return {"status": "error", "message": f"Failed to start shell: {e}"}

        client.terminal_session = session
        return {"status": "ok", "data": {"cols": cols, "rows": rows}}

    def _handle_terminal_input(self, client: WebSocketClient, params: dict) -> dict:
        """Forward keystrokes to the shell. Data is a string (utf-8)."""
        if client.terminal_session is None:
            return {"status": "error", "message": "No terminal session"}
        data = params.get('data', '')
        if isinstance(data, str):
            data = data.encode('utf-8', errors='replace')
        elif not isinstance(data, (bytes, bytearray)):
            return {"status": "error", "message": "Invalid input data"}
        client.terminal_session.write(data)
        return {"status": "ok"}

    def _handle_terminal_resize(self, client: WebSocketClient, params: dict) -> dict:
        """Send TIOCSWINSZ to the PTY so apps reflow."""
        if client.terminal_session is None:
            return {"status": "error", "message": "No terminal session"}
        try:
            cols = int(params.get('cols') or 80)
            rows = int(params.get('rows') or 24)
        except (TypeError, ValueError):
            return {"status": "error", "message": "Invalid cols/rows"}
        client.terminal_session.resize(cols, rows)
        return {"status": "ok"}

    async def _handle_terminal_close(self, client: WebSocketClient) -> dict:
        """Tear down this client's shell."""
        if client.terminal_session is None:
            return {"status": "ok"}
        try:
            await client.terminal_session.close()
        finally:
            client.terminal_session = None
        return {"status": "ok"}

    async def broadcast_state(self, topic: str, data: dict):
        """Broadcast state update to subscribed clients."""
        message = {
            "type": "state",
            "node": topic,
            "data": data,
        }

        async with self._lock:
            for client in self.clients.values():
                if topic in client.subscriptions or 'all' in client.subscriptions:
                    await self._send_to_client(client, message)

    async def broadcast_activity(self, text: str, level: str = 'info'):
        """Broadcast activity log entry to all clients."""
        message = {
            "type": "activity",
            "text": text,
            "level": level,
            "timestamp": time.time(),
        }

        async with self._lock:
            for client in self.clients.values():
                await self._send_to_client(client, message)

    async def broadcast_node_log(self, node_id: str, entry: Dict[str, Any]):
        """Push a single per-node log entry to subscribers of node_log/<id>.

        Goes through the same ``broadcast_state`` plumbing so existing
        subscription filtering applies — clients that haven't subscribed
        to this topic don't get the frame.
        """
        await self.broadcast_state(f'node_log/{node_id}', entry)

    async def start_broadcast_loop(self):
        """Start periodic state broadcasting."""
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())

    async def stop_broadcast_loop(self):
        """Stop the broadcast loop."""
        if self._broadcast_task:
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass

    async def _broadcast_loop(self):
        """Periodically broadcast system state to subscribers."""
        while True:
            try:
                await asyncio.sleep(1.0)  # 1 Hz

                # Broadcast system status to subscribers
                status = self.state_manager.get_system_status()
                await self.broadcast_state('system', status)

                # Refresh the host controller's runtime channels with
                # the same metrics and broadcast a pin_state so dashboard
                # widgets routed off host_controller.system_monitor.*
                # update with everything else.
                self.state_manager.update_host_controller_runtime()
                from saint_server.webserver.state_manager import HOST_CONTROLLER_NODE_ID
                host_state = self.state_manager.get_runtime_state(HOST_CONTROLLER_NODE_ID)
                if host_state:
                    await self.broadcast_state(f'pin_state/{HOST_CONTROLLER_NODE_ID}', host_state)

                # Broadcast node status
                nodes_data = {
                    "adopted": self.state_manager.get_adopted_nodes(),
                    "unadopted": self.state_manager.get_unadopted_nodes(),
                }
                await self.broadcast_state('nodes', nodes_data)

                # Broadcast LiveLink status if available
                if self._livelink_get_status:
                    livelink_status = self._livelink_get_status()
                    await self.broadcast_state('livelink', livelink_status)

            except asyncio.CancelledError:
                break
            except Exception as e:
                self.log('error', f'Broadcast loop error: {e}')

    async def close_all_connections(self):
        """Close all WebSocket connections."""
        async with self._lock:
            for client in list(self.clients.values()):
                try:
                    await client.ws.close()
                except Exception:
                    pass
            self.clients.clear()
