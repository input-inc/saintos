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


@dataclass
class WebSocketClient:
    """Represents a connected WebSocket client."""
    id: str
    ws: web.WebSocketResponse
    subscriptions: Set[str] = field(default_factory=set)
    connected_at: float = field(default_factory=time.time)
    authenticated: bool = False  # True if auth succeeded or auth not required


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

        # Callback for triggering firmware update (set by server_node)
        # Signature: (node_id: str, simulation: bool) -> None
        self._firmware_update_callback: Optional[Callable[[str, bool], None]] = None

        # Callback for sending factory reset to node (set by server_node)
        self._factory_reset_callback: Optional[Callable[[str], None]] = None

        # Throttle tracking: node_id -> last_send_time
        self._control_throttle: Dict[str, float] = {}

        # LiveLink callbacks (set by server_node)
        self._livelink_get_status: Optional[Callable[[], Dict[str, Any]]] = None
        self._livelink_get_routes: Optional[Callable[[], Any]] = None
        self._livelink_set_routes: Optional[Callable[[Any], None]] = None
        self._livelink_enable_route: Optional[Callable[[str, bool], None]] = None
        self._livelink_create_default_route: Optional[Callable[[str], Dict[str, Any]]] = None

        # ROS Bridge (set by server_node)
        self._ros_bridge: Optional['ROSBridge'] = None

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

    def set_firmware_update_callback(self, callback: Callable[[str, bool], None]):
        """Set callback for triggering firmware update. Callback takes (node_id, simulation)."""
        self._firmware_update_callback = callback

    def set_factory_reset_callback(self, callback: Callable[[str], None]):
        """Set callback for sending factory reset to node. Callback takes (node_id)."""
        self._factory_reset_callback = callback

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
        client = WebSocketClient(
            id=client_id,
            ws=ws,
            authenticated=not auth_required  # Auto-auth if no password set
        )

        async with self._lock:
            self.clients[client_id] = client
            self.state_manager.set_client_count(len(self.clients))

        self.log('info', f'WebSocket client connected: {client_id}')

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

        # Handle authentication message
        msg_type = message.get('type')
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
            target = params.get('target', 'all')
            self.log('warn', f'E-STOP triggered by client {client.id}, target: {target}')
            await self.broadcast_activity(f'E-STOP triggered (target: {target})', 'warn')
            return {"status": "ok", "message": f"E-STOP sent to {target}"}
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

        elif action == 'adopt_node':
            node_id = params.get('node_id')
            role = params.get('role')
            display_name = params.get('display_name')

            if not node_id or not role:
                return {"status": "error", "message": "Missing node_id or role"}

            result = self.state_manager.adopt_node(node_id, role, display_name)
            if result['success']:
                await self.broadcast_activity(f'Node {node_id} adopted as {role}', 'info')
            return {"status": "ok" if result['success'] else "error", "data": result}

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

        elif action == 'get_pin_config':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            config = self.state_manager.get_node_pin_config(node_id)
            if config is not None:
                return {"status": "ok", "data": config}
            else:
                return {"status": "error", "message": f"Node {node_id} not found"}

        elif action == 'save_pin_config':
            node_id = params.get('node_id')
            pin_configs = params.get('pins', {})

            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            result = self.state_manager.save_node_pin_config(node_id, pin_configs)
            return {"status": "ok" if result['success'] else "error", "data": result}

        elif action == 'validate_pin_config':
            node_id = params.get('node_id')
            pin_configs = params.get('pins', {})
            role = params.get('role')

            if not node_id or not role:
                return {"status": "error", "message": "Missing node_id or role"}

            result = self.state_manager.validate_pin_config(node_id, pin_configs, role)
            return {"status": "ok", "data": result}

        elif action == 'sync_pin_config':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            config_json = self.state_manager.get_firmware_config_json(node_id)
            if not config_json:
                return {"status": "error", "message": "No configuration to sync"}

            if self._sync_config_callback:
                self._sync_config_callback(node_id, config_json)
                await self.broadcast_activity(f'Syncing configuration to node {node_id}', 'info')
                return {"status": "ok", "data": {"message": "Configuration sync initiated", "success": True}}
            else:
                return {"status": "error", "message": "Config sync not available"}

        elif action == 'get_logs':
            limit = params.get('limit', 100)
            level = params.get('level')
            logs = self.state_manager.get_logs(limit=limit, level=level)
            return {"status": "ok", "data": {"logs": logs}}

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

            # Trigger firmware update via callback (default to hardware update)
            if self._firmware_update_callback:
                self._firmware_update_callback(node_id, False)  # simulation=False
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

            # Trigger firmware update via callback
            if self._firmware_update_callback:
                is_simulation = (build_type == 'simulation')
                self._firmware_update_callback(node_id, is_simulation)
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
        """Handle router configuration messages."""
        # Router configuration not yet implemented
        return {"status": "error", "message": "Router configuration not yet implemented"}

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

            # Check throttle
            now = time.time() * 1000  # ms
            last_send = self._control_throttle.get(node_id, 0)
            if now - last_send < CONTROL_THROTTLE_MS:
                # Throttled - still update desired state but don't send to node
                self.state_manager.update_pin_desired(node_id, gpio, value)
                # Broadcast updated state to all subscribers
                await self._broadcast_pin_state(node_id)
                return {"status": "ok", "message": "Throttled", "data": {"throttled": True}}

            # Update desired state
            self.state_manager.update_pin_desired(node_id, gpio, value)

            # Send to node via ROS2
            if self._send_control_callback:
                self._send_control_callback(node_id, gpio, value)
                self._control_throttle[node_id] = now

            # Broadcast updated state to all subscribers
            await self._broadcast_pin_state(node_id)

            return {"status": "ok", "data": {"throttled": False}}

        elif action == 'set_pin_values':
            # Batch update multiple pins
            node_id = params.get('node_id')
            pins = params.get('pins', [])  # List of {gpio, value}

            if not node_id or not pins:
                return {"status": "error", "message": "Missing node_id or pins"}

            # Check throttle
            now = time.time() * 1000
            last_send = self._control_throttle.get(node_id, 0)
            throttled = now - last_send < CONTROL_THROTTLE_MS

            for pin_data in pins:
                try:
                    gpio = int(pin_data.get('gpio'))
                    value = float(pin_data.get('value'))
                    self.state_manager.update_pin_desired(node_id, gpio, value)

                    if not throttled and self._send_control_callback:
                        self._send_control_callback(node_id, gpio, value)
                except (ValueError, TypeError, AttributeError):
                    continue

            if not throttled:
                self._control_throttle[node_id] = now

            return {"status": "ok", "data": {"throttled": throttled, "count": len(pins)}}

        elif action == 'get_runtime_state':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            state = self.state_manager.get_runtime_state(node_id)
            if state:
                return {"status": "ok", "data": state}
            else:
                return {"status": "ok", "data": None, "message": "No runtime state available"}

        elif action == 'get_controllable_pins':
            node_id = params.get('node_id')
            if not node_id:
                return {"status": "error", "message": "Missing node_id"}

            pins = self.state_manager.get_controllable_pins(node_id)
            return {"status": "ok", "data": {"pins": pins}}

        else:
            return {"status": "error", "message": f"Unknown control action: {action}"}

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
