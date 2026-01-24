"""
SAINT.OS WebSocket Handler

Manages WebSocket client connections and message handling.
"""

import asyncio
import json
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Set, Any, Optional, Callable

from aiohttp import web, WSMsgType

from saint_server.webserver.state_manager import StateManager


@dataclass
class WebSocketClient:
    """Represents a connected WebSocket client."""
    id: str
    ws: web.WebSocketResponse
    subscriptions: Set[str] = field(default_factory=set)
    connected_at: float = field(default_factory=time.time)


class WebSocketHandler:
    """Handles WebSocket connections and messages."""

    def __init__(self, state_manager: StateManager, logger=None):
        self.state_manager = state_manager
        self.logger = logger
        self.clients: Dict[str, WebSocketClient] = {}
        self._lock = asyncio.Lock()
        self._broadcast_task: Optional[asyncio.Task] = None

    def log(self, level: str, message: str):
        """Log a message if logger is available."""
        if self.logger:
            getattr(self.logger, level)(message)

    async def handle_connection(self, request: web.Request) -> web.WebSocketResponse:
        """Handle a new WebSocket connection."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        client_id = str(uuid.uuid4())[:8]
        client = WebSocketClient(id=client_id, ws=ws)

        async with self._lock:
            self.clients[client_id] = client
            self.state_manager.set_client_count(len(self.clients))

        self.log('info', f'WebSocket client connected: {client_id}')

        # Send connected confirmation
        await self._send_to_client(client, {
            "type": "connected",
            "client_id": client_id,
            "server_name": self.state_manager.state.server_name,
        })

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self._handle_message(client, msg.data)
                elif msg.type == WSMsgType.ERROR:
                    self.log('error', f'WebSocket error: {ws.exception()}')
                    break
        finally:
            async with self._lock:
                self.clients.pop(client_id, None)
                self.state_manager.set_client_count(len(self.clients))

            self.log('info', f'WebSocket client disconnected: {client_id}')

        return ws

    async def _send_to_client(self, client: WebSocketClient, message: dict):
        """Send a message to a specific client."""
        try:
            await client.ws.send_json(message)
        except Exception as e:
            self.log('error', f'Failed to send to client {client.id}: {e}')

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
        }.get(msg_type)

        if handler:
            response = await handler(client, action, params)
            response['id'] = msg_id
            await self._send_to_client(client, response)
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

            result = self.state_manager.reset_node(node_id, factory_reset)
            if result['success']:
                await self.broadcast_activity(f'Node {node_id} reset', 'info')
            return {"status": "ok" if result['success'] else "error", "data": result}

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
