"""
ROS Bridge - Bidirectional WebSocket-ROS2 bridge.

Manages subscriptions, publications, and message conversion between
WebSocket clients and ROS2 topics.

Uses unified endpoint paths - subscribe and publish use the same path:
  /saint/head  <- subscribe gives state, publish sends commands
"""

import time
from typing import Dict, List, Set, Any, Optional, Callable
from threading import Lock

from rclpy.node import Node

from saint_server.ros_bridge.qos_profiles import QOS_PROFILES
from saint_server.ros_bridge.message_types import (
    load_endpoints,
    get_endpoint,
    get_all_endpoints,
    introspect_message_channels,
    serialize_message,
    deserialize_message,
    EndpointInfo
)

# Throttle settings (reuse WebSocket handler constant)
CONTROL_THROTTLE_MS = 50  # Minimum ms between commands per topic

class ROSBridge:
    """
    Bidirectional bridge between WebSocket clients and ROS2 topics.

    Uses unified endpoint paths:
    - Subscribe to /saint/head -> receives HeadState from /saint/head/state
    - Publish to /saint/head -> sends HeadCommand to /saint/head/command

    Manages:
    - Subscriptions: ROS topics -> WebSocket clients
    - Publications: WebSocket clients -> ROS topics
    - Message conversion between ROS and JSON
    - Client tracking for cleanup on disconnect
    """

    def __init__(self, ros_node: Node, logger=None):
        """
        Initialize the ROS Bridge.

        Args:
            ros_node: The ROS2 node to use for subscriptions/publications
            logger: Optional logger instance
        """
        self._node = ros_node
        self._logger = logger
        self._lock = Lock()

        # Callback for sending ROS messages to WebSocket
        self._message_callback: Optional[Callable[[str, Dict[str, Any]], None]] = None

        # Listeners that observe ROS messages independently of WebSocket
        # broadcast (e.g. the routing graph evaluator). Each callback
        # receives the same (endpoint_path, serialized_data) as the
        # WebSocket broadcaster.
        self._routing_listeners: List[Callable[[str, Dict[str, Any]], None]] = []

        # Active ROS subscriptions: endpoint_path -> subscription object
        self._subscriptions: Dict[str, Any] = {}

        # Clients subscribed to each endpoint: endpoint_path -> set of client_ids
        self._endpoint_clients: Dict[str, Set[str]] = {}

        # Active ROS publishers: endpoint_path -> publisher object
        self._publishers: Dict[str, Any] = {}

        # Throttle tracking: endpoint_path -> last_publish_time (ms)
        self._publish_throttle: Dict[str, float] = {}

        # Per-topic channel buffer used by `set_topic_channel`. Lets a
        # controller update a single scalar field (e.g. /tracks linear.x)
        # without sending the whole message: we merge the field into the
        # buffer and republish the merged dict each time.
        # endpoint_path -> {field_path: scalar_value}
        self._channel_buffers: Dict[str, Dict[str, Any]] = {}

        # Track initialization
        self._initialized = False

    def log(self, level: str, message: str):
        """Log a message if logger is available."""
        if self._logger:
            getattr(self._logger, level)(message)

    def initialize(self):
        """Initialize the bridge and load endpoint definitions."""
        if self._initialized:
            return

        # Load endpoints from YAML
        endpoints = load_endpoints()

        loaded_count = sum(1 for e in endpoints.values() if e.state_type or e.command_type)
        self.log('info', f'ROS Bridge initialized with {loaded_count} endpoints')

        # Log which endpoints were loaded
        for path, endpoint in endpoints.items():
            capabilities = []
            if endpoint.state_type:
                capabilities.append(f'subscribe:{endpoint.state_type.__name__}')
            if endpoint.command_type:
                capabilities.append(f'publish:{endpoint.command_type.__name__}')
            if capabilities:
                self.log('debug', f'  {path}: {", ".join(capabilities)}')

        self._initialized = True

    def add_routing_listener(self, callback: Callable[[str, Dict[str, Any]], None]):
        """Register a listener that observes every ROS message we forward.

        Independent of the WebSocket-broadcast callback, so the routing
        evaluator can see messages even on topics no WS client is
        subscribed to (it keeps its own bridge subscription open under
        the reserved `_routing_evaluator` client id).
        """
        self._routing_listeners.append(callback)

    def set_message_callback(self, callback: Callable[[str, Dict[str, Any]], None]):
        """
        Set the callback for sending ROS messages to WebSocket clients.

        The callback receives (endpoint_path: str, data: dict) and should broadcast
        to subscribed WebSocket clients.

        Args:
            callback: Function to call when ROS message received
        """
        self._message_callback = callback

    def subscribe(self, endpoint_path: str, client_id: str) -> Dict[str, Any]:
        """
        Subscribe a WebSocket client to a ROS endpoint.

        Args:
            endpoint_path: Unified endpoint path (e.g., '/saint/head')
            client_id: WebSocket client ID

        Returns:
            Response dict with status and info
        """
        endpoint = get_endpoint(endpoint_path)

        if not endpoint:
            return {
                'status': 'error',
                'message': f'Unknown endpoint: {endpoint_path}. Use list_endpoints to see available endpoints.'
            }

        if not endpoint.state_type:
            return {
                'status': 'error',
                'message': f'Endpoint {endpoint_path} does not support subscribing (no state type).'
            }

        with self._lock:
            # Add client to endpoint subscribers
            if endpoint_path not in self._endpoint_clients:
                self._endpoint_clients[endpoint_path] = set()
            self._endpoint_clients[endpoint_path].add(client_id)

            # Create ROS subscription if not already subscribed
            if endpoint_path not in self._subscriptions:
                self._create_subscription(endpoint)

        self.log('info', f'Client {client_id} subscribed to {endpoint_path}')

        return {
            'status': 'ok',
            'message': f'Subscribed to {endpoint_path}',
            'data': {
                'endpoint': endpoint_path,
                'state_type': endpoint.state_type.__name__,
                'ros_topic': endpoint.state_topic,
            }
        }

    def unsubscribe(self, endpoint_path: str, client_id: str) -> Dict[str, Any]:
        """
        Unsubscribe a WebSocket client from a ROS endpoint.

        Args:
            endpoint_path: Unified endpoint path
            client_id: WebSocket client ID

        Returns:
            Response dict with status
        """
        with self._lock:
            if endpoint_path in self._endpoint_clients:
                self._endpoint_clients[endpoint_path].discard(client_id)

                # Remove subscription if no more clients
                if not self._endpoint_clients[endpoint_path]:
                    self._remove_subscription(endpoint_path)
                    del self._endpoint_clients[endpoint_path]

        self.log('info', f'Client {client_id} unsubscribed from {endpoint_path}')

        return {
            'status': 'ok',
            'message': f'Unsubscribed from {endpoint_path}'
        }

    def publish(self, endpoint_path: str, data: Dict[str, Any], client_id: str) -> Dict[str, Any]:
        """
        Publish a command to a ROS endpoint from a WebSocket client.

        Args:
            endpoint_path: Unified endpoint path (e.g., '/saint/head')
            data: Command data as dict
            client_id: WebSocket client ID (for logging)

        Returns:
            Response dict with status
        """
        endpoint = get_endpoint(endpoint_path)

        if not endpoint:
            return {
                'status': 'error',
                'message': f'Unknown endpoint: {endpoint_path}. Use list_endpoints to see available endpoints.'
            }

        if not endpoint.command_type:
            return {
                'status': 'error',
                'message': f'Endpoint {endpoint_path} does not support publishing (no command type).'
            }

        # Check throttle
        now = time.time() * 1000  # ms
        last_publish = self._publish_throttle.get(endpoint_path, 0)
        if now - last_publish < CONTROL_THROTTLE_MS:
            return {
                'status': 'ok',
                'message': 'Throttled',
                'data': {'throttled': True}
            }

        with self._lock:
            # Get or create publisher
            if endpoint_path not in self._publishers:
                self._create_publisher(endpoint)

            publisher = self._publishers.get(endpoint_path)
            if not publisher:
                return {
                    'status': 'error',
                    'message': f'Failed to create publisher for {endpoint_path}'
                }

            # Deserialize and publish
            try:
                msg = deserialize_message(data, endpoint.command_type)
                publisher.publish(msg)
                self._publish_throttle[endpoint_path] = now
            except Exception as e:
                self.log('error', f'Failed to publish to {endpoint_path}: {e}')
                return {
                    'status': 'error',
                    'message': f'Failed to publish: {str(e)}'
                }

        self.log('debug', f'Published to {endpoint_path} from client {client_id}')

        return {
            'status': 'ok',
            'data': {'throttled': False}
        }

    def list_endpoints(self) -> Dict[str, Any]:
        """
        List all available endpoints that can be subscribed/published.

        Returns:
            Response dict with endpoint list
        """
        endpoints = get_all_endpoints()
        result = {}

        with self._lock:
            for path, endpoint in endpoints.items():
                result[path] = {
                    'can_subscribe': endpoint.state_type is not None,
                    'can_publish': endpoint.command_type is not None,
                    'state_type': endpoint.state_type.__name__ if endpoint.state_type else None,
                    'command_type': endpoint.command_type.__name__ if endpoint.command_type else None,
                    'subscribed': path in self._subscriptions,
                    'client_count': len(self._endpoint_clients.get(path, set())),
                }

        return {
            'status': 'ok',
            'data': {'endpoints': result}
        }

    # Keep list_topics as alias for backwards compatibility
    def list_topics(self) -> Dict[str, Any]:
        """Alias for list_endpoints (backwards compatibility)."""
        return self.list_endpoints()

    def set_topic_channel(self, endpoint_path: str, field: str,
                          value: Any, client_id: str = "") -> Dict[str, Any]:
        """Update a single scalar field on a topic and republish.

        Bindings on the controller call this from the runtime axis-fire
        path: instead of constructing a full ROS message they hand us
        (topic, field, value). We keep a per-topic dict buffer, merge
        the addressed field into it, and publish the merged message.
        Field paths use the same syntax as input channels — dotted
        segments plus `[N]` for arrays (e.g. `linear.x`, `axes[3]`).
        """
        endpoint = get_endpoint(endpoint_path)
        if not endpoint:
            return {'status': 'error',
                    'message': f'Unknown endpoint: {endpoint_path}'}
        if not endpoint.command_type:
            return {'status': 'error',
                    'message': f'Endpoint {endpoint_path} does not support publishing.'}

        # Merge into buffer.
        with self._lock:
            buf = self._channel_buffers.setdefault(endpoint_path, {})
            try:
                _set_field_in_dict(buf, field, value)
            except ValueError as e:
                return {'status': 'error', 'message': str(e)}
            # Reuse the existing throttle (per-endpoint).
            now = time.time() * 1000
            last_publish = self._publish_throttle.get(endpoint_path, 0)
            if now - last_publish < CONTROL_THROTTLE_MS:
                return {'status': 'ok',
                        'message': 'Throttled',
                        'data': {'throttled': True, 'buffered': True}}
            if endpoint_path not in self._publishers:
                self._create_publisher(endpoint)
            publisher = self._publishers.get(endpoint_path)
            if not publisher:
                return {'status': 'error',
                        'message': f'Failed to create publisher for {endpoint_path}'}
            try:
                msg = deserialize_message(buf, endpoint.command_type)
                publisher.publish(msg)
                self._publish_throttle[endpoint_path] = now
            except Exception as e:
                self.log('error',
                         f'Failed to publish channel {field} to {endpoint_path}: {e}')
                return {'status': 'error',
                        'message': f'Failed to publish: {str(e)}'}
        self.log('debug',
                 f'set_topic_channel {endpoint_path} {field}={value} (client {client_id})')
        return {'status': 'ok', 'data': {'throttled': False}}

    def list_topic_channels(self) -> Dict[str, Any]:
        """For each subscribable endpoint, flatten its msg type into
        selectable scalar channels (e.g. /joy → axes[0..7], buttons[0..7]).

        The routing UI uses this to populate the channel picker when the
        operator adds a topic-input source on a sheet.
        """
        topics: List[Dict[str, Any]] = []
        for path, endpoint in get_all_endpoints().items():
            if endpoint.state_type is None:
                continue
            channels = introspect_message_channels(endpoint.state_type)
            topics.append({
                "topic": path,
                "state_type": endpoint.state_type.__name__,
                "channels": channels,
            })
        topics.sort(key=lambda t: t["topic"])
        return {"status": "ok", "data": {"topics": topics}}

    def client_disconnected(self, client_id: str):
        """
        Clean up subscriptions when a WebSocket client disconnects.

        Args:
            client_id: WebSocket client ID that disconnected
        """
        endpoints_to_remove = []

        with self._lock:
            for endpoint_path, clients in self._endpoint_clients.items():
                clients.discard(client_id)
                if not clients:
                    endpoints_to_remove.append(endpoint_path)

            for endpoint_path in endpoints_to_remove:
                self._remove_subscription(endpoint_path)
                del self._endpoint_clients[endpoint_path]

        if endpoints_to_remove:
            self.log('info', f'Cleaned up {len(endpoints_to_remove)} subscriptions for client {client_id}')

    def _create_subscription(self, endpoint: EndpointInfo):
        """Create a ROS subscription for an endpoint's state topic."""
        qos = QOS_PROFILES.get(endpoint.state_qos)

        def callback(msg):
            self._on_ros_message(endpoint.path, msg)

        sub = self._node.create_subscription(
            endpoint.state_type,
            endpoint.state_topic,
            callback,
            qos
        )
        self._subscriptions[endpoint.path] = sub
        self.log('info', f'Created ROS subscription: {endpoint.state_topic} -> {endpoint.path}')

    def _remove_subscription(self, endpoint_path: str):
        """Remove a ROS subscription."""
        if endpoint_path in self._subscriptions:
            self._node.destroy_subscription(self._subscriptions[endpoint_path])
            del self._subscriptions[endpoint_path]
            self.log('info', f'Removed ROS subscription for {endpoint_path}')

    def _create_publisher(self, endpoint: EndpointInfo):
        """Create a ROS publisher for an endpoint's command topic."""
        qos = QOS_PROFILES.get(endpoint.command_qos)

        pub = self._node.create_publisher(
            endpoint.command_type,
            endpoint.command_topic,
            qos
        )
        self._publishers[endpoint.path] = pub
        self.log('info', f'Created ROS publisher: {endpoint.path} -> {endpoint.command_topic}')

    def _on_ros_message(self, endpoint_path: str, msg: Any):
        """
        Handle incoming ROS message: forward to WebSocket clients and
        notify any registered routing listeners.

        Args:
            endpoint_path: Unified endpoint path
            msg: ROS message
        """
        # Bail early only if nobody at all is interested.
        with self._lock:
            has_clients = bool(self._endpoint_clients.get(endpoint_path))
        has_listeners = bool(self._routing_listeners)
        if not has_clients and not has_listeners:
            return

        # Serialize message using introspection
        try:
            data = serialize_message(msg)
        except Exception as e:
            self.log('error', f'Failed to serialize message from {endpoint_path}: {e}')
            return

        # Broadcast to WebSocket clients (if any).
        if has_clients and self._message_callback:
            try:
                self._message_callback(endpoint_path, data)
            except Exception as e:
                self.log('error', f'Failed to broadcast message from {endpoint_path}: {e}')

        # Notify routing listeners — they observe regardless of WS clients.
        for listener in self._routing_listeners:
            try:
                listener(endpoint_path, data)
            except Exception as e:
                self.log('error', f'Routing listener failed for {endpoint_path}: {e}')


def _set_field_in_dict(buf: Dict[str, Any], field: str, value: Any) -> None:
    """Set `field` (dotted + [index] path) inside the nested-dict buffer.

    Creates missing dict entries and grows lists with zero-padded entries
    as needed so the buffer always materializes the path. Used by
    set_topic_channel to merge partial channel updates into a single
    serializable buffer per topic.
    """
    if not field:
        raise ValueError("field is required")
    segments = _parse_field_path(field)
    cur: Any = buf
    for i, (name, indices) in enumerate(segments):
        is_last = (i == len(segments) - 1)
        # Navigate (or create) the named container.
        if name:
            if not isinstance(cur, dict):
                raise ValueError(f"Cannot descend into non-dict at '{name}'")
            if is_last and not indices:
                cur[name] = value
                return
            if name not in cur or (not indices and not isinstance(cur[name], dict)):
                cur[name] = {} if not indices else []
            cur = cur[name]
        # Apply [N] index suffixes left-to-right.
        for j, idx in enumerate(indices):
            inner_last = is_last and j == len(indices) - 1
            if not isinstance(cur, list):
                raise ValueError(f"Cannot index non-list at '{field}'")
            while len(cur) <= idx:
                cur.append(0.0)
            if inner_last:
                cur[idx] = value
                return
            if not isinstance(cur[idx], (list, dict)):
                cur[idx] = {}
            cur = cur[idx]


def _parse_field_path(field: str) -> List:
    """Tokenize `field` into [(name, [indices]), …]."""
    out = []
    for seg in field.split('.'):
        if not seg:
            continue
        name = seg
        indices: List[int] = []
        bracket = seg.find('[')
        if bracket != -1:
            name = seg[:bracket]
            rest = seg[bracket:]
            i = 0
            while i < len(rest):
                if rest[i] != '[':
                    raise ValueError(f"Bad field syntax '{field}' near '{rest[i:]}'")
                close = rest.find(']', i)
                if close == -1:
                    raise ValueError(f"Unterminated [] in field '{field}'")
                try:
                    indices.append(int(rest[i + 1:close]))
                except ValueError:
                    raise ValueError(f"Non-integer index in field '{field}'")
                i = close + 1
        out.append((name, indices))
    return out
