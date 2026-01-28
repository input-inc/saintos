"""
Auto-generated message serializers using ROS2 introspection.

This module loads endpoint definitions from endpoints.yaml and automatically
generates serializers/deserializers based on message field types.

No manual serializer code needed - just define endpoints in YAML and message
types in .msg files.
"""

import os
import importlib
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass

import yaml


@dataclass
class EndpointInfo:
    """Information about a registered endpoint."""
    path: str
    state_type: Optional[type] = None
    command_type: Optional[type] = None
    state_topic: Optional[str] = None  # Actual ROS topic for state
    command_topic: Optional[str] = None  # Actual ROS topic for command
    state_qos: str = 'state'
    command_qos: str = 'command'


# Registry of endpoints
_ENDPOINT_REGISTRY: Dict[str, EndpointInfo] = {}

# Cache of loaded message types
_MESSAGE_TYPE_CACHE: Dict[str, type] = {}


def _load_message_type(type_string: str) -> Optional[type]:
    """
    Dynamically load a ROS message type from a string like 'saint_os/msg/HeadState'.

    Args:
        type_string: Message type in format 'package/msg/MessageType'

    Returns:
        The message class, or None if not found
    """
    if type_string in _MESSAGE_TYPE_CACHE:
        return _MESSAGE_TYPE_CACHE[type_string]

    try:
        # Parse 'saint_os/msg/HeadState' -> module='saint_os.msg', class='HeadState'
        parts = type_string.split('/')
        if len(parts) != 3 or parts[1] != 'msg':
            return None

        module_name = f"{parts[0]}.{parts[1]}"
        class_name = parts[2]

        module = importlib.import_module(module_name)
        msg_class = getattr(module, class_name)

        _MESSAGE_TYPE_CACHE[type_string] = msg_class
        return msg_class

    except (ImportError, AttributeError) as e:
        return None


def _get_field_names(msg_class: type) -> List[str]:
    """Get field names from a ROS message class using introspection."""
    # ROS2 messages have get_fields_and_field_types() method
    if hasattr(msg_class, 'get_fields_and_field_types'):
        return list(msg_class.get_fields_and_field_types().keys())
    # Fallback: use __slots__
    elif hasattr(msg_class, '__slots__'):
        return [s for s in msg_class.__slots__ if not s.startswith('_')]
    return []


def _get_field_types(msg_class: type) -> Dict[str, str]:
    """Get field types from a ROS message class."""
    if hasattr(msg_class, 'get_fields_and_field_types'):
        return msg_class.get_fields_and_field_types()
    return {}


def _serialize_value(value: Any, field_type: str = '') -> Any:
    """
    Serialize a single value to JSON-compatible format.

    Handles ROS-specific types like Time, arrays, and nested messages.
    """
    if value is None:
        return None

    # Handle builtin_interfaces/Time
    if hasattr(value, 'sec') and hasattr(value, 'nanosec'):
        return value.sec + value.nanosec * 1e-9

    # Handle arrays/sequences
    if isinstance(value, (list, tuple)) or hasattr(value, '__iter__') and not isinstance(value, (str, bytes)):
        try:
            return [_serialize_value(v) for v in value]
        except TypeError:
            pass

    # Handle nested messages (have get_fields_and_field_types or __slots__)
    if hasattr(value, 'get_fields_and_field_types') or (hasattr(value, '__slots__') and not isinstance(value, (str, bytes, int, float, bool))):
        return serialize_message(value)

    # Handle numpy types
    type_name = type(value).__name__
    if 'numpy' in type(value).__module__ or type_name in ('float32', 'float64', 'int32', 'int64', 'uint32', 'uint64'):
        if 'int' in type_name:
            return int(value)
        return float(value)

    # Primitives pass through
    if isinstance(value, (bool, int, float, str)):
        return value

    # Bytes to string
    if isinstance(value, bytes):
        return value.decode('utf-8', errors='replace')

    # Last resort: string representation
    return str(value)


def serialize_message(msg: Any) -> Dict[str, Any]:
    """
    Serialize any ROS message to a dictionary using introspection.

    Args:
        msg: ROS message instance

    Returns:
        Dictionary with all fields serialized
    """
    result = {}

    field_types = _get_field_types(type(msg))
    field_names = _get_field_names(type(msg))

    for field_name in field_names:
        try:
            value = getattr(msg, field_name)
            field_type = field_types.get(field_name, '')
            result[field_name] = _serialize_value(value, field_type)
        except Exception:
            # Skip fields that can't be accessed
            pass

    return result


def _deserialize_value(value: Any, field_type: str, current_value: Any = None) -> Any:
    """
    Deserialize a JSON value to the appropriate ROS type.

    Args:
        value: JSON value
        field_type: ROS field type string (e.g., 'float32', 'string', 'sequence<float32>')
        current_value: Current field value (for type reference)

    Returns:
        Value in appropriate type for ROS message
    """
    if value is None:
        return current_value

    # Handle sequences/arrays
    if field_type.startswith('sequence<') or '[]' in field_type or field_type.startswith('array<'):
        if isinstance(value, list):
            # Extract element type
            if field_type.startswith('sequence<'):
                elem_type = field_type[9:-1]
            elif field_type.startswith('array<'):
                elem_type = field_type.split('<')[1].split(',')[0]
            else:
                elem_type = field_type.replace('[]', '')
            return [_deserialize_value(v, elem_type) for v in value]
        return value

    # Handle primitives
    if field_type in ('float', 'float32', 'float64', 'double'):
        return float(value)
    if field_type in ('int8', 'int16', 'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64', 'int', 'byte'):
        return int(value)
    if field_type in ('bool', 'boolean'):
        return bool(value)
    if field_type == 'string':
        return str(value)

    # Handle nested messages
    if '/' in field_type and isinstance(value, dict):
        nested_type = _load_message_type(field_type.replace('.', '/'))
        if nested_type:
            return deserialize_message(value, nested_type)

    return value


def deserialize_message(data: Dict[str, Any], msg_class: type) -> Any:
    """
    Deserialize a dictionary to a ROS message using introspection.

    Args:
        data: Dictionary with field values
        msg_class: ROS message class to instantiate

    Returns:
        ROS message instance
    """
    msg = msg_class()

    field_types = _get_field_types(msg_class)
    field_names = _get_field_names(msg_class)

    for field_name in field_names:
        if field_name in data:
            try:
                field_type = field_types.get(field_name, '')
                current_value = getattr(msg, field_name, None)
                value = _deserialize_value(data[field_name], field_type, current_value)
                setattr(msg, field_name, value)
            except Exception:
                # Skip fields that can't be set
                pass

    return msg


def load_endpoints(yaml_path: Optional[str] = None) -> Dict[str, EndpointInfo]:
    """
    Load endpoint definitions from YAML file.

    Args:
        yaml_path: Path to endpoints.yaml (default: alongside this module)

    Returns:
        Dictionary of endpoint path -> EndpointInfo
    """
    global _ENDPOINT_REGISTRY

    if yaml_path is None:
        yaml_path = os.path.join(os.path.dirname(__file__), 'endpoints.yaml')

    if not os.path.exists(yaml_path):
        return _ENDPOINT_REGISTRY

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    endpoints = config.get('endpoints', {})

    for path, settings in endpoints.items():
        state_type = None
        command_type = None

        # Load state message type
        if 'state_type' in settings:
            state_type = _load_message_type(settings['state_type'])

        # Load command message type
        if 'command_type' in settings:
            command_type = _load_message_type(settings['command_type'])

        # Determine actual ROS topic names
        state_topic = f"{path}/state" if state_type else None
        command_topic = f"{path}/command" if command_type else None

        endpoint = EndpointInfo(
            path=path,
            state_type=state_type,
            command_type=command_type,
            state_topic=state_topic,
            command_topic=command_topic,
            state_qos=settings.get('state_qos', 'state'),
            command_qos=settings.get('command_qos', 'command'),
        )

        _ENDPOINT_REGISTRY[path] = endpoint

    return _ENDPOINT_REGISTRY


def get_endpoint(path: str) -> Optional[EndpointInfo]:
    """Get endpoint info by path."""
    return _ENDPOINT_REGISTRY.get(path)


def get_all_endpoints() -> Dict[str, EndpointInfo]:
    """Get all registered endpoints."""
    return _ENDPOINT_REGISTRY.copy()


def get_endpoint_for_topic(topic: str) -> Optional[EndpointInfo]:
    """
    Find endpoint that handles a given ROS topic.

    Handles both unified paths (/saint/head) and explicit topics (/saint/head/state).
    """
    # Direct endpoint match
    if topic in _ENDPOINT_REGISTRY:
        return _ENDPOINT_REGISTRY[topic]

    # Check if it's a state/command topic under an endpoint
    if topic.endswith('/state') or topic.endswith('/command'):
        base_path = topic.rsplit('/', 1)[0]
        return _ENDPOINT_REGISTRY.get(base_path)

    return None
