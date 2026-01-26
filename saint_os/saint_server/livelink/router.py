"""
LiveLink Router

Maps blend shape data from Live Link Face to node control outputs.
Allows configurable mapping of facial expressions to servo positions,
LED states, and other outputs.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional, Callable, Tuple
from enum import Enum

from saint_server.livelink.blend_shapes import BlendShapes, BLEND_SHAPE_NAMES


class MappingType(str, Enum):
    """Types of blend shape to output mappings."""
    DIRECT = "direct"           # Direct 1:1 mapping with scale/offset
    RANGE = "range"             # Map blend shape range to output range
    EXPRESSION = "expression"   # Python expression (for combining blend shapes)
    THRESHOLD = "threshold"     # On/off based on threshold


@dataclass
class OutputMapping:
    """
    Maps a blend shape (or combination) to a node output.

    Examples:
        # Map left eye blink to left eyelid servo (0-180 degrees)
        OutputMapping(
            source="EyeBlinkLeft",
            node_id="head_node",
            output_name="eye_left_lid",
            mapping_type=MappingType.RANGE,
            input_min=0.0, input_max=1.0,
            output_min=0, output_max=180,
        )

        # Map eye horizontal look to servo position
        OutputMapping(
            source="expression",
            expression="eye_look_out_left - eye_look_in_left",
            node_id="head_node",
            output_name="eye_left_lr",
            mapping_type=MappingType.RANGE,
            input_min=-1.0, input_max=1.0,
            output_min=0, output_max=180,
        )
    """
    # Source blend shape or expression
    source: str  # Blend shape name, "expression", or computed property name
    expression: str = ""  # Python expression if source is "expression"

    # Target node and output
    node_id: str = ""
    output_name: str = ""  # Logical function name (e.g., "eye_left_lr")

    # Mapping configuration
    mapping_type: MappingType = MappingType.RANGE

    # Input range (blend shape values are typically 0-1)
    input_min: float = 0.0
    input_max: float = 1.0

    # Output range (servo angles, PWM %, etc.)
    output_min: float = 0.0
    output_max: float = 180.0

    # Optional threshold for THRESHOLD mapping type
    threshold: float = 0.5
    threshold_on_value: float = 1.0
    threshold_off_value: float = 0.0

    # Smoothing (0 = no smoothing, 1 = max smoothing)
    smoothing: float = 0.0

    # Enable/disable this mapping
    enabled: bool = True

    # Last computed value (for smoothing)
    _last_value: float = field(default=0.0, repr=False)

    def compute(self, blend_shapes: BlendShapes) -> float:
        """Compute output value from blend shapes."""
        if not self.enabled:
            return self._last_value

        # Get input value
        if self.source == "expression" and self.expression:
            input_value = self._eval_expression(blend_shapes)
        elif hasattr(blend_shapes, self.source.lower().replace(" ", "_")):
            # Check for computed property (e.g., eye_look_horizontal_left)
            input_value = getattr(blend_shapes, self.source.lower().replace(" ", "_"))
        else:
            # Direct blend shape lookup
            input_value = blend_shapes.get_value(self.source)

        # Apply mapping
        if self.mapping_type == MappingType.DIRECT:
            output_value = input_value

        elif self.mapping_type == MappingType.RANGE:
            # Clamp input to range
            input_value = max(self.input_min, min(self.input_max, input_value))
            # Map to output range
            normalized = (input_value - self.input_min) / (self.input_max - self.input_min)
            output_value = self.output_min + normalized * (self.output_max - self.output_min)

        elif self.mapping_type == MappingType.THRESHOLD:
            output_value = self.threshold_on_value if input_value >= self.threshold else self.threshold_off_value

        elif self.mapping_type == MappingType.EXPRESSION:
            output_value = self._eval_expression(blend_shapes)

        else:
            output_value = input_value

        # Apply smoothing
        if self.smoothing > 0:
            alpha = 1.0 - self.smoothing
            output_value = alpha * output_value + self.smoothing * self._last_value

        self._last_value = output_value
        return output_value

    def _eval_expression(self, blend_shapes: BlendShapes) -> float:
        """Evaluate a Python expression with blend shape values."""
        try:
            # Create a safe namespace with blend shape values
            namespace = blend_shapes.to_dict()
            # Add lowercase versions and computed properties
            namespace.update({
                'eye_look_horizontal_left': blend_shapes.eye_look_horizontal_left,
                'eye_look_horizontal_right': blend_shapes.eye_look_horizontal_right,
                'eye_look_vertical_left': blend_shapes.eye_look_vertical_left,
                'eye_look_vertical_right': blend_shapes.eye_look_vertical_right,
                'brow_left': blend_shapes.brow_left,
                'brow_right': blend_shapes.brow_right,
            })
            # Add safe math functions
            namespace['abs'] = abs
            namespace['min'] = min
            namespace['max'] = max

            return float(eval(self.expression, {"__builtins__": {}}, namespace))
        except Exception:
            return 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "source": self.source,
            "expression": self.expression,
            "node_id": self.node_id,
            "output_name": self.output_name,
            "mapping_type": self.mapping_type.value,
            "input_min": self.input_min,
            "input_max": self.input_max,
            "output_min": self.output_min,
            "output_max": self.output_max,
            "threshold": self.threshold,
            "threshold_on_value": self.threshold_on_value,
            "threshold_off_value": self.threshold_off_value,
            "smoothing": self.smoothing,
            "enabled": self.enabled,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'OutputMapping':
        """Create from dictionary."""
        return cls(
            source=data.get("source", ""),
            expression=data.get("expression", ""),
            node_id=data.get("node_id", ""),
            output_name=data.get("output_name", ""),
            mapping_type=MappingType(data.get("mapping_type", "range")),
            input_min=data.get("input_min", 0.0),
            input_max=data.get("input_max", 1.0),
            output_min=data.get("output_min", 0.0),
            output_max=data.get("output_max", 180.0),
            threshold=data.get("threshold", 0.5),
            threshold_on_value=data.get("threshold_on_value", 1.0),
            threshold_off_value=data.get("threshold_off_value", 0.0),
            smoothing=data.get("smoothing", 0.0),
            enabled=data.get("enabled", True),
        )


@dataclass
class LiveLinkRoute:
    """A named collection of output mappings."""
    name: str
    description: str = ""
    enabled: bool = True
    mappings: List[OutputMapping] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "enabled": self.enabled,
            "mappings": [m.to_dict() for m in self.mappings],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LiveLinkRoute':
        return cls(
            name=data.get("name", ""),
            description=data.get("description", ""),
            enabled=data.get("enabled", True),
            mappings=[OutputMapping.from_dict(m) for m in data.get("mappings", [])],
        )


class LiveLinkRouter:
    """
    Routes LiveLink blend shape data to node outputs.

    Manages a collection of routes (mapping configurations) and
    applies them to incoming blend shape data.
    """

    def __init__(self, logger=None):
        self.logger = logger
        self.routes: Dict[str, LiveLinkRoute] = {}
        self._output_callback: Optional[Callable[[str, str, float], None]] = None
        self._enabled = True
        self._last_update_time = 0.0
        self._update_count = 0

        # Throttling: minimum time between updates per node/output
        self._throttle_ms = 20  # 50 Hz max
        self._last_output_times: Dict[Tuple[str, str], float] = {}

    def set_output_callback(self, callback: Callable[[str, str, float], None]):
        """
        Set callback for sending output values to nodes.

        Callback signature: (node_id, output_name, value) -> None
        """
        self._output_callback = callback

    def add_route(self, route: LiveLinkRoute):
        """Add a route configuration."""
        self.routes[route.name] = route
        if self.logger:
            self.logger.info(f"LiveLink route added: {route.name} ({len(route.mappings)} mappings)")

    def remove_route(self, name: str) -> bool:
        """Remove a route by name."""
        if name in self.routes:
            del self.routes[name]
            return True
        return False

    def get_route(self, name: str) -> Optional[LiveLinkRoute]:
        """Get a route by name."""
        return self.routes.get(name)

    def enable_route(self, name: str, enabled: bool = True):
        """Enable or disable a route."""
        if name in self.routes:
            self.routes[name].enabled = enabled

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool):
        self._enabled = value

    def process(self, blend_shapes: BlendShapes):
        """
        Process blend shape data and send outputs to nodes.

        This is called for each incoming blend shape update.
        """
        if not self._enabled or not self._output_callback:
            return

        self._last_update_time = time.time()
        self._update_count += 1
        now_ms = time.time() * 1000

        # Process all enabled routes
        for route in self.routes.values():
            if not route.enabled:
                continue

            for mapping in route.mappings:
                if not mapping.enabled or not mapping.node_id or not mapping.output_name:
                    continue

                # Check throttle
                key = (mapping.node_id, mapping.output_name)
                last_time = self._last_output_times.get(key, 0)
                if now_ms - last_time < self._throttle_ms:
                    continue

                # Compute output value
                value = mapping.compute(blend_shapes)

                # Send to node
                try:
                    self._output_callback(mapping.node_id, mapping.output_name, value)
                    self._last_output_times[key] = now_ms
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"LiveLink output error: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Get router status."""
        return {
            "enabled": self._enabled,
            "route_count": len(self.routes),
            "routes": {name: route.enabled for name, route in self.routes.items()},
            "last_update_time": self._last_update_time,
            "update_count": self._update_count,
        }

    def get_routes_config(self) -> List[Dict[str, Any]]:
        """Get all routes as serializable dicts."""
        return [route.to_dict() for route in self.routes.values()]

    def load_routes_config(self, routes_data: List[Dict[str, Any]]):
        """Load routes from serialized config."""
        self.routes.clear()
        for route_data in routes_data:
            route = LiveLinkRoute.from_dict(route_data)
            self.routes[route.name] = route

    def create_default_head_route(self, node_id: str) -> LiveLinkRoute:
        """
        Create a default route for mapping face capture to head node.

        This provides sensible defaults for eye tracking and brow control.
        """
        mappings = [
            # Left eye horizontal (look left/right)
            OutputMapping(
                source="expression",
                expression="eye_look_horizontal_left",
                node_id=node_id,
                output_name="eye_left_lr",
                mapping_type=MappingType.RANGE,
                input_min=-1.0, input_max=1.0,
                output_min=45, output_max=135,  # Servo angle range
                smoothing=0.3,
            ),
            # Left eye vertical (look up/down)
            OutputMapping(
                source="expression",
                expression="eye_look_vertical_left",
                node_id=node_id,
                output_name="eye_left_ud",
                mapping_type=MappingType.RANGE,
                input_min=-1.0, input_max=1.0,
                output_min=45, output_max=135,
                smoothing=0.3,
            ),
            # Left eyelid (blink)
            OutputMapping(
                source="EyeBlinkLeft",
                node_id=node_id,
                output_name="eye_left_lid",
                mapping_type=MappingType.RANGE,
                input_min=0.0, input_max=1.0,
                output_min=0, output_max=90,  # 0=open, 90=closed
                smoothing=0.1,
            ),
            # Right eye horizontal
            OutputMapping(
                source="expression",
                expression="eye_look_horizontal_right",
                node_id=node_id,
                output_name="eye_right_lr",
                mapping_type=MappingType.RANGE,
                input_min=-1.0, input_max=1.0,
                output_min=45, output_max=135,
                smoothing=0.3,
            ),
            # Right eye vertical
            OutputMapping(
                source="expression",
                expression="eye_look_vertical_right",
                node_id=node_id,
                output_name="eye_right_ud",
                mapping_type=MappingType.RANGE,
                input_min=-1.0, input_max=1.0,
                output_min=45, output_max=135,
                smoothing=0.3,
            ),
            # Right eyelid
            OutputMapping(
                source="EyeBlinkRight",
                node_id=node_id,
                output_name="eye_right_lid",
                mapping_type=MappingType.RANGE,
                input_min=0.0, input_max=1.0,
                output_min=0, output_max=90,
                smoothing=0.1,
            ),
            # Left upper flap (brow) - pitch based on brow position
            OutputMapping(
                source="expression",
                expression="brow_left",
                node_id=node_id,
                output_name="upper_flap_left_pitch",
                mapping_type=MappingType.RANGE,
                input_min=-1.0, input_max=1.0,
                output_min=70, output_max=110,  # Slight movement range
                smoothing=0.4,
            ),
            # Right upper flap (brow)
            OutputMapping(
                source="expression",
                expression="brow_right",
                node_id=node_id,
                output_name="upper_flap_right_pitch",
                mapping_type=MappingType.RANGE,
                input_min=-1.0, input_max=1.0,
                output_min=70, output_max=110,
                smoothing=0.4,
            ),
        ]

        route = LiveLinkRoute(
            name=f"head_{node_id}",
            description=f"Default face capture mapping for head node {node_id}",
            enabled=True,
            mappings=mappings,
        )

        return route
