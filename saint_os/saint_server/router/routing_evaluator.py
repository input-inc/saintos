"""SAINT.OS routing graph evaluator.

Walks the per-controller sheets defined in `SystemRouting.sheets`, computes
operator outputs from ROS topic inputs, and emits peripheral commands.

Lifecycle:
  - `reconcile()` is called whenever the routing state changes; it
    refreshes the set of (topic, field) sources we need from ROS and
    ensures the bridge has an internal subscription for each topic.
  - `on_topic_message(topic, data)` is the hot path — fired from the ROS
    callback thread when a state message arrives. It updates the source
    cache and evaluates every sheet that references the topic.

The evaluator is intentionally single-threaded; sheets are small and the
operator catalog is pure functional, so re-evaluating from scratch on
each input tick is cheap and keeps the algorithm trivial.
"""

from __future__ import annotations

import math
from threading import Lock
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from saint_server.peripheral_model import (
    OPERATOR_CATALOG,
    NodeSheet,
    OperatorNode,
    SystemRouting,
    Wire,
)


# (topic, field) → most recent scalar value, or None if no message yet.
SourceKey = Tuple[str, str]


class RoutingEvaluator:
    """Evaluates routing sheets and emits peripheral / widget outputs."""

    def __init__(
        self,
        ros_bridge: Any,
        send_channel: Callable[[str, str, str, float, str], None],
        peripheral_type_lookup: Callable[[str, str], str],
        logger: Optional[Any] = None,
    ):
        self._bridge = ros_bridge
        self._send_channel = send_channel
        self._peripheral_type_lookup = peripheral_type_lookup
        self._logger = logger

        self._lock = Lock()
        # Latest scalar value seen for each (topic, field).
        self._sources: Dict[SourceKey, float] = {}
        # Currently-subscribed topics on behalf of the routing graph.
        self._subscribed_topics: Set[str] = set()
        # Snapshot of the routing graph used for evaluation; refreshed
        # on every reconcile().
        self._routing: Optional[SystemRouting] = None
        # Latest widget input values — exposed for the dashboard sheet
        # so widgets can render server-evaluated readings.
        self._widget_values: Dict[Tuple[str, str], float] = {}

    # ── public API ──────────────────────────────────────────────────

    def reconcile(self, routing: SystemRouting) -> None:
        """Adopt the latest routing graph and resync ROS subscriptions."""
        needed: Set[str] = set()
        for sheet in routing.sheets.values():
            for inp in sheet.inputs:
                if inp.topic:
                    needed.add(inp.topic)

        with self._lock:
            self._routing = routing
            to_add = needed - self._subscribed_topics
            to_drop = self._subscribed_topics - needed
            self._subscribed_topics = set(needed)

        for topic in to_add:
            self._subscribe(topic)
        for topic in to_drop:
            self._unsubscribe(topic)

    def on_topic_message(self, topic: str, data: Dict[str, Any]) -> None:
        """Hot path: a ROS message arrived for `topic`. Re-evaluate any
        sheet that references it.
        """
        routing = self._routing
        if routing is None:
            return

        # Update the source cache for every (topic, field) pair the
        # routing graph cares about — but only for this topic. Then
        # evaluate just the sheets that touch this topic.
        touched_sheets: List[NodeSheet] = []
        with self._lock:
            for sheet in routing.sheets.values():
                used_here = False
                for inp in sheet.inputs:
                    if inp.topic != topic:
                        continue
                    used_here = True
                    value = _extract_field(data, inp.field)
                    if value is not None:
                        self._sources[(inp.topic, inp.field)] = value
                if used_here:
                    touched_sheets.append(sheet)

        for sheet in touched_sheets:
            try:
                self._evaluate_sheet(sheet)
            except Exception as e:
                self._log("error",
                          f"Sheet '{sheet.node_id}' evaluation failed: {e}")

    def widget_value(self, widget_id: str, input_id: str) -> Optional[float]:
        """Last server-evaluated value seen for a dashboard widget input."""
        return self._widget_values.get((widget_id, input_id))

    # ── evaluation ──────────────────────────────────────────────────

    def _evaluate_sheet(self, sheet: NodeSheet) -> None:
        # Memoize operator outputs so each operator is computed at most
        # once per evaluation pass. `visiting` doubles as a cycle guard.
        op_cache: Dict[str, float] = {}
        visiting: Set[str] = set()

        def eval_operator(op_id: str) -> float:
            if op_id in op_cache:
                return op_cache[op_id]
            if op_id in visiting:
                # Cycle: break gracefully with 0.0 and log.
                self._log("warn",
                          f"Routing cycle detected at operator {op_id} on sheet {sheet.node_id}")
                return 0.0
            op_node = sheet.find_operator(op_id)
            if op_node is None:
                return 0.0
            visiting.add(op_id)
            try:
                inputs = self._collect_operator_inputs(sheet, op_node, eval_operator)
                value = _apply_operator(op_node, inputs)
            finally:
                visiting.discard(op_id)
            op_cache[op_id] = value
            return value

        # Walk all wires that terminate at a peripheral or widget sink
        # and resolve each source. Operator outputs are computed on-
        # demand via eval_operator.
        for wire in sheet.wires:
            if wire.sink.kind not in ("peripheral", "widget"):
                continue
            value = self._resolve_source(sheet, wire.source, eval_operator)
            if value is None:
                continue
            self._dispatch_sink(sheet, wire, value)

    def _collect_operator_inputs(self, sheet: NodeSheet, op_node: OperatorNode,
                                 eval_operator: Callable[[str], float]
                                 ) -> Dict[str, float]:
        op_type = OPERATOR_CATALOG.get(op_node.op)
        if op_type is None:
            return {}
        # Start with operator-spec defaults overridden by per-node defaults.
        inputs: Dict[str, float] = {}
        for spec in op_type.inputs:
            if spec.id in op_node.defaults:
                inputs[spec.id] = op_node.defaults[spec.id]
            else:
                inputs[spec.id] = spec.default

        # Apply any wires feeding this operator's input pins.
        for wire in sheet.wires:
            sink = wire.sink
            if sink.kind != "operator":
                continue
            if not sink.parts or sink.parts[0] != op_node.id:
                continue
            pin = sink.parts[1] if len(sink.parts) > 1 else None
            if pin is None:
                continue
            value = self._resolve_source(sheet, wire.source, eval_operator)
            if value is not None:
                inputs[pin] = value
        return inputs

    def _resolve_source(self, sheet: NodeSheet, source,
                        eval_operator: Callable[[str], float]
                        ) -> Optional[float]:
        if source.kind == "input":
            if not source.parts:
                return None
            inp = sheet.find_input(source.parts[0])
            if inp is None:
                return None
            return self._sources.get((inp.topic, inp.field))
        if source.kind == "operator":
            if not source.parts:
                return None
            return eval_operator(source.parts[0])
        # peripheral / widget sources are not supported as wire sources today.
        return None

    def _dispatch_sink(self, sheet: NodeSheet, wire: Wire, value: float) -> None:
        sink = wire.sink
        if sink.kind == "peripheral":
            if len(sink.parts) < 3:
                return
            node_id, peripheral_id, channel_id = sink.parts[0], sink.parts[1], sink.parts[2]
            ptype = ""
            try:
                ptype = self._peripheral_type_lookup(node_id, peripheral_id) or ""
            except Exception:
                ptype = ""
            try:
                self._send_channel(node_id, peripheral_id, channel_id, float(value), ptype)
            except Exception as e:
                self._log("error",
                          f"Failed to send channel {node_id}/{peripheral_id}/{channel_id}: {e}")
        elif sink.kind == "widget":
            if len(sink.parts) < 2:
                return
            self._widget_values[(sink.parts[0], sink.parts[1])] = float(value)

    # ── subscription plumbing ───────────────────────────────────────

    def _subscribe(self, topic: str) -> None:
        if not self._bridge:
            return
        try:
            self._bridge.subscribe(topic, _EVALUATOR_CLIENT_ID)
            self._log("info", f"Routing: subscribed to {topic}")
        except Exception as e:
            self._log("warn", f"Routing subscribe failed for {topic}: {e}")

    def _unsubscribe(self, topic: str) -> None:
        if not self._bridge:
            return
        try:
            self._bridge.unsubscribe(topic, _EVALUATOR_CLIENT_ID)
        except Exception as e:
            self._log("warn", f"Routing unsubscribe failed for {topic}: {e}")

    def _log(self, level: str, message: str) -> None:
        if self._logger:
            try:
                getattr(self._logger, level)(message)
            except Exception:
                pass


_EVALUATOR_CLIENT_ID = "_routing_evaluator"


# ── operator implementations ───────────────────────────────────────


def _apply_operator(node: OperatorNode, inputs: Dict[str, float]) -> float:
    op = node.op
    a = float(inputs.get("a", 0.0))
    b = float(inputs.get("b", 0.0))
    v = float(inputs.get("value", 0.0))
    if op == "add":
        return a + b
    if op == "subtract":
        return a - b
    if op == "multiply":
        return a * b
    if op == "scale":
        return v * float(inputs.get("factor", 1.0))
    if op == "max":
        return max(a, b)
    if op == "min":
        return min(a, b)
    if op == "clamp":
        lo = float(inputs.get("min", -1.0))
        hi = float(inputs.get("max", 1.0))
        if lo > hi:
            lo, hi = hi, lo
        return max(lo, min(hi, v))
    if op == "map_range":
        in_lo = float(inputs.get("in_min", -1.0))
        in_hi = float(inputs.get("in_max", 1.0))
        out_lo = float(inputs.get("out_min", 0.0))
        out_hi = float(inputs.get("out_max", 1.0))
        if in_hi == in_lo:
            mapped = out_lo
        else:
            mapped = out_lo + (v - in_lo) * (out_hi - out_lo) / (in_hi - in_lo)
        if node.params.get("clamp_output", True):
            mapped_lo, mapped_hi = (out_lo, out_hi) if out_lo <= out_hi else (out_hi, out_lo)
            mapped = max(mapped_lo, min(mapped_hi, mapped))
        return mapped
    if op == "lerp":
        t = float(inputs.get("t", 0.5))
        return a + (b - a) * t
    if op == "invert":
        return -v
    if op == "abs":
        return abs(v)
    if op == "deadband":
        thr = abs(float(inputs.get("threshold", 0.0)))
        return v if abs(v) > thr else 0.0
    if op == "curve":
        exp = float(inputs.get("exponent", 1.0))
        sign = -1.0 if v < 0 else 1.0
        return sign * math.pow(abs(v), exp)
    return 0.0


# ── topic field extraction ─────────────────────────────────────────


def _extract_field(data: Any, field: str) -> Optional[float]:
    """Extract `field` from a serialized ROS message dict.

    Field paths use dotted segments plus optional [index] suffixes, e.g.
    "axes[0]", "linear.x", "value". Returns None if the path doesn't
    resolve or the leaf isn't numeric.
    """
    if data is None:
        return None
    cur: Any = data
    if not field:
        # Whole-message scalar (e.g. std_msgs/Float32 serializer wraps it).
        return _coerce_scalar(cur)
    # Tokenize: split on '.' then handle '[N]' suffixes per segment.
    for raw_seg in field.split("."):
        if cur is None:
            return None
        seg = raw_seg
        # Pull off any [N] indices: name[3][1] etc.
        index_starts: List[int] = []
        bracket = seg.find("[")
        if bracket != -1:
            indices_str = seg[bracket:]
            seg = seg[:bracket]
            # Parse [N][M]...
            i = 0
            while i < len(indices_str):
                if indices_str[i] != "[":
                    break
                close = indices_str.find("]", i)
                if close == -1:
                    return None
                try:
                    index_starts.append(int(indices_str[i + 1:close]))
                except ValueError:
                    return None
                i = close + 1
        if seg:
            if isinstance(cur, dict):
                cur = cur.get(seg)
            else:
                cur = getattr(cur, seg, None)
        for idx in index_starts:
            try:
                cur = cur[idx]
            except (TypeError, IndexError, KeyError):
                return None
    return _coerce_scalar(cur)


def _coerce_scalar(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    return None
