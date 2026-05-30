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
import time
from threading import Lock
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from saint_server.peripheral_model import (
    OPERATOR_CATALOG,
    NodeSheet,
    OperatorNode,
    SystemRouting,
    Wire,
)


# Hot-path INFO logs (per-tick set_ws_input / set_channel emissions) are
# sampled to keep file-handler I/O bounded at high tick rates. We still
# always log the first message after a quiet gap so the operator sees a
# binding light up immediately when they push the stick.
_HOT_LOG_SAMPLE_N = 20      # log 1-of-N during steady streams
_HOT_LOG_IDLE_MS = 500.0    # also log if this much wallclock has passed


# (topic, field) → most recent scalar value, or None if no message yet.
SourceKey = Tuple[str, str]

# (sheet_node_id, ws_input_id) → most recent scalar value pushed by a
# controller. Per-sheet to keep input ids stable across renamed sheets.
WSInputKey = Tuple[str, str]


class RoutingEvaluator:
    """Evaluates routing sheets and emits peripheral / widget outputs."""

    def __init__(
        self,
        ros_bridge: Any,
        send_channel: Callable[[str, str, str, float, str], None],
        peripheral_type_lookup: Callable[[str, str], str],
        logger: Optional[Any] = None,
        on_values_changed: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        self._bridge = ros_bridge
        self._send_channel = send_channel
        self._peripheral_type_lookup = peripheral_type_lookup
        self._logger = logger
        # Optional sink for per-evaluation value snapshots. Called from
        # the ROS callback thread; the websocket handler wraps it with a
        # throttle + async-loop bounce so the routing UI can live-display
        # values flowing through each node.
        self._on_values_changed = on_values_changed

        self._lock = Lock()
        # Latest scalar value seen for each (topic, field).
        self._sources: Dict[SourceKey, float] = {}
        # Latest scalar value pushed by a controller into each WS input.
        self._ws_input_values: Dict[WSInputKey, float] = {}
        # Latest frame value pushed by an animation player, keyed by
        # URDF joint name. One cache shared across every sheet — any
        # input whose ``kind == "urdf_joint"`` and whose ``joint`` field
        # matches the key reads its value from here. The animation
        # player calls ``set_urdf_joint_value`` once per tick per
        # active value track.
        self._urdf_joint_values: Dict[str, float] = {}
        # Currently-subscribed topics on behalf of the routing graph.
        self._subscribed_topics: Set[str] = set()
        # Snapshot of the routing graph used for evaluation; refreshed
        # on every reconcile().
        self._routing: Optional[SystemRouting] = None
        # Latest widget input values — exposed for the dashboard sheet
        # so widgets can render server-evaluated readings.
        self._widget_values: Dict[Tuple[str, str], float] = {}
        # Per-sheet value snapshot: sheet_node_id → {kind: {id: value}}
        # where `kind` is one of "inputs", "ws_inputs", "operators",
        # "outputs", "widgets". Refreshed on each _evaluate_sheet pass
        # and shipped to the UI via _on_values_changed.
        self._sheet_values: Dict[str, Dict[str, Dict[str, float]]] = {}
        # System-wide E-Stop latch mirror. Driven by the websocket
        # handler's `estop` action so the evaluator can suppress
        # peripheral / output sink writes while estop is engaged.
        # Widgets still get their cached value updates — the dashboard
        # should keep rendering last-known telemetry, just nothing
        # commands motors. The firmware-level estop has already
        # neutralized the nodes; this gate just stops us from
        # republishing stale operator input that would resume motion
        # the moment estop releases.
        self._estop_active: bool = False
        # Hot-path log sampling state (see _hot_log).
        self._hot_log_count = 0
        self._hot_log_last_ms = 0.0

    # ── public API ──────────────────────────────────────────────────

    def set_estop_active(self, active: bool) -> None:
        """Toggle the e-stop gate. While active, ``_dispatch_sink``
        suppresses peripheral and output sink writes — no ROS publishes
        for motor commands or any other downstream control. Widget
        sinks still update so the dashboard reflects the latched
        state. Called from the websocket handler's `estop` action.
        """
        prev = self._estop_active
        self._estop_active = bool(active)
        if prev != self._estop_active:
            self._log("warn",
                      f"Routing evaluator estop gate: "
                      f"{'ENGAGED — suppressing peripheral/output writes' if self._estop_active else 'RELEASED'}")

    def reconcile(self, routing: SystemRouting) -> None:
        """Adopt the latest routing graph and resync ROS subscriptions."""
        needed: Set[str] = set()
        for sheet in routing.sheets.values():
            for inp in sheet.inputs:
                # Only ROS-topic inputs drive subscription state; URDF-
                # joint inputs read from the animation cache and don't
                # need a ROS subscription.
                if inp.kind == "topic" and inp.topic:
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

    def set_ws_input(self, sheet_id: str, input_id: str, value: float) -> bool:
        """Push a controller-driven value into a sheet's WS input.

        Hot path on every gamepad-axis tick. Updates the cache, then
        re-evaluates the owning sheet so peripheral sinks and ROS-output
        taps see the new value immediately. Returns False if the sheet
        or input doesn't exist so the caller can surface a useful error.
        """
        routing = self._routing
        if routing is None:
            self._log("warn",
                      f"set_ws_input {sheet_id}/{input_id}: evaluator has no "
                      "routing snapshot yet (reconcile not run?)")
            return False
        sheet = routing.sheets.get(sheet_id)
        if sheet is None:
            self._log("warn",
                      f"set_ws_input: sheet '{sheet_id}' not found "
                      f"(have: {list(routing.sheets.keys())})")
            return False
        if sheet.find_ws_input(input_id) is None:
            ws_ids = [w.id for w in sheet.ws_inputs]
            self._log("warn",
                      f"set_ws_input: ws_input '{input_id}' not on sheet "
                      f"'{sheet_id}' (have: {ws_ids})")
            return False
        try:
            scalar = float(value)
        except (TypeError, ValueError):
            self._log("warn", f"set_ws_input: non-numeric value {value!r}")
            return False

        with self._lock:
            self._ws_input_values[(sheet_id, input_id)] = scalar

        # Visible-by-default so the operator can see WS-input pushes in
        # the live log alongside set_topic_channel events. Sampled — see
        # _hot_log — to keep file-handler I/O bounded during stick bursts.
        self._hot_log(f"set_ws_input {sheet_id}/{input_id} = {scalar}")

        try:
            self._evaluate_sheet(sheet)
        except Exception as e:
            self._log("error",
                      f"Sheet '{sheet.node_id}' evaluation failed: {e}")
            return True

        if self._on_values_changed is not None:
            try:
                self._on_values_changed(self.get_value_snapshot())
            except Exception as e:
                self._log("error", f"routing_values broadcast failed: {e}")
        return True

    def set_urdf_joint_value(self, joint: str, value: float) -> bool:
        """Push a URDF-joint setpoint (typically from an animation
        player tick) into the source cache.

        Any ``InputNode`` with ``kind="urdf_joint"`` and matching
        ``joint`` reads from this cache, so the value flows through
        wires the same way a ROS-topic input does. Joints are global
        identifiers — multiple sheets can reference the same joint and
        they all see the same value per tick.

        Returns False if the routing graph isn't loaded yet so the
        caller can decide whether to retry later.
        """
        routing = self._routing
        if routing is None:
            return False
        try:
            scalar = float(value)
        except (TypeError, ValueError):
            self._log("warn",
                      f"set_urdf_joint_value: non-numeric {joint}={value!r}")
            return False

        with self._lock:
            self._urdf_joint_values[joint] = scalar

        # Find every sheet wiring an InputNode for this joint and re-evaluate.
        touched: List[NodeSheet] = []
        for sheet in routing.sheets.values():
            for inp in sheet.inputs:
                if inp.kind == "urdf_joint" and inp.joint == joint:
                    touched.append(sheet)
                    break
        for sheet in touched:
            try:
                self._evaluate_sheet(sheet)
            except Exception as e:
                self._log("error",
                          f"Sheet '{sheet.node_id}' evaluation failed: {e}")

        if touched and self._on_values_changed is not None:
            try:
                self._on_values_changed(self.get_value_snapshot())
            except Exception as e:
                self._log("error", f"routing_values broadcast failed: {e}")
        return True

    def clear_urdf_joint_value(self, joint: Optional[str] = None) -> None:
        """Drop cached URDF joint values. If ``joint`` is None, clear
        the entire cache (typically called when no animations are
        playing).
        """
        with self._lock:
            if joint is not None:
                self._urdf_joint_values.pop(joint, None)
            else:
                self._urdf_joint_values.clear()

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

        # Hand the freshly computed snapshot off to the UI broadcaster.
        if touched_sheets and self._on_values_changed is not None:
            try:
                self._on_values_changed(self.get_value_snapshot())
            except Exception as e:
                self._log("error", f"routing_values broadcast failed: {e}")

    def get_value_snapshot(self) -> Dict[str, Any]:
        """Most recent per-sheet values, formatted for the UI."""
        # Shallow-copy each sheet's value buckets so the consumer can
        # serialize without worrying about concurrent mutation.
        out: Dict[str, Any] = {"sheets": {}}
        for sheet_id, kinds in self._sheet_values.items():
            out["sheets"][sheet_id] = {k: dict(v) for k, v in kinds.items()}
        return out

    def widget_value(self, widget_id: str, input_id: str) -> Optional[float]:
        """Last server-evaluated value seen for a dashboard widget input."""
        return self._widget_values.get((widget_id, input_id))

    # ── evaluation ──────────────────────────────────────────────────

    def _evaluate_sheet(self, sheet: NodeSheet) -> None:
        # Memoize operator outputs so each operator is computed at most
        # once per evaluation pass. `visiting` doubles as a cycle guard.
        op_cache: Dict[str, float] = {}
        visiting: Set[str] = set()
        # Output node values for this tick — needed before we can resolve
        # any wire whose source.kind == "output" (the tap path that feeds
        # WS-input → math → ROS-publish → peripheral on one sheet).
        output_cache: Dict[str, float] = {}
        # Per-evaluation buckets — copied into self._sheet_values at the
        # end so the UI snapshot reflects this pass atomically.
        sheet_input_vals: Dict[str, float] = {}
        sheet_ws_input_vals: Dict[str, float] = {}
        sheet_output_vals: Dict[str, float] = {}
        sheet_widget_vals: Dict[str, float] = {}
        # Peripheral input-pin values — the value the routing graph just
        # commanded onto each (node_id, peripheral_id, channel_id) sink.
        # Keyed by "<node_id>/<peripheral_id>/<channel_id>" so the UI
        # can light up the actual receiving pin on the peripheral card.
        sheet_peripheral_vals: Dict[str, float] = {}

        # Capture input values so the UI can show what's flowing in.
        # URDF-joint inputs read from the joint cache; topic inputs from
        # the ROS source cache. Both surface in the same `inputs` bucket
        # since the UI only cares about the InputNode id.
        for inp in sheet.inputs:
            if inp.kind == "urdf_joint":
                v = self._urdf_joint_values.get(inp.joint)
            else:
                v = self._sources.get((inp.topic, inp.field))
            if v is not None:
                sheet_input_vals[inp.id] = v
        for ws in sheet.ws_inputs:
            v = self._ws_input_values.get((sheet.node_id, ws.id))
            if v is not None:
                sheet_ws_input_vals[ws.id] = v

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
                # Wire references an operator that isn't on this sheet —
                # almost always a stale id left behind by a delete-and-
                # recreate. Loud because the silent 0 here is the
                # ultimate cause of "math chain returns nothing".
                known = [o.id for o in sheet.operators]
                self._log("warn",
                          f"eval_operator: op '{op_id}' missing on sheet "
                          f"'{sheet.node_id}' (have: {known}). "
                          "Treating as 0 — fix the stale wire.")
                return 0.0
            visiting.add(op_id)
            try:
                inputs = self._collect_operator_inputs(sheet, op_node, eval_operator,
                                                      output_cache)
                value = _apply_operator(op_node, inputs)
            finally:
                visiting.discard(op_id)
            op_cache[op_id] = value
            self._log("debug",
                      f"eval {op_node.op}({op_id}) inputs={inputs} → {value}")
            return value

        # Pass 1: resolve and dispatch outputs first so output_cache is
        # populated before any peripheral wire tries to tap an output as
        # its source.
        for wire in sheet.wires:
            if wire.sink.kind != "output":
                continue
            value = self._resolve_source(sheet, wire.source, eval_operator,
                                         output_cache)
            if value is None:
                continue
            if wire.sink.parts:
                output_cache[wire.sink.parts[0]] = float(value)
                sheet_output_vals[wire.sink.parts[0]] = float(value)
            self._dispatch_sink(sheet, wire, value)

        # Pass 2: peripheral and widget sinks; these can reference both
        # operators and outputs as sources.
        for wire in sheet.wires:
            if wire.sink.kind not in ("peripheral", "widget"):
                continue
            value = self._resolve_source(sheet, wire.source, eval_operator,
                                         output_cache)
            if value is None:
                continue
            self._dispatch_sink(sheet, wire, value)
            if wire.sink.kind == "widget" and len(wire.sink.parts) >= 2:
                sheet_widget_vals[f"{wire.sink.parts[0]}/{wire.sink.parts[1]}"] = float(value)
            elif wire.sink.kind == "peripheral" and len(wire.sink.parts) >= 3:
                key = (f"{wire.sink.parts[0]}/"
                       f"{wire.sink.parts[1]}/"
                       f"{wire.sink.parts[2]}")
                sheet_peripheral_vals[key] = float(value)

        # Pass 3: eagerly evaluate every operator on the sheet so the
        # UI can show intermediate values even when nothing downstream
        # is pulling them yet. eval_operator memoizes via op_cache, so
        # operators already reached by a sink in Pass 1/2 are no-ops
        # here.
        for op in sheet.operators:
            try:
                eval_operator(op.id)
            except Exception as e:
                self._log("error",
                          f"Operator '{op.id}' eager-eval failed: {e}")

        # Atomically swap the per-sheet snapshot — operator values come
        # from the memoized cache so even operators that weren't pulled
        # by a sink (orphaned ops) are not surfaced.
        self._sheet_values[sheet.node_id] = {
            "inputs":      sheet_input_vals,
            "ws_inputs":   sheet_ws_input_vals,
            "operators":   {k: float(v) for k, v in op_cache.items()},
            "outputs":     sheet_output_vals,
            "widgets":     sheet_widget_vals,
            "peripherals": sheet_peripheral_vals,
        }

    def _collect_operator_inputs(self, sheet: NodeSheet, op_node: OperatorNode,
                                 eval_operator: Callable[[str], float],
                                 output_cache: Dict[str, float],
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
            value = self._resolve_source(sheet, wire.source, eval_operator,
                                         output_cache)
            if value is not None:
                inputs[pin] = value
        return inputs

    def _resolve_source(self, sheet: NodeSheet, source,
                        eval_operator: Callable[[str], float],
                        output_cache: Dict[str, float],
                        ) -> Optional[float]:
        if source.kind == "input":
            if not source.parts:
                return None
            inp = sheet.find_input(source.parts[0])
            if inp is None:
                return None
            if inp.kind == "urdf_joint":
                return self._urdf_joint_values.get(inp.joint)
            return self._sources.get((inp.topic, inp.field))
        if source.kind == "ws_input":
            if not source.parts:
                return None
            return self._ws_input_values.get((sheet.node_id, source.parts[0]))
        if source.kind == "operator":
            if not source.parts:
                return None
            return eval_operator(source.parts[0])
        if source.kind == "output":
            # Tap an Output node: its value was computed in pass 1 of
            # _evaluate_sheet. Returns None if this wire's source output
            # has no value yet (no wire feeding the output's sink, or
            # value was None for any reason).
            if not source.parts:
                return None
            return output_cache.get(source.parts[0])
        # peripheral / widget sources are not supported as wire sources today.
        return None

    def _dispatch_sink(self, sheet: NodeSheet, wire: Wire, value: float) -> None:
        sink = wire.sink
        # E-Stop gate: drop everything that would command actuators
        # (peripheral channels, ROS output publishes). Widget sinks
        # fall through so dashboard cards keep their last value
        # visible. The firmware-level estop already cut motor power
        # on each node; this prevents the next stick tick from
        # republishing stale operator input.
        if self._estop_active and sink.kind in ("peripheral", "output"):
            return
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
                # Include node_id in the log: peripheral_ids are scoped
                # per-node so e.g. "roboclaw-1" on the Left vs Right Track
                # Drive nodes both render as "roboclaw-1/motor" without
                # this prefix, which made the side-by-side log impossible
                # to read on a tank with one roboclaw per track. Sampled
                # via _hot_log so 50 Hz binding streams don't saturate
                # the daily file handler.
                self._hot_log(
                    f"set_channel {node_id}/{peripheral_id}/{channel_id} = {value:.3f}")
            except Exception as e:
                self._log("error",
                          f"Failed to send channel {node_id}/{peripheral_id}/{channel_id}: {e}")
        elif sink.kind == "widget":
            if len(sink.parts) < 2:
                return
            self._widget_values[(sink.parts[0], sink.parts[1])] = float(value)
        elif sink.kind == "output":
            # Output nodes republish onto a ROS topic channel via the
            # bridge's per-topic buffer — same path the controller
            # bindings use, so an Output is effectively the operator's
            # way to turn computed values back into ROS state.
            if not sink.parts:
                return
            out_node = sheet.find_output(sink.parts[0])
            if out_node is None or not out_node.topic:
                return
            try:
                self._bridge.set_topic_channel(
                    out_node.topic, out_node.field, float(value),
                    client_id="_routing_evaluator",
                )
            except Exception as e:
                self._log("error",
                          f"Failed to publish output {out_node.id} → {out_node.topic}.{out_node.field}: {e}")

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
            from saint_server.log_level import log_at
            log_at(self._logger, level, message)

    def _hot_log(self, message: str) -> None:
        """Sampled INFO log for per-tick lines. Logs 1-of-N during steady
        streams, plus always logs the first message after an idle gap so
        a binding firing fresh is still visible immediately."""
        self._hot_log_count += 1
        now_ms = time.monotonic() * 1000.0
        if (now_ms - self._hot_log_last_ms >= _HOT_LOG_IDLE_MS
                or self._hot_log_count % _HOT_LOG_SAMPLE_N == 0):
            self._hot_log_last_ms = now_ms
            self._log("info", message)


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
        # Accept both the current param name ("clamp") and the legacy
        # name ("clamp_output") so older saved sheets keep working.
        clamp = node.params.get("clamp", node.params.get("clamp_output", True))
        if clamp:
            in_min_a, in_max_a = (in_lo, in_hi) if in_lo <= in_hi else (in_hi, in_lo)
            v = max(in_min_a, min(in_max_a, v))
        if in_hi == in_lo:
            return out_lo
        return out_lo + (v - in_lo) * (out_hi - out_lo) / (in_hi - in_lo)
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
    if op == "select":
        a = float(inputs.get("a", 0.0))
        b = float(inputs.get("b", 0.0))
        prefer_a = bool(node.params.get("prefer_a", False))
        # The preferred input wins whenever it has a meaningful (non-
        # zero) value. Treats values below 1e-6 as "no signal" so a
        # joystick parked at its idle position lets the other source
        # — e.g. an animation — drive through.
        eps = 1e-6
        if prefer_a:
            return a if abs(a) > eps else b
        return b if abs(b) > eps else a
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
