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
        # "Compiled" wire topology, rebuilt only on reconcile() (i.e.
        # when the operator edits the wiring) — never on the per-tick
        # hot path. sheet.node_id → _SheetWireIndex. Replaces the
        # O(operators × wires) full-wire rescan that _evaluate_sheet /
        # _collect_operator_inputs used to do on every value tick: the
        # graph structure is static between edits, so we derive it once
        # and the tick path just runs it. See _build_wire_index.
        self._wire_index: Dict[str, "_SheetWireIndex"] = {}
        # Cross-tick incremental-eval state (only used for sheets whose
        # index.incremental_ok is True). Per sheet: the operator output
        # values and the leaf (ws_input/topic/joint) values from the last
        # evaluation. On the next tick we diff the leaves; operators whose
        # leaf-dependency set didn't change are seeded from _op_persist
        # and skip recomputation. Reset on reconcile (structure changed →
        # caches invalid).
        self._op_persist: Dict[str, Dict[str, float]] = {}
        self._leaf_persist: Dict[str, Dict[tuple, float]] = {}
        # Latest widget input values — exposed for the dashboard sheet
        # so widgets can render server-evaluated readings.
        self._widget_values: Dict[Tuple[str, str], float] = {}
        # Per-sheet value snapshot: sheet_node_id → {kind: {id: value}}
        # where `kind` is one of "inputs", "ws_inputs", "operators",
        # "outputs", "widgets", "signals". Refreshed on each
        # _evaluate_sheet pass and shipped to the UI via
        # _on_values_changed.
        self._sheet_values: Dict[str, Dict[str, Dict[str, float]]] = {}
        # Global named-signal table. Cross-sheet floats: when a wire on
        # sheet A sinks to kind="signal" with parts=[name], the value
        # lands here under `name`; when a wire on sheet B sources from
        # kind="signal" with the same name, it reads this. Survives
        # across ticks (a signal holds its last value until rewritten)
        # but resets on reconcile() so a structural reload doesn't
        # carry stale state.
        self._signals: Dict[str, float] = {}
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
        # Last value actually pushed to each peripheral channel, keyed by
        # (node_id, peripheral_id, channel_id). A sheet re-dispatches ALL
        # its peripheral sinks on every evaluation, so without this a
        # 60 fps animation republishes every static channel 60×/s and
        # floods /control (depth-1, newest-wins) — clobbering the one
        # channel that's actually moving. We gate on change so only moving
        # channels hit the wire. Reset on reconcile (wiring changed) and
        # on estop release (so held values re-arm the hardware).
        self._last_channel_sent: Dict[Tuple[str, str, str], float] = {}
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
            # Drop the change-gate cache on any transition. On release the
            # held values must re-send to re-arm the hardware (the gate
            # would otherwise swallow them as "unchanged" vs the pre-estop
            # send); on engage it just clears stale state.
            self._last_channel_sent.clear()
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
            # Compile the wire topology once per wiring change. Cheap
            # here (off the hot path); saves a full-wire rescan per
            # operator on every value tick.
            self._wire_index = {
                sheet.node_id: _build_wire_index(sheet)
                for sheet in routing.sheets.values()
            }
            # Structure changed — last-tick value caches are no longer
            # valid against the new operator/leaf set.
            self._op_persist = {}
            self._leaf_persist = {}
            # Rewiring can move/rename channels; force a fresh send to
            # every peripheral sink on the next eval.
            self._last_channel_sent = {}
            to_add = needed - self._subscribed_topics
            to_drop = self._subscribed_topics - needed
            self._subscribed_topics = set(needed)
            # Drop signal values for names that no longer have any
            # writer on the graph. Keeps the table from accumulating
            # ghost entries from deleted sheets / renamed signals.
            # A signal is "live" if any sheet has a SignalNode by that
            # name OR any wire sources/sinks reference it.
            live_names: Set[str] = set()
            for sheet in routing.sheets.values():
                for s in getattr(sheet, "signals", []):
                    if s.name: live_names.add(s.name)
                for w in sheet.wires:
                    for ep in (w.source, w.sink):
                        if ep.kind == "signal" and ep.parts:
                            live_names.add(ep.parts[0])
            self._signals = {k: v for k, v in self._signals.items() if k in live_names}

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

    def apply_animation_frame(
        self,
        joint_values: Dict[str, float],
        ws_values: Optional[Dict[Tuple[str, str], float]] = None,
    ) -> bool:
        """Apply a whole animation tick at once.

        An animation frame delivers many setpoints that are all known at
        the same instant — every URDF joint (and optionally every
        ws_input) a player's value tracks sampled this tick. Pushing them
        one at a time via :meth:`set_urdf_joint_value` re-evaluates each
        shared sheet AND rebuilds + broadcasts the full value snapshot
        once *per* setpoint, so an N-joint rig on one sheet costs ~N sheet
        evaluations and ~N snapshot builds per frame — all redundant.

        This collapses the frame into: write every cache once, evaluate
        the UNION of touched sheets exactly once, broadcast exactly once.
        The observable result (final sink values + snapshot) is identical
        to applying the same values per-track — see
        ``test_animation_frame_equivalence``.

        ``joint_values`` maps URDF joint name → setpoint.
        ``ws_values`` maps (sheet_id, input_id) → value (for ws_input
        value tracks). Returns False only if no routing graph is loaded.
        """
        routing = self._routing
        if routing is None:
            return False
        if not joint_values and not ws_values:
            return True

        # 1) Coerce + write every cache under one lock.
        clean_joints: Dict[str, float] = {}
        for joint, value in joint_values.items():
            try:
                clean_joints[joint] = float(value)
            except (TypeError, ValueError):
                self._log("warn",
                          f"apply_animation_frame: non-numeric {joint}={value!r}")
        clean_ws: Dict[Tuple[str, str], float] = {}
        if ws_values:
            for key, value in ws_values.items():
                try:
                    clean_ws[key] = float(value)
                except (TypeError, ValueError):
                    self._log("warn",
                              f"apply_animation_frame: non-numeric ws {key}={value!r}")

        with self._lock:
            self._urdf_joint_values.update(clean_joints)
            for key, value in clean_ws.items():
                self._ws_input_values[key] = value

        # 2) Union of sheets touched by any setpoint in this frame.
        touched: List[NodeSheet] = []
        seen: Set[str] = set()
        ws_sheets = {sid for (sid, _iid) in clean_ws}
        for sheet in routing.sheets.values():
            if sheet.node_id in seen:
                continue
            hit = sheet.node_id in ws_sheets
            if not hit:
                for inp in sheet.inputs:
                    if inp.kind == "urdf_joint" and inp.joint in clean_joints:
                        hit = True
                        break
            if hit:
                touched.append(sheet)
                seen.add(sheet.node_id)

        # 3) Evaluate each touched sheet exactly once.
        for sheet in touched:
            try:
                self._evaluate_sheet(sheet)
            except Exception as e:
                self._log("error",
                          f"Sheet '{sheet.node_id}' evaluation failed: {e}")

        # 4) Broadcast exactly once.
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
        # Wire topology compiled at reconcile(); lazy-build as a fallback
        # if a sheet is somehow evaluated before it was indexed.
        idx = self._wire_index.get(sheet.node_id)
        if idx is None:
            idx = _build_wire_index(sheet)
            self._wire_index[sheet.node_id] = idx
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

        # Incremental eval (leaf-pure sheets only): seed op_cache with
        # last tick's value for every operator whose leaf dependencies
        # are unchanged. eval_operator then returns those immediately
        # (cache hit) instead of re-resolving the chain — so pushing one
        # axis on an 18-channel sheet recomputes only that axis's chain.
        cur_leaves: Optional[Dict[tuple, float]] = None
        if idx.incremental_ok:
            cur_leaves = {}
            for iid, v in sheet_input_vals.items():
                cur_leaves[("i", iid)] = v
            for wid, v in sheet_ws_input_vals.items():
                cur_leaves[("w", wid)] = v
            prev_leaves = self._leaf_persist.get(sheet.node_id)
            prev_ops = self._op_persist.get(sheet.node_id)
            if prev_leaves is not None and prev_ops is not None:
                dirty = {
                    k for k in (cur_leaves.keys() | prev_leaves.keys())
                    if cur_leaves.get(k) != prev_leaves.get(k)
                }
                for op in sheet.operators:
                    deps = idx.op_leaf_deps.get(op.id)
                    if (deps is not None and op.id in prev_ops
                            and deps.isdisjoint(dirty)):
                        op_cache[op.id] = prev_ops[op.id]

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
                inputs = self._collect_operator_inputs(
                    sheet, op_node, eval_operator, output_cache,
                    idx.op_input_wires.get(op_id, ()))
                value = _apply_operator(op_node, inputs)
            finally:
                visiting.discard(op_id)
            op_cache[op_id] = value
            self._log("debug",
                      f"eval {op_node.op}({op_id}) inputs={inputs} → {value}")
            return value

        # Pass 0: write to the global signal table from any wire whose
        # sink is a signal endpoint. Doing this before the output pass
        # means an Output on the same sheet can also tap a freshly-
        # written signal via its `out` source — and it means signals
        # written here are visible to operator chains on OTHER sheets
        # this tick (sheet iteration order in evaluate() determines
        # last-writer-wins between conflicting writers).
        for wire in idx.signal_sinks:
            value = self._resolve_source(sheet, wire.source, eval_operator,
                                         output_cache)
            if value is None or not wire.sink.parts:
                continue
            self._signals[wire.sink.parts[0]] = float(value)

        # Pass 1: resolve and dispatch outputs first so output_cache is
        # populated before any peripheral wire tries to tap an output as
        # its source.
        for wire in idx.output_sinks:
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
        for wire in idx.perph_widget_sinks:
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

        # Per-sheet signal values — pull only the signals this sheet
        # actually declares so the snapshot is locally relevant. The
        # canonical global map lives at self._signals; this is a
        # projection for the UI.
        sheet_signal_vals: Dict[str, float] = {}
        for s in getattr(sheet, "signals", []):
            v = self._signals.get(s.name)
            if v is not None:
                sheet_signal_vals[s.id] = float(v)

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
            "signals":     sheet_signal_vals,
        }

        # Persist this tick's operator + leaf values for next-tick
        # incremental seeding. op_cache now holds every operator (Pass 3
        # eager-evaluated them all), so clean ops next tick get a correct
        # seed.
        if idx.incremental_ok and cur_leaves is not None:
            self._op_persist[sheet.node_id] = dict(op_cache)
            self._leaf_persist[sheet.node_id] = cur_leaves

    def _collect_operator_inputs(self, sheet: NodeSheet, op_node: OperatorNode,
                                 eval_operator: Callable[[str], float],
                                 output_cache: Dict[str, float],
                                 op_wires,
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

        # Apply the wires feeding this operator's input pins. `op_wires`
        # is the precompiled list of operator-sink wires already filtered
        # to this op (see _build_wire_index) — no full-wire rescan.
        for wire in op_wires:
            sink = wire.sink
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
        if source.kind == "signal":
            # Cross-sheet named float. Returns None until SOMETHING has
            # written this signal (this tick or a prior one); the IF
            # gate's else branch is the right way to model "until set,
            # behave as X".
            if not source.parts:
                return None
            v = self._signals.get(source.parts[0])
            return None if v is None else float(v)
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
            # Change-gate (see _last_channel_sent). A sheet re-dispatches
            # every peripheral sink on every evaluation; during a 60 fps
            # animation that republishes each holding channel 60×/s and
            # floods /control (depth-1 newest-wins), clobbering the one
            # channel that's moving — the servo then only catches sparse
            # updates ("moves only at the keyframe"). Send only when this
            # channel's commanded value actually changed. Also lets the
            # firmware's idle_disengage fire, since a held channel now
            # stops getting SET_TARGET instead of being re-armed each tick.
            fval = float(value)
            gkey = (node_id, peripheral_id, channel_id)
            if self._last_channel_sent.get(gkey) == fval:
                return
            ptype = ""
            try:
                ptype = self._peripheral_type_lookup(node_id, peripheral_id) or ""
            except Exception:
                ptype = ""
            try:
                self._send_channel(node_id, peripheral_id, channel_id, fval, ptype)
                self._last_channel_sent[gkey] = fval
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


# ── compiled wire topology ──────────────────────────────────────────


class _SheetWireIndex:
    """Precomputed wire groupings for one sheet, built at reconcile().

    The routing graph's structure only changes when the operator edits
    the wiring, but values flow every tick. Deriving "which wires sink
    where" was previously redone on every tick — and for operator inputs
    it rescanned ALL wires once per operator, i.e. O(operators × wires).
    Compiling these buckets once turns the per-tick cost back to linear.
    """
    __slots__ = ("signal_sinks", "output_sinks", "perph_widget_sinks",
                 "op_input_wires", "incremental_ok", "op_leaf_deps")

    def __init__(self) -> None:
        self.signal_sinks: List[Wire] = []        # sink.kind == "signal"
        self.output_sinks: List[Wire] = []        # sink.kind == "output"
        self.perph_widget_sinks: List[Wire] = []  # "peripheral" | "widget"
        # operator_id → wires whose sink is that operator's input pins
        self.op_input_wires: Dict[str, List[Wire]] = {}
        # Incremental (cross-tick memoized) eval is only sound when this
        # sheet's operator values are PURE functions of its own ws_input
        # / topic / joint leaves — i.e. nothing reads a signal (mutated
        # out-of-band by other sheets) or an output (recomputed within
        # the pass). On such sheets we can seed clean operators from last
        # tick and skip recomputing chains whose leaves didn't change.
        # Sheets with signals/outputs fall back to full evaluation so the
        # cross-sheet contract (a trigger re-evaluates ALL sinks) holds.
        self.incremental_ok: bool = False
        # operator_id → frozenset of leaf keys it transitively depends on.
        # Leaf keys: ("w", ws_input_id) | ("i", input_id). Only populated
        # when incremental_ok.
        self.op_leaf_deps: Dict[str, frozenset] = {}


def _build_wire_index(sheet: NodeSheet) -> _SheetWireIndex:
    idx = _SheetWireIndex()
    for wire in sheet.wires:
        kind = wire.sink.kind
        if kind == "operator":
            if wire.sink.parts:
                idx.op_input_wires.setdefault(
                    wire.sink.parts[0], []).append(wire)
        elif kind == "signal":
            idx.signal_sinks.append(wire)
        elif kind == "output":
            idx.output_sinks.append(wire)
        elif kind in ("peripheral", "widget"):
            idx.perph_widget_sinks.append(wire)

    # Compile per-operator leaf dependencies for incremental eval. Bail
    # to incremental_ok=False the moment we see a signal/output node or
    # an operator input sourced from one — those make a sheet's values
    # depend on state outside its own leaves.
    ok = (not sheet.signals) and (not sheet.outputs)
    op_leaf_deps: Dict[str, frozenset] = {}
    if ok:
        memo: Dict[str, frozenset] = {}
        safe = [True]   # mutable flag closed over by _deps

        def _deps(op_id: str, stack: frozenset) -> frozenset:
            cached = memo.get(op_id)
            if cached is not None:
                return cached
            if op_id in stack:
                return frozenset()      # cycle: contributes no new leaves
            acc: set = set()
            for wire in idx.op_input_wires.get(op_id, ()):
                src = wire.source
                if src.kind == "ws_input" and src.parts:
                    acc.add(("w", src.parts[0]))
                elif src.kind == "input" and src.parts:
                    acc.add(("i", src.parts[0]))
                elif src.kind == "operator" and src.parts:
                    acc |= _deps(src.parts[0], stack | {op_id})
                else:
                    # signal / output / unknown source → not leaf-pure.
                    safe[0] = False
            result = frozenset(acc)
            memo[op_id] = result
            return result

        for op in sheet.operators:
            op_leaf_deps[op.id] = _deps(op.id, frozenset())
        ok = ok and safe[0]
    idx.incremental_ok = ok
    idx.op_leaf_deps = op_leaf_deps if ok else {}
    return idx


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
    # ── logic ops ───────────────────────────────────────────────────
    # Float→boolean coercion via the same 0.5 threshold the catalog
    # documents; outputs are always 1.0 / 0.0 so chained logic stays
    # numerically clean and downstream comparisons are exact.
    if op in ("and", "or", "not", "in_range", "if"):
        def truthy(x: float) -> bool:
            return x >= 0.5
        if op == "and":
            return 1.0 if truthy(a) and truthy(b) else 0.0
        if op == "or":
            return 1.0 if truthy(a) or truthy(b) else 0.0
        if op == "not":
            return 1.0 if not truthy(v) else 0.0
        if op == "in_range":
            lo = float(inputs.get("min", -1.0))
            hi = float(inputs.get("max",  1.0))
            if lo > hi:
                lo, hi = hi, lo
            return 1.0 if (lo <= v <= hi) else 0.0
        if op == "if":
            cond = float(inputs.get("cond", 0.0))
            then_v = float(inputs.get("then", 0.0))
            else_v = float(inputs.get("else", 0.0))
            return then_v if truthy(cond) else else_v
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
