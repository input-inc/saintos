"""Lock-down tests for the router (gamepad) drive path — 2026-08
deadstick-run-on fix.

The July audit hardened the `control` path (throttle + neutral bypass)
but the actual gamepad path is `router/set_input`, which had none of
those protections and two per-tick costs inside the sequential WS
receive loop: an unsampled INFO log line and an awaited JSON ack back
to the controller. Under a stick-circle burst those slowed the drain
enough that a release-zero waited seconds behind stale frames.

These tests pin:
- router/set_input success produces NO ack (the controller never reads
  it) while errors still respond, and _handle_message honors a None
  response by not writing to the socket.
- router/set_input logs at debug, not info (the hot-path file-I/O fix).
- The change-gate re-asserts unchanged NON-neutral values for MOTOR
  peripherals every MOTOR_REASSERT_MS — the liveness feed the firmware
  dead-man (roboclaw) times against — while servos and neutral values
  stay change-gated (idle_disengage depends on it). That gate now
  lives in channel_arbiter (shared by every writer); these tests drive
  it through the evaluator's dispatch path, which is how production
  reaches it.
"""
from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import NodeSheet, RouteEndpoint, Wire
from saint_server.channel_arbiter import (
    ChannelArbiter, MOTOR_REASSERT_MS, STREAM,
)
from saint_server.router import routing_evaluator as re_mod
from saint_server.router.routing_evaluator import RoutingEvaluator
from saint_server.webserver.state_manager import StateManager
from saint_server.webserver.websocket_handler import WebSocketHandler


def run(coro):
    return asyncio.get_event_loop().run_until_complete(coro)


class FakeClient:
    id = "test-client"
    authenticated = True


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    yield loop
    loop.close()
    asyncio.set_event_loop(asyncio.new_event_loop())


@pytest.fixture
def handler(event_loop, tmp_path):
    sm = StateManager(server_name="test-server", config_dir=str(tmp_path))
    h = WebSocketHandler(sm)
    h._log_lines = []
    h.log = lambda level, msg: h._log_lines.append((level, msg))
    h._client_writes = []

    async def record_write(client, payload):
        h._client_writes.append(payload)

    h._send_to_client = record_write
    return h


# ── ack suppression ─────────────────────────────────────────────────

class TestSetInputAckSuppression:
    def test_successful_set_input_returns_none(self, handler):
        handler.state_manager.push_ws_input = lambda s, i, v: True
        resp = run(handler._handle_router(FakeClient(), "set_input", {
            "sheet_id": "sheet-1", "input_id": "in-1", "value": 0.5,
        }))
        assert resp is None

    def test_failed_set_input_still_returns_error(self, handler):
        handler.state_manager.push_ws_input = lambda s, i, v: False
        resp = run(handler._handle_router(FakeClient(), "set_input", {
            "sheet_id": "sheet-1", "input_id": "in-1", "value": 0.5,
        }))
        assert resp is not None and resp["status"] == "error"

    def test_handle_message_skips_socket_write_on_none_response(self, handler):
        handler.state_manager.push_ws_input = lambda s, i, v: True
        run(handler._handle_message(FakeClient(), (
            '{"id": "m1", "type": "router", "action": "set_input",'
            ' "params": {"sheet_id": "s", "input_id": "i", "value": 0.25}}'
        )))
        assert handler._client_writes == [], (
            "a successful streaming set_input must not await an ack write")

    def test_handle_message_still_acks_other_router_actions(self, handler):
        handler.state_manager.list_ws_inputs = lambda: []
        run(handler._handle_message(FakeClient(), (
            '{"id": "m2", "type": "router", "action": "list_websocket_inputs",'
            ' "params": {}}'
        )))
        assert len(handler._client_writes) == 1
        assert handler._client_writes[0]["status"] == "ok"
        assert handler._client_writes[0]["id"] == "m2"


# ── hot-path log level ──────────────────────────────────────────────

class TestRouterHotLogLevel:
    def test_set_input_logs_at_debug(self, handler):
        handler.state_manager.push_ws_input = lambda s, i, v: True
        run(handler._handle_message(FakeClient(), (
            '{"id": "m1", "type": "router", "action": "set_input",'
            ' "params": {"sheet_id": "s", "input_id": "i", "value": 0.25}}'
        )))
        levels = [lvl for lvl, msg in handler._log_lines if "set_input" in msg]
        assert levels and all(lvl == "debug" for lvl in levels), (
            "per-tick set_input must never hit the INFO file handler "
            f"(got {handler._log_lines})")

    def test_other_router_actions_still_log_at_info(self, handler):
        handler.state_manager.list_ws_inputs = lambda: []
        run(handler._handle_message(FakeClient(), (
            '{"id": "m2", "type": "router", "action": "list_websocket_inputs",'
            ' "params": {}}'
        )))
        assert any(lvl == "info" and "list_websocket_inputs" in msg
                   for lvl, msg in handler._log_lines)


# ── evaluator motor re-assert ───────────────────────────────────────

class _FakeClock:
    """Injectable millisecond clock so re-assert windows are tested by
    advancing time, not by sleeping."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, ms: float) -> None:
        self.t += ms


def make_evaluator(ptype="roboclaw"):
    """Evaluator wired to a real ChannelArbiter.

    The change-gate these tests pin used to live in the evaluator; it
    now lives in channel_arbiter, consulted by
    server_node.send_channel_command. The send_channel double below
    mirrors that call's should_send/record pair exactly, so the
    semantics stay covered end-to-end through the dispatch path rather
    than being re-tested in isolation.
    """
    sent = []
    clock = _FakeClock()
    arbiter = ChannelArbiter(clock=clock)

    def send_channel(node, per, ch, val, pt, owner=STREAM):
        if not arbiter.should_send(node, per, ch, val,
                                   owner=owner, peripheral_type=pt):
            return False
        sent.append((node, per, ch, val, pt))
        arbiter.record(node, per, ch, val, owner=owner)
        return True

    ev = RoutingEvaluator(
        ros_bridge=None,
        send_channel=send_channel,
        peripheral_type_lookup=lambda node, per: ptype,
        channel_arbiter=arbiter,
    )
    return ev, sent, clock


def motor_wire(channel="motor"):
    return Wire(
        id="w1",
        source=RouteEndpoint(kind="ws_input", parts=["in-1"]),
        sink=RouteEndpoint(kind="peripheral", parts=["node-1", "per-1", channel]),
    )


def dispatch(ev, value):
    ev._dispatch_sink(NodeSheet(node_id="sheet-1"), motor_wire(), value)




class TestMotorReassert:
    def test_changed_value_always_sends(self):
        ev, sent, clock = make_evaluator("roboclaw")
        dispatch(ev, 0.5)
        dispatch(ev, 0.6)
        assert [s[3] for s in sent] == [0.5, 0.6]

    def test_unchanged_motor_value_inside_window_is_gated(self):
        ev, sent, clock = make_evaluator("roboclaw")
        dispatch(ev, 0.5)
        dispatch(ev, 0.5)
        assert len(sent) == 1, "re-assert must respect the window, not spam"

    def test_unchanged_motor_value_reasserts_after_window(self):
        # THE dead-man feed: a held stick (controller heartbeat replays
        # the same value) must keep /control alive for motor channels.
        ev, sent, clock = make_evaluator("roboclaw")
        dispatch(ev, 0.5)
        clock.advance(MOTOR_REASSERT_MS + 1)
        dispatch(ev, 0.5)
        assert [s[3] for s in sent] == [0.5, 0.5]

    def test_syren_is_also_a_motor_type(self):
        ev, sent, clock = make_evaluator("syren")
        dispatch(ev, -0.7)
        clock.advance(MOTOR_REASSERT_MS + 1)
        dispatch(ev, -0.7)
        assert len(sent) == 2

    def test_neutral_motor_value_never_reasserts(self):
        # A stopped motor needs no liveness feed — and re-sending zeros
        # forever would defeat the firmware's ability to go quiet.
        ev, sent, clock = make_evaluator("roboclaw")
        dispatch(ev, 0.0)
        clock.advance(MOTOR_REASSERT_MS * 10)
        dispatch(ev, 0.0)
        assert len(sent) == 1

    def test_servo_types_stay_fully_change_gated(self):
        # idle_disengage regression tripwire: a held servo channel must
        # go quiet no matter how much time passes.
        ev, sent, clock = make_evaluator("maestro")
        dispatch(ev, 0.5)
        clock.advance(MOTOR_REASSERT_MS * 10)
        dispatch(ev, 0.5)
        assert len(sent) == 1

    def test_reassert_window_measured_from_last_send(self):
        ev, sent, clock = make_evaluator("roboclaw")
        dispatch(ev, 0.5)
        clock.advance(MOTOR_REASSERT_MS + 1)
        dispatch(ev, 0.5)                      # re-assert (2nd send)
        dispatch(ev, 0.5)                      # window fresh again — gated
        assert len(sent) == 2

    def test_estop_gate_still_blocks_motor_reasserts(self):
        ev, sent, clock = make_evaluator("roboclaw")
        dispatch(ev, 0.5)
        ev.set_estop_active(True)
        clock.advance(MOTOR_REASSERT_MS * 10)
        dispatch(ev, 0.5)
        assert len(sent) == 1, "estop must suppress re-asserts too"
