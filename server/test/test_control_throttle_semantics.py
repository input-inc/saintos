"""Lock-down tests for streaming-control throttle semantics.

These pin the latency-critical behaviors found by the 2026-07 control-
pipeline audit (docs/LATENCY_REDUCTION.md is the running design doc):

- Neutral (near-zero) values BYPASS the 50 ms per-channel throttle so a
  deadstick is never buffered behind the rate limit.
- Non-neutral values inside the throttle window are DROPPED, not
  buffered. That trailing-edge hole is plugged upstream: the controller
  re-emits a held deflection every input tick (mapper.rs value_active)
  and heartbeats every 500 ms, so the next allowed window always gets
  the freshest value. If either side of that contract changes, these
  tests are the tripwire.
- The change-dedup cache records only values actually SENT. A value
  that was throttle-dropped must not poison the cache, or the
  controller's heartbeat replay of it would be swallowed as
  "unchanged" and the actuator would strand at a stale setpoint.
- CONTROL_QOS stays BEST_EFFORT + KEEP_LAST(1) (newest-wins) and
  COMMAND_QOS stays RELIABLE + KEEP_LAST(8) (one-shots never dropped).
"""
from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager
from saint_server.webserver import websocket_handler as ws_mod
from saint_server.webserver.websocket_handler import (
    WebSocketHandler,
    CONTROL_THROTTLE_MS,
    NEUTRAL_EPSILON,
    CONTROL_CHANGE_EPSILON,
    is_neutral_value,
)


def run(coro):
    return asyncio.get_event_loop().run_until_complete(coro)


class FakeClient:
    id = "test-client"


@pytest.fixture
def event_loop():
    # WebSocketHandler.__init__ creates an asyncio.Lock, which on
    # Python 3.9 binds to the current event loop — so the loop must
    # exist before the handler and stay open for the whole test.
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    yield loop
    loop.close()
    # Leave a fresh usable loop behind — set_event_loop(None) would make
    # later tests' asyncio.get_event_loop() raise on Python 3.9.
    asyncio.set_event_loop(asyncio.new_event_loop())


@pytest.fixture
def handler(event_loop, tmp_path):
    sm = StateManager(server_name="test-server", config_dir=str(tmp_path))
    h = WebSocketHandler(sm)
    # The throttle/dedup logic under test lives entirely in the WS
    # handler; channel-catalog resolution has its own tests. Stub the
    # lookup so set_channel_value reaches the throttle gate.
    sm.lookup_channel = lambda node, per, ch: {
        "direction": "out",
        "capability": "pwm",
        "peripheral_type": "maestro",
    }
    h._sent = []
    h.set_send_channel_callback(
        lambda node, per, ch, value, ptype, raw_us=None:
            h._sent.append((per, ch, value, raw_us)))
    return h


def set_channel(handler, value, channel="ch0"):
    return run(handler._handle_control(FakeClient(), "set_channel_value", {
        "node_id": "node-1",
        "peripheral_id": "maestro-1",
        "channel_id": channel,
        "value": value,
    }))


def open_throttle_window(handler):
    """Backdate the last-send stamps so the next value is outside the
    50 ms window — deterministic, no sleeping."""
    for key in list(handler._control_throttle):
        handler._control_throttle[key] -= (CONTROL_THROTTLE_MS + 1)


class TestNeutralBypass:
    def test_first_value_sends_immediately(self, handler):
        result = set_channel(handler, 0.5)
        assert result["status"] == "ok"
        assert result["data"]["throttled"] is False
        assert len(handler._sent) == 1

    def test_neutral_bypasses_throttle_window(self, handler):
        # Non-neutral send opens a throttle window; a deadstick arriving
        # inside it must still go out immediately.
        set_channel(handler, 0.8)
        result = set_channel(handler, 0.0)
        assert result["data"]["throttled"] is False
        assert handler._sent[-1][2] == 0.0

    def test_near_zero_counts_as_neutral(self, handler):
        set_channel(handler, 0.8)
        result = set_channel(handler, NEUTRAL_EPSILON / 2)
        assert result["data"]["throttled"] is False

    def test_is_neutral_value_epsilon_edges(self):
        assert is_neutral_value(0.0)
        assert is_neutral_value(NEUTRAL_EPSILON)
        assert is_neutral_value(-NEUTRAL_EPSILON)
        assert not is_neutral_value(NEUTRAL_EPSILON * 2)


class TestThrottleWindow:
    def test_non_neutral_inside_window_is_dropped(self, handler):
        # Documents the trailing-edge drop: the freshest non-neutral
        # value inside the window never reaches the firmware. Upstream
        # re-emission is what recovers it — see module docstring.
        set_channel(handler, 0.5)
        result = set_channel(handler, 0.9)
        assert result["data"]["throttled"] is True
        assert len(handler._sent) == 1
        assert handler._sent[0][2] == 0.5

    def test_value_after_window_goes_through(self, handler):
        set_channel(handler, 0.5)
        open_throttle_window(handler)
        result = set_channel(handler, 0.9)
        assert result["data"]["throttled"] is False
        assert handler._sent[-1][2] == 0.9

    def test_throttle_is_per_channel(self, handler):
        # Two channels of the same peripheral have independent windows —
        # a tank's second track must not inherit the first track's
        # throttle stamp.
        set_channel(handler, 0.5, channel="ch0")
        result = set_channel(handler, 0.5, channel="ch1")
        assert result["data"]["throttled"] is False
        assert len(handler._sent) == 2


class TestChangeDedup:
    def test_identical_resend_is_suppressed(self, handler):
        # A held stick at constant deflection produces a 20 Hz stream of
        # identical values from the controller; the dedup keeps them off
        # the radio.
        set_channel(handler, 0.5)
        open_throttle_window(handler)
        result = set_channel(handler, 0.5)
        assert result["data"].get("unchanged") is True
        assert len(handler._sent) == 1

    def test_sub_epsilon_wiggle_is_suppressed(self, handler):
        set_channel(handler, 0.5)
        open_throttle_window(handler)
        result = set_channel(handler, 0.5 + CONTROL_CHANGE_EPSILON / 2)
        assert result["data"].get("unchanged") is True

    def test_throttle_dropped_value_does_not_poison_dedup(self, handler):
        # THE regression case: 0.9 arrives inside the window and is
        # dropped. The controller replays 0.9 later (heartbeat or
        # steady re-emit). If the dedup cache had recorded the dropped
        # 0.9, the replay would be swallowed as "unchanged" and the
        # actuator would stay at 0.5 forever.
        set_channel(handler, 0.5)
        dropped = set_channel(handler, 0.9)
        assert dropped["data"]["throttled"] is True
        open_throttle_window(handler)
        replay = set_channel(handler, 0.9)
        assert replay["data"]["throttled"] is False
        assert handler._sent[-1][2] == 0.9

    def test_dedup_records_on_send_including_neutral(self, handler):
        # After a stop is sent, repeated stops (controller heartbeat
        # re-asserting zero every 500 ms) dedupe instead of re-sending.
        set_channel(handler, 0.5)
        set_channel(handler, 0.0)
        open_throttle_window(handler)
        result = set_channel(handler, 0.0)
        assert result["data"].get("unchanged") is True
        assert len(handler._sent) == 2


class TestQosContract:
    def test_control_qos_is_best_effort_newest_wins(self):
        from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy
        from saint_server.server_node import CONTROL_QOS
        assert CONTROL_QOS.reliability == QoSReliabilityPolicy.BEST_EFFORT
        assert CONTROL_QOS.history == QoSHistoryPolicy.KEEP_LAST
        assert CONTROL_QOS.depth == 1

    def test_command_qos_is_reliable_with_queue(self):
        from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy
        from saint_server.server_node import COMMAND_QOS
        assert COMMAND_QOS.reliability == QoSReliabilityPolicy.RELIABLE
        assert COMMAND_QOS.history == QoSHistoryPolicy.KEEP_LAST
        assert COMMAND_QOS.depth == 8
