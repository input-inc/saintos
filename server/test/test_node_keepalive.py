"""Lock-down tests for the idle node keepalive.

Teensy nodes judge agent liveness by RX-with-data only (NativeEthernet's
udp.endPacket() succeeds even with the agent gone, so TX proves nothing).
An idle robot sends them nothing, so a HEALTHY node trips its 45 s
timeout and tears down a working micro-ROS session roughly every 60 s —
each cycle losing every best-effort /control write inside a 1-4 s window.

The firmware-side fix (rmw_uros_ping_agent from the main loop) hard-faults
the chip, so the keepalive lives on the server. These tests pin the two
properties that make it safe:

  * it fires for an idle, adopted, online node, and
  * it does NOT fire for a node we are actively writing to — injecting a
    frame there could displace a real setpoint on a depth-1 best-effort
    topic, which is the exact failure the control-path work just fixed.
"""
from __future__ import annotations

import os
import sys
import types
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.server_node import (
    NODE_KEEPALIVE_IDLE_S,
    NODE_KEEPALIVE_PAYLOAD,
    SaintServerNode,
)
from saint_server.webserver.state_manager import HOST_CONTROLLER_NODE_ID


class FakeNode:
    def __init__(self, online=True):
        self.online = online


def make_stub(nodes, now=1000.0):
    """Minimal stand-in exposing only what the keepalive touches."""
    published = []
    logger = MagicMock()

    def ensure_pub(node_id):
        pub = MagicMock()
        pub.publish.side_effect = lambda msg: published.append((node_id, msg.data))
        return pub

    stub = types.SimpleNamespace(
        state_manager=types.SimpleNamespace(
            state=types.SimpleNamespace(adopted_nodes=nodes)),
        get_logger=lambda: logger,
        _ensure_node_control_publisher=ensure_pub,
        _node_last_tx={},
        _mark_node_tx=lambda nid: stub._node_last_tx.__setitem__(nid, now),
        published=published,
        logger=logger,
    )
    return stub


def tick(stub, monkeypatch, now):
    monkeypatch.setattr("saint_server.server_node.time.monotonic", lambda: now)
    SaintServerNode._send_node_keepalives(stub)


def test_idle_node_gets_a_keepalive(monkeypatch):
    stub = make_stub({"teensy-1": FakeNode()})
    tick(stub, monkeypatch, 1000.0)
    assert stub.published == [("teensy-1", NODE_KEEPALIVE_PAYLOAD)]


def test_node_written_to_recently_is_skipped(monkeypatch):
    """THE safety property: a node under active control must never have a
    frame injected behind the operator's back."""
    stub = make_stub({"teensy-1": FakeNode()})
    stub._node_last_tx["teensy-1"] = 1000.0
    tick(stub, monkeypatch, 1000.0 + NODE_KEEPALIVE_IDLE_S - 0.1)
    assert stub.published == []


def test_keepalive_resumes_once_the_node_goes_quiet(monkeypatch):
    stub = make_stub({"teensy-1": FakeNode()})
    stub._node_last_tx["teensy-1"] = 1000.0
    tick(stub, monkeypatch, 1000.0 + NODE_KEEPALIVE_IDLE_S + 0.1)
    assert stub.published == [("teensy-1", NODE_KEEPALIVE_PAYLOAD)]


def test_offline_nodes_are_left_to_the_reconnect_path(monkeypatch):
    stub = make_stub({"teensy-1": FakeNode(online=False)})
    tick(stub, monkeypatch, 1000.0)
    assert stub.published == []


def test_host_controller_is_skipped(monkeypatch):
    """The virtual host node runs in-process — it has no wire to keep
    alive, and its /control topic is dead."""
    stub = make_stub({HOST_CONTROLLER_NODE_ID: FakeNode()})
    tick(stub, monkeypatch, 1000.0)
    assert stub.published == []


def test_each_node_is_tracked_independently(monkeypatch):
    stub = make_stub({"teensy-1": FakeNode(), "rp2040-1": FakeNode()})
    stub._node_last_tx["teensy-1"] = 1000.0
    tick(stub, monkeypatch, 1000.0 + 1.0)
    assert [n for n, _ in stub.published] == ["rp2040-1"]


def test_a_publish_failure_does_not_stop_other_nodes(monkeypatch):
    """One unhappy node must not deny the keepalive to the rest."""
    stub = make_stub({"bad": FakeNode(), "good": FakeNode()})
    published = stub.published

    def ensure_pub(node_id):
        pub = MagicMock()
        if node_id == "bad":
            pub.publish.side_effect = RuntimeError("publisher gone")
        else:
            pub.publish.side_effect = lambda msg: published.append((node_id, msg.data))
        return pub

    stub._ensure_node_control_publisher = ensure_pub
    tick(stub, monkeypatch, 1000.0)
    assert [n for n, _ in published] == ["good"]
    assert stub.logger.warn.called


def test_payload_is_a_firmware_noop():
    """Both platforms' pin_control_apply_json return false for an action
    they don't recognize, logging nothing — the frame only has to arrive.
    If this payload ever becomes a real action, this breaks loudly."""
    assert NODE_KEEPALIVE_PAYLOAD == '{"action":"keepalive"}'
    assert "set_pin" not in NODE_KEEPALIVE_PAYLOAD
    assert "set_channel" not in NODE_KEEPALIVE_PAYLOAD


def test_idle_window_leaves_margin_under_the_firmware_timeout():
    """Firmware CONNECTION_TIMEOUT_MS is 45 s and it re-checks every 5 s.
    Keep at least 2x margin so a single lost keepalive can't trip it."""
    assert NODE_KEEPALIVE_IDLE_S * 2 < 45.0
