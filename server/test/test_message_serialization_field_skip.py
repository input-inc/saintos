"""Field-skip behavior of the ROS message (de)serializer.

`serialize_message` / `deserialize_message` walk a ROS message field by
field and skip any field that raises. That per-field resilience is
correct — one weird field shouldn't drop the whole telemetry frame —
but it used to be SILENT: a field that consistently failed vanished
from the dashboard forever with no diagnostic, the same silent-failure
class as the announcement handler.

These tests pin BOTH invariants:
  1. resilience — a throwing field is skipped, the rest survive;
  2. diagnosability — the skip emits a debug log naming the field, so
     the loss is findable via log level instead of being invisible.
"""
from __future__ import annotations

import importlib.util
import logging
import pathlib

# Load message_types.py STANDALONE — importing it via the package would
# run saint_server/ros_bridge/__init__.py → bridge.py → qos_profiles.py,
# which needs rclpy QoS symbols the test stubs don't provide. The module
# has no rclpy import at top level (rosidl introspection is lazy inside
# its functions), so a direct file load is clean and dependency-free.
_MT_PATH = (pathlib.Path(__file__).resolve().parent.parent
            / "saint_server" / "ros_bridge" / "message_types.py")
_spec = importlib.util.spec_from_file_location("message_types_under_test", _MT_PATH)
mt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mt)


class _BoomField:
    """Attribute access raises — mimics a field the serializer can't read."""
    def __get__(self, obj, objtype=None):
        raise RuntimeError("cannot read this field")


class _Msg:
    """Stand-in ROS message: two good scalar fields + one that throws on
    access. We drive the serializer's field list directly via monkeypatch
    so we don't depend on real rosidl introspection in the test env."""
    def __init__(self):
        self.good_a = 1.5
        self.good_b = "ok"

    # `bad` raises when read.
    bad = _BoomField()


def _install_fields(monkeypatch, names, types=None):
    monkeypatch.setattr(mt, "_get_field_names", lambda _t: list(names))
    monkeypatch.setattr(mt, "_get_field_types",
                        lambda _t: dict(types or {}))


def test_serialize_skips_throwing_field_keeps_others(monkeypatch):
    _install_fields(monkeypatch, ["good_a", "bad", "good_b"])
    out = mt.serialize_message(_Msg())
    assert out == {"good_a": 1.5, "good_b": "ok"}   # bad dropped, rest intact


def test_serialize_skip_is_logged_for_diagnosis(monkeypatch, caplog):
    _install_fields(monkeypatch, ["good_a", "bad"])
    with caplog.at_level(logging.DEBUG, logger=mt.__name__):
        mt.serialize_message(_Msg())
    # The dropped field must be nameable from the logs — not silent.
    joined = " ".join(r.getMessage() for r in caplog.records)
    assert "bad" in joined, f"field-skip was not logged: {caplog.records!r}"


def test_deserialize_skips_unsettable_field_keeps_others(monkeypatch):
    # A target message whose `locked` attribute rejects assignment.
    class _Target:
        def __init__(self):
            self.value = 0.0
        def __setattr__(self, k, v):
            if k == "locked":
                raise AttributeError("read-only")
            object.__setattr__(self, k, v)

    _install_fields(monkeypatch, ["value", "locked"],
                    {"value": "float", "locked": "float"})
    msg = mt.deserialize_message({"value": 2.0, "locked": 9.0}, _Target)
    assert msg.value == 2.0          # good field set
    assert not hasattr(msg, "locked")  # bad field skipped, no crash


def test_deserialize_skip_is_logged_for_diagnosis(monkeypatch, caplog):
    class _Target:
        def __setattr__(self, k, v):
            if k == "locked":
                raise AttributeError("read-only")
            object.__setattr__(self, k, v)

    _install_fields(monkeypatch, ["locked"], {"locked": "float"})
    with caplog.at_level(logging.DEBUG, logger=mt.__name__):
        mt.deserialize_message({"locked": 9.0}, _Target)
    joined = " ".join(r.getMessage() for r in caplog.records)
    assert "locked" in joined, f"field-skip was not logged: {caplog.records!r}"
