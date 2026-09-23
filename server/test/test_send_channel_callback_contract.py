"""The State-tab slider path, end to end through the real signatures.

This is the test that was missing. Every other test of the control path
injects its own `send_channel` double, so they all kept passing while
production was broken: the websocket handler called the callback with
`owner=`, and the callback wired in `start_async_services` was a lambda
that restated the parameter list without it. The exception was caught by
the handler's try/except and logged, so the sliders silently did nothing
while the routing path — which calls the method directly — worked fine.

Pinning the CALLER's call shape against the CALLEE's real signature
catches that class of desync without needing a live server.
"""
from __future__ import annotations

import inspect
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.server_node import SaintServerNode


def test_send_channel_command_accepts_the_handlers_call():
    """The exact call websocket_handler makes for a slider write."""
    sig = inspect.signature(SaintServerNode.send_channel_command)
    sig.bind(
        None,                      # self
        "node-1", "maestro-1", "ch3", 0.5,
        "maestro",                 # peripheral_type, positional
        raw_us=None,
        owner="slider",
    )


def test_it_accepts_the_evaluator_call():
    """And the routing path's, which passes owner as a keyword too."""
    sig = inspect.signature(SaintServerNode.send_channel_command)
    sig.bind(None, "node-1", "maestro-1", "ch3", 0.5, "maestro",
             owner="stream")


def test_it_accepts_a_raw_microsecond_jog():
    sig = inspect.signature(SaintServerNode.send_channel_command)
    sig.bind(None, "node-1", "maestro-1", "ch3", None, "maestro",
             raw_us=2200, owner="slider")


def test_the_callback_is_wired_without_a_restating_wrapper():
    """A lambda that lists the parameters again is exactly how caller
    and callee drifted apart. Pass the bound method through instead, so
    a signature change cannot desync them."""
    src = inspect.getsource(SaintServerNode.start_async_services)
    m = re.search(r"set_send_channel_callback\(\s*(.*?)\)\s*\n", src, re.S)
    assert m, "could not find the send_channel callback wiring"
    wired = m.group(1).strip()
    assert "lambda" not in wired, (
        f"send_channel callback is wrapped in a lambda ({wired!r}); pass "
        f"self.send_channel_command directly so its signature cannot drift")
    assert "send_channel_command" in wired


def test_the_handler_still_passes_owner():
    """If the slider path ever stops tagging its writes, arbitration
    treats them as unattended stream traffic and change-gates the
    operator's hand."""
    from saint_server.webserver import websocket_handler as wh
    src = inspect.getsource(wh.WebSocketHandler._handle_control)
    assert "owner=SLIDER" in src
