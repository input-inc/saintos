"""Lock-down tests for WebSocket broadcast fan-out.

Why this file exists (measured on opensaint.local, 2026-09-22, while an
operator reported the track drives lagging behind the stick and running
on after deadstick):

The fan-out used to be

    async with self._lock:
        for client in self.clients.values():
            if <match>:
                await self._send_to_client(client, message)

which is sequential, holds the registration lock across network I/O, and
has no send deadline. A stale controller connection was found sitting at
413,689 bytes in Send-Q, not draining, keepalive timer 116 minutes out —
a peer that had gone away without a FIN. Writes to it blocked forever, so
that one client held `self._lock` indefinitely while every broadcast task
(created with `create_task`, ~50/s under stick motion) piled up behind
it without bound.

These tests pin the three properties that fixed it:
  1. sends run concurrently — one slow client does not set everyone's
     latency,
  2. the lock is released before any send, so connects/disconnects are
     never blocked by a slow client,
  3. a send that blocks past BROADCAST_SEND_TIMEOUT_S closes that client
     instead of stalling the broadcast.
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
    BROADCAST_SEND_TIMEOUT_S,
    WebSocketClient,
    WebSocketHandler,
)


@pytest.fixture
def event_loop():
    # WebSocketHandler.__init__ creates an asyncio.Lock, which binds to
    # the current loop — the loop must exist first and outlive the test.
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    yield loop
    loop.close()
    asyncio.set_event_loop(asyncio.new_event_loop())


@pytest.fixture
def handler(event_loop, tmp_path):
    sm = StateManager(server_name="test-server", config_dir=str(tmp_path))
    return WebSocketHandler(sm)


class FakeWS:
    """Minimal aiohttp WebSocketResponse stand-in.

    `delay` is how long each send_json takes; None means "never
    completes", which is what a wedged socket with a full kernel send
    buffer actually does.
    """

    def __init__(self, delay: float = 0.0):
        self.delay = delay
        self.sent = []
        self.closed = False

    async def send_json(self, message):
        if self.delay is None:
            await asyncio.Event().wait()  # blocks forever
        elif self.delay:
            await asyncio.sleep(self.delay)
        self.sent.append(message)

    async def close(self):
        self.closed = True


def add_client(handler, cid, ws, topics=()):
    client = WebSocketClient(id=cid, ws=ws, subscriptions=set(topics))
    handler.clients[cid] = client
    return client


def test_subscription_filter_still_applies(handler, event_loop):
    yes = FakeWS()
    no = FakeWS()
    add_client(handler, "yes", yes, ["pin_state/n1"])
    add_client(handler, "no", no, ["pin_state/other"])

    event_loop.run_until_complete(handler.broadcast_state("pin_state/n1", {"v": 1}))

    assert len(yes.sent) == 1
    assert yes.sent[0]["node"] == "pin_state/n1"
    assert no.sent == []


def test_all_subscription_receives_every_topic(handler, event_loop):
    ws = FakeWS()
    add_client(handler, "star", ws, ["all"])
    event_loop.run_until_complete(handler.broadcast_state("anything", {}))
    assert len(ws.sent) == 1


def test_sends_run_concurrently_not_sequentially(handler, event_loop):
    """Ten clients each taking 100 ms must finish in ~100 ms, not ~1 s.

    This is the property that keeps one client on a congested link from
    setting the broadcast latency for everyone else.
    """
    for i in range(10):
        add_client(handler, f"c{i}", FakeWS(delay=0.1), ["t"])

    start = event_loop.time()
    event_loop.run_until_complete(handler.broadcast_state("t", {}))
    elapsed = event_loop.time() - start

    assert elapsed < 0.5, f"fan-out serialised: took {elapsed:.2f}s for 10x100ms"


def test_a_wedged_client_does_not_delay_a_healthy_one(handler, event_loop):
    """The regression, in one test.

    The wedged client never completes its send. The healthy client must
    still have its frame before the wedged one's deadline expires.
    """
    wedged = FakeWS(delay=None)
    healthy = FakeWS()
    add_client(handler, "wedged", wedged, ["t"])
    add_client(handler, "healthy", healthy, ["t"])

    async def scenario():
        task = asyncio.ensure_future(handler.broadcast_state("t", {"v": 1}))
        # Well before BROADCAST_SEND_TIMEOUT_S, so we are observing
        # concurrency and not the timeout path.
        await asyncio.sleep(0.05)
        assert len(healthy.sent) == 1, "healthy client blocked behind wedged one"
        return task

    task = event_loop.run_until_complete(scenario())
    # Let the wedged client hit its deadline so the broadcast completes.
    event_loop.run_until_complete(task)
    assert wedged.closed is True


def test_registration_lock_is_free_during_sends(handler, event_loop):
    """A blocked send must not hold the lock that guards self.clients.

    Previously the lock was held for the whole fan-out, so a wedged
    client blocked connects and disconnects for as long as it was wedged
    — which, with no send timeout, was forever.
    """
    add_client(handler, "wedged", FakeWS(delay=None), ["t"])

    async def scenario():
        task = asyncio.ensure_future(handler.broadcast_state("t", {}))
        await asyncio.sleep(0.05)
        # If the lock were still held this would block until the send
        # deadline; assert it is obtainable promptly.
        await asyncio.wait_for(handler._lock.acquire(), timeout=0.5)
        handler._lock.release()
        return task

    task = event_loop.run_until_complete(scenario())
    event_loop.run_until_complete(task)


def test_wedged_client_is_closed_at_the_deadline(handler, event_loop, monkeypatch):
    """A send that blocks past the deadline closes the client.

    It is not retried: wait_for cancels the write, which may already have
    put a partial frame on the wire, so the stream is no longer
    trustworthy. Closing forces a clean reconnect.
    """
    monkeypatch.setattr(ws_mod, "BROADCAST_SEND_TIMEOUT_S", 0.05)
    ws = FakeWS(delay=None)
    client = add_client(handler, "wedged", ws, ["t"])

    event_loop.run_until_complete(handler.broadcast_state("t", {}))

    assert ws.closed is True
    assert client.send_timeouts == 1


def test_timeout_counter_resets_on_a_good_send(handler, event_loop):
    ws = FakeWS()
    client = add_client(handler, "c", ws, ["t"])
    client.send_timeouts = 2

    event_loop.run_until_complete(handler.broadcast_state("t", {}))

    assert client.send_timeouts == 0


def test_a_disconnected_client_does_not_break_the_broadcast(handler, event_loop):
    """A client that raises mid-broadcast must not stop the others."""

    class BrokenWS(FakeWS):
        async def send_json(self, message):
            raise ConnectionResetError("peer went away")

    add_client(handler, "broken", BrokenWS(), ["t"])
    ok = FakeWS()
    add_client(handler, "ok", ok, ["t"])

    event_loop.run_until_complete(handler.broadcast_state("t", {}))

    assert len(ok.sent) == 1


def test_activity_broadcast_goes_to_every_client(handler, event_loop):
    """broadcast_activity has no subscription filter — all clients get it."""
    a, b = FakeWS(), FakeWS()
    add_client(handler, "a", a)
    add_client(handler, "b", b, ["unrelated"])

    event_loop.run_until_complete(handler.broadcast_activity("hello", "warn"))

    assert len(a.sent) == 1 and len(b.sent) == 1
    assert a.sent[0]["type"] == "activity"


def test_ros_state_uses_the_ros_prefixed_subscription_key(handler, event_loop):
    sub = FakeWS()
    unsub = FakeWS()
    add_client(handler, "sub", sub, ["ros:/joint_states"])
    # Subscribed to the bare topic, not the ros: key — must not match.
    add_client(handler, "unsub", unsub, ["/joint_states"])

    event_loop.run_until_complete(
        handler.broadcast_ros_state("/joint_states", {"p": [1]}))

    assert len(sub.sent) == 1
    assert sub.sent[0]["type"] == "ros_state"
    assert unsub.sent == []


def test_no_subscribers_is_a_cheap_noop(handler, event_loop):
    ws = FakeWS()
    add_client(handler, "c", ws, ["other"])
    event_loop.run_until_complete(handler.broadcast_state("t", {}))
    assert ws.sent == []
