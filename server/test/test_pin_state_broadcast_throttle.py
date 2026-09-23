"""Coverage for the per-node pin_state broadcast throttle.

Why this exists (measured on opensaint.local, 2026-09-22, chasing an
operator report that the track drives lagged behind the stick and kept
rotating after deadstick):

`SaintServerNode._broadcast_pin_state` fired on every `/state` message a
node published, and each broadcast carried the node's COMPLETE runtime
state — every pin and every channel, ~1.9 KB of JSON. RP2040 nodes
publish at 10 Hz, and there are four of them, so the server was pushing
~40 full-state frames/s — a steady, measured **76 KB/s** — at the Steam
Deck, against ~6 KB/s of control coming back.

The Deck is a station on the Pi's own 2.4 GHz AP (wlan0, channel 9,
20 MHz, one radio). That radio is half-duplex: every frame the Pi sends
downstream is airtime the Deck cannot use to send its next setpoint. The
control stream consequently arrived in clumps — set_input inter-arrival
p50 3 ms but p99 324 ms, max 463 ms, with the server provably idle inside
those gaps, so the delay was in the air rather than in our event loop.

Nothing in the control path reads this broadcast: `update_pin_actual` has
already run and the routing evaluator consults the state manager
directly. It is display data, so coalescing it is free.

These tests bind the methods to a SimpleNamespace standing in for a
SaintServerNode — a real one needs rclpy init, an async loop and log
dirs, none of which the throttle decision needs.
"""
from __future__ import annotations

import asyncio
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.server_node import SaintServerNode

# The default; an instance takes its window from
# websocket.pin_state_interval_ms (see TestConfigurableInterval).
INTERVAL = SaintServerNode._PIN_STATE_INTERVAL_S

# The throttle reads time.monotonic(), so tests must own it — otherwise
# advancing the fake loop's clock moves the timers but not the window the
# production code is measuring against.
NOW = [1000.0]


@pytest.fixture(autouse=True)
def frozen_clock(monkeypatch):
    import time as real_time
    monkeypatch.setattr(real_time, "monotonic", lambda: NOW[0])
    NOW[0] = 1000.0
    yield


class FakeLoop:
    """Loop stand-in with a test-driven clock.

    `call_later` honours its delay against that clock. A fake loop that
    ran deferred work immediately would fire the trailing edge the
    instant it was scheduled and so hide the throttle completely — which
    is exactly how the first draft of these tests fooled itself into
    passing against the un-throttled code.

    `call_soon_threadsafe` runs inline: the real loop runs it promptly,
    and every production use on this path is a lambda that only schedules
    or emits.
    """

    def __init__(self, clock):
        self.clock = clock          # single-element list holding "now"
        self.timers = []            # (due, fn, args)

    def call_soon_threadsafe(self, fn, *args):
        fn(*args)

    def call_later(self, delay, fn, *args):
        self.timers.append((self.clock[0] + delay, fn, args))

    @property
    def pending(self):
        return len(self.timers)

    def advance_to(self, now):
        """Move the clock to `now`, running whatever comes due."""
        self.clock[0] = now
        due = [t for t in self.timers if t[0] <= now]
        self.timers = [t for t in self.timers if t[0] > now]
        for _due, fn, args in sorted(due, key=lambda t: t[0]):
            fn(*args)


class Harness:
    """A SaintServerNode stand-in plus the knobs these tests need."""

    def __init__(self, state_fn=None, interval=None):
        self.interval = INTERVAL if interval is None else interval
        self.clock = NOW          # shared with the patched time.monotonic
        self.loop = FakeLoop(self.clock)
        self.sent = []

        sent = self.sent

        class FakeHandler:
            async def broadcast_state(self, topic, data):
                sent.append((topic, data))

        stub = types.SimpleNamespace(
            _pin_state_interval_s=self.interval,
            _pin_state_last_emit={},
            _pin_state_pending={},
            _async_loop=self.loop,
            web_server=types.SimpleNamespace(ws_handler=FakeHandler()),
            state_manager=types.SimpleNamespace(
                get_runtime_state=state_fn or (
                    lambda n: {"node_id": n, "channels": []}),
            ),
        )
        # The methods reach each other through self.
        stub._emit_pin_state = lambda n: SaintServerNode._emit_pin_state(stub, n)
        stub._flush_pin_state = lambda n: SaintServerNode._flush_pin_state(stub, n)
        self.stub = stub

    def publish(self, node_id):
        """One `/state` message arriving from `node_id`."""
        SaintServerNode._broadcast_pin_state(self.stub, node_id)

    def advance(self, seconds):
        self.loop.advance_to(self.clock[0] + seconds)

    @property
    def topics(self):
        return [topic for topic, _data in self.sent]

    @property
    def payloads(self):
        return [data for _topic, data in self.sent]


async def drain():
    """Let the tasks created by _emit_pin_state run to completion.

    The emit path ends in `asyncio.create_task(broadcast_state(...))`, so
    a frame has not reached the fan-out until the loop gets a turn.
    """
    for _ in range(3):
        await asyncio.sleep(0)


def run(coro_fn):
    """Run one async test body on a private loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro_fn())
    finally:
        loop.close()
        asyncio.set_event_loop(asyncio.new_event_loop())


class TestLeadingEdge:
    def test_first_publish_for_a_node_emits_immediately(self):
        """No added latency for the frame that starts a burst."""
        h = Harness()

        async def body():
            h.publish("rp2040_a")
            await drain()
            assert h.topics == ["pin_state/rp2040_a"]
            # Nothing deferred: the leading edge already covered it.
            assert h.loop.pending == 0

        run(body)

    def test_a_publish_after_the_window_emits_immediately_again(self):
        h = Harness()

        async def body():
            h.publish("n")
            h.advance(INTERVAL + 0.01)
            h.publish("n")
            await drain()
            assert len(h.sent) == 2
            assert h.loop.pending == 0

        run(body)


class TestCoalescing:
    def test_a_burst_inside_one_window_collapses_to_two_frames(self):
        """Ten 10 Hz publishes inside one window cost 2 frames, not 10."""
        h = Harness()

        async def body():
            h.publish("n")               # leading edge
            await drain()
            assert len(h.sent) == 1

            for _ in range(9):           # all inside the window
                h.advance(0.01)
                h.publish("n")
            await drain()
            assert len(h.sent) == 1, "intra-window publishes must not emit"
            assert h.loop.pending == 1, "one trailing fire, however many calls"

            h.advance(INTERVAL)          # let the trail come due
            await drain()
            assert len(h.sent) == 2

        run(body)

    def test_the_trailing_edge_always_lands(self):
        """The last frame of a burst must reach the UI.

        Without it a gauge sticks on whichever intermediate value won the
        leading edge — on a motor channel that means the dashboard shows
        the robot still driving after it has actually stopped.
        """
        values = iter([{"v": 1}, {"v": 2}, {"v": 3}])
        h = Harness(state_fn=lambda n: next(values))

        async def body():
            h.publish("n")               # emits v=1
            h.advance(0.01)
            h.publish("n")               # coalesced
            h.advance(0.01)
            h.publish("n")               # coalesced
            h.advance(INTERVAL)
            await drain()
            # The trail reads state fresh at flush time, so it carries the
            # newest value, not a snapshot taken when it was queued.
            assert h.payloads == [{"v": 1}, {"v": 2}]

        run(body)

    def test_the_trail_rearms_for_the_next_burst(self):
        h = Harness()

        async def body():
            h.publish("n")
            h.advance(0.01)
            h.publish("n")
            assert h.loop.pending == 1
            h.advance(INTERVAL)
            await drain()
            assert h.loop.pending == 0

            h.advance(0.01)              # second burst, fresh window
            h.publish("n")
            assert h.loop.pending == 1, "trail must rearm after firing"

        run(body)

    def test_the_scheduled_delay_covers_the_rest_of_the_window(self):
        h = Harness()

        async def body():
            h.publish("n")
            await drain()
            h.advance(0.05)
            h.publish("n")
            due = h.loop.timers[0][0]
            assert due - h.clock[0] == pytest.approx(INTERVAL - 0.05)

        run(body)

    def test_a_fired_trail_does_not_block_the_next_leading_edge(self):
        h = Harness()

        async def body():
            h.publish("n")
            h.advance(0.01)
            h.publish("n")               # sets the pending flag
            h.advance(INTERVAL)          # trail fires, clears it
            await drain()
            n_before = len(h.sent)
            h.advance(INTERVAL + 0.01)   # well past the window
            h.publish("n")
            await drain()
            assert len(h.sent) == n_before + 1

        run(body)


class TestPerNodeIsolation:
    def test_each_node_gets_its_own_window(self):
        """One chatty node must not throttle another node's first frame."""
        h = Harness()

        async def body():
            for n in ("a", "b", "c"):
                h.publish(n)
            await drain()
            assert h.topics == ["pin_state/a", "pin_state/b", "pin_state/c"]
            assert h.loop.pending == 0

        run(body)

    def test_one_nodes_burst_does_not_defer_another(self):
        h = Harness()

        async def body():
            h.publish("a")
            h.advance(0.01)
            h.publish("a")               # a is now inside its window
            h.publish("b")               # b is untouched — must emit now
            await drain()
            assert h.topics == ["pin_state/a", "pin_state/b"]

        run(body)

    def test_throttle_buckets_are_per_instance(self):
        """Regression guard: these must not be class-level dicts.

        A class-level dict is mutated in place, so every instance — and
        every test — would share one throttle state.
        """
        a, b = Harness(), Harness()
        assert a.stub._pin_state_last_emit is not b.stub._pin_state_last_emit
        # And the class must not carry a mutable default either.
        assert not hasattr(SaintServerNode, "_pin_state_last_emit")
        assert not hasattr(SaintServerNode, "_pin_state_pending")


class TestGuards:
    def test_no_emit_without_a_loop(self):
        h = Harness()
        h.stub._async_loop = None
        # Returns before creating a task, so this needs no running loop.
        h.publish("n")
        assert h.sent == []

    def test_no_emit_without_a_web_server(self):
        h = Harness()
        h.stub.web_server = None
        h.publish("n")
        assert h.sent == []

    def test_absent_runtime_state_is_skipped_quietly(self):
        h = Harness(state_fn=lambda n: None)

        async def body():
            h.publish("n")
            await drain()
            assert h.sent == []

        run(body)


class TestRateBudget:
    def test_four_10hz_nodes_are_held_under_the_old_rate(self):
        """The whole point, as a number.

        Four RP2040s publishing at 10 Hz produced 40 frames/s (~76 KB/s
        to the Deck). Per node the ceiling is now one leading plus one
        trailing frame per INTERVAL.
        """
        h = Harness()
        nodes = [f"rp2040_{i}" for i in range(4)]

        async def body():
            # One second of 10 Hz publishing from every node.
            for _tick in range(10):
                for n in nodes:
                    h.publish(n)
                h.advance(0.1)
                await drain()

            # Steady state settles at one frame per window per node: the
            # trailing fire re-arms the window, so the next publish
            # 100 ms later is inside it and coalesces again.
            per_node = len(h.sent) / len(nodes)
            assert per_node == pytest.approx(1.0 / INTERVAL, abs=1.0), (
                f"{per_node:g} Hz/node, expected ~{1.0 / INTERVAL:g} Hz")
            # The un-throttled path sent one per publish: 4 nodes x 10 Hz.
            assert len(h.sent) < 40, "no better than the unthrottled path"
            print(f"\n  {len(h.sent)} frames/s vs 40 unthrottled "
                  f"({40 / len(h.sent):.1f}x reduction)")

        run(body)

    def test_a_single_node_at_100hz_is_still_capped(self):
        """Rate is bounded by the window, not by the publisher."""
        h = Harness()

        async def body():
            for _ in range(100):         # 100 Hz for one second
                h.publish("n")
                h.advance(0.01)
                await drain()
            assert len(h.sent) <= 2.0 / INTERVAL + 1

        run(body)


class TestConfigurableInterval:
    """The window comes from websocket.pin_state_interval_ms.

    It is tunable at runtime because the right value depends on the link:
    raise it when control feels laggy on a congested AP, lower it when
    dashboard gauges feel steppy. Neither end is knowable from source.
    """

    def test_a_longer_window_emits_less(self):
        slow = Harness(interval=1.0)
        fast = Harness(interval=0.1)

        async def body():
            for h in (slow, fast):
                for _ in range(20):        # 1 s of 20 Hz
                    h.publish("n")
                    h.advance(0.05)
                    await drain()
            assert len(slow.sent) < len(fast.sent)

        run(body)

    def test_zero_disables_the_throttle(self):
        """0 restores the pre-2026-09 behaviour: one frame per publish.

        Kept as an escape hatch so a rig that genuinely wants 10 Hz
        gauges can have them without a code change.
        """
        h = Harness(interval=0.0)

        async def body():
            for _ in range(10):
                h.publish("n")
                h.advance(0.1)
                await drain()
            assert len(h.sent) == 10
            assert h.loop.pending == 0, "nothing should ever be deferred"

        run(body)

    def test_the_default_matches_the_class_constant(self):
        # A drifting default would make docs/LATENCY_REDUCTION.md's
        # numbers wrong without anything failing.
        from saint_server.config import WebSocketConfig
        assert (WebSocketConfig().pin_state_interval_ms / 1000.0
                == SaintServerNode._PIN_STATE_INTERVAL_S)
