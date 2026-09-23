"""Lock-down tests for channel write arbitration.

These pin the operator-facing rule — a pose board activation overrides
every other control; a slider owns its channel while the operator moves
it — and the cross-writer failures that rule exists to fix.

The failures were observed on the robot (Head Node, 24-channel Maestro):
re-activating a pose silently did nothing for exactly the channels the
operator had last nudged with a slider, and dragging a slider back to a
value a pose had overwritten did nothing either. Both reported success,
because each writer kept its own "already sent that" cache and neither
could see the other's writes.

Everything below drives the real ChannelArbiter through the same
should_send/record pair that server_node.send_channel_command uses.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.channel_arbiter import (
    BOARD,
    CHANGE_EPSILON,
    MOTOR_REASSERT_MS,
    SLIDER,
    STREAM,
    ChannelArbiter,
)

NODE = "teensy41_head"
PER = "maestro-1"
CH = "ch9"


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, ms: float) -> None:
        self.t += ms


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def arbiter(clock):
    return ChannelArbiter(clock=clock)


def write(arb, value, owner=STREAM, ptype="maestro", raw_us=None, ch=CH):
    """Mirror of send_channel_command's arbitration: ask, then record
    only what actually went out."""
    if not arb.should_send(NODE, PER, ch, value, owner=owner,
                           peripheral_type=ptype, raw_us=raw_us):
        return False
    arb.record(NODE, PER, ch, value, owner=owner, raw_us=raw_us)
    return True


# ── the rule the operator asked for ─────────────────────────────────

class TestBoardAuthority:
    def test_board_reasserts_after_a_slider_moved_the_channel(self, arbiter):
        """THE pose-board bug: activate a board, nudge a slider, then
        re-activate the same board. The second activation must reach the
        hardware — previously the evaluator compared against its own
        cache, saw no change, and sent nothing while the servo sat where
        the slider had left it."""
        assert write(arbiter, 0.40, owner=BOARD) is True
        assert write(arbiter, -0.35, owner=SLIDER) is True
        assert write(arbiter, 0.40, owner=BOARD) is True, \
            "re-activating a board must always re-assert its channels"

    def test_board_reasserts_its_own_unchanged_value(self, arbiter):
        # Clicking the same board twice with nothing in between still
        # re-sends: it is the only recovery path for a /control frame
        # dropped in flight, since servos are never re-asserted.
        assert write(arbiter, 0.40, owner=BOARD) is True
        assert write(arbiter, 0.40, owner=BOARD) is True

    def test_board_overrides_a_stream_holding_the_channel(self, arbiter):
        assert write(arbiter, 0.20, owner=STREAM) is True
        assert write(arbiter, 0.20, owner=BOARD) is True

    def test_board_activation_takes_ownership(self, arbiter):
        write(arbiter, -0.35, owner=SLIDER)
        assert arbiter.owner_of(NODE, PER, CH) == SLIDER
        write(arbiter, 0.40, owner=BOARD)
        assert arbiter.owner_of(NODE, PER, CH) == BOARD


class TestSliderAuthority:
    def test_slider_value_stands_after_release(self, arbiter, clock):
        """A parked slider is not a writer: nothing re-asserts the board
        underneath it, so the value the operator dragged to persists
        until the next board activation."""
        write(arbiter, 0.40, owner=BOARD)
        write(arbiter, -0.35, owner=SLIDER)
        clock.advance(60_000)
        assert arbiter.last_value(NODE, PER, CH) == pytest.approx(-0.35)
        assert arbiter.owner_of(NODE, PER, CH) == SLIDER

    def test_slider_can_command_a_value_a_board_overwrote(self, arbiter):
        """Mirror of the board bug. The slider path's private cache used
        to answer {"unchanged": true} here and send nothing."""
        write(arbiter, 0.10, owner=SLIDER)
        write(arbiter, 0.80, owner=BOARD)      # board moves it away
        assert write(arbiter, 0.10, owner=SLIDER) is True

    def test_slider_repeat_of_the_held_value_is_still_gated(self, arbiter):
        # Not a regression: if the hardware verifiably holds this value,
        # skipping the write is correct and keeps a held servo channel
        # quiet enough for the firmware's idle_disengage to fire.
        assert write(arbiter, 0.40, owner=SLIDER) is True
        assert write(arbiter, 0.40, owner=SLIDER) is False


# ── invalidation: the hardware moved behind our back ────────────────

class TestInvalidation:
    def test_node_reconnect_forgets_cached_state(self, arbiter):
        """A node that re-initialized micro-ROS came back at its boot/home
        state, and any write issued during the gap went nowhere —
        /control is best-effort. The next command must not be suppressed
        as 'already there'."""
        write(arbiter, 0.40, owner=SLIDER)
        assert arbiter.invalidate_node(NODE, reason="node reconnected") == 1
        assert write(arbiter, 0.40, owner=SLIDER) is True

    def test_peripheral_reconnect_forgets_only_that_peripheral(self, arbiter):
        write(arbiter, 0.40, owner=SLIDER, ch="ch1")
        arbiter.record(NODE, "roboclaw-1", "motor", 0.5, owner=STREAM)
        assert arbiter.invalidate_peripheral(NODE, PER) == 1
        assert arbiter.last_value(NODE, "roboclaw-1", "motor") == 0.5

    def test_invalidation_is_scoped_to_one_node(self, arbiter):
        write(arbiter, 0.40, owner=SLIDER)
        arbiter.record("other-node", PER, CH, 0.40, owner=SLIDER)
        arbiter.invalidate_node(NODE)
        assert arbiter.last_value("other-node", PER, CH) == 0.40

    def test_us_preview_drops_the_normalized_cache(self, arbiter):
        """The extent dialer jogs in absolute microseconds, moving the
        servo off the normalized map. If the cached normalized value
        survived, dragging the slider back to it would be swallowed —
        a reproducible dead slider."""
        write(arbiter, 0.40, owner=SLIDER)
        assert write(arbiter, None, owner=SLIDER, raw_us=2200) is True
        assert arbiter.last_value(NODE, PER, CH) is None
        assert write(arbiter, 0.40, owner=SLIDER) is True


# ── stream gating survives the move ─────────────────────────────────

class TestStreamGating:
    def test_unchanged_stream_value_is_gated(self, arbiter):
        assert write(arbiter, 0.5, owner=STREAM) is True
        assert write(arbiter, 0.5, owner=STREAM) is False

    def test_sub_epsilon_wiggle_is_gated(self, arbiter):
        write(arbiter, 0.5, owner=STREAM)
        assert write(arbiter, 0.5 + CHANGE_EPSILON / 2, owner=STREAM) is False

    def test_changed_stream_value_passes(self, arbiter):
        write(arbiter, 0.5, owner=STREAM)
        assert write(arbiter, 0.6, owner=STREAM) is True

    def test_motor_reasserts_after_window(self, arbiter, clock):
        write(arbiter, 0.5, owner=STREAM, ptype="roboclaw")
        assert write(arbiter, 0.5, owner=STREAM, ptype="roboclaw") is False
        clock.advance(MOTOR_REASSERT_MS + 1)
        assert write(arbiter, 0.5, owner=STREAM, ptype="roboclaw") is True

    def test_servo_never_reasserts(self, arbiter, clock):
        write(arbiter, 0.5, owner=STREAM, ptype="maestro")
        clock.advance(MOTOR_REASSERT_MS * 100)
        assert write(arbiter, 0.5, owner=STREAM, ptype="maestro") is False

    def test_neutral_motor_never_reasserts(self, arbiter, clock):
        write(arbiter, 0.0, owner=STREAM, ptype="roboclaw")
        clock.advance(MOTOR_REASSERT_MS * 10)
        assert write(arbiter, 0.0, owner=STREAM, ptype="roboclaw") is False


class TestIdleDisengageExpiry:
    """A Maestro channel with idle_disengage_ms drops PWM and goes limp
    after that window. A repeat of the value it already holds must then
    be allowed through to re-engage it, or the servo stays released."""

    def test_repeat_passes_once_the_channel_has_disengaged(self, clock):
        arb = ChannelArbiter(idle_disengage_lookup=lambda n, p, c: 1000,
                             clock=clock)
        assert write(arb, 0.5, owner=STREAM) is True
        assert write(arb, 0.5, owner=STREAM) is False
        clock.advance(1001)
        assert write(arb, 0.5, owner=STREAM) is True

    def test_always_on_channel_stays_gated(self, clock):
        arb = ChannelArbiter(idle_disengage_lookup=lambda n, p, c: 0,
                             clock=clock)
        write(arb, 0.5, owner=STREAM)
        clock.advance(600_000)
        assert write(arb, 0.5, owner=STREAM) is False


class TestRecordDiscipline:
    def test_a_suppressed_write_is_not_recorded(self, arbiter, clock):
        """record() is only called after a publish actually happens.
        Caching a value the firmware never received is precisely how the
        old caches went stale."""
        write(arbiter, 0.5, owner=STREAM)
        before = arbiter.last_value(NODE, PER, CH)
        clock.advance(10)
        write(arbiter, 0.5 + CHANGE_EPSILON / 4, owner=STREAM)   # suppressed
        assert arbiter.last_value(NODE, PER, CH) == before
