"""Single arbitration point for peripheral channel writes.

Every writer that commands a peripheral channel — the State tab's
sliders, pose boards, the routing graph (controller sticks, RC, the
animation player) — funnels through
``server_node.send_channel_command``. This module is the decision it
consults: *should this write actually go out, and who owns this channel
now?*

Why it exists
-------------
There used to be two independent "I already sent that" caches:

  * ``websocket_handler._control_last_value`` for slider writes, and
  * ``routing_evaluator._last_channel_sent`` for sheet/pose dispatches.

Neither could see the other's writes, and each was cleared only by a
narrow event (a manual Sync; an e-stop release). Anything else that
moved the hardware — the other writer, a dropped best-effort /control
frame, a firmware idle-disengage, a node reconnect, a Maestro re-plug —
left both caches believing the firmware held a value it did not. The
next command for that value was then suppressed as "unchanged", and the
channel stayed wrong until something happened to command a *different*
number.

On the robot that read as: re-activating a pose board silently did
nothing for exactly the channels you had last nudged by hand, and
dragging a slider back to a value a pose had overwritten did nothing
either. Both reported success.

One cache, updated by every writer, fixes the class of bug. Ownership on
top of it gives the operator-facing precedence rule:

  * A **board** activation is latched authority — it force-sends every
    channel in the pose, so re-activating a board always re-asserts the
    hardware no matter who moved it or what got dropped.
  * A **slider** is the operator's hand. It takes ownership while it
    moves, and its value then stands until the next board activation —
    a parked slider is not a writer and must never block one. It stays
    change-gated against the shared cache, which is correct once that
    cache is trustworthy: re-commanding a value the hardware verifiably
    already holds is a no-op worth skipping.
  * A **stream** (routing sheet, controller stick, animation tick) is
    continuous and unattended, so it stays rate-gated: it re-sends only
    on real change, plus the two liveness exceptions below.

Note the asymmetry is deliberate: suppression exists to stop unattended
50 Hz sources from flooding /control, not to second-guess a human.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

# ── owners ──────────────────────────────────────────────────────────
BOARD = "board"      # pose board activation — latched authority
SLIDER = "slider"    # operator dragging a State-tab control
STREAM = "stream"    # routing sheets: sticks, RC, animation player

# Motors re-assert an unchanged non-neutral value on this cadence so the
# firmware dead-man can tell "held stick" from "dead link". Servos are
# excluded on purpose — their idle_disengage depends on a held channel
# going quiet. Mirrors the constants this replaced in routing_evaluator.
MOTOR_REASSERT_TYPES = frozenset({"roboclaw", "syren"})
MOTOR_REASSERT_MS = 300.0

# Values this close to zero are "stop". A stopped motor needs no
# liveness feed, so an unchanged neutral never re-asserts.
NEUTRAL_EPSILON = 0.02

# Below this delta a stream re-send is considered the same value.
# 0.005 ~ 5 us on a servo's pulse range — under typical servo deadband.
CHANGE_EPSILON = 0.005

ChannelKey = Tuple[str, str, str]   # (node_id, peripheral_id, channel_id)


@dataclass
class _Entry:
    value: Optional[float]   # last value actually sent; None = unknown
    sent_ms: float
    owner: str
    # Last value a STREAM actually put on the wire for this channel.
    # Deliberately separate from `value`: `value` is what the hardware
    # holds (every writer updates it), while this is the stream's own
    # bookkeeping. A stream must re-send when ITS value changes, not
    # when someone else moves the channel — see should_send.
    stream_value: Optional[float] = None


class ChannelArbiter:
    """Owns "what does the firmware hold, and who put it there".

    ``idle_disengage_lookup(node, peripheral, channel) -> int`` reports a
    channel's configured idle-disengage window in ms (0 = always on).
    A Maestro channel that has disengaged has physically gone limp, so a
    repeat of the value it already holds MUST be allowed through to
    re-engage it — that lookup is what makes the dedupe expire instead of
    stranding a released servo.
    """

    def __init__(
        self,
        idle_disengage_lookup: Optional[Callable[[str, str, str], int]] = None,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        self._entries: Dict[ChannelKey, _Entry] = {}
        self._idle_lookup = idle_disengage_lookup
        # Injectable so tests advance time instead of sleeping.
        self._clock = clock or (lambda: time.monotonic() * 1000.0)
        self._lock = threading.Lock()

    # ── decisions ───────────────────────────────────────────────────

    def should_send(
        self,
        node_id: str,
        peripheral_id: str,
        channel_id: str,
        value: Optional[float],
        owner: str = STREAM,
        peripheral_type: str = "",
        raw_us: Optional[int] = None,
    ) -> bool:
        """Decide whether this write reaches the firmware.

        Call :meth:`record` after a send actually goes out — the two are
        separate so a transport failure doesn't poison the cache with a
        value the firmware never received.
        """
        key = (node_id, peripheral_id, channel_id)
        now = self._clock()

        with self._lock:
            entry = self._entries.get(key)

            # Absolute-microsecond jog (Maestro extent dialer). Always
            # goes out, and it moves the servo off the normalized map
            # entirely — so the cached normalized value is now a lie and
            # must be dropped, or dragging the slider back to where it
            # was would be swallowed as "unchanged". That exact sequence
            # was a reproducible dead slider.
            if raw_us is not None:
                if entry is not None:
                    entry.value = None
                    entry.stream_value = None
                return True

            if value is None:
                return True

            # Latched operator authority. A board activation re-asserts
            # its entire pose unconditionally — that is what makes
            # re-clicking a board always work after a slider nudge, a
            # dropped /control frame, or a firmware re-home, and it is
            # the only recovery path the system has for a write that was
            # lost in flight (nothing else ever retries a servo).
            #
            # Sliders deliberately do NOT force. If the cache says the
            # hardware already holds this value, and the cache is now
            # trustworthy (one shared record, invalidated on every event
            # that moves hardware behind our back), then not sending is
            # correct — and keeping sliders gated preserves two things
            # that matter: /control doesn't take a 20 Hz stream of
            # identical values from a parked control, and a held servo
            # channel still goes quiet long enough for the firmware's
            # idle_disengage to fire.
            if owner == BOARD:
                return True

            if entry is None:
                return True

            # Which value does this writer measure "change" against?
            #
            # A SLIDER is the operator's hand, so it asks the useful
            # question: does the hardware already hold this? That is
            # `entry.value`, which every writer updates — it is what
            # lets a slider command a value a board overwrote.
            #
            # A STREAM is unattended and re-dispatches continuously, so
            # it must ask a different question: has MY value changed
            # since I last sent it? Comparing a stream against
            # `entry.value` makes it re-assert every time another owner
            # writes the channel — and a pose board's setpoints stay in
            # the evaluator's caches long after the activation, so every
            # sheet evaluation re-offers them. Gated the wrong way, that
            # stale pose value snaps the servo back on the very next
            # tick and the State sliders become unusable.
            reference = entry.stream_value if owner == STREAM else entry.value
            if reference is None:
                return True

            if abs(value - reference) >= CHANGE_EPSILON:
                return True

            # Unchanged stream value from here down.
            if abs(value) <= NEUTRAL_EPSILON:
                return False

            since = now - entry.sent_ms
            if (peripheral_type in MOTOR_REASSERT_TYPES
                    and since >= MOTOR_REASSERT_MS):
                return True

            idle_ms = self._idle_disengage_ms(node_id, peripheral_id, channel_id)
            if idle_ms > 0 and since >= idle_ms:
                return True

            return False

    def record(
        self,
        node_id: str,
        peripheral_id: str,
        channel_id: str,
        value: Optional[float],
        owner: str = STREAM,
        raw_us: Optional[int] = None,
    ) -> None:
        """Note a write that actually reached the firmware."""
        key = (node_id, peripheral_id, channel_id)
        now = self._clock()
        with self._lock:
            # A raw-us jog leaves the normalized value unknown (see
            # should_send) but still marks ownership and liveness. It
            # also moves the servo off the normalized map, so the
            # stream's bookkeeping is void too.
            cached = None if raw_us is not None else value
            prev = self._entries.get(key)
            if raw_us is not None:
                stream_value = None
            elif owner == STREAM:
                stream_value = value
            else:
                # A board or slider write does not change what the
                # stream last emitted, so its gate must survive: this is
                # what keeps a held pose value quiet after the operator
                # has moved the channel by hand.
                stream_value = prev.stream_value if prev else None
            self._entries[key] = _Entry(value=cached, sent_ms=now,
                                        owner=owner, stream_value=stream_value)

    # ── invalidation ────────────────────────────────────────────────
    #
    # Every one of these is a moment the hardware's state changed
    # underneath us, so "the firmware already holds X" stops being true.
    # Missing these hooks is what made the old caches go stale and stay
    # stale.

    def invalidate_node(self, node_id: str, reason: str = "") -> int:
        """Forget everything cached for one node. Call on reconnect, on
        config sync, on e-stop — anything that re-homes or releases its
        channels. Returns how many entries were dropped."""
        with self._lock:
            stale = [k for k in self._entries if k[0] == node_id]
            for k in stale:
                self._entries.pop(k, None)
            return len(stale)

    def invalidate_peripheral(self, node_id: str, peripheral_id: str) -> int:
        """Forget one peripheral's channels — e.g. a Maestro that just
        re-enumerated on USB and drove every channel to its home."""
        with self._lock:
            stale = [k for k in self._entries
                     if k[0] == node_id and k[1] == peripheral_id]
            for k in stale:
                self._entries.pop(k, None)
            return len(stale)

    def invalidate_all(self, reason: str = "") -> int:
        with self._lock:
            n = len(self._entries)
            self._entries.clear()
            return n

    # ── introspection (UI / diagnostics) ────────────────────────────

    def owner_of(self, node_id: str, peripheral_id: str,
                 channel_id: str) -> Optional[str]:
        with self._lock:
            entry = self._entries.get((node_id, peripheral_id, channel_id))
            return entry.owner if entry else None

    def last_value(self, node_id: str, peripheral_id: str,
                   channel_id: str) -> Optional[float]:
        with self._lock:
            entry = self._entries.get((node_id, peripheral_id, channel_id))
            return entry.value if entry else None

    def snapshot(self) -> Dict[str, Dict[str, object]]:
        """Flat view for diagnostics: "who last wrote each channel"."""
        with self._lock:
            return {
                "/".join(k): {"value": e.value, "owner": e.owner}
                for k, e in self._entries.items()
            }

    # ── internals ───────────────────────────────────────────────────

    def _idle_disengage_ms(self, node_id: str, peripheral_id: str,
                           channel_id: str) -> int:
        if self._idle_lookup is None:
            return 0
        try:
            return max(0, int(self._idle_lookup(node_id, peripheral_id, channel_id)))
        except Exception:
            # A lookup failure must not silently turn into "never expire"
            # for a channel that disengages; 0 is the conservative answer
            # only because the caller already handles the changed-value
            # case. Swallowing is deliberate: this runs on the control
            # hot path and a config-shaped surprise shouldn't drop writes.
            return 0
