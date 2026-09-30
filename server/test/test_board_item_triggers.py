"""Board-item triggers: an animation firing a sound or another animation.

The animation editor presents poses, sounds and animations under one
"Board Item" affordance. Poses stay VALUE tracks — a pose is a weighted
clip with blending and per-joint overrides, which a one-shot fire cannot
express. Sounds and animations become trigger keyframes, which is what
these cover:

  * dispatch — the right call for the right kind, at the right time;
  * `duration` — the operator-dragged bar. It stops a LOOPING item at
    time+duration and is display-only for a one-shot, because cutting a
    one-shot short is a different feature from choosing how long to
    repeat;
  * the two guards on nesting, which exist because the trigger graph is
    authored and never validated: an animation cannot trigger itself,
    and nesting is depth-limited so a cycle self-reference misses
    (A → B → A) cannot spawn players until the process dies;
  * teardown — whatever an animation started, it stops when it ends.
"""
from __future__ import annotations

import os
import sys

import asyncio

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.models import Animation, TriggerKeyframe
from saint_server.animation.player import AnimationPlayer, MAX_ANIMATION_NESTING


@pytest.fixture(autouse=True)
def event_loop():
    """A loop for the whole module.

    AnimationPlayer builds an asyncio.Event in its constructor, which on
    3.9 binds to the current loop at construction — so even the purely
    synchronous cases here need one present. This repo has no
    pytest-asyncio, so the async cases drive their coroutines directly.
    """
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    yield loop
    loop.close()
    asyncio.set_event_loop(None)


class FakeBoard:
    def __init__(self, looping_sounds=(), looping_anims=(), node="n1"):
        self.calls = []
        self._looping_sounds = set(looping_sounds)
        self._looping_anims = set(looping_anims)
        self._node = node
        self.refuse = False

    def play_sound(self, sound_id):
        self.calls.append(("play_sound", sound_id))
        return self._node

    def stop_sound(self, node_id):
        self.calls.append(("stop_sound", node_id))

    async def start_animation(self, animation_id, depth):
        self.calls.append(("start_animation", animation_id, depth))
        return not self.refuse

    async def stop_animation(self, animation_id):
        self.calls.append(("stop_animation", animation_id))

    def sound_is_looping(self, sound_id):
        return sound_id in self._looping_sounds

    def animation_is_looping(self, animation_id):
        return animation_id in self._looping_anims


class Logged:
    def __init__(self):
        self.lines = []

    def __call__(self, level, msg):
        self.lines.append((level, msg))


def make_player(board=None, anim_id="show", depth=0, triggers=()):
    anim = Animation(id=anim_id, name=anim_id, duration=10.0)
    player = AnimationPlayer(
        anim,
        set_urdf_joint_value=lambda *_: True,
        set_ws_input=lambda *_: True,
        set_topic_channel=lambda *_: True,
        estop_active=lambda: False,
        board=board,
        depth=depth,
    )
    log = Logged()
    player._log = log                                   # noqa: SLF001
    player._logged = log
    return player


def kf(kind, item, t=1.0, duration=0.0):
    return TriggerKeyframe(time=t, target_kind=kind, target=[item],
                           value=None, duration=duration)


# ── sounds ──────────────────────────────────────────────────────────


def test_a_sound_board_item_plays_it():
    board = FakeBoard()
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "fanfare"))
    assert ("play_sound", "fanfare") in board.calls


def test_a_looping_sound_with_a_dragged_length_is_stopped_at_the_end():
    board = FakeBoard(looping_sounds={"siren"})
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "siren", t=1.0, duration=2.0))
    assert p._pending_stops == [(3.0, "sound", "n1")]
    p._fire_pending_stops(2.9, 3.1)
    assert ("stop_sound", "n1") in board.calls


def test_a_one_shot_sound_is_never_cut_short():
    # `duration` on a non-looping item is the display bar only: the clip
    # ends when it ends. Truncating it is a different feature.
    board = FakeBoard()                       # "fanfare" does not loop
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "fanfare", t=1.0, duration=0.5))
    assert p._pending_stops == []


def test_a_looping_sound_with_no_dragged_length_runs_on():
    board = FakeBoard(looping_sounds={"siren"})
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "siren", t=1.0, duration=0.0))
    assert p._pending_stops == []


def test_a_sound_that_cannot_be_resolved_schedules_no_stop():
    board = FakeBoard(looping_sounds={"siren"}, node="")
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "siren", t=1.0, duration=2.0))
    assert p._pending_stops == []


# ── nested animations ───────────────────────────────────────────────


def test_an_animation_board_item_starts_it():
    asyncio.get_event_loop().run_until_complete(_test_an_animation_board_item_starts_it())


async def _test_an_animation_board_item_starts_it():
    board = FakeBoard()
    p = make_player(board, anim_id="show")
    await p._start_nested("wave", kf("animation", "wave"))
    assert ("start_animation", "wave", 1) in board.calls


def test_an_animation_cannot_trigger_itself():
    board = FakeBoard()
    p = make_player(board, anim_id="show")
    p._dispatch_trigger(kf("animation", "show"))
    assert not any(c[0] == "start_animation" for c in board.calls)
    assert any("triggers itself" in m for _l, m in p._logged.lines)


def test_nesting_is_depth_limited():
    board = FakeBoard()
    p = make_player(board, anim_id="deep", depth=MAX_ANIMATION_NESTING - 1)
    p._dispatch_trigger(kf("animation", "another"))
    assert not any(c[0] == "start_animation" for c in board.calls)
    assert any("nests deeper" in m for _l, m in p._logged.lines)


def test_nesting_below_the_limit_is_allowed():
    board = FakeBoard()
    p = make_player(board, anim_id="ok", depth=0)
    p._dispatch_trigger(kf("animation", "another"))
    # Dispatched off the tick path; no running loop in the test, so the
    # coroutine is closed rather than run — what matters is that it got
    # past both guards without logging a refusal.
    assert not any("nests deeper" in m or "triggers itself" in m
                   for _l, m in p._logged.lines)


def test_a_refused_nested_animation_schedules_no_stop():
    asyncio.get_event_loop().run_until_complete(_test_a_refused_nested_animation_schedules_no_stop())


async def _test_a_refused_nested_animation_schedules_no_stop():
    board = FakeBoard(looping_anims={"idle"})
    board.refuse = True
    p = make_player(board)
    await p._start_nested("idle", kf("animation", "idle", t=1.0, duration=3.0))
    assert p._pending_stops == []


def test_a_looping_nested_animation_is_stopped_at_the_dragged_length():
    asyncio.get_event_loop().run_until_complete(_test_a_looping_nested_animation_is_stopped_at_the_dragged_length())


async def _test_a_looping_nested_animation_is_stopped_at_the_dragged_length():
    board = FakeBoard(looping_anims={"idle"})
    p = make_player(board)
    await p._start_nested("idle", kf("animation", "idle", t=1.0, duration=3.0))
    assert p._pending_stops == [(4.0, "animation", "idle")]


# ── windowing + teardown ────────────────────────────────────────────


def test_a_stop_outside_the_window_does_not_fire():
    board = FakeBoard(looping_sounds={"siren"})
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "siren", t=1.0, duration=2.0))
    p._fire_pending_stops(0.0, 1.0)
    assert not any(c[0] == "stop_sound" for c in board.calls)
    assert p._pending_stops


def test_a_stop_fires_once_and_is_forgotten():
    board = FakeBoard(looping_sounds={"siren"})
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "siren", t=1.0, duration=2.0))
    p._fire_pending_stops(2.9, 3.1)
    p._fire_pending_stops(2.9, 3.1)
    assert [c for c in board.calls if c[0] == "stop_sound"] == [("stop_sound", "n1")]


def test_ending_the_animation_stops_what_it_started():
    # A nested looping sound outliving its parent is discovered as a
    # robot that will not stop making noise.
    board = FakeBoard(looping_sounds={"siren"})
    p = make_player(board)
    p._dispatch_trigger(kf("sound", "siren", t=1.0, duration=99.0))
    p._stop_all_board_items()
    assert ("stop_sound", "n1") in board.calls
    assert p._pending_stops == []


# ── degradation ─────────────────────────────────────────────────────


def test_no_board_control_logs_rather_than_raising():
    p = make_player(None)
    p._dispatch_trigger(kf("sound", "fanfare"))
    assert any("no board control" in m for _l, m in p._logged.lines)


def test_a_board_item_with_no_id_is_skipped():
    board = FakeBoard()
    p = make_player(board)
    p._dispatch_trigger(TriggerKeyframe(time=1.0, target_kind="sound",
                                        target=[], value=None))
    assert board.calls == []


# ── the model ───────────────────────────────────────────────────────


def test_duration_round_trips():
    k = TriggerKeyframe(time=1.0, target_kind="sound", target=["s"],
                        value=None, duration=2.5)
    assert TriggerKeyframe.from_dict(k.to_dict()).duration == 2.5


def test_a_missing_duration_is_natural_length():
    k = TriggerKeyframe.from_dict(
        {"time": 1.0, "target_kind": "sound", "target": ["s"]})
    assert k.duration == 0.0


def test_a_negative_duration_reads_as_natural_length():
    k = TriggerKeyframe.from_dict(
        {"time": 1.0, "target_kind": "sound", "target": ["s"], "duration": -4})
    assert k.duration == 0.0
