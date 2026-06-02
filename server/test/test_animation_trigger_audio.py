"""Tests for the animation trigger-track → peripheral_command path.

Animations align value tracks (joint movements) with trigger tracks
(discrete events). The audio_player use case adds a third dispatch
target: a peripheral_command trigger that sends play_file("foo.wav")
to a Pi node at a specific timecode in the animation. These tests
exercise that dispatch path end-to-end:

  1. Model roundtrip: a peripheral_command keyframe survives
     Animation.to_dict() → Animation.from_dict() with its dict-shaped
     value intact (previously TriggerKeyframe assumed float values).
  2. Player wiring: AnimationPlayer fires the registered callback at
     the right timecode with the right (node, peripheral, command,
     args).
  3. Player ergonomics: a bare string value desugars to
     play_file(filename) so the UI doesn't have to construct the
     nested object every time.
  4. Backward-compat: existing ws_input and topic triggers still
     dispatch through their numeric paths and are unaffected.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.models import (
    Animation,
    TriggerKeyframe,
    TriggerTrack,
)
from saint_server.animation.player import AnimationPlayer


# ── helpers ─────────────────────────────────────────────────────────


def _anim_with_trigger(kf: TriggerKeyframe) -> Animation:
    return Animation(
        id="anim_audio_cue",
        name="audio cue",
        duration=10.0,
        fps=60,
        loop=False,
        trigger_tracks=[TriggerTrack(id="t0", name="cues", keyframes=[kf])],
    )


class _CallSpy:
    def __init__(self):
        self.calls = []

    def __call__(self, *args, **kwargs):
        self.calls.append((args, dict(kwargs)))


def _make_player(anim, send_peripheral_command=None):
    """Build a player wired to spies for every dispatch path. Doesn't
    start the asyncio task — we drive _fire_triggers directly so the
    tests stay synchronous and deterministic."""
    return AnimationPlayer(
        anim,
        set_urdf_joint_value=_CallSpy(),
        set_ws_input=_CallSpy(),
        set_topic_channel=_CallSpy(),
        send_peripheral_command=send_peripheral_command,
        estop_active=lambda: False,
    )


# ── 1. Model roundtrip ─────────────────────────────────────────────


def test_peripheral_command_keyframe_roundtrips():
    """The animation JSON on disk must preserve the dict-shaped value
    for a peripheral_command trigger. Earlier TriggerKeyframe code
    only flowed numeric values reliably through from_dict; the
    Any-typed `value` field has to survive a json-style roundtrip."""
    kf = TriggerKeyframe(
        time=3.5,
        target_kind="peripheral_command",
        target=["rpi5_abc123", "onboard_audio"],
        value={"command": "play_file", "args": {"filename": "intro.wav"}},
        label="audio cue at 3.5s",
    )
    anim = _anim_with_trigger(kf)

    d = anim.to_dict()
    restored = Animation.from_dict(d)

    track = restored.trigger_tracks[0]
    assert len(track.keyframes) == 1
    restored_kf = track.keyframes[0]
    assert restored_kf.target_kind == "peripheral_command"
    assert restored_kf.target == ["rpi5_abc123", "onboard_audio"]
    assert restored_kf.value == {
        "command": "play_file",
        "args": {"filename": "intro.wav"},
    }
    assert restored_kf.time == 3.5
    assert restored_kf.label == "audio cue at 3.5s"


def test_peripheral_command_keyframe_with_bare_string_value():
    """Operators authoring a quick audio cue should be able to type
    just the filename without constructing the nested {command, args}
    object. The string must survive roundtrip too."""
    kf = TriggerKeyframe(
        time=1.0,
        target_kind="peripheral_command",
        target=["rpi5_abc", "onboard_audio"],
        value="ding.wav",
    )
    anim = _anim_with_trigger(kf)
    restored = Animation.from_dict(anim.to_dict())
    assert restored.trigger_tracks[0].keyframes[0].value == "ding.wav"


# ── 2. Player wiring ───────────────────────────────────────────────


def test_player_fires_peripheral_command_at_crossed_timecode():
    """The playhead crossing a keyframe's time must invoke the
    send_peripheral_command callback with the parsed args."""
    kf = TriggerKeyframe(
        time=2.0,
        target_kind="peripheral_command",
        target=["rpi5_xyz", "onboard_audio"],
        value={"command": "play_file", "args": {"filename": "boom.wav"}},
    )
    spy = _CallSpy()
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=spy)

    # Simulate playhead crossing t=2.0.
    player._fire_triggers(1.9, 2.1)
    assert len(spy.calls) == 1
    args, _ = spy.calls[0]
    node_id, peripheral_id, command, kwargs = args
    assert node_id == "rpi5_xyz"
    assert peripheral_id == "onboard_audio"
    assert command == "play_file"
    assert kwargs == {"filename": "boom.wav"}


def test_player_skips_trigger_before_playhead_reaches_it():
    """A trigger at t=5 must NOT fire when the playhead is at t=3."""
    kf = TriggerKeyframe(
        time=5.0, target_kind="peripheral_command",
        target=["n", "p"], value="x.wav",
    )
    spy = _CallSpy()
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=spy)
    player._fire_triggers(2.9, 3.0)
    assert spy.calls == []


def test_player_does_not_double_fire_when_window_already_passed():
    """The (t_prev, t_now] half-open window means each trigger fires
    exactly once per pass through its time. A second tick whose
    t_prev is already past the keyframe must NOT re-fire it."""
    kf = TriggerKeyframe(
        time=2.0, target_kind="peripheral_command",
        target=["n", "p"], value="x.wav",
    )
    spy = _CallSpy()
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=spy)
    player._fire_triggers(1.9, 2.1)
    player._fire_triggers(2.1, 2.5)
    assert len(spy.calls) == 1


def test_bare_string_value_desugars_to_play_file():
    """The most common audio-cue authoring shape — operator types a
    filename. The player should expand that into
    play_file({filename})."""
    kf = TriggerKeyframe(
        time=1.0,
        target_kind="peripheral_command",
        target=["rpi5_abc", "onboard_audio"],
        value="hello.wav",
    )
    spy = _CallSpy()
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=spy)
    player._fire_triggers(0.0, 1.5)
    args, _ = spy.calls[0]
    assert args[2] == "play_file"
    assert args[3] == {"filename": "hello.wav"}


def test_player_warns_when_no_callback_wired_but_does_not_crash():
    """If state_manager hasn't been handed a send_peripheral_command
    callable yet (e.g. server_node still booting), a peripheral_command
    trigger must drop with a warn — not raise. The other tracks on the
    same animation must keep ticking."""
    kf = TriggerKeyframe(
        time=1.0, target_kind="peripheral_command",
        target=["n", "p"], value="x.wav",
    )
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=None)
    # Must not raise.
    player._fire_triggers(0.0, 1.5)


def test_missing_target_is_logged_and_skipped():
    """Defensive: a malformed keyframe (empty target list) must not
    crash the player. Other tracks should keep firing."""
    bad = TriggerKeyframe(
        time=1.0, target_kind="peripheral_command",
        target=[], value="x.wav",
    )
    good = TriggerKeyframe(
        time=1.2, target_kind="peripheral_command",
        target=["n", "p"], value="ok.wav",
    )
    anim = Animation(
        id="a", name="a", duration=5.0, fps=60,
        trigger_tracks=[TriggerTrack(id="t", name="t",
                                     keyframes=[bad, good])],
    )
    spy = _CallSpy()
    player = _make_player(anim, send_peripheral_command=spy)
    player._fire_triggers(0.0, 2.0)
    # Bad keyframe dropped; good one still fires.
    assert len(spy.calls) == 1
    assert spy.calls[0][0][1] == "p"


def test_invalid_value_type_is_logged_and_skipped():
    """value=42 (a bare int) is neither a dict nor a string — the
    player should log a warn and skip rather than crash with
    AttributeError on .get."""
    kf = TriggerKeyframe(
        time=1.0, target_kind="peripheral_command",
        target=["n", "p"], value=42,
    )
    spy = _CallSpy()
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=spy)
    player._fire_triggers(0.0, 1.5)
    assert spy.calls == []


def test_missing_command_field_is_skipped():
    """A dict value with no `command` key has nothing to dispatch."""
    kf = TriggerKeyframe(
        time=1.0, target_kind="peripheral_command",
        target=["n", "p"], value={"args": {"filename": "x.wav"}},
    )
    spy = _CallSpy()
    player = _make_player(_anim_with_trigger(kf), send_peripheral_command=spy)
    player._fire_triggers(0.0, 1.5)
    assert spy.calls == []


# ── 3. Estop suppresses triggers (existing behavior, regression guard) ──


def test_estop_suppresses_peripheral_command_triggers():
    """Trigger tracks must respect the system e-stop. An estop-active
    state means no commands fire — including audio cues, which
    matters for "kill the music" semantics during an emergency."""
    kf = TriggerKeyframe(
        time=1.0, target_kind="peripheral_command",
        target=["n", "p"], value="x.wav",
    )
    spy = _CallSpy()
    player = AnimationPlayer(
        _anim_with_trigger(kf),
        set_urdf_joint_value=_CallSpy(),
        set_ws_input=_CallSpy(),
        set_topic_channel=_CallSpy(),
        send_peripheral_command=spy,
        estop_active=lambda: True,    # estop ON
    )
    player._fire_triggers(0.0, 1.5)
    assert spy.calls == []


# ── 4. Backward compat ─────────────────────────────────────────────


def test_ws_input_trigger_still_dispatches_numerically():
    """Adding the new target_kind must not change the existing
    ws_input dispatch path."""
    kf = TriggerKeyframe(
        time=1.0, target_kind="ws_input",
        target=["sheet1", "wsin1"], value=0.5,
    )
    set_ws = _CallSpy()
    player = AnimationPlayer(
        _anim_with_trigger(kf),
        set_urdf_joint_value=_CallSpy(),
        set_ws_input=set_ws,
        set_topic_channel=_CallSpy(),
        send_peripheral_command=None,
        estop_active=lambda: False,
    )
    player._fire_triggers(0.0, 1.5)
    assert len(set_ws.calls) == 1
    args, _ = set_ws.calls[0]
    assert args == ("sheet1", "wsin1", 0.5)


def test_topic_trigger_still_dispatches_numerically():
    kf = TriggerKeyframe(
        time=1.0, target_kind="topic",
        target=["/cmd_audio", "trigger"], value=1.0,
    )
    set_topic = _CallSpy()
    player = AnimationPlayer(
        _anim_with_trigger(kf),
        set_urdf_joint_value=_CallSpy(),
        set_ws_input=_CallSpy(),
        set_topic_channel=set_topic,
        send_peripheral_command=None,
        estop_active=lambda: False,
    )
    player._fire_triggers(0.0, 1.5)
    assert len(set_topic.calls) == 1
    args, _ = set_topic.calls[0]
    assert args[:3] == ("/cmd_audio", "trigger", 1.0)
    # The 4th arg is the source label _animation_<id>.
    assert args[3] == "_animation_anim_audio_cue"


# ── 5. Mixed trigger types in one animation ───────────────────────


def test_mixed_targets_fan_out_to_correct_callbacks():
    """A movement-sequence animation typically combines:
      - value tracks driving joints,
      - ws_input triggers steering routing,
      - a peripheral_command trigger firing the audio cue.
    Verify they all land on the right callable from one tick."""
    anim = Animation(
        id="multi", name="multi", duration=10.0, fps=60,
        trigger_tracks=[
            TriggerTrack(id="audio", name="audio", keyframes=[
                TriggerKeyframe(
                    time=1.0, target_kind="peripheral_command",
                    target=["rpi5_main", "onboard_audio"],
                    value="cue.wav",
                ),
            ]),
            TriggerTrack(id="routing", name="routing", keyframes=[
                TriggerKeyframe(
                    time=1.5, target_kind="ws_input",
                    target=["sheet1", "wsin_brake"], value=1.0,
                ),
            ]),
        ],
    )
    audio_spy = _CallSpy()
    ws_spy = _CallSpy()
    player = AnimationPlayer(
        anim,
        set_urdf_joint_value=_CallSpy(),
        set_ws_input=ws_spy,
        set_topic_channel=_CallSpy(),
        send_peripheral_command=audio_spy,
        estop_active=lambda: False,
    )
    player._fire_triggers(0.0, 2.0)
    assert len(audio_spy.calls) == 1
    assert len(ws_spy.calls) == 1
    assert audio_spy.calls[0][0][2] == "play_file"
    assert ws_spy.calls[0][0] == ("sheet1", "wsin_brake", 1.0)
