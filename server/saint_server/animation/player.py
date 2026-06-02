"""Animation playback engine.

One ``AnimationPlayer`` per active animation. Owns the timeline clock,
samples value tracks each tick into the routing evaluator's animation
cache, and fires trigger keyframes when they're crossed.

The player is asyncio-based — it spawns a task that sleeps between
ticks so a 60 Hz animation costs roughly 60 wake-ups/sec without
burning a thread. The routing evaluator's methods are documented as
thread-safe via its internal lock, so calling them from the asyncio
loop is fine.
"""

from __future__ import annotations

import asyncio
import time
from typing import Awaitable, Callable, Dict, List, Optional

from saint_server.animation.models import (
    Animation,
    TriggerKeyframe,
    TriggerTrack,
)


# Dispatch protocol: the player doesn't import the evaluator or bridge
# directly to keep the dependency graph one-way. The registry wires up
# callables that fan out to those modules.
#
# ``SetUrdfJointValue(joint_name, value)`` — pushes a sampled track
# value into the evaluator's URDF-joint cache so any InputNode whose
# ``kind == "urdf_joint"`` and matching ``joint`` field receives the
# update. The track's id is treated as the joint name (operators bind
# tracks to joints in the animation editor by giving the track the
# joint's URDF name).
SetUrdfJointValue = Callable[[str, float], bool]
SetWSInput = Callable[[str, str, float], bool]
SetTopicChannel = Callable[[str, str, float, str], Dict]
# Out-of-band peripheral command — used by trigger tracks to fire
# string-arg commands like audio_player.play_file. Signature matches
# server_node.send_peripheral_command (node_id, peripheral_id, command,
# args). Optional: if None at construction, peripheral_command
# triggers are dropped with a warn.
SendPeripheralCommand = Callable[[str, str, str, Dict], None]
EstopGate = Callable[[], bool]   # returns True iff estop is engaged


# Floor on the per-tick sleep. Going below this is wasteful — asyncio's
# scheduler has its own resolution and 1 ms is well past it. A 60 Hz
# animation has dt=16.7 ms, a 30 Hz has dt=33.3 ms — both comfortable.
_MIN_TICK_SLEEP = 0.001


class AnimationPlayer:
    """Plays back a single Animation.

    Holds wall-clock state (``_t``) advanced on each tick. The player
    decides when triggers fire based on the (t_prev, t_now] window
    crossed during a tick, so a brief pause won't drop triggers that
    fall inside the resumed window.
    """

    def __init__(
        self,
        anim: Animation,
        set_urdf_joint_value: SetUrdfJointValue,
        set_ws_input: SetWSInput,
        set_topic_channel: SetTopicChannel,
        estop_active: EstopGate,
        send_peripheral_command: Optional[SendPeripheralCommand] = None,
        on_finished: Optional[Callable[[str], None]] = None,
        logger=None,
    ):
        self.anim = anim
        self._set_urdf_joint_value = set_urdf_joint_value
        self._set_ws_input = set_ws_input
        self._set_topic_channel = set_topic_channel
        self._send_peripheral_command = send_peripheral_command
        self._estop_active = estop_active
        self._on_finished = on_finished
        self.logger = logger

        self._t = 0.0
        self._task: Optional[asyncio.Task] = None
        self._paused = asyncio.Event()
        self._paused.set()      # set = running; cleared = paused
        self._stop_requested = False

    # ── lifecycle ───────────────────────────────────────────────────

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._stop_requested = False
        self._task = asyncio.create_task(self._run())

    def pause(self) -> None:
        self._paused.clear()

    def resume(self) -> None:
        self._paused.set()

    def seek(self, t: float) -> None:
        self._t = max(0.0, float(t))

    async def stop(self) -> None:
        self._stop_requested = True
        self._paused.set()      # so the wait loop wakes
        if self._task is not None:
            try:
                await self._task
            except Exception:
                pass
        self._task = None
        # Drop cached values so downstream peripherals settle at neutral
        # rather than holding the last animation frame indefinitely.
        for track in self.anim.value_tracks:
            self._set_urdf_joint_value(track.id, 0.0)

    @property
    def is_running(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def is_paused(self) -> bool:
        return not self._paused.is_set()

    @property
    def current_time(self) -> float:
        return self._t

    def state_snapshot(self) -> Dict:
        return {
            "id": self.anim.id,
            "name": self.anim.name,
            "duration": self.anim.duration,
            "t": self._t,
            "running": self.is_running,
            "paused": self.is_paused,
            "loop": self.anim.loop,
        }

    # ── tick loop ───────────────────────────────────────────────────

    async def _run(self) -> None:
        fps = max(1, int(self.anim.fps or 60))
        dt = 1.0 / fps
        loop = asyncio.get_event_loop()
        next_tick = loop.time()
        last_t = self._t

        try:
            while not self._stop_requested:
                if not self._paused.is_set():
                    await self._paused.wait()
                    if self._stop_requested:
                        break
                    # Resume — drop the carry-forward so triggers from
                    # the pause window don't all fire at resume.
                    last_t = self._t
                    next_tick = loop.time()

                # Sample value tracks first so the routing graph sees
                # the new values before we dispatch triggers (which
                # may rely on the same animation's value tracks via
                # downstream operators).
                self._tick_value_tracks(self._t)
                self._fire_triggers(last_t, self._t)

                last_t = self._t
                self._t += dt
                if self._t >= self.anim.duration and self.anim.duration > 0:
                    if self.anim.loop:
                        self._t = 0.0
                        last_t = 0.0
                    else:
                        # Land one final frame at duration so the value
                        # tracks reach their last keyframe before we stop.
                        self._tick_value_tracks(self.anim.duration)
                        break

                next_tick += dt
                sleep = max(_MIN_TICK_SLEEP, next_tick - loop.time())
                await asyncio.sleep(sleep)
        finally:
            if self._on_finished is not None:
                try:
                    self._on_finished(self.anim.id)
                except Exception as e:
                    self._log("error", f"on_finished callback failed: {e}")

    def _tick_value_tracks(self, t: float) -> None:
        for track in self.anim.value_tracks:
            try:
                v = track.value_at(t)
                # Track id is the URDF joint name — see the
                # SetUrdfJointValue docstring above.
                self._set_urdf_joint_value(track.id, v)
            except Exception as e:
                self._log("warn",
                          f"value-track {self.anim.id}/{track.id} failed: {e}")

    def _fire_triggers(self, t_prev: float, t_now: float) -> None:
        if t_now <= t_prev:
            return
        estop = False
        try:
            estop = self._estop_active()
        except Exception:
            pass
        if estop:
            # E-stop suppresses triggers entirely — they're discrete
            # commands. Value tracks still update the cache (handled by
            # the caller); the sink gate in the evaluator drops the
            # downstream peripheral writes.
            return

        for track in self.anim.trigger_tracks:
            for kf in track.fires_in(t_prev, t_now):
                self._dispatch_trigger(kf)

    def _dispatch_trigger(self, kf: TriggerKeyframe) -> None:
        try:
            if kf.target_kind == "ws_input":
                if len(kf.target) < 2:
                    return
                sheet_id, ws_input_id = kf.target[0], kf.target[1]
                self._set_ws_input(sheet_id, ws_input_id, float(kf.value or 0.0))
            elif kf.target_kind == "topic":
                if len(kf.target) < 2:
                    return
                endpoint, field = kf.target[0], kf.target[1]
                self._set_topic_channel(
                    endpoint, field, float(kf.value or 0.0),
                    f"_animation_{self.anim.id}",
                )
            elif kf.target_kind == "peripheral_command":
                self._dispatch_peripheral_command(kf)
            else:
                self._log("warn",
                          f"Unknown trigger target_kind: {kf.target_kind}")
        except Exception as e:
            self._log("error",
                      f"Trigger dispatch failed at t={kf.time}: {e}")

    def _dispatch_peripheral_command(self, kf: TriggerKeyframe) -> None:
        """Fire a peripheral_command trigger — the path animations use
        to send audio_player.play_file (or any future non-numeric
        peripheral command) at a specific timecode.

        target is [node_id, peripheral_id]; value carries the command
        + args. Bare-string values are desugared to play_file so the
        TriggerEditor UI can offer a single "filename" field without
        forcing operators to construct a nested object."""
        if len(kf.target) < 2:
            self._log("warn",
                      f"peripheral_command trigger needs [node_id, "
                      f"peripheral_id] target, got {kf.target!r}")
            return
        if self._send_peripheral_command is None:
            self._log("warn",
                      f"peripheral_command trigger at t={kf.time} but no "
                      f"send_peripheral_command callback wired; dropping")
            return
        node_id, peripheral_id = kf.target[0], kf.target[1]

        command: str
        args: Dict
        v = kf.value
        if isinstance(v, dict):
            command = str(v.get("command") or "")
            args_in = v.get("args") or {}
            args = dict(args_in) if isinstance(args_in, dict) else {}
        elif isinstance(v, str):
            # Operator typed a bare filename — the common case.
            command = "play_file"
            args = {"filename": v}
        else:
            self._log("warn",
                      f"peripheral_command value must be a dict or "
                      f"filename string; got {type(v).__name__}")
            return
        if not command:
            self._log("warn",
                      f"peripheral_command trigger missing command field")
            return
        self._send_peripheral_command(node_id, peripheral_id, command, args)

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


class AnimationPlayerRegistry:
    """Tracks active AnimationPlayer instances keyed by animation id.

    Starting an already-playing animation replaces the live player —
    the operator's intent is "start from the beginning" rather than
    accumulating instances. The registry takes ownership of the
    callable wiring so individual players don't need to know about
    the evaluator or bridge.
    """

    def __init__(
        self,
        set_urdf_joint_value: SetUrdfJointValue,
        set_ws_input: SetWSInput,
        set_topic_channel: SetTopicChannel,
        estop_active: EstopGate,
        send_peripheral_command: Optional[SendPeripheralCommand] = None,
        logger=None,
    ):
        self._set_urdf_joint_value = set_urdf_joint_value
        self._set_ws_input = set_ws_input
        self._set_topic_channel = set_topic_channel
        self._send_peripheral_command = send_peripheral_command
        self._estop_active = estop_active
        self.logger = logger
        self._players: Dict[str, AnimationPlayer] = {}

    async def start(self, anim: Animation, loop: Optional[bool] = None) -> AnimationPlayer:
        # Stop any prior instance — start means "start from t=0".
        if anim.id in self._players:
            await self._players[anim.id].stop()
        if loop is not None:
            anim.loop = bool(loop)
        player = AnimationPlayer(
            anim,
            set_urdf_joint_value=self._set_urdf_joint_value,
            set_ws_input=self._set_ws_input,
            set_topic_channel=self._set_topic_channel,
            send_peripheral_command=self._send_peripheral_command,
            estop_active=self._estop_active,
            on_finished=self._on_player_finished,
            logger=self.logger,
        )
        self._players[anim.id] = player
        await player.start()
        return player

    async def stop(self, animation_id: str) -> bool:
        player = self._players.get(animation_id)
        if player is None:
            return False
        await player.stop()
        self._players.pop(animation_id, None)
        return True

    def pause(self, animation_id: str) -> bool:
        player = self._players.get(animation_id)
        if player is None:
            return False
        player.pause()
        return True

    def resume(self, animation_id: str) -> bool:
        player = self._players.get(animation_id)
        if player is None:
            return False
        player.resume()
        return True

    def seek(self, animation_id: str, t: float) -> bool:
        player = self._players.get(animation_id)
        if player is None:
            return False
        player.seek(t)
        return True

    def state(self) -> List[Dict]:
        return [p.state_snapshot() for p in self._players.values()]

    def is_active(self, animation_id: str) -> bool:
        return animation_id in self._players

    async def stop_all(self) -> None:
        for aid in list(self._players.keys()):
            await self.stop(aid)

    def _on_player_finished(self, animation_id: str) -> None:
        # Drop from the registry when the player completes naturally.
        # Stop() already handles this when explicitly invoked; this
        # covers the non-looping run-to-end path.
        self._players.pop(animation_id, None)
