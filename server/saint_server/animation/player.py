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
from typing import Awaitable, Callable, Dict, List, Optional, Protocol, Tuple

from saint_server.animation.frame import (
    PoseLookup,
    relaxed_frame,
    resolve_frame,
)
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
# ``ApplyFrame(joint_values, ws_values)`` — applies a whole tick's worth
# of value-track setpoints in ONE evaluator call (one sheet eval per
# touched sheet + one UI-snapshot broadcast), instead of one re-eval +
# broadcast PER track. Optional; when None the player falls back to the
# per-track set_* path. See RoutingEvaluator.apply_animation_frame.
ApplyFrame = Callable[[Dict[str, float], Dict], bool]
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


#: How many animations may nest before we refuse to start another.
#: An animation triggering an animation is a legitimate way to build a
#: show out of parts, but the graph is authored, not validated — a cycle
#: that self-reference checks miss (A → B → A) would otherwise spawn
#: players until the process died. Four is deeper than any real show and
#: shallow enough to stay debuggable.
MAX_ANIMATION_NESTING = 4


class BoardControl(Protocol):
    """What a player needs to fire board items.

    Kept as one object rather than four loose callables because the
    registry supplies all of them together, and a partially-wired set is
    a worse failure than none.
    """

    def play_sound(self, sound_id: str) -> Optional[str]:
        """Play a soundboard entry. Returns the node it plays on (so it
        can be stopped again), or None if it could not be resolved."""

    def stop_sound(self, node_id: str) -> None:
        ...

    async def start_animation(self, animation_id: str, depth: int) -> bool:
        """Start a nested animation. Returns False if it was refused."""

    async def stop_animation(self, animation_id: str) -> None:
        ...

    def sound_is_looping(self, sound_id: str) -> bool:
        ...

    def animation_is_looping(self, animation_id: str) -> bool:
        ...

    def sound_length(self, sound_id: str) -> float:
        """The clip's length in seconds, 0 if unknown. Optional: a board
        without it makes every started sound count as still playing when
        the animation stops."""
        ...


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
        apply_frame: Optional[ApplyFrame] = None,
        on_finished: Optional[Callable[[str], None]] = None,
        pose_lookup: Optional[PoseLookup] = None,
        neutral: Optional[Dict[str, float]] = None,
        board: Optional["BoardControl"] = None,
        depth: int = 0,
        logger=None,
    ):
        self.anim = anim
        # Board-item control (play a sound, start a nested animation).
        # Absent = those triggers log and are skipped rather than failing
        # the whole animation.
        self._board = board
        # How many animations deep this one is. Guards runaway nesting —
        # see _dispatch_board_item.
        self._depth = depth
        # Board items this animation started that must be stopped again,
        # as (at_time, kind, key). Only looping items land here: a
        # one-shot ends on its own.
        self._pending_stops: List[Tuple[float, str, str]] = []
        # Everything this animation started, so stopping the animation
        # (Stop, or reaching its end) stops it too. Sounds are keyed by
        # node, since playback is stop-and-replace per node, and carry the
        # wall-clock time the clip ends (None = loops, or length unknown).
        # A sound already past its end is left alone at stop: stopping its
        # node then would cut off whatever the operator played since.
        self._started_sounds: Dict[str, Optional[float]] = {}
        self._started_animations: set = set()
        self._set_urdf_joint_value = set_urdf_joint_value
        self._set_ws_input = set_ws_input
        self._set_topic_channel = set_topic_channel
        self._apply_frame = apply_frame
        self._send_peripheral_command = send_peripheral_command
        self._estop_active = estop_active
        self._on_finished = on_finished
        # Pose tracks resolve their weight curve against a pose's joint
        # values; without a lookup they contribute nothing. `neutral` is
        # the base a pose blends up from on untouched joints — the rig's
        # neutral pose when there is one, else implicit zeros.
        self._pose_lookup = pose_lookup
        self._neutral = neutral or {}
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
        # Drive every target this animation touches back to neutral, so
        # downstream peripherals settle rather than holding the last
        # frame indefinitely. Note this is NOT "resolve the frame with
        # zero weights" — a pose track at weight 0 contributes nothing,
        # which would strand the joints it was moving. See
        # frame.relaxed_frame.
        joint_values, ws_values = relaxed_frame(
            self.anim, pose_lookup=self._pose_lookup, neutral=self._neutral)
        if not joint_values and not ws_values:
            return
        if self._apply_frame is not None:
            try:
                self._apply_frame(joint_values, ws_values)
            except Exception as e:
                self._log("error", f"relax frame {self.anim.id} failed: {e}")
            return
        for joint, value in joint_values.items():
            try:
                self._set_urdf_joint_value(joint, value)
            except Exception as e:
                self._log("warn", f"relax joint {joint} failed: {e}")
        for (sheet_id, input_id), value in ws_values.items():
            try:
                self._set_ws_input(sheet_id, input_id, value)
            except Exception as e:
                self._log("warn",
                          f"relax ws {sheet_id}/{input_id} failed: {e}")

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
        # True while last_t has not been played yet, so a trigger sitting
        # exactly on it still fires. See TriggerTrack.fires_in.
        include_start = True

        try:
            while not self._stop_requested:
                if not self._paused.is_set():
                    await self._paused.wait()
                    if self._stop_requested:
                        break
                    # Resume — drop the carry-forward so triggers from
                    # the pause window don't all fire at resume.
                    last_t = self._t
                    include_start = True
                    next_tick = loop.time()

                # Sample value tracks first so the routing graph sees
                # the new values before we dispatch triggers (which
                # may rely on the same animation's value tracks via
                # downstream operators).
                self._tick_value_tracks(self._t)
                self._fire_triggers(last_t, self._t, include_start)
                self._fire_pending_stops(last_t, self._t)
                include_start = False

                last_t = self._t
                self._t += dt
                if self._t >= self.anim.duration and self.anim.duration > 0:
                    if self.anim.loop:
                        self._t = 0.0
                        last_t = 0.0
                        include_start = True
                    else:
                        # Land one final frame at duration so the value
                        # tracks reach their last keyframe before we stop.
                        self._tick_value_tracks(self.anim.duration)
                        break

                next_tick += dt
                sleep = max(_MIN_TICK_SLEEP, next_tick - loop.time())
                await asyncio.sleep(sleep)
        finally:
            # Whatever this animation started, it owns until it ends.
            self._stop_all_board_items()
            if self._on_finished is not None:
                try:
                    self._on_finished(self.anim.id)
                except Exception as e:
                    self._log("error", f"on_finished callback failed: {e}")

    def _tick_value_tracks(self, t: float) -> None:
        """Resolve one frame and push it into the routing graph.

        Resolution (including pose-track layering, which is order
        dependent) lives in ``frame.resolve_frame`` so the player, the
        editor's Live Preview, and the client's 3D viewport can't drift
        apart on what a frame means.
        """
        joint_values, ws_values = resolve_frame(
            self.anim, t,
            pose_lookup=self._pose_lookup,
            neutral=self._neutral,
            on_error=lambda track_id, e: self._log(
                "warn", f"value-track {self.anim.id}/{track_id} failed: {e}"),
        )
        if not joint_values and not ws_values:
            return

        # Fast path: batch the whole frame into ONE evaluator call so the
        # shared sheet is evaluated once and the UI snapshot broadcast
        # once per frame — not once per track.
        if self._apply_frame is not None:
            try:
                self._apply_frame(joint_values, ws_values)
            except Exception as e:
                self._log("error",
                          f"apply_animation_frame {self.anim.id} failed: {e}")
            return

        # Per-setpoint fallback for registries without the batch call.
        for joint, value in joint_values.items():
            try:
                self._set_urdf_joint_value(joint, value)
            except Exception as e:
                self._log("warn", f"set_urdf_joint_value {joint} failed: {e}")
        for (sheet_id, input_id), value in ws_values.items():
            try:
                self._set_ws_input(sheet_id, input_id, value)
            except Exception as e:
                self._log("warn",
                          f"set_ws_input {sheet_id}/{input_id} failed: {e}")

    def _fire_triggers(self, t_prev: float, t_now: float,
                       include_start: bool = False) -> None:
        if t_now < t_prev or (t_now == t_prev and not include_start):
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
            for kf in track.fires_in(t_prev, t_now, include_start):
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
            elif kf.target_kind in ("sound", "animation"):
                self._dispatch_board_item(kf)
            else:
                self._log("warn",
                          f"Unknown trigger target_kind: {kf.target_kind}")
        except Exception as e:
            self._log("error",
                      f"Trigger dispatch failed at t={kf.time}: {e}")

    def _dispatch_board_item(self, kf: TriggerKeyframe) -> None:
        """Fire a board item — a sound or another animation.

        Poses are deliberately not here: a pose is a weighted clip with
        blending and per-joint overrides, which a one-shot fire cannot
        express, so it stays a value track. The editor still presents all
        three under one "Board Item" affordance.

        A LOOPING item is stopped again at ``time + duration`` (the bar
        the operator dragged on the timeline). A one-shot ends on its
        own, so `duration` is display only and no stop is scheduled —
        otherwise dragging a non-looping bar would silently truncate the
        clip, which is a different feature.
        """
        if not kf.target:
            self._log("warn", f"{kf.target_kind} trigger at t={kf.time} "
                              f"has no item id")
            return
        item_id = str(kf.target[0])
        if self._board is None:
            self._log("warn",
                      f"{kf.target_kind} trigger at t={kf.time} but no board "
                      f"control wired; dropping")
            return

        if kf.target_kind == "sound":
            node_id = self._board.play_sound(item_id)
            if not node_id:
                return
            looping = self._board.sound_is_looping(item_id)
            self._started_sounds[node_id] = self._sound_deadline(
                item_id, kf, looping)
            if kf.duration > 0 and looping:
                self._pending_stops.append(
                    (kf.time + kf.duration, "sound", node_id))
            return

        # animation
        if item_id == self.anim.id:
            # Self-reference is the one cycle we can name precisely, so
            # say so rather than letting the depth limit swallow it.
            self._log("warn",
                      f"Animation '{self.anim.id}' triggers itself at "
                      f"t={kf.time}; skipped")
            return
        if self._depth + 1 >= MAX_ANIMATION_NESTING:
            self._log("warn",
                      f"Animation '{self.anim.id}' nests deeper than "
                      f"{MAX_ANIMATION_NESTING} at t={kf.time}; "
                      f"'{item_id}' not started")
            return
        # start_animation is async and we are on the tick path, so hand
        # it to the loop rather than blocking the frame.
        self._spawn(self._start_nested(item_id, kf))

    async def _start_nested(self, item_id: str, kf: TriggerKeyframe) -> None:
        if self._board is None:
            return
        try:
            started = await self._board.start_animation(item_id, self._depth + 1)
        except Exception as e:
            self._log("error", f"start_animation({item_id}) failed: {e}")
            return
        if started:
            self._started_animations.add(item_id)
        if started and kf.duration > 0 \
                and self._board.animation_is_looping(item_id):
            self._pending_stops.append(
                (kf.time + kf.duration, "animation", item_id))

    def _sound_deadline(self, sound_id: str, kf: TriggerKeyframe,
                        looping: bool) -> Optional[float]:
        """Wall-clock time a one-shot clip started now will end, or None
        when it loops or its length isn't known."""
        if looping:
            return None
        length = float(kf.clip_length or 0)
        if length <= 0:
            lookup = getattr(self._board, "sound_length", None)
            try:
                length = float(lookup(sound_id) or 0) if lookup else 0.0
            except Exception:
                length = 0.0
        if length <= 0:
            return None
        return time.monotonic() + length

    def _spawn(self, coro) -> None:
        """Run a coroutine off the tick path, never blocking a frame."""
        try:
            asyncio.get_event_loop().create_task(coro)
        except RuntimeError:
            # No running loop (unit test calling _fire_triggers directly).
            coro.close()

    def _fire_pending_stops(self, t_prev: float, t_now: float) -> None:
        """Stop looping board items whose dragged length has elapsed.

        Uses the same (t_prev, t_now] window the triggers do, so a pause
        can't drop a stop and a loop-around can't fire it twice.
        """
        if not self._pending_stops:
            return
        due = [p for p in self._pending_stops if t_prev < p[0] <= t_now]
        if not due:
            return
        self._pending_stops = [p for p in self._pending_stops if p not in due]
        for _at, kind, key in due:
            # Already stopped: don't stop it again when the animation ends.
            if kind == "sound":
                self._started_sounds.pop(key, None)
            else:
                self._started_animations.discard(key)
            try:
                if kind == "sound":
                    self._board.stop_sound(key)
                else:
                    self._spawn(self._board.stop_animation(key))
            except Exception as e:
                self._log("error", f"stopping {kind} '{key}' failed: {e}")

    def _stop_all_board_items(self) -> None:
        """Stop everything this animation started that is still running.

        Called when the animation itself stops, by Stop or by reaching its
        end. A clip that runs past the end of the timeline is cut there,
        and a nested looping animation outliving its parent is the kind of
        thing an operator discovers as a robot that will not stop moving.
        """
        pending, self._pending_stops = self._pending_stops, []
        sounds, self._started_sounds = self._started_sounds, {}
        animations, self._started_animations = self._started_animations, set()
        if self._board is None:
            return
        now = time.monotonic()
        to_stop = set()
        for _at, kind, key in pending:
            to_stop.add((kind, key))
        for node_id, ends_at in sounds.items():
            if ends_at is None or now < ends_at:
                to_stop.add(("sound", node_id))
        for animation_id in animations:
            to_stop.add(("animation", animation_id))
        for kind, key in sorted(to_stop):
            try:
                if kind == "sound":
                    self._board.stop_sound(key)
                else:
                    self._spawn(self._board.stop_animation(key))
            except Exception as e:
                self._log("error", f"stopping {kind} '{key}' failed: {e}")

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
        apply_frame: Optional[ApplyFrame] = None,
        pose_source: Optional[Callable[[], PoseLookup]] = None,
        neutral_source: Optional[Callable[[], Dict[str, float]]] = None,
        board: Optional[BoardControl] = None,
        logger=None,
    ):
        self._set_urdf_joint_value = set_urdf_joint_value
        self._set_ws_input = set_ws_input
        self._set_topic_channel = set_topic_channel
        self._apply_frame = apply_frame
        self._send_peripheral_command = send_peripheral_command
        self._estop_active = estop_active
        # Factories, not values: each playback gets a fresh pose lookup
        # (and so a fresh cache), which is what makes a pose edited
        # between runs take effect on the next start.
        self._pose_source = pose_source
        self._neutral_source = neutral_source
        # Board-item control. The registry is also what a nested
        # animation trigger comes back through, so the object it hands
        # players routes start_animation back here.
        self._board = board
        self.logger = logger
        self._players: Dict[str, AnimationPlayer] = {}

    async def start(self, anim: Animation, loop: Optional[bool] = None,
                    depth: int = 0) -> AnimationPlayer:
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
            apply_frame=self._apply_frame,
            estop_active=self._estop_active,
            on_finished=self._on_player_finished,
            board=self._board,
            depth=depth,
            pose_lookup=self._pose_source() if self._pose_source else None,
            neutral=self._neutral_source() if self._neutral_source else None,
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
