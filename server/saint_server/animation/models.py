"""Data classes for poses, animations, value tracks, and trigger tracks.

Wraps the existing curve infrastructure in
``saint_server/unreal/animation.py`` (CurveKey, AnimationCurve,
CurveInterpolation) so we don't reinvent keyframe interpolation —
that module's been waiting for a consumer since it was authored.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from saint_server.unreal.animation import (
    AnimationCurve,
    CurveInterpolation,
    CurveKey,
)


# ── value tracks ────────────────────────────────────────────────────


@dataclass
class ValueTrack:
    """One continuous-value channel on an animation timeline.

    The track's value at any time t comes from its underlying
    AnimationCurve. ``target_kind`` selects where the sampled value is
    pushed each tick — mirroring TriggerKeyframe's target model:

      * ``"urdf_joint"`` (default) — ``track.id`` is the URDF joint
        name; the value goes to ``set_urdf_joint_value(track.id, v)``.
        Backward-compatible with every track authored before sheet
        binding existed (those tracks have no ``target_kind`` and
        deserialize to this).
      * ``"ws_input"`` — ``target`` is ``[sheet_id, ws_input_id]``; the
        value goes to ``set_ws_input(...)`` — the same path poses and
        controller gamepad bindings use. This lets an animation drive a
        controller routing-sheet input directly, so animations can be
        authored with NO URDF at all.
    """
    id: str
    name: str
    curve: AnimationCurve
    target_kind: str = "urdf_joint"
    target: List[str] = field(default_factory=list)

    def value_at(self, t: float) -> float:
        return self.curve.get_value_at_time(t)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "target_kind": self.target_kind,
            "target": list(self.target),
            "curve": {
                "name": self.curve.name,
                "keys": [
                    {
                        "time": k.time,
                        "value": k.value,
                        "interp": int(k.interp),
                        "arrive_tangent": k.arrive_tangent,
                        "leave_tangent": k.leave_tangent,
                    }
                    for k in self.curve.keys
                ],
            },
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ValueTrack":
        curve_d = d.get("curve") or {}
        keys = [
            CurveKey(
                time=float(k.get("time", 0.0)),
                value=float(k.get("value", 0.0)),
                interp=CurveInterpolation(int(k.get("interp", 1))),
                arrive_tangent=float(k.get("arrive_tangent", 0.0)),
                leave_tangent=float(k.get("leave_tangent", 0.0)),
            )
            for k in curve_d.get("keys", [])
        ]
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            target_kind=str(d.get("target_kind", "urdf_joint")),
            target=[str(p) for p in (d.get("target") or [])],
            curve=AnimationCurve(name=str(curve_d.get("name", "")), keys=keys),
        )


# ── trigger tracks ──────────────────────────────────────────────────


@dataclass
class TriggerKeyframe:
    """A discrete event that fires once at ``time``.

    ``target_kind`` selects the dispatch path:
      * ``"ws_input"`` — target is [sheet_id, ws_input_id]; the
        animation player calls routing_evaluator.set_ws_input(...) with
        ``value`` cast to float.
      * ``"topic"`` — target is [endpoint_path, field]; the animation
        player calls ros_bridge.set_topic_channel(...) with ``value``
        cast to float.
      * ``"peripheral_command"`` — target is [node_id, peripheral_id];
        ``value`` is a dict ``{"command": str, "args": dict}`` (or just
        a string filename, which is desugared to
        ``{"command": "play_file", "args": {"filename": ...}}`` so
        operators authoring an audio cue can skip the wrapper).
        Dispatched through server_node.send_peripheral_command, which
        publishes the same JSON action as a websocket-issued
        peripheral_command — i.e. an animation-fired play_file is
        wire-indistinguishable from one a UI button sent.
    """
    time: float
    target_kind: str
    target: List[str]
    value: Any
    label: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "target_kind": self.target_kind,
            "target": list(self.target),
            "value": self.value,
            "label": self.label,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TriggerKeyframe":
        return cls(
            time=float(d.get("time", 0.0)),
            target_kind=str(d.get("target_kind", "ws_input")),
            target=[str(p) for p in (d.get("target") or [])],
            value=d.get("value"),
            label=str(d.get("label", "")),
        )


@dataclass
class TriggerTrack:
    id: str
    name: str
    keyframes: List[TriggerKeyframe] = field(default_factory=list)

    def fires_in(self, t_prev: float, t_now: float) -> List[TriggerKeyframe]:
        """Keyframes whose ``time`` is in (t_prev, t_now]."""
        return [
            k for k in self.keyframes
            if t_prev < k.time <= t_now
        ]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "keyframes": [k.to_dict() for k in self.keyframes],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TriggerTrack":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            keyframes=sorted(
                [TriggerKeyframe.from_dict(k) for k in d.get("keyframes", [])],
                key=lambda k: k.time,
            ),
        )


# ── animation container ─────────────────────────────────────────────


@dataclass
class Animation:
    id: str
    name: str
    duration: float = 0.0
    fps: int = 60
    loop: bool = False
    # Material icon name — surfaced in the sidebar so operators can
    # recognize "wave hello" vs "salute" at a glance. Defaults to the
    # generic "animation" icon when unset.
    icon: str = ""
    # Single-level group (operator-defined string). Empty means
    # ungrouped — appears under an "Ungrouped" section in the UI.
    group: str = ""
    value_tracks: List[ValueTrack] = field(default_factory=list)
    trigger_tracks: List[TriggerTrack] = field(default_factory=list)
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "duration": self.duration,
            "fps": self.fps,
            "loop": self.loop,
            "icon": self.icon,
            "group": self.group,
            "value_tracks": [t.to_dict() for t in self.value_tracks],
            "trigger_tracks": [t.to_dict() for t in self.trigger_tracks],
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Animation":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            duration=float(d.get("duration", 0.0)),
            fps=int(d.get("fps", 60)),
            loop=bool(d.get("loop", False)),
            icon=str(d.get("icon", "")),
            group=str(d.get("group", "")),
            value_tracks=[ValueTrack.from_dict(t) for t in d.get("value_tracks", [])],
            trigger_tracks=[TriggerTrack.from_dict(t) for t in d.get("trigger_tracks", [])],
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def stamp(self) -> None:
        """Update the modified timestamp; set created on first save."""
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now


# ── poses ───────────────────────────────────────────────────────────


@dataclass
class PoseSetpoint:
    """A single value to push into the routing graph when a pose applies.

    Setpoints address WS inputs by ``(sheet_id, ws_input_id)`` — the
    same convention the controller gamepad bindings use. Applying a
    pose is just a fan-out of ``routing_evaluator.set_ws_input`` calls.
    """
    sheet_id: str
    ws_input_id: str
    value: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sheet_id": self.sheet_id,
            "ws_input_id": self.ws_input_id,
            "value": self.value,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "PoseSetpoint":
        return cls(
            sheet_id=str(d["sheet_id"]),
            ws_input_id=str(d["ws_input_id"]),
            value=float(d.get("value", 0.0)),
        )


@dataclass
class Pose:
    id: str
    name: str
    icon: str = ""        # material-icon name; sidebar/management UI uses it
    group: str = ""       # single-level group ("" → Ungrouped bucket)
    setpoints: List[PoseSetpoint] = field(default_factory=list)
    description: str = ""
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "icon": self.icon,
            "group": self.group,
            "description": self.description,
            "setpoints": [s.to_dict() for s in self.setpoints],
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Pose":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            icon=str(d.get("icon", "")),
            group=str(d.get("group", "")),
            description=str(d.get("description", "")),
            setpoints=[PoseSetpoint.from_dict(s) for s in d.get("setpoints", [])],
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def stamp(self) -> None:
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now


@dataclass
class Sound:
    """A soundboard entry: an audio file that a specific node plays.

    Node-scoped — ``file_path`` is an absolute path on ``node_id``'s own
    storage (files are never shipped between nodes) and playback comes out
    of that node's ``output_device`` (blank → the node's default ALSA
    device). ``position`` gives the operator an explicit ordering within a
    group (poses/animations are alpha-sorted; sounds are drag-ordered).
    """
    id: str
    name: str
    icon: str = ""            # material-icon name; default "volume_up" in UI
    group: str = ""           # single-level group ("" → Ungrouped bucket)
    node_id: str = ""         # which node plays this sound
    file_path: str = ""       # absolute path on that node
    output_device: str = ""   # ALSA device id ("" → node default)
    volume: float = 1.0       # 0.0–1.0
    start_time: float = 0.0   # seek offset in seconds at play
    loop: bool = False
    loop_count: int = 0       # repeats when loop on; 0 → infinite
    position: int = 0         # explicit order within the list/group
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "icon": self.icon,
            "group": self.group,
            "node_id": self.node_id,
            "file_path": self.file_path,
            "output_device": self.output_device,
            "volume": self.volume,
            "start_time": self.start_time,
            "loop": self.loop,
            "loop_count": self.loop_count,
            "position": self.position,
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Sound":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            icon=str(d.get("icon", "")),
            group=str(d.get("group", "")),
            node_id=str(d.get("node_id", "")),
            file_path=str(d.get("file_path", "")),
            output_device=str(d.get("output_device", "")),
            volume=float(d.get("volume", 1.0)),
            start_time=float(d.get("start_time", 0.0)),
            loop=bool(d.get("loop", False)),
            loop_count=int(d.get("loop_count", 0)),
            position=int(d.get("position", 0)),
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def stamp(self) -> None:
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now
