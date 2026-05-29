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
    AnimationCurve. The (animation_id, track id) pair is the routing-
    graph source identity — operators wire that source onto sheets.
    """
    id: str
    name: str
    curve: AnimationCurve

    def value_at(self, t: float) -> float:
        return self.curve.get_value_at_time(t)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
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
            curve=AnimationCurve(name=str(curve_d.get("name", "")), keys=keys),
        )


# ── trigger tracks ──────────────────────────────────────────────────


@dataclass
class TriggerKeyframe:
    """A discrete event that fires once at ``time``.

    ``target_kind`` selects the dispatch path:
      * ``"ws_input"`` — target is [sheet_id, ws_input_id]; the
        animation player calls routing_evaluator.set_ws_input(...).
      * ``"topic"``    — target is [endpoint_path, field]; the
        animation player calls ros_bridge.set_topic_channel(...).
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
