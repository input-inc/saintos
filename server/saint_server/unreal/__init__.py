"""Animation primitives shared with the Animation Builder.

Originally hosted Unreal Engine data-asset parsers (`uasset_reader`,
`data_asset_reader`) — those were removed when the Animation Builder
replaced the Moods view. The keyframe-curve types here are now consumed
by `saint_server.animation.models`; the package name is preserved to
keep the import paths stable.
"""

from saint_server.unreal.animation import (
    AnimationSequence,
    AnimationCurve,
    CurveKey,
    CurveInterpolation,
    BoneTrack,
)

__all__ = [
    'AnimationSequence',
    'AnimationCurve',
    'CurveKey',
    'CurveInterpolation',
    'BoneTrack',
]
