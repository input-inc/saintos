"""
SAINT.OS Unreal Engine Data Reader

Provides parsing capabilities for Unreal Engine asset files (.uasset),
with focus on animation sequences and data assets for robot motion playback.

Supported:
- .uasset file structure parsing
- Animation sequence extraction (curves, keyframes)
- Bone transform data
- Blueprint Data Assets (DA_* files) for UE5

Limitations:
- Tested primarily with UE 4.27 and UE 5.x formats
- Some compressed formats may not be fully supported
- Blueprint logic is not interpreted

For real-time animation streaming, consider using the WebSocket bridge
to stream data directly from Unreal at runtime instead.
"""

from saint_server.unreal.uasset_reader import UAssetReader
from saint_server.unreal.animation import (
    AnimationSequence,
    AnimationCurve,
    CurveKey,
    BoneTrack,
)
from saint_server.unreal.data_asset_reader import (
    DataAssetReader,
    DataAsset,
    read_data_asset,
)

__all__ = [
    'UAssetReader',
    'AnimationSequence',
    'AnimationCurve',
    'CurveKey',
    'BoneTrack',
    'DataAssetReader',
    'DataAsset',
    'read_data_asset',
]
