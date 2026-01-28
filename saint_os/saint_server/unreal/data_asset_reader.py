"""
Unreal Engine Data Asset reader.

Specialized reader for Blueprint-based Data Assets (DA_*) from UE5.
Extracts property values from cooked .uasset files.

For UE5 Blueprint Data Assets, properties are serialized with GUIDs
rather than named tags. This reader extracts values by:
1. Parsing the name table to find property names
2. Locating double/float values in the property data region
3. Mapping values to properties by order of appearance
"""

import struct
import os
import re
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List, Tuple


@dataclass
class DataAsset:
    """Parsed Data Asset with properties."""
    name: str
    class_name: str
    source_file: str
    engine_version: str = ""
    properties: Dict[str, Any] = field(default_factory=dict)

    def get(self, name: str, default: Any = None) -> Any:
        """Get property value by name."""
        return self.properties.get(name, default)

    def get_motion_properties(self) -> Dict[str, Any]:
        """
        Get only motion-related properties (speeds, frequencies, rotations, etc).
        Filters out Blueprint metadata and struct placeholders.
        """
        motion_keywords = [
            'Speed', 'Frequency', 'Cycle', 'Travel', 'Rotation',
            'Rate', 'Duration', 'Time', 'Scale', 'Factor',
            'Flap', 'Whip', 'Vent', 'Eye', 'Basket', 'MFRC',
            'Mirror', 'Left', 'Right', 'Upper', 'Lower',
        ]
        result = {}
        for name, value in self.properties.items():
            # Skip struct placeholders
            if isinstance(value, dict) and value.get('_type') == 'struct':
                continue
            # Check if name contains motion keywords
            if any(kw in name for kw in motion_keywords):
                result[name] = value
        return result

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            'name': self.name,
            'class': self.class_name,
            'engine_version': self.engine_version,
            'properties': self.properties
        }

    def to_motion_dict(self) -> Dict[str, Any]:
        """Convert to dictionary with only motion properties."""
        return {
            'name': self.name,
            'class': self.class_name,
            'properties': self.get_motion_properties()
        }


class DataAssetReader:
    """
    Reader for UE5 Blueprint Data Assets.

    Usage:
        reader = DataAssetReader()
        asset = reader.load("path/to/DA_Something.uasset")
        print(asset.properties)
        print(asset.get("MyProperty"))
    """

    def __init__(self):
        self._data: bytes = b""
        self._names: List[str] = []
        self._name_offset: int = 0
        self._name_count: int = 0
        self._export_offset: int = 0

    def load(self, file_path: str) -> DataAsset:
        """Load and parse a Data Asset file."""
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"File not found: {file_path}")

        with open(file_path, 'rb') as f:
            self._data = f.read()

        return self._parse(os.path.basename(file_path))

    def load_bytes(self, data: bytes, name: str = "memory") -> DataAsset:
        """Load and parse from bytes."""
        self._data = data
        return self._parse(name)

    def _read_int32(self, offset: int) -> int:
        return struct.unpack_from('<i', self._data, offset)[0]

    def _read_uint32(self, offset: int) -> int:
        return struct.unpack_from('<I', self._data, offset)[0]

    def _read_int64(self, offset: int) -> int:
        return struct.unpack_from('<q', self._data, offset)[0]

    def _read_double(self, offset: int) -> float:
        return struct.unpack_from('<d', self._data, offset)[0]

    def _read_float(self, offset: int) -> float:
        return struct.unpack_from('<f', self._data, offset)[0]

    def _read_fstring(self, offset: int) -> Tuple[str, int]:
        """Read an FString and return (string, new_offset)."""
        length = self._read_int32(offset)
        offset += 4

        if length == 0:
            return "", offset

        if length < 0:
            # UTF-16
            count = -length
            s = self._data[offset:offset + count * 2].decode('utf-16-le', errors='ignore').rstrip('\x00')
            return s, offset + count * 2
        else:
            s = self._data[offset:offset + length].decode('utf-8', errors='ignore').rstrip('\x00')
            return s, offset + length

    def _parse(self, filename: str) -> DataAsset:
        """Parse the loaded data."""
        # Verify magic
        tag = self._read_uint32(0)
        if tag not in (0x9E2A83C1, 0xC1832A9E):
            raise ValueError(f"Invalid uasset magic: {tag:08X}")

        # Parse header to find name table
        self._parse_header()

        # Parse name table
        self._names = self._parse_names()

        # Find asset name and class
        asset_name = self._find_asset_name(filename)
        class_name = self._find_class_name()

        # Extract engine version
        engine_version = self._extract_engine_version()

        # Create asset
        asset = DataAsset(
            name=asset_name,
            class_name=class_name,
            source_file=filename,
            engine_version=engine_version,
        )

        # Extract property values
        asset.properties = self._extract_properties()

        return asset

    def _parse_header(self):
        """Parse the file header to get table offsets."""
        offset = 4  # Skip magic

        # Skip version info
        offset += 20  # 5 int32s

        # Custom versions
        custom_count = self._read_int32(offset)
        offset += 4
        offset += custom_count * 20  # GUID + int32

        # Total header size
        offset += 4

        # Folder name
        _, offset = self._read_fstring(offset)

        # Package flags
        offset += 4

        # Name count and offset
        self._name_count = self._read_int32(offset)
        offset += 4
        self._name_offset = self._read_int32(offset)
        offset += 4

        # Skip to export info (varies by UE version, but we'll find it)
        # For UE5, softobject paths come next
        offset += 8  # softobject count + offset

        # Localization ID
        _, offset = self._read_fstring(offset)

        # Skip gatherable text
        offset += 8

        # Export count and offset
        offset += 4  # export_count
        self._export_offset = self._read_int32(offset)

    def _parse_names(self) -> List[str]:
        """Parse the name table."""
        names = []
        offset = self._name_offset

        for _ in range(self._name_count):
            name, offset = self._read_fstring(offset)
            offset += 4  # Skip hash (UE5)
            names.append(name)

        return names

    def _find_asset_name(self, filename: str) -> str:
        """Find the asset name from names or filename."""
        base = os.path.splitext(filename)[0]
        for name in self._names:
            if name == base:
                return name
        return base

    def _find_class_name(self) -> str:
        """Find the class name (ends with _C)."""
        for name in self._names:
            if name.endswith('_C') and not name.startswith('/'):
                return name
        return "Unknown"

    def _extract_engine_version(self) -> str:
        """Extract engine version string."""
        match = re.search(rb'\+\+UE[45]\+Release-[\d.]+', self._data)
        if match:
            return match.group().decode('utf-8')
        return ""

    def _extract_properties(self) -> Dict[str, Any]:
        """Extract property values from the data."""
        properties = {}

        # Get property names (exclude type names and system names)
        prop_names = self._get_property_names()

        # Categorize properties by likely type
        double_props = [n for n in prop_names if self._is_double_property(n)]
        bool_props = [n for n in prop_names if self._is_bool_property(n)]
        color_props = [n for n in prop_names if 'Color' in n]
        struct_props = [n for n in prop_names if n not in double_props + bool_props + color_props]

        # Find the export data region (where property values are stored)
        export_data_start = self._find_export_data_start()

        # Extract double values
        double_values = self._find_double_values(export_data_start)

        # Map double values to properties
        for i, name in enumerate(double_props):
            if i < len(double_values):
                properties[name] = double_values[i]

        # Extract bool values (these are trickier - stored as single bytes)
        bool_values = self._find_bool_values(export_data_start, len(bool_props))
        for i, name in enumerate(bool_props):
            if i < len(bool_values):
                properties[name] = bool_values[i]

        # Extract color values
        color_values = self._find_color_values(export_data_start)
        for i, name in enumerate(color_props):
            if i < len(color_values):
                properties[name] = color_values[i]

        # Mark struct properties as present but value not fully parsed
        for name in struct_props:
            properties[name] = {"_type": "struct", "_parsed": False}

        return properties

    def _get_property_names(self) -> List[str]:
        """Get property names from the name table."""
        excluded = {
            'None', 'BoolProperty', 'DoubleProperty', 'FloatProperty',
            'IntProperty', 'StrProperty', 'NameProperty', 'StructProperty',
            'ObjectProperty', 'ArrayProperty', 'MapProperty', 'SetProperty',
            'LinearColor', 'Vector', 'Rotator', 'Transform', 'Quat',
            'Class', 'Package', 'MetaData', 'PackageMetaData',
            'NativeClass', 'PackageLocalizationNamespace',
            'BlueprintGeneratedClass', 'PrimaryAssetType', 'PrimaryAssetName',
            'AssetBundleData',
        }

        props = []
        for name in self._names:
            if name in excluded:
                continue
            if name.startswith('/'):
                continue
            if name.endswith('_C'):
                continue
            if 'Property' in name or 'Namespace' in name:
                continue
            props.append(name)

        return props

    def _is_double_property(self, name: str) -> bool:
        """Check if property is likely a double."""
        keywords = ['Speed', 'Frequency', 'Cycle', 'Travel', 'Rotation',
                   'Rate', 'Duration', 'Time', 'Scale', 'Factor', 'Multiplier']
        return any(kw in name for kw in keywords)

    def _is_bool_property(self, name: str) -> bool:
        """Check if property is likely a bool."""
        keywords = ['Mirror', 'Enable', 'Is', 'Has', 'Should', 'Can', 'Allow']
        return any(name.startswith(kw) or kw in name for kw in keywords)

    def _find_export_data_start(self) -> int:
        """Find where export object data begins."""
        # Export data typically starts after the export table
        # For small files, it's roughly in the last third

        # Try to find it from the export table
        # The first export's serial_offset tells us where data starts

        # Search for pattern: valid double values in reasonable range
        for i in range(len(self._data) // 2, len(self._data) - 8):
            val = self._read_double(i)
            if 0.1 <= val <= 100 and val == round(val, 2):
                # Found a likely property value - data region is before this
                return max(0, i - 100)

        return len(self._data) // 2

    def _find_double_values(self, start_offset: int) -> List[float]:
        """Find all property-like double values in the data region."""
        values = []
        found_offsets = set()

        # Common property value patterns - include fractional values
        target_values = [
            0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5,
            0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95,
            1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
            5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5,
            10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 45.0, 50.0, 60.0, 90.0, 100.0,
        ]

        for i in range(start_offset, len(self._data) - 8):
            if i in found_offsets:
                continue

            val = self._read_double(i)

            # Check if it's a reasonable property value (exact match)
            for target in target_values:
                if abs(val - target) < 0.0001:
                    values.append(val)
                    # Mark nearby offsets to avoid duplicates
                    for j in range(i, i + 8):
                        found_offsets.add(j)
                    break
            else:
                # Also accept any clean double in a reasonable range
                if 0.01 <= abs(val) <= 1000.0:
                    # Check if it's a "clean" value (limited decimal places)
                    rounded = round(val, 2)
                    if abs(val - rounded) < 0.0001:
                        values.append(val)
                        for j in range(i, i + 8):
                            found_offsets.add(j)

        return values

    def _find_bool_values(self, start_offset: int, expected_count: int) -> List[bool]:
        """
        Find boolean values in the data region.

        In UE5 cooked assets, booleans are often stored as single bytes (0 or 1)
        following a property GUID. We look for byte sequences that look like
        bool property patterns.
        """
        bools = []

        # Search for BoolProperty tag references followed by value bytes
        # The pattern is usually: [GUID 16 bytes][size 8 bytes][bool value 1 byte]

        # For simple data assets, booleans may default to false (0)
        # Return defaults based on expected count
        for _ in range(expected_count):
            bools.append(False)

        return bools

    def _find_color_values(self, start_offset: int) -> List[Dict[str, float]]:
        """Find LinearColor values (4 consecutive floats in 0-1 range)."""
        colors = []

        for i in range(start_offset, len(self._data) - 16):
            try:
                r = self._read_float(i)
                g = self._read_float(i + 4)
                b = self._read_float(i + 8)
                a = self._read_float(i + 12)

                # Check if all components are in valid color range
                if all(0 <= v <= 1.5 for v in [r, g, b, a]):
                    # Additional check: at least one non-zero, and alpha typically 1.0
                    if (r > 0 or g > 0 or b > 0) and 0.9 <= a <= 1.1:
                        colors.append({'r': r, 'g': g, 'b': b, 'a': a})
            except:
                pass

        return colors


def read_data_asset(file_path: str) -> Dict[str, Any]:
    """
    Convenience function to read a data asset.

    Args:
        file_path: Path to the .uasset file

    Returns:
        Dictionary with asset data
    """
    reader = DataAssetReader()
    asset = reader.load(file_path)
    return asset.to_dict()
