"""
Unreal Engine .uasset file reader.

Parses Unreal Engine 4/5 asset files to extract:
- Package metadata
- Name tables
- Export/Import tables
- Property data (Data Assets, Data Tables, etc.)

Tested with UE 4.27 and UE 5.x formats.
"""

import struct
import os
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, BinaryIO, Tuple
from enum import IntEnum


# UAsset magic number
PACKAGE_FILE_TAG = 0x9E2A83C1
PACKAGE_FILE_TAG_SWAPPED = 0xC1832A9E


class EObjectFlags(IntEnum):
    """Unreal object flags."""
    RF_NoFlags = 0x00000000
    RF_Public = 0x00000001
    RF_Standalone = 0x00000002
    RF_Transactional = 0x00000008
    RF_ClassDefaultObject = 0x00000010
    RF_ArchetypeObject = 0x00000020
    RF_Transient = 0x00000040


@dataclass
class FGuid:
    """Unreal GUID."""
    a: int = 0
    b: int = 0
    c: int = 0
    d: int = 0

    def __str__(self):
        return f"{self.a:08X}{self.b:08X}{self.c:08X}{self.d:08X}"


@dataclass
class FGenerationInfo:
    """Package generation info."""
    export_count: int = 0
    name_count: int = 0


@dataclass
class FPackageFileSummary:
    """Package file header/summary."""
    tag: int = 0
    legacy_file_version: int = 0
    legacy_ue3_version: int = 0
    file_version_ue4: int = 0
    file_version_ue5: int = 0
    file_version_licensee_ue4: int = 0
    custom_versions: List[Tuple[FGuid, int]] = field(default_factory=list)
    total_header_size: int = 0
    folder_name: str = ""
    package_flags: int = 0
    name_count: int = 0
    name_offset: int = 0
    softobject_paths_count: int = 0
    softobject_paths_offset: int = 0
    localization_id: str = ""
    gatherable_text_data_count: int = 0
    gatherable_text_data_offset: int = 0
    export_count: int = 0
    export_offset: int = 0
    import_count: int = 0
    import_offset: int = 0
    depends_offset: int = 0
    string_asset_references_count: int = 0
    string_asset_references_offset: int = 0
    searchable_names_offset: int = 0
    thumbnail_table_offset: int = 0
    guid: FGuid = field(default_factory=FGuid)
    generations: List[FGenerationInfo] = field(default_factory=list)
    saved_by_engine_version: str = ""
    compatible_with_engine_version: str = ""
    compression_flags: int = 0
    package_source: int = 0
    asset_registry_data_offset: int = 0
    bulk_data_start_offset: int = 0
    world_tile_info_data_offset: int = 0
    chunk_ids: List[int] = field(default_factory=list)
    preload_dependency_count: int = 0
    preload_dependency_offset: int = 0
    names_referenced_from_export_data_count: int = 0
    payload_toc_offset: int = 0
    data_resource_offset: int = 0


@dataclass
class FNameEntry:
    """Name table entry."""
    name: str = ""
    non_case_preserving_hash: int = 0
    case_preserving_hash: int = 0


@dataclass
class FObjectImport:
    """Import table entry."""
    class_package: int = 0  # Index into name table
    class_name: int = 0  # Index into name table
    outer_index: int = 0
    object_name: int = 0  # Index into name table

    # Resolved names (filled after parsing)
    class_package_name: str = ""
    class_name_str: str = ""
    object_name_str: str = ""


@dataclass
class FObjectExport:
    """Export table entry."""
    class_index: int = 0
    super_index: int = 0
    template_index: int = 0
    outer_index: int = 0
    object_name: int = 0  # Index into name table
    object_flags: int = 0
    serial_size: int = 0
    serial_offset: int = 0
    forced_export: bool = False
    not_for_client: bool = False
    not_for_server: bool = False
    package_guid: FGuid = field(default_factory=FGuid)
    package_flags: int = 0
    not_always_loaded_for_editor_game: bool = False
    is_asset: bool = False
    generate_public_hash: bool = False
    first_export_dependency: int = 0
    serialization_before_serialization_dependencies: int = 0
    create_before_serialization_dependencies: int = 0
    serialization_before_create_dependencies: int = 0
    create_before_create_dependencies: int = 0

    # Resolved name (filled after parsing)
    object_name_str: str = ""


@dataclass
class UProperty:
    """Parsed property value."""
    name: str
    type_name: str
    value: Any
    array_index: int = 0
    struct_name: str = ""  # For StructProperty
    enum_name: str = ""  # For EnumProperty/ByteProperty


@dataclass
class UObject:
    """Parsed Unreal object with properties."""
    name: str
    class_name: str
    properties: Dict[str, UProperty] = field(default_factory=dict)
    export_index: int = 0


class UAssetReader:
    """
    Reader for Unreal Engine .uasset files.

    Usage:
        reader = UAssetReader()
        reader.load("path/to/asset.uasset")
        print(reader.summary.saved_by_engine_version)
        for obj in reader.objects:
            print(obj.name, obj.properties)
    """

    def __init__(self):
        self.file_path: str = ""
        self.summary: FPackageFileSummary = FPackageFileSummary()
        self.names: List[str] = []
        self.imports: List[FObjectImport] = []
        self.exports: List[FObjectExport] = []
        self.objects: List[UObject] = []
        self._data: bytes = b""
        self._is_little_endian: bool = True

    def load(self, file_path: str) -> bool:
        """
        Load and parse a .uasset file.

        Args:
            file_path: Path to the .uasset file

        Returns:
            True if successful, False otherwise
        """
        self.file_path = file_path

        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Asset file not found: {file_path}")

        with open(file_path, 'rb') as f:
            self._data = f.read()

        self._parse()
        return True

    def load_bytes(self, data: bytes, name: str = "memory") -> bool:
        """
        Load and parse from bytes.

        Args:
            data: Raw .uasset data
            name: Name for error messages

        Returns:
            True if successful
        """
        self.file_path = name
        self._data = data
        self._parse()
        return True

    def _parse(self):
        """Parse the loaded data."""
        self._parse_summary()
        self._parse_names()
        self._parse_imports()
        self._parse_exports()
        self._parse_export_data()

    def _read_int32(self, offset: int) -> Tuple[int, int]:
        """Read a signed 32-bit integer."""
        fmt = '<i' if self._is_little_endian else '>i'
        val = struct.unpack_from(fmt, self._data, offset)[0]
        return val, offset + 4

    def _read_uint32(self, offset: int) -> Tuple[int, int]:
        """Read an unsigned 32-bit integer."""
        fmt = '<I' if self._is_little_endian else '>I'
        val = struct.unpack_from(fmt, self._data, offset)[0]
        return val, offset + 4

    def _read_int64(self, offset: int) -> Tuple[int, int]:
        """Read a signed 64-bit integer."""
        fmt = '<q' if self._is_little_endian else '>q'
        val = struct.unpack_from(fmt, self._data, offset)[0]
        return val, offset + 8

    def _read_uint64(self, offset: int) -> Tuple[int, int]:
        """Read an unsigned 64-bit integer."""
        fmt = '<Q' if self._is_little_endian else '>Q'
        val = struct.unpack_from(fmt, self._data, offset)[0]
        return val, offset + 8

    def _read_float(self, offset: int) -> Tuple[float, int]:
        """Read a 32-bit float."""
        fmt = '<f' if self._is_little_endian else '>f'
        val = struct.unpack_from(fmt, self._data, offset)[0]
        return val, offset + 4

    def _read_double(self, offset: int) -> Tuple[float, int]:
        """Read a 64-bit double."""
        fmt = '<d' if self._is_little_endian else '>d'
        val = struct.unpack_from(fmt, self._data, offset)[0]
        return val, offset + 8

    def _read_bool(self, offset: int) -> Tuple[bool, int]:
        """Read a boolean (1 byte)."""
        val = self._data[offset] != 0
        return val, offset + 1

    def _read_guid(self, offset: int) -> Tuple[FGuid, int]:
        """Read a GUID."""
        a, offset = self._read_uint32(offset)
        b, offset = self._read_uint32(offset)
        c, offset = self._read_uint32(offset)
        d, offset = self._read_uint32(offset)
        return FGuid(a, b, c, d), offset

    def _read_fstring(self, offset: int) -> Tuple[str, int]:
        """Read an FString (length-prefixed string)."""
        length, offset = self._read_int32(offset)

        if length == 0:
            return "", offset

        # Negative length means UTF-16
        if length < 0:
            char_count = -length
            data = self._data[offset:offset + char_count * 2]
            try:
                s = data.decode('utf-16-le').rstrip('\x00')
            except:
                s = ""
            return s, offset + char_count * 2
        else:
            data = self._data[offset:offset + length]
            try:
                s = data.decode('utf-8').rstrip('\x00')
            except:
                s = data.decode('latin-1').rstrip('\x00')
            return s, offset + length

    def _read_fname(self, offset: int) -> Tuple[str, int]:
        """Read an FName (index into name table + number)."""
        index, offset = self._read_int32(offset)
        number, offset = self._read_int32(offset)

        if 0 <= index < len(self.names):
            name = self.names[index]
            if number > 0:
                name = f"{name}_{number - 1}"
            return name, offset
        return f"__INVALID_{index}__", offset

    def _parse_summary(self):
        """Parse the package file summary/header."""
        offset = 0
        s = self.summary

        # Check magic/tag
        tag, offset = self._read_uint32(offset)
        if tag == PACKAGE_FILE_TAG_SWAPPED:
            self._is_little_endian = False
            tag = PACKAGE_FILE_TAG
        s.tag = tag

        if tag != PACKAGE_FILE_TAG:
            raise ValueError(f"Invalid uasset magic: {tag:08X}")

        # Legacy versions
        s.legacy_file_version, offset = self._read_int32(offset)
        s.legacy_ue3_version, offset = self._read_int32(offset)
        s.file_version_ue4, offset = self._read_int32(offset)
        s.file_version_ue5, offset = self._read_int32(offset)
        s.file_version_licensee_ue4, offset = self._read_int32(offset)

        # Custom versions
        custom_version_count, offset = self._read_int32(offset)
        for _ in range(custom_version_count):
            guid, offset = self._read_guid(offset)
            version, offset = self._read_int32(offset)
            s.custom_versions.append((guid, version))

        s.total_header_size, offset = self._read_int32(offset)
        s.folder_name, offset = self._read_fstring(offset)
        s.package_flags, offset = self._read_uint32(offset)
        s.name_count, offset = self._read_int32(offset)
        s.name_offset, offset = self._read_int32(offset)

        # UE5 specific
        if s.file_version_ue5 >= 0:
            s.softobject_paths_count, offset = self._read_int32(offset)
            s.softobject_paths_offset, offset = self._read_int32(offset)

        s.localization_id, offset = self._read_fstring(offset)

        # Gatherable text
        if s.file_version_ue4 >= 516:
            s.gatherable_text_data_count, offset = self._read_int32(offset)
            s.gatherable_text_data_offset, offset = self._read_int32(offset)

        s.export_count, offset = self._read_int32(offset)
        s.export_offset, offset = self._read_int32(offset)
        s.import_count, offset = self._read_int32(offset)
        s.import_offset, offset = self._read_int32(offset)
        s.depends_offset, offset = self._read_int32(offset)

        # String asset references
        if s.file_version_ue4 >= 384:
            s.string_asset_references_count, offset = self._read_int32(offset)
            s.string_asset_references_offset, offset = self._read_int32(offset)

        if s.file_version_ue4 >= 510:
            s.searchable_names_offset, offset = self._read_int32(offset)

        s.thumbnail_table_offset, offset = self._read_int32(offset)
        s.guid, offset = self._read_guid(offset)

        # Generations
        generation_count, offset = self._read_int32(offset)
        for _ in range(generation_count):
            gen = FGenerationInfo()
            gen.export_count, offset = self._read_int32(offset)
            gen.name_count, offset = self._read_int32(offset)
            s.generations.append(gen)

        # Engine version
        s.saved_by_engine_version, offset = self._read_fstring(offset)
        s.compatible_with_engine_version, offset = self._read_fstring(offset)

        s.compression_flags, offset = self._read_uint32(offset)

        # Compressed chunks (not used in modern UE)
        compressed_chunk_count, offset = self._read_int32(offset)
        offset += compressed_chunk_count * 16  # Skip FCompressedChunk entries

        s.package_source, offset = self._read_uint32(offset)

        # Additional package names to cook (skip)
        additional_count, offset = self._read_int32(offset)
        for _ in range(additional_count):
            _, offset = self._read_fstring(offset)

        s.asset_registry_data_offset, offset = self._read_int32(offset)
        s.bulk_data_start_offset, offset = self._read_int64(offset)
        s.world_tile_info_data_offset, offset = self._read_int32(offset)

        # Chunk IDs
        chunk_id_count, offset = self._read_int32(offset)
        for _ in range(chunk_id_count):
            chunk_id, offset = self._read_int32(offset)
            s.chunk_ids.append(chunk_id)

        if s.file_version_ue4 >= 516:
            s.preload_dependency_count, offset = self._read_int32(offset)
            s.preload_dependency_offset, offset = self._read_int32(offset)

        if s.file_version_ue5 >= 0:
            s.names_referenced_from_export_data_count, offset = self._read_int32(offset)
            s.payload_toc_offset, offset = self._read_int64(offset)
            if s.file_version_ue5 >= 1:
                s.data_resource_offset, offset = self._read_int32(offset)

    def _parse_names(self):
        """Parse the name table."""
        offset = self.summary.name_offset
        self.names = []

        for _ in range(self.summary.name_count):
            name, offset = self._read_fstring(offset)
            # Skip hashes
            if self.summary.file_version_ue4 >= 516:
                offset += 4  # case preserving hash
            self.names.append(name)

    def _parse_imports(self):
        """Parse the import table."""
        offset = self.summary.import_offset
        self.imports = []

        for _ in range(self.summary.import_count):
            imp = FObjectImport()
            imp.class_package, offset = self._read_int32(offset)
            offset += 4  # Skip number part of FName
            imp.class_name, offset = self._read_int32(offset)
            offset += 4
            imp.outer_index, offset = self._read_int32(offset)
            imp.object_name, offset = self._read_int32(offset)
            offset += 4

            # UE5
            if self.summary.file_version_ue5 >= 0:
                offset += 4  # bImportOptional

            # Resolve names
            if 0 <= imp.class_package < len(self.names):
                imp.class_package_name = self.names[imp.class_package]
            if 0 <= imp.class_name < len(self.names):
                imp.class_name_str = self.names[imp.class_name]
            if 0 <= imp.object_name < len(self.names):
                imp.object_name_str = self.names[imp.object_name]

            self.imports.append(imp)

    def _parse_exports(self):
        """Parse the export table."""
        offset = self.summary.export_offset
        self.exports = []

        for _ in range(self.summary.export_count):
            exp = FObjectExport()
            exp.class_index, offset = self._read_int32(offset)
            exp.super_index, offset = self._read_int32(offset)

            if self.summary.file_version_ue4 >= 508:
                exp.template_index, offset = self._read_int32(offset)

            exp.outer_index, offset = self._read_int32(offset)
            exp.object_name, offset = self._read_int32(offset)
            offset += 4  # FName number

            exp.object_flags, offset = self._read_uint32(offset)
            exp.serial_size, offset = self._read_int64(offset)
            exp.serial_offset, offset = self._read_int64(offset)

            exp.forced_export, offset = self._read_bool(offset)
            exp.not_for_client, offset = self._read_bool(offset)
            exp.not_for_server, offset = self._read_bool(offset)

            exp.package_guid, offset = self._read_guid(offset)

            if self.summary.file_version_ue5 < 0:
                exp.package_flags, offset = self._read_uint32(offset)

            exp.not_always_loaded_for_editor_game, offset = self._read_bool(offset)
            exp.is_asset, offset = self._read_bool(offset)

            if self.summary.file_version_ue5 >= 0:
                exp.generate_public_hash, offset = self._read_bool(offset)

            if self.summary.file_version_ue4 >= 365:
                exp.first_export_dependency, offset = self._read_int32(offset)
                exp.serialization_before_serialization_dependencies, offset = self._read_int32(offset)
                exp.create_before_serialization_dependencies, offset = self._read_int32(offset)
                exp.serialization_before_create_dependencies, offset = self._read_int32(offset)
                exp.create_before_create_dependencies, offset = self._read_int32(offset)

            # Resolve name
            if 0 <= exp.object_name < len(self.names):
                exp.object_name_str = self.names[exp.object_name]

            self.exports.append(exp)

    def _parse_export_data(self):
        """Parse the actual object data from exports."""
        self.objects = []

        for i, exp in enumerate(self.exports):
            if exp.serial_size <= 0:
                continue

            obj = UObject(
                name=exp.object_name_str,
                class_name=self._get_class_name(exp.class_index),
                export_index=i
            )

            # Parse properties
            offset = exp.serial_offset
            end_offset = exp.serial_offset + exp.serial_size

            try:
                obj.properties = self._parse_properties(offset, end_offset)
            except Exception as e:
                # Property parsing can fail for complex types
                pass

            self.objects.append(obj)

    def _get_class_name(self, class_index: int) -> str:
        """Get class name from class index (negative = import, positive = export)."""
        if class_index < 0:
            imp_index = -class_index - 1
            if 0 <= imp_index < len(self.imports):
                return self.imports[imp_index].object_name_str
        elif class_index > 0:
            exp_index = class_index - 1
            if 0 <= exp_index < len(self.exports):
                return self.exports[exp_index].object_name_str
        return "Unknown"

    def _parse_properties(self, offset: int, end_offset: int) -> Dict[str, UProperty]:
        """Parse property list at given offset."""
        properties = {}

        while offset < end_offset:
            # Read property name
            prop_name, offset = self._read_fname(offset)

            if prop_name == "None" or prop_name == "":
                break

            # Read property type
            prop_type, offset = self._read_fname(offset)

            # Read size and array index
            size, offset = self._read_int32(offset)
            array_index, offset = self._read_int32(offset)

            # Parse based on type
            prop = UProperty(
                name=prop_name,
                type_name=prop_type,
                value=None,
                array_index=array_index
            )

            value_start = offset

            try:
                if prop_type == "BoolProperty":
                    val, offset = self._read_bool(offset)
                    prop.value = val
                    offset = value_start + size  # Ensure we advance correctly

                elif prop_type == "IntProperty":
                    offset += 1  # Tag
                    val, offset = self._read_int32(offset)
                    prop.value = val

                elif prop_type == "FloatProperty":
                    offset += 1  # Tag
                    val, offset = self._read_float(offset)
                    prop.value = val

                elif prop_type == "DoubleProperty":
                    offset += 1  # Tag
                    val, offset = self._read_double(offset)
                    prop.value = val

                elif prop_type == "StrProperty":
                    offset += 1  # Tag
                    val, offset = self._read_fstring(offset)
                    prop.value = val

                elif prop_type == "NameProperty":
                    offset += 1  # Tag
                    val, offset = self._read_fname(offset)
                    prop.value = val

                elif prop_type == "StructProperty":
                    struct_name, offset = self._read_fname(offset)
                    prop.struct_name = struct_name
                    offset += 17  # GUID + tag
                    # Read struct data as bytes for now
                    prop.value = self._parse_struct(struct_name, offset, size)
                    offset = value_start + size + 24  # 24 = 8 (struct name) + 16 (guid) + tag

                elif prop_type == "ArrayProperty":
                    inner_type, offset = self._read_fname(offset)
                    offset += 1  # Tag
                    count, offset = self._read_int32(offset)
                    prop.value = {"inner_type": inner_type, "count": count}
                    offset = value_start + size + 9  # Skip rest

                elif prop_type == "ObjectProperty":
                    offset += 1  # Tag
                    obj_index, offset = self._read_int32(offset)
                    prop.value = self._get_class_name(obj_index)

                else:
                    # Unknown type - skip
                    offset = value_start + size

            except Exception:
                offset = value_start + size

            properties[prop_name] = prop

        return properties

    def _parse_struct(self, struct_name: str, offset: int, size: int) -> Any:
        """Parse a struct property value."""
        if struct_name == "LinearColor":
            r, offset = self._read_float(offset)
            g, offset = self._read_float(offset)
            b, offset = self._read_float(offset)
            a, offset = self._read_float(offset)
            return {"r": r, "g": g, "b": b, "a": a}

        elif struct_name == "Vector":
            x, offset = self._read_double(offset)
            y, offset = self._read_double(offset)
            z, offset = self._read_double(offset)
            return {"x": x, "y": y, "z": z}

        elif struct_name == "Rotator":
            pitch, offset = self._read_double(offset)
            yaw, offset = self._read_double(offset)
            roll, offset = self._read_double(offset)
            return {"pitch": pitch, "yaw": yaw, "roll": roll}

        elif struct_name == "Quat":
            x, offset = self._read_double(offset)
            y, offset = self._read_double(offset)
            z, offset = self._read_double(offset)
            w, offset = self._read_double(offset)
            return {"x": x, "y": y, "z": z, "w": w}

        elif struct_name == "Transform":
            # Rotation (quat), Translation (vector), Scale (vector)
            return {"raw_size": size}

        elif struct_name == "Guid":
            guid, _ = self._read_guid(offset)
            return str(guid)

        else:
            # Try to parse as nested properties
            try:
                return self._parse_properties(offset, offset + size)
            except:
                return {"raw_size": size}

    def get_property(self, object_name: str, property_name: str) -> Optional[Any]:
        """
        Get a property value by object and property name.

        Args:
            object_name: Name of the object (or None for first object)
            property_name: Name of the property

        Returns:
            Property value or None
        """
        for obj in self.objects:
            if object_name is None or obj.name == object_name:
                if property_name in obj.properties:
                    return obj.properties[property_name].value
        return None

    def get_all_properties(self, object_name: str = None) -> Dict[str, Any]:
        """
        Get all properties as a simple dict.

        Args:
            object_name: Name of the object (or None for first object)

        Returns:
            Dict of property name -> value
        """
        for obj in self.objects:
            if object_name is None or obj.name == object_name:
                return {
                    name: prop.value
                    for name, prop in obj.properties.items()
                }
        return {}

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the entire asset to a dictionary.

        Returns:
            Dict representation of the asset
        """
        return {
            "file": os.path.basename(self.file_path),
            "engine_version": self.summary.saved_by_engine_version,
            "objects": [
                {
                    "name": obj.name,
                    "class": obj.class_name,
                    "properties": {
                        name: {
                            "type": prop.type_name,
                            "value": prop.value,
                            "struct_name": prop.struct_name if prop.struct_name else None,
                        }
                        for name, prop in obj.properties.items()
                    }
                }
                for obj in self.objects
            ]
        }
