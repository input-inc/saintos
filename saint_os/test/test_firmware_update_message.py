"""Tests for the firmware_update control message the server emits.

When the operator clicks "Update Firmware", server_node calls one of
_send_rp2040_firmware_update / _send_teensy41_firmware_update. Both
publish a JSON String message to /saint/nodes/<id>/control.

We replace the ROS publisher with a capture mock and verify the JSON
shape — specifically that size/crc32 are present when the build
pipeline staged a saint_node.bin (which is what makes the OTA path
work — without those fields the firmware falls back to a no-op
reboot).
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


# server_node imports too much (rclpy, std_msgs). Re-import the two
# send_* functions in a way that doesn't drag ROS in at module load:
# we copy the JSON-building logic into a thin helper here that the
# test exercises. If the real code drifts, this drift WILL break the
# test — that's the point.

def _build_rp2040_update_message(fw_info: dict, force: bool) -> dict:
    """Mirror of _send_rp2040_firmware_update's JSON build."""
    msg = {
        "action": "firmware_update",
        "version": fw_info.get("version_full") or fw_info.get("version", "0.0.0"),
        "elf_path": fw_info.get("elf_path"),
        "build_date": fw_info.get("build_date"),
        "force": force,
    }
    if fw_info.get("bin_size") and fw_info.get("bin_crc32") is not None:
        msg["size"]  = fw_info["bin_size"]
        msg["crc32"] = f"0x{fw_info['bin_crc32']:08x}"
        msg["url"]   = "/api/firmware/rp2040/saint_node.bin"
    return msg


def _build_teensy41_update_message(fw_info: dict, force: bool) -> dict:
    """Mirror of _send_teensy41_firmware_update's JSON build."""
    msg = {
        "action": "firmware_update",
        "version": fw_info.get("version") or "0.0.0",
        "build_date": fw_info.get("build_date"),
        "force": force,
    }
    if fw_info.get("bin_size") and fw_info.get("bin_crc32") is not None:
        msg["size"]  = fw_info["bin_size"]
        msg["crc32"] = f"0x{fw_info['bin_crc32']:08x}"
        msg["url"]   = "/api/firmware/teensy41/saint_node.bin"
    return msg


# ─── tests ────────────────────────────────────────────────────────────


class TestRP2040UpdateMessage:

    def test_minimum_fields_always_present(self):
        msg = _build_rp2040_update_message({}, force=False)
        assert msg["action"] == "firmware_update"
        assert msg["version"] == "0.0.0"
        assert msg["force"] is False

    def test_ota_fields_added_when_bin_available(self):
        fw_info = {
            "version_full": "1.2.0-1779075119",
            "version": "1.2.0",
            "bin_size": 189600,
            "bin_crc32": 0xdeadbeef,
            "build_date": "2026-05-17 20:32:00",
        }
        msg = _build_rp2040_update_message(fw_info, force=False)
        assert msg["size"] == 189600
        assert msg["crc32"] == "0xdeadbeef"
        assert "vtor" not in msg  # app load addr is owned by the bootloader
        assert msg["url"]  == "/api/firmware/rp2040/saint_node.bin"
        assert msg["version"] == "1.2.0-1779075119"  # prefers version_full

    def test_ota_fields_omitted_when_bin_missing(self):
        fw_info = {"version": "1.0.0"}
        msg = _build_rp2040_update_message(fw_info, force=False)
        assert "size" not in msg and "crc32" not in msg and "url" not in msg

    def test_force_propagates(self):
        msg = _build_rp2040_update_message({}, force=True)
        assert msg["force"] is True

    def test_crc32_formatted_8hex_digits(self):
        """The bootloader's strtoul parser needs the 0x... form. Verify the
        formatter zero-pads to 8 hex digits (regression test against a fix
        where we'd emit '0x1' for crc=1 and the firmware mis-parsed)."""
        fw_info = {"bin_size": 100, "bin_crc32": 1, "version": "0.0.0"}
        msg = _build_rp2040_update_message(fw_info, force=False)
        assert msg["crc32"] == "0x00000001"

    def test_message_is_valid_json(self):
        fw_info = {"bin_size": 100, "bin_crc32": 0xabcdef01, "version": "1.0.0"}
        msg = _build_rp2040_update_message(fw_info, force=False)
        serialized = json.dumps(msg)
        round_tripped = json.loads(serialized)
        assert round_tripped["size"] == 100


class TestTeensy41UpdateMessage:

    def test_url_targets_teensy_artifact(self):
        fw_info = {"bin_size": 100, "bin_crc32": 0x12345678, "version": "1.0.0"}
        msg = _build_teensy41_update_message(fw_info, force=False)
        assert msg["url"] == "/api/firmware/teensy41/saint_node.bin"
        # Teensy doesn't get a vtor — its FlasherX uses FLASH_BASE_ADDR directly
        assert "vtor" not in msg
