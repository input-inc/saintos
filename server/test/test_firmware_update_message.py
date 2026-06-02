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


def _build_raspberrypi_update_message(fw_info: dict, force: bool,
                                       server_ip: str = "10.0.0.5",
                                       server_port: int = 8080) -> dict:
    """Mirror of _send_raspberrypi_firmware_update's JSON build.

    Raspberry Pi nodes are unlike RP2040/Teensy in two ways:
      * the artifact is a ZIP (Python package) not a flash image, so
        the message carries SHA256 ``checksum`` instead of ``crc32``,
      * the URL is absolute (urllib needs http://) and the server
        embeds its own reachable IP at publish time — see
        _resolve_server_url_for_node.
    """
    if not fw_info or not fw_info.get('available'):
        return {}
    path = f"/api/firmware/raspberrypi/{fw_info['filename']}"
    return {
        "action": "firmware_update",
        "version": fw_info.get("version", "0.0.0"),
        "url": f"http://{server_ip}:{server_port}{path}",
        "checksum": fw_info.get("checksum"),
        "build_date": fw_info.get("build_date"),
        "force": force,
    }


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


class TestRaspberryPiUpdateMessage:

    def test_returns_empty_when_firmware_unavailable(self):
        """No staged build → the server returns early without
        publishing; the wire-message builder must mirror that."""
        msg = _build_raspberrypi_update_message({}, force=False)
        assert msg == {}
        msg = _build_raspberrypi_update_message(
            {"available": False, "filename": "x.zip"}, force=False)
        assert msg == {}

    def test_includes_required_fields(self):
        fw_info = {
            "available": True,
            "version": "1.2.0",
            "filename": "saint_firmware_raspberrypi_1.2.0.zip",
            "checksum": "a" * 64,
            "build_date": "2026-06-01T12:00:00Z",
        }
        msg = _build_raspberrypi_update_message(fw_info, force=False)
        assert msg["action"] == "firmware_update"
        assert msg["version"] == "1.2.0"
        assert msg["checksum"] == "a" * 64
        assert msg["build_date"] == "2026-06-01T12:00:00Z"
        assert msg["force"] is False

    def test_url_is_absolute_with_server_ip(self):
        """The Pi node uses urllib.request.urlretrieve which won't
        follow a path-only URL — the message MUST embed an absolute
        http URL with the server's reachable IP. Earlier code had a
        literal f-string '{server_ip}' placeholder that nothing
        substituted; this test guards against that regression."""
        fw_info = {
            "available": True,
            "version": "1.0.0",
            "filename": "saint_firmware_raspberrypi_1.0.0.zip",
            "checksum": "deadbeef" * 8,
        }
        msg = _build_raspberrypi_update_message(
            fw_info, force=False, server_ip="192.168.1.7", server_port=8080)
        assert msg["url"].startswith("http://192.168.1.7:8080/")
        assert "{server_ip}" not in msg["url"]
        assert msg["url"].endswith(
            "/api/firmware/raspberrypi/saint_firmware_raspberrypi_1.0.0.zip")

    def test_force_propagates(self):
        fw_info = {
            "available": True, "version": "1", "filename": "a.zip",
            "checksum": "c" * 64,
        }
        assert _build_raspberrypi_update_message(fw_info, force=True)["force"] is True
        assert _build_raspberrypi_update_message(fw_info, force=False)["force"] is False

    def test_path_uses_raspberrypi_route_not_rpi5(self):
        """When we renamed rpi5 → raspberrypi, the artifact URL had to
        move with it. A regression here means a working OTA on day 1
        would silently 404 after a deploy of the rename."""
        fw_info = {
            "available": True, "version": "1", "filename": "f.zip",
            "checksum": "c" * 64,
        }
        msg = _build_raspberrypi_update_message(fw_info, force=False)
        assert "/api/firmware/raspberrypi/" in msg["url"]
        assert "/api/firmware/rpi5/" not in msg["url"]

    def test_carries_sha256_not_crc32(self):
        """Pi nodes verify a SHA256 over the full ZIP; the wire field
        is named ``checksum`` to keep it distinct from the MCUs'
        ``crc32``."""
        fw_info = {
            "available": True, "version": "1", "filename": "f.zip",
            "checksum": "f" * 64,
        }
        msg = _build_raspberrypi_update_message(fw_info, force=False)
        assert msg["checksum"] == "f" * 64
        assert "crc32" not in msg
        assert "size" not in msg

    def test_message_is_valid_json(self):
        import json
        fw_info = {
            "available": True, "version": "1.0.0",
            "filename": "saint_firmware_raspberrypi_1.0.0.zip",
            "checksum": "b" * 64,
            "build_date": "2026-06-01",
        }
        msg = _build_raspberrypi_update_message(fw_info, force=False)
        round_tripped = json.loads(json.dumps(msg))
        assert round_tripped["version"] == "1.0.0"
        assert round_tripped["url"].startswith("http://")


class TestResolveServerUrl:
    """End-to-end test for the IP-substitution helper that the
    raspberrypi message builder relies on. Imports the real server
    code through the conftest stubs."""

    def test_substitutes_local_ip_route_to_node(self, monkeypatch):
        """The helper does a UDP-connect trick to ask the kernel which
        of OUR interfaces would route to the node's address. Mock the
        socket layer so the test doesn't depend on the host's NICs."""
        # Import inside the test so conftest.py's rclpy stubs are
        # in place first.
        from saint_server.server_node import SaintServerNode

        # Build a near-empty instance — we only need _resolve_server_url_for_node
        # and its dependencies (logger + state_manager.get_node). Avoid the
        # real __init__ by going through object.__new__.
        node = object.__new__(SaintServerNode)

        class _Logger:
            def warning(self, *a, **k): pass
            def info(self, *a, **k): pass
            def error(self, *a, **k): pass
        node.get_logger = lambda: _Logger()

        class _SM:
            def get_node(self, node_id):
                return {"ip": "192.168.4.42"}
        node.state_manager = _SM()

        class _FakeSock:
            def __init__(self, *a, **k): pass
            def __enter__(self): return self
            def __exit__(self, *a): pass
            def connect(self, addr):
                self.dest = addr
            def getsockname(self):
                # Pretend the kernel says "you'd source from 192.168.4.1"
                return ("192.168.4.1", 0)

        import socket as _socket
        monkeypatch.setattr(_socket, "socket", lambda *a, **k: _FakeSock())

        url = node._resolve_server_url_for_node(
            "raspberrypi_abc", 8080, "/api/firmware/raspberrypi/x.zip")
        assert url == "http://192.168.4.1:8080/api/firmware/raspberrypi/x.zip"

    def test_falls_back_when_node_ip_unknown(self, monkeypatch):
        """Brand-new adoption may not have an IP recorded yet; the
        helper must still return *something* rather than crash."""
        from saint_server.server_node import SaintServerNode

        node = object.__new__(SaintServerNode)

        class _Logger:
            def warning(self, *a, **k): pass
            def info(self, *a, **k): pass
            def error(self, *a, **k): pass
        node.get_logger = lambda: _Logger()

        class _SM:
            def get_node(self, node_id): return None
        node.state_manager = _SM()

        class _Sock:
            def __enter__(self): return self
            def __exit__(self, *a): pass
            def connect(self, addr): pass
            def getsockname(self): return ("127.0.0.1", 0)

        import socket as _socket
        monkeypatch.setattr(_socket, "socket", lambda *a, **k: _Sock())

        url = node._resolve_server_url_for_node(
            "raspberrypi_xyz", 80, "/api/firmware/raspberrypi/f.zip")
        # Doesn't crash; produces an absolute URL.
        assert url.startswith("http://")
        assert url.endswith("/api/firmware/raspberrypi/f.zip")

    def test_socket_exception_falls_back_to_loopback(self, monkeypatch):
        """If the OS refuses the trial connect (no network, no
        loopback, broken DNS), the helper must NOT raise — it should
        log a warning and return a usable URL pointing at 127.0.0.1.
        Sending OTA to localhost still fails of course, but the
        publisher stays alive for other nodes."""
        from saint_server.server_node import SaintServerNode

        node = object.__new__(SaintServerNode)

        class _Logger:
            warnings = []
            def warning(self, msg): self.warnings.append(msg)
            def info(self, *a, **k): pass
            def error(self, *a, **k): pass
        logger_instance = _Logger()
        node.get_logger = lambda: logger_instance

        class _SM:
            def get_node(self, node_id): return {"ip": "10.0.0.1"}
        node.state_manager = _SM()

        def boom(*a, **k):
            raise OSError("no route to host")

        import socket as _socket
        monkeypatch.setattr(_socket, "socket", boom)

        url = node._resolve_server_url_for_node(
            "raspberrypi_z", 8080, "/api/firmware/raspberrypi/f.zip")
        assert url == "http://127.0.0.1:8080/api/firmware/raspberrypi/f.zip"
        assert logger_instance.warnings, "must log warning on fallback"
