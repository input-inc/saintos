"""Firmware artifact listing for the Settings → Firmware download rows.

The listing has to cover what's actually staged. The original directory
scan only looked for ['.zip', '.tar.gz', '.tgz', '.elf', '.AppImage'],
which missed every flashable image we ship — .uf2 for RP2040, .hex for
Teensy, .tar.zst for the Pi — so a scan surfaced debug symbols and
nothing you could flash.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

pytest.importorskip("aiohttp", reason="http_server requires aiohttp")

from saint_server.webserver.http_server import WebServer


@pytest.fixture
def fw_root(tmp_path):
    """A firmware root shaped like server/resources/firmware/."""
    def mk(rel, size=16):
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(b"\x00" * size)
        return p

    mk("rp2040/saint_node.uf2", 100)
    mk("rp2040/saint_node_combined.uf2", 200)
    mk("rp2040/saint_ota_bootloader.uf2", 50)
    mk("rp2040/saint_node.elf", 400)
    mk("rp2040/saint_node.bin", 80)
    mk("rp2040/generated/version.h", 10)          # not an artifact
    mk("teensy41/firmware.hex", 300)
    mk("teensy41/saint_node.bin", 90)
    mk("raspberrypi/saint_firmware_raspberrypi_1.1.0.tar.zst", 500)
    return tmp_path


@pytest.fixture
def server(tmp_path, fw_root):
    web_root = tmp_path / "web"
    web_root.mkdir(exist_ok=True)
    return WebServer(web_root=str(web_root), firmware_root=str(fw_root))


def _names(files):
    return [f["filename"] for f in files]


def test_flashable_images_are_listed(server, fw_root):
    """The whole point: .uf2 / .hex / .tar.zst must be downloadable."""
    files = server._list_firmware_files(fw_root / "rp2040")
    assert "saint_node.uf2" in _names(files)

    tee = server._list_firmware_files(fw_root / "teensy41")
    assert "firmware.hex" in _names(tee)

    pi = server._list_firmware_files(fw_root / "raspberrypi")
    assert "saint_firmware_raspberrypi_1.1.0.tar.zst" in _names(pi)


def test_flashable_sorts_ahead_of_debug_artifacts(server, fw_root):
    """An operator scanning the list should hit the image before the
    debug symbols — .elf and .bin are the least likely thing wanted."""
    names = _names(server._list_firmware_files(fw_root / "rp2040"))
    assert names.index("saint_node.uf2") < names.index("saint_node.elf")
    assert names.index("saint_node.uf2") < names.index("saint_node.bin")


def test_directories_and_unknown_extensions_skipped(server, fw_root):
    """`generated/` holds build headers, not artifacts."""
    names = _names(server._list_firmware_files(fw_root / "rp2040"))
    assert "generated" not in names
    assert not any(n.endswith(".h") for n in names)


def test_urls_are_routable_and_sizes_real(server, fw_root):
    """Each entry must carry a URL the download route actually serves,
    and the true size — the UI renders both."""
    for f in server._list_firmware_files(fw_root / "rp2040"):
        assert f["url"] == f"/api/firmware/rp2040/{f['filename']}"
        assert f["size"] == (fw_root / "rp2040" / f["filename"]).stat().st_size


def test_info_json_types_still_get_files(server, fw_root):
    """Types shipping an info.json used to return only that file's
    contents, leaving the dashboard no way to learn what was on disk —
    which is most types, since Pi/Teensy/controller all ship one."""
    (fw_root / "teensy41" / "info.json").write_text(
        '{"version": "1.2.3", "available": true}')
    info = server._get_firmware_info("teensy41")
    assert info["version"] == "1.2.3", "info.json contents must survive"
    assert "firmware.hex" in _names(info["files"])


def test_type_with_only_unlisted_files_returns_none(server, tmp_path):
    empty = tmp_path / "fwroot2" / "rp2040"
    empty.mkdir(parents=True)
    (empty / "README.md").write_text("nothing to flash here")
    srv = WebServer(web_root=str(tmp_path), firmware_root=str(tmp_path / "fwroot2"))
    assert srv._get_firmware_info("rp2040") is None


def test_missing_type_dir_returns_none(server):
    assert server._get_firmware_info("nonexistent") is None


def test_listing_does_not_checksum(server, fw_root, monkeypatch):
    """Hashing every artifact to render a download button would mean
    SHA256 over a ~630 MB Pi bundle on each page load."""
    def boom(*_a, **_k):
        raise AssertionError("_calculate_checksum called during listing")
    monkeypatch.setattr(server, "_calculate_checksum", boom)
    server._list_firmware_files(fw_root / "raspberrypi")
