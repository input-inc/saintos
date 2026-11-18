"""Tests for per-Debian-release dist tarball selection.

The server ships one tarball per Debian release: the bookworm build keeps
the legacy unsuffixed name, trixie adds a `_trixie` suffix. Every update
path (GitHub, USB, browser upload) must only offer the variant that matches
the host — a bookworm build's debs conflict with trixie's t64 libraries
(libssl3 vs libssl3t64) and its ROS tree needs libpython3.11.
"""
from __future__ import annotations

import asyncio
import io
import json

import pytest

import saint_server.update_manager as um
from saint_server.update_manager import (
    TARBALL_RE,
    UpdateManager,
    host_debian_release,
    tarball_debian_release,
    tarball_fits_host,
)


def _run(coro):
    return asyncio.get_event_loop().run_until_complete(coro)


BOOKWORM = "saint-os_0.2.4_arm64_kilted.tar.zst"
TRIXIE = "saint-os_0.2.4_arm64_kilted_trixie.tar.zst"


# ── name parsing ───────────────────────────────────────────────────────

@pytest.mark.parametrize("name, version, release", [
    (BOOKWORM, "0.2.4", "bookworm"),
    (TRIXIE, "0.2.4", "trixie"),
    ("saint-os_0.5.0-local.7f290be_arm64_kilted_trixie.tar.zst",
     "0.5.0-local.7f290be", "trixie"),
    ("saint-os_v0.5.1_arm64_jazzy.tar.gz", "0.5.1", "bookworm"),
])
def test_tarball_re_parses_release(name, version, release):
    m = TARBALL_RE.match(name)
    assert m is not None
    assert m.group("version") == version
    assert m.group("arch") == "arm64"
    assert tarball_debian_release(m) == release


def test_legacy_regex_cannot_see_trixie_tarball():
    """Servers already in the field parse with the pre-variant regex. The
    trixie name must NOT match it, or a bookworm host would OTA onto it."""
    import re
    legacy = re.compile(
        r"^saint-os_v?(?P<version>[0-9]+\.[0-9]+\.[0-9]+(?:[-+][^_]+)?)"
        r"_(?P<arch>[a-z0-9]+)_(?P<ros>[a-z0-9]+)\.tar\.(?:gz|zst)$"
    )
    assert legacy.match(BOOKWORM)
    assert not legacy.match(TRIXIE)


def test_fits_host():
    assert tarball_fits_host(TARBALL_RE.match(BOOKWORM), "bookworm")
    assert not tarball_fits_host(TARBALL_RE.match(BOOKWORM), "trixie")
    assert tarball_fits_host(TARBALL_RE.match(TRIXIE), "trixie")
    assert not tarball_fits_host(TARBALL_RE.match(TRIXIE), "bookworm")
    # Unknown host (dev box, no /etc/os-release) filters nothing.
    assert tarball_fits_host(TARBALL_RE.match(TRIXIE), None)


# ── host detection ─────────────────────────────────────────────────────

@pytest.mark.parametrize("contents, expected", [
    ('ID=debian\nVERSION_CODENAME=trixie\n', "trixie"),
    ('PRETTY_NAME="Raspbian GNU/Linux 12 (bookworm)"\nID=raspbian\n'
     'VERSION_CODENAME=bookworm\n', "bookworm"),
    ('ID=ubuntu\nVERSION_CODENAME=noble\n', "bookworm"),
    ('ID=fedora\nVERSION_ID=40\n', None),
    ('ID=debian\n', None),
])
def test_host_debian_release(tmp_path, contents, expected):
    f = tmp_path / "os-release"
    f.write_text(contents)
    assert host_debian_release(str(f)) == expected


def test_host_debian_release_missing_file(tmp_path):
    assert host_debian_release(str(tmp_path / "nope")) is None


# ── update paths ───────────────────────────────────────────────────────

def _manager(monkeypatch, release):
    monkeypatch.setattr(um, "host_debian_release", lambda: release)
    mgr = UpdateManager()
    mgr.state.installed_version = "0.2.3"
    return mgr


@pytest.mark.parametrize("host, expected", [
    ("bookworm", BOOKWORM),
    ("trixie", TRIXIE),
])
def test_github_picks_host_variant(monkeypatch, host, expected):
    """The release carries both tarballs; the host gets its own, regardless
    of asset order."""
    payload = {
        "tag_name": "v0.2.4",
        "assets": [
            {"name": TRIXIE, "browser_download_url": "https://x/t", "size": 2},
            {"name": BOOKWORM, "browser_download_url": "https://x/b", "size": 1},
        ],
    }
    monkeypatch.setattr(
        um.urllib.request, "urlopen",
        lambda req, timeout=None: io.BytesIO(json.dumps(payload).encode()))
    release = _manager(monkeypatch, host)._fetch_latest_release()
    assert release.asset_name == expected


def test_github_no_matching_variant(monkeypatch):
    """A release that predates trixie builds offers a trixie host nothing to
    download rather than the bookworm tarball."""
    payload = {"tag_name": "v0.2.3", "assets": [
        {"name": "saint-os_0.2.3_arm64_kilted.tar.zst",
         "browser_download_url": "https://x/b", "size": 1}]}
    monkeypatch.setattr(
        um.urllib.request, "urlopen",
        lambda req, timeout=None: io.BytesIO(json.dumps(payload).encode()))
    release = _manager(monkeypatch, "trixie")._fetch_latest_release()
    assert release.asset_url is None


def test_usb_scan_skips_other_variant(monkeypatch, tmp_path):
    mount = tmp_path / "media" / "stick"
    mount.mkdir(parents=True)
    (mount / BOOKWORM).write_bytes(b"b")
    (mount / TRIXIE).write_bytes(b"t")
    mgr = _manager(monkeypatch, "trixie")
    monkeypatch.setattr(um.UpdateManager, "_walk_mounts", staticmethod(
        lambda root: iter([str(mount / BOOKWORM), str(mount / TRIXIE)])
        if root == "/media" else iter([])))
    monkeypatch.setattr(um.os.path, "isdir", lambda p: p == "/media")

    found = mgr._scan_usb_unprivileged()
    assert [r.asset_name for r in found] == [TRIXIE]


def test_register_upload_rejects_other_variant(monkeypatch, tmp_path):
    tarball = tmp_path / BOOKWORM
    tarball.write_bytes(b"x")
    mgr = _manager(monkeypatch, "trixie")
    with pytest.raises(ValueError, match="bookworm build"):
        _run(mgr.register_uploaded_tarball(str(tarball)))
    assert mgr.state.status != "downloaded"

