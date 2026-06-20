"""Tests for the browser update-upload path.

Two layers:

  1. UpdateManager.register_uploaded_tarball — the new server-side logic
     that turns a freshly-uploaded tarball into the 'downloaded' state the
     existing Install flow consumes. Pure stdlib, runs on the bare host.

  2. The /api/update/upload and /api/update/log HTTP routes — exercised
     against a real aiohttp test server. These need aiohttp (and a running
     loop), so they importorskip and run via a manual loop helper to avoid
     a hard pytest-asyncio dependency, matching test_cors_middleware.py.
"""
from __future__ import annotations

import asyncio

import pytest

from saint_server.update_manager import UpdateManager


def _run(coro):
    """Run a coroutine synchronously — avoids a pytest-asyncio dependency."""
    return asyncio.get_event_loop().run_until_complete(coro)


VALID_NAME = "saint-os_2.0.1_arm64_kilted.tar.zst"


# ── register_uploaded_tarball ──────────────────────────────────────────

def test_register_uploaded_flips_to_downloaded(tmp_path):
    """A valid tarball name + real file → status 'downloaded', staged path
    set, and a synthetic 'upload'-sourced release the UI can render."""
    tarball = tmp_path / VALID_NAME
    tarball.write_bytes(b"x" * 1234)

    mgr = UpdateManager()
    state = _run(mgr.register_uploaded_tarball(str(tarball)))

    assert state.status == "downloaded"
    assert state.staged_tarball == str(tarball)
    assert state.download_received == 1234
    assert state.last_error is None
    assert state.github_release is not None
    assert state.github_release.version == "2.0.1"
    assert state.github_release.source == "upload"
    assert state.github_release.asset_name == VALID_NAME


def test_register_uploaded_allows_downgrade(tmp_path):
    """Unlike GitHub/USB, an explicit upload is NOT gated on being newer —
    reinstalling or downgrading is a legitimate operator choice."""
    tarball = tmp_path / "saint-os_1.0.0_arm64_kilted.tar.zst"
    tarball.write_bytes(b"data")

    mgr = UpdateManager()
    mgr.state.installed_version = "9.9.9"   # uploaded version is older
    state = _run(mgr.register_uploaded_tarball(str(tarball)))

    assert state.status == "downloaded"
    assert state.github_release.version == "1.0.0"


def test_register_uploaded_rejects_bad_name(tmp_path):
    """A file whose name isn't a dist tarball must be rejected — the name
    is what apply-update.sh and the version parser key off of."""
    bad = tmp_path / "not-a-tarball.zip"
    bad.write_bytes(b"data")

    mgr = UpdateManager()
    with pytest.raises(ValueError):
        _run(mgr.register_uploaded_tarball(str(bad)))
    assert mgr.state.status != "downloaded"


def test_register_uploaded_rejects_missing_file(tmp_path):
    """A valid name but no file on disk is a FileNotFoundError, not a
    silently-staged ghost tarball."""
    mgr = UpdateManager()
    with pytest.raises(FileNotFoundError):
        _run(mgr.register_uploaded_tarball(str(tmp_path / VALID_NAME)))


# ── HTTP routes (need real aiohttp) ────────────────────────────────────

def _make_webserver(tmp_path, manager):
    """Build a minimal WebServer with the two update routes wired and the
    staging/log dirs pointed at tmp_path. Skips if aiohttp is absent."""
    pytest.importorskip("aiohttp.test_utils")
    from aiohttp import web
    from saint_server.webserver import http_server
    import saint_server.update_manager as um

    # Point staging + log dirs at the tmp tree. The upload handler imports
    # UPDATE_STAGING from update_manager at call time, so patching the
    # module attribute here takes effect.
    staging = tmp_path / "updates"
    logs = tmp_path / "log"
    staging.mkdir()
    logs.mkdir()
    http_server.UPDATE_LOG_DIR = logs
    um.UPDATE_STAGING = staging

    # A bare object carrying just what the two handlers touch, with the
    # real (unbound) handler methods bound onto it.
    ws_handler = type("WS", (), {"_update_manager": manager})()
    obj = type("Srv", (), {
        "ws_handler": ws_handler,
        "log": lambda self, *a, **k: None,
        "NO_CACHE_HEADERS": {},
        "_handle_update_upload": http_server.WebServer._handle_update_upload,
        "_handle_update_log": http_server.WebServer._handle_update_log,
    })()

    app = web.Application()
    app.router.add_post("/api/update/upload", obj._handle_update_upload)
    app.router.add_get("/api/update/log", obj._handle_update_log)
    return app, staging, logs


def test_upload_route_streams_and_stages(tmp_path):
    """POST /api/update/upload streams the file to the staging dir and
    flips the manager to 'downloaded'."""
    pytest.importorskip("aiohttp.test_utils")
    from aiohttp.test_utils import TestClient, TestServer
    from aiohttp import FormData

    mgr = UpdateManager()
    app, staging, _ = _make_webserver(tmp_path, mgr)

    async def scenario():
        async with TestClient(TestServer(app)) as client:
            form = FormData()
            form.add_field("file", b"y" * 5000, filename=VALID_NAME,
                           content_type="application/octet-stream")
            resp = await client.post("/api/update/upload", data=form)
            assert resp.status == 200
            body = await resp.json()
            assert body["version"] == "2.0.1"
            assert (staging / VALID_NAME).is_file()
            assert (staging / VALID_NAME).stat().st_size == 5000
            assert mgr.state.status == "downloaded"

    _run(scenario())


def test_upload_route_rejects_bad_name(tmp_path):
    """A non-tarball filename is a 400 and nothing lands in staging."""
    pytest.importorskip("aiohttp.test_utils")
    from aiohttp.test_utils import TestClient, TestServer
    from aiohttp import FormData

    mgr = UpdateManager()
    app, staging, _ = _make_webserver(tmp_path, mgr)

    async def scenario():
        async with TestClient(TestServer(app)) as client:
            form = FormData()
            form.add_field("file", b"junk", filename="evil.sh",
                           content_type="application/octet-stream")
            resp = await client.post("/api/update/upload", data=form)
            assert resp.status == 400
            assert list(staging.iterdir()) == []

    _run(scenario())


def test_upload_route_503_without_manager(tmp_path):
    """No update manager wired yet → 503, not a crash."""
    pytest.importorskip("aiohttp.test_utils")
    from aiohttp.test_utils import TestClient, TestServer
    from aiohttp import FormData

    app, _, _ = _make_webserver(tmp_path, None)

    async def scenario():
        async with TestClient(TestServer(app)) as client:
            form = FormData()
            form.add_field("file", b"x", filename=VALID_NAME)
            resp = await client.post("/api/update/upload", data=form)
            assert resp.status == 503

    _run(scenario())


def test_log_route_offset_and_newest(tmp_path):
    """GET /api/update/log tails the newest update-*.log from the given
    offset and reports the next offset."""
    pytest.importorskip("aiohttp.test_utils")
    from aiohttp.test_utils import TestClient, TestServer

    app, _, logs = _make_webserver(tmp_path, UpdateManager())

    async def scenario():
        async with TestClient(TestServer(app)) as client:
            # No log yet → present=False.
            r0 = await (await client.get("/api/update/log")).json()
            assert r0["present"] is False

            (logs / "update-20260101-000000.log").write_text("hello\n")
            r1 = await (await client.get("/api/update/log?offset=0")).json()
            assert r1["present"] is True
            assert r1["data"] == "hello\n"
            assert r1["offset"] == 6

            # Append and read only the delta from the prior offset.
            (logs / "update-20260101-000000.log").write_text("hello\nworld\n")
            r2 = await (await client.get(f"/api/update/log?offset={r1['offset']}")).json()
            assert r2["data"] == "world\n"

    _run(scenario())
