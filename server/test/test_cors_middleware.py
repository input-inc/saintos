"""Regression tests for the aiohttp CORS middleware.

The Tauri-based SAINT Controller runs with origin `tauri://localhost`
and cross-origin-fetches `/api/firmware/controller` during its update
check. Without the right Access-Control-Allow-Origin header, WebKit
blocks the response and the operator sees "TypeError: Load failed"
with no useful context. These tests pin down the three behaviours
that fix matters for:

  1. Normal 200 responses get the header attached.
  2. OPTIONS preflight requests short-circuit to 204 with the
     Allow-Methods + Allow-Headers + Max-Age headers.
  3. Errors raised as HTTPException still get the header — otherwise
     the browser hides the error body and the operator gets "Load
     failed" instead of "HTTP 404".

The middleware is leaf-level and lives at module scope in
http_server.py, so we exercise it directly with mock Request +
handler instead of standing up a full aiohttp test client.
"""

from __future__ import annotations

import asyncio
from unittest.mock import MagicMock

import pytest
from aiohttp import web

from saint_server.webserver.http_server import (
    CORS_ALLOW_ORIGIN,
    CORS_ALLOW_METHODS,
    CORS_ALLOW_HEADERS,
    CORS_MAX_AGE,
    cors_middleware,
)


def _run(coro):
    """Run a coroutine synchronously. Avoids pulling pytest-asyncio just
    for these three tests — the middleware is plain async/await."""
    return asyncio.get_event_loop().run_until_complete(coro)


def _mock_get_request() -> MagicMock:
    req = MagicMock()
    req.method = "GET"
    return req


def _mock_options_request() -> MagicMock:
    req = MagicMock()
    req.method = "OPTIONS"
    return req


def test_cors_middleware_attaches_origin_to_normal_response():
    """Normal happy-path request: handler returns a 200, middleware
    must tack Access-Control-Allow-Origin onto the response. Without
    this, the Tauri webview's update check fails silently."""
    async def handler(request):
        return web.Response(text="ok", status=200)

    response = _run(cors_middleware(_mock_get_request(), handler))
    assert response.status == 200
    assert response.headers["Access-Control-Allow-Origin"] == CORS_ALLOW_ORIGIN


def test_cors_middleware_short_circuits_options_preflight():
    """OPTIONS preflight must NOT reach the handler. Returns 204 +
    the Allow-Methods/Allow-Headers/Max-Age headers so the browser
    can complete its preflight even for endpoints that don't have
    OPTIONS routes registered."""
    handler_called = []

    async def handler(request):
        handler_called.append(True)
        return web.Response(status=200)

    response = _run(cors_middleware(_mock_options_request(), handler))
    assert response.status == 204
    assert handler_called == [], "OPTIONS preflight must short-circuit before the handler"
    assert response.headers["Access-Control-Allow-Origin"] == CORS_ALLOW_ORIGIN
    assert response.headers["Access-Control-Allow-Methods"] == CORS_ALLOW_METHODS
    assert response.headers["Access-Control-Allow-Headers"] == CORS_ALLOW_HEADERS
    assert response.headers["Access-Control-Max-Age"] == CORS_MAX_AGE


def test_cors_middleware_attaches_origin_to_http_exception():
    """Raised HTTPException paths (the standard aiohttp idiom for
    returning errors) still need the CORS header. Otherwise WebKit
    treats the response as opaque, the operator's UI shows the
    cryptic "Load failed" instead of the actual HTTP status, and
    we lose visibility into which endpoint actually failed."""
    async def handler(request):
        raise web.HTTPNotFound(text="firmware not found")

    with pytest.raises(web.HTTPNotFound) as excinfo:
        _run(cors_middleware(_mock_get_request(), handler))

    # The exception's headers dict must carry the CORS header. aiohttp
    # serializes HTTPException.headers into the actual response, so
    # checking the exception attribute is equivalent to checking the
    # wire response.
    assert excinfo.value.status == 404
    assert excinfo.value.headers["Access-Control-Allow-Origin"] == CORS_ALLOW_ORIGIN
