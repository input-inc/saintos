"""
SAINT.OS HTTP Server

Serves static files and provides WebSocket endpoint for the admin interface.
"""

import hashlib
import json
import mimetypes
import os
from pathlib import Path
from typing import Optional, Dict, Any

from aiohttp import web

from saint_server.animation.urdf_store import URDFStore, URDFStoreError
from saint_server.webserver.state_manager import StateManager
from saint_server.webserver.websocket_handler import WebSocketHandler


# Hard cap on URDF + mesh bundle uploads. 64 MB is generous for any
# real robot — meshes are typically a few MB each, and we expect tens
# of links at most. Larger bundles probably contain unrelated assets
# (textures, animations) that don't belong in the URDF store.
MAX_URDF_UPLOAD_BYTES = 64 * 1024 * 1024


# CORS policy. Required because the SAINT Controller AppImage (Tauri
# webview) runs with origin `tauri://localhost` and cross-origin-fetches
# this server's /api/firmware/* endpoints during the in-app update
# check. Without these headers WebKit blocks the response with the
# canonical "TypeError: Load failed" — same symptom on any future tool
# that serves the SPA from a different origin.
#
# Security note: the server lives on a private AP behind WPA2.
# `Access-Control-Allow-Origin: *` here only changes what the browser
# will let JS read from the response — it doesn't grant any access the
# network itself wouldn't already provide. Stuff that should be
# authenticated (WebSocket management actions) uses its own password
# auth flow, untouched by these headers.
CORS_ALLOW_ORIGIN = "*"
CORS_ALLOW_METHODS = "GET, POST, OPTIONS"
CORS_ALLOW_HEADERS = "Content-Type, Authorization"
CORS_MAX_AGE = "600"


@web.middleware
async def cors_middleware(request, handler):
    """Aiohttp middleware that attaches CORS headers to every response
    and short-circuits OPTIONS preflight requests with 204 + the
    headers the browser wants. Lifted to module scope so the
    behaviour can be regression-tested without standing up a full
    WebServer instance."""
    if request.method == "OPTIONS":
        # Short-circuit preflight requests. Some Tauri webviews preflight
        # simple GETs too, which would 404 against our routes if we let
        # them through. 204 No Content + the CORS headers is the canonical
        # response.
        return web.Response(status=204, headers={
            "Access-Control-Allow-Origin": CORS_ALLOW_ORIGIN,
            "Access-Control-Allow-Methods": CORS_ALLOW_METHODS,
            "Access-Control-Allow-Headers": CORS_ALLOW_HEADERS,
            "Access-Control-Max-Age": CORS_MAX_AGE,
        })
    try:
        response = await handler(request)
    except web.HTTPException as e:
        # Errors raised as HTTPException objects still need CORS headers
        # — otherwise the browser hides the error body and the operator
        # sees "Load failed" instead of "HTTP 404 firmware/foo not found".
        e.headers["Access-Control-Allow-Origin"] = CORS_ALLOW_ORIGIN
        raise
    response.headers["Access-Control-Allow-Origin"] = CORS_ALLOW_ORIGIN
    return response


class WebServer:
    """HTTP server for static files and WebSocket endpoint."""

    # Supported firmware types. 'controller' is the Steam Deck operator app
    # (Tauri AppImage); the others are node firmware. They share the same
    # /api/firmware* surface — the controller just happens to ship a single
    # .AppImage file instead of a .zip.
    FIRMWARE_TYPES = ['rp2040', 'rpi5', 'teensy41', 'controller']

    def __init__(
        self,
        web_root: str,
        port: int = 80,
        host: str = '0.0.0.0',
        server_name: str = 'SAINT-01',
        logger=None,
        state_manager: Optional[StateManager] = None,
        firmware_root: Optional[str] = None
    ):
        self.web_root = web_root
        self.port = port
        self.host = host
        self.server_name = server_name
        self.logger = logger

        # Firmware root defaults to resources/firmware relative to server
        if firmware_root:
            self.firmware_root = firmware_root
        else:
            server_dir = Path(__file__).parent.parent.parent
            self.firmware_root = str(server_dir / 'resources' / 'firmware')

        # Components - use provided state_manager or create new one
        self.state_manager = state_manager or StateManager(
            server_name=server_name, logger=logger
        )
        self.ws_handler = WebSocketHandler(self.state_manager, logger=logger)

        # URDF + mesh store. Sits beside the rest of the runtime
        # config (nodes/, system_routing.yaml). The store handles the
        # safe-extract + validate flow; the HTTP layer just shuttles
        # bytes back and forth.
        self.urdf_store = URDFStore(self.state_manager.config_dir, logger=logger)

        # aiohttp components
        self.app: Optional[web.Application] = None
        self.runner: Optional[web.AppRunner] = None
        self.site: Optional[web.TCPSite] = None

    def log(self, level: str, message: str):
        """Log a message if logger is available."""
        if self.logger:
            getattr(self.logger, level)(message)

    async def start(self):
        """Start the HTTP server."""
        self.app = web.Application()

        # Debug middleware to log requests
        @web.middleware
        async def debug_middleware(request, handler):
            self.log('debug', f'Request: {request.method} {request.path}')
            try:
                response = await handler(request)
                self.log('debug', f'Response: {response.status} for {request.path}')
                return response
            except web.HTTPException as e:
                self.log('debug', f'HTTP Exception: {e.status} for {request.path}')
                raise

        self.app.middlewares.append(debug_middleware)

        # CORS for the Tauri webview's cross-origin firmware fetches.
        # Defined at module scope (see top of file) so it's importable
        # by the test suite without spinning up a WebServer.
        self.app.middlewares.append(cors_middleware)

        # Add routes
        self.app.router.add_get('/api/ws', self.ws_handler.handle_connection)
        self.app.router.add_get('/api/status', self._handle_status)

        # Firmware routes
        self.app.router.add_get('/api/firmware', self._handle_firmware_list)
        self.app.router.add_get('/api/firmware/{fw_type}', self._handle_firmware_type_info)
        self.app.router.add_get('/api/firmware/{fw_type}/{filename}', self._handle_firmware_download)

        # Robot URDF routes — uploaded model + meshes served back to
        # the animation builder UI (and, in a follow-on phase, the
        # controller's 3D viewer).
        self.app.router.add_get('/api/robot/metadata', self._handle_urdf_metadata)
        self.app.router.add_get('/api/robot/urdf', self._handle_urdf_get)
        self.app.router.add_post('/api/robot/urdf', self._handle_urdf_upload)
        self.app.router.add_delete('/api/robot/urdf', self._handle_urdf_delete)
        self.app.router.add_get('/api/robot/joints', self._handle_urdf_joints)
        self.app.router.add_get('/api/robot/meshes/{filename}', self._handle_urdf_mesh)

        # Animation import — Pololu Maestro save-file conversion.
        self.app.router.add_post('/api/animations/import/maestro',
                                 self._handle_maestro_import)

        # Serve index.html for root
        self.app.router.add_get('/', self._handle_index)

        # Log resolved web_root
        resolved_root = os.path.abspath(self.web_root)
        self.log('info', f'Web root resolved to: {resolved_root}')

        # Serve static files using catch-all handler
        if os.path.isdir(self.web_root):
            self.app.router.add_get('/{path:.*}', self._handle_static)
            self.log('info', f'Serving static files from: {self.web_root}')
            # List JS files for debugging
            js_dir = os.path.join(resolved_root, 'js')
            if os.path.isdir(js_dir):
                js_files = os.listdir(js_dir)
                self.log('info', f'JS files available: {js_files}')
        else:
            self.log('warning', f'Web root directory not found: {self.web_root}')

        # Start the server
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()

        self.site = web.TCPSite(self.runner, self.host, self.port)
        await self.site.start()

        # Start broadcast loop
        await self.ws_handler.start_broadcast_loop()

        self.log('info', f'HTTP server listening on {self.host}:{self.port}')

    async def stop(self):
        """Stop the HTTP server gracefully."""
        self.log('info', 'Stopping HTTP server...')

        # Stop broadcast loop
        await self.ws_handler.stop_broadcast_loop()

        # Close all WebSocket connections
        await self.ws_handler.close_all_connections()

        # Stop the server
        if self.site:
            await self.site.stop()

        if self.runner:
            await self.runner.cleanup()

        self.log('info', 'HTTP server stopped')

    # Cache-control for the dashboard's HTML and JS/CSS. We don't want
    # the browser to silently serve a months-old peripherals.js the next
    # time the operator hits the dashboard after a server upgrade. With
    # this header set, aiohttp still emits an ETag, so unchanged files
    # respond 304 and the actual transfer is tiny — we only pay for the
    # round trip, not the bytes. Files under /api/firmware/ (.uf2/.bin)
    # don't go through these handlers and keep their default caching.
    NO_CACHE_HEADERS = {
        "Cache-Control": "no-cache, must-revalidate",
    }

    async def _handle_index(self, request: web.Request) -> web.Response:
        """Handle requests to root path - serve index.html."""
        index_path = os.path.join(self.web_root, 'index.html')
        if os.path.isfile(index_path):
            return web.FileResponse(index_path, headers=self.NO_CACHE_HEADERS)
        else:
            return web.Response(
                text='SAINT.OS Server - Web interface not found',
                status=404
            )

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Handle status API endpoint (REST fallback)."""
        status = self.state_manager.get_system_status()
        return web.json_response(status)

    async def _handle_static(self, request: web.Request) -> web.Response:
        """Handle static file requests."""
        path = request.match_info.get('path', '')

        # Prevent directory traversal
        if '..' in path:
            return web.Response(text='Forbidden', status=403)

        # Resolve the file path (use normpath instead of realpath to avoid symlink issues)
        file_path = os.path.normpath(os.path.join(self.web_root, path))

        # Security check - ensure file is within web_root
        web_root_norm = os.path.normpath(self.web_root)
        if not file_path.startswith(web_root_norm + os.sep) and file_path != web_root_norm:
            self.log('warning', f'Path outside web root: {path} -> {file_path}')
            return web.Response(text='Forbidden', status=403)

        # Check if file exists
        if not os.path.isfile(file_path):
            self.log('debug', f'File not found: {file_path}')
            return web.Response(text='Not Found', status=404)

        # Get content type
        content_type, _ = mimetypes.guess_type(file_path)
        if content_type is None:
            content_type = 'application/octet-stream'

        self.log('debug', f'Serving: {file_path} as {content_type}')
        return web.FileResponse(file_path, headers=self.NO_CACHE_HEADERS)

    def _get_firmware_info(self, fw_type: str) -> Optional[Dict[str, Any]]:
        """Get firmware info for a specific type."""
        fw_dir = Path(self.firmware_root) / fw_type

        if not fw_dir.is_dir():
            return None

        # Look for info.json or version file
        info_file = fw_dir / 'info.json'
        if info_file.exists():
            try:
                with open(info_file, 'r') as f:
                    info = json.load(f)
                    # Add computed fields
                    info['type'] = fw_type
                    return info
            except Exception as e:
                self.log('error', f'Failed to read firmware info: {e}')

        # Fallback: scan for firmware files
        packages = []
        for ext in ['.zip', '.tar.gz', '.tgz', '.elf', '.AppImage']:
            for f in fw_dir.glob(f'*{ext}'):
                stat = f.stat()
                packages.append({
                    'filename': f.name,
                    'size': stat.st_size,
                    'modified': stat.st_mtime,
                    'checksum': self._calculate_checksum(f),
                })

        if packages:
            # Sort by modified time, newest first
            packages.sort(key=lambda x: x['modified'], reverse=True)
            return {
                'type': fw_type,
                'packages': packages,
                'latest': packages[0]['filename'] if packages else None,
            }

        return None

    def _calculate_checksum(self, file_path: Path) -> str:
        """Calculate SHA256 checksum of a file."""
        sha256 = hashlib.sha256()
        with open(file_path, 'rb') as f:
            for chunk in iter(lambda: f.read(8192), b''):
                sha256.update(chunk)
        return sha256.hexdigest()

    async def _handle_firmware_list(self, request: web.Request) -> web.Response:
        """Handle request for list of available firmware types."""
        result = {
            'firmware_types': [],
            'firmware_root': self.firmware_root,
        }

        for fw_type in self.FIRMWARE_TYPES:
            info = self._get_firmware_info(fw_type)
            if info:
                result['firmware_types'].append(info)

        return web.json_response(result)

    async def _handle_firmware_type_info(self, request: web.Request) -> web.Response:
        """Handle request for specific firmware type info."""
        fw_type = request.match_info.get('fw_type', '')

        if fw_type not in self.FIRMWARE_TYPES:
            return web.json_response(
                {'error': f'Unknown firmware type: {fw_type}'},
                status=400
            )

        info = self._get_firmware_info(fw_type)
        if not info:
            return web.json_response(
                {'error': f'No firmware available for type: {fw_type}'},
                status=404
            )

        return web.json_response(info)

    async def _handle_firmware_download(self, request: web.Request) -> web.Response:
        """Handle firmware file download."""
        fw_type = request.match_info.get('fw_type', '')
        filename = request.match_info.get('filename', '')

        if fw_type not in self.FIRMWARE_TYPES:
            return web.Response(text='Invalid firmware type', status=400)

        # Security: prevent directory traversal
        if '..' in filename or '/' in filename or '\\' in filename:
            return web.Response(text='Forbidden', status=403)

        fw_path = Path(self.firmware_root) / fw_type / filename

        if not fw_path.is_file():
            self.log('warning', f'Firmware not found: {fw_path}')
            return web.Response(text='Not Found', status=404)

        self.log('info', f'Serving firmware: {fw_path}')
        return web.FileResponse(fw_path)

    # ── URDF model routes ───────────────────────────────────────────

    async def _handle_urdf_metadata(self, request: web.Request) -> web.Response:
        meta = self.urdf_store.get_metadata()
        if meta is None:
            return web.json_response({'installed': False})
        return web.json_response({'installed': True, **meta.to_dict()})

    async def _handle_urdf_get(self, request: web.Request) -> web.Response:
        urdf_path = self.urdf_store.get_urdf_path()
        if not urdf_path:
            return web.Response(text='No URDF installed', status=404)
        return web.FileResponse(
            urdf_path,
            headers={
                **self.NO_CACHE_HEADERS,
                'Content-Type': 'application/xml',
            },
        )

    async def _handle_urdf_joints(self, request: web.Request) -> web.Response:
        """Return the list of actuatable joints in the installed URDF.

        Powers the URDF-joint picker in the routing UI's Add Input
        modal. Returns an empty list when no URDF is installed — the
        frontend disables the URDF-joint option in that case.
        """
        joints = self.urdf_store.list_joints()
        return web.json_response({'joints': joints}, headers=self.NO_CACHE_HEADERS)

    async def _handle_urdf_mesh(self, request: web.Request) -> web.Response:
        filename = request.match_info.get('filename', '')
        mesh_path = self.urdf_store.get_mesh_path(filename)
        if not mesh_path:
            return web.Response(text='Mesh not found', status=404)
        return web.FileResponse(mesh_path, headers=self.NO_CACHE_HEADERS)

    async def _handle_urdf_upload(self, request: web.Request) -> web.Response:
        """Accept a multipart upload with a single file part.

        The part's filename determines treatment:
          * ``.zip`` — extracted as a URDF bundle (URDF + optional meshes/)
          * ``.urdf`` / ``.xacro`` — installed as a single primitive-only model
        """
        if request.content_length is not None and request.content_length > MAX_URDF_UPLOAD_BYTES:
            return web.json_response(
                {'error': f'Upload exceeds {MAX_URDF_UPLOAD_BYTES} byte limit'},
                status=413,
            )

        try:
            reader = await request.multipart()
        except Exception as e:
            return web.json_response(
                {'error': f'Expected multipart upload: {e}'},
                status=400,
            )

        field = await reader.next()
        if field is None:
            return web.json_response({'error': 'No upload field'}, status=400)

        filename = (field.filename or 'upload.bin').strip()
        # Bound the read so a giant multipart can't drain memory.
        buf = bytearray()
        while True:
            chunk = await field.read_chunk(size=64 * 1024)
            if not chunk:
                break
            buf.extend(chunk)
            if len(buf) > MAX_URDF_UPLOAD_BYTES:
                return web.json_response(
                    {'error': f'Upload exceeds {MAX_URDF_UPLOAD_BYTES} byte limit'},
                    status=413,
                )

        try:
            if filename.lower().endswith('.zip'):
                meta = self.urdf_store.install_from_zip(bytes(buf), filename)
            else:
                meta = self.urdf_store.install_from_urdf(bytes(buf), filename)
        except URDFStoreError as e:
            return web.json_response({'error': str(e)}, status=400)
        except Exception as e:
            self.log('error', f'URDF install failed: {e}')
            return web.json_response({'error': f'Install failed: {e}'}, status=500)

        return web.json_response({'installed': True, **meta.to_dict()})

    async def _handle_urdf_delete(self, request: web.Request) -> web.Response:
        removed = self.urdf_store.delete()
        return web.json_response({'removed': removed})

    # ── Animation import ────────────────────────────────────────────

    async def _handle_maestro_import(self, request: web.Request) -> web.Response:
        """Parse a Pololu Maestro `.xml` save file into a candidate
        Animation. The response is NOT persisted — the frontend lets
        the operator rename tracks first, then sends a normal
        ``save_animation`` WS action.
        """
        try:
            reader = await request.multipart()
        except Exception as e:
            return web.json_response(
                {'error': f'Expected multipart upload: {e}'}, status=400)

        sequence_name = None
        xml_bytes: Optional[bytes] = None
        while True:
            field = await reader.next()
            if field is None:
                break
            if field.name == 'file':
                buf = bytearray()
                while True:
                    chunk = await field.read_chunk(size=64 * 1024)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    if len(buf) > MAX_URDF_UPLOAD_BYTES:
                        return web.json_response(
                            {'error': 'Upload too large'}, status=413)
                xml_bytes = bytes(buf)
            elif field.name == 'sequence':
                sequence_name = (await field.text()).strip() or None

        if not xml_bytes:
            return web.json_response({'error': 'Missing file field'}, status=400)

        # Defer the import so the parser only loads when needed.
        from saint_server.animation.maestro_import import (
            parse_maestro_xml, list_sequences,
        )
        try:
            if sequence_name is None:
                # Always return the available sequence list so the UI
                # can offer a picker on multi-sequence files even when
                # the first sequence is the default.
                sequences = list_sequences(xml_bytes)
            else:
                sequences = None
            result = parse_maestro_xml(xml_bytes, sequence_name=sequence_name)
        except ValueError as e:
            return web.json_response({'error': str(e)}, status=400)
        except Exception as e:
            self.log('error', f'Maestro import failed: {e}')
            return web.json_response({'error': f'Import failed: {e}'}, status=500)

        body = {
            'animation': result.animation.to_dict(),
            'channels': [
                {
                    'index': c.index,
                    'min_value': c.min_value,
                    'max_value': c.max_value,
                    'keyframe_count': c.keyframe_count,
                }
                for c in result.channels
            ],
            'sequence_name': result.sequence_name,
            'frame_count': result.raw_frame_count,
        }
        if sequences is not None:
            body['sequences'] = sequences
        return web.json_response(body)
