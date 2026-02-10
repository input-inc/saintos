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

from saint_server.webserver.state_manager import StateManager
from saint_server.webserver.websocket_handler import WebSocketHandler


class WebServer:
    """HTTP server for static files and WebSocket endpoint."""

    # Supported firmware types
    FIRMWARE_TYPES = ['rp2040', 'rpi5', 'teensy41']

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

        # Add routes
        self.app.router.add_get('/api/ws', self.ws_handler.handle_connection)
        self.app.router.add_get('/api/status', self._handle_status)

        # Firmware routes
        self.app.router.add_get('/api/firmware', self._handle_firmware_list)
        self.app.router.add_get('/api/firmware/{fw_type}', self._handle_firmware_type_info)
        self.app.router.add_get('/api/firmware/{fw_type}/{filename}', self._handle_firmware_download)

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

    async def _handle_index(self, request: web.Request) -> web.Response:
        """Handle requests to root path - serve index.html."""
        index_path = os.path.join(self.web_root, 'index.html')
        if os.path.isfile(index_path):
            return web.FileResponse(index_path)
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
        return web.FileResponse(file_path)

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
        for ext in ['.zip', '.tar.gz', '.tgz', '.elf']:
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
