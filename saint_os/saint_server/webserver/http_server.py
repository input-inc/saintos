"""
SAINT.OS HTTP Server

Serves static files and provides WebSocket endpoint for the admin interface.
"""

import mimetypes
import os
from typing import Optional

from aiohttp import web

from saint_server.webserver.state_manager import StateManager
from saint_server.webserver.websocket_handler import WebSocketHandler


class WebServer:
    """HTTP server for static files and WebSocket endpoint."""

    def __init__(
        self,
        web_root: str,
        port: int = 80,
        host: str = '0.0.0.0',
        server_name: str = 'SAINT-01',
        logger=None,
        state_manager: Optional[StateManager] = None
    ):
        self.web_root = web_root
        self.port = port
        self.host = host
        self.server_name = server_name
        self.logger = logger

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
