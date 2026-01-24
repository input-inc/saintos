"""
SAINT.OS HTTP Server

Serves static files and provides WebSocket endpoint for the admin interface.
"""

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
        logger=None
    ):
        self.web_root = web_root
        self.port = port
        self.host = host
        self.server_name = server_name
        self.logger = logger

        # Components
        self.state_manager = StateManager(server_name=server_name)
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

        # Add routes
        self.app.router.add_get('/api/ws', self.ws_handler.handle_connection)
        self.app.router.add_get('/api/status', self._handle_status)

        # Serve index.html for root
        self.app.router.add_get('/', self._handle_index)

        # Serve static files
        if os.path.isdir(self.web_root):
            self.app.router.add_static('/', self.web_root, show_index=False)
            self.log('info', f'Serving static files from: {self.web_root}')
        else:
            self.log('warn', f'Web root directory not found: {self.web_root}')

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
