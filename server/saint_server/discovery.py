"""
SAINT.OS Node Discovery Service

UDP broadcast listener that helps firmware nodes find the server.
Nodes broadcast a discovery request, server responds with its IP and agent port.

Protocol:
  Request:  "SAINT?" (broadcast to UDP port 8889)
  Response: "SAINT!<ip>:<agent_port>" (e.g., "SAINT!192.168.0.104:8888")
"""

import asyncio
import socket
import struct
import logging
from typing import Optional, Callable

# Discovery protocol constants
DISCOVERY_PORT = 8889
DISCOVERY_REQUEST = b"SAINT?"
DISCOVERY_RESPONSE_PREFIX = b"SAINT!"


class DiscoveryService:
    """
    UDP discovery service for SAINT nodes.

    Listens for broadcast discovery requests and responds with
    the server's IP address and micro-ROS agent port.
    """

    def __init__(
        self,
        agent_port: int = 8888,
        discovery_port: int = DISCOVERY_PORT,
        logger: Optional[logging.Logger] = None
    ):
        self.agent_port = agent_port
        self.discovery_port = discovery_port
        self.logger = logger
        self._transport: Optional[asyncio.DatagramTransport] = None
        self._protocol: Optional['DiscoveryProtocol'] = None
        self._running = False

    def log(self, level: str, message: str):
        """Log a message."""
        if self.logger:
            getattr(self.logger, level)(message)
        else:
            print(f"[Discovery] {message}")

    async def start(self):
        """Start the discovery service."""
        if self._running:
            return

        loop = asyncio.get_event_loop()

        try:
            # Create UDP socket that can receive broadcasts
            self._transport, self._protocol = await loop.create_datagram_endpoint(
                lambda: DiscoveryProtocol(self),
                local_addr=('0.0.0.0', self.discovery_port),
                family=socket.AF_INET,
                allow_broadcast=True
            )

            # Enable broadcast on the socket
            sock = self._transport.get_extra_info('socket')
            if sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            self._running = True
            self.log('info', f'Discovery service listening on UDP port {self.discovery_port}')

        except Exception as e:
            self.log('error', f'Failed to start discovery service: {e}')
            raise

    async def stop(self):
        """Stop the discovery service."""
        if self._transport:
            self._transport.close()
            self._transport = None
            self._protocol = None
        self._running = False
        self.log('info', 'Discovery service stopped')

    def get_local_ip(self, dest_addr: str = None) -> str:
        """
        Get the local IP address that would be used to reach a destination.

        If dest_addr is provided, returns the IP of the interface that would
        route to that address. Otherwise returns a best-guess local IP.
        """
        try:
            # Create a dummy socket to determine which interface would be used
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                # Use the requester's network or a public IP to determine route
                target = dest_addr or '8.8.8.8'
                s.connect((target, 80))
                return s.getsockname()[0]
        except Exception:
            # Fallback: try to get any non-localhost IP
            try:
                hostname = socket.gethostname()
                return socket.gethostbyname(hostname)
            except Exception:
                return '127.0.0.1'

    def handle_discovery_request(self, addr: tuple) -> bytes:
        """
        Handle a discovery request and return the response.

        Args:
            addr: (ip, port) of the requester

        Returns:
            Response bytes to send back
        """
        requester_ip = addr[0]

        # Get our IP address on the same network as the requester
        our_ip = self.get_local_ip(requester_ip)

        self.log('info', f'Discovery request from {requester_ip}, responding with {our_ip}:{self.agent_port}')

        # Build response: "SAINT!<ip>:<port>"
        response = f"{DISCOVERY_RESPONSE_PREFIX.decode()}{our_ip}:{self.agent_port}"
        return response.encode()


class DiscoveryProtocol(asyncio.DatagramProtocol):
    """Asyncio protocol for handling discovery UDP packets."""

    def __init__(self, service: DiscoveryService):
        self.service = service
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport

    def datagram_received(self, data: bytes, addr: tuple):
        """Handle incoming UDP datagram."""
        # Check if it's a discovery request
        if data.strip() == DISCOVERY_REQUEST:
            try:
                response = self.service.handle_discovery_request(addr)
                self.transport.sendto(response, addr)
            except Exception as e:
                self.service.log('error', f'Error handling discovery request: {e}')
        else:
            # Log unknown packets for debugging
            self.service.log('debug', f'Unknown packet from {addr}: {data[:50]}')

    def error_received(self, exc: Exception):
        self.service.log('error', f'Discovery protocol error: {exc}')

    def connection_lost(self, exc: Optional[Exception]):
        if exc:
            self.service.log('warn', f'Discovery connection lost: {exc}')
