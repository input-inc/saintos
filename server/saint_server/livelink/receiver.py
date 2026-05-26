"""
LiveLink Face UDP Receiver

Receives facial motion capture data from Epic Games Live Link Face iOS app.
The app sends UDP packets containing ARKit blend shape data.

Protocol format (Live Link Face app):
- 4 bytes: Packet type (usually 0x00000001 for face data)
- 1 byte: Subject name length
- N bytes: Subject name (UTF-8)
- 4 bytes: Frame number (uint32)
- 4 bytes: Sub-frame (float)
- 4 bytes: Denominator (float)
- 4 bytes: Blend shape count (uint32)
- N*4 bytes: Blend shape values (float32 array)
"""

import asyncio
import math
import struct
import time
from dataclasses import dataclass
from typing import Optional, Callable, List, Tuple, Any

from saint_server.livelink.blend_shapes import BlendShapes, EXTENDED_BLEND_SHAPE_COUNT


@dataclass
class LiveLinkConfig:
    """Configuration for LiveLink receiver."""
    port: int = 11111  # Default Live Link Face port
    bind_address: str = "0.0.0.0"
    timeout: float = 5.0  # Connection timeout in seconds


class LiveLinkProtocol(asyncio.DatagramProtocol):
    """UDP protocol handler for Live Link Face data."""

    def __init__(self, callback: Callable[[BlendShapes], None], logger=None,
                 on_connect: Optional[Callable[[str, str], None]] = None,
                 on_disconnect: Optional[Callable[[str], None]] = None):
        self.callback = callback
        self.logger = logger
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.transport = None
        self.last_packet_time = 0.0
        self.packet_count = 0
        self.error_count = 0

        # Connection tracking
        self._connected = False
        self._current_source: Optional[Tuple[str, int]] = None
        self._current_subject: Optional[str] = None

    def connection_made(self, transport):
        self.transport = transport
        if self.logger:
            self.logger.info("LiveLink UDP receiver ready")

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        """Handle incoming UDP packet from Live Link Face app."""
        now = time.time()
        self.last_packet_time = now
        self.packet_count += 1

        # Log first few packets for debugging
        if self.logger and self.packet_count <= 3:
            self.logger.info(
                f"LiveLink UDP packet #{self.packet_count} from {addr[0]}:{addr[1]}, "
                f"size={len(data)} bytes, header={data[:8].hex() if len(data) >= 8 else data.hex()}"
            )

        try:
            blend_shapes = self._parse_packet(data)
            if blend_shapes:
                # Check if this is a new connection or source change
                if not self._connected or self._current_source != addr:
                    self._handle_new_connection(addr, blend_shapes.subject_name)
                elif self._current_subject != blend_shapes.subject_name:
                    # Subject name changed - just update tracking
                    self._current_subject = blend_shapes.subject_name

                self.callback(blend_shapes)
        except Exception as e:
            self.error_count += 1
            if self.logger and self.error_count <= 10:
                self.logger.warning(f"LiveLink packet parse error: {e}")

    def _handle_new_connection(self, addr: Tuple[str, int], subject_name: str):
        """Handle a new LiveLink connection."""
        self._connected = True
        self._current_source = addr
        self._current_subject = subject_name

        if self.logger:
            self.logger.info(
                f"LiveLink connection established from {addr[0]}:{addr[1]} "
                f"(subject: {subject_name})"
            )

        if self.on_connect:
            try:
                self.on_connect(f"{addr[0]}:{addr[1]}", subject_name)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"LiveLink on_connect callback error: {e}")

    def check_timeout(self, timeout: float) -> bool:
        """Check if connection has timed out. Returns True if disconnected."""
        if not self._connected:
            return False

        if time.time() - self.last_packet_time > timeout:
            self._handle_disconnect()
            return True
        return False

    def _handle_disconnect(self):
        """Handle LiveLink disconnection."""
        if not self._connected:
            return

        source_str = f"{self._current_source[0]}:{self._current_source[1]}" if self._current_source else "unknown"

        if self.logger:
            self.logger.info(
                f"LiveLink connection lost from {source_str} "
                f"(subject: {self._current_subject}, packets: {self.packet_count})"
            )

        if self.on_disconnect:
            try:
                self.on_disconnect(source_str)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"LiveLink on_disconnect callback error: {e}")

        self._connected = False
        self._current_source = None
        self._current_subject = None

    def _parse_packet(self, data: bytes) -> Optional[BlendShapes]:
        """
        Parse Live Link Face UDP packet.

        Protocol format (Live Link Face iOS app):
        - 1 byte: Version (1 or 6)
        - 36 bytes: Device UUID (as ASCII string, no null terminator in v1)
        - 1 byte: Subject name length
        - N bytes: Subject name
        - 4 bytes: Frame number (uint32)
        - 4 bytes: Sub-frame numerator (uint32)
        - 4 bytes: Frame rate numerator (uint32)
        - 4 bytes: Frame rate denominator (uint32)
        - 1 byte: Blend shape count
        - N*4 bytes: Blend shape values (float32)
        """
        if len(data) < 50:  # Minimum reasonable packet size
            if self.logger and self.packet_count <= 5:
                self.logger.debug(f"LiveLink packet too short: {len(data)} bytes")
            return None

        offset = 0

        # Version (1 byte)
        version = data[offset]
        offset += 1

        if version not in (1, 6):
            if self.logger and self.packet_count <= 5:
                self.logger.warning(f"LiveLink unknown version {version}, attempting to parse anyway")

        # Device UUID (36 bytes for v1, 37 bytes for v6 with null terminator)
        uuid_length = 36 if version == 1 else 37
        if offset + uuid_length > len(data):
            return None
        device_uuid = data[offset:offset + uuid_length].decode('utf-8', errors='ignore').rstrip('\x00')
        offset += uuid_length

        # Subject name length (1 byte)
        if offset >= len(data):
            return None
        name_length = data[offset]
        offset += 1

        if name_length > 100 or offset + name_length > len(data):
            if self.logger and self.packet_count <= 5:
                self.logger.debug(f"LiveLink invalid name length: {name_length}")
            return None

        # Subject name
        subject_name = data[offset:offset + name_length].decode('utf-8', errors='ignore')
        offset += name_length

        # Frame number (4 bytes)
        if offset + 4 > len(data):
            return None
        frame_number = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        # Sub-frame numerator (4 bytes)
        if offset + 4 > len(data):
            return None
        sub_frame = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        # Frame rate numerator (4 bytes)
        if offset + 4 > len(data):
            return None
        fps_num = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        # Frame rate denominator (4 bytes)
        if offset + 4 > len(data):
            return None
        fps_denom = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        # Blend shape count (1 byte)
        if offset >= len(data):
            return None
        blend_count = data[offset]
        offset += 1

        # Validate blend shape count (ARKit has 52, Live Link Face sends 61)
        if blend_count > 70:  # Sanity check
            if self.logger and self.packet_count <= 5:
                self.logger.debug(f"LiveLink blend count too high: {blend_count}")
            return None

        # Blend shape values (float32 each)
        values = []
        for i in range(min(blend_count, 61)):  # Live Link Face sends up to 61
            if offset + 4 > len(data):
                break
            value = struct.unpack_from('<f', data, offset)[0]
            offset += 4
            # Sanitize: replace NaN/Inf with 0, clamp to reasonable range
            if math.isnan(value) or math.isinf(value):
                value = 0.0
            else:
                value = max(-1.0, min(2.0, value))  # Clamp to reasonable range
            values.append(value)

        # Log first successful parse
        if self.logger and self.packet_count <= 3:
            self.logger.info(
                f"LiveLink packet parsed: version={version}, subject='{subject_name}', "
                f"frame={frame_number}, fps={fps_num}/{fps_denom}, "
                f"blend_count={blend_count}, values_read={len(values)}"
            )

        # Calculate timestamp from frame info
        timestamp = time.time()

        # Use all 61 values (52 ARKit blend shapes + 9 head/eye rotation values)
        return BlendShapes.from_array(
            values=values[:EXTENDED_BLEND_SHAPE_COUNT],
            subject_name=subject_name if subject_name else device_uuid[:8],
            timestamp=timestamp
        )

    def error_received(self, exc):
        if self.logger:
            self.logger.error(f"LiveLink UDP error: {exc}")


class LiveLinkReceiver:
    """
    LiveLink Face receiver service.

    Listens for UDP packets from the Live Link Face iOS app and
    converts them to BlendShapes objects.
    """

    def __init__(self, config: Optional[LiveLinkConfig] = None, logger=None):
        self.config = config or LiveLinkConfig()
        self.logger = logger
        self.transport = None
        self.protocol = None
        self._running = False
        self._callbacks: List[Callable[[BlendShapes], None]] = []
        self._last_blend_shapes: Optional[BlendShapes] = None
        self._timeout_check_task: Optional[asyncio.Task] = None

        # Connection event callbacks
        self._on_connect_callbacks: List[Callable[[str, str], None]] = []
        self._on_disconnect_callbacks: List[Callable[[str], None]] = []

    def add_callback(self, callback: Callable[[BlendShapes], None]):
        """Add a callback to be called when new blend shape data arrives."""
        self._callbacks.append(callback)

    def remove_callback(self, callback: Callable[[BlendShapes], None]):
        """Remove a callback."""
        if callback in self._callbacks:
            self._callbacks.remove(callback)

    def on_connect(self, callback: Callable[[str, str], None]):
        """Add callback for when a LiveLink source connects. Args: (source_addr, subject_name)"""
        self._on_connect_callbacks.append(callback)

    def on_disconnect(self, callback: Callable[[str], None]):
        """Add callback for when a LiveLink source disconnects. Args: (source_addr)"""
        self._on_disconnect_callbacks.append(callback)

    def _on_data(self, blend_shapes: BlendShapes):
        """Internal callback when data is received."""
        self._last_blend_shapes = blend_shapes

        # Call all registered callbacks
        for callback in self._callbacks:
            try:
                callback(blend_shapes)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"LiveLink callback error: {e}")

    def _handle_connect(self, source_addr: str, subject_name: str):
        """Internal callback when a LiveLink source connects."""
        for callback in self._on_connect_callbacks:
            try:
                callback(source_addr, subject_name)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"LiveLink on_connect callback error: {e}")

    def _handle_disconnect(self, source_addr: str):
        """Internal callback when a LiveLink source disconnects."""
        for callback in self._on_disconnect_callbacks:
            try:
                callback(source_addr)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"LiveLink on_disconnect callback error: {e}")

    async def _timeout_check_loop(self):
        """Periodically check for connection timeout."""
        while self._running:
            try:
                await asyncio.sleep(1.0)  # Check every second
                if self.protocol:
                    self.protocol.check_timeout(self.config.timeout)
            except asyncio.CancelledError:
                break
            except Exception as e:
                if self.logger:
                    self.logger.error(f"LiveLink timeout check error: {e}")

    async def start(self, loop: Optional[asyncio.AbstractEventLoop] = None):
        """Start the LiveLink receiver."""
        if self._running:
            return

        loop = loop or asyncio.get_event_loop()

        if self.logger:
            self.logger.info(
                f"Starting LiveLink receiver on {self.config.bind_address}:{self.config.port}"
            )

        self.transport, self.protocol = await loop.create_datagram_endpoint(
            lambda: LiveLinkProtocol(
                self._on_data,
                self.logger,
                on_connect=self._handle_connect,
                on_disconnect=self._handle_disconnect
            ),
            local_addr=(self.config.bind_address, self.config.port)
        )

        self._running = True

        # Start timeout check task
        self._timeout_check_task = asyncio.create_task(self._timeout_check_loop())

        if self.logger:
            self.logger.info(f"LiveLink receiver started on port {self.config.port}")

    async def stop(self):
        """Stop the LiveLink receiver."""
        if not self._running:
            return

        self._running = False

        # Cancel timeout check task
        if self._timeout_check_task:
            self._timeout_check_task.cancel()
            try:
                await self._timeout_check_task
            except asyncio.CancelledError:
                pass
            self._timeout_check_task = None

        if self.transport:
            self.transport.close()
            self.transport = None

        if self.logger:
            self.logger.info("LiveLink receiver stopped")

    @property
    def is_running(self) -> bool:
        """Check if receiver is running."""
        return self._running

    @property
    def is_connected(self) -> bool:
        """Check if we're receiving data (packet within timeout period)."""
        if not self._running or not self.protocol:
            return False
        return (time.time() - self.protocol.last_packet_time) < self.config.timeout

    @property
    def last_blend_shapes(self) -> Optional[BlendShapes]:
        """Get the most recent blend shapes data."""
        return self._last_blend_shapes

    @property
    def stats(self) -> dict:
        """Get receiver statistics."""
        if not self.protocol:
            return {
                "running": False,
                "connected": False,
                "packet_count": 0,
                "error_count": 0,
                "last_packet_time": 0,
            }

        return {
            "running": self._running,
            "connected": self.is_connected,
            "packet_count": self.protocol.packet_count,
            "error_count": self.protocol.error_count,
            "last_packet_time": self.protocol.last_packet_time,
            "port": self.config.port,
        }
