"""
SAINT.OS Main Server Node

The central ROS2 node that coordinates all server functionality:
- Node adoption and management
- Firmware distribution
- WebSocket gateway
- LiveLink receiver
- RC receiver
- Input/Output routing
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Two server→node topics split by delivery semantics:
#
#   /saint/nodes/<id>/control  →  CONTROL_QOS  (BEST_EFFORT, depth=1)
#     Streaming peripheral writes (set_pin, set_channel) from joystick
#     and the routing evaluator. Freshest-wins; a dropped UDP packet
#     just means the next sample (50 Hz) supersedes it. Avoids the
#     deadstick-queues-behind-stale-values pathology of RELIABLE.
#
#   /saint/nodes/<id>/command  →  COMMAND_QOS  (RELIABLE, depth=8)
#     Operator one-shots that MUST land: firmware_update, factory_reset,
#     restart, identify, estop, clear_estop, roboclaw_debug. RELIABLE
#     so a single dropped packet doesn't silently lose an estop or an
#     OTA trigger. Depth 8 is generous — covers operator key-mashing
#     during a node's brief disconnect.
#
# Compatibility note: nodes flashed BEFORE the topic split don't
# subscribe to /command, so one-shots sent here silently no-op until
# the node is re-flashed once manually (BOOTSEL/USB) with current
# firmware. After that, OTA self-update keeps the firmware current.
CONTROL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=8,
    durability=QoSDurabilityPolicy.VOLATILE,
)
# /saint/nodes/<id>/log
#   BEST_EFFORT to match the firmware writer. RELIABLE here used to
#   wedge the micro-XRCE-DDS output stream once a saint_log burst
#   filled its 4-slot history (the writer waited indefinitely for
#   ACKs that never came back during the same callback). Logs are
#   diagnostic, depth=20 keeps the recent window for the per-node
#   Logs tab. The sync-ack flow rides this topic — a dropped ack
#   just means the operator re-clicks Sync.
LOG_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=20,
    durability=QoSDurabilityPolicy.VOLATILE,
)
# /saint/nodes/announce  (shared, all nodes publish here)
# /saint/nodes/<id>/state (per-node, 10 Hz)
#   BEST_EFFORT for telemetry-direction topics. The Teensy's
#   micro-XRCE-DDS client wedges RELIABLE publishers across the
#   board after the first few publishes (same root cause as the
#   /log wedge — output stream history fills with unacked samples
#   inside callbacks before the agent has a chance to ACK). A
#   BEST_EFFORT subscriber accepts both RELIABLE and BEST_EFFORT
#   publishers, so the existing RP2040 nodes (whose RELIABLE
#   publishers DO work) keep flowing while the Teensy's
#   BEST_EFFORT publishers also match. Sync ACK still rides
#   /announce — losing one sample just delays the pill flip until
#   the next 1 Hz tick.
TELEMETRY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Use std_msgs/String for node announcements (matches firmware)
from std_msgs.msg import String

import asyncio
import os
import signal
import threading
from typing import Optional, Dict, Any, List

from saint_server.webserver.state_manager import (
    StateManager, HOST_CONTROLLER_NODE_ID)
from saint_server.host_peripherals import HostPeripheralManager
from saint_server.peripheral_logger import PeripheralLogger
from saint_server.roles import RoleManager
from saint_server.livelink import LiveLinkReceiver, LiveLinkRouter
from saint_server.livelink.receiver import LiveLinkConfig
from saint_server.livelink.router import LiveLinkRoute, OutputMapping, MappingType
from saint_server.discovery import DiscoveryService


# Known peripheral prefixes the firmware uses in log lines today. The
# match is case-insensitive and only fires when the prefix is followed
# by a colon, so generic "foo: bar" log lines from non-peripheral
# subsystems don't get mis-tagged. List is intentionally small — we'd
# rather under-tag than mis-tag.
_PERIPHERAL_LOG_PREFIXES = (
    "roboclaw", "fas100", "jbd", "neopixel", "maestro", "syren", "tic",
    "tmc2208",
)


def _extract_peripheral_from_text(text: str) -> Optional[str]:
    """Best-effort peripheral name from a node log line. Looks for a
    leading known-prefix followed by ``:`` (case-insensitive). Returns
    the canonical (lowercase) name or ``None`` when no prefix matches.
    Cheap — runs once per inbound log frame, only when the firmware
    didn't tag the message itself."""
    if not text:
        return None
    head = text.lstrip()
    for prefix in _PERIPHERAL_LOG_PREFIXES:
        if head[:len(prefix)].lower() == prefix and len(head) > len(prefix):
            sep = head[len(prefix)]
            if sep in (":", "_", "-"):
                return prefix
    return None


class SaintServerNode(Node):
    """Main SAINT.OS server node."""

    def __init__(self):
        super().__init__('saint_server')

        # Declare parameters
        self.declare_parameter('server_name', 'SAINT-01')
        self.declare_parameter('web_port', 80)
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('livelink_enabled', True)
        self.declare_parameter('livelink_discovery_port', 54321)
        self.declare_parameter('livelink_data_port', 54322)
        self.declare_parameter('rc_enabled', False)
        self.declare_parameter('rc_protocol', 'sbus')
        self.declare_parameter('discovery_enabled', True)
        self.declare_parameter('discovery_port', 8889)
        self.declare_parameter('agent_port', 8888)

        # Get parameters
        self.server_name = self.get_parameter('server_name').value
        self.web_port = self.get_parameter('web_port').value
        self.websocket_port = self.get_parameter('websocket_port').value

        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Async infrastructure
        self._async_loop: Optional[asyncio.AbstractEventLoop] = None
        self._async_thread: Optional[threading.Thread] = None
        self._shutdown_event: Optional[asyncio.Event] = None
        # Owns in-process BLE drivers for the host_controller virtual
        # node (currently: JBD BMSes). Created once the async loop is
        # up — see _run_async_loop.
        self._host_peripheral_manager: Optional[HostPeripheralManager] = None

        # State manager - shared with web server
        self.state_manager = StateManager(
            server_name=self.server_name,
            logger=self.get_logger()
        )

        # File-backed logging tree under $SAINT_LOG_DIR (default
        # /var/log/saint-os):
        #   saint-server.log                      — every activity event
        #   nodes/<node_id>.log                   — per-node activity events
        #   peripherals/<node_id>__<periph>.log   — per-peripheral telemetry (opt-in)
        #
        # SAINT_LOG_DIR is set by the systemd unit and pre-created +
        # chowned to the service user by install.sh, so production
        # paths are writable. We test the target directory itself
        # (after ensuring it exists) — earlier versions checked the
        # PARENT's writability, which is /var/log for the production
        # path and is root-owned, so the check always failed and the
        # logger silently fell through to /tmp/saint-os (which is
        # isolated by PrivateTmp=true and thus invisible from outside
        # the service).
        #
        # If the configured dir isn't writable we fall back to
        # /tmp/saint-os so the server still starts; the journal log
        # is the audit trail when the file sinks are unavailable.
        base_dir = os.environ.get("SAINT_LOG_DIR") or "/var/log/saint-os"
        try:
            os.makedirs(base_dir, exist_ok=True)
            if not os.access(base_dir, os.W_OK):
                raise PermissionError(f"{base_dir} not writable")
        except (OSError, PermissionError) as e:
            fallback = "/tmp/saint-os"
            self.get_logger().warn(
                f"saint-os log dir: {base_dir} unusable ({e}); "
                f"falling back to {fallback}"
            )
            os.makedirs(fallback, exist_ok=True)
            base_dir = fallback

        # Server-wide activity file — one line per _log_activity call,
        # rotating daily at midnight. Plain-text format so an operator
        # can tail it without a parser.
        import logging as _logging
        from saint_server.file_log import make_daily_handler, PerKeyLogger
        _activity_fmt = _logging.Formatter(
            "%(asctime)s %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        )
        self._server_activity_logger = _logging.getLogger("saint_os.activity")
        self._server_activity_logger.setLevel(_logging.DEBUG)
        self._server_activity_logger.propagate = False
        # Idempotent in case __init__ runs twice (tests, reloads).
        for _h in list(self._server_activity_logger.handlers):
            self._server_activity_logger.removeHandler(_h)
        self._server_activity_logger.addHandler(make_daily_handler(
            os.path.join(base_dir, "saint-server.log"),
            formatter=_activity_fmt,
        ))

        # Per-node activity files. PerKeyLogger lazily opens
        # nodes/<node_id>.log on the first event for that node, each
        # with its own daily-rotating handler (with rotate-on-restart).
        self._per_node_activity_logger = PerKeyLogger(
            base_dir=os.path.join(base_dir, "nodes"),
            logger_name_prefix="saint_os.nodes",
            formatter=_activity_fmt,
        )

        # Hook them into the state manager so every _log_activity call
        # also writes to the file sinks.
        self.state_manager.set_activity_file_logger(
            server_logger=self._server_activity_logger,
            per_node_logger=self._per_node_activity_logger,
        )

        # Per-peripheral telemetry logger — opt-in per peripheral,
        # writes NDJSON to ${SAINT_LOG_DIR}/peripherals/<node>__<id>.log
        # with daily rotation.
        peripherals_dir = os.path.join(base_dir, "peripherals")
        os.makedirs(peripherals_dir, exist_ok=True)
        self.peripheral_logger = PeripheralLogger(
            log_dir=peripherals_dir,
            logger=self.get_logger(),
        )
        # Hand it to the state manager so update_pin_actual can feed
        # samples and so the WS log_enabled toggle finds it.
        self.state_manager.set_peripheral_logger(self.peripheral_logger)

        # Apply the configured log level. Default in config is WARNING
        # so per-tick INFO from the routing evaluator and ROS bridge
        # stays off the file handlers + rclpy console at 50 Hz —
        # operators bump it from the Logs tab when debugging. Lives
        # here (after file handlers are attached but before any heavy
        # logging starts) so the level takes hold for everything we
        # subsequently emit.
        try:
            from saint_server.config import get_config
            from saint_server.log_level import apply_log_level
            applied = apply_log_level(get_config().logging.level, ros_node=self)
            self.get_logger().info(f'Log level set to {applied}')
        except Exception as e:
            self.get_logger().warn(f'Failed to apply configured log level: {e}')

        # Web server components (set during start_async_services)
        self.web_server = None

        # Role manager
        self.role_manager = RoleManager(logger=self.get_logger())

        # LiveLink components
        self.livelink_receiver: Optional[LiveLinkReceiver] = None
        self.livelink_router: Optional[LiveLinkRouter] = None
        self._livelink_enabled = self.get_parameter('livelink_enabled').value

        # Discovery service (helps nodes find the server)
        self.discovery_service: Optional[DiscoveryService] = None
        self._discovery_enabled = self.get_parameter('discovery_enabled').value
        self._discovery_port = self.get_parameter('discovery_port').value
        self._agent_port = self.get_parameter('agent_port').value

        # ROS Bridge for WebSocket-ROS communication
        self.ros_bridge = None

        # Dynamic publishers/subscribers for node communication
        self._node_config_pubs: Dict[str, Any] = {}  # node_id -> publisher
        self._node_caps_subs: Dict[str, Any] = {}    # node_id -> subscription
        self._node_control_pubs: Dict[str, Any] = {} # node_id -> control publisher (streaming, BEST_EFFORT)
        self._node_command_pubs: Dict[str, Any] = {} # node_id -> command publisher (one-shots, RELIABLE)
        self._node_state_subs: Dict[str, Any] = {}   # node_id -> state subscription
        self._node_log_subs: Dict[str, Any] = {}     # node_id -> log subscription
        self._node_ble_scan_subs: Dict[str, Any] = {}  # node_id -> ble_scan_results subscription
        self._node_update_progress_subs: Dict[str, Any] = {}  # node_id -> update_progress subscription
        self._node_soundboard_subs: Dict[str, Any] = {}  # node_id -> [soundboard_fs/devices/result subs]

        # Sync-ACK signal carried in /announce. Maps node_id to the most
        # recent value of last_config_save_{ok,fail}_ms we've already
        # acked on. When a fresh announce shows a larger value than
        # what's here, we treat the increase as the firmware's "I just
        # saved config" handshake and flip the UI's Sync pill. Used to
        # be sourced from /log, but post-boot /log frames historically
        # dropped on RP2040 (the underlying empty-payload bug is now
        # fixed in saint_log_emit_ros) — /announce is the independent
        # writer that demonstrably keeps publishing through any /log
        # regression, so the sync handshake rides it.
        self._node_last_save_ok_ms: Dict[str, int] = {}
        self._node_last_save_fail_ms: Dict[str, int] = {}

        # Initialize components
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self._init_timers()

        self.get_logger().info(f'SAINT.OS Server "{self.server_name}" initialized')

    def _init_publishers(self):
        """Initialize ROS2 publishers."""
        # System status publisher
        # self.status_pub = self.create_publisher(SystemStatus, '/saint/system/status', 10)
        pass

    def _init_subscribers(self):
        """Initialize ROS2 subscribers."""
        # Node announcements (using std_msgs/String with JSON payload).
        # TELEMETRY_QOS = BEST_EFFORT so it matches both RELIABLE
        # (RP2040 today) and BEST_EFFORT (Teensy after the wedge fix)
        # publishers — see the TELEMETRY_QOS comment above.
        self.announcement_sub = self.create_subscription(
            String,
            '/saint/nodes/announce',
            self._on_node_announcement,
            TELEMETRY_QOS,
            callback_group=self.callback_group
        )
        self.get_logger().info('Subscribed to /saint/nodes/announce')

    def _init_services(self):
        """Initialize ROS2 services."""
        # Firmware services
        # self.firmware_check_srv = self.create_service(
        #     FirmwareCheck,
        #     '/saint/firmware/check',
        #     self._handle_firmware_check
        # )
        pass

    def _init_timers(self):
        """Initialize periodic timers."""
        # Status broadcast timer (also checks for node timeouts)
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self._periodic_update,
            callback_group=self.callback_group
        )

        # Cleanup timer for stale offline nodes (every 30 seconds)
        self.cleanup_timer = self.create_timer(
            30.0,
            self._cleanup_stale_nodes,
            callback_group=self.callback_group
        )

    def _on_node_announcement(self, msg: String):
        """Handle node announcement from micro-ROS nodes."""
        raw = msg.data
        try:
            # Log announcement for debugging
            self.get_logger().debug(f'Received announcement ({len(raw)} bytes): {raw[:100]}...')

            # Parse once; reused for state-mismatch reconcile below.
            announced_node_id = None
            announced_state = ''
            parse_error: Optional[str] = None
            try:
                import json
                parsed = json.loads(raw)
                announced_node_id = parsed.get('node_id')
                announced_state = parsed.get('state', '')
            except Exception as e:
                parse_error = f'{type(e).__name__}: {e}'

            # Truncated / malformed announcement: surface it to the
            # operator. Without this branch, the failure only goes to
            # journalctl (the rclpy logger) and the dashboard's Logs
            # tab shows nothing — operators chase phantom symptoms like
            # "Sync to Node has no response" because the upstream
            # buffer overflow is invisible.
            #
            # We extract node_id with a regex even from a truncated
            # payload (it's the second key in the announce schema, so
            # it's almost always intact) and attribute the warning to
            # that node's log. If even the node_id is unreachable, the
            # warning lands in the global activity feed.
            if parse_error is not None:
                self._report_announcement_parse_error(raw, parse_error)
                # Skip the rest of the pipeline — without a valid
                # announcement we have no node_id to wire subscribers
                # / publishers to.
                return

            is_new = self.state_manager.update_node_from_announcement(raw)
            if is_new:
                self.get_logger().info(f'New node discovered via announcement')
            elif announced_node_id:
                self.get_logger().debug(f'Updated node {announced_node_id}')

            if announced_node_id:
                # Subscribe to everything the firmware publishes for
                # this node, so data flows whether or not the operator
                # has triggered a control / config push yet. State at
                # 10 Hz feeds the Live Readings tab + widget dashboard;
                # log lines feed the per-node Logs tab; capabilities
                # are kept for legacy nodes that still publish them.
                self._ensure_node_log_subscriber(announced_node_id)
                self._ensure_node_state_subscriber(announced_node_id)
                self._ensure_node_capabilities_subscriber(announced_node_id)
                # Pi-side nodes publish ble_scan_results when the
                # operator triggers a scan; other targets never
                # publish here, but the subscription is free.
                self._ensure_node_ble_scan_subscriber(announced_node_id)
                # Soundboard reply topics (filesystem browse, audio-device
                # enumeration, play acks). Cheap idle subscriptions on
                # non-Pi nodes that never publish here.
                self._ensure_node_soundboard_subscribers(announced_node_id)
                # Set up the OTA progress subscriber eagerly too. We
                # used to create it lazily from send_firmware_update_command
                # right before publishing, but DDS subscription matching
                # takes a few seconds — and the Pi-side updater starts
                # spraying download-progress messages immediately after
                # receiving the command. The race window dropped most
                # of the bar's "0 % → 100 %" climb on the floor (only
                # the very late messages, after match completed, made
                # it through), so the operator saw a stuck-at-0 bar
                # that only jumped to 100 % at the end. Pre-creating
                # here gives DDS plenty of time to match before any OTA
                # fires; the cost is one extra subscription per
                # adopted node, which is cheap.
                self._ensure_node_update_progress_subscriber(announced_node_id)

                # Pre-create per-node publishers at first contact so DDS
                # discovery completes during the quiet stretch between
                # boot and the operator's first Sync / control / command
                # action. Without this, the publishers were created
                # lazily inside send_config_to_node / send_control_command
                # / send_node_command — and the very first publish on
                # that brand-new publisher races with DDS matching: the
                # firmware's subscription has existed since boot, but
                # DDS still needs ~1-5 s to match the just-created
                # publisher, and VOLATILE durability drops any messages
                # published before the match completes. Operators then
                # see "Pushed peripheral config (Sync to Node)" on the
                # server log with no corresponding "Config received" on
                # the firmware side, and the sync silently fails.
                self._ensure_node_config_publisher(announced_node_id)
                self._ensure_node_control_publisher(announced_node_id)
                self._ensure_node_command_publisher(announced_node_id)

                # Auto-reconcile firmware-server state mismatch. If we
                # have this node in adopted_nodes but it announces
                # UNADOPTED — typically an OTA wiped or invalidated its
                # flash-saved peripheral config — re-push our config so
                # the firmware can re-enter ACTIVE. Without this the
                # operator has to notice the divergence and hit Sync to
                # Node by hand.
                self._maybe_reconcile_adopted_unadopted(
                    announced_node_id, announced_state)

                # /announce-borne sync ACK. The firmware bumps
                # last_config_save_{ok,fail}_ms inside its config
                # callback after pin_config_save() returns. Edge-trigger
                # on either field increasing.
                self._maybe_handle_announce_sync_ack(announced_node_id, parsed)
        except Exception as e:
            # Catch-all is deliberate: this runs on the ROS callback
            # thread, so a propagating exception would tear down the
            # announcement subscription and stop ALL nodes from being
            # discovered. We must not crash here — but we must also not
            # fail SILENTLY. Malformed payloads are already handled above
            # (parse errors return early via _report_announcement_parse_error),
            # so reaching this block means a SERVER-SIDE BUG — a missing/
            # renamed collaborator (AttributeError), a bad call signature
            # (TypeError), etc. Those used to log a one-line message that
            # read like a bad-payload problem and carried no traceback, so
            # whole stretches of the handler (publishers, reconcile) could
            # silently no-op. Log the exception type + full traceback so
            # the failing call is locatable, and say plainly it's likely a
            # bug, not bad data.
            import traceback
            preview_len = len(raw) if raw is not None else 0
            self.get_logger().error(
                f'BUG processing announcement ({preview_len} bytes): '
                f'{type(e).__name__}: {e} — the payload already parsed, so '
                f'this is a server-side error, not a malformed announcement. '
                f'Downstream steps for this node were skipped.\n'
                f'{traceback.format_exc()}'
                f'  RAW: {raw!r}'
            )

    # 30 s minimum between "announcement parse error" log entries for
    # the same node. A node with a truncated payload announces at 1 Hz,
    # so without rate-limiting we'd flood the per-node log buffer with
    # 3600 identical warnings per hour. 30 s gives the operator enough
    # signal to notice + investigate without drowning out other logs.
    _ANNOUNCE_ERROR_COOLDOWN_S = 30.0

    def _report_announcement_parse_error(self, raw: str, parse_error: str) -> None:
        """Surface a malformed announcement to the dashboard.

        The original handler caught the json.JSONDecodeError, logged it
        to rclpy (→ journalctl), and moved on. That made buffer
        overflows on the firmware side functionally invisible: the
        dashboard's Logs tab showed nothing, and the operator had no
        way to attribute the missing-announcement symptom to its
        actual cause. This routes the warning into the per-node log
        when we can identify the source, and into the global activity
        feed otherwise.

        The "second key in the schema" heuristic for extracting
        node_id from a truncated payload is robust because the
        firmware's announcement_buffer fills front-to-back via
        snprintf, and the node_id field is emitted right after the
        opening brace. Truncation only ever cuts the tail.
        """
        import re
        node_id: Optional[str] = None
        m = re.search(r'"node_id"\s*:\s*"([^"]+)"', raw or '')
        if m:
            node_id = m.group(1)

        # Cap the preview so a 1024-byte payload doesn't bloat the log
        # rows in the dashboard table. 200 B is enough to see the
        # opening of the JSON and the spot where truncation happened.
        preview = (raw or '')[:200]
        if raw and len(raw) > 200:
            preview += '…'

        msg = (
            f'Malformed announcement ({len(raw) if raw else 0} bytes) — {parse_error}. '
            f'Common cause: firmware announcement_buffer overflow (see '
            f'firmware/rp2040/src/main.c:announcement_buffer). '
            f'Preview: {preview!r}'
        )

        # Always emit to the structured log so journalctl history is
        # intact. The per-node / activity-feed routing below is the
        # operator-facing surface.
        self.get_logger().warn(f'Announcement parse error from '
                                f'{node_id or "<unknown>"}: {parse_error}')

        import time
        now = time.time()
        if not hasattr(self, '_announce_error_last_reported'):
            self._announce_error_last_reported = {}  # type: ignore[attr-defined]

        key = node_id or '<global>'
        last = self._announce_error_last_reported.get(key, 0.0)
        if now - last < self._ANNOUNCE_ERROR_COOLDOWN_S:
            return
        self._announce_error_last_reported[key] = now

        if node_id:
            # Per-node Logs tab will pick this up via the existing
            # log_node_event → broadcast_node_log fanout.
            self.state_manager.log_node_event(node_id, msg, 'warn')
        else:
            # Couldn't recover a node_id (extreme truncation, or a
            # totally different payload format). Fall back to the
            # global activity feed.
            try:
                self.state_manager._log_activity(msg, 'warn')
            except Exception:
                pass

    # 10 s between reconcile pushes for the same node. Firmware takes a
    # few hundred ms to apply + flash-save and then needs to announce
    # again with state=ACTIVE before we know the recovery worked. 10 s
    # is comfortably longer than that worst case while still being
    # short enough for an operator to see "it healed itself" on the
    # Logs tab without waiting forever.
    _RECONCILE_COOLDOWN_S = 10.0

    def _maybe_reconcile_adopted_unadopted(self, node_id: str,
                                            announced_state: str) -> None:
        """Re-push peripheral config when an adopted node says UNADOPTED.

        Rate-limited per node via NodeInfo.last_reconcile_push_at so a
        firmware that keeps failing apply doesn't get hammered at 1 Hz
        (the announcement cadence in UNADOPTED state). Once the
        firmware successfully applies, its next announcement will say
        ACTIVE and this guard short-circuits.
        """
        if announced_state != "UNADOPTED":
            return
        node = self.state_manager.state.adopted_nodes.get(node_id)
        if not node:
            return

        import time
        now = time.time()
        if now - node.last_reconcile_push_at < self._RECONCILE_COOLDOWN_S:
            return

        config_json = self.state_manager.get_firmware_config_json(node_id)
        if not config_json:
            # Adopted but no non-builtin peripherals yet. Send an empty
            # peripherals array — RP2040's pin_config_apply_json accepts
            # `{"peripherals":[]}` (apply_peripherals_json walks an empty
            # array without error and returns true → ACTIVE transition).
            # Teensy main.cpp:176-186 treats any configure as adoption,
            # so the same payload works there too.
            config_json = '{"action":"configure","version":0,"peripherals":[]}'

        node.last_reconcile_push_at = now
        self.send_config_to_node(node_id, config_json)
        self.state_manager.log_node_event(
            node_id,
            "Adopted node announced UNADOPTED — re-pushing peripheral config",
            "warn",
        )

    def _periodic_update(self):
        """Periodic update - check timeouts, publish status."""
        # Check for nodes that have timed out
        timed_out = self.state_manager.check_node_timeouts()
        if timed_out:
            self.get_logger().info(f'Nodes timed out: {timed_out}')

    def _cleanup_stale_nodes(self):
        """Remove unadopted nodes that have been offline too long."""
        removed = self.state_manager.remove_offline_unadopted_nodes(max_offline_seconds=60.0)
        if removed > 0:
            self.get_logger().debug(f'Removed {removed} stale unadopted nodes')

    # =========================================================================
    # Pin Configuration Communication
    # =========================================================================

    def _sanitize_topic_name(self, name: str) -> str:
        """Sanitize a string for use in ROS2 topic names."""
        # Replace invalid characters with underscores
        return ''.join(c if c.isalnum() or c == '_' else '_' for c in name)

    def _ensure_node_config_publisher(self, node_id: str):
        """Ensure a config publisher exists for a node."""
        if node_id in self._node_config_pubs:
            return self._node_config_pubs[node_id]

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/config'

        pub = self.create_publisher(
            String,
            topic,
            10,
            callback_group=self.callback_group
        )
        self._node_config_pubs[node_id] = pub
        self.get_logger().info(f'Created config publisher: {topic}')
        return pub

    def _ensure_node_capabilities_subscriber(self, node_id: str):
        """Ensure a capabilities subscriber exists for a node."""
        if node_id in self._node_caps_subs:
            return

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/capabilities'

        def callback(msg: String):
            self._on_node_capabilities(node_id, msg)

        sub = self.create_subscription(
            String,
            topic,
            callback,
            10,
            callback_group=self.callback_group
        )
        self._node_caps_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to capabilities: {topic}')

    def _on_node_capabilities(self, node_id: str, msg: String):
        """Handle capabilities message from a node."""
        try:
            self.get_logger().info(f'Received capabilities from {node_id}: {msg.data[:200]}...' if len(msg.data) > 200 else f'Received capabilities from {node_id}: {msg.data}')
            success = self.state_manager.update_node_capabilities(node_id, msg.data)
            if success:
                self.get_logger().info(f'Capabilities stored successfully for {node_id}')
                # Mark node as synced (config confirmed by node publishing capabilities)
                self.state_manager.mark_node_synced(node_id, success=True)
                # Broadcast update to WebSocket clients
                self._broadcast_node_capabilities(node_id)
                self._broadcast_sync_status(node_id)
            else:
                self.get_logger().warn(f'Failed to store capabilities for {node_id}')
        except Exception as e:
            self.get_logger().error(f'Error processing capabilities from {node_id}: {e}')

    def _broadcast_node_capabilities(self, node_id: str):
        """Broadcast capabilities update to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                caps = self.state_manager.get_node_capabilities(node_id)
                if caps:
                    # Include peripheral sync_status in the capabilities payload
                    peripherals = self.state_manager.get_node_peripherals(node_id)
                    if peripherals:
                        caps["sync_status"] = peripherals.get("sync_status", "unknown")
                        caps["last_synced"] = peripherals.get("last_synced")
                    caps_copy = dict(caps)
                    topic = f'node_capabilities/{node_id}'
                    self._async_loop.call_soon_threadsafe(
                        lambda c=caps_copy, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, c)
                        )
                    )

    def _broadcast_sync_status(self, node_id: str):
        """Broadcast peripheral sync status to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                peripherals = self.state_manager.get_node_peripherals(node_id)
                if peripherals:
                    status_data = {
                        "node_id": node_id,
                        "sync_status": peripherals.get("sync_status", "unknown"),
                        "last_synced": peripherals.get("last_synced"),
                    }
                    topic = f'sync_status/{node_id}'
                    self._async_loop.call_soon_threadsafe(
                        lambda d=status_data, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, d)
                        )
                    )

    def send_config_to_node(self, node_id: str, config_json: str):
        """Send pin configuration to a node via ROS2."""
        pub = self._ensure_node_config_publisher(node_id)
        self._ensure_node_capabilities_subscriber(node_id)

        msg = String()
        msg.data = config_json

        pub.publish(msg)
        self.get_logger().info(f'Sent config to node {node_id}')
        self.state_manager.log_node_event(
            node_id, "Pushed peripheral config (Sync to Node)", "info")

        # Mark as syncing (not yet confirmed)
        # The node will publish capabilities after applying config

    def request_node_capabilities(self, node_id: str):
        """Request capabilities from a node."""
        pub = self._ensure_node_config_publisher(node_id)
        self._ensure_node_capabilities_subscriber(node_id)

        # Send capabilities request
        msg = String()
        msg.data = '{"action":"request_capabilities"}'

        pub.publish(msg)
        self.get_logger().info(f'Requested capabilities from node {node_id}')
        self.state_manager.log_node_event(
            node_id, "Requested capabilities refresh", "info")

    # =========================================================================
    # Pin Control Communication
    # =========================================================================

    def _ensure_node_control_publisher(self, node_id: str):
        """Ensure a control publisher exists for a node."""
        if node_id in self._node_control_pubs:
            return self._node_control_pubs[node_id]

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/control'

        pub = self.create_publisher(
            String,
            topic,
            CONTROL_QOS,
            callback_group=self.callback_group
        )
        self._node_control_pubs[node_id] = pub
        self.get_logger().info(f'Created control publisher: {topic} (best-effort, depth=1)')
        return pub

    def _ensure_node_command_publisher(self, node_id: str):
        """Ensure a command publisher exists for a node.

        Commands are operator one-shot actions (firmware_update,
        factory_reset, identify, estop, …) that must not be dropped.
        Separate topic + RELIABLE QoS so they don't share the streaming
        /control topic's fire-and-forget semantics.
        """
        if node_id in self._node_command_pubs:
            return self._node_command_pubs[node_id]

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/command'

        pub = self.create_publisher(
            String,
            topic,
            COMMAND_QOS,
            callback_group=self.callback_group
        )
        self._node_command_pubs[node_id] = pub
        self.get_logger().info(f'Created command publisher: {topic} (reliable, depth=8)')
        return pub

    def _ensure_node_state_subscriber(self, node_id: str):
        """Ensure a state subscriber exists for a node."""
        if node_id in self._node_state_subs:
            return

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/state'

        def callback(msg: String):
            self._on_node_state(node_id, msg)

        sub = self.create_subscription(
            String,
            topic,
            callback,
            TELEMETRY_QOS,
            callback_group=self.callback_group
        )
        self._node_state_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to state: {topic}')

    def _ensure_node_log_subscriber(self, node_id: str):
        """Ensure a log subscriber exists for a node.

        The firmware publishes JSON-encoded lines to
        /saint/nodes/<id>/log; we route each into the per-node
        log buffer so they appear in the node-detail Logs tab.
        """
        if node_id in self._node_log_subs:
            return

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/log'

        def callback(msg: String):
            self._on_node_log(node_id, msg)

        # LOG_QOS = BEST_EFFORT depth=20. Must match the firmware's
        # log publisher (also BEST_EFFORT). RELIABLE here used to leave
        # the firmware's micro-XRCE-DDS writer wedged after a 4+ line
        # burst; see the LOG_QOS docstring above.
        sub = self.create_subscription(
            String,
            topic,
            callback,
            LOG_QOS,
            callback_group=self.callback_group
        )
        self._node_log_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to log: {topic}')

    def _ensure_node_ble_scan_subscriber(self, node_id: str):
        """Subscribe to /saint/nodes/<id>/ble_scan_results. The Pi node
        publishes one frame per scan completion (or busy/error). We
        forward each frame to WebSocket clients via the normal ROS-
        bridge broadcast so the Peripherals modal can render the
        discovered device list."""
        if node_id in self._node_ble_scan_subs:
            return
        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/ble_scan_results'

        def callback(msg: String):
            self._on_node_ble_scan(node_id, msg)

        sub = self.create_subscription(
            String, topic, callback, TELEMETRY_QOS,
            callback_group=self.callback_group,
        )
        self._node_ble_scan_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to ble_scan: {topic}')

    def _on_node_ble_scan(self, node_id: str, msg: String):
        """Forward a scan-result frame to WebSocket subscribers.
        Payload shape lives in firmware/raspberrypi/saint_node/node.py's
        `_handle_ble_scan` worker."""
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warning(
                f'ble_scan_results: invalid JSON from {node_id}: {msg.data!r}')
            return
        topic = f'ble_scan_results/{node_id}'
        self._broadcast_ws_state(topic, data)

    def _ensure_node_update_progress_subscriber(self, node_id: str):
        """Subscribe to /saint/nodes/<id>/update_progress and forward each
        frame to WebSocket clients as `update_progress/<node_id>`. The
        Pi-side updater publishes a frame per stage transition during
        an OTA: {stage: 'downloading'|'verifying'|'extracting'|...,
        percent: 0..100, status: 'in_progress'|'complete'|'failed',
        message?: str}. Without this subscription the Pi's progress
        publications fall on a deaf server and the UI's update modal
        has no way to render the long Pi download as anything but a
        spinning ball."""
        if node_id in self._node_update_progress_subs:
            return
        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/update_progress'

        def callback(msg: String):
            self._on_node_update_progress(node_id, msg)

        sub = self.create_subscription(
            String, topic, callback, TELEMETRY_QOS,
            callback_group=self.callback_group,
        )
        self._node_update_progress_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to update_progress: {topic}')

    def _on_node_update_progress(self, node_id: str, msg: String):
        """Forward an update-progress frame to WebSocket subscribers."""
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warning(
                f'update_progress: invalid JSON from {node_id}: {msg.data!r}')
            return
        topic = f'update_progress/{node_id}'
        self._broadcast_ws_state(topic, data)

    def _ensure_node_soundboard_subscribers(self, node_id: str):
        """Subscribe to a node's three soundboard reply topics and forward
        each frame to WebSocket clients. All three carry a ``request_id``
        the frontend matches against its pending browse / device / play
        request (same async pattern as ble_scan_results)."""
        if node_id in self._node_soundboard_subs:
            return
        sanitized_id = self._sanitize_topic_name(node_id)
        subs = []
        for suffix, ws_prefix in (
            ("soundboard_fs", "soundboard_fs"),
            ("soundboard_devices", "soundboard_devices"),
            ("soundboard_result", "soundboard_result"),
        ):
            topic = f'/saint/nodes/{sanitized_id}/{suffix}'

            def make_cb(prefix):
                def callback(msg: String):
                    self._on_node_soundboard(prefix, node_id, msg)
                return callback

            subs.append(self.create_subscription(
                String, topic, make_cb(ws_prefix), TELEMETRY_QOS,
                callback_group=self.callback_group,
            ))
        self._node_soundboard_subs[node_id] = subs
        self.get_logger().info(f'Subscribed to soundboard topics for {node_id}')

    def _on_node_soundboard(self, ws_prefix: str, node_id: str, msg: String):
        """Forward a soundboard reply frame to WebSocket subscribers as
        ``<ws_prefix>/<node_id>``."""
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warning(
                f'{ws_prefix}: invalid JSON from {node_id}: {msg.data!r}')
            return
        self._broadcast_ws_state(f'{ws_prefix}/{node_id}', data)

    def send_soundboard_command(self, node_id: str, action: str,
                                args: Optional[dict] = None,
                                request_id: str = "") -> None:
        """Push a soundboard control action to a node.

        ``action`` is one of ``soundboard_list_dir`` /
        ``soundboard_list_devices`` / ``soundboard_play`` /
        ``soundboard_stop``. Replies (except stop) come back on the
        node's soundboard reply topics keyed by ``request_id``.
        """
        import json as _json
        pub = self._ensure_node_control_publisher(node_id)
        self._ensure_node_soundboard_subscribers(node_id)
        payload = {"action": action, "request_id": str(request_id)}
        if args:
            payload["args"] = dict(args)
        msg = String()
        msg.data = _json.dumps(payload)
        pub.publish(msg)
        self.get_logger().info(
            f'Sent {action} to {node_id} (request_id={request_id!r})')

    def send_ble_scan_command(self, node_id: str, duration_s: float = 8.0,
                              filter_jbd: bool = True,
                              request_id: str = "") -> None:
        """Push a `ble_scan` control action to the node. Results come
        back asynchronously on the ble_scan_results topic."""
        # host_controller's BLE stack lives in-process — there's no
        # Pi-node firmware on the wire to receive a /control message.
        # Dispatch directly to the HostPeripheralManager and broadcast
        # the result the same way (`ble_scan_results/<node_id>`) so
        # the UI's scan modal flow is identical for both paths.
        if node_id == HOST_CONTROLLER_NODE_ID:
            self._dispatch_host_ble_scan(duration_s, filter_jbd, request_id)
            return

        import json as _json
        pub = self._ensure_node_control_publisher(node_id)
        self._ensure_node_ble_scan_subscriber(node_id)
        payload = {
            "action": "ble_scan",
            "duration_s": float(duration_s),
            "filter_jbd": bool(filter_jbd),
            "request_id": str(request_id),
        }
        msg = String()
        msg.data = _json.dumps(payload)
        pub.publish(msg)
        self.get_logger().info(
            f'Sent ble_scan to {node_id}: '
            f'duration_s={duration_s} filter_jbd={filter_jbd} '
            f'request_id={request_id!r}')

    def _dispatch_host_ble_scan(self, duration_s: float, filter_jbd: bool,
                                request_id: str) -> None:
        """Run a scan on the server's BLE adapter and broadcast results
        on `ble_scan_results/host_controller` once it completes. Mirrors
        the payload shape a Pi-node would publish so the frontend's
        result handler is unchanged."""
        if self._async_loop is None:
            self._broadcast_ws_state(
                f'ble_scan_results/{HOST_CONTROLLER_NODE_ID}',
                {"status": "error",
                 "message": "host BLE stack not ready (no async loop)",
                 "request_id": request_id})
            return

        import time
        from saint_server.host_peripherals.scanner import scan_jbd_devices

        started_at = time.time()

        topic = f'ble_scan_results/{HOST_CONTROLLER_NODE_ID}'

        def on_progress(devices_so_far):
            # Fire each time a new device is discovered so the
            # operator's modal populates incrementally instead of
            # staring at a spinner for the full scan window. Same
            # envelope shape as the final result, with status=scanning
            # so the frontend keeps the progress indicator up.
            self._broadcast_ws_state(topic, {
                "status": "scanning",
                "devices": devices_so_far,
                "duration_s": duration_s,
                "filter_jbd": filter_jbd,
                "started_at": started_at,
                "request_id": request_id,
            })

        async def _run():
            try:
                devices = await scan_jbd_devices(
                    duration_s=duration_s,
                    filter_service=filter_jbd,
                    on_progress=on_progress)
                payload = {
                    "status": "ok",
                    "devices": devices,
                    "duration_s": duration_s,
                    "filter_jbd": filter_jbd,
                    "started_at": started_at,
                    "finished_at": time.time(),
                    "request_id": request_id,
                }
            except Exception as e:
                self.get_logger().warning(f'host BLE scan failed: {e}')
                payload = {
                    "status": "error",
                    "message": str(e),
                    "request_id": request_id,
                }
            self._broadcast_ws_state(topic, payload)

        try:
            asyncio.run_coroutine_threadsafe(_run(), self._async_loop)
        except RuntimeError as e:
            self._broadcast_ws_state(
                topic,
                {"status": "error",
                 "message": f"failed to schedule scan: {e}",
                 "request_id": request_id})
            return
        self.get_logger().info(
            f'Started host BLE scan: duration_s={duration_s} '
            f'filter_jbd={filter_jbd} request_id={request_id!r}')

    def _on_node_log(self, node_id: str, msg: String):
        """Receive a firmware log line and feed it into the per-node buffer.

        Firmware emits {"level": "info|warn|error", "text": "…", "uptime_ms": N}.
        Anything malformed gets stored as a raw-payload "warn" so we never lose
        a hint that the firmware tried to say something.

        We also use specific log texts as sync acks. Since the firmware
        no longer publishes a capabilities message after applying
        config (stripped in 1b42bd6), the pre-existing sync_status
        flow had no signal to flip from "pending" to "synced". The
        "Config saved to flash" log line is now that signal — it fires
        from main.c's config_subscription_callback after pin_config_save()
        returns true.
        """
        try:
            import json
            data = json.loads(msg.data)
            level = str(data.get('level', 'info'))
            text = str(data.get('text', ''))
            up = data.get('uptime_ms')
            if up is not None:
                text = f'[+{int(up) / 1000.0:.3f}s] {text}'
            if not text:
                return
            # Forward an optional peripheral tag if the firmware
            # included one (e.g. {"peripheral": "roboclaw-1"}). Falls
            # back to a leading-prefix extraction so the existing
            # "roboclaw: …" / "FAS100: …" patterns also light up the
            # column without a firmware-side change.
            peripheral = data.get('peripheral')
            if not peripheral:
                peripheral = _extract_peripheral_from_text(text)
            self.state_manager.log_node_event(
                node_id, text, level, peripheral=peripheral)
            self._maybe_handle_sync_ack(node_id, text)
        except Exception as e:
            self.state_manager.log_node_event(
                node_id, f'(malformed log frame: {e}) {msg.data[:120]!r}', 'warn')

    def _maybe_handle_announce_sync_ack(self, node_id: str, parsed: dict) -> None:
        """Edge-trigger sync-ACK from the /announce JSON. The firmware
        bumps last_config_save_ok_ms (uptime ms at the moment the save
        returned true) inside config_subscription_callback. We remember
        the last value we saw per node and flip the UI's pill the first
        time it strictly increases. Same for the fail variant. A node
        reboot resets uptime → the counter goes back to 0; the strict-
        increase check needs to treat that as "no event" not "decreased
        from 5000 → 0 = ack", so we early-out when the new value is
        smaller than the cached one (and update the cache).
        """
        try:
            ok = int(parsed.get('last_config_save_ok_ms') or 0)
            fail = int(parsed.get('last_config_save_fail_ms') or 0)
        except (TypeError, ValueError):
            return

        prev_ok = self._node_last_save_ok_ms.get(node_id, 0)
        prev_fail = self._node_last_save_fail_ms.get(node_id, 0)

        # Reboot: uptime restarted, drop cache and wait for the next
        # tick to set a new baseline. Don't fire an ack.
        if ok < prev_ok:
            self._node_last_save_ok_ms[node_id] = ok
            return
        if fail < prev_fail:
            self._node_last_save_fail_ms[node_id] = fail
            return

        if ok > prev_ok:
            self._node_last_save_ok_ms[node_id] = ok
            self.state_manager.mark_node_synced(node_id, success=True)
            self._broadcast_sync_status(node_id)
            self.state_manager.log_node_event(
                node_id, f"Config saved to flash (sync ACK via /announce, uptime_ms={ok})",
                "info")
        if fail > prev_fail:
            self._node_last_save_fail_ms[node_id] = fail
            self.state_manager.mark_node_synced(node_id, success=False)
            self._broadcast_sync_status(node_id)
            self.state_manager.log_node_event(
                node_id, f"Config apply/save failed (sync NACK via /announce, uptime_ms={fail})",
                "error")

    def _maybe_handle_sync_ack(self, node_id: str, text: str) -> None:
        """If the firmware reported it finished applying + saving config,
        flip the node's sync_status to ``synced`` and broadcast the
        update so the UI's "Sync to Node" pill updates.

        Recognized ack markers across firmware targets:
          * "Config saved to flash"  — RP2040 / Teensy (pin_config_save)
          * "Config saved to disk"   — Pi node (YAML save under /etc/)

        Both indicate "I received the config push, applied it, AND
        durably persisted it." Without that durable signal we can't
        safely call sync 'complete' — a node that applied in memory
        but failed its persist would lose the config on reboot.

        Failure markers mirror the success ones across targets.
        """
        if "Config saved to flash" in text or "Config saved to disk" in text:
            self.state_manager.mark_node_synced(node_id, success=True)
            self._broadcast_sync_status(node_id)
        elif ("Config apply failed" in text
              or "Flash save failed" in text
              or "Config save failed" in text):
            self.state_manager.mark_node_synced(node_id, success=False)
            self._broadcast_sync_status(node_id)

    def _on_node_state(self, node_id: str, msg: String):
        """Handle state message from a node."""
        try:
            import json
            data = json.loads(msg.data)
            pins_data = data.get('pins', [])
            # Peripheral-first state path (transitional — see
            # docs/PERIPHERAL_FIRST_MIGRATION.md). Drivers that have
            # migrated emit channel records here as
            # [{peripheral_id, channel_id, value}, ...]; unmigrated
            # drivers send nothing here and still publish via pins[].
            channels_data = data.get('channels', [])

            # Update state manager
            self.state_manager.update_pin_actual(node_id, pins_data, channels_data)

            # Broadcast to WebSocket clients
            self._broadcast_pin_state(node_id)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid state JSON from {node_id}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing state from {node_id}: {e}')

    def _broadcast_pin_state(self, node_id: str):
        """Broadcast pin state update to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                state = self.state_manager.get_runtime_state(node_id)
                if state:
                    topic = f'pin_state/{node_id}'
                    state_copy = state
                    self._async_loop.call_soon_threadsafe(
                        lambda s=state_copy, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, s)
                        )
                    )

    def send_control_command(self, node_id: str, gpio: int, value: float):
        """Send pin control command to a node via ROS2."""
        import json

        pub = self._ensure_node_control_publisher(node_id)
        self._ensure_node_state_subscriber(node_id)

        control_data = {
            "action": "set_pin",
            "gpio": gpio,
            "value": value
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().debug(f'Sent control to {node_id}: GPIO {gpio} = {value}')

    def send_channel_command(self, node_id: str, peripheral_id: str,
                             channel_id: str, value: float,
                             peripheral_type: str = "",
                             raw_us: Optional[int] = None):
        """Send channel-addressed control to a node via ROS2.

        Operator-visible names go on the wire — the firmware does its
        own (peripheral_id, channel_id) → driver dispatch. The server
        does no GPIO translation; that indirection is being deleted.

        `peripheral_type` is the catalog type (e.g. "neopixel",
        "roboclaw") — empty string if unknown. Firmware uses it to
        route writes for peripherals that don't live in pin_config (the
        built-in status NeoPixel is the canonical case — its instance
        id is "onboard_neopixel" but the firmware only knows it as
        type="neopixel").
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)
        self._ensure_node_state_subscriber(node_id)

        control_data = {
            "action": "set_channel",
            "peripheral": peripheral_id,
            "channel": channel_id,
            "value": value,
        }
        if peripheral_type:
            control_data["type"] = peripheral_type
        # Live extent-dial preview: absolute microseconds. The firmware
        # drives the Maestro channel straight to this pulse (bypassing
        # the normalized value→pulse map + per-channel clamp) so the
        # operator can dial in start/end/center/home visually. Not a
        # persisted write — `value` is ignored firmware-side when `us`
        # is present on a Maestro channel.
        if raw_us is not None:
            control_data["us"] = int(raw_us)

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().debug(
            f'Sent channel control to {node_id}: '
            f'{peripheral_type}/{peripheral_id}/{channel_id} = {value}')

    def send_peripheral_command(self, node_id: str, peripheral_id: str,
                                command: str, args: Dict[str, Any]):
        """Send an out-of-band, structured command to a peripheral on a
        node. The numeric set_channel path can't carry strings; this is
        how audio_player.play_file(filename) and similar reach the
        firmware. Routed over the RELIABLE command topic so a UDP drop
        doesn't silently lose a one-shot trigger.
        """
        import json

        pub = self._ensure_node_command_publisher(node_id)

        control_data = {
            "action": "peripheral_command",
            "peripheral": peripheral_id,
            "command": command,
            "args": args or {},
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().debug(
            f'Sent peripheral command to {node_id}: '
            f'{peripheral_id}.{command}({args})')

    def send_factory_reset_command(self, node_id: str):
        """Send factory reset command to a node via ROS2.

        This tells the node to:
        1. Clear its saved pin configuration
        2. Transition back to UNADOPTED state
        """
        import json

        pub = self._ensure_node_command_publisher(node_id)

        control_data = {
            "action": "factory_reset"
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent factory reset command to {node_id}')
        self.state_manager.log_node_event(
            node_id, "Sent factory-reset command", "warn")

    def send_restart_command(self, node_id: str):
        """Send restart command to a node via ROS2.

        This tells the node to reboot itself.
        """
        import json

        pub = self._ensure_node_command_publisher(node_id)

        control_data = {
            "action": "restart"
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent restart command to {node_id}')
        self.state_manager.log_node_event(node_id, "Sent restart command", "info")

    def send_identify_command(self, node_id: str):
        """Send identify command to a node via ROS2.

        This tells the node to blink its LED to help identify it physically.
        """
        import json

        pub = self._ensure_node_command_publisher(node_id)

        control_data = {
            "action": "identify"
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent identify command to {node_id}')
        self.state_manager.log_node_event(node_id, "Sent identify (blink LED) command", "info")

    def send_estop_command(self, node_id: str, action: str = "engage"):
        """Send emergency stop command to a node via ROS2.

        Args:
            node_id: target node id
            action: "engage" to assert e-stop (motors → 0, peripheral
                e-stop hooks fire, e.g. RoboClaw drives its estop_pin
                HIGH), or "release" to clear the latch and return
                peripherals to their normal control path.
        """
        import json

        pub = self._ensure_node_command_publisher(node_id)

        # JSON wire format kept backward-compatible: the legacy
        # firmware path matches on "action":"estop" for the engage
        # case. New firmware also matches "clear_estop" for release.
        # We emit BOTH the legacy action and an explicit state field
        # so old controllers keep stopping while new ones can also
        # release.
        if action == "release":
            wire_action = "clear_estop"
        else:
            wire_action = "estop"
        control_data = {
            "action": wire_action,
            "state": "engaged" if action == "engage" else "released",
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        verb = "engage" if action == "engage" else "release"
        self.get_logger().info(f'Sent emergency stop {verb} to {node_id}')
        self.state_manager.log_node_event(
            node_id, f"Sent emergency stop {verb}",
            "warn" if action == "engage" else "info")

    def send_roboclaw_debug_command(self, node_id: str, payload: dict):
        """Forward a RoboClaw wire-level debug payload to the node.

        `payload` is the {op, op-specific keys} dict; we wrap it in
        the control envelope ({"action":"roboclaw_debug", ...}) and
        publish. The firmware replies asynchronously via the normal
        log stream with a 'roboclaw_dbg:' prefix.
        """
        import json

        pub = self._ensure_node_command_publisher(node_id)
        control_data = {"action": "roboclaw_debug", **payload}
        msg = String()
        msg.data = json.dumps(control_data)
        pub.publish(msg)
        self.get_logger().info(
            f'Sent roboclaw_debug {payload.get("op")} to {node_id}')
        self.state_manager.log_node_event(
            node_id, f"roboclaw_debug {payload.get('op')} dispatched", "info")

    def send_firmware_update_command(self, node_id: str, simulation: bool = False, force: bool = False):
        """Send firmware update command to a node via ROS2.

        This tells the node to:
        1. For RP2040 simulation: Restart and load new firmware ELF
        2. For RP2040 hardware: Enter bootloader mode for UF2 update
        3. For Pi 5: Download and install update package from server

        The actual update mechanism depends on the node type.

        Args:
            node_id: Target node ID
            simulation: Whether this is a simulation build
            force: If True, skip version comparison on node side
        """
        import json

        # Determine node type from state manager
        node_info = self.state_manager.get_node(node_id)
        hw_type = node_info.get('hw', '') if node_info else ''

        pub = self._ensure_node_command_publisher(node_id)
        # Subscribe to the node's update_progress topic BEFORE pushing
        # the firmware_update command. Pi OTAs are slow (minutes) and
        # users need a live progress bar; without this subscription the
        # Pi's progress publications go unheard and the modal can only
        # render a vacuous spinner.
        self._ensure_node_update_progress_subscriber(node_id)

        if 'Raspberry Pi' in hw_type or 'raspberrypi' in hw_type.lower():
            # Raspberry Pi node (any generation) — server stages a
            # zipped Python package; the node downloads + extracts it
            # in-place over HTTP.
            self._send_raspberrypi_firmware_update(pub, node_id, force)
        elif 'Teensy' in hw_type or 'teensy' in node_id.lower():
            # Teensy 4.1 — same OTA wire format as RP2040 (size + crc32),
            # different resource path so the server serves the right .bin.
            self._send_teensy41_firmware_update(pub, node_id, force)
        else:
            # RP2040 node
            self._send_rp2040_firmware_update(pub, node_id, simulation, force)

    def _resolve_server_url_for_node(self, node_id: str, port: int,
                                     path: str) -> str:
        """Build an absolute http URL the node can fetch from.

        The node's saint-node updater uses ``urllib.request.urlretrieve``
        which won't resolve a relative path; the server has to embed
        its own reachable IP. We pick the IP of *our* interface that
        routes to the node's last-known address, so multi-homed
        servers (Ethernet + WiFi AP, NVMe + USB tether) hand the node
        a URL on the same subnet it adopted us on.
        """
        node = self.state_manager.get_node(node_id)
        node_ip = (node or {}).get('ip') or ''
        # Mirror discovery.py's heuristic: open a UDP socket toward the
        # node and ask the OS which local address it would source from.
        # No packets actually leave (SOCK_DGRAM connect is a route lookup).
        server_ip = "127.0.0.1"
        try:
            import socket
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                target = node_ip or '8.8.8.8'
                s.connect((target, 80))
                server_ip = s.getsockname()[0]
        except Exception as e:
            self.get_logger().warning(
                f'Could not derive server IP for {node_id} '
                f'(node_ip={node_ip!r}): {e}; falling back to {server_ip}')
        return f"http://{server_ip}:{port}{path}"

    def _send_raspberrypi_firmware_update(self, pub, node_id: str, force: bool = False):
        """Send firmware update command to a Raspberry Pi node.

        Pi nodes ship a zipped Python package and use urllib to fetch
        it over HTTP. The URL embedded here must be absolute — see
        _resolve_server_url_for_node for how the reachable server IP
        is derived from the node's last-known address.
        """
        import json

        fw_info = self.state_manager.get_firmware_info_for_type('raspberrypi')

        if not fw_info or not fw_info.get('available'):
            self.get_logger().error(
                f'No Raspberry Pi firmware available for update')
            return

        server_port = getattr(self.web_server, 'port', 80) if hasattr(self, 'web_server') else 80
        download_url = self._resolve_server_url_for_node(
            node_id, server_port,
            f'/api/firmware/raspberrypi/{fw_info["filename"]}')

        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version", "0.0.0"),
            "url": download_url,
            "checksum": fw_info.get("checksum"),
            "build_date": fw_info.get("build_date"),
            "force": force,
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(
            f'Sent Raspberry Pi firmware update to {node_id} '
            f'(version: {fw_info.get("version")}, force: {force}, '
            f'url: {download_url})')

    def _send_teensy41_firmware_update(self, pub, node_id: str, force: bool = False):
        """Send firmware update command to a Teensy 4.1 node.

        Teensy reads `size` + `crc32` from this message and downloads
        /api/firmware/teensy41/saint_node.bin over HTTP, then uses
        FlashTxx to swap it in. Without those fields the node falls
        back to a no-op SCB_AIRCR restart.
        """
        import json

        fw_info = self.state_manager.get_firmware_info_for_type('teensy41')
        if not fw_info or not fw_info.get('available'):
            self.get_logger().error('No Teensy 4.1 firmware available for update')
            return

        # Prefer the full version string ("1.0.0-<githash>-<unix_ts>") so
        # the node can extract the build timestamp out of it. Without the
        # trailing unix_ts the node's up-to-date check sees server_ts=0
        # and silently bails before starting the download. Matches the
        # RP2040 flow in _send_rp2040_firmware_update.
        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version_full") or fw_info.get("version") or "0.0.0",
            "build_date": fw_info.get("build_date"),
            "force": force,
        }

        if fw_info.get("bin_size") and fw_info.get("bin_crc32") is not None:
            control_data["size"]  = fw_info["bin_size"]
            control_data["crc32"] = f"0x{fw_info['bin_crc32']:08x}"
            control_data["url"]   = "/api/firmware/teensy41/saint_node.bin"

        msg = String()
        msg.data = json.dumps(control_data)
        pub.publish(msg)
        self.get_logger().info(
            f'Sent Teensy 4.1 firmware update command to {node_id} '
            f'(version: {control_data["version"]}, '
            f'size: {fw_info.get("bin_size")}, crc32: {control_data.get("crc32")}, '
            f'force: {force})'
        )

    def _send_rp2040_firmware_update(self, pub, node_id: str, simulation: bool, force: bool = False):
        """Send firmware update command to an RP2040 node.

        With the OTA bootloader present, the node uses `size` + `crc32`
        from this message to fetch /api/firmware/rp2040/saint_node.bin
        from the server (DHCP-gateway) and verify the download. Without
        a bootloader the node falls back to a no-op reboot.
        """
        import json

        # Get firmware info from state manager (includes bin size + crc32)
        fw_info = self.state_manager.get_server_firmware_info()

        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version_full") or fw_info.get("version", "0.0.0"),
            "elf_path": fw_info.get("elf_path"),
            "build_date": fw_info.get("build_date"),
            "force": force,
        }

        # OTA metadata — present only if the build pipeline produced the
        # raw .bin (i.e. the SAINT_OS_OTA_BOOTLOADER build path). When
        # missing, the node sees no size/crc32 and falls back to legacy
        # reboot behavior.
        if fw_info.get("bin_size") and fw_info.get("bin_crc32") is not None:
            control_data["size"] = fw_info["bin_size"]
            control_data["crc32"] = f"0x{fw_info['bin_crc32']:08x}"
            control_data["url"]  = f"/api/firmware/rp2040/saint_node.bin"

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(
            f'Sent RP2040 firmware update command to {node_id} '
            f'(version: {fw_info.get("version_full") or fw_info.get("version")}, '
            f'size: {fw_info.get("bin_size")}, crc32: {control_data.get("crc32")}, '
            f'force: {force})'
        )
        self.state_manager.log_node_event(
            node_id,
            f"Firmware update triggered → {fw_info.get('version_full') or fw_info.get('version')}"
            + (" (force)" if force else ""),
            "info"
        )

        # For simulation nodes, also trigger the node manager to install and restart
        if simulation:
            self._update_simulation_node(node_id)

    def _update_simulation_node(self, node_id: str):
        """Update a simulation node by calling the node manager.

        This:
        1. Copies firmware from build_sim to install directory
        2. Restarts the Renode simulation
        """
        import subprocess
        import os

        # Path to node manager script
        node_manager_path = os.path.join(
            os.path.dirname(__file__),
            '..', 'firmware', 'rp2040', 'simulation', 'saint_node_manager.py'
        )

        if not os.path.exists(node_manager_path):
            # Try alternate path (when running from install)
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_dir = get_package_share_directory('saint_os')
                node_manager_path = os.path.join(
                    pkg_dir, '..', '..', '..', '..', 'src', 'saint_os',
                    'firmware', 'rp2040', 'simulation', 'saint_node_manager.py'
                )
            except Exception:
                pass

        if not os.path.exists(node_manager_path):
            self.get_logger().error(f'Node manager not found at {node_manager_path}')
            return

        try:
            # Run update command (installs firmware and restarts node)
            result = subprocess.run(
                ['python3', node_manager_path, 'update', node_id],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.get_logger().info(f'Simulation node {node_id} updated and restarted')
                if result.stdout:
                    self.get_logger().info(f'Node manager output: {result.stdout}')
            else:
                self.get_logger().error(f'Failed to update simulation node: {result.stderr}')

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout updating simulation node {node_id}')
        except Exception as e:
            self.get_logger().error(f'Error updating simulation node {node_id}: {e}')

    # =========================================================================
    # LiveLink Integration
    # =========================================================================

    async def _start_livelink(self):
        """Start the LiveLink receiver and router."""
        try:
            # Create router with output callback
            self.livelink_router = LiveLinkRouter(logger=self.get_logger())
            self.livelink_router.set_output_callback(self._livelink_output_callback)

            # Create receiver
            config = LiveLinkConfig(
                port=11111,  # Default Live Link Face port
                bind_address="0.0.0.0",
                timeout=5.0
            )
            self.livelink_receiver = LiveLinkReceiver(config=config, logger=self.get_logger())

            # Connect receiver to router
            self.livelink_receiver.add_callback(self.livelink_router.process)

            # Also broadcast blend shapes to WebSocket clients for visualization
            self.livelink_receiver.add_callback(self._broadcast_blend_shapes)

            # Add connection event handlers for activity logging
            self.livelink_receiver.on_connect(self._on_livelink_connect)
            self.livelink_receiver.on_disconnect(self._on_livelink_disconnect)

            # Start receiver
            await self.livelink_receiver.start(loop=self._async_loop)

            self.get_logger().info(f'LiveLink receiver started on port {config.port}')

            # Set up WebSocket handlers for LiveLink
            if self.web_server and self.web_server.ws_handler:
                self.web_server.ws_handler.set_livelink_callbacks(
                    get_status=self.get_livelink_status,
                    get_routes=self.get_livelink_routes,
                    set_routes=self.set_livelink_routes,
                    enable_route=self.enable_livelink_route,
                    create_default_route=self.create_livelink_default_route,
                )

        except Exception as e:
            self.get_logger().error(f'Failed to start LiveLink: {e}')

    async def _stop_livelink(self):
        """Stop the LiveLink receiver."""
        if self.livelink_receiver:
            await self.livelink_receiver.stop()
            self.get_logger().info('LiveLink receiver stopped')

    def _on_livelink_connect(self, source_addr: str, subject_name: str):
        """Handle LiveLink connection event."""
        self._broadcast_activity(
            f'LiveLink connected: {subject_name} from {source_addr}',
            'info'
        )

    def _on_livelink_disconnect(self, source_addr: str):
        """Handle LiveLink disconnection event."""
        self._broadcast_activity(
            f'LiveLink disconnected: {source_addr}',
            'warn'
        )

    _last_blend_broadcast = 0.0  # Throttle blend shape broadcasts

    def _broadcast_blend_shapes(self, blend_shapes):
        """Broadcast blend shape data to WebSocket clients for visualization."""
        import time
        import math

        # Throttle to ~10 Hz
        now = time.time()
        if now - self._last_blend_broadcast < 0.1:
            return
        self._last_blend_broadcast = now

        if self.web_server and self.web_server.ws_handler and self._async_loop:
            # Helper to sanitize float values (NaN/Inf are not valid JSON)
            def sanitize(value):
                if math.isnan(value) or math.isinf(value):
                    return 0.0
                return value

            # Convert to dict for serialization
            data = blend_shapes.to_dict()

            # Sanitize all float values
            for key, value in data.items():
                if isinstance(value, float):
                    data[key] = sanitize(value)

            data['subject_name'] = blend_shapes.subject_name
            data['timestamp'] = blend_shapes.timestamp

            # Add computed properties (sanitized)
            data['eye_look_horizontal_left'] = sanitize(blend_shapes.eye_look_horizontal_left)
            data['eye_look_horizontal_right'] = sanitize(blend_shapes.eye_look_horizontal_right)
            data['eye_look_vertical_left'] = sanitize(blend_shapes.eye_look_vertical_left)
            data['eye_look_vertical_right'] = sanitize(blend_shapes.eye_look_vertical_right)
            data['brow_left'] = sanitize(blend_shapes.brow_left)
            data['brow_right'] = sanitize(blend_shapes.brow_right)

            self._async_loop.call_soon_threadsafe(
                lambda d=data: asyncio.create_task(
                    self.web_server.ws_handler.broadcast_state('livelink/blend_shapes', d)
                )
            )

    def _livelink_output_callback(self, node_id: str, output_name: str, value: float):
        """Callback from LiveLink router to send an output to a node.

        TODO: rewire through the system routing graph. The old code did
        a GPIO lookup via `pin_config.logical_name`; with peripherals owning
        their pins, the right replacement is to find a Route whose source
        is `livelink:<output_name>` (signal kind) and whose sink is a
        peripheral channel, then dispatch accordingly. Disabled until the
        routing engine ships.
        """
        self.get_logger().debug(
            f'LiveLink output {output_name} for {node_id} ignored — routing engine pending'
        )

    def get_livelink_status(self) -> Dict[str, Any]:
        """Get LiveLink status for WebSocket API."""
        result = {
            "enabled": self._livelink_enabled,
            "receiver": None,
            "router": None
        }

        if self.livelink_receiver:
            result["receiver"] = self.livelink_receiver.stats

        if self.livelink_router:
            result["router"] = self.livelink_router.get_status()

        return result

    def get_livelink_routes(self) -> List[Dict[str, Any]]:
        """Get all LiveLink routes."""
        if self.livelink_router:
            return self.livelink_router.get_routes_config()
        return []

    def set_livelink_routes(self, routes_data: List[Dict[str, Any]]):
        """Set LiveLink routes from config data."""
        if self.livelink_router:
            self.livelink_router.load_routes_config(routes_data)
            self.get_logger().info(f'Loaded {len(routes_data)} LiveLink routes')

    def enable_livelink_route(self, route_name: str, enabled: bool):
        """Enable or disable a LiveLink route."""
        if self.livelink_router:
            self.livelink_router.enable_route(route_name, enabled)
            self.get_logger().info(f'LiveLink route "{route_name}" {"enabled" if enabled else "disabled"}')

    def create_livelink_default_route(self, node_id: str) -> Dict[str, Any]:
        """Create and add a default head route for a node."""
        if self.livelink_router:
            route = self.livelink_router.create_default_head_route(node_id)
            self.livelink_router.add_route(route)
            return route.to_dict()
        return {}

    def _run_async_loop(self):
        """Run the asyncio event loop in a background thread."""
        self._async_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._async_loop)
        self._shutdown_event = asyncio.Event()

        # Construct the manager up-front so other call sites that
        # check `self._host_peripheral_manager is not None` see it
        # consistently. The actual reconcile wiring happens inside
        # _async_main, once the loop is running (reconcile schedules
        # coroutines on the loop and needs it live).
        self._host_peripheral_manager = HostPeripheralManager(
            loop=self._async_loop,
            state_cb=self._host_peripheral_state_cb,
            log_cb=self._host_peripheral_log_cb,
        )

        try:
            self._async_loop.run_until_complete(self._async_main())
        except Exception as e:
            self.get_logger().error(f'Async loop error: {e}')
        finally:
            try:
                if self._host_peripheral_manager is not None:
                    self._host_peripheral_manager.shutdown()
            except Exception:
                pass
            self._async_loop.close()

    def _host_peripheral_state_cb(self, peripheral_id: str,
                                  channel_id: str, value: float) -> None:
        """Bridge from a host_peripherals driver to NodeRuntimeState.
        Called on the async loop thread; mutating the channels dict
        races with the broadcast tick but the practical impact is just
        an occasional empty ChannelRuntimeState being created twice."""
        runtime = self.state_manager.get_or_create_runtime_state(
            HOST_CONTROLLER_NODE_ID)
        if runtime is None:
            return
        runtime.set_channel(peripheral_id, channel_id, float(value))

    def _host_peripheral_log_cb(self, level: str, msg: str) -> None:
        log = self.get_logger()
        fn = getattr(log, level if level in ("debug", "info", "warning",
                                              "warn", "error") else "info",
                     log.info)
        # rclpy uses `warning` not `warn`.
        if level == "warn":
            fn = log.warning
        try:
            fn(msg)
        except Exception:
            pass

    async def _async_main(self):
        """Main async coroutine managing all async services."""
        await self.start_async_services()

        # Loop is live now — safe to wire the host_peripheral reconcile
        # callback. State manager fires it once immediately with the
        # currently-loaded config so any BMSes saved in host_controller's
        # YAML come up without operator action.
        if self._host_peripheral_manager is not None:
            # Flush first so any stale BlueZ link from a previous
            # saint-os process (SIGKILL, OOM, reboot, fast restart)
            # gets torn down before we try to reconnect — JBD radios
            # only allow one central connection at a time and won't
            # accept us back until the stale link is dropped. Awaited
            # so the reconcile that follows sees a clean adapter.
            try:
                await self._host_peripheral_manager.flush_stale_connections()
            except Exception as e:
                self.get_logger().warning(
                    f'host_peripherals flush failed: {e}')
            self.state_manager.set_host_peripheral_reconcile_callback(
                self._host_peripheral_manager.reconcile)

        # Wait for shutdown signal
        await self._shutdown_event.wait()

        await self.stop_async_services()

    async def start_async_services(self):
        """Start async services (WebSocket, HTTP server, etc.)."""
        self.get_logger().info('Starting async services...')

        try:
            # Import here to avoid circular imports
            from saint_server.webserver import WebServer

            # Vue build output lives under web/dist/ — that's the
            # production UI. See web/MIGRATION.md.
            try:
                from ament_index_python.packages import get_package_share_directory
                web_root = os.path.join(get_package_share_directory('saint_os'), 'web', 'dist')
            except Exception:
                # Fallback for development
                web_root = os.path.join(os.path.dirname(__file__), '..', 'web', 'dist')

            # Create and start web server with shared state manager
            self.web_server = WebServer(
                web_root=web_root,
                port=self.web_port,
                server_name=self.server_name,
                logger=self.get_logger(),
                state_manager=self.state_manager
            )

            # Set up activity callback to broadcast to WebSocket clients
            self.state_manager.set_activity_callback(
                lambda msg, level: self._broadcast_activity(msg, level)
            )

            # Per-node log callback — feeds the Logs tab on node-detail.
            self.state_manager.set_node_log_callback(
                lambda node_id, entry: self._broadcast_node_log(node_id, entry)
            )

            # Set up pin config sync callback
            if self.web_server.ws_handler:
                self.web_server.ws_handler.set_sync_config_callback(
                    lambda node_id, config_json: self.send_config_to_node(node_id, config_json)
                )
                self.web_server.ws_handler.set_request_capabilities_callback(
                    lambda node_id: self.request_node_capabilities(node_id)
                )
                self.web_server.ws_handler.set_send_control_callback(
                    lambda node_id, gpio, value: self.send_control_command(node_id, gpio, value)
                )
                self.web_server.ws_handler.set_send_channel_callback(
                    lambda node_id, peripheral_id, channel_id, value, peripheral_type, raw_us=None:
                        self.send_channel_command(node_id, peripheral_id,
                                                  channel_id, value,
                                                  peripheral_type,
                                                  raw_us=raw_us)
                )
                self.web_server.ws_handler.set_send_peripheral_command_callback(
                    lambda node_id, peripheral_id, command, args:
                        self.send_peripheral_command(node_id, peripheral_id,
                                                     command, args)
                )
                # Hand the same publisher to state_manager so animation
                # trigger tracks of kind peripheral_command (audio cues
                # synced to a movement timecode) can fire commands
                # through the same wire path as the WS-issued one.
                self.state_manager.set_peripheral_command_sender(
                    self.send_peripheral_command)
                self.web_server.ws_handler.set_firmware_update_callback(
                    lambda node_id, simulation, force: self.send_firmware_update_command(node_id, simulation, force)
                )
                self.web_server.ws_handler.set_factory_reset_callback(
                    lambda node_id: self.send_factory_reset_command(node_id)
                )
                self.web_server.ws_handler.set_restart_node_callback(
                    lambda node_id: self.send_restart_command(node_id)
                )
                self.web_server.ws_handler.set_identify_node_callback(
                    lambda node_id: self.send_identify_command(node_id)
                )
                self.web_server.ws_handler.set_estop_node_callback(
                    lambda node_id, action: self.send_estop_command(node_id, action)
                )
                self.web_server.ws_handler.set_roboclaw_debug_callback(
                    lambda node_id, payload: self.send_roboclaw_debug_command(node_id, payload)
                )
                self.web_server.ws_handler.set_ble_scan_callback(
                    lambda node_id, duration_s, filter_jbd, request_id:
                        self.send_ble_scan_command(
                            node_id, duration_s, filter_jbd, request_id)
                )
                self.web_server.ws_handler.set_soundboard_callback(
                    lambda node_id, sb_action, args, request_id:
                        self.send_soundboard_command(
                            node_id, sb_action, args, request_id)
                )

                # Initialize ROS Bridge for WebSocket-ROS communication
                try:
                    from saint_server.ros_bridge import ROSBridge
                    self.ros_bridge = ROSBridge(self, logger=self.get_logger())
                    self.ros_bridge.initialize()
                    self.web_server.ws_handler.set_ros_bridge(self.ros_bridge)
                    # Animation trigger tracks need the bridge to
                    # publish on `topic`-kind dispatches; the registry
                    # spins up once both bridge + evaluator are present.
                    self.state_manager.set_ros_bridge(self.ros_bridge)
                    # Override the callback to use thread-safe async broadcast
                    self.ros_bridge.set_message_callback(self._broadcast_ros_message)
                    self.get_logger().info('ROS Bridge initialized and connected to WebSocket handler')

                    # Routing graph evaluator — drives peripheral channels
                    # from ROS topic inputs through user-defined sheets.
                    try:
                        from saint_server.router.routing_evaluator import RoutingEvaluator
                        self.routing_evaluator = RoutingEvaluator(
                            ros_bridge=self.ros_bridge,
                            send_channel=self.send_channel_command,
                            peripheral_type_lookup=self.state_manager.lookup_peripheral_type,
                            logger=self.get_logger(),
                            on_values_changed=self._broadcast_routing_values,
                        )
                        self.ros_bridge.add_routing_listener(
                            self.routing_evaluator.on_topic_message
                        )
                        self.state_manager.set_routing_evaluator(self.routing_evaluator)
                        self.get_logger().info('Routing graph evaluator initialized')
                    except Exception as e:
                        self.get_logger().warn(f'Routing evaluator not available: {e}')
                except Exception as e:
                    self.get_logger().warn(f'ROS Bridge not available: {e}')

            await self.web_server.start()

            self.get_logger().info(f'Web server started on http://localhost:{self.web_port}/')

            # Update manager: GitHub release polling + USB scan + apply.
            # Wired to the WebSocket handler so state changes broadcast to
            # clients on the 'update' topic.
            try:
                from saint_server.update_manager import UpdateManager
                self.update_manager = UpdateManager(logger=self.get_logger())
                if self.web_server.ws_handler:
                    self.web_server.ws_handler.set_update_manager(self.update_manager)
                # Fire a non-blocking startup check. Offline robots will
                # land on 'no_network' status quickly (5s timeout) and the
                # UI will surface a "Check for Updates" button.
                asyncio.create_task(self.update_manager.check_github())
                self.get_logger().info('Update manager initialized')
            except Exception as e:
                self.get_logger().warn(f'Update manager not available: {e}')

        except ImportError as e:
            self.get_logger().warn(f'Web server not available: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')

        # Start LiveLink receiver if enabled
        if self._livelink_enabled:
            await self._start_livelink()

        # Start discovery service if enabled
        if self._discovery_enabled:
            await self._start_discovery()

    async def _start_discovery(self):
        """Start the node discovery service."""
        try:
            self.discovery_service = DiscoveryService(
                agent_port=self._agent_port,
                discovery_port=self._discovery_port,
                logger=self.get_logger()
            )
            await self.discovery_service.start()
            self.get_logger().info(
                f'Discovery service started on UDP port {self._discovery_port} '
                f'(agent port: {self._agent_port})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start discovery service: {e}')

    async def _stop_discovery(self):
        """Stop the discovery service."""
        if self.discovery_service:
            await self.discovery_service.stop()
            self.discovery_service = None

    def _broadcast_activity(self, message: str, level: str):
        """Broadcast activity message to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            # Queue the activity broadcast in the async loop
            if self._async_loop:
                self._async_loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(
                        self.web_server.ws_handler.broadcast_activity(message, level)
                    )
                )

    def _broadcast_node_log(self, node_id: str, entry: Dict[str, Any]):
        """Push a per-node log entry to subscribers (thread-safe)."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                self._async_loop.call_soon_threadsafe(
                    lambda nid=node_id, e=entry: asyncio.create_task(
                        self.web_server.ws_handler.broadcast_node_log(nid, e)
                    )
                )

    def _broadcast_ros_message(self, topic: str, data: Dict[str, Any]):
        """Broadcast ROS message to WebSocket clients (thread-safe).

        Uses the ros-bridge envelope: `{type: "ros_state", topic, data}`
        with subscription key `ros:<topic>`. Right wire format for
        forwarding actual rclpy topics through the ROS bridge to web
        clients that subscribe via `ros:` prefix.

        For server-originated state pushed to topic-keyed UI subscribers
        (`pin_state/<id>`, `sync_status/<id>`, scan-results, etc.) use
        `_broadcast_ws_state` — its envelope matches the frontend's
        `ws.on('state', …)` listener and `ws.subscribe([topic], …)`
        flow.
        """
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                # Use call_soon_threadsafe to schedule broadcast in async loop
                self._async_loop.call_soon_threadsafe(
                    lambda t=topic, d=data: asyncio.create_task(
                        self.web_server.ws_handler.broadcast_ros_state(t, d)
                    )
                )

    def _broadcast_ws_state(self, topic: str, data: Dict[str, Any]):
        """Push a `{type: "state", node: topic, data}` frame to web
        clients subscribed to `topic`. Matches the WS pattern that
        backs `pin_state/<node_id>`, `sync_status/<node_id>`,
        `ble_scan_results/<node_id>`, etc. Threadsafe."""
        if not (self.web_server and self.web_server.ws_handler
                and self._async_loop):
            return
        self._async_loop.call_soon_threadsafe(
            lambda t=topic, d=data: asyncio.create_task(
                self.web_server.ws_handler.broadcast_state(t, d)
            )
        )

    # Throttle state for the routing-evaluator value broadcast. We want
    # live values on the routing UI but a Joy stream at 30Hz drags too
    # much over the WebSocket — clamp to 20Hz / 50ms with a trailing
    # edge so the final snapshot in any burst always reaches the UI.
    _ROUTING_VALUES_INTERVAL_S = 0.05
    _last_routing_values_emit: float = 0.0
    _pending_routing_snapshot: Optional[Dict[str, Any]] = None
    _routing_trail_scheduled: bool = False

    def _broadcast_routing_values(self, snapshot: Dict[str, Any]):
        """Forward the routing evaluator's value snapshot to subscribers.

        Called from the ROS callback thread on every sheet evaluation.
        Leading-edge broadcast every 50ms; intra-window updates are
        coalesced into `_pending_routing_snapshot` and emitted by a
        single deferred trailing-edge fire. Without the trail, the
        last snapshot in a burst (e.g. the all-zero state when the
        operator releases the joystick) silently disappears and the
        UI sticks on whichever intermediate value was the leading-edge
        broadcast.
        """
        if not (self.web_server and self.web_server.ws_handler and self._async_loop):
            return
        import time as _time
        now = _time.monotonic()
        elapsed = now - self._last_routing_values_emit
        if elapsed >= self._ROUTING_VALUES_INTERVAL_S:
            # Leading edge: emit immediately, clear any pending trail
            # so we don't double-send the same snapshot.
            self._last_routing_values_emit = now
            self._pending_routing_snapshot = None
            self._emit_routing_snapshot(snapshot)
            return
        # Inside the throttle window — stash the latest snapshot and
        # schedule a single deferred fire if one isn't already pending.
        self._pending_routing_snapshot = snapshot
        if self._routing_trail_scheduled:
            return
        self._routing_trail_scheduled = True
        delay = max(0.0, self._ROUTING_VALUES_INTERVAL_S - elapsed)
        self._async_loop.call_soon_threadsafe(
            lambda d=delay: self._async_loop.call_later(d, self._flush_routing_trail)
        )

    def _flush_routing_trail(self):
        """Trailing-edge fire of the most-recent pending snapshot."""
        self._routing_trail_scheduled = False
        snap = self._pending_routing_snapshot
        self._pending_routing_snapshot = None
        if snap is None:
            return
        import time as _time
        self._last_routing_values_emit = _time.monotonic()
        self._emit_routing_snapshot(snap)

    def _emit_routing_snapshot(self, snapshot: Dict[str, Any]):
        """Schedule the actual websocket broadcast."""
        handler = self.web_server.ws_handler
        self._async_loop.call_soon_threadsafe(
            lambda s=snapshot: asyncio.create_task(
                handler.broadcast_state('routing_values', s)
            )
        )

    async def stop_async_services(self):
        """Stop async services gracefully."""
        self.get_logger().info('Stopping async services...')

        # Stop discovery service
        if self.discovery_service:
            await self._stop_discovery()

        # Stop LiveLink
        if self.livelink_receiver:
            await self._stop_livelink()

        if self.web_server:
            await self.web_server.stop()

    def start(self):
        """Start the async services in a background thread."""
        self._async_thread = threading.Thread(
            target=self._run_async_loop,
            name='saint-async',
            daemon=True
        )
        self._async_thread.start()
        self.get_logger().info('Async services thread started')

    def shutdown(self):
        """Signal async services to shut down and wait for thread."""
        if self._async_loop and self._shutdown_event:
            # Signal the async loop to stop
            self._async_loop.call_soon_threadsafe(self._shutdown_event.set)

        if self._async_thread and self._async_thread.is_alive():
            self._async_thread.join(timeout=5.0)
            if self._async_thread.is_alive():
                self.get_logger().warn('Async thread did not stop in time')

        # Flush the peripheral logger's disk queue so we don't lose the
        # last second of samples on a clean exit. Daemon thread would
        # die with the process anyway; shutdown() drains the queue first.
        try:
            self.peripheral_logger.shutdown()
        except Exception as e:
            self.get_logger().warn(f'PeripheralLogger shutdown failed: {e}')


def main(args=None):
    """Main entry point for the SAINT.OS server."""
    rclpy.init(args=args)

    node = SaintServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Use threading Event for cooperative shutdown
    shutdown_requested = threading.Event()

    def signal_handler(sig, frame):
        node.get_logger().info('Shutdown signal received')
        shutdown_requested.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.get_logger().info('SAINT.OS Server starting...')
        node.start()  # Start async services thread

        # Spin with timeout to allow checking shutdown flag
        while not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down...')
        node.shutdown()  # Signal async services to stop
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
