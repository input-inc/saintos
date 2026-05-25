"""Shared pytest fixtures + stubs for SAINT.OS server tests.

The server depends on rclpy / aiohttp / ament_index_python at import
time. The host doesn't have any of those, so we stub them into
sys.modules before saint_server imports them — every test that wants
to import a real saint_server module pulls in this fixture file.

This keeps the unit tests host-runnable: `python3 -m pytest server/test`
from a clean machine just works.
"""
from __future__ import annotations

import sys
import types


def _stub(name: str) -> types.ModuleType:
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


# --- rclpy -------------------------------------------------------------
_stub("rclpy")
_node_mod = _stub("rclpy.node")
class _Node:  # noqa: D401  (matches rclpy's Node base class shape we use)
    """Minimal stand-in for rclpy.node.Node."""
_node_mod.Node = _Node

_qos = _stub("rclpy.qos")
# QoSProfile is constructed with keyword args (reliability=, history=,
# depth=, durability=…). The default `type(...)` produced an arg-less
# __init__ which broke server_node.py's module-level CONTROL_QOS
# initialization. Accept any kwargs and just stash them.
class _QoSProfile:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
_qos.QoSProfile = _QoSProfile
for n, vals in (
    ("QoSReliabilityPolicy", {"RELIABLE": 1, "BEST_EFFORT": 2}),
    ("QoSHistoryPolicy",     {"KEEP_LAST": 1, "KEEP_ALL": 2}),
    ("QoSDurabilityPolicy",  {"VOLATILE": 1, "TRANSIENT_LOCAL": 2}),
):
    setattr(_qos, n, type(n, (), vals))

_cb = _stub("rclpy.callback_groups")
_cb.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})

_exec = _stub("rclpy.executors")
_exec.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})

# --- std_msgs ----------------------------------------------------------
_stub("std_msgs")
_std_msg = _stub("std_msgs.msg")
class _String:
    def __init__(self):
        self.data = ""
class _Bool:
    def __init__(self):
        self.data = False
_std_msg.String = _String
_std_msg.Bool = _Bool

# --- ament_index_python ------------------------------------------------
_aip = _stub("ament_index_python")
_aip_pkg = _stub("ament_index_python.packages")
def _missing_pkg(_name):
    raise FileNotFoundError("test stub: ament_index not available")
_aip_pkg.get_package_share_directory = _missing_pkg

# --- aiohttp / WebSockets ----------------------------------------------
# saint_server/__init__.py eagerly imports SaintServerNode which pulls
# in websocket_handler which needs aiohttp.WSMsgType. Stub the bits we
# need so tests run without aiohttp installed.
try:
    import aiohttp  # noqa: F401  (real one wins when installed)
except ImportError:
    _aio = _stub("aiohttp")
    _aio_web = _stub("aiohttp.web")
    _aio.web = _aio_web

    class _WSMsgType:
        TEXT   = 1
        BINARY = 2
        CLOSE  = 8
        CLOSED = 0x100
        ERROR  = 0x101
    _aio.WSMsgType = _WSMsgType
    _aio_web.Request  = type("Request", (), {})
    _aio_web.Response = type("Response", (), {})
    _aio_web.json_response = lambda *a, **kw: None
    _aio_web.FileResponse  = type("FileResponse", (), {})
    _aio_web.WebSocketResponse = type("WebSocketResponse", (), {})
    _aio_web.Application = type("Application", (), {})

try:
    import websockets  # noqa: F401
except ImportError:
    _stub("websockets")

# --- psutil (state_manager imports it at module top) -------------------
try:
    import psutil  # noqa: F401
except ImportError:
    _ps = _stub("psutil")
    _ps.cpu_percent = lambda interval=None: 0.0
    _ps.virtual_memory = lambda: type("M", (), {"percent": 0.0})()
