#!/usr/bin/env python3
"""End-to-end test of the firmware-server sync recovery flow.

What this exercises that pytest can't:

  * **Publisher pre-create at announcement time.**  Unit tests can only
    assert ``_ensure_node_*_publisher`` is called; they can't observe
    real DDS pub/sub matching.  This script starts a Renode-emulated
    RP2040 talking micro-ROS to a real agent talking DDS to a real
    saint_server — so if the publisher race we fixed is still happening
    in some other form, the first Sync to Node will silently drop and
    we'll see the firmware never log "Config received".
  * **Auto-reconcile after an OTA-like flash wipe.**  The script
    factory-resets the node mid-test to simulate the post-OTA scenario
    where the firmware boots into UNADOPTED but the server still has
    it in adopted_nodes.  The server's announcement handler should
    detect the divergence within ~1 s and push the saved config; the
    firmware should log "Config received" / "Config applied OK" /
    "Config saved to flash" within the 10 s cooldown window.
  * **Sync to Node round-trip.**  Drives the management WebSocket
    action the dashboard's Sync button calls, then asserts the
    matching firmware-side log lines arrive.

This is intentionally a single self-contained script — not a pytest
file — because the test fixtures (Renode VM, micro-ROS agent, real
saint_server) take 5-10 s to spin up and don't fit pytest's
per-function lifecycle.

Prerequisites (the script checks each and exits with a clear message
if any are missing):

  * Renode installed.  Set ``RENODE_PATH`` if not in the default
    macOS / Linux locations.
  * ``ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888``
    running in another shell.  This needs a sourced ROS2 jazzy/humble
    environment.
  * SAINT.OS server running on localhost (default
    ``ws://localhost:9090``).  Override with ``--ws-url``.
  * RP2040 simulation firmware built: ``cd firmware/rp2040 &&
    ./build.sh sim``.  The .elf is loaded into Renode.

Usage::

    # Just verify prerequisites — does not start anything.
    ./test_sync_recovery.py --check-only

    # Full run.
    ./test_sync_recovery.py [--ws-url ws://localhost:9090] [--password 12345]

Exit codes:
  0 — every phase passed.
  1 — at least one assertion failed (details printed).
  2 — prerequisites missing.
  3 — internal error setting up the test (Renode failed to launch, etc.).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
# SAINT.OS runs a single HTTP server on port 80 that upgrades to a
# WebSocket at /api/ws. Same URL the controller's first-launch dialog
# defaults to (controller/README.md "ws://opensaint.local/api/ws").
DEFAULT_WS_URL = "ws://localhost:80/api/ws"
DEFAULT_PASSWORD = "12345"
TEST_NODE_ID = "rp2040_synctest"
RECONCILE_WAIT_S = 15.0
SYNC_WAIT_S = 10.0
ADOPT_WAIT_S = 30.0


# ─── Prerequisite checks ───────────────────────────────────────────────

def _check_renode() -> Optional[str]:
    candidates = [
        os.environ.get("RENODE_PATH"),
        os.path.expanduser("~/Applications/Renode.app/Contents/MacOS/Renode"),
        shutil.which("renode"),
        shutil.which("Renode"),
    ]
    for c in candidates:
        if c and Path(c).exists():
            return None
    return ("Renode not found. Install from https://renode.io or set "
            "RENODE_PATH to its binary.")


def _check_agent_running() -> Optional[str]:
    """The micro-ROS agent listens on UDP 8888 by default.

    The agent binary registers in lsof as ``MicroXRCEAgent`` (or
    ``MicroXRCE`` truncated), not ``micro_ros_agent`` — the rclpy
    launch wrapper exec()'s the underlying C++ binary. Match anything
    that's actually bound to the port instead of grepping for a
    specific process name.
    """
    try:
        out = subprocess.run(
            ["lsof", "-nP", "-iUDP:8888"],
            capture_output=True, text=True, timeout=5,
        )
        # `lsof -i` exits 0 only when something is bound. The header
        # row alone counts as no match; require at least one data row.
        lines = [ln for ln in (out.stdout or "").splitlines() if ln.strip()]
        if out.returncode == 0 and len(lines) >= 2:
            return None
    except (subprocess.SubprocessError, FileNotFoundError):
        pass
    return ("micro-ROS agent doesn't appear to be listening on UDP 8888. "
            "Run: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888")


def _check_sim_firmware() -> Optional[str]:
    """The sim build of saint_node.elf must exist for Renode to load it.

    Two paths are valid:
      * firmware/rp2040/build_sim/saint_node.elf — the bare cmake output
      * firmware/rp2040/install/simulation/saint_node.elf — what
        `make install_sim` (and the Docker bind mount) put in place.
    The .resc Renode loads references the install/simulation path
    specifically, so that's the one we actually need; the build_sim
    fallback covers a hand-cmake workflow where the operator hasn't
    run install_sim yet.
    """
    install = REPO_ROOT / "firmware" / "rp2040" / "install" / "simulation" / "saint_node.elf"
    build = REPO_ROOT / "firmware" / "rp2040" / "build_sim" / "saint_node.elf"
    if install.exists() or build.exists():
        return None
    return (f"Sim firmware not installed: {install} missing. "
            "Build + install with: cd firmware/rp2040 && ./build.sh sim "
            "&& (cd build_sim && make install_sim)")


def _check_websockets_lib() -> Optional[str]:
    try:
        import websockets  # noqa: F401
        return None
    except ImportError:
        return ("Python 'websockets' library not installed. "
                "Install: pip3 install websockets")


def _check_server_reachable(ws_url: str) -> Optional[str]:
    """Quick TCP probe of the server's WebSocket port."""
    import socket
    from urllib.parse import urlparse
    parsed = urlparse(ws_url)
    host = parsed.hostname or "localhost"
    port = parsed.port or (443 if parsed.scheme == "wss" else 80)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    try:
        sock.connect((host, port))
        return None
    except (OSError, socket.timeout) as e:
        return f"SAINT.OS server not reachable at {ws_url} ({e})."
    finally:
        sock.close()


def check_prereqs(ws_url: str, need_server: bool = True) -> list[str]:
    """Verify everything the test needs is in place.

    ``need_server=False`` skips the websockets-lib + server-reachable
    probes — useful for the --no-server smoke-test mode that only
    proves the firmware build + agent + Renode path is alive.
    """
    problems: list[str] = []
    checks = [_check_renode, _check_agent_running, _check_sim_firmware]
    if need_server:
        checks += [_check_websockets_lib]
    for check in checks:
        msg = check()
        if msg:
            problems.append(msg)
    if need_server:
        msg = _check_server_reachable(ws_url)
        if msg:
            problems.append(msg)
    return problems


# ─── WebSocket client ──────────────────────────────────────────────────

class WsClient:
    """Thin async wrapper that speaks the SAINT.OS management protocol."""

    def __init__(self, ws_url: str, password: str):
        self.ws_url = ws_url
        self.password = password
        self.ws = None
        self._req_id = 0
        self._pending: dict[str, asyncio.Future] = {}
        self._node_log_queue: asyncio.Queue = asyncio.Queue()
        self._sync_status_queue: asyncio.Queue = asyncio.Queue()
        self._reader_task: Optional[asyncio.Task] = None

    async def connect(self):
        import websockets
        self.ws = await websockets.connect(self.ws_url)
        self._reader_task = asyncio.create_task(self._reader())
        # Login
        await self.ws.send(json.dumps({
            "type": "auth", "action": "login", "password": self.password,
        }))

    async def _reader(self):
        async for raw in self.ws:
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            # Correlated response
            if "id" in msg and msg["id"] in self._pending:
                fut = self._pending.pop(msg["id"])
                if not fut.done():
                    fut.set_result(msg)
                continue
            # Topic broadcasts
            if msg.get("type") == "state":
                node = msg.get("node", "")
                if node.startswith("node_log/"):
                    await self._node_log_queue.put((node, msg.get("data")))
                elif node.startswith("sync_status/"):
                    await self._sync_status_queue.put((node, msg.get("data")))

    async def request(self, action: str, params: dict | None = None,
                      msg_type: str = "management", timeout: float = 10.0) -> dict:
        self._req_id += 1
        rid = f"e2e_{self._req_id}"
        fut: asyncio.Future = asyncio.get_event_loop().create_future()
        self._pending[rid] = fut
        await self.ws.send(json.dumps({
            "id": rid, "type": msg_type, "action": action, "params": params or {},
        }))
        return await asyncio.wait_for(fut, timeout=timeout)

    async def subscribe(self, topics: list[str], rate_hz: int = 5):
        await self.ws.send(json.dumps({
            "type": "subscribe", "action": "subscribe",
            "params": {"topics": topics, "rate_hz": rate_hz},
        }))

    async def wait_for_node_log_containing(
        self, node_id: str, needle: str, timeout: float,
    ) -> Optional[dict]:
        topic = f"node_log/{node_id}"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                seen_topic, entry = await asyncio.wait_for(
                    self._node_log_queue.get(), timeout=remaining,
                )
            except asyncio.TimeoutError:
                return None
            if seen_topic != topic:
                continue
            text = (entry or {}).get("text", "")
            if needle in text:
                return entry
        return None

    async def close(self):
        if self._reader_task:
            self._reader_task.cancel()
        if self.ws:
            await self.ws.close()


# ─── Test phases ───────────────────────────────────────────────────────

def _run_node_manager(*args: str) -> subprocess.CompletedProcess:
    """Shell out to node_manager.py with the given args."""
    cmd = [sys.executable, str(SCRIPT_DIR / "node_manager.py"), *args]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=30)


def _say(phase: str, msg: str, ok: bool = True):
    sym = "✓" if ok else "✗"
    print(f"[{phase:14s}] {sym} {msg}")


def run_smoke_test() -> int:
    """No-server mode: build was OK + agent is up + sim node boots and
    establishes its micro-ROS session. This is what's verifiable on a
    Mac dev box without a Linux server in the loop.

    Reads the node's dedicated UART-capture file (set up by node_manager
    via Renode's CreateFileBackend on uart0) for firmware printf output.
    Falls back to the operational log only when the UART file doesn't
    exist yet, in which case we'll likely see only Renode warnings.
    """
    failures = 0
    node_created = False
    uart_log = SCRIPT_DIR / "logs" / f"{TEST_NODE_ID}.uart.log"
    op_log = SCRIPT_DIR / "logs" / f"{TEST_NODE_ID}.log"
    log_path = uart_log

    print("Smoke test (no server) — sim node bring-up only.\n")
    try:
        _say("setup", f"creating sim node {TEST_NODE_ID}")
        r = _run_node_manager("create", TEST_NODE_ID, "--type", "rp2040")
        if r.returncode != 0 and "already exists" not in r.stderr:
            _say("setup", f"create failed: {r.stderr.strip()}", ok=False)
            return 3
        node_created = True

        _say("setup", "starting node in Renode")
        r = _run_node_manager("start", TEST_NODE_ID)
        if r.returncode != 0:
            _say("setup", f"start failed: {r.stderr.strip()}", ok=False)
            return 3

        # Each marker we expect, in approximate order. Times allow for
        # Renode's startup + the firmware's discovery + micro-ROS init.
        markers = [
            ("micro-ROS initialized", 60.0),
            ("Successfully subscribed to: /saint/nodes/", 30.0),
            ("Node ready", 30.0),
        ]
        found = {}
        for marker, timeout in markers:
            _say("phase 1", f"waiting up to {timeout:.0f}s for '{marker}'…")
            deadline = time.monotonic() + timeout
            hit = False
            while time.monotonic() < deadline:
                if log_path.exists():
                    try:
                        content = log_path.read_text(errors="replace")
                    except OSError:
                        content = ""
                    if marker in content:
                        hit = True
                        break
                time.sleep(1.0)
            if hit:
                _say("phase 1", f"saw '{marker}'")
                found[marker] = True
            else:
                _say("phase 1",
                     f"never saw '{marker}' — check {log_path}",
                     ok=False)
                failures += 1
                # Print last 20 lines of the log to help diagnose.
                if log_path.exists():
                    print(f"  ─── last 20 lines of {log_path} ───")
                    for line in log_path.read_text(errors="replace").splitlines()[-20:]:
                        print(f"  {line}")
                break

    finally:
        if node_created:
            if failures:
                # Stop but don't remove — leaves the log + .resc behind
                # so an operator can diagnose what the firmware actually
                # printed (or didn't). Subsequent runs auto-clean via
                # the pre-remove in the setup phase.
                _say("teardown",
                     f"stopping sim node — log preserved at {log_path}")
                _run_node_manager("stop", TEST_NODE_ID)
            else:
                _say("teardown", "stopping + removing sim node")
                _run_node_manager("stop", TEST_NODE_ID)
                _run_node_manager("remove", TEST_NODE_ID)

    print()
    if failures:
        print(f"=== smoke test failed ({failures} marker(s) missing) ===")
        return 1
    print("=== smoke test passed — firmware/agent path is alive ===")
    print("(Server-dependent phases skipped. Re-run without --no-server")
    print(" once you have a SAINT.OS server reachable.)")
    return 0


async def run_test(ws_url: str, password: str) -> int:
    failures = 0
    node_created = False

    print(f"E2E sync-recovery test — target server {ws_url}\n")

    try:
        # Pre-clean: previous compose-up runs leave the node in
        # nodes.json inside the container, which makes the create
        # below fail with "already exists". Best-effort remove so
        # every run starts from a known state.
        _run_node_manager("remove", TEST_NODE_ID)

        # Phase 1: create + start sim node. node_manager prints
        # "already exists" to STDOUT (not stderr) and exits non-zero
        # on the dup case, so check both streams.
        _say("setup", f"creating sim node {TEST_NODE_ID}")
        r = _run_node_manager("create", TEST_NODE_ID, "--type", "rp2040")
        combined = (r.stdout or "") + (r.stderr or "")
        if r.returncode != 0 and "already exists" not in combined:
            _say("setup",
                 f"create failed: {combined.strip() or '(no output)'}",
                 ok=False)
            return 3
        node_created = True

        _say("setup", "starting node in Renode")
        r = _run_node_manager("start", TEST_NODE_ID)
        if r.returncode != 0:
            combined = (r.stdout or "") + (r.stderr or "")
            _say("setup", f"start failed: {combined.strip()}", ok=False)
            return 3

        # Phase 2: connect to server WebSocket
        client = WsClient(ws_url, password)
        await client.connect()
        await asyncio.sleep(0.5)  # let auth_result settle
        _say("setup", "connected + authenticated to server")

        await client.subscribe([
            f"node_log/{TEST_NODE_ID}",
            f"sync_status/{TEST_NODE_ID}",
        ], rate_hz=10)

        # Phase 3: wait for unadopted announcement
        _say("phase 1", "waiting for node to announce as UNADOPTED…")
        deadline = time.monotonic() + ADOPT_WAIT_S
        unadopted_seen = False
        while time.monotonic() < deadline:
            resp = await client.request("list_unadopted")
            nodes = (resp.get("data") or {}).get("nodes") or []
            if any(n.get("node_id") == TEST_NODE_ID for n in nodes):
                unadopted_seen = True
                break
            await asyncio.sleep(1.0)
        if not unadopted_seen:
            _say("phase 1", "node never announced — agent or sim failure",
                 ok=False)
            failures += 1
            return failures
        _say("phase 1", "node announced")

        # Phase 4: adopt + sync
        _say("phase 2", "adopting node")
        await client.request("adopt_node", {
            "node_id": TEST_NODE_ID,
            "role": "cradle_base",
            "display_name": "SyncTest",
            "board_id": "feather_rp2040_w5500",
        })
        _say("phase 2", "syncing initial config (publisher pre-create check)")
        await client.request("sync_node_peripherals", {"node_id": TEST_NODE_ID})

        # If the publisher pre-create fix is working, "Config received"
        # arrives in <SYNC_WAIT_S; if not, this times out.
        entry = await client.wait_for_node_log_containing(
            TEST_NODE_ID, "Config received", SYNC_WAIT_S)
        if entry is None:
            _say("phase 2",
                 f"firmware never logged 'Config received' within {SYNC_WAIT_S}s "
                 "— publisher pre-create / DDS race still broken",
                 ok=False)
            failures += 1
        else:
            _say("phase 2", "firmware acknowledged Sync to Node")

            saved = await client.wait_for_node_log_containing(
                TEST_NODE_ID, "Config saved to flash", SYNC_WAIT_S)
            if saved is None:
                _say("phase 2",
                     "got 'Config received' but not 'Config saved to flash' "
                     "— check for 'Config apply failed' in node logs",
                     ok=False)
                failures += 1
            else:
                _say("phase 2", "config persisted to flash")

        # Phase 5: simulate OTA — reset node (clears flash on Renode sim),
        # observe auto-reconcile.
        _say("phase 3", "resetting node to simulate post-OTA flash wipe")
        _run_node_manager("reset", TEST_NODE_ID)
        _run_node_manager("start", TEST_NODE_ID)

        _say("phase 3",
             f"waiting up to {RECONCILE_WAIT_S}s for server auto-reconcile")
        entry = await client.wait_for_node_log_containing(
            TEST_NODE_ID,
            "Adopted node announced UNADOPTED — re-pushing peripheral config",
            RECONCILE_WAIT_S)
        if entry is None:
            _say("phase 3",
                 "server never logged a reconcile push — auto-recovery broken",
                 ok=False)
            failures += 1
        else:
            _say("phase 3", "server detected divergence + pushed config")

            entry = await client.wait_for_node_log_containing(
                TEST_NODE_ID, "Config saved to flash", SYNC_WAIT_S)
            if entry is None:
                _say("phase 3",
                     "reconcile push reached firmware but flash-save didn't follow",
                     ok=False)
                failures += 1
            else:
                _say("phase 3", "node healed itself end-to-end")

        await client.close()

    finally:
        if node_created:
            if failures:
                # Preserve log + .resc so the operator can diagnose
                # which hop failed. Next run's pre-clean removes them.
                _say("teardown",
                     "stopping sim node (logs preserved for diagnosis)")
                _run_node_manager("stop", TEST_NODE_ID)
            else:
                _say("teardown", "stopping + removing sim node")
                _run_node_manager("stop", TEST_NODE_ID)
                _run_node_manager("remove", TEST_NODE_ID)

    print()
    if failures:
        print(f"=== {failures} phase(s) failed ===")
    else:
        print("=== all phases passed ===")
    return 1 if failures else 0


# ─── Entry point ───────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ws-url", default=DEFAULT_WS_URL,
                        help=f"SAINT.OS WebSocket URL (default {DEFAULT_WS_URL})")
    parser.add_argument("--password", default=DEFAULT_PASSWORD,
                        help="Server auth password")
    parser.add_argument("--check-only", action="store_true",
                        help="Only verify prerequisites; do not run the test")
    parser.add_argument("--no-server", action="store_true",
                        help="Skip server-dependent phases — only verify "
                             "that the firmware sim boots and joins the agent. "
                             "Useful on a Mac dev box where ROS2 (and so "
                             "saint_server) doesn't run natively.")
    args = parser.parse_args()

    print("Checking prerequisites…")
    problems = check_prereqs(args.ws_url, need_server=not args.no_server)
    if problems:
        print("\nPrerequisites missing:")
        for p in problems:
            print(f"  - {p}")
        return 2
    print("All prerequisites OK.\n")

    if args.check_only:
        return 0

    try:
        if args.no_server:
            return run_smoke_test()
        return asyncio.run(run_test(args.ws_url, args.password))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
