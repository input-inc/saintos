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
RECONCILE_WAIT_S = 15.0
SYNC_WAIT_S = 10.0
ADOPT_WAIT_S = 30.0

# Per-node-type defaults for the Renode-sim path. Each entry pins:
#   - default_node_id: what node_manager.py names the sim node
#   - board_id:        what the harness passes to /adopt_node (must match
#                      a YAML under server/config/boards/<type>/)
#   - firmware_path:   the .elf the .resc loads — also the file the
#                      prereq check looks for
# `--node-type` selects between rp2040 (current green path) and
# teensy41 (which still uses node_manager's same create/start/reset
# verbs — only the firmware ELF and renode platform are different).
NODE_TYPES = {
    "rp2040": {
        "default_node_id": "rp2040_synctest",
        "board_id":        "feather_rp2040_w5500",
        "firmware_paths": [
            REPO_ROOT / "firmware/rp2040/install/simulation/saint_node.elf",
            REPO_ROOT / "firmware/rp2040/build_sim/saint_node.elf",
        ],
        "firmware_build_hint": (
            "cd firmware/rp2040 && ./build.sh sim "
            "&& (cd build_sim && make install_sim)"
        ),
    },
    "teensy41": {
        "default_node_id": "teensy41_synctest",
        "board_id":        "teensy41_native_eth",
        "firmware_paths": [
            REPO_ROOT / "firmware/teensy41/build/simulation/firmware.elf",
        ],
        "firmware_build_hint": (
            "cd firmware/teensy41 && ~/.platformio/penv/bin/pio run -e simulation"
        ),
    },
}

# Back-compat default for callers that don't pass --node-type yet.
TEST_NODE_ID = NODE_TYPES["rp2040"]["default_node_id"]


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


def _check_sim_firmware(node_type: str = "rp2040") -> Optional[str]:
    """The sim build of the firmware ELF must exist for Renode to load it.

    Looks at every path the .resc for `node_type` might reference (see
    `NODE_TYPES[node_type]["firmware_paths"]`) and accepts the first
    one that exists. The RP2040 path covers both the bare cmake output
    and `make install_sim`'s install dir; the Teensy path is a single
    PlatformIO `build/simulation` slot. If none exist, the error
    message names the first (canonical) path and quotes the per-type
    build hint.
    """
    cfg = NODE_TYPES.get(node_type)
    if cfg is None:
        return f"Unknown --node-type '{node_type}' (expected one of: {list(NODE_TYPES)})"
    for path in cfg["firmware_paths"]:
        if Path(path).exists():
            return None
    primary = cfg["firmware_paths"][0]
    return (f"Sim firmware not built: {primary} missing. "
            f"Build with: {cfg['firmware_build_hint']}")


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


def check_prereqs(ws_url: str, need_server: bool = True,
                  node_type: str = "rp2040") -> list[str]:
    """Verify everything the test needs is in place.

    ``need_server=False`` skips the websockets-lib + server-reachable
    probes — useful for the --no-server smoke-test mode that only
    proves the firmware build + agent + Renode path is alive.
    ``node_type`` controls which firmware ELF is required to exist.
    """
    problems: list[str] = []
    for check in (_check_renode, _check_agent_running):
        msg = check()
        if msg:
            problems.append(msg)
    msg = _check_sim_firmware(node_type)
    if msg:
        problems.append(msg)
    if need_server:
        msg = _check_websockets_lib()
        if msg:
            problems.append(msg)
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
            if os.environ.get("HARNESS_TRACE_STATE"):
                text_preview = ""
                if msg.get("type") == "activity":
                    text_preview = f" text={msg.get('text', '')[:80]!r}"
                elif msg.get("type") == "state":
                    text_preview = f" data={str(msg.get('data', ''))[:80]!r}"
                print(f"  [ws trace] type={msg.get('type')!r} node={msg.get('node')!r}{text_preview}",
                      flush=True)
            # Per-node `state` frames on node_log/<id> are the
            # canonical channel, but the server ALSO mirrors every log
            # event to the unconditional `activity` broadcast. The
            # state path was empirically silent in the docker e2e
            # (likely a subscription-filter quirk we haven't pinned
            # down yet) so we treat both as valid signals — text
            # matching against either gets us through the test.
            if msg.get("type") == "state":
                node = msg.get("node", "")
                if node.startswith("node_log/"):
                    await self._node_log_queue.put((node, msg.get("data")))
                elif node.startswith("sync_status/"):
                    await self._sync_status_queue.put((node, msg.get("data")))
            elif msg.get("type") == "activity":
                # Bridge activity → node_log queue with a synthetic
                # node prefix so the existing wait helpers (which
                # filter by "node_log/<id>") still match. The text is
                # already prefixed with [+uptime.s] for firmware logs,
                # so substring matches against "Config received" /
                # "Config saved to flash" / "Adopted node announced
                # UNADOPTED — re-pushing peripheral config" all work.
                synthetic_entry = {
                    "text": msg.get("text", ""),
                    "level": msg.get("level", "info"),
                    "time": msg.get("timestamp"),
                }
                # The activity stream isn't per-node, so feed it under
                # both possible node_log/ queue keys the wait helpers
                # might look up. wait_for_node_log_containing checks
                # the topic prefix, so a non-prefixed bucket would be
                # missed — use a sentinel topic the helper accepts.
                await self._node_log_queue.put(
                    ("node_log/activity", synthetic_entry))

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
        """Wait for a log line whose text contains `needle`.

        Accepts entries from either:
        - The per-node `node_log/<id>` state-frame stream (preferred,
          matches dashboard's Logs-tab feed).
        - The unconditional `activity` broadcast bridged under the
          sentinel topic `node_log/activity` (covers the case where
          state-frame delivery isn't firing for some reason).
        """
        per_node_topic = f"node_log/{node_id}"
        activity_topic = "node_log/activity"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                seen_topic, entry = await asyncio.wait_for(
                    self._node_log_queue.get(), timeout=remaining,
                )
            except asyncio.TimeoutError:
                return None
            if seen_topic not in (per_node_topic, activity_topic):
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


def _publish_ros2_string(topic: str, data: str) -> bool:
    """Publish a one-shot std_msgs/String to a ROS2 topic via the CLI.

    Used by the fake-firmware test path to inject test-only control
    messages (e.g. "lose_config" on /test/fake_firmware/<id>/control)
    that exercise the server's auto-reconcile flow. Shelling out is
    cheaper than pulling rclpy into the harness; the CLI itself comes
    with the ROS2 install the container already has sourced.
    """
    # ros2 topic pub --once expects YAML-formatted args; escape inner
    # quotes so the JSON string passes through cleanly.
    yaml_arg = f"data: \"{data.replace(chr(34), chr(92) + chr(34))}\""
    cmd = [
        "ros2", "topic", "pub", "--once",
        topic, "std_msgs/msg/String", yaml_arg,
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    return r.returncode == 0


def _say(phase: str, msg: str, ok: bool = True):
    sym = "✓" if ok else "✗"
    print(f"[{phase:14s}] {sym} {msg}")


def run_smoke_test(node_type: str = "rp2040", node_id: Optional[str] = None) -> int:
    """No-server mode: build was OK + agent is up + sim node boots and
    establishes its micro-ROS session. This is what's verifiable on a
    Mac dev box without a Linux server in the loop.

    Reads the node's dedicated UART-capture file (set up by node_manager
    via Renode's CreateFileBackend on the firmware's console UART) for
    firmware printf output. Falls back to the operational log only when
    the UART file doesn't exist yet, in which case we'll likely see
    only Renode warnings.
    """
    failures = 0
    node_created = False
    target_node_id = node_id or NODE_TYPES[node_type]["default_node_id"]
    uart_log = SCRIPT_DIR / "logs" / f"{target_node_id}.uart.log"
    op_log = SCRIPT_DIR / "logs" / f"{target_node_id}.log"
    log_path = uart_log

    print(f"Smoke test (no server) — sim node bring-up only — type={node_type}.\n")
    try:
        # Pre-clean: previous runs may have left the node behind. node_manager
        # treats "remove" as best-effort.
        _run_node_manager("remove", target_node_id)

        _say("setup", f"creating sim node {target_node_id}")
        r = _run_node_manager("create", target_node_id, "--type", node_type)
        if r.returncode != 0 and "already exists" not in r.stderr:
            _say("setup", f"create failed: {r.stderr.strip()}", ok=False)
            return 3
        node_created = True

        _say("setup", "starting node in Renode")
        r = _run_node_manager("start", target_node_id)
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
                _run_node_manager("stop", target_node_id)
            else:
                _say("teardown", "stopping + removing sim node")
                _run_node_manager("stop", target_node_id)
                _run_node_manager("remove", target_node_id)

    print()
    if failures:
        print(f"=== smoke test failed ({failures} marker(s) missing) ===")
        return 1
    print("=== smoke test passed — firmware/agent path is alive ===")
    print("(Server-dependent phases skipped. Re-run without --no-server")
    print(" once you have a SAINT.OS server reachable.)")
    return 0


async def run_test(ws_url: str, password: str,
                    fake_firmware: bool = False,
                    existing_node: bool = False,
                    node_id: Optional[str] = None,
                    node_type: str = "rp2040") -> int:
    failures = 0
    node_created = False

    type_cfg = NODE_TYPES[node_type]
    target_node_id = node_id or type_cfg["default_node_id"]
    # The host-assigned name we register with node_manager. Stays put
    # even when the firmware announces under a different chip-ID-derived
    # name (sim chip-IDs read 0; see Phase 1 below). Used for every
    # node_manager.py invocation: create / start / reset / stop / remove.
    sim_node_id = target_node_id
    board_id = type_cfg["board_id"]
    if existing_node:
        flavor = "existing-node"
    elif fake_firmware:
        flavor = "fake-firmware"
    else:
        flavor = f"Renode/{node_type}"
    print(f"E2E sync-recovery test — {flavor} — target {ws_url}\n")
    print(f"  test node: {target_node_id} (board_id={board_id})\n")

    try:
        if existing_node:
            # ── Existing-hardware path: the node is already adopted on
            # the target server. Skip the Renode bring-up + adoption
            # dance; just connect, sync, and watch for the firmware's
            # log lines. Used to reproduce real-hardware regressions
            # that the Renode + fake-firmware paths can't surface.
            _say("setup", f"targeting existing adopted node {target_node_id}")
        elif not fake_firmware:
            # ── Renode-sim path: orchestrate the sim node via node_manager.
            # Pre-clean: previous compose-up runs leave the node in
            # nodes.json inside the container, which makes the create
            # below fail with "already exists". Best-effort remove so
            # every run starts from a known state.
            _run_node_manager("remove", target_node_id)

            # Phase 1: create + start sim node. node_manager prints
            # "already exists" to STDOUT (not stderr) and exits non-zero
            # on the dup case, so check both streams.
            _say("setup", f"creating sim node {target_node_id} (type={node_type})")
            r = _run_node_manager("create", target_node_id, "--type", node_type)
            combined = (r.stdout or "") + (r.stderr or "")
            if r.returncode != 0 and "already exists" not in combined:
                _say("setup",
                     f"create failed: {combined.strip() or '(no output)'}",
                     ok=False)
                return 3
            node_created = True

            _say("setup", "starting node in Renode")
            r = _run_node_manager("start", target_node_id)
            if r.returncode != 0:
                combined = (r.stdout or "") + (r.stderr or "")
                _say("setup", f"start failed: {combined.strip()}", ok=False)
                return 3
        else:
            # ── Fake-firmware path: the python stand-in is already
            # running (entrypoint.sh started it before launching the
            # harness). Nothing to start here; just verify announcements
            # are flowing.
            _say("setup", f"using fake firmware (already running)")

        # Phase 2: connect to server WebSocket
        client = WsClient(ws_url, password)
        await client.connect()
        await asyncio.sleep(0.5)  # let auth_result settle
        _say("setup", "connected + authenticated to server")

        await client.subscribe([
            f"node_log/{target_node_id}",
            f"sync_status/{target_node_id}",
        ], rate_hz=10)

        if not existing_node:
            # Phase 1: wait for unadopted announcement.
            #
            # In simulation, the firmware names itself from the chip's
            # unique-ID fuses (OCOTP on Teensy, ID_FLASH on RP2040), which
            # read back as zero under Renode — every fresh sim node
            # announces as e.g. "teensy41_0000000000000000" / "rp2040_…0",
            # not the host-assigned name we passed to node_manager.
            # node_manager's NodeId on the persistent_storage peripheral
            # only sets the backing filename, not the firmware's view of
            # itself. So we prefer an exact match (in case someone
            # pre-populated storage with the assigned ID) but fall back
            # to the first announcement whose node_id starts with
            # "<chip_family>_" — that's the firmware we just booted.
            _say("phase 1", "waiting for node to announce as UNADOPTED…")
            deadline = time.monotonic() + ADOPT_WAIT_S
            chip_prefix = f"{node_type}_"
            announced = None
            while time.monotonic() < deadline:
                resp = await client.request("list_unadopted")
                nodes = (resp.get("data") or {}).get("nodes") or []
                announced = next(
                    (n for n in nodes if n.get("node_id") == target_node_id),
                    None,
                )
                if announced is None:
                    announced = next(
                        (n for n in nodes
                         if (n.get("node_id") or "").startswith(chip_prefix)),
                        None,
                    )
                if announced:
                    break
                await asyncio.sleep(1.0)
            if announced is None:
                _say("phase 1", "node never announced — agent or sim failure",
                     ok=False)
                failures += 1
                return failures
            announced_id = announced.get("node_id") or target_node_id
            if announced_id != target_node_id:
                _say("phase 1",
                     f"node announced as {announced_id} "
                     f"(host-assigned {target_node_id} ignored — sim chip-ID "
                     f"isn't modelled). Rewiring subscriptions.")
                # Resubscribe under the announced node_id; the prior
                # subscription is harmless to leave behind.
                target_node_id = announced_id
                await client.subscribe([
                    f"node_log/{target_node_id}",
                    f"sync_status/{target_node_id}",
                ], rate_hz=10)
            else:
                _say("phase 1", "node announced")

            # Phase 2: adopt + sync
            _say("phase 2", f"adopting node (board_id={board_id})")
            await client.request("adopt_node", {
                "node_id": target_node_id,
                "role": "cradle_base",
                "display_name": "SyncTest",
                "board_id": board_id,
            })

        # Give the agent's just-created DDS endpoints time to discover
        # and match with the server's subscribers. Without this,
        # firmware /log publishes that fire from inside the very first
        # config_subscription_callback (immediately after sync below)
        # can land on the agent before any DDS reader is bound, and
        # the agent forwards them as empty payloads — visible at the
        # server as `(malformed log frame ... char 0)) ''` warnings.
        # 1 s is the typical DDS unicast-discovery + readers-matched
        # window; longer is unnecessary, shorter is racy.
        await asyncio.sleep(1.0)

        _say("phase 2", "syncing initial config (publisher pre-create check)")
        await client.request("sync_node_peripherals",
                             {"node_id": target_node_id})

        # If the publisher pre-create fix is working, "Config received"
        # arrives in <SYNC_WAIT_S; if not, this times out.
        entry = await client.wait_for_node_log_containing(
            target_node_id, "Config received", SYNC_WAIT_S)
        if entry is None:
            _say("phase 2",
                 f"firmware never logged 'Config received' within {SYNC_WAIT_S}s "
                 "— publisher pre-create / DDS race still broken",
                 ok=False)
            failures += 1
        else:
            _say("phase 2", "firmware acknowledged Sync to Node")

            saved = await client.wait_for_node_log_containing(
                target_node_id, "Config saved to flash", SYNC_WAIT_S)
            if saved is None:
                _say("phase 2",
                     "got 'Config received' but not 'Config saved to flash' "
                     "— check for 'Config apply failed' in node logs",
                     ok=False)
                failures += 1
            else:
                _say("phase 2", "config persisted to flash")

        if existing_node:
            # Skip Phase 3 on real hardware: factory-resetting a live
            # node would erase the operator's actual config. Phase 2's
            # sync result is what this mode is for.
            _say("phase 3", "skipped (existing hardware — no reset)")
            await client.close()
            print()
            if failures:
                print(f"=== {failures} phase(s) failed ===")
            else:
                print("=== all phases passed ===")
            return 1 if failures else 0

        # Phase 3: simulate post-OTA flash wipe, observe auto-reconcile.
        # Two paths depending on what's hosting the firmware:
        #   - Renode sim: reset the VM, which clears the persistent
        #     storage backend → boot back to UNADOPTED.
        #   - Fake firmware: publish to the test-only control channel
        #     so the python node drops its state to UNADOPTED without
        #     telling the server. This is what makes the auto-reconcile
        #     path observable (server still thinks the node is adopted).
        _say("phase 3", "simulating post-OTA flash wipe on the node")
        if fake_firmware:
            _publish_ros2_string(
                f"/test/fake_firmware/{target_node_id}/control",
                json.dumps({"action": "lose_config"}),
            )
        else:
            # node_manager only knows the sim-side host-assigned name
            # (sim_node_id) — target_node_id may have been swapped to the
            # firmware's announced id in Phase 1.
            _run_node_manager("reset", sim_node_id)
            _run_node_manager("start", sim_node_id)

        _say("phase 3",
             f"waiting up to {RECONCILE_WAIT_S}s for server auto-reconcile")
        entry = await client.wait_for_node_log_containing(
            target_node_id,
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
                target_node_id, "Config saved to flash", SYNC_WAIT_S)
            if entry is None:
                _say("phase 3",
                     "reconcile push reached firmware but flash-save didn't follow",
                     ok=False)
                failures += 1
            else:
                _say("phase 3", "node healed itself end-to-end")

        # ── Phase 4: basic control — identify_node ───────────────────
        # Exercises the /command subscription path (RELIABLE one-shots).
        # `identify_node` is the cheapest action that ends in a
        # firmware-side side effect we can observe: the firmware's
        # `dispatch_action_buffer` for action=="identify" calls
        # `led_identify(5)`, which Serial.printf's `LED identify:
        # flashing 5 times`. If /command dispatches correctly, that
        # line goes via saint_log_emit_local → Serial → /log topic
        # (when called from saint_log_publish wrappers) — but identify
        # only Serial-prints, so we have to check the UART log
        # file directly, not /log. node_manager owns the UART file
        # capture so the path is stable across sim runs.
        _say("phase 4", "issuing identify_node (RELIABLE /command path)")
        identify_uart_log = SCRIPT_DIR / "logs" / f"{sim_node_id}.uart.log"
        # Snapshot file length so we only match lines emitted AFTER
        # the WS call. Without this, a stale "LED identify" from a
        # previous run trivially passes.
        identify_offset = (identify_uart_log.stat().st_size
                            if identify_uart_log.exists() else 0)
        await client.request("identify_node", {"node_id": target_node_id})
        identify_deadline = time.monotonic() + SYNC_WAIT_S
        identify_seen = False
        while time.monotonic() < identify_deadline:
            if identify_uart_log.exists():
                with open(identify_uart_log, "rb") as f:
                    f.seek(identify_offset)
                    if b"LED identify" in f.read():
                        identify_seen = True
                        break
            await asyncio.sleep(0.5)
        if identify_seen:
            _say("phase 4", "firmware processed identify (LED flash logged)")
        else:
            _say("phase 4",
                 f"firmware never logged 'LED identify' within {SYNC_WAIT_S}s "
                 "— /command subscription may not be dispatching",
                 ok=False)
            failures += 1

        # ── Phase 5: OTA update flow ─────────────────────────────────
        # Issues firmware_update via the /command path. The server's
        # force_firmware_update assembles the payload (version + size
        # + crc32 of the staged firmware) and publishes it to
        # /saint/nodes/<id>/command. The firmware's
        # `handle_firmware_update` Serial.printf's "FIRMWARE UPDATE
        # REQUESTED" before deciding whether to apply it. On
        # SIMULATION it halts after — node_manager would relaunch
        # with the new ELF in a real OTA flow.
        _say("phase 5", "issuing force_firmware_update (OTA path)")
        ota_uart_log = SCRIPT_DIR / "logs" / f"{sim_node_id}.uart.log"
        ota_offset = (ota_uart_log.stat().st_size
                      if ota_uart_log.exists() else 0)
        # build_type=simulation tells the server which staged firmware
        # to broadcast — under SIMULATION we want the sim ELF, not the
        # hardware .bin. force_firmware_update bypasses the "is your
        # current version older" check, which we want for a smoke run.
        # The server's force_firmware_update gate now routes by chip
        # family (Teensy nodes look up server/resources/firmware/teensy41/,
        # RP2040 nodes still hit firmware/rp2040/build_sim/) so the
        # call works on both platforms.
        ota_resp = await client.request("force_firmware_update",
                                        {"node_id": target_node_id,
                                         "build_type": "simulation"})
        ota_resp_msg = ota_resp.get("message") or ""
        if ota_resp.get("status") != "ok":
            _say("phase 5",
                 f"server rejected force_firmware_update: {ota_resp_msg}",
                 ok=False)
            failures += 1
        else:
            ota_deadline = time.monotonic() + SYNC_WAIT_S
            ota_seen = False
            while time.monotonic() < ota_deadline:
                if ota_uart_log.exists():
                    with open(ota_uart_log, "rb") as f:
                        f.seek(ota_offset)
                        if b"FIRMWARE UPDATE REQUESTED" in f.read():
                            ota_seen = True
                            break
                await asyncio.sleep(0.5)
            if ota_seen:
                _say("phase 5", "firmware acknowledged OTA request")
            else:
                _say("phase 5",
                     f"firmware never logged 'FIRMWARE UPDATE REQUESTED' "
                     f"within {SYNC_WAIT_S}s",
                     ok=False)
                failures += 1

        # ── Phase 6: server-initiated unadopt + re-adopt ─────────────
        # `remove_node` is the operator's "Forget this node" button.
        # The server sends factory_reset to the firmware (which clears
        # flash + reboots), then drops the node from its adopted set.
        # After the sim node restarts, it announces fresh as UNADOPTED
        # — different from Phase 3's "server still thinks it's adopted,
        # auto-reconcile pushes config back" path. Re-adopting from
        # this state should walk through the full Phase 2 sequence
        # again.
        _say("phase 6",
             "issuing remove_node (server-initiated factory reset)")
        remove_resp = await client.request("remove_node",
                                           {"node_id": target_node_id})
        if remove_resp.get("status") != "ok":
            _say("phase 6",
                 f"server rejected remove_node: "
                 f"{remove_resp.get('message') or '(no msg)'}",
                 ok=False)
            failures += 1
        else:
            # The firmware halts after factory_reset under SIMULATION;
            # node_manager has to restart it to see the post-wipe boot.
            _say("phase 6", "restarting sim node after factory reset")
            _run_node_manager("stop", sim_node_id)
            _run_node_manager("reset", sim_node_id)
            _run_node_manager("start", sim_node_id)

            # Wait for the firmware to announce again (under whatever
            # node_id its chip-derived hash gives — same as Phase 1).
            _say("phase 6", "waiting for fresh UNADOPTED announcement")
            chip_prefix = f"{node_type}_"
            re_announce_deadline = time.monotonic() + ADOPT_WAIT_S
            re_announced = None
            while time.monotonic() < re_announce_deadline:
                resp = await client.request("list_unadopted")
                nodes = (resp.get("data") or {}).get("nodes") or []
                re_announced = next(
                    (n for n in nodes
                     if (n.get("node_id") or "").startswith(chip_prefix)),
                    None,
                )
                if re_announced:
                    break
                await asyncio.sleep(1.0)
            if not re_announced:
                _say("phase 6",
                     "no fresh announcement after factory reset",
                     ok=False)
                failures += 1
            else:
                target_node_id = re_announced.get("node_id")
                _say("phase 6", f"node re-announced as {target_node_id}")
                # Re-subscribe under the (possibly new) announced id.
                await client.subscribe([
                    f"node_log/{target_node_id}",
                    f"sync_status/{target_node_id}",
                ], rate_hz=10)
                # Re-adopt and verify the config flows again.
                await client.request("adopt_node", {
                    "node_id": target_node_id,
                    "role": "cradle_base",
                    "display_name": "SyncTest-Readopt",
                    "board_id": board_id,
                })
                await asyncio.sleep(1.0)  # let DDS settle (Phase 2 logic)
                await client.request("sync_node_peripherals",
                                     {"node_id": target_node_id})
                entry = await client.wait_for_node_log_containing(
                    target_node_id, "Config saved to flash", SYNC_WAIT_S)
                if entry:
                    _say("phase 6", "re-adoption ack'd via /announce")
                else:
                    _say("phase 6",
                         f"re-adopt issued but no 'Config saved to "
                         f"flash' ack within {SYNC_WAIT_S}s",
                         ok=False)
                    failures += 1

        await client.close()

    finally:
        if node_created:
            # Renode path owns a separately-spawned process; clean it up.
            # Always use sim_node_id (node_manager's view) regardless of
            # whether Phase 1 rewired target_node_id to the announced id.
            if failures:
                # Preserve log + .resc so the operator can diagnose
                # which hop failed. Next run's pre-clean removes them.
                _say("teardown",
                     "stopping sim node (logs preserved for diagnosis)")
                _run_node_manager("stop", sim_node_id)
            else:
                _say("teardown", "stopping + removing sim node")
                _run_node_manager("stop", sim_node_id)
                _run_node_manager("remove", sim_node_id)
        # Fake-firmware lifetime is managed by entrypoint.sh; nothing
        # to do here — entrypoint's trap kills it on container exit.

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
    parser.add_argument("--fake-firmware", action="store_true",
                        help="Run against the Python fake-firmware "
                             "stand-in instead of a Renode sim node. "
                             "Assumes the fake firmware is already "
                             "running (entrypoint.sh starts it in the "
                             "container when MODE=fake). Bypasses "
                             "node_manager + Renode + XRCE, exercises "
                             "everything else in the server chain.")
    parser.add_argument("--existing-node", action="store_true",
                        help="Target an already-adopted node on a "
                             "running SAINT.OS server (real hardware). "
                             "Skips Renode bring-up, adoption, and the "
                             "Phase 3 factory-reset — only exercises "
                             "the sync_node_peripherals → firmware "
                             "log ack loop. Use --node-id to pick "
                             "which adopted node to drive.")
    parser.add_argument("--node-type", default="rp2040",
                        choices=sorted(NODE_TYPES),
                        help="Sim-node type to drive. Picks the firmware "
                             "ELF the prereq check expects, the default "
                             "node_id, and the board_id passed to "
                             "/adopt_node. Default: rp2040 (the green "
                             "path); use teensy41 for the Teensy 4.1 sim.")
    parser.add_argument("--node-id", default=None,
                        help="Node ID to drive the test against. "
                             "Default: per-type — `rp2040_synctest` / "
                             "`teensy41_synctest`. Use rp2040_fakefw to "
                             "match the fake firmware's default.")
    args = parser.parse_args()

    # Resolve a default node_id from --node-type if --node-id wasn't passed.
    resolved_node_id = args.node_id or NODE_TYPES[args.node_type]["default_node_id"]

    print("Checking prerequisites…")
    # Renode is only needed for the non-fake, non-existing path.
    need_renode = not args.fake_firmware and not args.existing_node \
                  and not args.no_server
    problems = check_prereqs(
        args.ws_url,
        need_server=not args.no_server,
        node_type=args.node_type,
    )
    if not need_renode:
        # Filter out Renode-specific complaints since we don't need it.
        problems = [p for p in problems
                    if "Renode" not in p and "Sim firmware" not in p]
    if args.existing_node:
        # Existing-hardware mode: agent + firmware live on the remote
        # box we're connecting to via --ws-url. The harness only needs
        # the websockets lib + server reachability locally.
        problems = [p for p in problems if "micro-ROS agent" not in p]
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
            return run_smoke_test(node_type=args.node_type, node_id=resolved_node_id)
        return asyncio.run(run_test(
            args.ws_url, args.password,
            fake_firmware=args.fake_firmware,
            existing_node=args.existing_node,
            node_id=resolved_node_id,
            node_type=args.node_type,
        ))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
