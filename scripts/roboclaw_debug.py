#!/usr/bin/env python3
"""Interactive RoboClaw wire-level diagnostic CLI.

Talks to the saint_server WebSocket endpoint, dispatches roboclaw_debug
operations to a specified node, and tails the node's log stream looking
for the firmware's "roboclaw_dbg:" replies.

Usage:
  python3 scripts/roboclaw_debug.py --server 192.168.x.x --node <node_id>
  [--password PASS] [--port 80]

Then at the prompt:
  raw 8015 4 200            # tx bytes (hex), read_len, timeout_ms
  raw 80520011 4 200        # GETTEMP with CRC bytes 00 11 (example)
  sniff 2000                # listen passively for 2 seconds
  reconfigure 19200         # rebind PIO at 19200 baud
  reconfigure 38400 swap=1  # also force swap on
  nodes                     # list nodes seen by the server
  quit

Requires: pip install websockets
"""
import argparse
import asyncio
import json
import sys
import time
import uuid

try:
    import websockets
except ImportError:
    print("ERROR: install websockets first → pip install websockets",
          file=sys.stderr)
    sys.exit(1)


class WSClient:
    """Single WS connection with a central receive loop.

    ws.recv() can only have one waiter at a time, so we run one
    dispatcher that demuxes responses (matched by request id) from
    state/activity frames (pushed to subscribers).
    """

    def __init__(self, ws):
        self.ws = ws
        self._pending: dict[str, asyncio.Future] = {}
        self._log_queue: asyncio.Queue = asyncio.Queue()
        self._recv_task: asyncio.Task | None = None
        self._closed = False

    async def start(self):
        self._recv_task = asyncio.create_task(self._dispatch())

    async def stop(self):
        self._closed = True
        if self._recv_task:
            self._recv_task.cancel()

    async def _dispatch(self):
        try:
            async for raw in self.ws:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                msg_id = msg.get("id")
                if msg_id and msg_id in self._pending:
                    fut = self._pending.pop(msg_id)
                    if not fut.done():
                        fut.set_result(msg)
                    continue
                # Otherwise it's an unsolicited frame — interesting if
                # it's a state frame on a log topic, else ignore.
                if msg.get("type") == "state":
                    await self._log_queue.put(msg)
                elif msg.get("type") == "activity":
                    await self._log_queue.put(msg)
        except (websockets.ConnectionClosed, asyncio.CancelledError):
            pass
        finally:
            # Reject any in-flight requests so callers don't hang forever.
            for fut in self._pending.values():
                if not fut.done():
                    fut.set_exception(ConnectionError("WS closed"))
            self._pending.clear()

    async def call(self, msg_type, action, params, timeout=5.0):
        msg_id = str(uuid.uuid4())[:8]
        fut = asyncio.get_event_loop().create_future()
        self._pending[msg_id] = fut
        await self.ws.send(json.dumps({
            "id": msg_id, "type": msg_type, "action": action, "params": params,
        }))
        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending.pop(msg_id, None)
            return {"status": "error", "message": "WS call timed out"}

    async def recv_log(self, timeout=None):
        if timeout is None:
            return await self._log_queue.get()
        try:
            return await asyncio.wait_for(self._log_queue.get(), timeout)
        except asyncio.TimeoutError:
            return None


async def authenticate(ws, password):
    """Read the initial 'connected' frame, send auth if required."""
    raw = await ws.recv()
    msg = json.loads(raw)
    if msg.get("type") != "connected":
        print(f"unexpected first frame: {msg}", file=sys.stderr)
        sys.exit(1)
    if msg.get("auth_required"):
        if not password:
            print("server requires auth — pass --password", file=sys.stderr)
            sys.exit(1)
        await ws.send(json.dumps({"type": "auth", "password": password}))
        reply = json.loads(await ws.recv())
        ok_field = reply.get("status") == "ok" or reply.get("type") == "auth_ok"
        if not ok_field:
            print(f"auth failed: {reply}", file=sys.stderr)
            sys.exit(1)


def format_log_frame(frame, node_id):
    """Return a printable line for a state/activity frame, or None to skip."""
    if frame.get("type") == "state":
        if frame.get("node") != f"node_log/{node_id}":
            return None
        entry = frame.get("data") or {}
        text = entry.get("text") or ""
        if "roboclaw" not in text.lower():
            return None
        t = entry.get("time")
        ts = time.strftime("%H:%M:%S", time.localtime(t)) if t else "--:--:--"
        return f"  [{ts} {entry.get('level', 'info')}] {text}"
    if frame.get("type") == "activity":
        text = frame.get("text") or ""
        if "roboclaw" not in text.lower():
            return None
        t = frame.get("timestamp")
        ts = time.strftime("%H:%M:%S", time.localtime(t)) if t else "--:--:--"
        return f"  [{ts} activity] {text}"
    return None


async def drain_logs(client, node_id, deadline_secs=0.6):
    """Wait briefly for any log frames produced by the most recent call."""
    end = time.time() + deadline_secs
    while True:
        remaining = end - time.time()
        if remaining <= 0:
            break
        frame = await client.recv_log(timeout=remaining)
        if frame is None:
            break
        line = format_log_frame(frame, node_id)
        if line:
            print(line)


def parse_swap_kv(token):
    if not token.startswith("swap="):
        return None
    try:
        return int(token.split("=", 1)[1])
    except (ValueError, IndexError):
        return None


async def run_repl(client, node_id):
    print(f"# roboclaw debug — connected, node={node_id}")
    print("# commands: raw <hex> <read_len> <timeout_ms>  |  "
          "sniff <duration_ms>  |  reconfigure <baud> [swap=0|1]  |  "
          "nodes  |  quit")

    loop = asyncio.get_event_loop()
    while True:
        # Drain any buffered log frames before showing the prompt.
        await drain_logs(client, node_id, deadline_secs=0.05)

        try:
            line = await loop.run_in_executor(None, lambda: input("rc> "))
        except EOFError:
            print()
            return
        line = line.strip()
        if not line:
            continue
        if line in ("quit", "exit", "q"):
            return
        parts = line.split()
        cmd = parts[0]

        if cmd == "raw":
            if len(parts) < 4:
                print("usage: raw <hex> <read_len> <timeout_ms>")
                continue
            tx_hex = parts[1]
            read_len = int(parts[2])
            timeout_ms = int(parts[3])
            resp = await client.call("management", "roboclaw_debug", {
                "node_id": node_id, "op": "raw",
                "tx_hex": tx_hex, "read_len": read_len,
                "timeout_ms": timeout_ms,
            })
            if resp.get("status") != "ok":
                print(f"  error: {resp.get('message')}")

        elif cmd == "sniff":
            if len(parts) < 2:
                print("usage: sniff <duration_ms>")
                continue
            duration_ms = int(parts[1])
            resp = await client.call("management", "roboclaw_debug", {
                "node_id": node_id, "op": "sniff",
                "duration_ms": duration_ms,
            })
            if resp.get("status") != "ok":
                print(f"  error: {resp.get('message')}")

        elif cmd == "reconfigure":
            if len(parts) < 2:
                print("usage: reconfigure <baud> [swap=0|1]")
                continue
            payload = {"node_id": node_id, "op": "reconfigure",
                       "baud": int(parts[1])}
            for tok in parts[2:]:
                sv = parse_swap_kv(tok)
                if sv is not None:
                    payload["swap"] = sv
            resp = await client.call("management", "roboclaw_debug", payload)
            if resp.get("status") != "ok":
                print(f"  error: {resp.get('message')}")

        elif cmd == "nodes":
            resp = await client.call("management", "list_adopted", {})
            for n in (resp.get("data") or {}).get("nodes") or []:
                print(f"  {n.get('node_id'):<32} {n.get('display_name', '')}")
            continue  # No firmware reply for this one — skip drain wait.

        else:
            print(f"unknown: {cmd}")
            continue

        # Give the firmware ~1s to publish its reply, then drain.
        await drain_logs(client, node_id, deadline_secs=1.0)


async def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--server", required=True, help="server host or IP")
    ap.add_argument("--port", type=int, default=80, help="HTTP port (default 80)")
    ap.add_argument("--node", required=True, help="target node_id")
    ap.add_argument("--password", default=None, help="WS password (if set)")
    args = ap.parse_args()

    uri = f"ws://{args.server}:{args.port}/api/ws"
    async with websockets.connect(uri) as ws:
        await authenticate(ws, args.password)
        client = WSClient(ws)
        await client.start()
        sub = await client.call("subscribe", "subscribe",
                                {"topics": [f"node_log/{args.node}"]})
        if sub.get("status") != "ok":
            print(f"subscribe failed: {sub}", file=sys.stderr)
            sys.exit(1)
        try:
            await run_repl(client, args.node)
        finally:
            await client.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
