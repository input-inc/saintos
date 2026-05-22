"""Per-peripheral telemetry logger.

Three-tier sink fed from the firmware-state hot path:
  - 30 s and 60 s in-memory deques per (node_id, peripheral_id, channel_id)
    for live graphing and recent-history download.
  - NDJSON file under SAINT_LOG_DIR/peripherals/, rotated daily,
    last 5 days retained (TimedRotatingFileHandler does both).

Logging is opt-in: ``record()`` is a no-op unless the peripheral is in
the enabled set. The state manager calls ``set_enabled()`` whenever a
peripheral's ``log_enabled`` flag changes (or on config load).
"""

import json
import logging
import logging.handlers
import os
import queue
import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

# 10 Hz ceiling per the design discussion. We size deques generously so a
# misbehaving peripheral can't quietly grow memory if it overshoots.
_RATE_HZ = 10
_LIVE_30S_LEN = 30 * _RATE_HZ
_LIVE_60S_LEN = 60 * _RATE_HZ

# Disk-writer queue cap. If the writer thread falls this far behind (disk
# stalled, etc.) we start dropping rather than ballooning memory. At
# 10 Hz × ~50 channels that's >20 s of buffer, which is plenty.
_QUEUE_MAXSIZE = 10_000

_ChannelKey = Tuple[str, str, str]  # (node_id, peripheral_id, channel_id)


class PeripheralLogger:
    def __init__(self, log_dir: str, logger: Optional[logging.Logger] = None):
        self._log_dir = log_dir
        self._logger = logger
        self._lock = threading.Lock()
        self._enabled: set[Tuple[str, str]] = set()  # (node_id, peripheral_id)
        self._live_30s: Dict[_ChannelKey, Deque[Tuple[float, float]]] = {}
        self._live_60s: Dict[_ChannelKey, Deque[Tuple[float, float]]] = {}

        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, "peripherals.log")

        # Plain file handler, not the root logger — keeps NDJSON lines
        # separate from anything Python's logging hierarchy might write.
        self._file_logger = logging.getLogger("saint_os.peripherals")
        self._file_logger.setLevel(logging.INFO)
        self._file_logger.propagate = False
        # Idempotent across re-instantiation (tests, reloads).
        for h in list(self._file_logger.handlers):
            self._file_logger.removeHandler(h)
        self._handler = logging.handlers.TimedRotatingFileHandler(
            log_path, when="midnight", backupCount=5, encoding="utf-8"
        )
        self._handler.setFormatter(logging.Formatter("%(message)s"))
        self._file_logger.addHandler(self._handler)

        self._queue: "queue.Queue[Optional[str]]" = queue.Queue(maxsize=_QUEUE_MAXSIZE)
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="peripheral-logger", daemon=True
        )
        self._writer_thread.start()
        self._dropped = 0

    # ---- public API -------------------------------------------------------

    def set_enabled(self, node_id: str, peripheral_id: str, enabled: bool) -> None:
        """Toggle logging for one peripheral. Idempotent."""
        key = (node_id, peripheral_id)
        with self._lock:
            if enabled:
                self._enabled.add(key)
            else:
                self._enabled.discard(key)
                # Free the deques so a long-running server doesn't keep
                # state for a peripheral that's been disabled.
                for ckey in [k for k in self._live_30s if k[0] == node_id and k[1] == peripheral_id]:
                    self._live_30s.pop(ckey, None)
                    self._live_60s.pop(ckey, None)

    def is_enabled(self, node_id: str, peripheral_id: str) -> bool:
        with self._lock:
            return (node_id, peripheral_id) in self._enabled

    def record(self, node_id: str, peripheral_id: str, channel_id: str,
               value: float, ts: Optional[float] = None) -> None:
        """Hot path. ~µs when disabled, ~µs+queue-put when enabled."""
        # Fast bail: cheap set lookup, no I/O.
        key = (node_id, peripheral_id)
        if key not in self._enabled:
            return
        if ts is None:
            ts = time.time()
        ckey: _ChannelKey = (node_id, peripheral_id, channel_id)
        with self._lock:
            d30 = self._live_30s.get(ckey)
            if d30 is None:
                d30 = deque(maxlen=_LIVE_30S_LEN)
                self._live_30s[ckey] = d30
            d60 = self._live_60s.get(ckey)
            if d60 is None:
                d60 = deque(maxlen=_LIVE_60S_LEN)
                self._live_60s[ckey] = d60
            d30.append((ts, value))
            d60.append((ts, value))

        # Disk: serialize outside the lock; drop rather than block if the
        # writer thread is wedged.
        line = json.dumps({
            "ts": ts,
            "node": node_id,
            "periph": peripheral_id,
            "channel": channel_id,
            "v": value,
        }, separators=(",", ":"))
        try:
            self._queue.put_nowait(line)
        except queue.Full:
            self._dropped += 1
            if self._logger and self._dropped % 1000 == 1:
                self._logger.warn(
                    f"PeripheralLogger queue full — dropped {self._dropped} samples"
                )

    def history(self, node_id: str, peripheral_id: str, channel_id: str,
                window: str = "30s") -> List[Tuple[float, float]]:
        """Snapshot of the requested ring buffer. Returns [] if no samples."""
        ckey: _ChannelKey = (node_id, peripheral_id, channel_id)
        with self._lock:
            src = self._live_30s if window == "30s" else self._live_60s
            d = src.get(ckey)
            return list(d) if d else []

    def shutdown(self, timeout: float = 2.0) -> None:
        """Flush the writer thread and close the file handler."""
        self._queue.put(None)
        self._writer_thread.join(timeout=timeout)
        self._handler.close()

    # ---- internals --------------------------------------------------------

    def _writer_loop(self) -> None:
        while True:
            try:
                line = self._queue.get()
            except Exception:
                continue
            if line is None:
                return
            try:
                self._file_logger.info(line)
            except Exception as e:
                # Never let a disk hiccup kill the thread.
                if self._logger:
                    self._logger.error(f"PeripheralLogger write failed: {e}")
