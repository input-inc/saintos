"""File-backed logging with daily rotation that survives server downtime.

Three roles:
  - Server activity log → ``$SAINT_LOG_DIR/saint-server.log``
  - Per-node activity log → ``$SAINT_LOG_DIR/nodes/<node_id>.log``
  - Per-peripheral telemetry log →
    ``$SAINT_LOG_DIR/peripherals/<node_id>__<peripheral_id>.log``

Each handler is a ``TimedRotatingFileHandler(when="midnight")``, so
live rotation happens at midnight while the process is running.

In addition, :func:`rotate_if_stale` renames the current file to a
dated backup BEFORE creating the handler if the file's mtime is from
an earlier day — so a server that was off across midnight still
rotates correctly on the next start (instead of appending today's
entries to yesterday's file).
"""

from __future__ import annotations

import datetime as _dt
import logging
import logging.handlers
import os
import threading
from typing import Optional

# 14 days seems like a good default — enough to debug a weeklong issue
# without filling the rover's SSD on a misconfigured peripheral that's
# emitting a lot of telemetry. Override per-handler if needed.
DEFAULT_BACKUP_COUNT = 14


def _today_midnight_ts() -> float:
    """Local-time start-of-today as a unix timestamp."""
    now = _dt.datetime.now()
    return _dt.datetime(now.year, now.month, now.day).timestamp()


def rotate_if_stale(path: str) -> None:
    """If ``path`` exists with mtime before today, rename it to
    ``<path>.<file's date>`` so a fresh file gets opened when the
    handler runs.

    Mirrors TimedRotatingFileHandler's "midnight" naming
    (suffix = ``YYYY-MM-DD``) so the dated backups stay consistent
    whether they were produced live at midnight or by this catch-up
    rename on a cold start.
    """
    if not os.path.exists(path):
        return
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return
    if mtime >= _today_midnight_ts():
        return
    file_date = _dt.date.fromtimestamp(mtime).isoformat()
    backup = f"{path}.{file_date}"
    # If a backup with that date already exists (server stopped and
    # restarted multiple times in the same day after midnight rotation
    # already moved one), tack on the mtime so we don't clobber.
    if os.path.exists(backup):
        backup = f"{backup}.{int(mtime)}"
    try:
        os.rename(path, backup)
    except OSError:
        # If the rename fails, the existing file will keep growing
        # until TimedRotatingFileHandler eventually rotates it, but
        # we don't want to crash the server over a log file.
        pass


def make_daily_handler(
    path: str,
    formatter: Optional[logging.Formatter] = None,
    backup_count: int = DEFAULT_BACKUP_COUNT,
) -> logging.Handler:
    """Build a daily-rotating handler at ``path`` that ALSO catches up
    on rotation if the file is stale (server was off across midnight).
    """
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    rotate_if_stale(path)
    handler = logging.handlers.TimedRotatingFileHandler(
        path, when="midnight", backupCount=backup_count, encoding="utf-8"
    )
    handler.suffix = "%Y-%m-%d"
    handler.setFormatter(formatter or logging.Formatter("%(message)s"))
    return handler


def _sanitize_key(name: str) -> str:
    """Filesystem-safe name. Replace anything outside [A-Za-z0-9_.-]
    with ``_`` so a node id (or arbitrary key) with unusual characters
    still produces a valid filename.
    """
    out: list[str] = []
    for ch in name:
        if ch.isalnum() or ch in ("-", "_", "."):
            out.append(ch)
        else:
            out.append("_")
    return "".join(out) or "unnamed"


class PerKeyLogger:
    """A pool of file loggers keyed by an arbitrary string.

    The handler for each key is built lazily on the first ``log()``
    call and lives until shutdown — TimedRotatingFileHandler rotates
    at midnight on its own, no per-call work.

    Used for per-node activity files and per-peripheral telemetry
    files. The key is sanitized for the filename.
    """

    def __init__(
        self,
        base_dir: str,
        logger_name_prefix: str,
        formatter: Optional[logging.Formatter] = None,
        backup_count: int = DEFAULT_BACKUP_COUNT,
    ):
        self._base_dir = base_dir
        self._name_prefix = logger_name_prefix
        self._formatter = formatter
        self._backup_count = backup_count
        self._lock = threading.Lock()
        self._loggers: dict[str, logging.Logger] = {}
        os.makedirs(base_dir, exist_ok=True)

    def _logger_for(self, key: str) -> logging.Logger:
        with self._lock:
            existing = self._loggers.get(key)
            if existing is not None:
                return existing
            safe = _sanitize_key(key)
            path = os.path.join(self._base_dir, f"{safe}.log")
            # Use a unique logger name per key so each gets its own
            # handler. propagate=False keeps these out of the root /
            # ROS logger chains.
            logger = logging.getLogger(f"{self._name_prefix}.{safe}")
            logger.setLevel(logging.INFO)
            logger.propagate = False
            for h in list(logger.handlers):
                logger.removeHandler(h)
            logger.addHandler(make_daily_handler(
                path,
                formatter=self._formatter,
                backup_count=self._backup_count,
            ))
            self._loggers[key] = logger
            return logger

    def log(self, key: str, message: str, level: int = logging.INFO) -> None:
        self._logger_for(key).log(level, message)

    def shutdown(self) -> None:
        with self._lock:
            for logger in self._loggers.values():
                for h in list(logger.handlers):
                    try:
                        h.close()
                    except Exception:
                        pass
                    logger.removeHandler(h)
            self._loggers.clear()
