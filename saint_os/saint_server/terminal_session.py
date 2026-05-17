"""SAINT.OS web terminal — PTY-backed shell sessions over WebSocket.

Each TerminalSession owns one bash subprocess running as the current
service user (saint). Input arrives from the WebSocket; output is read
from the PTY in an executor thread and forwarded back to the client.
The handler module owns one TerminalSession per WebSocket client.

Audit logging: every newline-terminated input line is written to
journald under the tag `saint-os-terminal`, with the originating client
id. Captures intent (what the operator typed) without recording every
keystroke. Output is not logged.
"""

from __future__ import annotations

import asyncio
import fcntl
import logging
import logging.handlers
import os
import pty
import select
import shutil
import signal
import struct
import termios
import traceback
from typing import Awaitable, Callable, Optional

# Audit logger. Uses syslog on /dev/log so systemd-journald picks it up
# under the configured SyslogIdentifier. Filter with:
#     journalctl -t saint-os-terminal -f
_audit_logger: Optional[logging.Logger] = None


def _get_audit_logger() -> logging.Logger:
    """Lazy syslog audit logger — only attached on first use."""
    global _audit_logger
    if _audit_logger is not None:
        return _audit_logger

    logger = logging.getLogger("saint_os.terminal_audit")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    try:
        handler = logging.handlers.SysLogHandler(address="/dev/log")
        handler.ident = "saint-os-terminal: "
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(handler)
    except OSError:
        # No /dev/log (e.g., running on macOS dev box) — fall back to
        # the parent process's stderr so audit lines aren't silently lost.
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("AUDIT %(message)s"))
        logger.addHandler(handler)
    _audit_logger = logger
    return logger


class TerminalSession:
    """A single PTY-backed bash session bound to a WebSocket client."""

    DEFAULT_COLS = 80
    DEFAULT_ROWS = 24

    def __init__(
        self,
        on_output: Callable[[bytes], Awaitable[None]],
        on_exit: Optional[Callable[[int], Awaitable[None]]] = None,
        client_id: str = "?",
        logger: Optional[logging.Logger] = None,
    ):
        self._on_output = on_output
        self._on_exit = on_exit
        self._client_id = client_id
        self._logger = logger or logging.getLogger(__name__)

        self._pid: Optional[int] = None
        self._master_fd: Optional[int] = None
        self._read_task: Optional[asyncio.Task] = None
        self._closed = False
        self._input_buffer = bytearray()  # accumulates bytes between newlines for audit

    # ── lifecycle ────────────────────────────────────────────────────

    async def start(self, cols: int = DEFAULT_COLS, rows: int = DEFAULT_ROWS):
        """Spawn the shell. Safe to call once."""
        if self._pid is not None:
            return

        shell = shutil.which("bash") or "/bin/bash"
        pid, master_fd = pty.fork()
        if pid == 0:
            # Child: become the shell. Set a friendly environment so
            # interactive tools (vim, htop) behave.
            env = {
                **os.environ,
                "TERM": "xterm-256color",
                "LANG": os.environ.get("LANG", "C.UTF-8"),
                "PS1": r"\u@\h:\w$ ",
            }
            # PWD: start in the service user's home if it exists.
            home = os.path.expanduser("~")
            if home and os.path.isdir(home):
                try:
                    os.chdir(home)
                except OSError:
                    pass
            # Replace child process. `-l` makes it a login shell so
            # /etc/profile + ~/.bashrc are sourced.
            try:
                os.execvpe(shell, [shell, "-l"], env)
            except OSError:
                os._exit(127)

        # Parent.
        self._pid = pid
        self._master_fd = master_fd
        self._set_winsize(cols, rows)

        # Make reads from the master FD non-blocking so the read loop
        # can pump in chunks without ever stalling the event loop.
        flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
        fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        self._logger.info(
            f"terminal: spawned shell pid={pid} for client={self._client_id} ({cols}x{rows})"
        )

        loop = asyncio.get_event_loop()
        self._read_task = loop.create_task(self._read_loop())

    async def close(self):
        """Kill the shell and clean up. Idempotent."""
        if self._closed:
            return
        self._closed = True

        if self._pid is not None:
            try:
                os.kill(self._pid, signal.SIGHUP)
            except ProcessLookupError:
                pass

        if self._read_task is not None:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
            except Exception:
                self._logger.error(f"terminal read loop error during close\n{traceback.format_exc()}")
            self._read_task = None

        if self._master_fd is not None:
            try:
                os.close(self._master_fd)
            except OSError:
                pass
            self._master_fd = None

        # Reap so we don't leave zombies.
        if self._pid is not None:
            try:
                os.waitpid(self._pid, os.WNOHANG)
            except ChildProcessError:
                pass
            self._pid = None

        self._logger.info(f"terminal: closed for client={self._client_id}")

    # ── I/O ─────────────────────────────────────────────────────────

    def write(self, data: bytes):
        """Write input bytes to the PTY. Audit-logs full lines."""
        if self._master_fd is None or self._closed:
            return
        try:
            os.write(self._master_fd, data)
        except OSError as e:
            self._logger.warning(f"terminal: write failed: {e}")
            return

        # Audit: accumulate and emit on each LF / CR.
        self._input_buffer.extend(data)
        while True:
            # Find first newline-ish byte.
            try:
                idx_nl = self._input_buffer.index(0x0A)
            except ValueError:
                idx_nl = -1
            try:
                idx_cr = self._input_buffer.index(0x0D)
            except ValueError:
                idx_cr = -1
            candidates = [i for i in (idx_nl, idx_cr) if i >= 0]
            if not candidates:
                break
            idx = min(candidates)
            line = bytes(self._input_buffer[:idx])
            del self._input_buffer[: idx + 1]
            # Skip empty submissions.
            if line:
                try:
                    text = line.decode("utf-8", errors="replace").rstrip("\r")
                    _get_audit_logger().info(
                        "client=%s pid=%s cmd=%s",
                        self._client_id, self._pid, text,
                    )
                except Exception:
                    self._logger.error(f"audit log emit failed\n{traceback.format_exc()}")

    def resize(self, cols: int, rows: int):
        if self._master_fd is None or self._closed:
            return
        self._set_winsize(cols, rows)

    def _set_winsize(self, cols: int, rows: int):
        if self._master_fd is None:
            return
        # struct winsize: rows, cols, xpixel, ypixel
        winsize = struct.pack("HHHH", max(rows, 1), max(cols, 1), 0, 0)
        try:
            fcntl.ioctl(self._master_fd, termios.TIOCSWINSZ, winsize)
        except OSError as e:
            self._logger.debug(f"terminal: TIOCSWINSZ failed: {e}")

    async def _read_loop(self):
        """Read from the PTY master and push to the client.

        Uses select() with a short timeout so we can be cancelled
        promptly when the session closes.
        """
        assert self._master_fd is not None
        fd = self._master_fd
        try:
            while not self._closed:
                ready, _, _ = await asyncio.get_event_loop().run_in_executor(
                    None, select.select, [fd], [], [], 0.5
                )
                if not ready:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except OSError:
                    # PTY closed / shell exited.
                    break
                if not chunk:
                    break
                try:
                    await self._on_output(chunk)
                except Exception:
                    self._logger.error(f"terminal: on_output handler raised\n{traceback.format_exc()}")
        except asyncio.CancelledError:
            raise
        except Exception:
            self._logger.error(f"terminal: read loop unexpected error\n{traceback.format_exc()}")
        finally:
            # Notify handler of shell exit (best-effort).
            if self._on_exit is not None and not self._closed:
                exit_code = -1
                if self._pid is not None:
                    try:
                        _, status = os.waitpid(self._pid, os.WNOHANG)
                        exit_code = (status >> 8) if status else -1
                    except ChildProcessError:
                        pass
                try:
                    await self._on_exit(exit_code)
                except Exception:
                    self._logger.error(f"terminal: on_exit handler raised\n{traceback.format_exc()}")
