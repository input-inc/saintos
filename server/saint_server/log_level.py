"""Runtime log-level control.

Server config carries a `logging.level` field (see config/__init__.py).
At startup and on settings save, the level string ("DEBUG" / "INFO" /
"WARNING" / "ERROR" / "CRITICAL") is applied here to both logger trees
the server uses:

  * Python `logging`: the `saint_os` root logger, which the file-backed
    handlers in file_log.py and the per-key node/peripheral loggers all
    hang off. Setting this gates everything that flows to disk.
  * rclpy (`rcutils`): the `saint_server` node logger, used by the ROS
    bridge and routing evaluator via `self.get_logger().info(...)`.
    Without this, the per-tick INFO lines still spew to the ROS console
    even after the Python side is muted.

Default is WARNING so the per-tick set_channel / set_topic_channel
lines stay off the file handler at 50 Hz. Operators bump it to INFO
(or DEBUG) from the Settings panel when diagnosing a binding issue.
"""

from __future__ import annotations

import logging
from typing import Optional


_VALID_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")

# Where rclpy lives — name must match Node.__init__('saint_server') in
# server_node.py. Kept here so log_level.py is the single source of
# truth for "which rclpy logger does the server speak through".
_RCLPY_LOGGER_NAME = "saint_server"


def normalize_level(level: str) -> str:
    """Canonicalize a level string. Returns "WARNING" for unknown input
    so a malformed config file never silences the server entirely."""
    if not isinstance(level, str):
        return "WARNING"
    up = level.strip().upper()
    return up if up in _VALID_LEVELS else "WARNING"


def apply_log_level(level: str, ros_node: Optional[object] = None) -> str:
    """Apply ``level`` to the Python `saint_os` logger tree and (if
    available) the rclpy `saint_server` logger.

    Returns the normalized level actually applied — callers can persist
    that back into config so a typo doesn't leave the file disagreeing
    with the live runtime.

    ros_node is optional: when present, we also re-set the rclpy
    severity on its specific logger instance, which covers cases where
    rcutils caches a severity per-logger-name and the global
    set_logger_level() call hasn't propagated yet.
    """
    canonical = normalize_level(level)
    py_level = getattr(logging, canonical)

    # Python side — gates file handlers and any other downstream consumers.
    # Walk every `saint_os.*` logger because several of them
    # (file_log.PerKeyLogger and the saint-server.log activity logger)
    # call setLevel() themselves, which shadows a parent-level change.
    # Touching each one directly is robust against future sub-loggers
    # too — no need to know their names in advance.
    logging.getLogger("saint_os").setLevel(py_level)
    manager = logging.getLogger("saint_os").manager
    for logger_name, logger in list(manager.loggerDict.items()):
        if isinstance(logger, logging.Logger) and (
            logger_name == "saint_os" or logger_name.startswith("saint_os.")
        ):
            logger.setLevel(py_level)

    # rclpy side — only import on demand so this module stays usable in
    # unit tests that don't have a ROS install on the path.
    try:
        import rclpy.logging as _rl
        severity = getattr(_rl.LoggingSeverity, canonical, None)
        if severity is not None:
            _rl.set_logger_level(_RCLPY_LOGGER_NAME, severity)
            if ros_node is not None:
                # rcutils tracks severity per logger object; resetting
                # it here ensures the node's own get_logger() reflects
                # the new level immediately rather than on next message.
                try:
                    ros_node.get_logger().set_level(severity)
                except Exception:
                    pass
    except ImportError:
        # No rclpy available (unit tests, tooling). Python side is
        # enough — file handler I/O still gets gated.
        pass

    return canonical


def log_at(logger, level: str, message: str) -> None:
    """Dispatch `message` to `logger.<level>()` from a stable per-severity line.

    rclpy's rcutils logger identifies each log call by source file + line.
    If the same source line emits at more than one severity over its
    lifetime, the second severity trips a
    ``ValueError: Logger severity cannot be changed between calls.``
    Wrappers that did ``getattr(logger, level)(message)`` looked clean
    but funnelled every severity through ONE line, so they tripped this
    check the moment a dashboard log-level change started actually
    invoking the previously-filtered debug paths.

    This helper sidesteps that by having each severity emit from its
    own dedicated line below. rclpy treats each elif branch as a
    distinct call site, so a single severity is forever pinned to a
    single line — no matter how many callers funnel through here.
    """
    if logger is None:
        return
    if level == 'debug':
        logger.debug(message)
    elif level == 'info':
        logger.info(message)
    elif level == 'warn' or level == 'warning':
        logger.warning(message)
    elif level == 'error':
        logger.error(message)
    elif level == 'fatal' or level == 'critical':
        try:
            logger.fatal(message)
        except AttributeError:
            logger.critical(message)
    else:
        logger.info(message)
