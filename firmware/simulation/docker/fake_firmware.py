#!/usr/bin/env python3
"""Python stand-in for the SAINT firmware, for end-to-end server tests.

Speaks the same ROS2 protocol the real firmware does, minus the XRCE
bridge that's only relevant on actual hardware. Used by the docker e2e
when Renode-side simulation isn't usable (RP2040 bootrom/XIP modelling
gap in current Renode versions blocks the real firmware from booting).

Topics:
    publishes  /saint/nodes/announce                     1 Hz, like firmware
    publishes  /saint/nodes/<id>/log                     on state events
    subscribes /saint/nodes/<id>/config                  applies → log
    subscribes /saint/nodes/<id>/control                 best-effort stream
    subscribes /saint/nodes/<id>/command                 reset / restart / estop
    subscribes /test/fake_firmware/<id>/control          test-only escape hatch
                                                         (simulates flash loss
                                                         without telling the
                                                         server, for testing
                                                         the auto-reconcile path)

State machine matches firmware: UNADOPTED → ACTIVE on first valid
configure receipt. ACTIVE → UNADOPTED on factory_reset or test reset.

Each side-effect that the real firmware would log via saint_log_publish
is mirrored here so the dashboard's Logs tab + the e2e harness see the
same wire format.
"""
from __future__ import annotations

import argparse
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import String


# Match server_node.py:CONTROL_QOS. Joystick stream uses best-effort so
# DDS won't queue stale samples behind a single dropped packet.
CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class FakeFirmware(Node):
    """Minimal SAINT firmware shape, in Python."""

    def __init__(self, node_id: str, hw_model: str = "Fake RP2040 (Python)"):
        super().__init__(f"fake_firmware_{node_id}")
        self.node_id = node_id
        self.hw_model = hw_model
        self.state = "UNADOPTED"
        self.boot_time = time.time()
        self.announce_count = 0
        self.config_apply_should_fail = False  # for failure-path tests

        # ─── Publishers ────────────────────────────────────────────
        # Announcement is one-to-server, log is per-node. Both
        # reliable + keep_last(10) — matches what saint_log_publish
        # uses on the firmware side.
        self.announce_pub = self.create_publisher(
            String, "/saint/nodes/announce", 10)
        self.log_pub = self.create_publisher(
            String, f"/saint/nodes/{node_id}/log", 10)

        # ─── Subscriptions ─────────────────────────────────────────
        self.config_sub = self.create_subscription(
            String, f"/saint/nodes/{node_id}/config",
            self._on_config, 10)
        self.control_sub = self.create_subscription(
            String, f"/saint/nodes/{node_id}/control",
            self._on_control, CONTROL_QOS)
        self.command_sub = self.create_subscription(
            String, f"/saint/nodes/{node_id}/command",
            self._on_command, 10)

        # Test-only control channel. Lets the e2e harness simulate
        # "firmware lost its flash config without telling the server"
        # — which is the scenario the auto-reconcile is designed for.
        self.test_sub = self.create_subscription(
            String, f"/test/fake_firmware/{node_id}/control",
            self._on_test_control, 10)

        # 1 Hz announce timer matches firmware/shared/include/saint_types.h
        # ANNOUNCE_INTERVAL_MS = 1000.
        self.create_timer(1.0, self._publish_announcement)

        self.get_logger().info(f"FakeFirmware {node_id} ready")
        self._publish_log("info", "Fake firmware ready. Waiting for adoption.")

    # ── Outbound ───────────────────────────────────────────────────

    def _publish_announcement(self) -> None:
        self.announce_count += 1
        uptime_s = int(time.time() - self.boot_time)
        payload = {
            "node_id": self.node_id,
            "chip_family": "rp2040",
            "chip_id": "0x10002927",
            "mac": "02:5C:E8:E7:9F:55",
            "ip": "192.168.1.100",
            "hw": self.hw_model,
            "fw": "fake-1.0.0",
            "bl_fw": "fake-bl-1.0",
            "fw_build": "fake-build",
            "state": self.state,
            "uptime": uptime_s,
            "cpu_temp": 25.0,
            "peripherals": {
                "fake_connected": True,
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.announce_pub.publish(msg)

    def _publish_log(self, level: str, text: str) -> None:
        """Mirror saint_log_publish from the firmware side."""
        entry = {
            "level": level,
            "text": text,
            "uptime_ms": int((time.time() - self.boot_time) * 1000),
        }
        msg = String()
        msg.data = json.dumps(entry)
        self.log_pub.publish(msg)

    # ── Inbound ────────────────────────────────────────────────────

    def _on_config(self, msg: String) -> None:
        """Mirror config_subscription_callback in firmware main.c."""
        data = msg.data or ""
        n = len(data)
        # Direct stdout breadcrumb so we can confirm the callback fired
        # even if subsequent _publish_log goes nowhere. Mirrors what
        # the real firmware does with a printf() to UART.
        print(f"[fake_firmware] _on_config received {n} bytes: {data[:80]!r}",
              flush=True)
        # Same substring check the real firmware does.
        if ('"action":"configure"' not in data
                and '"action": "configure"' not in data):
            return

        self._publish_log("info", f"Config received ({n} bytes), applying…")
        time.sleep(0.05)  # Mimic ~50 ms apply latency on the firmware.

        if self.config_apply_should_fail:
            self._publish_log("error", "Config apply failed")
            return

        self._publish_log("info", "Config applied OK")
        self._publish_log("info", "Config saved to flash")

        if self.state != "ACTIVE":
            self._publish_log("info", "Adopted — entering ACTIVE state")
            self.state = "ACTIVE"

    def _on_control(self, msg: String) -> None:
        # Streaming joystick / control messages — the real firmware
        # routes these straight into peripheral set_value() calls. For
        # the e2e we just receipt-log occasionally so floods don't
        # spam the dashboard.
        if self.announce_count % 50 == 0:  # ~once every 50 s of operation
            self._publish_log("debug", f"control msg ({len(msg.data or '')} bytes)")

    def _on_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or "{}")
        except Exception:
            self._publish_log("warn",
                f"Malformed command JSON: {(msg.data or '')[:80]!r}")
            return

        action = data.get("action", "")
        if action == "factory_reset":
            self._publish_log("warn", "Factory reset received — clearing config")
            self.state = "UNADOPTED"
            self.config_apply_should_fail = False
        elif action == "restart":
            self._publish_log("info", "Restart command received — rebooting")
            self.boot_time = time.time()
            self.state = "UNADOPTED"
            self.announce_count = 0
        elif action == "identify":
            self._publish_log("info", "Identify command — blinking LED (simulated)")
        elif action == "estop":
            self._publish_log("warn", "ESTOP — all channels stopped")
        else:
            self._publish_log("debug", f"Unknown command action: {action!r}")

    def _on_test_control(self, msg: String) -> None:
        """Test-only escape hatch.

        Lets the e2e harness manipulate the fake firmware's state
        without going through the real wire-protocol commands. Used
        specifically for testing the server's auto-reconcile path
        (simulate "firmware lost flash without telling the server").
        """
        try:
            data = json.loads(msg.data or "{}")
        except Exception:
            return
        action = data.get("action", "")

        if action == "lose_config":
            # Pretend an OTA wiped the flash-saved config. Don't tell
            # the server — that's what makes the server's auto-
            # reconcile path observable from this side: server's
            # adopted_nodes still has us, but we'll start announcing
            # UNADOPTED.
            self._publish_log("warn",
                "(test) simulated flash-config loss — going UNADOPTED")
            self.state = "UNADOPTED"
        elif action == "fail_next_apply":
            self.config_apply_should_fail = True
        elif action == "reboot_fresh":
            # Full reboot simulation: announce count, uptime, state.
            self.boot_time = time.time()
            self.state = "UNADOPTED"
            self.announce_count = 0
        elif action == "send_malformed_announce":
            # Bypass the regular announcement loop to send something
            # truncated/garbage. Lets the e2e exercise the server's
            # _report_announcement_parse_error visibility path.
            bad = (
                '{"node_id":"' + self.node_id + '","chip_family":"rp2040'
            )  # JSON cut mid-string
            msg_out = String()
            msg_out.data = bad
            self.announce_pub.publish(msg_out)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--node-id", default="rp2040_fakefw",
                        help="Node identifier to announce as.")
    parser.add_argument("--hw-model", default="Fake RP2040 (Python)",
                        help="hw_model string in announcements.")
    args = parser.parse_args()

    rclpy.init()
    node = FakeFirmware(args.node_id, hw_model=args.hw_model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
