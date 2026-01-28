#!/usr/bin/env python3
"""
SAINT.OS Raspberry Pi 5 Node Simulation Manager

Manages simulated Pi 5 nodes for development and testing.
Each simulated node runs as a separate Python process with mock GPIO.

Usage:
    ./rpi5_node_manager.py create <node_id> [--role ROLE]
    ./rpi5_node_manager.py start <node_id>
    ./rpi5_node_manager.py stop <node_id>
    ./rpi5_node_manager.py list
    ./rpi5_node_manager.py reset <node_id>
    ./rpi5_node_manager.py start-all
    ./rpi5_node_manager.py stop-all
    ./rpi5_node_manager.py logs <node_id>
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Any, Optional, List
from datetime import datetime


class NodeManager:
    """Manages simulated Pi 5 nodes."""

    def __init__(self):
        self.base_dir = Path(__file__).parent
        self.nodes_file = self.base_dir / "nodes.json"
        self.logs_dir = self.base_dir / "logs"
        self.config_dir = self.base_dir / "node_configs"

        # Ensure directories exist
        self.logs_dir.mkdir(exist_ok=True)
        self.config_dir.mkdir(exist_ok=True)

        # Load nodes registry
        self.nodes = self._load_nodes()

        # Path to saint_node module
        self.saint_node_path = self.base_dir.parent / "saint_node"

    def _load_nodes(self) -> Dict[str, Any]:
        """Load nodes registry from JSON file."""
        if self.nodes_file.exists():
            try:
                with open(self.nodes_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Warning: Failed to load nodes.json: {e}")
        return {}

    def _save_nodes(self):
        """Save nodes registry to JSON file."""
        with open(self.nodes_file, 'w') as f:
            json.dump(self.nodes, f, indent=2)

    def _is_process_running(self, pid: int) -> bool:
        """Check if a process is running."""
        try:
            os.kill(pid, 0)
            return True
        except (OSError, ProcessLookupError):
            return False

    def create(self, node_id: str, role: Optional[str] = None,
               display_name: Optional[str] = None) -> bool:
        """Create a new simulated node."""
        if node_id in self.nodes:
            print(f"Error: Node '{node_id}' already exists")
            return False

        # Create node entry
        self.nodes[node_id] = {
            "id": node_id,
            "role": role,
            "display_name": display_name or node_id,
            "created": datetime.now().isoformat(),
            "pid": None,
            "status": "stopped"
        }

        # Create node-specific config directory
        node_config_dir = self.config_dir / node_id
        node_config_dir.mkdir(exist_ok=True)

        # Create initial config file
        config = {
            "node_id": node_id,
            "role": role,
            "display_name": display_name or node_id,
            "adopted": role is not None,
            "pins": {},
            "network": {
                "ros_domain_id": 0
            }
        }

        config_file = node_config_dir / "config.yaml"
        import yaml
        with open(config_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

        self._save_nodes()
        print(f"Created node: {node_id}")
        return True

    def start(self, node_id: str) -> bool:
        """Start a simulated node."""
        if node_id not in self.nodes:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        node = self.nodes[node_id]

        # Check if already running
        if node.get("pid") and self._is_process_running(node["pid"]):
            print(f"Node '{node_id}' is already running (PID: {node['pid']})")
            return False

        # Prepare environment
        env = os.environ.copy()
        env["SAINT_SIMULATION"] = "1"
        env["SAINT_NODE_ID"] = node_id
        env["SAINT_CONFIG_DIR"] = str(self.config_dir / node_id)

        # ROS2 environment
        env["ROS_DOMAIN_ID"] = str(node.get("ros_domain_id", 0))

        # Log file
        log_file = self.logs_dir / f"{node_id}.log"

        # Build command
        cmd = [
            sys.executable, "-m", "saint_node.node"
        ]

        # Start process
        try:
            with open(log_file, 'a') as log:
                log.write(f"\n{'='*60}\n")
                log.write(f"Starting node at {datetime.now().isoformat()}\n")
                log.write(f"{'='*60}\n")
                log.flush()

                proc = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    cwd=str(self.saint_node_path.parent),
                    start_new_session=True
                )

            node["pid"] = proc.pid
            node["status"] = "running"
            node["started"] = datetime.now().isoformat()
            self._save_nodes()

            print(f"Started node '{node_id}' (PID: {proc.pid})")
            print(f"Log file: {log_file}")
            return True

        except Exception as e:
            print(f"Error starting node: {e}")
            return False

    def stop(self, node_id: str) -> bool:
        """Stop a simulated node."""
        if node_id not in self.nodes:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        node = self.nodes[node_id]
        pid = node.get("pid")

        if not pid:
            print(f"Node '{node_id}' is not running")
            return False

        if not self._is_process_running(pid):
            print(f"Node '{node_id}' process not found (stale PID)")
            node["pid"] = None
            node["status"] = "stopped"
            self._save_nodes()
            return True

        # Send SIGTERM
        try:
            os.kill(pid, signal.SIGTERM)
            print(f"Sent SIGTERM to node '{node_id}' (PID: {pid})")

            # Wait for graceful shutdown
            for _ in range(10):
                time.sleep(0.5)
                if not self._is_process_running(pid):
                    break
            else:
                # Force kill
                print(f"Node didn't stop gracefully, sending SIGKILL")
                os.kill(pid, signal.SIGKILL)
                time.sleep(0.5)

            node["pid"] = None
            node["status"] = "stopped"
            self._save_nodes()

            print(f"Stopped node '{node_id}'")
            return True

        except Exception as e:
            print(f"Error stopping node: {e}")
            return False

    def reset(self, node_id: str) -> bool:
        """Factory reset a node (clear config)."""
        if node_id not in self.nodes:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        # Stop if running
        self.stop(node_id)

        # Remove config
        node_config_dir = self.config_dir / node_id
        if node_config_dir.exists():
            import shutil
            shutil.rmtree(node_config_dir)
            node_config_dir.mkdir()

        # Reset node state
        self.nodes[node_id]["role"] = None
        self.nodes[node_id]["status"] = "stopped"
        self._save_nodes()

        print(f"Reset node '{node_id}'")
        return True

    def remove(self, node_id: str) -> bool:
        """Remove a node completely."""
        if node_id not in self.nodes:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        # Stop if running
        self.stop(node_id)

        # Remove config directory
        node_config_dir = self.config_dir / node_id
        if node_config_dir.exists():
            import shutil
            shutil.rmtree(node_config_dir)

        # Remove log file
        log_file = self.logs_dir / f"{node_id}.log"
        if log_file.exists():
            log_file.unlink()

        # Remove from registry
        del self.nodes[node_id]
        self._save_nodes()

        print(f"Removed node '{node_id}'")
        return True

    def list_nodes(self) -> List[Dict[str, Any]]:
        """List all nodes with their status."""
        result = []

        for node_id, node in self.nodes.items():
            # Check actual process status
            pid = node.get("pid")
            if pid and self._is_process_running(pid):
                status = "running"
            else:
                status = "stopped"
                if node.get("pid"):
                    node["pid"] = None
                    node["status"] = "stopped"

            result.append({
                "id": node_id,
                "role": node.get("role"),
                "display_name": node.get("display_name"),
                "status": status,
                "pid": node.get("pid"),
                "created": node.get("created")
            })

        self._save_nodes()
        return result

    def start_all(self) -> int:
        """Start all stopped nodes."""
        started = 0
        for node_id in self.nodes:
            node = self.nodes[node_id]
            if not node.get("pid") or not self._is_process_running(node["pid"]):
                if self.start(node_id):
                    started += 1
        return started

    def stop_all(self) -> int:
        """Stop all running nodes."""
        stopped = 0
        for node_id in self.nodes:
            node = self.nodes[node_id]
            if node.get("pid") and self._is_process_running(node["pid"]):
                if self.stop(node_id):
                    stopped += 1
        return stopped

    def logs(self, node_id: str, follow: bool = False, lines: int = 50):
        """Show logs for a node."""
        log_file = self.logs_dir / f"{node_id}.log"

        if not log_file.exists():
            print(f"No logs found for node '{node_id}'")
            return

        if follow:
            # Use tail -f
            subprocess.run(["tail", "-f", str(log_file)])
        else:
            # Show last N lines
            subprocess.run(["tail", "-n", str(lines), str(log_file)])


def main():
    parser = argparse.ArgumentParser(
        description="SAINT.OS Pi 5 Node Simulation Manager"
    )
    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # Create command
    create_parser = subparsers.add_parser("create", help="Create a new node")
    create_parser.add_argument("node_id", help="Unique node identifier")
    create_parser.add_argument("--role", help="Node role (head, arms_left, etc.)")
    create_parser.add_argument("--name", help="Display name")

    # Start command
    start_parser = subparsers.add_parser("start", help="Start a node")
    start_parser.add_argument("node_id", help="Node identifier")

    # Stop command
    stop_parser = subparsers.add_parser("stop", help="Stop a node")
    stop_parser.add_argument("node_id", help="Node identifier")

    # Reset command
    reset_parser = subparsers.add_parser("reset", help="Factory reset a node")
    reset_parser.add_argument("node_id", help="Node identifier")

    # Remove command
    remove_parser = subparsers.add_parser("remove", help="Remove a node completely")
    remove_parser.add_argument("node_id", help="Node identifier")

    # List command
    subparsers.add_parser("list", help="List all nodes")

    # Start-all command
    subparsers.add_parser("start-all", help="Start all nodes")

    # Stop-all command
    subparsers.add_parser("stop-all", help="Stop all nodes")

    # Logs command
    logs_parser = subparsers.add_parser("logs", help="Show node logs")
    logs_parser.add_argument("node_id", help="Node identifier")
    logs_parser.add_argument("-f", "--follow", action="store_true",
                            help="Follow log output")
    logs_parser.add_argument("-n", "--lines", type=int, default=50,
                            help="Number of lines to show")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    manager = NodeManager()

    if args.command == "create":
        return 0 if manager.create(args.node_id, args.role, args.name) else 1

    elif args.command == "start":
        return 0 if manager.start(args.node_id) else 1

    elif args.command == "stop":
        return 0 if manager.stop(args.node_id) else 1

    elif args.command == "reset":
        return 0 if manager.reset(args.node_id) else 1

    elif args.command == "remove":
        return 0 if manager.remove(args.node_id) else 1

    elif args.command == "list":
        nodes = manager.list_nodes()
        if not nodes:
            print("No nodes configured")
            return 0

        print(f"\n{'ID':<20} {'Role':<15} {'Status':<10} {'PID':<10}")
        print("-" * 60)
        for node in nodes:
            print(f"{node['id']:<20} {node['role'] or '-':<15} "
                  f"{node['status']:<10} {node['pid'] or '-':<10}")
        print()
        return 0

    elif args.command == "start-all":
        count = manager.start_all()
        print(f"Started {count} node(s)")
        return 0

    elif args.command == "stop-all":
        count = manager.stop_all()
        print(f"Stopped {count} node(s)")
        return 0

    elif args.command == "logs":
        manager.logs(args.node_id, args.follow, args.lines)
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
