#!/usr/bin/env python3
"""
SAINT.OS Unified Node Simulation Manager

Manages simulated nodes for both RP2040 (via Renode) and Raspberry Pi 5 (native Python).

Usage:
    ./node_manager.py create <node_id> --type <rp2040|rpi5> [options]
    ./node_manager.py start <node_id>
    ./node_manager.py stop <node_id>
    ./node_manager.py list
    ./node_manager.py reset <node_id>
    ./node_manager.py remove <node_id>
    ./node_manager.py logs <node_id> [-f]
    ./node_manager.py start-all [--type TYPE]
    ./node_manager.py stop-all [--type TYPE]

Node Types:
    rp2040  - Simulated via Renode (micro-ROS, requires micro-ROS agent)
    rpi5    - Native Python process (full ROS2, no agent needed)
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional, List


class NodeManager:
    """Unified manager for RP2040 and Pi 5 simulated nodes."""

    # Node types
    TYPE_RP2040 = "rp2040"
    TYPE_RPI5 = "rpi5"
    VALID_TYPES = [TYPE_RP2040, TYPE_RPI5]

    def __init__(self):
        # Base directories
        self.script_dir = Path(__file__).parent.resolve()
        self.firmware_dir = self.script_dir.parent

        # Central state and storage
        self.state_file = self.script_dir / "nodes.json"
        self.logs_dir = self.script_dir / "logs"
        self.logs_dir.mkdir(exist_ok=True)

        # RP2040 specific paths
        self.rp2040_dir = self.firmware_dir / "rp2040"
        self.renode_rp2040_path = self.rp2040_dir / "simulation" / "renode_rp2040"
        self.rp2040_install_dir = self.rp2040_dir / "install" / "simulation"
        self.rp2040_build_dir = self.rp2040_dir / "build_sim"
        self.rp2040_storage_dir = self.script_dir / "rp2040_storage"
        self.rp2040_scripts_dir = self.script_dir / "rp2040_scripts"

        # Pi 5 specific paths
        self.rpi5_dir = self.firmware_dir / "rpi5"
        self.rpi5_config_dir = self.script_dir / "rpi5_configs"

        # Renode path (configurable via environment)
        self.renode_path = os.environ.get(
            "RENODE_PATH",
            os.path.expanduser("~/Applications/Renode.app/Contents/MacOS/Renode")
        )

        # Base UDP port for RP2040 nodes
        self.base_udp_port = 9999

        # Load state
        self.state = self._load_state()

    def _load_state(self) -> Dict[str, Any]:
        """Load node state from JSON file."""
        if self.state_file.exists():
            try:
                with open(self.state_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Warning: Failed to load state: {e}")
        return {"nodes": {}, "next_udp_port": self.base_udp_port}

    def _save_state(self):
        """Save node state to JSON file."""
        with open(self.state_file, 'w') as f:
            json.dump(self.state, f, indent=2)

    def _is_process_running(self, pid: int) -> bool:
        """Check if a process is running."""
        try:
            os.kill(pid, 0)
            return True
        except (OSError, ProcessLookupError):
            return False

    def _ensure_dirs(self):
        """Create necessary directories."""
        self.logs_dir.mkdir(exist_ok=True)
        self.rp2040_storage_dir.mkdir(exist_ok=True)
        self.rp2040_scripts_dir.mkdir(exist_ok=True)
        self.rpi5_config_dir.mkdir(exist_ok=True)

    # =========================================================================
    # Common Operations
    # =========================================================================

    def create(self, node_id: str, node_type: str, role: Optional[str] = None,
               display_name: Optional[str] = None, udp_port: Optional[int] = None) -> bool:
        """Create a new simulated node."""
        if node_type not in self.VALID_TYPES:
            print(f"Error: Invalid node type '{node_type}'. Use: {self.VALID_TYPES}")
            return False

        if node_id in self.state["nodes"]:
            print(f"Error: Node '{node_id}' already exists")
            return False

        self._ensure_dirs()

        # Create node entry
        node = {
            "id": node_id,
            "type": node_type,
            "role": role,
            "display_name": display_name or node_id,
            "created": datetime.now().isoformat(),
            "pid": None,
            "status": "stopped",
        }

        # Type-specific setup
        if node_type == self.TYPE_RP2040:
            success = self._create_rp2040(node_id, node, udp_port)
        else:
            success = self._create_rpi5(node_id, node, role, display_name)

        if success:
            self.state["nodes"][node_id] = node
            self._save_state()
            print(f"Created {node_type} node: {node_id}")
            return True
        return False

    def start(self, node_id: str) -> bool:
        """Start a simulated node."""
        if node_id not in self.state["nodes"]:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        node = self.state["nodes"][node_id]

        # Check if already running
        if node.get("pid") and self._is_process_running(node["pid"]):
            print(f"Node '{node_id}' is already running (PID: {node['pid']})")
            return False

        # Type-specific start
        if node["type"] == self.TYPE_RP2040:
            return self._start_rp2040(node_id, node)
        else:
            return self._start_rpi5(node_id, node)

    def stop(self, node_id: str) -> bool:
        """Stop a running node."""
        if node_id not in self.state["nodes"]:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        node = self.state["nodes"][node_id]
        pid = node.get("pid")

        if not pid:
            print(f"Node '{node_id}' is not running")
            return True

        if not self._is_process_running(pid):
            print(f"Node '{node_id}' process not found (stale PID)")
            node["pid"] = None
            node["status"] = "stopped"
            self._save_state()
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
            self._save_state()
            print(f"Stopped node '{node_id}'")
            return True

        except Exception as e:
            print(f"Error stopping node: {e}")
            return False

    def reset(self, node_id: str) -> bool:
        """Factory reset a node (clear config/storage)."""
        if node_id not in self.state["nodes"]:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        # Stop if running
        self.stop(node_id)

        node = self.state["nodes"][node_id]

        # Type-specific reset
        if node["type"] == self.TYPE_RP2040:
            storage_path = self.rp2040_storage_dir / f"{node_id}.bin"
            if storage_path.exists():
                storage_path.unlink()
        else:
            config_dir = self.rpi5_config_dir / node_id
            if config_dir.exists():
                shutil.rmtree(config_dir)
                config_dir.mkdir()

        node["role"] = None
        node["status"] = "stopped"
        self._save_state()
        print(f"Reset node '{node_id}'")
        return True

    def remove(self, node_id: str) -> bool:
        """Remove a node completely."""
        if node_id not in self.state["nodes"]:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        # Stop if running
        self.stop(node_id)

        node = self.state["nodes"][node_id]

        # Type-specific cleanup
        if node["type"] == self.TYPE_RP2040:
            storage_path = self.rp2040_storage_dir / f"{node_id}.bin"
            script_path = self.rp2040_scripts_dir / f"{node_id}.resc"
            if storage_path.exists():
                storage_path.unlink()
            if script_path.exists():
                script_path.unlink()
        else:
            config_dir = self.rpi5_config_dir / node_id
            if config_dir.exists():
                shutil.rmtree(config_dir)

        # Remove log file
        log_file = self.logs_dir / f"{node_id}.log"
        if log_file.exists():
            log_file.unlink()

        # Remove from state
        del self.state["nodes"][node_id]
        self._save_state()
        print(f"Removed node '{node_id}'")
        return True

    def list_nodes(self, filter_type: Optional[str] = None) -> List[Dict[str, Any]]:
        """List all nodes with their status."""
        result = []

        for node_id, node in self.state["nodes"].items():
            if filter_type and node["type"] != filter_type:
                continue

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
                "type": node["type"],
                "role": node.get("role"),
                "display_name": node.get("display_name"),
                "status": status,
                "pid": node.get("pid"),
                "udp_port": node.get("udp_port"),  # RP2040 only
            })

        self._save_state()
        return result

    def start_all(self, filter_type: Optional[str] = None) -> int:
        """Start all stopped nodes."""
        started = 0
        for node_id, node in self.state["nodes"].items():
            if filter_type and node["type"] != filter_type:
                continue
            if not node.get("pid") or not self._is_process_running(node["pid"]):
                if self.start(node_id):
                    started += 1
        return started

    def stop_all(self, filter_type: Optional[str] = None) -> int:
        """Stop all running nodes."""
        stopped = 0
        for node_id, node in self.state["nodes"].items():
            if filter_type and node["type"] != filter_type:
                continue
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
            subprocess.run(["tail", "-f", str(log_file)])
        else:
            subprocess.run(["tail", "-n", str(lines), str(log_file)])

    # =========================================================================
    # RP2040 Specific
    # =========================================================================

    def _create_rp2040(self, node_id: str, node: Dict, udp_port: Optional[int]) -> bool:
        """Create RP2040 node configuration."""
        # Assign UDP port
        if udp_port is None:
            udp_port = self.state["next_udp_port"]
            self.state["next_udp_port"] = udp_port - 1

        node["udp_port"] = udp_port

        # Generate Renode script
        script_content = self._generate_rp2040_resc(node_id, udp_port)
        script_path = self.rp2040_scripts_dir / f"{node_id}.resc"

        with open(script_path, 'w') as f:
            f.write(script_content)

        print(f"  UDP Port: {udp_port}")
        print(f"  Script: {script_path}")
        return True

    def _generate_rp2040_resc(self, node_id: str, udp_port: int) -> str:
        """Generate Renode script for RP2040 node."""
        firmware_path = self.rp2040_install_dir / "saint_node.elf"

        return f'''# SAINT.OS RP2040 Node: {node_id}
# Auto-generated by node_manager.py

$machine_name="{node_id}"

path add @{self.renode_rp2040_path}

# Include peripheral C# definitions and create machine
include @{self.renode_rp2040_path}/cores/initialize_peripherals_simple.resc

# Load standard platform
machine LoadPlatformDescription @{self.renode_rp2040_path}/boards/adafruit_feather_rp2040.repl

# Configure peripherals for this specific node
sysbus.persistent_storage StoragePath "{self.rp2040_storage_dir}"
sysbus.persistent_storage NodeId "{node_id}"
sysbus.udp_bridge LocalPort {udp_port}

# Load bootrom and firmware
sysbus LoadELF @{self.renode_rp2040_path}/bootroms/rp2040/b2.elf
sysbus LoadELF @{firmware_path}

# Set vector table
sysbus.cpu0 VectorTableOffset 0x00000000
sysbus.cpu1 VectorTableOffset 0x00000000

# Keep cpu1 halted
cpu1 IsHalted true

# Log to file
logFile @{self.logs_dir}/{node_id}_renode.log true

# Show UART output
showAnalyzer sysbus.uart0

echo "SAINT.OS RP2040 Node: {node_id}"
echo "  UDP Port: {udp_port}"
echo "  Storage: {self.rp2040_storage_dir}/{node_id}.bin"

start
'''

    def _start_rp2040(self, node_id: str, node: Dict) -> bool:
        """Start an RP2040 node in Renode."""
        script_path = self.rp2040_scripts_dir / f"{node_id}.resc"

        if not script_path.exists():
            print(f"Error: Renode script not found: {script_path}")
            return False

        if not os.path.exists(self.renode_path):
            print(f"Error: Renode not found at: {self.renode_path}")
            print("Set RENODE_PATH environment variable or install Renode")
            return False

        # Check firmware
        firmware_path = self.rp2040_install_dir / "saint_node.elf"
        if not firmware_path.exists():
            print(f"Error: Firmware not found: {firmware_path}")
            print("Build firmware with: cd firmware/rp2040/build_sim && make install_sim")
            return False

        # Start Renode in background
        log_file = self.logs_dir / f"{node_id}.log"
        cmd = [self.renode_path, str(script_path)]

        try:
            with open(log_file, 'a') as log:
                log.write(f"\n{'='*60}\n")
                log.write(f"Starting RP2040 node at {datetime.now().isoformat()}\n")
                log.write(f"{'='*60}\n")

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )

            node["pid"] = proc.pid
            node["status"] = "running"
            node["started"] = datetime.now().isoformat()
            self._save_state()

            print(f"Started RP2040 node '{node_id}' (PID: {proc.pid})")
            print(f"  UDP Port: {node['udp_port']}")
            print(f"  Requires: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888")
            return True

        except Exception as e:
            print(f"Error starting node: {e}")
            return False

    # =========================================================================
    # Pi 5 Specific
    # =========================================================================

    def _create_rpi5(self, node_id: str, node: Dict,
                     role: Optional[str], display_name: Optional[str]) -> bool:
        """Create Pi 5 node configuration."""
        # Create config directory
        config_dir = self.rpi5_config_dir / node_id
        config_dir.mkdir(exist_ok=True)

        # Create initial config file
        import yaml
        config = {
            "node_id": node_id,
            "role": role,
            "display_name": display_name or node_id,
            "adopted": role is not None,
            "pins": {},
            "network": {"ros_domain_id": 0},
        }

        config_file = config_dir / "config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

        print(f"  Config: {config_dir}")
        return True

    def _start_rpi5(self, node_id: str, node: Dict) -> bool:
        """Start a Pi 5 node as native Python process."""
        # Prepare environment
        env = os.environ.copy()
        env["SAINT_SIMULATION"] = "1"
        env["SAINT_NODE_ID"] = node_id
        env["SAINT_CONFIG_DIR"] = str(self.rpi5_config_dir / node_id)
        env["ROS_DOMAIN_ID"] = str(node.get("ros_domain_id", 0))

        # Add saint_node to Python path
        saint_node_path = self.rpi5_dir
        if "PYTHONPATH" in env:
            env["PYTHONPATH"] = f"{saint_node_path}:{env['PYTHONPATH']}"
        else:
            env["PYTHONPATH"] = str(saint_node_path)

        # Log file
        log_file = self.logs_dir / f"{node_id}.log"

        # Build command
        cmd = [sys.executable, "-m", "saint_node.node"]

        try:
            with open(log_file, 'a') as log:
                log.write(f"\n{'='*60}\n")
                log.write(f"Starting Pi 5 node at {datetime.now().isoformat()}\n")
                log.write(f"{'='*60}\n")
                log.flush()

                proc = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    cwd=str(saint_node_path),
                    start_new_session=True
                )

            node["pid"] = proc.pid
            node["status"] = "running"
            node["started"] = datetime.now().isoformat()
            self._save_state()

            print(f"Started Pi 5 node '{node_id}' (PID: {proc.pid})")
            print(f"  Log: {log_file}")
            return True

        except Exception as e:
            print(f"Error starting node: {e}")
            return False

    # =========================================================================
    # Firmware Update (RP2040 only)
    # =========================================================================

    def update_firmware(self, node_id: Optional[str] = None) -> bool:
        """Update RP2040 firmware from build to install directory."""
        build_elf = self.rp2040_build_dir / "saint_node.elf"
        build_version = self.rp2040_build_dir / "generated" / "version.h"

        if not build_elf.exists():
            print(f"Error: No firmware build found at {build_elf}")
            print("Run 'make' in firmware/rp2040/build_sim/ first")
            return False

        # Ensure install directory exists
        self.rp2040_install_dir.mkdir(parents=True, exist_ok=True)

        # Copy firmware files
        install_elf = self.rp2040_install_dir / "saint_node.elf"
        install_version = self.rp2040_install_dir / "version.h"

        print(f"Installing firmware to {self.rp2040_install_dir}")
        shutil.copy2(build_elf, install_elf)
        print(f"  Copied: saint_node.elf")

        if build_version.exists():
            shutil.copy2(build_version, install_version)
            print(f"  Copied: version.h")

        # Restart node if specified
        if node_id:
            node = self.state["nodes"].get(node_id)
            if node and node["type"] == self.TYPE_RP2040:
                print(f"\nRestarting node '{node_id}'...")
                self.stop(node_id)
                time.sleep(1)
                self.start(node_id)

        return True


def main():
    parser = argparse.ArgumentParser(
        description="SAINT.OS Unified Node Simulation Manager",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Node Types:
  rp2040    RP2040 microcontroller (Renode + micro-ROS)
  rpi5      Raspberry Pi 5 (native Python + ROS2)

Examples:
  %(prog)s create head_sim --type rpi5 --role head
  %(prog)s create mcu_node --type rp2040 --port 9999
  %(prog)s start head_sim
  %(prog)s list
  %(prog)s logs head_sim -f
"""
    )
    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # Create command
    create_p = subparsers.add_parser("create", help="Create a new node")
    create_p.add_argument("node_id", help="Unique node identifier")
    create_p.add_argument("--type", "-t", required=True, choices=NodeManager.VALID_TYPES,
                         help="Node type (rp2040 or rpi5)")
    create_p.add_argument("--role", "-r", help="Node role (head, arms_left, etc.)")
    create_p.add_argument("--name", "-n", help="Display name")
    create_p.add_argument("--port", "-p", type=int, help="UDP port (RP2040 only)")

    # Start command
    start_p = subparsers.add_parser("start", help="Start a node")
    start_p.add_argument("node_id", help="Node identifier")

    # Stop command
    stop_p = subparsers.add_parser("stop", help="Stop a node")
    stop_p.add_argument("node_id", help="Node identifier")

    # Reset command
    reset_p = subparsers.add_parser("reset", help="Factory reset a node")
    reset_p.add_argument("node_id", help="Node identifier")

    # Remove command
    remove_p = subparsers.add_parser("remove", help="Remove a node completely")
    remove_p.add_argument("node_id", help="Node identifier")

    # List command
    list_p = subparsers.add_parser("list", help="List all nodes")
    list_p.add_argument("--type", "-t", choices=NodeManager.VALID_TYPES,
                       help="Filter by node type")

    # Logs command
    logs_p = subparsers.add_parser("logs", help="Show node logs")
    logs_p.add_argument("node_id", help="Node identifier")
    logs_p.add_argument("-f", "--follow", action="store_true", help="Follow log output")
    logs_p.add_argument("-n", "--lines", type=int, default=50, help="Number of lines")

    # Start-all command
    start_all_p = subparsers.add_parser("start-all", help="Start all nodes")
    start_all_p.add_argument("--type", "-t", choices=NodeManager.VALID_TYPES,
                            help="Filter by node type")

    # Stop-all command
    stop_all_p = subparsers.add_parser("stop-all", help="Stop all nodes")
    stop_all_p.add_argument("--type", "-t", choices=NodeManager.VALID_TYPES,
                           help="Filter by node type")

    # Update firmware command (RP2040 only)
    update_p = subparsers.add_parser("update-firmware", help="Update RP2040 firmware")
    update_p.add_argument("node_id", nargs="?", help="Node to restart after update")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    manager = NodeManager()

    if args.command == "create":
        return 0 if manager.create(
            args.node_id, args.type, args.role, args.name, args.port
        ) else 1

    elif args.command == "start":
        return 0 if manager.start(args.node_id) else 1

    elif args.command == "stop":
        return 0 if manager.stop(args.node_id) else 1

    elif args.command == "reset":
        return 0 if manager.reset(args.node_id) else 1

    elif args.command == "remove":
        return 0 if manager.remove(args.node_id) else 1

    elif args.command == "list":
        nodes = manager.list_nodes(args.type)
        if not nodes:
            print("No nodes configured")
            return 0

        print(f"\n{'ID':<20} {'Type':<8} {'Role':<12} {'Status':<10} {'PID':<8} {'Port':<6}")
        print("-" * 70)
        for node in nodes:
            port = node.get('udp_port') or '-'
            print(f"{node['id']:<20} {node['type']:<8} {node['role'] or '-':<12} "
                  f"{node['status']:<10} {node['pid'] or '-':<8} {port:<6}")
        print()
        return 0

    elif args.command == "logs":
        manager.logs(args.node_id, args.follow, args.lines)
        return 0

    elif args.command == "start-all":
        count = manager.start_all(args.type)
        print(f"Started {count} node(s)")
        return 0

    elif args.command == "stop-all":
        count = manager.stop_all(args.type)
        print(f"Stopped {count} node(s)")
        return 0

    elif args.command == "update-firmware":
        return 0 if manager.update_firmware(args.node_id) else 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
