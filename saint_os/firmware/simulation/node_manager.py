#!/usr/bin/env python3
"""
SAINT.OS Unified Node Simulation Manager

Manages simulated nodes for both RP2040 (via Renode) and Raspberry Pi 5 (native Python).

Usage:
    ./node_manager.py create <node_id> --type <rp2040|rpi5> [options]
    ./node_manager.py start <node_id>
    ./node_manager.py run <node_id>              # Start + follow logs (Ctrl+C detaches)
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

    def _check_process_health(self, node_id: str) -> dict:
        """
        Check detailed health status of a node's process.

        Returns dict with:
            - running: bool - process exists
            - healthy: bool - process appears healthy
            - exit_code: int or None - if process exited
            - message: str - status description
        """
        if node_id not in self.state["nodes"]:
            return {"running": False, "healthy": False, "exit_code": None,
                    "message": "Node not found"}

        node = self.state["nodes"][node_id]
        pid = node.get("pid")

        if not pid:
            return {"running": False, "healthy": False, "exit_code": None,
                    "message": "No PID recorded"}

        try:
            # Check if process exists
            os.kill(pid, 0)

            # For more detailed health, check /proc on Linux or ps on macOS
            import platform
            if platform.system() == "Darwin":
                result = subprocess.run(
                    ["ps", "-p", str(pid), "-o", "state="],
                    capture_output=True, text=True
                )
                if result.returncode == 0:
                    state = result.stdout.strip()
                    # macOS states: R=running, S=sleeping, U=uninterruptible, etc.
                    if state and state[0] in "RSU":
                        return {"running": True, "healthy": True, "exit_code": None,
                                "message": f"Process running (state: {state})"}

            return {"running": True, "healthy": True, "exit_code": None,
                    "message": "Process running"}

        except ProcessLookupError:
            # Process doesn't exist - try to get exit code
            return {"running": False, "healthy": False, "exit_code": -1,
                    "message": "Process terminated unexpectedly"}
        except OSError as e:
            return {"running": False, "healthy": False, "exit_code": None,
                    "message": f"Error checking process: {e}"}

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

    def start(self, node_id: str, foreground: bool = False) -> bool:
        """Start a simulated node.

        Args:
            node_id: Node identifier
            foreground: If True, run in foreground with output to terminal (blocking).
                        If False, run in background with output to log file.
        """
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
            return self._start_rp2040(node_id, node, foreground=foreground)
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

    def health_check(self, filter_type: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Check health of all running nodes and report status.

        Returns list of nodes with health issues.
        """
        issues = []

        for node_id, node in self.state["nodes"].items():
            if filter_type and node["type"] != filter_type:
                continue

            if not node.get("pid"):
                continue  # Not supposed to be running

            health = self._check_process_health(node_id)

            if not health["running"]:
                issues.append({
                    "node_id": node_id,
                    "type": node["type"],
                    "issue": "crashed",
                    "message": health["message"],
                    "exit_code": health.get("exit_code"),
                })
                # Update state to reflect crash
                node["pid"] = None
                node["status"] = "crashed"
                self._save_state()
            elif not health["healthy"]:
                issues.append({
                    "node_id": node_id,
                    "type": node["type"],
                    "issue": "unhealthy",
                    "message": health["message"],
                })

        return issues

    def restart(self, node_id: str) -> bool:
        """Stop and start a node."""
        print(f"Restarting node '{node_id}'...")
        self.stop(node_id)
        time.sleep(1)
        return self.start(node_id)

    def restart_crashed(self, filter_type: Optional[str] = None) -> int:
        """
        Check for crashed nodes and restart them.

        Returns count of restarted nodes.
        """
        issues = self.health_check(filter_type)
        restarted = 0

        for issue in issues:
            if issue["issue"] == "crashed":
                node_id = issue["node_id"]
                print(f"Detected crashed node: {node_id} - {issue['message']}")
                if self.start(node_id):
                    print(f"Successfully restarted: {node_id}")
                    restarted += 1
                else:
                    print(f"Failed to restart: {node_id}")

        return restarted

    def monitor(self, interval: float = 5.0, auto_restart: bool = True,
                filter_type: Optional[str] = None):
        """
        Continuously monitor nodes and optionally auto-restart crashed ones.

        Args:
            interval: Check interval in seconds
            auto_restart: Whether to automatically restart crashed nodes
            filter_type: Only monitor nodes of this type
        """
        print(f"Starting node monitor (interval: {interval}s, auto_restart: {auto_restart})")
        print("Press Ctrl+C to stop monitoring\n")

        try:
            while True:
                # Check all nodes
                issues = self.health_check(filter_type)

                if issues:
                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Health check found {len(issues)} issue(s):")
                    for issue in issues:
                        print(f"  - {issue['node_id']}: {issue['issue']} - {issue['message']}")

                    if auto_restart:
                        for issue in issues:
                            if issue["issue"] == "crashed":
                                node_id = issue["node_id"]
                                print(f"  Auto-restarting: {node_id}")
                                if self.start(node_id):
                                    print(f"  Successfully restarted: {node_id}")
                                else:
                                    print(f"  Failed to restart: {node_id}")

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n\nMonitor stopped")

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

    def logs(self, node_id: str, follow: bool = False, lines: int = 50,
             log_type: str = "all"):
        """Show logs for a node.

        Args:
            node_id: Node identifier
            follow: Follow log output (like tail -f)
            lines: Number of lines to show initially
            log_type: 'uart' for firmware console output, 'renode' for Renode system logs,
                      'all' for combined output
        """
        node = self.state["nodes"].get(node_id)
        if not node:
            print(f"Error: Node '{node_id}' does not exist")
            return

        # Determine which log file(s) to show based on node type
        if node["type"] == self.TYPE_RP2040:
            renode_log = self.logs_dir / f"{node_id}_renode.log"
            main_log = self.logs_dir / f"{node_id}.log"

            if log_type == "uart":
                # UART output is in the main log (Renode stdout)
                log_file = main_log
            elif log_type == "renode":
                log_file = renode_log if renode_log.exists() else main_log
            else:  # all
                # For 'all', show both logs
                if follow:
                    # Follow both logs using tail -f
                    log_files = []
                    if main_log.exists():
                        log_files.append(str(main_log))
                    if renode_log.exists():
                        log_files.append(str(renode_log))

                    if not log_files:
                        print(f"No logs found for node '{node_id}'")
                        return

                    print(f"Following logs: {', '.join(log_files)}")
                    print("Press Ctrl+C to stop\n")
                    try:
                        subprocess.run(["tail", "-f"] + log_files)
                    except KeyboardInterrupt:
                        print("\n\nStopped following logs")
                    return
                else:
                    print(f"=== Console Output ({main_log.name}) ===")
                    if main_log.exists():
                        subprocess.run(["tail", "-n", str(lines), str(main_log)])
                    else:
                        print("  (no console log yet)")
                    print(f"\n=== Renode Log ({renode_log.name}) ===")
                    if renode_log.exists():
                        subprocess.run(["tail", "-n", str(lines // 2), str(renode_log)])
                    else:
                        print("  (no Renode log yet)")
                    return
        else:
            # Pi 5 nodes just have a single log
            log_file = self.logs_dir / f"{node_id}.log"

        if not log_file.exists():
            print(f"No logs found for node '{node_id}'")
            print(f"  Expected: {log_file}")
            return

        if follow:
            self._follow_log(log_file)
        else:
            subprocess.run(["tail", "-n", str(lines), str(log_file)])

    def _follow_log(self, log_file: Path):
        """Follow a log file with proper Ctrl+C handling."""
        import select
        
        try:
            with open(log_file, 'r') as f:
                # Go to end of file
                f.seek(0, 2)
                
                while True:
                    line = f.readline()
                    if line:
                        print(line, end='', flush=True)
                    else:
                        # No new data, wait a bit
                        time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nStopped following logs (node still running)")
            print(f"  To stop node: ./node_manager.py stop <node_id>")

    def run(self, node_id: str, lines: int = 50, log_type: str = "uart") -> bool:
        """Start a node in foreground with output to terminal. Ctrl+C stops the node."""
        # Run in foreground mode - this blocks until node exits or Ctrl+C
        return self.start(node_id, foreground=True)

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

# Reduce CPU translation block size to minimize dirty address accumulation
# (helps prevent memory exhaustion from frequent memory-mapped I/O)
cpu0 MaximumBlockSize 1

# Show UART output in Renode console (required for --console mode)
showAnalyzer sysbus.uart0

# Log all UART output at highest verbosity
logLevel -1 sysbus.uart0

echo "SAINT.OS RP2040 Node: {node_id}"
echo "  UDP Port: {udp_port}"
echo "  Storage: {self.rp2040_storage_dir}/{node_id}.bin"

start
'''

    def _start_rp2040(self, node_id: str, node: Dict, foreground: bool = False) -> bool:
        """Start an RP2040 node in Renode.

        Args:
            node_id: Node identifier
            node: Node state dict
            foreground: If True, run in foreground with output to terminal.
                        If False, run in background with output to log file.
        """
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

        log_file = self.logs_dir / f"{node_id}.log"
        # Use --disable-xwt for headless operation (no GUI)
        cmd = [self.renode_path, "--disable-xwt", "--console", str(script_path)]

        try:
            if foreground:
                # Run in foreground - output goes directly to terminal
                print(f"\n{'='*60}")
                print(f"Starting RP2040 node '{node_id}' in foreground")
                print(f"  UDP Port: {node['udp_port']}")
                print(f"  Press Ctrl+C to stop")
                print(f"  Requires: micro-ROS agent on port 8888")
                print(f"{'='*60}\n")

                proc = subprocess.Popen(
                    cmd,
                    stdout=sys.stdout,
                    stderr=sys.stderr,
                    stdin=sys.stdin,
                )

                node["pid"] = proc.pid
                node["status"] = "running"
                node["started"] = datetime.now().isoformat()
                self._save_state()

                try:
                    proc.wait()
                except KeyboardInterrupt:
                    print("\n\nStopping node...")
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()

                node["pid"] = None
                node["status"] = "stopped"
                self._save_state()
                return True
            else:
                # Run in background - output goes to log file
                with open(log_file, 'a') as log:
                    log.write(f"\n{'='*60}\n")
                    log.write(f"Starting RP2040 node at {datetime.now().isoformat()}\n")
                    log.write(f"{'='*60}\n")
                    log.flush()

                    proc = subprocess.Popen(
                        cmd,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        stdin=subprocess.DEVNULL,
                    )

                node["pid"] = proc.pid
                node["status"] = "running"
                node["started"] = datetime.now().isoformat()
                self._save_state()

                print(f"Started RP2040 node '{node_id}' (PID: {proc.pid})")
                print(f"  UDP Port: {node['udp_port']}")
                print(f"  Log: {log_file}")
                print(f"  Requires: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888")
                return True

        except Exception as e:
            print(f"Error starting node: {e}")
            import traceback
            traceback.print_exc()
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

    def regenerate_script(self, node_id: str) -> bool:
        """Regenerate Renode script for an existing RP2040 node.

        Useful when the script template has been updated and you want to
        apply changes without recreating the node.
        """
        if node_id not in self.state["nodes"]:
            print(f"Error: Node '{node_id}' does not exist")
            return False

        node = self.state["nodes"][node_id]
        if node["type"] != self.TYPE_RP2040:
            print(f"Error: Node '{node_id}' is not an RP2040 node")
            return False

        udp_port = node.get("udp_port")
        if not udp_port:
            print(f"Error: Node '{node_id}' has no UDP port assigned")
            return False

        # Stop if running
        was_running = node.get("pid") and self._is_process_running(node["pid"])
        if was_running:
            print(f"Stopping node '{node_id}'...")
            self.stop(node_id)

        # Regenerate script
        script_content = self._generate_rp2040_resc(node_id, udp_port)
        script_path = self.rp2040_scripts_dir / f"{node_id}.resc"

        with open(script_path, 'w') as f:
            f.write(script_content)

        print(f"Regenerated Renode script: {script_path}")

        # Restart if was running
        if was_running:
            print(f"Restarting node '{node_id}'...")
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
  %(prog)s run head_sim          # Start and follow logs (Ctrl+C to detach)
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

    # Run command (start + follow logs)
    run_p = subparsers.add_parser("run", help="Start a node and follow its logs")
    run_p.add_argument("node_id", help="Node identifier")
    run_p.add_argument("-n", "--lines", type=int, default=50, help="Initial lines to show")
    run_p.add_argument("--log-type", choices=["uart", "renode"], default="uart",
                       help="Log type to follow: uart (firmware console) or renode (system)")

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
    logs_p.add_argument("--type", choices=["uart", "renode", "all"], default="all",
                       help="Log type: uart (firmware console), renode (system), all (default)")

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

    # Regenerate script command (RP2040 only)
    regen_p = subparsers.add_parser("regenerate-script",
                                    help="Regenerate Renode script for RP2040 node")
    regen_p.add_argument("node_id", help="Node identifier")

    # Health check command
    health_p = subparsers.add_parser("health", help="Check health of running nodes")
    health_p.add_argument("--type", "-t", choices=NodeManager.VALID_TYPES,
                         help="Filter by node type")

    # Restart command
    restart_p = subparsers.add_parser("restart", help="Restart a node")
    restart_p.add_argument("node_id", help="Node identifier")

    # Monitor command
    monitor_p = subparsers.add_parser("monitor", help="Monitor nodes and auto-restart on crash")
    monitor_p.add_argument("--interval", "-i", type=float, default=5.0,
                          help="Check interval in seconds (default: 5)")
    monitor_p.add_argument("--no-restart", action="store_true",
                          help="Don't auto-restart crashed nodes")
    monitor_p.add_argument("--type", "-t", choices=NodeManager.VALID_TYPES,
                          help="Filter by node type")

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

    elif args.command == "run":
        return 0 if manager.run(args.node_id, args.lines, args.log_type) else 1

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
        manager.logs(args.node_id, args.follow, args.lines, args.type)
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

    elif args.command == "regenerate-script":
        return 0 if manager.regenerate_script(args.node_id) else 1

    elif args.command == "health":
        issues = manager.health_check(args.type)
        if not issues:
            print("All running nodes are healthy")
        else:
            print(f"Found {len(issues)} issue(s):")
            for issue in issues:
                print(f"  {issue['node_id']}: {issue['issue']} - {issue['message']}")
        return 0

    elif args.command == "restart":
        return 0 if manager.restart(args.node_id) else 1

    elif args.command == "monitor":
        manager.monitor(
            interval=args.interval,
            auto_restart=not args.no_restart,
            filter_type=args.type
        )
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
