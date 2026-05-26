# SAINT.OS Node Simulation

Unified simulation manager for both RP2040 and Raspberry Pi 5 nodes.

## Quick Start

```bash
# Create nodes
./node_manager.py create head_sim --type rpi5 --role head
./node_manager.py create mcu_node --type rp2040

# Start nodes
./node_manager.py start head_sim
./node_manager.py start mcu_node

# Check status
./node_manager.py list

# View logs
./node_manager.py logs head_sim -f

# Stop nodes
./node_manager.py stop-all
```

## Node Types

| Type | Platform | Emulation | ROS | Requirements |
|------|----------|-----------|-----|--------------|
| `rp2040` | RP2040 MCU | Renode VM | micro-ROS | Renode, micro-ROS agent |
| `rpi5` | Raspberry Pi 5 | Native Python | Full ROS2 | ROS2 installed |

## Commands

| Command | Description |
|---------|-------------|
| `create <id> --type <type>` | Create a new simulated node |
| `start <id>` | Start a node |
| `stop <id>` | Stop a node |
| `list [--type TYPE]` | List all nodes |
| `logs <id> [-f]` | View node logs |
| `reset <id>` | Factory reset (clear config) |
| `remove <id>` | Delete node completely |
| `start-all [--type TYPE]` | Start all nodes |
| `stop-all [--type TYPE]` | Stop all nodes |
| `update-firmware [<id>]` | Update RP2040 firmware |

## Create Options

```bash
./node_manager.py create <node_id> --type <rp2040|rpi5> [options]

Options:
  --type, -t     Node type (required): rp2040 or rpi5
  --role, -r     Node role: head, arms_left, arms_right, tracks, etc.
  --name, -n     Display name
  --port, -p     UDP port (RP2040 only, auto-assigned if not specified)
```

## Examples

### Simulate a Complete Robot

```bash
# Create all nodes
./node_manager.py create sim_head --type rpi5 --role head --name "Head Controller"
./node_manager.py create sim_left_arm --type rp2040 --role arms_left
./node_manager.py create sim_right_arm --type rp2040 --role arms_right
./node_manager.py create sim_tracks --type rp2040 --role tracks

# Start RP2040 nodes (need micro-ROS agent first)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &

# Start all nodes
./node_manager.py start-all

# Check status
./node_manager.py list

# Stop when done
./node_manager.py stop-all
```

### Mixed Development

```bash
# Start only Pi 5 nodes (no agent needed)
./node_manager.py start-all --type rpi5

# Or only RP2040 nodes
./node_manager.py start-all --type rp2040
```

## Directory Structure

```
firmware/simulation/
├── node_manager.py      # Unified manager script
├── nodes.json           # Node registry
├── logs/                # Node output logs
├── rp2040_storage/      # RP2040 persistent storage
├── rp2040_scripts/      # Generated Renode scripts
└── rpi5_configs/        # Pi 5 node configurations
```

## RP2040 Nodes

RP2040 nodes run in Renode and communicate via micro-ROS:

```
RP2040 Node (Renode)  <--UDP-->  micro-ROS Agent  <--DDS-->  SAINT.OS Server
```

**Requirements:**
- Renode installed (set `RENODE_PATH` if not in default location)
- Firmware built: `cd firmware/rp2040/build_sim && make && make install_sim`
- micro-ROS agent running: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`

**Firmware Updates:**
```bash
# Build new firmware
cd firmware/rp2040/build_sim
make

# Install and optionally restart a node
./node_manager.py update-firmware sim_mcu
```

## Pi 5 Nodes

Pi 5 nodes run as native Python processes with mock GPIO:

```
Pi 5 Node (Python)  <--DDS-->  SAINT.OS Server
```

**Requirements:**
- ROS2 Jazzy or Humble sourced
- Python 3.10+

**Environment Variables:**
- `SAINT_SIMULATION=1` - Automatically set, enables mock GPIO
- `SAINT_NODE_ID` - Automatically set to node ID
- `SAINT_CONFIG_DIR` - Points to node-specific config directory

## Troubleshooting

### RP2040 node won't start
- Check Renode is installed: `ls ~/Applications/Renode.app` (macOS)
- Set custom path: `export RENODE_PATH=/path/to/renode`
- Check firmware exists: `ls firmware/rp2040/install/simulation/saint_node.elf`

### Pi 5 node won't start
- Source ROS2: `source /opt/ros/jazzy/setup.bash`
- Check Python path: `python3 -c "import saint_node"`

### Nodes not appearing in SAINT.OS
- For RP2040: Ensure micro-ROS agent is running
- Check ROS_DOMAIN_ID matches between nodes and server
- Verify network connectivity

### View detailed logs
```bash
# Follow logs in real-time
./node_manager.py logs <node_id> -f

# For RP2040, also check Renode logs
tail -f logs/<node_id>_renode.log
```
