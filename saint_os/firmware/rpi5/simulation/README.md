# Raspberry Pi 5 Node Simulation

This directory contains tools for simulating Pi 5 nodes without actual hardware.
Simulated nodes run as Python processes with mock GPIO, communicating via standard ROS2.

## Quick Start

```bash
# Create a simulated node
./rpi5_node_manager.py create sim_head --role head --name "Simulated Head"

# Start the node
./rpi5_node_manager.py start sim_head

# View logs
./rpi5_node_manager.py logs sim_head -f

# Stop the node
./rpi5_node_manager.py stop sim_head
```

## Requirements

- Python 3.10+
- ROS2 Jazzy or Humble (sourced)
- PyYAML (`pip install pyyaml`)

## Commands

| Command | Description |
|---------|-------------|
| `create <id> [--role ROLE]` | Create a new simulated node |
| `start <id>` | Start a node |
| `stop <id>` | Stop a node |
| `list` | List all nodes with status |
| `logs <id> [-f]` | View node logs |
| `reset <id>` | Factory reset (clear config) |
| `remove <id>` | Delete node completely |
| `start-all` | Start all nodes |
| `stop-all` | Stop all nodes |

## How It Works

1. **Mock GPIO**: The `SAINT_SIMULATION=1` environment variable enables mock GPIO mode
2. **Separate Configs**: Each node has its own config directory in `node_configs/`
3. **Process Management**: Nodes run as separate Python processes with their own PID
4. **ROS2 Communication**: Uses standard ROS2 DDS (same network as SAINT.OS server)

## Directory Structure

```
simulation/
├── rpi5_node_manager.py  # Node management script
├── nodes.json            # Node registry
├── node_configs/         # Per-node configuration
│   ├── sim_head/
│   │   └── config.yaml
│   └── sim_arm/
│       └── config.yaml
└── logs/                 # Node output logs
    ├── sim_head.log
    └── sim_arm.log
```

## Environment Variables

| Variable | Description |
|----------|-------------|
| `SAINT_SIMULATION` | Set to "1" to enable simulation mode |
| `SAINT_NODE_ID` | Override auto-generated node ID |
| `SAINT_CONFIG_DIR` | Override config directory path |
| `ROS_DOMAIN_ID` | ROS2 domain ID (default: 0) |

## Integration with SAINT.OS

Simulated nodes appear in the SAINT.OS web UI just like real hardware:

1. Start the SAINT.OS server
2. Start simulated node(s)
3. Open the web UI - nodes appear as "unadopted"
4. Adopt and configure nodes as normal
5. Control commands are received but GPIO is mocked

## Multiple Nodes Example

```bash
# Create several nodes
./rpi5_node_manager.py create sim_head --role head
./rpi5_node_manager.py create sim_left_arm --role arms_left
./rpi5_node_manager.py create sim_right_arm --role arms_right

# Start all
./rpi5_node_manager.py start-all

# Check status
./rpi5_node_manager.py list

# Stop all
./rpi5_node_manager.py stop-all
```

## Debugging

View real-time logs:
```bash
./rpi5_node_manager.py logs sim_head -f
```

Check ROS2 topics:
```bash
ros2 topic list | grep saint
ros2 topic echo /saint/nodes/announce
```

## Comparison with RP2040 Simulation

| Aspect | RP2040 | Pi 5 |
|--------|--------|------|
| Emulator | Renode | Native Python |
| ROS | micro-ROS + Agent | Full ROS2 |
| GPIO | Memory-mapped peripheral | Mock Python class |
| Networking | UDP Bridge peripheral | Direct DDS |
| Startup | ~5 seconds | ~1 second |
