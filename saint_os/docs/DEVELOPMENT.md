# SAINT.OS Development Guide

This guide covers setting up a development environment for SAINT.OS.

## Prerequisites

- ROS2 Humble (via conda or native install)
- Python 3.10+
- Docker (for macOS development)

## Environment Setup

Before running any ROS2 commands, activate the ROS2 environment:

**macOS (conda):**
```zsh
conda activate ros2_env
```

**Linux (native install):**
```bash
source /opt/ros/humble/setup.bash
```

## Quick Start (Recommended)

The easiest way to start development is with the dev script:

```zsh
# From the repository root
./saint_os/scripts/dev.sh
```

This script will:
1. Activate the conda environment (macOS) or source ROS2 (Linux)
2. Build the workspace
3. Source the workspace setup
4. Print instructions for running the micro-ROS agent
5. Start the SAINT.OS server

**With auto-rebuild on file changes:**
```zsh
./saint_os/scripts/dev.sh --watch
```

The `--watch` flag uses `fswatch` to automatically rebuild when Python files change.

**Note:** You still need to run the micro-ROS agent in a separate terminal (the script will print the command).

## Platform-Specific Setup

### Linux (Native Development)

On Linux (including Raspberry Pi), install the micro-ROS agent:

```bash
sudo apt install ros-humble-micro-ros-agent
```

Then build and launch:

```bash
source /opt/ros/humble/setup.bash
# From the repository root
cd saint_os
colcon build --symlink-install
source install/setup.bash
ros2 launch saint_os saint_server.launch.py
```

### macOS Development

The micro-ROS agent doesn't have a native macOS package. You can use Docker or build locally.

**Option A: Docker (easiest)**

```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

**Option B: Build Micro-XRCE-DDS-Agent locally**

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make

# Run the agent
./MicroXRCEAgent udp4 -p 8888
```

**Start the SAINT.OS server** (in a separate terminal):

```zsh
conda activate ros2_env
# From the repository root
cd saint_os
colcon build --symlink-install
source install/setup.zsh
ros2 launch saint_os saint_server.launch.py
```

Or use the dev script (recommended):
```zsh
./saint_os/scripts/dev.sh
```

## Running the Full Stack

### Components

1. **SAINT.OS Server** - Main ROS2 node with web interface
2. **micro-ROS Agent** - Bridges microcontroller nodes to ROS2
3. **Simulated Nodes** (optional) - Renode-based RP2040 simulation

### Quick Start (Linux/Raspberry Pi)

```bash
source /opt/ros/humble/setup.bash
# From the repository root
cd saint_os
source install/setup.bash  # after initial build
ros2 launch saint_os saint_server.launch.py
```

### Quick Start (macOS)

Terminal 1 - micro-ROS Agent (choose one):
```bash
# Docker
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4

# Or local build (adjust path as needed)
/path/to/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888
```

Terminal 2 - SAINT.OS Server:
```zsh
# From the repository root
./saint_os/scripts/dev.sh
```

### Starting Simulated Nodes

For testing without hardware, use the Renode-based simulation:

```bash
# From the repository root
cd saint_os/firmware/rp2040/simulation

# Create a node (first time only)
python3 saint_node_manager.py create node1

# Start the node
python3 saint_node_manager.py start node1 --wait
```

Or run Renode directly:
```bash
renode simulation/nodes/node1.resc
```

## Web Interface

Once the server is running, access the admin interface at:

- http://localhost:80/

The interface shows:
- System status (CPU, memory, uptime)
- Adopted and unadopted nodes
- Activity log

## Configuration

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `server_name` | SAINT-01 | Server identifier |
| `web_port` | 80 | Web interface port |
| `agent_port` | 8888 | micro-ROS agent UDP port |
| `enable_agent` | auto | Enable agent (auto-detected by platform) |

Example with custom settings:

```bash
ros2 launch saint_os saint_server.launch.py server_name:=MY-ROBOT web_port:=8080
```

## Troubleshooting

### "micro-ROS agent not found"

On Linux:
```bash
sudo apt install ros-humble-micro-ros-agent
```

On macOS, use Docker (see above).

### "Port 80 requires root"

Either run with sudo or use a higher port:
```bash
ros2 launch saint_os saint_server.launch.py web_port:=8080
```

### Nodes not appearing in web interface

1. Check that the micro-ROS agent is running
2. Verify nodes are publishing to `/saint/nodes/announce`
3. Check the agent port matches the firmware configuration (default: 8888)

### Docker networking on macOS

If nodes can't connect through Docker, ensure you're using port mapping:
```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888
```

Note: `--net=host` doesn't work on macOS Docker due to VM networking.

## Building

```bash
# Activate ROS2 environment first (see Environment Setup above)

# From the repository root
cd saint_os
colcon build --symlink-install

# Source the workspace for your shell:
source install/setup.bash  # Linux (bash)
source install/setup.zsh   # macOS (zsh)
```

## Testing

```bash
# From the saint_os directory
colcon test --packages-select saint_os

# View results
colcon test-result --verbose
```
