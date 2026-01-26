# SAINT.OS Development Guide

This guide covers setting up a development environment for SAINT.OS.

## Prerequisites

- ROS2 Humble (via conda or native install)
- Python 3.10+
- Docker (for macOS development)

## Platform-Specific Setup

### Linux (Native Development)

On Linux (including Raspberry Pi), install the micro-ROS agent:

```bash
sudo apt install ros-humble-micro-ros-agent
```

Then launch everything with a single command:

```bash
ros2 launch saint_os saint_server.launch.py
```

### macOS Development

The micro-ROS agent doesn't have a native macOS package. Use Docker instead:

**1. Install Docker Desktop** (if not already installed)
   - Download from https://www.docker.com/products/docker-desktop/

**2. Start the micro-ROS agent in Docker:**

```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

**3. In another terminal, start the SAINT.OS server:**

```bash
cd ~/Projects/OpenSAINT/SaintOS/source
source install/setup.bash
ros2 launch saint_os saint_server.launch.py
```

Or run just the server node (without trying to start the agent):

```bash
ros2 run saint_os saint_server
```

## Running the Full Stack

### Components

1. **SAINT.OS Server** - Main ROS2 node with web interface
2. **micro-ROS Agent** - Bridges microcontroller nodes to ROS2
3. **Simulated Nodes** (optional) - Renode-based RP2040 simulation

### Quick Start (Linux/Raspberry Pi)

```bash
# Single command starts server + agent
ros2 launch saint_os saint_server.launch.py
```

### Quick Start (macOS)

Terminal 1 - micro-ROS Agent:
```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

Terminal 2 - SAINT.OS Server:
```bash
cd ~/Projects/OpenSAINT/SaintOS/source
source install/setup.bash
ros2 launch saint_os saint_server.launch.py
```

### Starting Simulated Nodes

For testing without hardware, use the Renode-based simulation:

```bash
cd saint_os/firmware/rp2040/simulation

# Create a node (first time only)
python3 saint_node_manager.py create node1

# Start the node
python3 saint_node_manager.py start node1 --wait
```

Or run Renode directly:
```bash
~/Applications/Renode.app/Contents/MacOS/Renode simulation/nodes/node1.resc
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
cd ~/Projects/OpenSAINT/SaintOS/source
colcon build --packages-select saint_os --symlink-install
source install/setup.bash
```

## Testing

```bash
# Run tests
colcon test --packages-select saint_os

# View results
colcon test-result --verbose
```
