# SAINT.OS Node Simulation with Renode

This document describes how to run the SAINT.OS node firmware in the Renode RP2040 simulator with full UDP network connectivity.

## Overview

The simulation uses a custom **UDP Bridge** peripheral that provides direct UDP socket access from the simulated firmware to the host network. This allows:

- Multiple simulated nodes running simultaneously
- Real network communication with the micro-ROS agent
- Full ROS2 topic/service communication with the SAINT.OS server

## Prerequisites

1. **Renode** - Download from https://renode.io/ (tested with v1.16.0)
   - macOS: Install to `~/Applications/Renode.app`

2. **Renode RP2040 Package** - Already included in `firmware/rp2040/simulation/renode_rp2040/`

3. **Firmware built for simulation**
   ```bash
   cd firmware/rp2040
   mkdir -p build && cd build
   cmake -DSIMULATION=ON ..
   make
   ```

   > **Note:** The Pico SDK and micro-ROS paths are auto-detected from:
   > - `lib/pico-sdk` (local) or `~/pico-sdk` or `$PICO_SDK_PATH`
   > - `lib/micro_ros_raspberrypi_pico_sdk` (local) or `$MICRO_ROS_PATH`
   >
   > The node manager expects firmware at `build/saint_node.elf`. If you use a
   > different build directory (e.g., `build_sim`), copy the firmware:
   > ```bash
   > cp build_sim/saint_node.elf build/
   > ```

4. **micro-ROS Agent** (for ROS2 connectivity)

   On Linux:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

   On macOS (via Docker):
   ```bash
   docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4
   ```

## Running a Single Node

### Using the Node Manager (Recommended)

The node manager script handles creating and running simulated nodes:

```bash
cd firmware/rp2040/simulation

# Create a node (first time only)
python3 saint_node_manager.py create node1

# Start the node (foreground)
python3 saint_node_manager.py start node1 --wait

# Or start in background
python3 saint_node_manager.py start node1
python3 saint_node_manager.py stop node1
```

### Using Renode Directly

```bash
cd firmware/rp2040/simulation
~/Applications/Renode.app/Contents/MacOS/Renode nodes/node1.resc
```

The simulation auto-starts. Use Renode commands:
```
pause    # Pause simulation
start    # Resume simulation
quit     # Exit Renode
```

### Start the micro-ROS Agent

In another terminal:

Linux:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

macOS (Docker):
```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

### Verify Connection

```bash
ros2 topic list
ros2 topic echo /saint/nodes/announce
```

## Running Multiple Nodes

Create and start multiple nodes:

```bash
cd firmware/rp2040/simulation

# Create nodes
python3 saint_node_manager.py create node1 --port 9999
python3 saint_node_manager.py create node2 --port 9998
python3 saint_node_manager.py create node3 --port 9997

# Start all nodes (background)
python3 saint_node_manager.py start-all

# List status
python3 saint_node_manager.py list

# Stop all
python3 saint_node_manager.py stop-all
```

Each node gets its own UDP port for communication:
- Node 1: UDP port 9999
- Node 2: UDP port 9998
- Node 3: UDP port 9997

All nodes communicate with the same micro-ROS agent on port 8888.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Host Machine                             │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Renode Simulation                      │  │
│  │  ┌──────────┐   ┌──────────┐   ┌──────────┐             │  │
│  │  │  Node 1  │   │  Node 2  │   │  Node 3  │   ...       │  │
│  │  │ RP2040   │   │ RP2040   │   │ RP2040   │             │  │
│  │  │UDP:9999  │   │UDP:9998  │   │UDP:9997  │             │  │
│  │  └────┬─────┘   └────┬─────┘   └────┬─────┘             │  │
│  └───────┼──────────────┼──────────────┼────────────────────┘  │
│          │              │              │                        │
│          └──────────────┼──────────────┘                        │
│                         │ UDP                                   │
│                         ▼                                       │
│                ┌─────────────────┐                              │
│                │  micro-ROS      │                              │
│                │    Agent        │                              │
│                │  UDP:8888       │                              │
│                └────────┬────────┘                              │
│                         │ DDS                                   │
│                         ▼                                       │
│                ┌─────────────────┐                              │
│                │  SAINT.OS       │                              │
│                │    Server       │                              │
│                └─────────────────┘                              │
└─────────────────────────────────────────────────────────────────┘
```

## Build Modes

| Mode | Transport | stdio | Use Case |
|------|-----------|-------|----------|
| Hardware (default) | W5500 Ethernet (UDP) | USB | Real Adafruit Feather RP2040 |
| Simulation | UDP Bridge | UART0 | Renode simulation |

Build for simulation:
```bash
cmake -DSIMULATION=ON ..
make
```

Build for hardware:
```bash
cmake ..   # or cmake -DSIMULATION=OFF ..
make
```

## UDP Bridge Peripheral

The simulation uses a custom Renode peripheral (`UDPBridge`) that provides:

- Memory-mapped interface at `0x50300000`
- Direct UDP socket to host network
- TX/RX buffers (1280 bytes each)
- Configurable local port and remote address

**Register Map:**
| Offset | Name | Description |
|--------|------|-------------|
| 0x00 | Control | Write: send/open/close/recv bits; Read: status |
| 0x04 | TX Length | Bytes to transmit |
| 0x08 | RX Length | Bytes received (read-only) |
| 0x0C | Remote IP | Target IP address (little-endian) |
| 0x10 | Remote Port | Target UDP port |
| 0x14 | Local Port | Local UDP port |
| 0x100-0x5FF | TX Buffer | Transmit data buffer |
| 0x600-0xAFF | RX Buffer | Receive data buffer (read-only) |

## Useful Renode Commands

| Command | Description |
|---------|-------------|
| `start` | Start all machines |
| `pause` | Pause all machines |
| `quit` | Exit Renode |
| `mach set "node1"` | Select a specific machine |
| `sysbus.uart0 DumpHistoryBuffer` | Show debug output |
| `emulation RunFor "0:0:30"` | Run for 30 seconds |

## Limitations

1. **PIO Not Emulated** - PIO (used for NeoPixel LED on real hardware) is not functional in simulation.

2. **Timing** - Renode runs slower than real-time. Network timeouts may need adjustment.

3. **No DHCP** - Simulation uses static IP addresses.

## Troubleshooting

**Port Already in Use:**
If you see "Failed to open UDP socket", another process is using the port. Either:
- Stop the other process
- Change the local port in the firmware or simulation script

**micro-ROS Agent Not Connecting:**
1. Verify the agent is running:
   - Linux: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
   - macOS: `docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888`
2. Check the agent address in firmware matches (default: 127.0.0.1:8888 for simulation)
3. On macOS, ensure Docker is using port mapping (`-p 8888:8888/udp`)

**No ROS2 Topics:**
1. Wait for the node to initialize (watch the UART output)
2. Verify the agent shows a connected client
3. Check `ros2 topic list` and `ros2 node list`

## Files

| File | Description |
|------|-------------|
| `simulation/saint_node_manager.py` | Node lifecycle management script |
| `simulation/nodes/<node_id>.resc` | Generated Renode scripts per node |
| `simulation/node_storage/` | Persistent storage files per node |
| `simulation/renode_rp2040/` | Renode RP2040 package with custom peripherals |
| `simulation/renode_rp2040/emulation/peripherals/network/udp_bridge.cs` | UDP bridge peripheral |
| `simulation/renode_rp2040/emulation/peripherals/storage/persistent_storage.cs` | Persistent storage peripheral |
| `transport/transport_udp_bridge.c` | Firmware UDP transport (simulation) |
| `transport/transport_w5500.c` | Hardware ethernet transport |
