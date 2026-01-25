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

2. **Renode RP2040 Package** - Community RP2040 support for Renode
   ```bash
   cd ~
   git clone https://github.com/matgla/Renode_RP2040.git
   ```

3. **Firmware built for simulation**
   ```bash
   cd firmware/rp2040/build
   export PICO_SDK_PATH=~/pico-sdk
   export MICRO_ROS_PATH=~/micro_ros_raspberrypi_pico_sdk
   cmake -DSIMULATION=ON ..
   make
   ```

4. **micro-ROS Agent** (for ROS2 connectivity)
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

## Running a Single Node

**Terminal 1 - Start Renode:**
```bash
cd ~/Renode_RP2040
~/Applications/Renode.app/Contents/MacOS/Renode run_saint_node.resc
```

Then in the Renode console:
```
(saint_node) start
```

**Terminal 2 - Start micro-ROS Agent:**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 3 - Verify Connection:**
```bash
ros2 topic list
ros2 topic echo /saint/nodes/announce
```

## Running Multiple Nodes

For multi-node simulation, use the multi-node script:

```bash
cd ~/Renode_RP2040
~/Applications/Renode.app/Contents/MacOS/Renode run_multi_nodes.resc
```

This creates 3 simulated nodes, each with its own UDP port:
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
1. Verify the agent is running: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
2. Check the agent address in firmware matches (default: 192.168.1.10:8888)
3. If running on the same machine, use `127.0.0.1` as the agent IP

**No ROS2 Topics:**
1. Wait for the node to initialize (watch the UART output)
2. Verify the agent shows a connected client
3. Check `ros2 topic list` and `ros2 node list`

## Files

| File | Description |
|------|-------------|
| `~/Renode_RP2040/run_saint_node.resc` | Single node simulation |
| `~/Renode_RP2040/run_multi_nodes.resc` | Multi-node simulation |
| `~/Renode_RP2040/emulation/peripherals/network/udp_bridge.cs` | UDP bridge peripheral |
| `firmware/rp2040/transport/transport_udp_bridge.c` | Firmware UDP transport |
| `firmware/rp2040/transport/transport_w5500.c` | Hardware ethernet transport |
