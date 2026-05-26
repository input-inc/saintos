# SAINT.OS Raspberry Pi 5 Node Firmware

ROS2 node firmware for Raspberry Pi 5, providing GPIO control and sensor capabilities for the SAINT robotics platform.

## Overview

Unlike the RP2040 microcontroller firmware which uses micro-ROS, this firmware runs as a standard ROS2 node on Linux. It communicates directly with the SAINT.OS server using DDS (no micro-ROS agent required).

## Features

- **GPIO Control**: Digital I/O, PWM, Servo control via libgpiod
- **Same Protocol**: Uses identical message protocol as RP2040 nodes
- **Persistent Config**: Configuration stored in `/etc/saint-node/`
- **systemd Service**: Runs as a system service with auto-restart
- **Hot-pluggable**: Supports dynamic pin configuration from server

## Requirements

- Raspberry Pi 5 (also works on Pi 4 with modifications)
- Raspberry Pi OS (Bookworm or later)
- ROS2 Jazzy or Humble
- Python 3.11+
- libgpiod

## Installation

### 1. Install ROS2

Follow the official ROS2 installation guide for Raspberry Pi:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 2. Install SAINT Node

```bash
cd firmware/rpi5
sudo ./scripts/install.sh
```

### 3. Start the Service

```bash
# Start now
sudo systemctl start saint-node

# Enable auto-start on boot
sudo systemctl enable saint-node

# View logs
journalctl -u saint-node -f
```

## Configuration

Configuration is stored in `/etc/saint-node/config.yaml`:

```yaml
node_id: rpi5_abc123def456
role: head
display_name: "Head Controller"
adopted: true
pins:
  "12":
    mode: pwm
    logical_name: pan_servo
    frequency: 50
  "13":
    mode: pwm
    logical_name: tilt_servo
    frequency: 50
  "17":
    mode: digital_out
    logical_name: led_status
network:
  agent_host: 192.168.1.1
  agent_port: 8888
```

## Available GPIO Pins

| GPIO | Name | Capabilities |
|------|------|--------------|
| 2, 3 | I2C1 | digital_io, i2c |
| 4 | GPCLK0 | digital_io |
| 5, 6 | GPIO | digital_io, pwm_soft |
| 7, 8 | SPI CE | digital_io, spi |
| 9, 10, 11 | SPI | digital_io, spi |
| 12, 13 | PWM | digital_io, pwm, servo |
| 16, 17 | GPIO | digital_io, pwm_soft |
| 18, 19 | PWM | digital_io, pwm, servo |
| 20-27 | GPIO | digital_io, pwm_soft |

**Reserved Pins** (not available):
- GPIO 0, 1: I2C0 (ID EEPROM)
- GPIO 14, 15: UART0 (console)

## ROS2 Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/saint/nodes/announce` | std_msgs/String | Node announcements (1 Hz) |
| `/saint/nodes/<id>/capabilities` | std_msgs/String | Hardware capabilities |
| `/saint/nodes/<id>/state` | std_msgs/String | Pin states (10 Hz) |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/saint/nodes/<id>/config` | std_msgs/String | Configuration commands |
| `/saint/nodes/<id>/control` | std_msgs/String | Runtime control |

## Message Protocol

All messages use JSON payloads in std_msgs/String, identical to RP2040 nodes.

### Announcement
```json
{
  "node_id": "rpi5_abc123def456",
  "mac": "dc:a6:32:xx:xx:xx",
  "ip": "192.168.1.100",
  "hw": "Raspberry Pi 5",
  "fw": "1.0.0",
  "state": "ACTIVE",
  "uptime": 3600,
  "role": "head"
}
```

### Configuration
```json
{
  "action": "configure",
  "pins": {
    "12": {"mode": "servo", "logical_name": "pan"},
    "13": {"mode": "servo", "logical_name": "tilt"}
  }
}
```

### Control
```json
{
  "action": "set_pin",
  "gpio": 12,
  "value": 90.0
}
```

## Development

### Running Manually

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Run node
python3 -m saint_node.node
```

### Testing Without Hardware

The GPIO controller includes a mock mode when `gpiod` is not available, allowing development on non-Pi systems.

## Troubleshooting

### Service won't start

Check logs:
```bash
journalctl -u saint-node -n 50
```

Common issues:
- ROS2 not installed or not found
- GPIO permissions (ensure running as root or in gpio group)
- Python dependencies missing

### GPIO access denied

Ensure the service runs as root or add to gpio group:
```bash
sudo usermod -aG gpio $USER
```

### Node not discovered by server

- Check network connectivity
- Verify ROS_DOMAIN_ID matches server
- Check firewall rules for DDS ports (7400-7500 UDP)

## License

Part of the SAINT.OS project.
