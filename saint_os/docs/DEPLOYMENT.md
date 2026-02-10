# SAINT.OS Raspberry Pi Deployment Guide

This guide covers deploying SAINT.OS on a Raspberry Pi for production use.

## Hardware Requirements

- Raspberry Pi 4 (2GB+ RAM recommended)
- MicroSD card (16GB+)
- Ethernet connection (recommended) or WiFi
- Power supply (5V 3A)

## OS Setup

1. Install Ubuntu 22.04 Server (64-bit) for Raspberry Pi
2. Update the system:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

## ROS2 Installation

Install ROS2 Humble:

```bash
# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions

# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent
```

## SAINT.OS Installation

```bash
# Create workspace
mkdir -p ~/saint_ws/src
cd ~/saint_ws/src

# Clone or copy SAINT.OS source
# (copy from your development machine or clone from repository)

# Build
cd ~/saint_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source the workspace
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/saint_ws/install/setup.bash" >> ~/.bashrc
```

## Auto-Start on Boot

Create a systemd service to start SAINT.OS automatically:

```bash
sudo nano /etc/systemd/system/saint-os.service
```

Add the following content:

```ini
[Unit]
Description=SAINT.OS Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=ubuntu
Group=ubuntu
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/ubuntu/saint_ws/install/setup.bash && ros2 launch saint_os saint_server.launch.py'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable saint-os
sudo systemctl start saint-os
```

Check status:
```bash
sudo systemctl status saint-os
journalctl -u saint-os -f  # View logs
```

## Network Configuration

### Static IP (Recommended)

Edit `/etc/netplan/50-cloud-init.yaml`:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.1.10/24
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
```

Apply:
```bash
sudo netplan apply
```

### Firewall Rules

If using UFW:

```bash
sudo ufw allow 80/tcp    # Web interface
sudo ufw allow 8888/udp  # micro-ROS agent
sudo ufw allow 9090/tcp  # WebSocket (if separate)
```

## Configuration

### Server Name

Edit the launch or create a config file:

```bash
# Option 1: Launch argument
ros2 launch saint_os saint_server.launch.py server_name:=ROBOT-01

# Option 2: Create a launch wrapper
nano ~/saint_ws/start_saint.sh
```

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/saint_ws/install/setup.bash
ros2 launch saint_os saint_server.launch.py \
    server_name:=ROBOT-01 \
    web_port:=80 \
    agent_port:=8888
```

Update the systemd service to use this script.

## Monitoring

### View Logs

```bash
# Systemd logs
journalctl -u saint-os -f

# ROS2 logs
ros2 topic echo /rosout
```

### Check Node Status

```bash
ros2 node list
ros2 topic list
ros2 topic echo /saint/nodes/announce
```

### Web Interface

Access from any browser on the network:
```
http://<raspberry-pi-ip>/
```

## Updating

```bash
# Stop the service
sudo systemctl stop saint-os

# Update source code
cd ~/saint_ws/src/saint_os
git pull  # or copy new files

# Rebuild
cd ~/saint_ws
colcon build --symlink-install

# Restart
sudo systemctl start saint-os
```

## Troubleshooting

### Service won't start

Check logs:
```bash
journalctl -u saint-os -n 50
```

Common issues:
- Missing ROS2 setup: Ensure paths in service file are correct
- Port 80 in use: Check with `sudo lsof -i :80`
- Permission denied: Ensure user has access to required ports

### Nodes not connecting

1. Check agent is running: `ps aux | grep micro_ros`
2. Check firewall: `sudo ufw status`
3. Verify network connectivity between Pi and nodes
4. Check agent port: nodes must use same port (default 8888)

### High CPU usage

The micro-ROS agent can use significant CPU. If needed:
- Reduce agent verbosity (remove `-v4` flag)
- Increase agent polling interval

## Security Considerations

For production deployments:

1. **Change the default password** — The web interface password defaults to `12345`. Change it in `saint_server/config/server_config.yaml`:
   ```yaml
   websocket:
     password: 'your-secure-password'
   ```
   Restart the server after editing. The password can also be changed at runtime from the web UI settings.
2. **Use HTTPS** for web interface (requires reverse proxy like nginx)
3. **Restrict network access** with firewall rules
4. **Keep system updated** with security patches
5. **Disable unused services**
