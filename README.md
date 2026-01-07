# RUITONG ROS2 Tracking Package

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

ROS2 package for integrating RUITONG optical tracking system (RUITONG SE/MAX series) into robotics applications. This package provides real-time surgical tool tracking with 60Hz publishing frequency.

## Features

- ✨ Real-time tracking at 60Hz
- 🎯 6-DOF pose estimation for surgical tools
- 📊 Gravity vector and system status monitoring
- 🔧 Support for multiple tool configurations
- 🚀 Low-latency C++ implementation
- 📡 Standard ROS2 message interfaces

---

## Table of Contents

- [System Requirements](#system-requirements)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [ROS2 Interface](#ros2-interface)
- [Network Setup](#network-setup)
- [Troubleshooting](#troubleshooting)

---

## System Requirements

**Operating System:**
- Linux Ubuntu 22.04 or higher
- Kernel 5.15.0+

**Dependencies:**
- ROS2 Humble/Iron/Jazzy
- CMake 3.10+
- C++14 compiler (GCC 11.0+)
- RUITONG SDK (included in `lib/`)

**Hardware:**
- RUITONG SE or RUITONG MAX tracking system
- Ethernet connection (Link-Local network required)

---

## Package Structure

```
ruitong_demo/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS2 package manifest
├── README.md                   # This file
│
├── msg/                        # Custom message definitions
│   ├── ToolTrackingMsg.msg    # Single tool tracking data
│   └── TrackingDataMsg.msg    # Complete system tracking data
│
├── src/
│   └── ruitong_tracking_node.cpp  # Main ROS2 node (60Hz publisher)
│
├── include/                    # RUITONG SDK headers
│   ├── ARMDCombinedAPI.h
│   ├── Tracking.h
│   ├── UdpConnection.h
│   └── ... (other SDK headers)
│
├── lib/                        # RUITONG SDK libraries
│   └── libARMDsterotracking.so
│
├── tool/                       # Tool calibration files (*.arom)
│   └── Tool.arom              # Generated on first run
```

---

## Installation

### 1. Clone into ROS2 Workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ruitong_demo.git
cd ~/ros2_ws
```

### 2. Install Dependencies

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Install ROS2 dependencies (if any)
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select ruitong_demo
```

### 4. Source Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Quick Start

### 1. Network Setup

Configure your Ethernet adapter for Link-Local connection:

```bash
# Using NetworkManager CLI
sudo nmcli connection modify "Wired connection 1" ipv4.method link-local
sudo nmcli connection up "Wired connection 1"

# Verify Link-Local IP (should be 169.254.x.x)
ip addr show
```

### 2. Run the Node

```bash
# Default configuration (device hostname: RT-MAX000752.local)
ros2 run ruitong_demo ruitong_tracking_node

# With custom device IP
ros2 run ruitong_demo ruitong_tracking_node --ros-args \
  -p hostname:=169.254.1.100

# With custom tool path
ros2 run ruitong_demo ruitong_tracking_node --ros-args \
  -p hostname:=RT-MAX000752.local \
  -p tool_path:=/path/to/your/tools
```

### 3. Monitor Topics

```bash
# List available topics
ros2 topic list

# View tracking data
ros2 topic echo /tracking_data

# Check publishing frequency
ros2 topic hz /tracking_data
# Expected: average rate: 120.000
```

---

## Configuration

### Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hostname` | string | `RT-MAX000752.local` | Device hostname or IP address |
| `tool_path` | string | `./tool` | Directory containing .arom calibration files |

### Set Parameters at Runtime

```bash
ros2 run ruitong_demo ruitong_tracking_node --ros-args \
  -p hostname:=192.168.1.100 \
  -p tool_path:=/home/user/calibrations
```

---

## ROS2 Interface

### Published Topics

#### `/tracking_data` - Complete Tracking Information
- **Type:** `ruitong_demo/msg/TrackingDataMsg`
- **Frequency:** 60 Hz
- **QoS:** Reliable, queue size 100

**Message Structure:**

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id              # "ruitong_tracker"

float64[3] gravity_vector      # Gravity vector [x, y, z]
int32 markers_number           # Total detected markers
int32 stray_markers_number     # Unmatched markers

ToolTrackingMsg[] tools        # Array of tracked tools
  string tool_name             # Tool identifier
  string transformation_status # "Enabled", "Disabled", etc.
  bool match_status            # True if successfully matched
  float64[] marker_positions   # [x1,y1,z1, x2,y2,z2, ...] in mm
  string acquisition_time      # "YYYY-MM-DD HH:MM:SS:mmm"
  float64 matching_error       # RMS error in mm
  float64[16] transformation_matrix  # 4x4 matrix (row-major)
  float64[3] tip_position      # Tool tip [x, y, z] in mm
```

### Message Examples

```bash
# View message definition
ros2 interface show ruitong_demo/msg/TrackingDataMsg

# Record data to bag file
ros2 bag record /tracking_data

# Echo specific field
ros2 topic echo /tracking_data/tools[0]/tip_position
```

---

## Network Setup

### RUITONG Device Connection

The RUITONG tracking system requires **Link-Local Only** (169.254.x.x) network configuration.

#### GUI Method (Ubuntu)

1. Open **Settings** → **Network**
2. Select your wired connection
3. Go to **IPv4** tab
4. Set **IPv4 Method** to **Link-Local Only**
5. Click **Apply**
6. Reconnect the network

#### Command Line Method

```bash
# Configure Link-Local
sudo nmcli connection modify "Wired connection 1" ipv4.method link-local
sudo nmcli connection up "Wired connection 1"

# Verify connection
ping RT-MAX000752.local
```

#### Find Device IP

```bash
# Using avahi (mDNS)
avahi-resolve -n RT-MAX000752.local

# Scan local network
arp-scan --localnet | grep -i ruitong
```

---

## Troubleshooting

### Connection Issues

**Problem:** `getaddrinfo Error code Name or service not known`

**Solution:**
- Verify network adapter is configured for Link-Local (169.254.x.x)
- Check device is powered on and connected via Ethernet
- Try using IP address instead of hostname

**Problem:** `Connection failed`

**Solution:**
```bash
# Check physical connection
ip link show

# Test network reachability
ping 169.254.1.1  # Try common device IPs

# Check device LED indicators (refer to device manual)
```

### Build Errors

**Problem:** `cannot find -lARMDsterotracking`

**Solution:**
```bash
# Verify library exists
ls -l lib/libARMDsterotracking.so

# Check library path is correct
echo $LD_LIBRARY_PATH
```

### Runtime Issues

**Problem:** No data publishing

**Solution:**
- Check if tool is visible to cameras (all markers lit)
- Verify `.arom` files in `tool/` directory match your physical tool
- Monitor node output for warnings: `ros2 run ruitong_demo ruitong_tracking_node`

---

## Development

### Adding New Tools

1. Create `.arom` calibration file using RUITONG calibration software
2. Place file in `tool/` directory
3. Restart node - it will automatically load all `.arom` files

### Custom Message Processing

Example subscriber in Python:

```python
import rclpy
from rclpy.node import Node
from ruitong_demo.msg import TrackingDataMsg

class TrackingSubscriber(Node):
    def __init__(self):
        super().__init__('tracking_subscriber')
        self.subscription = self.create_subscription(
            TrackingDataMsg,
            'tracking_data',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        for tool in msg.tools:
            if tool.match_status:
                self.get_logger().info(
                    f'{tool.tool_name} tip: {tool.tip_position}')

rclpy.init()
node = TrackingSubscriber()
rclpy.spin(node)
```

---

## License

The RUITONG SDK libraries in `lib/` are proprietary to ARMD (Aorui Medical Technology, Beijing) and subject to their licensing terms.

---

## Contact & Support

- **Issues:** [GitHub Issues](https://github.com/yourusername/ruitong_demo/issues)
- **RUITONG Support:** Contact your ARMD representative
- **SDK Documentation:** See `瑞瞳SDK说明书Linux C++版.pdf`

---

## Citation

If you use this package in your research, please cite:

```bibtex
@software{ruitong_ros2,
  title = {RUITONG ROS2 Tracking Package},
  author = {Your Name},
  year = {2026},
  url = {https://github.com/yourusername/ruitong_demo}
}
```

---

## Acknowledgments

- ARMD (Aorui Medical Technology, Beijing) for the RUITONG tracking system and SDK
- ROS2 community for the excellent robotics middleware
