# LD Lidar ROS 2 Package

## ROS 2 package for LDRobot lidar - Based on Nav2 Lifecycle nodes

[Get the Lidar](#get-the-lidar) • [Features](#features) • [Install](#installation) • [Quick Start](#quick-start) • [Visualization](#visualization) • [USB Auto-Recovery](#usb-auto-recovery) • [Parameters](#parameters) • [Troubleshooting](#troubleshooting) • [Deployment](#raspberry-pi-deployment)

This package is designed to work with the DToF 2D Lidar sensors [LD19](https://www.ldrobot.com/product/en/112), [LD06](https://www.ldrobot.com/product/en/98), and STL-27L made by [LDRobot](https://www.ldrobot.com/en).

LD19             |  LD06
:-------------------------:|:-------------------------:
![ld19](https://user-images.githubusercontent.com/3648617/204473718-803d25d9-605a-4eaa-a047-d5d3524eead8.png)  |  ![ld06](https://user-images.githubusercontent.com/3648617/204473720-97f72c31-188e-4f5c-b98b-1033a5afe91e.png)

## Features

✨ **New in this version:**

- 🔌 **USB Auto-Recovery**: Automatically reconnects when USB is unplugged and replugged (30-second retry with data verification)
- 📊 **Lightweight Visualization**: Interactive matplotlib-based polar plot (replaces RViz2 for Raspberry Pi)
- 🔄 **Lifecycle Management**: Nav2-compatible lifecycle node with robust error handling
- 🛡️ **Production Ready**: Thread-safe reconnection, deadlock fixes, and comprehensive diagnostics
- 🎯 **Angle Cropping**: Software-based field of view limiting (configurable min/max angles)

## Get the lidar

My lidar (LD19) comes from the [LDRobot kickstarter campaing](https://www.kickstarter.com/projects/ldrobot/ld-air-lidar-360-tof-sensor-for-all-robotic-applications) ended in 2021.

LDRobot then created also an [Indiegogo campaign](https://www.indiegogo.com/projects/ld-air-lidar-tof-sensor-for-robotic-applications--3#/) for the LD19.

LDRobot today distributes the Lidar through third-party resellers:

- Waveshare: [LD19](https://www.waveshare.com/wiki/DTOF_LIDAR_LD19)
- Innomaker: [LD06](https://www.inno-maker.com/product/lidar-ld06/)
- Other: [Search on Google](https://www.google.com/search?q=ld19+lidar&newwindow=1&sxsrf=ALiCzsb2xd4qTTA78N00mP9-PP5HY4axZw:1669710673586&source=lnms&tbm=shop&sa=X&ved=2ahUKEwjYns78_NL7AhVLVfEDHf2PDk8Q_AUoA3oECAIQBQ&cshid=1669710734415350&biw=1862&bih=882&dpr=1)

## The node in action

LD19 Lifecycle            |  LD19 outdoor
:-------------------------:|:-------------------------:
[![LD19 Lifecycle](https://img.youtube.com/vi/mbKwmK3Yjus/mqdefault.jpg)](https://youtu.be/mbKwmK3Yjus) | [![LD19 outdoor](https://img.youtube.com/vi/zyggXjW6cDo/mqdefault.jpg)](https://youtu.be/zyggXjW6cDo)

## Installation

### Requirements

- **ROS 2 Humble** or **ROS 2 Jazzy** (Rolling not yet supported)
- **Platform**: Ubuntu 22.04 (x86_64 or ARM64 for Raspberry Pi 4)
- **Hardware**: LD06, LD19, or STL-27L lidar

### Install Steps

1. **Clone the repository:**
```bash
cd ~/ros2_ws/src/
git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git
```

2. **Install dependencies:**
```bash
# System dependencies
sudo apt install libudev-dev

# Python dependencies for visualization (optional)
sudo apt install python3-matplotlib python3-numpy python3-tk
```

3. **Set up udev rules:**
```bash
cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
```

4. **Build the packages:**
```bash
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ldlidar_component ldlidar_node --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

5. **Source the workspace:**
```bash
source ~/ros2_ws/install/setup.bash
# Or add to ~/.bashrc for persistence:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Quick Start

### Launch with Visualization (Recommended for Development)

```bash
ros2 launch ldlidar_node ldlidar_with_viz.launch.py
```

This launches:
- Lifecycle manager (automatic node activation)
- Robot state publisher (TF transforms)
- LDLidar component
- **Interactive visualization** (matplotlib polar plot)

### Launch Without Visualization (Headless/Production)

```bash
ros2 launch ldlidar_node ldlidar_with_mgr.launch.py
```

### Verify It's Working

```bash
# Check if scan data is publishing
ros2 topic hz /ldlidar_node/scan
# Expected: ~10-13 Hz

# View scan data
ros2 topic echo /ldlidar_node/scan

# Check lifecycle state
ros2 lifecycle get /ldlidar_node
# Expected: active [3]
```

## Visualization

### Lightweight Interactive Visualization

Designed as a **RViz2 replacement** for resource-constrained systems like Raspberry Pi 4:

**Features:**
- 🎨 Real-time polar plot with color-coded range (red=close, blue=far)
- 🖱️ **Pan**: Left-click and drag
- 🔍 **Zoom**: Scroll wheel
- 📏 Auto-adjusts to lidar parameters (range + angle limits)
- ⚡ Lightweight: Uses matplotlib (CPU-only, ~150MB RAM vs RViz2's 500MB+)
- 🔌 Works over SSH with X11 forwarding

**Launch with Visualization:**
```bash
ros2 launch ldlidar_node ldlidar_with_viz.launch.py
```

**Configure Visualization Parameters:**
```bash
ros2 run ldlidar_node ldlidar_visualizer.py --ros-args \
  -p scan_topic:=/ldlidar_node/scan \
  -p update_rate:=10.0 \
  -p point_size:=5.0 \
  -p colormap:=jet_r
```

**For Remote/SSH Use:**
```bash
ssh -X user@raspberry-pi
ros2 launch ldlidar_node ldlidar_with_viz.launch.py
```

## USB Auto-Recovery

### Automatic Reconnection Feature

The lidar node automatically recovers from USB disconnections **without manual intervention**:

**How It Works:**
1. 🔴 **Detection**: Counts consecutive timeouts (5 timeouts = ~5 seconds)
2. 🔄 **Reconnection**: Retries every second for up to 30 seconds
3. ⏳ **Stabilization**: Waits 3 seconds for motor spin-up
4. ✅ **Verification**: Confirms actual laser scan data before declaring success
5. 📊 **Recovery**: Resumes normal operation automatically

**What You'll See:**
```
[ERROR] get ldlidar data is time out... (Timeout 1/5)
[ERROR] get ldlidar data is time out... (Timeout 2/5)
...
[WARN] Multiple timeouts detected. Attempting to reconnect...
[INFO] Will retry every second for up to 30 seconds. Please reconnect the USB cable.
[INFO] Reconnection attempt 3/30...
[INFO] Communication established on attempt 3. Waiting for lidar to stabilize...
[INFO] Verifying data availability...
[INFO] Successfully reconnected on attempt 3! Lidar is operational.
```

**User Action Required:**
- Simply plug the USB cable back in within 30 seconds
- No need to restart the node or application
- If 30 seconds elapse, the node deactivates (restart required)

**Key Parameters:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| Timeout threshold | 5 | Number of consecutive timeouts before reconnection |
| Max attempts | 30 | Total reconnection attempts (30 seconds) |
| Retry interval | 1s | Time between reconnection attempts |
| Stabilization delay | 3s | Wait for lidar motor to spin up |
| Data verification | 3 attempts | Confirms valid scan data before success |

## Ground Scanning Topological Mapper

### Overview

The **Ground Mapper** generates a 2D topological map by stacking LiDAR scans based on drone velocity, simulating a ground scanner that creates a continuous map as the drone moves forward.

**How It Works:**
- Receives LaserScan messages and converts from polar to Cartesian coordinates
- Stacks scans along the Y-axis based on configurable drone velocity
- Displays a real-time scrolling 2D map showing terrain/obstacles

**Quick Start:**
```bash
# Launch with default velocity (1.0 m/s)
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py

# Launch with custom velocity
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.5
```

**Key Parameters** (configured in `ldlidar.yaml`):
- `drone_velocity_y`: Forward velocity in m/s (default: 1.0)
- `map_buffer_time`: Seconds of scan data to keep (default: 10.0)
- `map_resolution`: Grid resolution in meters/pixel (default: 0.05)
- `update_rate`: Visualization refresh rate in Hz (default: 10.0)
- `colormap`: Distance colormap (default: 'viridis')

**Interactive Controls:**
- **Scroll wheel**: Zoom in/out
- **Right-click + drag**: Pan the view
- **Automatic scrolling**: Map scrolls as drone moves forward

**Testing Procedure:**
```bash
# 1. Static test (velocity=0) - Verify coordinate conversion
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.0

# 2. Slow movement (velocity=0.5) - Verify stacking logic
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.5

# 3. Normal operation (velocity=1.0) - Real-world usage
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=1.0
```

**Algorithm:**
- Y-position increment: `Δy = velocity_y × Δt`
- Point conversion: `x = range × cos(angle)`, `y = y_offset + range × sin(angle)`
- Grid-based mapping with automatic cleanup of old scans

**Performance:**
- CPU: ~3-5% on Raspberry Pi 4
- Memory: ~50-200 MB (depends on buffer_time)
- Grid limit: Auto-scaled to max 2000×2000 pixels

For complete documentation, see [GROUND_MAPPER_IMPLEMENTATION.md](GROUND_MAPPER_IMPLEMENTATION.md) and [GROUND_MAPPER_CHECKLIST.md](GROUND_MAPPER_CHECKLIST.md).

## Launch Files

### Available Launch Files

| Launch File | Description | Use Case |
|-------------|-------------|----------|
| `ldlidar_bringup.launch.py` | Basic launch (no lifecycle manager) | Manual lifecycle control |
| `ldlidar_simple.launch.py` | Lifecycle manager only | Headless deployment |
| `ldlidar_with_mgr.launch.py` | Lifecycle manager + robot state publisher | Production deployment |
| `ldlidar_with_viz.launch.py` | Everything + visualization | Development/debugging |
| `ldlidar_ground_mapper.launch.py` | Everything + ground scanning mapper | Ground mapping/topological mapping |
| `ldlidar_rviz2.launch.py` | With RViz2 (desktop only) | Full visualization |
| `ldlidar_slam.launch.py` | With SLAM Toolbox | Mapping applications |

## Configuration

### Edit Parameters

The default values can be modified in [`ldlidar.yaml`](ldlidar_node/params/ldlidar.yaml):

Open a terminal console and enter the following command to start the node with customized parameters:

    ros2 launch ldlidar_node ldlidar_bringup.launch.py

The [`ldlidar_bringup.launch.py`](ldlidar_node/launch/ldlidar_bringup.launch.py) starts a ROS 2 Container, which loads the LDLidar Component as a plugin.

The [`ldlidar_bringup.launch.py`](ldlidar_node/launch/ldlidar_bringup.launch.py) script also starts a `robot_state_publisher` node that provides the static TF transform of the 
LDLidar [`ldlidar_base`->`ldlidar_link`], and provides the ldlidar description in the `/robot_description`.

![TF](./images/ldlidar_tf.png)

### Lifecycle

The `ldlidar` node is based on the [`ROS2 lifecycle` architecture](https://design.ros2.org/articles/node_lifecycle.html), hence it starts in the `UNCONFIGURED` state.
To configure the node, load all the parameters, establish a connection, and activate the scan publisher, the lifecycle services must be called in sequence.

Open a new terminal console and enter the following command:

    ros2 lifecycle set /ldlidar_node configure

If the node is correctly configured and the connection is established, `Transitioning successful` is returned. If there are errors, `Transitioning failed` is returned. Check the node log for details on any connection issues.

The node is now in the `INACTIVE` state, enter the following command to activate:

    ros2 lifecycle set /ldlidar_node activate

The node is now activated and the `/ldlidar_node/scan` topic of type `sensor_msgs/msg/LaserScan` is available to be subscribed.

#### Launch file with YAML parameters and Lifecycle manager

Thanks to the [Nav2](https://navigation.ros.org/index.html) project, you can launch a [`lifecycle_manager`](https://navigation.ros.org/configuration/packages/configuring-lifecycle.html) node that handles the state transitions described above.
An example launch file, [`ldlidar_with_mgr.launch.py`](ldlidar_node/launch/ldlidar_with_mgr.launch.py), demonstrates how to start the `ldlidar_node` with parameters loaded from the 
`ldlidar.yaml` file. It also starts the `lifecycle_manager`, configured with the [`lifecycle_mgr.yaml`](ldlidar_node/config/lifecycle_mgr.yaml) file, to automatically manage the 
lifecycle transitions:

    ros2 launch ldlidar_node ldlidar_with_mgr.launch.py

The `ldlidar_with_mgr.launch.py` script automatically starts the `ldlidar_node` by including the `ldlidar_bringup.launch.py` launch file.

## Parameters

Following the list of node parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `general.debug_mode` | bool | false | Enable debug messages |
| `comm.serial_port` | string | `/dev/ldlidar` | Serial port path |
| `comm.baudrate` | int | 230400 | Serial baudrate (LD19: 230400, STL27L: 921600) |
| `comm.timeout_msec` | int | 1000 | Communication timeout (ms) |
| **Lidar Configuration** | | | |
| `lidar.model` | string | `LDLiDAR_LD19` | Model: LD06, LD19, or STL27L |
| `lidar.rot_verse` | string | `CCW` | Rotation direction: CW (upside-down) or CCW |
| `lidar.units` | string | `M` | Distance units: M, CM, or MM |
| `lidar.frame_id` | string | `ldlidar_link` | TF frame name |
| `lidar.bins` | int | 455 | Fixed bin count (0=dynamic, 455=SLAM compatible) |
| `lidar.range_min` | float | 0.02 | Minimum valid range (m) |
| `lidar.range_max` | float | 12.0 | Maximum valid range (m) |
| **Angle Cropping** | | | |
| `lidar.enable_angle_crop` | bool | false | Enable field of view limiting |
| `lidar.angle_crop_min` | float | 0.0 | Minimum angle (degrees) |
| `lidar.angle_crop_max` | float | 360.0 | Maximum angle (degrees) |

**Example Configuration for 180° Front Sector:**
```yaml
lidar:
  enable_angle_crop: true
  angle_crop_min: 270.0  # -90° (left)
  angle_crop_max: 90.0   # +90° (right)
  # Result: 180° front arc only
```

## Troubleshooting

### USB Connection Issues

#### "Permission denied" on serial port
```bash
sudo usermod -aG dialout $USER
sudo reboot
```

#### Can't find `/dev/ldlidar` or `/dev/ttyUSB0`
```bash
# Check USB devices
ls -l /dev/ttyUSB*
dmesg | grep tty

# Verify udev rules
ls -l /dev/ldlidar

# Reinstall udev rules if needed
cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
sudo reboot
```

#### Auto-recovery not working
1. **Wait 5 seconds**: Reconnection starts after 5 consecutive timeouts
2. **Check USB connection**: `ls -l /dev/ttyUSB*` should show device
3. **Verify power supply**: Raspberry Pi needs 5V, 3A for stable operation
4. **Try different USB port**: USB 3.0 (blue) ports preferred
5. **Check cable quality**: Poor cables cause intermittent issues

### Visualization Issues

#### "No display found"
```bash
# For SSH, use X11 forwarding
ssh -X user@raspberry-pi

# Or set DISPLAY manually
export DISPLAY=:0
```

#### ImportError: No module named 'tkinter'
```bash
sudo apt install python3-tk
```

#### Slow/laggy visualization
```bash
# Reduce update rate
ros2 run ldlidar_node ldlidar_visualizer.py --ros-args -p update_rate:=5.0

# Or reduce point size
ros2 run ldlidar_node ldlidar_visualizer.py --ros-args -p point_size:=3.0
```

### Data Issues

#### No scan data published
```bash
# Check node status
ros2 lifecycle get /ldlidar_node

# Manually activate if needed
ros2 lifecycle set /ldlidar_node configure
ros2 lifecycle set /ldlidar_node activate

# Check for errors
ros2 topic echo /diagnostics
```

#### Incorrect angle mapping
The angle cropping logic was fixed in the latest version. If you still see inverted angles:
```bash
# Rebuild with latest code
cd ~/ros2_ws
colcon build --packages-select ldlidar_component --cmake-clean-cache
source install/setup.bash
```

### Performance Issues

#### High CPU usage on Raspberry Pi
```bash
# Check current usage
htop

# Reduce visualization update rate
ros2 run ldlidar_node ldlidar_visualizer.py --ros-args -p update_rate:=5.0

# Or run without visualization
ros2 launch ldlidar_node ldlidar_with_mgr.launch.py
```

## Raspberry Pi Deployment

### System Requirements
- **Hardware**: Raspberry Pi 4 (2GB minimum, 4GB+ recommended)
- **OS**: Raspberry Pi OS 64-bit or Ubuntu 22.04 ARM64
- **ROS**: ROS2 Humble
- **Power**: 5V, 3A power supply (official adapter recommended)

### Installation on Raspberry Pi

Follow the same [installation steps](#installation) above. For headless deployment:

```bash
# Install dependencies (skip visualization if not needed)
sudo apt install libudev-dev

# Build without visualization dependencies
cd ~/ros2_ws
colcon build --packages-select ldlidar_component ldlidar_node

# Launch headless
ros2 launch ldlidar_node ldlidar_with_mgr.launch.py
```

### Auto-Start on Boot

Create systemd service:
```bash
sudo nano /etc/systemd/system/ldlidar.service
```

Content:
```ini
[Unit]
Description=LDLidar ROS2 Node
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/pi/ros2_ws/install/setup.bash && ros2 launch ldlidar_node ldlidar_with_mgr.launch.py"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable ldlidar.service
sudo systemctl start ldlidar.service

# Check status
sudo systemctl status ldlidar.service
```

## Display scan on RViz2

The launch file `ldlidar_rviz2.launch.py` starts the `ldlidar_node` node, the `lifecycle_manager` node, and a preconfigured instance of RViz2 to display the 2D laser scan provided by the LDRobot sensors. This is an example to demonstrate how to correctly setup RViz2 to be used with the `ldlidar_node` node.

Open a terminal console and enter the following command:

    ros2 launch ldlidar_node ldlidar_rviz2.launch.py

![Rviz2](./images/ldlidar_rviz2.png)

## Robot Integration

Follow these steps to integrate the LDLidar sensor into your robot configuration:

1. **Provide TF Transform**: Ensure there is a TF transform from `base_link` to `ldlidar_base`, positioned at the center of the lidar scanner base. The `ldlidar_base` -> `ldlidar_link` transform is provided by the `robot_state_publisher` started by the `ldlidar_bringup.launch.py` launch file.

2. **Modify Configuration**: Update the [`ldlidar.yaml`](ldlidar_node/config/ldlidar.yaml) file to match your robot's configuration.

3. **Include Launch File**: Add the [`ldlidar_bringup.launch.py`](ldlidar_node/launch/ldlidar_bringup.launch.py) to your robot's bringup launch file. Refer to the [example provided](#launch-file-with-yaml-parameters-and-lifecycle-manager).

4. **Handle Lifecycle**: Properly manage the node's lifecycle. You can use the Nav2 `lifecycle_manager` by including it in your bringup launch file. Follow the [example provided](#launch-file-with-yaml-parameters-and-lifecycle-manager).

5. **Deploy and Test**: Deploy your configuration and test the system to ensure everything is working correctly.

Enjoy your fully integrated lidar system!

## SLAM Toolbox example

The launch file `ldlidar_slam.launch.py` shows how to use the node with the [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) package to generate a 2D map for robot navigation.

![Slam](./images/ld19_slam.png)

## Recent Improvements

### Version Highlights

**🔌 USB Auto-Recovery System**
- Automatic reconnection after USB disconnection
- 30-second retry window with 1-second intervals
- 3-second stabilization delay for motor spin-up
- Data verification before declaring success
- Thread-safe implementation with deadlock prevention

**📊 Lightweight Visualization**
- Matplotlib-based polar plot (replaces RViz2)
- Interactive pan and zoom controls
- Color-coded range display
- Auto-adjusts to lidar parameters
- Optimized for Raspberry Pi 4 (~150MB RAM vs RViz2's 500MB+)
- Works over SSH with X11 forwarding

**🛠️ Bug Fixes**
- Fixed inverted angle cropping logic
- Fixed LaserScan angle_min/angle_max not reflecting crop settings
- Fixed thread deadlock on deactivation
- Fixed buffer overflow during shutdown
- Enhanced reconnection stability

**🎯 Enhanced Angle Cropping**
- Software-based field of view limiting
- Configurable min/max angles in degrees
- Visualization automatically adapts to cropped range
- Grid lines only drawn within active sensor range

## Benchmarking

It is possible to benchmark the node to evaluate the overall performance by using the [NVIDIA® ISAAC ROS ros2_benchmark package](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark).

First of all install the [ros2_benchmark package](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark/tree/main?tab=readme-ov-file#quickstart).

Launch the benchmark:

    cd ~/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_node/test/
    launch_test ldlidar_benchmark.py

the final result should be similar to

    +--------------------------------------------------------------------------------------------+
    |                                  LD Lidar Live Benchmark                                   |
    |                                        Final Report                                        |
    +--------------------------------------------------------------------------------------------+
    | [Scan] Delta between First & Last Received Frames (ms) : 4900.138                          |
    | [Scan] Mean Playback Frame Rate (fps) : 9.936                                              |
    | [Scan] Mean Frame Rate (fps) : 9.936                                                       |
    | [Scan] # of Missed Frames : 0.000                                                          |
    | [Scan] # of Frames Sent : 49.000                                                           |
    | [Scan] First Sent to First Received Latency (ms) : 0.075                                   |
    | [Scan] Last Sent to Last Received Latency (ms) : 0.113                                     |
    | [Scan] First Frame End-to-end Latency (ms) : 0.075                                         |
    | [Scan] Last Frame End-to-end Latency (ms) : 0.113                                          |
    | [Scan] Max. End-to-End Latency (ms) : 0.172                                                |
    | [Scan] Min. End-to-End Latency (ms) : 0.049                                                |
    | [Scan] Mean End-to-End Latency (ms) : 0.098                                                |
    | [Scan] Max. Frame-to-Frame Jitter (ms) : 100.142                                           |
    | [Scan] Min. Frame-to-Frame Jitter (ms) : 0.000                                             |
    | [Scan] Mean Frame-to-Frame Jitter (ms) : 17.865                                            |
    | [Scan] Frame-to-Frame Jitter Std. Deviation (ms) : 12.793                                  |
    +--------------------------------------------------------------------------------------------+
    | Baseline Overall CPU Utilization (%) : 0.000                                               |
    | Max. Overall CPU Utilization (%) : 79.167                                                  |
    | Min. Overall CPU Utilization (%) : 0.000                                                   |
    | Mean Overall CPU Utilization (%) : 1.179                                                   |
    | Std Dev Overall CPU Utilization (%) : 3.964                                                |
    +--------------------------------------------------------------------------------------------+
    | [metadata] Test Name : LD Lidar Live Benchmark                                             |
    | [metadata] Test File Path : /home/walter/devel/ros2/ros2_walt/src/ldrobot-lidar-ros2/ldlidar_node/test/ldlidar_benchmark.py |
    | [metadata] Test Datetime : 2024-11-25T22:12:54Z                                            |
    | [metadata] Device Hostname : walter-Legion-5-15ACH6H                                       |
    | [metadata] Device Architecture : x86_64                                                    |
    | [metadata] Device OS : Linux 6.8.0-40-generic #40~22.04.3-Ubuntu SMP PREEMPT_DYNAMIC Tue Jul 30 17:30:19 UTC 2 |
    | [metadata] Idle System CPU Util. (%) : 0.333                                               |
    | [metadata] Benchmark Mode : 3                                                              |
    +--------------------------------------------------------------------------------------------+

---

## Additional Resources

### Documentation
- [CHANGELOG.md](CHANGELOG.md) - Version history and release notes
- [CONTRIBUTING.md](CONTRIBUTING.md) - Contribution guidelines
- [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) - Community guidelines

### External Links
- [LDRobot Official Website](https://www.ldrobot.com/en)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [GitHub Repository](https://github.com/Myzhar/ldrobot-lidar-ros2)

### Support
- **Issues**: [GitHub Issues](https://github.com/Myzhar/ldrobot-lidar-ros2/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Myzhar/ldrobot-lidar-ros2/discussions)

## License

Apache License 2.0 - See [LICENSE](LICENSE) file for details

## Credits

- **Original Package**: [Myzhar](https://github.com/Myzhar)
- **USB Auto-Recovery**: Developed for Raspberry Pi 4 deployment
- **Visualization Node**: Lightweight alternative to RViz2
- **Contributors**: See [GitHub Contributors](https://github.com/Myzhar/ldrobot-lidar-ros2/graphs/contributors)

---

**Last Updated**: October 17, 2025  
**Compatible with**: ROS2 Humble, Jazzy | Raspberry Pi 4, x86_64 Desktop
