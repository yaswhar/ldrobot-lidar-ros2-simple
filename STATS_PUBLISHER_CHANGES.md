# LDLidar Statistics Publisher/Logger Changes

## Overview
The statistics system has been refactored to follow ROS2 best practices using a publisher/subscriber pattern.

## Architecture

### Publisher Node: `ldlidar_stats.py`
- **Purpose**: Computes statistics from laser scan data and publishes to topic
- **Subscribes to**: `/ldlidar_node/scan` (LaserScan)
- **Publishes to**: `/ldlidar_stats` (String)
- **Statistics computed**:
  - Minimum distance and its angle
  - Maximum distance and its angle
  - Average distance across valid points
- **Output format**: `Row - min: (dist, angle) - max: (dist, angle) - avg: dist`
- **Update rate**: Configurable via `update_rate` parameter (default: 1.0 Hz)

### Logger Node: `ldlidar_stats_logger.py`
- **Purpose**: Subscribes to statistics topic and logs to terminal + file
- **Subscribes to**: `/ldlidar_stats` (String)
- **Outputs**:
  1. Terminal (real-time logging via ROS logger)
  2. Text file (timestamped, persistent storage)
- **File format**: `ldlidar_stats_YYYYMMDD_HHMMSS.txt`
- **Log directory**: Configurable via `log_file_path` parameter (default: `~/shahrokhi`)
- **Terminal window**: Launches in separate xterm window for visibility

## Files Modified/Created

### New Files
- `ldlidar_node/scripts/ldlidar_stats_logger.py` - Subscriber node for file logging

### Modified Files
- `ldlidar_node/scripts/ldlidar_stats.py` - Refactored to publisher-only
- `ldlidar_node/CMakeLists.txt` - Added logger script to installation
- `ldlidar_node/launch/ldlidar_with_stats.launch.py` - Updated to launch both nodes
- `ldlidar_node/launch/ldlidar_with_stats_and_viz.launch.py` - Updated to launch both nodes

## Configuration

Parameters are defined in `ldlidar_node/params/ldlidar.yaml`:

```yaml
ldlidar_stats_publisher:
  ros__parameters:
    scan_topic: '/ldlidar_node/scan'
    update_rate: 1.0  # Statistics computation rate in Hz

ldlidar_stats_logger:
  ros__parameters:
    log_file_path: '~/shahrokhi'  # Directory for log files (tilde expanded)
```

## Usage

### Launch with Statistics (No Visualization)
```bash
ros2 launch ldlidar_node ldlidar_with_stats.launch.py
```

This launches:
- LDLidar node
- Statistics publisher (publishes to `/ldlidar_stats`)
- Statistics logger (logs to terminal + file in separate xterm window)

### Launch with Statistics and Visualization
```bash
ros2 launch ldlidar_node ldlidar_with_stats_and_viz.launch.py
```

This launches all of the above plus:
- Visualization node with click-to-annotate feature

### Manual Node Testing

**Test publisher only:**
```bash
ros2 run ldlidar_node ldlidar_stats.py
```

**View published statistics:**
```bash
ros2 topic echo /ldlidar_stats
```

**Test logger only (requires publisher running):**
```bash
ros2 run ldlidar_node ldlidar_stats_logger.py
```

## Build Instructions

After making changes, rebuild and install:

```bash
cd ~/ros2_ws
colcon build --packages-select ldlidar_node --symlink-install
source install/setup.bash
```

The `--symlink-install` flag allows Python script changes to take effect without rebuilding.

## Benefits of Publisher/Subscriber Pattern

1. **Separation of Concerns**: Publishing and logging are independent
2. **Flexibility**: Can have multiple subscribers to `/ldlidar_stats`
3. **Debugging**: Easy to inspect topic data with `ros2 topic echo`
4. **Modularity**: Logger can be started/stopped independently
5. **ROS2 Best Practice**: Standard pattern for data flow in ROS2

## Troubleshooting

**Logger terminal doesn't appear:**
- Ensure `xterm` is installed: `sudo apt install xterm`
- Alternatively, remove the `prefix='xterm -e'` line to log in main terminal

**Log file not created:**
- Check permissions on the configured directory
- Logger will fallback to `/tmp` if directory creation fails
- Check logger node output for warnings

**No statistics published:**
- Verify LDLidar node is publishing: `ros2 topic hz /ldlidar_node/scan`
- Check publisher node logs for errors
- Ensure scan data contains valid points

**Statistics seem incorrect:**
- Invalid points (NaN, inf, out of range) are automatically filtered
- Angles are in degrees
- Distances are in meters (as provided by LDLidar)
