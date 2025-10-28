# LDLidar Enhancement Documentation

## Overview

This document describes the two new features added to the LDLidar ROS2 package for Raspberry Pi 4:

1. **Hover Tooltips in Visualizer** - Interactive distance and angle display when hovering over lidar points
2. **Statistics Analyzer Node** - Real-time computation and logging of min/max/average distance statistics

---

## Feature 1: Hover Tooltips in Visualizer

### Description
The visualization window now displays distance and angle information when you hover your mouse over any lidar point.

### Changes Made
- **File Modified**: `ldlidar_node/scripts/ldlidar_visualizer.py`
- Added hover tooltip annotation that shows `(distance, angle)` format
- Tooltip appears within 30cm threshold of cursor position
- Yellow tooltip box with arrow pointing to the exact point

### Usage
1. Launch the visualizer:
   ```bash
   ros2 launch ldlidar_node ldlidar_with_viz.launch.py
   ```

2. Move your mouse over points in the visualization window
3. Tooltip will display: `(X.XXm, XXX.X°)` format
   - Example: `(2.45m, 135.2°)`

### Implementation Details
- Tooltip threshold: 0.3m (30cm) in data coordinates
- Updates in real-time as mouse moves
- Does not interfere with pan/zoom controls
- Automatically hides when no point is nearby

---

## Feature 2: Statistics Analyzer Node

### Description
A new ROS2 node that continuously analyzes lidar scan data and computes:
- Minimum distance and its angle
- Maximum distance and its angle
- Average distance across all valid points

The statistics are:
- Displayed in the terminal
- Written to a timestamped log file
- Computed at an adjustable rate (default: 1 Hz / every second)

### Files Created/Modified

#### New Files:
1. **`ldlidar_node/scripts/ldlidar_stats.py`** - The statistics analyzer node
2. **`ldlidar_node/launch/ldlidar_with_stats.launch.py`** - Launch file for lidar + stats
3. **`ldlidar_node/launch/ldlidar_with_stats_and_viz.launch.py`** - Launch file for lidar + stats + visualization

#### Modified Files:
1. **`ldlidar_node/params/ldlidar.yaml`** - Added stats analyzer parameters
2. **`ldlidar_node/CMakeLists.txt`** - Added installation of stats script

### Parameters

Added to `ldlidar.yaml`:

```yaml
ldlidar_stats_analyzer:
  ros__parameters:
    scan_topic: '/ldlidar_node/scan'      # Topic to subscribe for laser scan data
    update_rate: 1.0                       # Statistics computation rate in Hz
    log_file_path: '~/Desktop'            # Directory to save log files
```

### Output Format

**Terminal Output:**
```
[INFO] [ldlidar_stats_analyzer]: 1 - min: (0.45m, 90.3°) - max: (8.23m, 270.1°) - avg: 3.45m
[INFO] [ldlidar_stats_analyzer]: 2 - min: (0.46m, 89.8°) - max: (8.25m, 269.9°) - avg: 3.47m
[INFO] [ldlidar_stats_analyzer]: 3 - min: (0.44m, 91.2°) - max: (8.22m, 270.5°) - avg: 3.44m
```

**Log File Format:**
```
LDLidar Statistics Log
Started: 2025-10-28 14:32:15
Format: Row - min: (dist, angle) - max: (dist, angle) - average distance
================================================================================
1 - min: (0.45m, 90.3°) - max: (8.23m, 270.1°) - avg: 3.45m
2 - min: (0.46m, 89.8°) - max: (8.25m, 269.9°) - avg: 3.47m
3 - min: (0.44m, 91.2°) - max: (8.22m, 270.5°) - avg: 3.44m
...
================================================================================
Ended: 2025-10-28 14:35:22
```

**Log File Naming:**
- Format: `ldlidar_stats_YYYYMMDD_HHMMSS.txt`
- Example: `ldlidar_stats_20251028_143215.txt`
- Location: Configurable via `log_file_path` parameter (default: `~/Desktop`)

### Usage

#### Option 1: Launch with Statistics Only
```bash
ros2 launch ldlidar_node ldlidar_with_stats.launch.py
```

This launches:
- LDLidar node
- Lifecycle manager
- Robot state publisher
- Statistics analyzer node

#### Option 2: Launch with Statistics and Visualization
```bash
ros2 launch ldlidar_node ldlidar_with_stats_and_viz.launch.py
```

This launches everything from Option 1 plus:
- Interactive visualization window (with hover tooltips!)

#### Option 3: Run Statistics Analyzer Separately
If you already have the lidar running:
```bash
ros2 run ldlidar_node ldlidar_stats.py --ros-args -p update_rate:=2.0 -p log_file_path:=/home/pi/lidar_logs
```

### Configuration Options

You can override parameters at launch time:

```bash
# Change update rate to 2 Hz (twice per second)
ros2 run ldlidar_node ldlidar_stats.py --ros-args -p update_rate:=2.0

# Change log file location
ros2 run ldlidar_node ldlidar_stats.py --ros-args -p log_file_path:=/home/pi/Documents

# Subscribe to different topic
ros2 run ldlidar_node ldlidar_stats.py --ros-args -p scan_topic:=/custom_scan
```

Or edit `ldlidar.yaml`:

```yaml
ldlidar_stats_analyzer:
  ros__parameters:
    update_rate: 2.0                      # Compute stats twice per second
    log_file_path: '/home/pi/Documents'  # Save logs to Documents folder
```

---

## Building and Installation

After making these changes, rebuild the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ldlidar_node --symlink-install
source install/setup.bash
```

The `--symlink-install` flag allows you to modify Python scripts without rebuilding.

---

## Technical Details

### Statistics Computation
- **Valid Points**: Only points within `range_min` to `range_max` that are finite (not NaN or Inf)
- **Angles**: Reported in degrees (0-360°)
- **Precision**: 
  - Distance: 2 decimal places (0.01m = 1cm)
  - Angle: 1 decimal place (0.1°)
  - Average: 2 decimal places

### Performance Considerations
- **Update Rate**: Default 1 Hz is recommended for Raspberry Pi 4
  - Higher rates (e.g., 10 Hz) work fine but generate more data
  - Lower rates (e.g., 0.5 Hz) reduce CPU load
- **File I/O**: Asynchronous write operations don't block scan processing
- **Memory**: Minimal memory footprint (~5-10 MB)

### Thread Safety
- Both nodes (visualizer and stats) subscribe independently
- No shared state between nodes
- Can run simultaneously without conflicts

---

## Troubleshooting

### Stats Analyzer Issues

**Problem**: No log file created
```bash
# Check permissions
ls -ld ~/Desktop
# Ensure directory exists
mkdir -p ~/Desktop
```

**Problem**: No terminal output
```bash
# Check if node is running
ros2 node list | grep stats

# Check if receiving data
ros2 topic hz /ldlidar_node/scan
```

**Problem**: "No scan data received yet" warning
```bash
# Verify lidar is publishing
ros2 topic echo /ldlidar_node/scan --once

# Check lifecycle state
ros2 lifecycle get /ldlidar_node
```

### Visualizer Tooltip Issues

**Problem**: Tooltip not appearing
- Ensure you're hovering within 30cm of a point
- Try zooming in for better precision
- Check that scan data is valid (not all NaN)

**Problem**: Tooltip shows wrong angle
- Angles are in sensor coordinates (0° = top)
- This matches the visualization orientation

---

## Example Session

```bash
# Terminal 1: Launch everything
ros2 launch ldlidar_node ldlidar_with_stats_and_viz.launch.py

# Output:
[INFO] [lifecycle_manager]: Creating lifecycle manager...
[INFO] [ldlidar_node]: State: 'active [3]'
[INFO] [ldlidar_visualizer]: LDLidar Visualizer started
[INFO] [ldlidar_stats_analyzer]: LDLidar Statistics Analyzer started
[INFO] [ldlidar_stats_analyzer]: Log file created: /home/pi/Desktop/ldlidar_stats_20251028_143215.txt
[INFO] [ldlidar_stats_analyzer]: 1 - min: (0.45m, 90.3°) - max: (8.23m, 270.1°) - avg: 3.45m
[INFO] [ldlidar_stats_analyzer]: 2 - min: (0.46m, 89.8°) - max: (8.25m, 269.9°) - avg: 3.47m
...

# The visualization window opens automatically
# Hover over points to see (distance, angle) tooltips
# Stats are logged every second to terminal and file
```

---

## Files Summary

### Modified Files:
1. `ldlidar_node/scripts/ldlidar_visualizer.py` - Added hover tooltips
2. `ldlidar_node/params/ldlidar.yaml` - Added stats parameters
3. `ldlidar_node/CMakeLists.txt` - Added stats script installation

### New Files:
1. `ldlidar_node/scripts/ldlidar_stats.py` - Statistics analyzer node
2. `ldlidar_node/launch/ldlidar_with_stats.launch.py` - Launch with stats
3. `ldlidar_node/launch/ldlidar_with_stats_and_viz.launch.py` - Launch with stats and viz
4. `ENHANCEMENTS.md` - This documentation file

---

## Future Enhancements

Possible improvements for future versions:
- Export statistics to CSV format for data analysis
- Add configurable distance/angle units (metric/imperial, radians/degrees)
- Plot statistics trends over time
- Add alerts for min/max distance thresholds
- Support for multiple lidar sensors
- ROS2 service to query current statistics on demand

---

## License

These enhancements maintain the same Apache License 2.0 as the original package.

## Author

Enhancement developed for Raspberry Pi 4 LDLidar deployment.

**Date**: October 28, 2025
