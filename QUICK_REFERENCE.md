# Quick Reference: LDLidar Enhancements

## What Was Added

### 1. Hover Tooltips on Visualization ✅
- **What**: Mouse hover shows distance and angle for each lidar point
- **Format**: `(X.XXm, XXX.X°)` - Example: `(2.45m, 135.2°)`
- **How to use**: Launch viz and hover mouse over points

### 2. Statistics Analyzer Node ✅
- **What**: Logs min/max/avg distance with angles every second
- **Output**: Terminal + timestamped text file on Desktop
- **Format**: `Row - min: (dist, angle) - max: (dist, angle) - avg: dist`

---

## Quick Start Guide

### Launch with Stats Only (Headless)
```bash
ros2 launch ldlidar_node ldlidar_with_stats.launch.py
```

### Launch with Stats + Visualization (Recommended)
```bash
ros2 launch ldlidar_node ldlidar_with_stats_and_viz.launch.py
```

### Original Launch (No Stats)
```bash
ros2 launch ldlidar_node ldlidar_with_viz.launch.py  # Still has hover tooltips!
```

---

## Configuration

Edit `~/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_node/params/ldlidar.yaml`:

```yaml
ldlidar_stats_analyzer:
  ros__parameters:
    scan_topic: '/ldlidar_node/scan'
    update_rate: 1.0        # Change to 0.5 for slower updates, 2.0 for faster
    log_file_path: '~/Desktop'  # Change to '~/Documents' or other path
```

---

## Files Changed

### Modified:
- `ldlidar_node/scripts/ldlidar_visualizer.py` - Added hover tooltips
- `ldlidar_node/params/ldlidar.yaml` - Added stats parameters
- `ldlidar_node/CMakeLists.txt` - Install stats script

### Created:
- `ldlidar_node/scripts/ldlidar_stats.py` - Stats analyzer node
- `ldlidar_node/launch/ldlidar_with_stats.launch.py` - Launch with stats
- `ldlidar_node/launch/ldlidar_with_stats_and_viz.launch.py` - Launch with both
- `ENHANCEMENTS.md` - Full documentation
- `QUICK_REFERENCE.md` - This file

---

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select ldlidar_node --symlink-install
source install/setup.bash
```

---

## Example Output

### Terminal:
```
[INFO] [ldlidar_stats_analyzer]: Log file created: /home/pi/Desktop/ldlidar_stats_20251028_143215.txt
[INFO] [ldlidar_stats_analyzer]: 1 - min: (0.45m, 90.3°) - max: (8.23m, 270.1°) - avg: 3.45m
[INFO] [ldlidar_stats_analyzer]: 2 - min: (0.46m, 89.8°) - max: (8.25m, 269.9°) - avg: 3.47m
```

### Log File (Desktop/ldlidar_stats_20251028_143215.txt):
```
LDLidar Statistics Log
Started: 2025-10-28 14:32:15
Format: Row - min: (dist, angle) - max: (dist, angle) - average distance
================================================================================
1 - min: (0.45m, 90.3°) - max: (8.23m, 270.1°) - avg: 3.45m
2 - min: (0.46m, 89.8°) - max: (8.25m, 269.9°) - avg: 3.47m
...
```

---

## Troubleshooting

**No log file?**
```bash
mkdir -p ~/Desktop  # Ensure Desktop exists
```

**Stats not updating?**
```bash
ros2 topic hz /ldlidar_node/scan  # Should show ~10 Hz
ros2 lifecycle get /ldlidar_node  # Should be "active [3]"
```

**Tooltip not showing?**
- Hover closer to points (within ~30cm in data units)
- Zoom in for better precision

---

For complete documentation, see `ENHANCEMENTS.md`
