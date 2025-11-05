# Ground Mapper Update - Polar Coordinate Implementation

## Summary of Changes

The `ldlidar_ground_mapper.py` has been completely refactored to work in **polar coordinates** instead of Cartesian coordinates, providing a visualization similar to `ldlidar_visualizer.py` but with **accumulating scans** over time.

## Key Changes

### 1. **Data Storage in Polar Coordinates**
- **Before**: Stored full `LaserScan` messages and converted to Cartesian coordinates
- **After**: Stores only valid points in polar format (angles, ranges) with offset tracking
- **Buffer format**: `(timestamp, offset_distance, angles[], ranges[])`

### 2. **Visualization Style**
- **Before**: Grid-based 2D map with Y-axis stacking (like a ground-scanning radar)
- **After**: Polar scatter plot (like the visualizer) showing accumulated points from multiple scans
- **Display**: Matches `ldlidar_visualizer.py` style with circular polar grid

### 3. **Coordinate Transform**
- Uses the **same transformation** as the visualizer:
  - Rotate by 90° so sensor 0° appears at top
  - Negate X for clockwise rotation
  - Formula: `adjusted_angles = π/2 - angles`

### 4. **Scan Frequency Detection**
- Now reads `scan_time` from LaserScan messages to determine scanning frequency
- Displays scan frequency in Hz when first scan is received
- Uses this for accurate offset distance calculation

### 5. **Parameter Changes**
In `ldlidar.yaml`:
- **Removed**: `map_resolution` (no longer needed for polar view)
- **Added**: `point_size` (controls scatter plot point size)
- **Changed**: `colormap` default from 'viridis' to 'jet_r' (matches visualizer)

## How It Works

### Scanning Direction
The scanning direction is defined by the **angle crop parameters** in `ldlidar.yaml`:
- `angle_crop_min`: Start angle of scanning arc (degrees)
- `angle_crop_max`: End angle of scanning arc (degrees)
- The arc between these angles is considered the **scanning direction**

### Velocity Integration
- The lidar moves at `drone_velocity_y` m/s in the scanning direction
- Each scan is timestamped and an offset distance is calculated
- Offset = velocity × time_elapsed
- **Note**: Currently, offset is tracked but NOT applied to visualization (all scans overlap in polar view)

### Buffer Management
- Scans are stored for `map_buffer_time` seconds (default: 10s)
- Old scans are automatically removed when they exceed the buffer time
- This creates a **sliding window** of accumulated scans

## Visualization Features

### Display
- **Polar grid**: Concentric circles for range, radial lines for angles
- **Color coding**: Red (close) to Blue (far) based on range
- **Accumulation**: All scans in buffer are displayed simultaneously
- **Real-time stats**: Shows scan count, points, distance traveled, time span, velocity

### Interactive Controls
- **Scroll wheel**: Zoom in/out
- **Right-click + drag**: Pan the view
- **Auto-scaling**: Limits zoom to prevent going beyond sensor range

## Usage

```bash
# Launch with default velocity (1.0 m/s)
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py

# Launch with custom velocity
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.5

# Static test (velocity = 0) - All scans overlap perfectly
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.0
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scan_topic` | string | `/ldlidar_node/scan` | LaserScan topic to subscribe to |
| `drone_velocity_y` | float | 1.0 | Velocity in scanning direction (m/s) |
| `map_buffer_time` | float | 10.0 | Seconds of scan data to keep |
| `update_rate` | float | 10.0 | Visualization refresh rate (Hz) |
| `point_size` | float | 3.0 | Scatter plot point size |
| `colormap` | string | 'jet_r' | Matplotlib colormap name |
| `save_map` | bool | false | Auto-save maps periodically |
| `map_output_dir` | string | '/workspace/maps' | Directory for saved maps |

## Technical Details

### Scan Frequency
- Automatically detected from `msg.scan_time` field in LaserScan
- Typical values: 8-10 Hz for LD19, varies by model
- Used for offset calculation: `offset += velocity × time_delta`

### Angle Cropping
- Respects the angle cropping settings from lidar configuration
- Only processes points within `angle_crop_min` to `angle_crop_max`
- Grid lines are drawn only within the active scanning arc

### Memory Efficiency
- Stores only **valid points** (filters NaN, inf, out-of-range)
- Buffer automatically prunes old scans
- No grid allocation (unlike previous Cartesian version)

## Differences from ldlidar_visualizer.py

| Feature | Visualizer | Ground Mapper |
|---------|-----------|---------------|
| **Scans shown** | Only latest scan | Accumulated scans in buffer |
| **Time window** | Single snapshot | Configurable buffer (e.g., 10s) |
| **Purpose** | Real-time monitoring | Topological mapping |
| **Velocity** | N/A | Tracks distance traveled |
| **Stats** | Current scan only | Aggregate over buffer |
| **Point interaction** | Click to show range/angle | Pan/zoom only |

## Future Enhancements (Optional)

1. **Offset Application**: Apply the calculated offset to shift scans along the scanning direction
   - Would create a "scanning path" visualization
   - Requires 3D-like view or projection

2. **Trail Mode**: Fade older scans with transparency based on age

3. **Density Map**: Convert accumulated points to density heatmap

4. **Export**: Save accumulated point cloud in standard format (PCD, XYZ)

## Testing

### Static Test (velocity = 0)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.0
```
**Expected**: All scans overlap perfectly in polar view, creating a dense point cloud

### Slow Motion (velocity = 0.5)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.5
```
**Expected**: Offset tracks motion, but view remains polar (scans accumulate)

### Normal Operation (velocity = 1.0)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=1.0
```
**Expected**: Full-speed offset tracking with accumulated polar view

---

## Implementation Notes

The key insight is that for ground scanning, the lidar is moving in the direction defined by its scanning arc. The polar coordinate system naturally represents this, as each scan shows obstacles at various angles and ranges from the lidar's perspective. By accumulating these scans over time, we build up a topological map of the scanned area.

The velocity parameter tracks how far the lidar has traveled, which could be used in future versions to create a "path view" or to apply spatial transformations to the accumulated scans.
