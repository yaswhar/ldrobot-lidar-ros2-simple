# Ground Mapper Fixes - November 5, 2025

## Issues Fixed

### 1. **RuntimeError: deque mutated during iteration**

**Problem**: The scan buffer (deque) was being modified by the ROS callback thread while the matplotlib animation thread was iterating over it, causing a runtime error.

**Solution**: Create a snapshot copy of the buffer before iterating:
```python
scan_buffer_snapshot = list(self.scan_buffer)  # Create a copy
for timestamp, offset, angles, ranges in scan_buffer_snapshot:
    # Process scans safely
```

### 2. **Missing Offset Application in Scanning Direction**

**Problem**: The polar coordinate center wasn't moving with the lidar's velocity. All scans were overlapping at the origin.

**Solution**: Calculate the scanning direction from angle crop parameters and apply offset:

```python
# Calculate scanning direction (middle of scanning arc)
scan_center_angle = (angle_min + angle_max) / 2.0
# Example: (210° + 330°) / 2 = 270° (left direction)

# Transform to display coordinates
scanning_direction_rad = π/2 - scan_center_angle

# Apply offset to each scan
offset_x = offset * cos(scanning_direction_rad)
offset_y = offset * sin(scanning_direction_rad)

# Shift all points in this scan
x = x + offset_x
y = y + offset_y
```

### 3. **Auto-Scaling View**

**Added**: Automatic adjustment of plot limits to show all accumulated data with 10% margin:

```python
# Calculate bounds
x_min, x_max = np.min(all_x), np.max(all_x)
y_min, y_max = np.min(all_y), np.max(all_y)

# Add margin and set limits
self.ax.set_xlim(x_min - margin, x_max + margin)
self.ax.set_ylim(y_min - margin, y_max + margin)
```

## How It Works Now

### Scanning Direction Calculation

From your configuration (`ldlidar.yaml`):
- `angle_crop_min: 210.0°`
- `angle_crop_max: 330.0°`
- **Scanning direction**: (210° + 330°) / 2 = **270°** (pointing left in sensor frame)

### Coordinate System

1. **Sensor coordinates**: 
   - 0° = forward (along sensor's "nose")
   - 90° = right
   - 180° = backward
   - 270° = left

2. **Display coordinates** (after 90° rotation):
   - 0° sensor → top of display
   - 90° sensor → right of display
   - 180° sensor → bottom of display
   - 270° sensor → left of display

### Motion Simulation

For your configuration (270° scanning direction):
- Each new scan is offset further to the **left** in the display
- Creates a "trail" of scans showing the ground being scanned
- Distance between scans = velocity × time_between_scans

Example with velocity = 1.0 m/s and scan_freq = 10 Hz:
- Scan 0: offset = 0.0 m
- Scan 1: offset = 0.1 m (shifted 0.1m to the left)
- Scan 2: offset = 0.2 m (shifted 0.2m to the left)
- ...and so on

After 10 seconds at 1 m/s:
- ~100 scans accumulated
- Spanning ~10 meters in the scanning direction
- Creating a "topological map" of the scanned ground

## Updated Log Messages

The node now displays:
```
[INFO] Ground Mapper started
[INFO] Subscribing to: /ldlidar_node/scan
[INFO] Scanning velocity: 1.0 m/s
[INFO] Map buffer: 10.0 seconds
[INFO] Update rate: 10.0 Hz
[INFO] Display: Polar coordinates (like visualizer with accumulation)
[INFO] First scan received: range=[0.03, 1.00]m, angle=[210.0, 330.0]°, scan_freq=9.9Hz
[INFO] Scanning direction: 270.0° (center moves outward in this direction as lidar travels)
```

## Visualization Title

The title now shows:
```
Ground Scanning Map - Accumulating in 270° Direction
Scans: 45 | Points: 12,345 | Distance: 4.50m | Time: 4.5s | Velocity: 1.00m/s
```

## Testing

### Static Test (velocity = 0)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.0
```
**Expected**: All scans overlap at origin (no offset applied)

### Slow Motion (velocity = 0.5)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=0.5
```
**Expected**: Scans spread slowly in 270° direction

### Normal Speed (velocity = 1.0)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py
```
**Expected**: Scans spread at 1 m/s in 270° direction, creating a ground map

### High Speed (velocity = 2.0)
```bash
ros2 launch ldlidar_node ldlidar_ground_mapper.launch.py drone_velocity_y:=2.0
```
**Expected**: Faster accumulation, larger coverage area

## Angle Crop Settings Examples

### Forward Scanning (0° to 180°)
```yaml
angle_crop_min: 0.0
angle_crop_max: 180.0
```
- Scanning direction: 90° (right)
- Scans accumulate to the right

### Backward Scanning (180° to 360°)
```yaml
angle_crop_min: 180.0
angle_crop_max: 360.0
```
- Scanning direction: 270° (left)
- Scans accumulate to the left

### Narrow Forward Arc (330° to 30°)
```yaml
angle_crop_min: 330.0
angle_crop_max: 30.0
```
- Scanning direction: 0° (forward)
- Scans accumulate forward (upward in display)

## Performance Notes

- **Thread Safety**: Snapshot copy prevents deque mutation errors
- **Memory Usage**: Buffer limited by `map_buffer_time` (default 10s)
- **Update Rate**: 10 Hz provides smooth visualization without excessive CPU
- **Auto-scaling**: View adjusts to show all data as coverage grows

## Visual Behavior

As the lidar "moves" in the scanning direction:
1. New scans appear offset from previous ones
2. Creates a "scanning trail" effect
3. Points accumulate showing terrain/obstacles
4. Buffer rotates (old scans drop off after buffer time)
5. View auto-scales to keep all visible

This simulates a lidar mounted on a moving platform (e.g., drone) scanning the ground below as it moves forward.
