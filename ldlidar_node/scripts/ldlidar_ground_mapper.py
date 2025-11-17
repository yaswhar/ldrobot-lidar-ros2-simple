#!/usr/bin/env python3
"""
LDLidar 3D Ground Mapper Node
Generates a 3D topological map by stacking lidar scans with elevation angle
based on drone velocity, creating a true 3D representation of the ground.

Features:
- Subscribes to LaserScan data
- Applies pitch/elevation angle to create tilted scan planes
- Stacks angled scan planes in 3D space along travel direction
- Adaptive spatial resolution based on drone velocity
  * Higher velocity → lower spatial resolution (fewer samples per meter)
  * Lower velocity → higher spatial resolution (more samples per meter)
  * Constant time resolution (update_rate) maintained
- Interactive 3D visualization with rotation, zoom, and pan
- Right-handed coordinate system: X=Forward, Y=Right, Z=Down (nadir)
- Configurable map buffer time
- Runtime parameter updates supported

Coordinate System:
- X-axis: Forward (travel direction component)
- Y-axis: Right (lateral/sideways)
- Z-axis: Down/Nadir (positive Z points downward, Z=0 at lidar mount)

Spatial Resolution Logic:
- target_spatial_resolution = drone_velocity / update_rate
- Example: 1.0 m/s / 10 Hz = 0.1 m (10 cm between samples)
- Example: 0.5 m/s / 10 Hz = 0.05 m (5 cm between samples)
- Example: 2.0 m/s / 10 Hz = 0.2 m (20 cm between samples)

3D Transformation:
- Elevation angle (α) creates pitch around Y-axis
- For point at range r, sensor angle θ:
  * x_world = r·cos(θ)·cos(α) + offset_x
  * y_world = r·sin(θ) + offset_y
  * z_world = r·cos(θ)·sin(α) + offset_z
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive display
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d import Axes3D
import math
from collections import deque
import os
from datetime import datetime


class GroundMapper(Node):
    def __init__(self):
        super().__init__('ground_mapper',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        # Get shared parameters from lidar namespace in /** 
        # With automatically_declare_parameters_from_overrides=True, 
        # parameters from YAML are automatically available
        scan_topic = self.get_parameter_or('lidar.scan_topic', 
                                           rclpy.Parameter('lidar.scan_topic', 
                                                          rclpy.Parameter.Type.STRING, 
                                                          '/ldlidar_node/scan')).value
        self.drone_velocity = self.get_parameter_or('lidar.drone_velocity',
                                                    rclpy.Parameter('lidar.drone_velocity',
                                                                   rclpy.Parameter.Type.DOUBLE,
                                                                   1.0)).value
        
        # Get other lidar parameters (nested under lidar: in YAML)
        self.angle_crop_min = self.get_parameter_or('lidar.angle_crop_min',
                                                     rclpy.Parameter('lidar.angle_crop_min',
                                                                    rclpy.Parameter.Type.DOUBLE,
                                                                    0.0)).value
        self.angle_crop_max = self.get_parameter_or('lidar.angle_crop_max',
                                                     rclpy.Parameter('lidar.angle_crop_max',
                                                                    rclpy.Parameter.Type.DOUBLE,
                                                                    360.0)).value
        self.range_max = self.get_parameter_or('lidar.range_max',
                                               rclpy.Parameter('lidar.range_max',
                                                              rclpy.Parameter.Type.DOUBLE,
                                                              12.0)).value
        self.elevation_angle = self.get_parameter_or('lidar.elevation_angle',
                                                      rclpy.Parameter('lidar.elevation_angle',
                                                                     rclpy.Parameter.Type.DOUBLE,
                                                                     -30.0)).value
        self.elevation_angle_rad = np.radians(self.elevation_angle)
        
        # Get node-specific parameters (from ground_mapper: namespace)
        self.map_buffer_time = self.get_parameter_or('map_buffer_time',
                                                      rclpy.Parameter('map_buffer_time',
                                                                     rclpy.Parameter.Type.DOUBLE,
                                                                     120.0)).value
        self.update_rate = self.get_parameter_or('update_rate',
                                                  rclpy.Parameter('update_rate',
                                                                 rclpy.Parameter.Type.DOUBLE,
                                                                 10.0)).value
        self.point_size = self.get_parameter_or('point_size',
                                                 rclpy.Parameter('point_size',
                                                                rclpy.Parameter.Type.DOUBLE,
                                                                3.0)).value
        self.colormap = self.get_parameter_or('colormap',
                                               rclpy.Parameter('colormap',
                                                              rclpy.Parameter.Type.STRING,
                                                              'jet_r')).value
        
        # Calculate spatial resolution based on velocity and update rate
        # Spatial resolution = velocity / update_rate
        # e.g., 1.0 m/s / 10 Hz = 0.1 m = 10 cm between samples
        self.target_spatial_resolution = self.drone_velocity / self.update_rate
        
        # Set up parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Initialize data storage - store points in polar coordinates
        # Each entry: (timestamp, offset_distance, angles[], ranges[])
        self.scan_buffer = deque()
        self.offset_distance = 0.0  # Current offset distance based on velocity
        self.last_timestamp = None
        self.last_accepted_scan_timestamp = None  # For decimation tracking
        self.first_scan_received = False
        
        # Scan parameters
        self.scan_params = {
            'angle_min': 0.0,
            'angle_max': 2 * np.pi,
            'range_min': 0.0,
            'range_max': 12.0,
            'scan_time': 0.0  # Will be updated from messages
        }
        
        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.get_logger().info(f'Ground Mapper started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Drone velocity: {self.drone_velocity} m/s')
        self.get_logger().info(f'Elevation angle: {self.elevation_angle}° (pitch)')
        self.get_logger().info(f'Target sampling rate: {self.update_rate} Hz')
        self.get_logger().info(f'Spatial resolution: {self.target_spatial_resolution:.3f} m ({self.target_spatial_resolution*100:.1f} cm)')
        self.get_logger().info(f'Map buffer: {self.map_buffer_time} seconds ({self.map_buffer_time/60:.1f} minutes)')
        self.get_logger().info(f'Display: 3D coordinates (X=Forward, Y=Right, Z=Down/Nadir)')
        self.get_logger().info('Controls: Left-drag=rotate, Right-drag/Scroll=zoom, Middle-drag=pan, Space=pause')
        
        # Setup matplotlib figure
        self.setup_plot()
        
        # Pause state (set in setup_plot, but initialize here too for clarity)
        self.paused = False
        
    def scan_callback(self, msg):
        """Store scan in polar coordinates with offset based on velocity and spatial resolution"""
        # Skip if paused
        if self.paused:
            return
            
        # Get timestamp in seconds
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Update scan parameters from first message
        if not self.first_scan_received:
            self.scan_params['angle_min'] = msg.angle_min
            self.scan_params['angle_max'] = msg.angle_max
            self.scan_params['range_min'] = msg.range_min
            self.scan_params['range_max'] = msg.range_max
            self.scan_params['scan_time'] = msg.scan_time if msg.scan_time > 0 else 0.1
            self.last_timestamp = current_timestamp
            self.last_accepted_scan_timestamp = current_timestamp
            self.first_scan_received = True
            
            scan_freq = 1.0 / self.scan_params['scan_time'] if self.scan_params['scan_time'] > 0 else 10.0
            scan_center_deg = math.degrees((msg.angle_min + msg.angle_max) / 2.0)
            
            self.get_logger().info(
                f'First scan received: '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m, '
                f'angle=[{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}]°, '
                f'scan_freq={scan_freq:.1f}Hz'
            )
            self.get_logger().info(
                f'Scanning direction: {scan_center_deg:.1f}° '
                f'(center moves outward in this direction as lidar travels)'
            )
        
        # Calculate time delta and update offset (always track distance)
        if self.last_timestamp is not None:
            delta_t = current_timestamp - self.last_timestamp
            # Calculate distance traveled in scanning direction
            delta_offset = self.drone_velocity * delta_t
            self.offset_distance += delta_offset
        
        # Update last timestamp (for distance tracking)
        self.last_timestamp = current_timestamp
        
        # DECIMATION LOGIC: Only accept scan if enough distance has been traveled
        # This implements constant TIME resolution (update_rate) but variable SPATIAL resolution (based on velocity)
        if self.last_accepted_scan_timestamp is not None:
            time_since_last_accepted = current_timestamp - self.last_accepted_scan_timestamp
            distance_since_last_accepted = self.drone_velocity * time_since_last_accepted
            
            # Check if we've traveled enough distance based on target spatial resolution
            if distance_since_last_accepted < self.target_spatial_resolution:
                # Skip this scan - haven't traveled far enough yet
                return
        
        # Accept this scan
        self.last_accepted_scan_timestamp = current_timestamp
        
        # Extract scan data
        num_points = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        ranges = np.array(msg.ranges)
        
        # Filter valid points
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        valid_angles = angles[valid_mask]
        valid_ranges = ranges[valid_mask]
        
        # Store scan with offset (in polar coordinates)
        self.scan_buffer.append((current_timestamp, self.offset_distance, valid_angles, valid_ranges))
        
        # Cleanup old scans
        self.cleanup_old_scans(current_timestamp)
        
    def cleanup_old_scans(self, current_timestamp):
        """Remove scans older than map_buffer_time"""
        cutoff_time = current_timestamp - self.map_buffer_time
        while self.scan_buffer and self.scan_buffer[0][0] < cutoff_time:
            self.scan_buffer.popleft()
    
    def parameter_callback(self, params):
        """Callback for parameter changes"""
        from rcl_interfaces.msg import SetParametersResult
        
        recalculate_resolution = False
        
        for param in params:
            if param.name == 'lidar.drone_velocity':
                self.drone_velocity = param.value
                recalculate_resolution = True
                self.get_logger().info(f'Updated lidar.drone_velocity to {self.drone_velocity} m/s')
            elif param.name == 'update_rate':
                self.update_rate = param.value
                recalculate_resolution = True
                self.get_logger().info(f'Updated update_rate to {self.update_rate} Hz')
            elif param.name == 'lidar.angle_crop_min':
                self.angle_crop_min = param.value
                self.get_logger().info(f'Updated angle_crop_min to {self.angle_crop_min}°')
            elif param.name == 'lidar.angle_crop_max':
                self.angle_crop_max = param.value
                self.get_logger().info(f'Updated angle_crop_max to {self.angle_crop_max}°')
            elif param.name == 'lidar.range_max':
                self.range_max = param.value
                self.get_logger().info(f'Updated range_max to {self.range_max} m')
            elif param.name == 'lidar.elevation_angle':
                self.elevation_angle = param.value
                self.elevation_angle_rad = np.radians(self.elevation_angle)
                self.get_logger().info(f'Updated elevation_angle to {self.elevation_angle}° (pitch)')
            elif param.name == 'map_buffer_time':
                self.map_buffer_time = param.value
                self.get_logger().info(f'Updated map_buffer_time to {self.map_buffer_time} s')
            elif param.name == 'point_size':
                self.point_size = param.value
                if hasattr(self, 'scatter'):
                    self.scatter.set_sizes([self.point_size])
                self.get_logger().info(f'Updated point_size to {self.point_size}')
            elif param.name == 'colormap':
                self.colormap = param.value
                self.get_logger().info(f'Updated colormap to {self.colormap}')
        
        # Recalculate spatial resolution if velocity or update_rate changed
        if recalculate_resolution:
            self.target_spatial_resolution = self.drone_velocity / self.update_rate
            self.get_logger().info(
                f'Recalculated spatial resolution: {self.target_spatial_resolution:.3f} m '
                f'({self.target_spatial_resolution*100:.1f} cm) based on '
                f'velocity={self.drone_velocity} m/s and rate={self.update_rate} Hz'
            )
        
        return SetParametersResult(successful=True)
    
    def setup_plot(self):
        """Setup the matplotlib plot for 3D visualization"""
        # Create figure for 3D display
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Set initial isometric view
        self.ax.view_init(elev=30, azim=45)
        
        # Use tight layout
        self.fig.tight_layout(pad=2.0)
        
        # Pause state
        self.paused = False
        
        # Initial plot limits (will be updated dynamically)
        max_range = self.range_max * 1.05
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        self.ax.set_zlim(-max_range, max_range)
        
        # Setup colormap for range visualization
        self.norm = Normalize(vmin=0, vmax=self.range_max)
        self.cmap = plt.get_cmap(self.colormap)
        self.sm = ScalarMappable(norm=self.norm, cmap=self.cmap)
        
        # Add colorbar
        self.cbar = plt.colorbar(self.sm, ax=self.ax, pad=0.1, shrink=0.8)
        self.cbar.set_label('Range (m)', rotation=270, labelpad=20)
        
        # Initialize 3D scatter plot
        self.scatter = self.ax.scatter([], [], [], c=[], s=self.point_size,
                                       cmap=self.cmap, norm=self.norm,
                                       alpha=0.6, edgecolors='none',
                                       depthshade=True)
        
        # Labels and title
        self.ax.set_xlabel('X - Forward [m]', fontsize=12, labelpad=10)
        self.ax.set_ylabel('Y - Right [m]', fontsize=12, labelpad=10)
        self.ax.set_zlabel('Z - Down/Nadir [m]', fontsize=12, labelpad=10)
        self.title = self.ax.set_title(
            '3D Ground Scanning Topological Map\nWaiting for data...',
            pad=20, fontsize=12, fontweight='bold'
        )
        
        # Enable interactive features (3D has built-in rotation, zoom, pan)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Add pause button
        from matplotlib.widgets import Button
        pause_ax = self.fig.add_axes([0.81, 0.025, 0.15, 0.04])
        self.pause_btn = Button(pause_ax, 'Pause')
        self.pause_btn.on_clicked(self.toggle_pause)
    
    def update_plot(self, frame):
        """Update the plot with all accumulated scans in 3D view"""
        if not self.first_scan_received or not self.scan_buffer:
            return self.scatter,
        
        # Calculate scanning direction (middle of the scanning arc)
        scan_center_angle = (self.scan_params['angle_min'] + self.scan_params['angle_max']) / 2.0
        # Transform to display coordinates (90° rotation, negate for clockwise)
        scanning_direction_rad = np.pi/2 - scan_center_angle
        
        # Elevation angle for pitch rotation
        alpha = self.elevation_angle_rad
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        # Collect all points from all scans in buffer
        # Create a snapshot of the buffer to avoid mutation during iteration
        all_x = []
        all_y = []
        all_z = []
        all_ranges = []
        
        scan_buffer_snapshot = list(self.scan_buffer)  # Create a copy
        
        for timestamp, offset, angles, ranges in scan_buffer_snapshot:
            if len(ranges) == 0:
                continue
            
            # Convert polar to 2D cartesian (sensor's scanning plane)
            # Rotate by 90° so sensor 0° appears at top, negate X for clockwise
            adjusted_angles = np.pi/2 - angles
            x_2d = ranges * np.cos(adjusted_angles)
            y_2d = ranges * np.sin(adjusted_angles)
            
            # Apply pitch rotation around Y-axis (elevation angle)
            # The scan plane is tilted, but the lidar itself moves horizontally
            # For negative elevation (nose-down): points below lidar have POSITIVE Z
            # For positive elevation (nose-up): points above lidar have NEGATIVE Z
            # x_2d represents the forward component in sensor's tilted frame
            x_world = x_2d * cos_alpha  # Horizontal forward component
            y_world = y_2d  # Lateral component unchanged
            z_world = -x_2d * sin_alpha  # NEGATIVE sign: downward is positive Z
            
            # Calculate offset for lidar movement (positive X direction only)
            # The lidar moves forward in +X direction in world coordinates
            offset_x = offset  # Direct forward movement in +X
            offset_y = 0.0     # No lateral movement
            offset_z = 0.0     # No vertical movement - lidar stays at constant height
            
            # Apply offset
            x = x_world + offset_x
            y = y_world + offset_y
            z = z_world + offset_z
            
            all_x.extend(x)
            all_y.extend(y)
            all_z.extend(z)
            all_ranges.extend(ranges)
        
        if not all_x:
            return self.scatter,
        
        # Convert to numpy arrays
        all_x = np.array(all_x)
        all_y = np.array(all_y)
        all_z = np.array(all_z)
        all_ranges = np.array(all_ranges)
        
        # Update 3D scatter plot using _offsets3d
        self.scatter._offsets3d = (all_x, all_y, all_z)
        self.scatter.set_array(all_ranges)
        
        # Auto-adjust 3D plot limits based on data (only if not paused)
        # When paused, user can manually interact with the 3D view
        if len(all_x) > 0 and not self.paused:
            margin = 0.1  # 10% margin
            
            # Calculate data extents for all three axes
            x_min, x_max = np.min(all_x), np.max(all_x)
            y_min, y_max = np.min(all_y), np.max(all_y)
            z_min, z_max = np.min(all_z), np.max(all_z)
            
            # Calculate ranges
            x_range = x_max - x_min if x_max > x_min else self.range_max
            y_range = y_max - y_min if y_max > y_min else self.range_max
            z_range = z_max - z_min if z_max > z_min else self.range_max
            
            # Add margin to each axis
            x_margin = x_range * margin
            y_margin = y_range * margin
            z_margin = z_range * margin
            
            # Center Y-axis around zero by making limits symmetric
            y_center = (y_min + y_max) / 2
            y_half_range = max(abs(y_min - y_center), abs(y_max - y_center)) + y_margin
            
            # Set adaptive limits (Y-axis centered, Z allows negative values)
            self.ax.set_xlim(x_min - x_margin, x_max + x_margin)
            self.ax.set_ylim(y_center - y_half_range, y_center + y_half_range)
            self.ax.set_zlim(z_min - z_margin, z_max + z_margin)
        
        # Update normalization range
        if len(all_ranges) > 0:
            data_min = self.scan_params['range_min']
            data_max = self.scan_params['range_max']
            self.norm = Normalize(vmin=data_min, vmax=data_max)
            self.scatter.set_norm(self.norm)
            self.sm.set_norm(self.norm)
            self.cbar.update_normal(self.sm)
        
        # Update title with stats
        total_scans = len(self.scan_buffer)
        total_points = len(all_ranges)
        distance_traveled = self.offset_distance
        
        # Calculate time span and actual scan rate
        if total_scans > 1:
            time_span = self.scan_buffer[-1][0] - self.scan_buffer[0][0]
            actual_scan_rate = (total_scans - 1) / time_span if time_span > 0 else 0.0
        else:
            time_span = 0.0
            actual_scan_rate = 0.0
        
        # Calculate actual spatial resolution
        if total_scans > 1 and distance_traveled > 0:
            actual_spatial_res = distance_traveled / (total_scans - 1)
        else:
            actual_spatial_res = self.target_spatial_resolution
        
        # Calculate and display scanning direction
        scan_direction_deg = math.degrees(scan_center_angle)
        
        pause_status = " [PAUSED]" if self.paused else ""
        self.title.set_text(
            f'3D Ground Scanning Map - Pitch: {self.elevation_angle:.1f}° | Scanning: {scan_direction_deg:.0f}°{pause_status}\n'
            f'Scans: {total_scans} | Points: {total_points} | Distance: {distance_traveled:.2f}m | Time: {time_span:.1f}s\n'
            f'Velocity: {self.drone_velocity:.2f} m/s | Sample Rate: {actual_scan_rate:.1f} Hz | '
            f'Spatial Res: {actual_spatial_res*100:.1f} cm (target: {self.target_spatial_resolution*100:.1f} cm)'
        )
        
        return self.scatter,
    
    def toggle_pause(self, event=None):
        """Toggle pause state"""
        self.paused = not self.paused
        if self.paused:
            self.pause_btn.label.set_text('Resume')
            self.get_logger().info('Ground mapper paused')
        else:
            self.pause_btn.label.set_text('Pause')
            self.get_logger().info('Ground mapper resumed')
        self.fig.canvas.draw_idle()
    
    def on_key_press(self, event):
        """Handle keyboard shortcuts"""
        if event.key == ' ':  # Space bar to toggle pause
            self.toggle_pause()
    
    def run(self):
        """Run the visualization"""
        # Setup animation
        interval_ms = int(1000.0 / self.update_rate)
        self.anim = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=interval_ms,
            blit=False,
            cache_frame_data=False
        )
        
        # Show plot
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    mapper = GroundMapper()
    
    try:
        # Run the mapper in a separate thread
        import threading
        ros_thread = threading.Thread(target=lambda: rclpy.spin(mapper), daemon=True)
        ros_thread.start()
        
        # Run the matplotlib event loop (blocking)
        mapper.run()
        
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
