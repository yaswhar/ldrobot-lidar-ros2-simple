#!/usr/bin/env python3
"""
LDLidar Ground Mapper Node
Generates a 2D topological map by stacking lidar scans in polar coordinates
based on drone velocity, simulating a ground scanner.

Features:
- Subscribes to LaserScan data
- Stacks scans in POLAR coordinates (preserving angle/range format)
- Displays like ldlidar_visualizer with accumulating points
- Real-time scrolling visualization showing scan history
- Configurable map buffer time
- Optional map saving functionality
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
import math
from collections import deque
import os
from datetime import datetime


class GroundMapper(Node):
    def __init__(self):
        super().__init__('ldlidar_ground_mapper')
        
        # Declare parameters - read from shared /** namespace
        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('drone_velocity', 1.0)
        self.declare_parameter('lidar.angle_crop_min', 0.0)
        self.declare_parameter('lidar.angle_crop_max', 360.0)
        self.declare_parameter('lidar.range_max', 12.0)
        
        # Node-specific parameters from ground_mapper namespace
        self.declare_parameter('map_buffer_time', 120.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('point_size', 3.0)
        self.declare_parameter('colormap', 'jet_r')
        
        # Get shared parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.drone_velocity = self.get_parameter('drone_velocity').value
        self.angle_crop_min = self.get_parameter('lidar.angle_crop_min').value
        self.angle_crop_max = self.get_parameter('lidar.angle_crop_max').value
        self.range_max = self.get_parameter('lidar.range_max').value
        
        # Get node-specific parameters
        self.map_buffer_time = self.get_parameter('map_buffer_time').value
        self.update_rate = self.get_parameter('update_rate').value
        self.point_size = self.get_parameter('point_size').value
        self.colormap = self.get_parameter('colormap').value
        
        # Initialize data storage - store points in polar coordinates
        # Each entry: (timestamp, offset_distance, angles[], ranges[])
        self.scan_buffer = deque()
        self.offset_distance = 0.0  # Current offset distance based on velocity
        self.last_timestamp = None
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
        self.get_logger().info(f'Map buffer: {self.map_buffer_time} seconds ({self.map_buffer_time/60:.1f} minutes)')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Display: Polar coordinates (like visualizer with accumulation)')
        self.get_logger().info('Controls: Right-click drag=pan, Scroll=zoom, Space=pause')
        
        # Setup matplotlib figure
        self.setup_plot()
        
        # Pause state (set in setup_plot, but initialize here too for clarity)
        self.paused = False
        
    def scan_callback(self, msg):
        """Store scan in polar coordinates with offset based on velocity"""
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
        
        # Calculate time delta and update offset
        if self.last_timestamp is not None:
            delta_t = current_timestamp - self.last_timestamp
            # Calculate distance traveled in scanning direction
            delta_offset = self.drone_velocity * delta_t
            self.offset_distance += delta_offset
        
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
        
        # Update last timestamp
        self.last_timestamp = current_timestamp
        
        # Cleanup old scans
        self.cleanup_old_scans(current_timestamp)
        
    def cleanup_old_scans(self, current_timestamp):
        """Remove scans older than map_buffer_time"""
        cutoff_time = current_timestamp - self.map_buffer_time
        while self.scan_buffer and self.scan_buffer[0][0] < cutoff_time:
            self.scan_buffer.popleft()
    
    def setup_plot(self):
        """Setup the matplotlib plot for polar visualization (like ldlidar_visualizer)"""
        # Create figure with smaller size
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111)
        
        # Set equal aspect ratio for proper circular display
        self.ax.set_aspect('equal', adjustable='box')
        
        # Use tight layout
        self.fig.tight_layout(pad=2.0)
        
        # Pause state
        self.paused = False
        
        # Initial plot limits (will be updated when first scan arrives)
        max_range = 12.0 * 1.05
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        
        # Setup colormap for range visualization (same as visualizer)
        self.norm = Normalize(vmin=0, vmax=12.0)
        self.cmap = plt.get_cmap(self.colormap)
        self.sm = ScalarMappable(norm=self.norm, cmap=self.cmap)
        
        # Add colorbar
        self.cbar = plt.colorbar(self.sm, ax=self.ax, pad=0.05, shrink=0.9)
        self.cbar.set_label('Range (m)', rotation=270, labelpad=20)
        
        # Initialize scatter plot
        self.scatter = self.ax.scatter([], [], c=[], s=self.point_size,
                                       cmap=self.cmap, norm=self.norm,
                                       alpha=0.6, edgecolors='none')
        
        # Labels and title
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.title = self.ax.set_title(
            'Ground Scanning Topological Map (Polar View)\nWaiting for data...',
            pad=10, fontsize=12, fontweight='bold'
        )
        
        # Draw polar grid
        self.draw_polar_grid(max_range)
        
        # Enable interactive features
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Pan state
        self.panning = False
        self.pan_start = None
        
        # Add pause button
        from matplotlib.widgets import Button
        pause_ax = self.fig.add_axes([0.81, 0.025, 0.15, 0.04])
        self.pause_btn = Button(pause_ax, 'Pause')
        self.pause_btn.on_clicked(self.toggle_pause)
    
    def draw_polar_grid(self, max_range):
        """Draw polar grid lines (circles and radial lines) - same as visualizer"""
        # Draw concentric circles
        num_circles = 6
        for i in range(1, num_circles + 1):
            r = (max_range / num_circles) * i
            circle = plt.Circle((0, 0), r, fill=False, color='gray',
                               alpha=0.3, linewidth=0.5, linestyle='--')
            self.ax.add_patch(circle)
            # Add range labels
            self.ax.text(0, r, f'{r:.1f}m', ha='center', va='bottom',
                        fontsize=8, color='gray', alpha=0.7)
        
        # Draw radial lines within sensor angle range
        # Transform: rotate by 90° so 0° is at top, negate X for clockwise
        angle_min_deg = math.degrees(self.scan_params['angle_min'])
        angle_max_deg = math.degrees(self.scan_params['angle_max'])
        
        # Draw radial lines every 30 degrees
        start_angle = int(np.ceil(angle_min_deg / 30.0)) * 30
        sensor_angle_deg = start_angle
        while sensor_angle_deg <= angle_max_deg:
            display_angle_rad = np.radians(90 - sensor_angle_deg)
            x = [0, max_range * np.cos(display_angle_rad)]
            y = [0, max_range * np.sin(display_angle_rad)]
            self.ax.plot(x, y, color='gray', alpha=0.3, linewidth=0.5, linestyle='--')
            # Add angle labels
            label_r = max_range * 1.05
            label_x = label_r * np.cos(display_angle_rad)
            label_y = label_r * np.sin(display_angle_rad)
            self.ax.text(label_x, label_y, f'{sensor_angle_deg:.0f}°', ha='center', va='center',
                        fontsize=8, color='gray', alpha=0.7)
            sensor_angle_deg += 30
        
        # Draw boundary lines at exact min/max angles
        for boundary_angle_deg in [angle_min_deg, angle_max_deg]:
            display_angle_rad = np.radians(90 - boundary_angle_deg)
            x = [0, max_range * np.cos(display_angle_rad)]
            y = [0, max_range * np.sin(display_angle_rad)]
            self.ax.plot(x, y, color='red', alpha=0.5, linewidth=1.0, linestyle='-')
        
        # Draw origin marker
        self.ax.plot(0, 0, 'k+', markersize=10, markeredgewidth=2)
        
    def update_plot(self, frame):
        """Update the plot with all accumulated scans in polar view"""
        if not self.first_scan_received or not self.scan_buffer:
            return self.scatter,
        
        # Calculate scanning direction (middle of the scanning arc)
        scan_center_angle = (self.scan_params['angle_min'] + self.scan_params['angle_max']) / 2.0
        # Transform to display coordinates (90° rotation, negate for clockwise)
        scanning_direction_rad = np.pi/2 - scan_center_angle
        
        # Collect all points from all scans in buffer
        # Create a snapshot of the buffer to avoid mutation during iteration
        all_x = []
        all_y = []
        all_ranges = []
        
        scan_buffer_snapshot = list(self.scan_buffer)  # Create a copy
        
        for timestamp, offset, angles, ranges in scan_buffer_snapshot:
            if len(ranges) == 0:
                continue
            
            # Convert polar to cartesian with same transform as visualizer
            # Rotate by 90° so sensor 0° appears at top, negate X for clockwise
            adjusted_angles = np.pi/2 - angles
            x = ranges * np.cos(adjusted_angles)
            y = ranges * np.sin(adjusted_angles)
            
            # Apply offset in the scanning direction
            # Move the center of polar coordinates outward in scanning direction
            offset_x = offset * np.cos(scanning_direction_rad)
            offset_y = offset * np.sin(scanning_direction_rad)
            
            x = x + offset_x
            y = y + offset_y
            
            all_x.extend(x)
            all_y.extend(y)
            all_ranges.extend(ranges)
        
        if not all_x:
            return self.scatter,
        
        # Convert to numpy arrays
        all_x = np.array(all_x)
        all_y = np.array(all_y)
        all_ranges = np.array(all_ranges)
        
        # Update scatter plot
        self.scatter.set_offsets(np.c_[all_x, all_y])
        self.scatter.set_array(all_ranges)
        
        # Calculate the chord length for y-axis based on angle crop range
        # Use configured angle crop parameters from YAML
        angle_crop_min_rad = np.radians(self.angle_crop_min)
        angle_crop_max_rad = np.radians(self.angle_crop_max)
        angle_span = angle_crop_max_rad - angle_crop_min_rad
        max_range = self.range_max
        # Chord length = 2 * R * sin(angle_span / 2)
        # Use 1.2 times the max_range for the effective radius
        effective_radius = max_range * 1.2
        chord_length = 2 * effective_radius * np.sin(angle_span / 2)
        
        # Auto-adjust plot limits to follow the lidar (only if not paused)
        # When paused, user can manually pan to see historical data
        if len(all_x) > 0 and not self.paused:
            margin = 0.1  # 10% margin
            
            # Get the extent of current data
            x_min, x_max = np.min(all_x), np.max(all_x)
            y_min, y_max = np.min(all_y), np.max(all_y)
            
            # Window dimensions based on specifications:
            # Height: 1.2 times the chord length
            # Width: 2 times the height
            window_height = chord_length * 1.2
            window_width = window_height * 2.0
            
            # Center the view on the CURRENT lidar position (latest offset)
            # This ensures the window follows the lidar in real-time
            # Calculate current lidar center position based on offset and scanning direction
            current_lidar_x = self.offset_distance * np.cos(scanning_direction_rad)
            current_lidar_y = self.offset_distance * np.sin(scanning_direction_rad)
            
            # Use the current lidar position as the view center for both axes
            view_center_x = current_lidar_x
            view_center_y = current_lidar_y
            
            # Add margin
            x_margin = window_width * margin
            y_margin = window_height * margin
            
            # Set limits with the lidar-centered view
            self.ax.set_xlim(view_center_x - window_width/2 - x_margin, view_center_x + window_width/2 + x_margin)
            self.ax.set_ylim(view_center_y - window_height/2 - y_margin, view_center_y + window_height/2 + y_margin)
        
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
        
        # Calculate time span
        if total_scans > 0:
            time_span = self.scan_buffer[-1][0] - self.scan_buffer[0][0]
        else:
            time_span = 0.0
        
        # Calculate and display scanning direction
        scan_direction_deg = math.degrees(scan_center_angle)
        
        pause_status = " [PAUSED]" if self.paused else ""
        self.title.set_text(
            f'Ground Scanning Map - Accumulating in {scan_direction_deg:.0f}° Direction{pause_status}\n'
            f'Scans: {total_scans} | Points: {total_points} | '
            f'Distance: {distance_traveled:.2f}m | Time: {time_span:.1f}s | '
            f'Velocity: {self.drone_velocity:.2f}m/s'
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
    
    def on_scroll(self, event):
        """Handle zoom with scroll wheel"""
        if event.inaxes != self.ax:
            return
        
        # Get current limits
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        
        # Zoom factor
        zoom_factor = 1.2 if event.button == 'down' else 0.8
        
        # Calculate new limits centered on mouse position or current center
        if event.xdata is not None and event.ydata is not None:
            x_center = event.xdata
            y_center = event.ydata
        else:
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
        
        x_range = (x_max - x_min) / 2
        y_range = (y_max - y_min) / 2
        
        new_x_range = x_range * zoom_factor
        new_y_range = y_range * zoom_factor
        
        # Don't zoom out beyond original limits
        max_range = self.scan_params['range_max'] * 1.05
        if new_x_range > max_range:
            new_x_range = max_range
        if new_y_range > max_range:
            new_y_range = max_range
        
        # Don't zoom in too much
        if new_x_range < 0.5:
            new_x_range = 0.5
        if new_y_range < 0.5:
            new_y_range = 0.5
        
        self.ax.set_xlim(x_center - new_x_range, x_center + new_x_range)
        self.ax.set_ylim(y_center - new_y_range, y_center + new_y_range)
        self.fig.canvas.draw_idle()
    
    def on_click(self, event):
        """Handle click events - right click starts panning"""
        if event.inaxes != self.ax:
            return
        
        # Right click - start panning
        if event.button == 3:
            self.panning = True
            self.pan_start = (event.xdata, event.ydata)
    
    def on_release(self, event):
        """Handle button release - stop panning"""
        if event.button == 3:
            self.panning = False
            self.pan_start = None
    
    def on_motion(self, event):
        """Handle mouse motion for panning"""
        if not self.panning or self.pan_start is None or event.inaxes != self.ax:
            return
        
        # Calculate the difference
        dx = event.xdata - self.pan_start[0]
        dy = event.ydata - self.pan_start[1]
        
        # Get current limits
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        
        # Update limits
        self.ax.set_xlim(x_min - dx, x_max - dx)
        self.ax.set_ylim(y_min - dy, y_max - dy)
        
        # Update pan start
        self.pan_start = (event.xdata - dx, event.ydata - dy)
        
        self.fig.canvas.draw_idle()
    
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
