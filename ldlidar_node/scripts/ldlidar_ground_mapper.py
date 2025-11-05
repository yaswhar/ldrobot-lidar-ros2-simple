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
        
        # Declare parameters
        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('drone_velocity_y', 1.0)  # m/s in scanning direction
        self.declare_parameter('map_buffer_time', 10.0)  # seconds
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('point_size', 3.0)  # Point size for visualization
        self.declare_parameter('colormap', 'jet_r')  # matplotlib colormap (matches visualizer)
        self.declare_parameter('save_map', False)  # auto-save map images
        self.declare_parameter('map_output_dir', '/workspace/maps')
        
        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.drone_velocity_y = self.get_parameter('drone_velocity_y').value
        self.map_buffer_time = self.get_parameter('map_buffer_time').value
        self.update_rate = self.get_parameter('update_rate').value
        self.point_size = self.get_parameter('point_size').value
        self.colormap = self.get_parameter('colormap').value
        self.save_map = self.get_parameter('save_map').value
        self.map_output_dir = self.get_parameter('map_output_dir').value
        
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
        self.get_logger().info(f'Scanning velocity: {self.drone_velocity_y} m/s')
        self.get_logger().info(f'Map buffer: {self.map_buffer_time} seconds')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Display: Polar coordinates (like visualizer with accumulation)')
        
        # Setup matplotlib figure
        self.setup_plot()
        
        # Save counter
        self.save_counter = 0
        
    def scan_callback(self, msg):
        """Store scan in polar coordinates with offset based on velocity"""
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
            self.get_logger().info(
                f'First scan received: '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m, '
                f'angle=[{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}]°, '
                f'scan_freq={scan_freq:.1f}Hz'
            )
        
        # Calculate time delta and update offset
        if self.last_timestamp is not None:
            delta_t = current_timestamp - self.last_timestamp
            # Calculate distance traveled in scanning direction
            delta_offset = self.drone_velocity_y * delta_t
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
        # Create figure
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111)
        
        # Set equal aspect ratio for proper circular display
        self.ax.set_aspect('equal')
        
        # Use tight layout
        self.fig.tight_layout(pad=2.0)
        
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
        
        # Pan state
        self.panning = False
        self.pan_start = None
    
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
        
        # Collect all points from all scans in buffer
        all_x = []
        all_y = []
        all_ranges = []
        
        for timestamp, offset, angles, ranges in self.scan_buffer:
            if len(ranges) == 0:
                continue
            
            # Convert polar to cartesian with same transform as visualizer
            # Rotate by 90° so sensor 0° appears at top, negate X for clockwise
            adjusted_angles = np.pi/2 - angles
            x = ranges * np.cos(adjusted_angles)
            y = ranges * np.sin(adjusted_angles)
            
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
        
        self.title.set_text(
            f'Ground Scanning Map - Polar View (Accumulated Scans)\n'
            f'Scans: {total_scans} | Points: {total_points} | '
            f'Distance: {distance_traveled:.2f}m | Time: {time_span:.1f}s | '
            f'Velocity: {self.drone_velocity_y:.2f}m/s'
        )
        
        # Save map if enabled
        if self.save_map and self.save_counter % 100 == 0:  # Save every 100 frames
            self.save_map_image()
        self.save_counter += 1
        
        return self.scatter,
    
    def save_map_image(self):
        """Save current map as image file"""
        try:
            # Create output directory if it doesn't exist
            if not os.path.exists(self.map_output_dir):
                os.makedirs(self.map_output_dir)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'ground_map_{timestamp}.png'
            filepath = os.path.join(self.map_output_dir, filename)
            
            # Save figure
            self.fig.savefig(filepath, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Map saved to: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {str(e)}')
    
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
