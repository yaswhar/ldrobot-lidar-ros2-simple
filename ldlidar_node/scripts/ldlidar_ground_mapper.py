#!/usr/bin/env python3
"""
LDLidar Ground Mapper Node
Generates a 2D topological map by stacking lidar scans in the Y-direction
based on drone velocity, simulating a ground scanner.

Features:
- Subscribes to LaserScan data
- Stacks scans based on configurable Y-axis velocity
- Real-time scrolling 2D map visualization
- Configurable map buffer time and resolution
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
        self.declare_parameter('drone_velocity_y', 1.0)  # m/s in Y-direction
        self.declare_parameter('map_buffer_time', 10.0)  # seconds
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('colormap', 'viridis')  # matplotlib colormap
        self.declare_parameter('save_map', False)  # auto-save map images
        self.declare_parameter('map_output_dir', '/workspace/maps')
        
        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.drone_velocity_y = self.get_parameter('drone_velocity_y').value
        self.map_buffer_time = self.get_parameter('map_buffer_time').value
        self.update_rate = self.get_parameter('update_rate').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.colormap = self.get_parameter('colormap').value
        self.save_map = self.get_parameter('save_map').value
        self.map_output_dir = self.get_parameter('map_output_dir').value
        
        # Initialize data storage
        self.scan_buffer = deque()  # [(timestamp, y_offset, scan_msg)]
        self.y_offset = 0.0  # Current Y position based on velocity
        self.last_timestamp = None
        self.first_scan_received = False
        
        # Scan parameters
        self.scan_params = {
            'angle_min': 0.0,
            'angle_max': 2 * np.pi,
            'range_min': 0.0,
            'range_max': 12.0
        }
        
        # Map grid
        self.map_grid = None
        self.grid_extent = None  # [x_min, x_max, y_min, y_max]
        
        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.get_logger().info(f'Ground Mapper started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Drone velocity: {self.drone_velocity_y} m/s')
        self.get_logger().info(f'Map buffer: {self.map_buffer_time} seconds')
        self.get_logger().info(f'Map resolution: {self.map_resolution} m/pixel')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        
        # Setup matplotlib figure
        self.setup_plot()
        
        # Save counter
        self.save_counter = 0
        
    def scan_callback(self, msg):
        """Store scan with timestamp and calculate Y-offset based on velocity"""
        # Get timestamp in seconds
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Update scan parameters from first message
        if not self.first_scan_received:
            self.scan_params['angle_min'] = msg.angle_min
            self.scan_params['angle_max'] = msg.angle_max
            self.scan_params['range_min'] = msg.range_min
            self.scan_params['range_max'] = msg.range_max
            self.last_timestamp = current_timestamp
            self.first_scan_received = True
            self.get_logger().info(
                f'First scan received: '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m, '
                f'angle=[{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}]°'
            )
        
        # Calculate time delta
        if self.last_timestamp is not None:
            delta_t = current_timestamp - self.last_timestamp
            # Update Y-offset based on velocity
            delta_y = self.drone_velocity_y * delta_t
            self.y_offset += delta_y
        
        # Store scan with timestamp and y_offset
        self.scan_buffer.append((current_timestamp, self.y_offset, msg))
        
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
        """Setup the matplotlib plot for 2D map visualization"""
        # Create figure
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111)
        
        # Use tight layout
        self.fig.tight_layout(pad=2.0)
        
        # Setup colormap for range visualization
        self.norm = Normalize(vmin=0, vmax=12.0)
        self.cmap = plt.get_cmap(self.colormap)
        self.sm = ScalarMappable(norm=self.norm, cmap=self.cmap)
        
        # Add colorbar
        self.cbar = plt.colorbar(self.sm, ax=self.ax, pad=0.02, shrink=0.9)
        self.cbar.set_label('Range (m)', rotation=270, labelpad=20)
        
        # Initialize image plot
        # Start with empty map
        empty_map = np.zeros((100, 100))
        self.image = self.ax.imshow(
            empty_map,
            cmap=self.cmap,
            norm=self.norm,
            origin='lower',
            aspect='auto',
            interpolation='nearest',
            extent=[-6, 6, 0, 10]  # Initial extent [x_min, x_max, y_min, y_max]
        )
        
        # Labels and title
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position - Flight Direction (m)', fontsize=12)
        self.title = self.ax.set_title(
            'Ground Scanning Topological Map\nWaiting for data...',
            pad=10, fontsize=12, fontweight='bold'
        )
        
        # Add grid
        self.ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        
        # Enable interactive features
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        
        # Pan state
        self.panning = False
        self.pan_start = None
        
    def generate_map(self):
        """Generate 2D map from scan buffer"""
        if not self.scan_buffer:
            return None, None
        
        # Collect all valid points
        all_x = []
        all_y = []
        all_ranges = []
        
        for timestamp, y_offset, scan in self.scan_buffer:
            num_points = len(scan.ranges)
            angles = np.linspace(scan.angle_min, scan.angle_max, num_points)
            ranges = np.array(scan.ranges)
            
            # Filter valid points
            valid_mask = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
            valid_angles = angles[valid_mask]
            valid_ranges = ranges[valid_mask]
            
            if len(valid_ranges) == 0:
                continue
            
            # Convert polar to cartesian
            # X: perpendicular to flight direction (scan width)
            # Y: along flight direction (stacked with offset)
            x_local = valid_ranges * np.cos(valid_angles)
            y_local = valid_ranges * np.sin(valid_angles)
            
            # Apply Y-offset for stacking
            y_global = y_offset + y_local
            
            all_x.extend(x_local)
            all_y.extend(y_global)
            all_ranges.extend(valid_ranges)
        
        if not all_x:
            return None, None
        
        all_x = np.array(all_x)
        all_y = np.array(all_y)
        all_ranges = np.array(all_ranges)
        
        # Calculate map bounds
        x_min, x_max = np.min(all_x), np.max(all_x)
        y_min, y_max = np.min(all_y), np.max(all_y)
        
        # Add small margin
        x_margin = (x_max - x_min) * 0.05 if x_max > x_min else 0.5
        y_margin = (y_max - y_min) * 0.05 if y_max > y_min else 0.5
        
        x_min -= x_margin
        x_max += x_margin
        y_min -= y_margin
        y_max += y_margin
        
        # Calculate grid dimensions
        grid_width = int(np.ceil((x_max - x_min) / self.map_resolution))
        grid_height = int(np.ceil((y_max - y_min) / self.map_resolution))
        
        # Limit grid size to prevent memory issues
        max_grid_size = 2000
        if grid_width > max_grid_size or grid_height > max_grid_size:
            scale = max(grid_width / max_grid_size, grid_height / max_grid_size)
            grid_width = int(grid_width / scale)
            grid_height = int(grid_height / scale)
            self.get_logger().warn(f'Grid size limited to {grid_width}x{grid_height}')
        
        # Create empty grid
        map_grid = np.full((grid_height, grid_width), np.nan)
        
        # Populate grid
        for i in range(len(all_x)):
            x = all_x[i]
            y = all_y[i]
            r = all_ranges[i]
            
            # Convert to grid coordinates
            grid_x = int((x - x_min) / self.map_resolution)
            grid_y = int((y - y_min) / self.map_resolution)
            
            # Check bounds
            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                # Use minimum distance if multiple points fall in same cell
                if np.isnan(map_grid[grid_y, grid_x]):
                    map_grid[grid_y, grid_x] = r
                else:
                    map_grid[grid_y, grid_x] = min(map_grid[grid_y, grid_x], r)
        
        # Grid extent for imshow
        extent = [x_min, x_max, y_min, y_max]
        
        return map_grid, extent
    
    def update_plot(self, frame):
        """Update the plot with latest map data"""
        if not self.first_scan_received or not self.scan_buffer:
            return self.image,
        
        # Generate map
        map_grid, extent = self.generate_map()
        
        if map_grid is None:
            return self.image,
        
        # Update image
        self.image.set_data(map_grid)
        self.image.set_extent(extent)
        
        # Update normalization range
        valid_data = map_grid[~np.isnan(map_grid)]
        if len(valid_data) > 0:
            data_min = np.min(valid_data)
            data_max = np.max(valid_data)
            self.norm = Normalize(vmin=data_min, vmax=data_max)
            self.image.set_norm(self.norm)
            self.sm.set_norm(self.norm)
            self.cbar.update_normal(self.sm)
        
        # Auto-adjust axis limits to show all data
        self.ax.set_xlim(extent[0], extent[1])
        self.ax.set_ylim(extent[2], extent[3])
        
        # Update title with stats
        total_scans = len(self.scan_buffer)
        total_points = np.sum(~np.isnan(map_grid))
        y_travel = self.y_offset
        
        self.title.set_text(
            f'Ground Scanning Topological Map\n'
            f'Scans: {total_scans} | Points: {total_points} | '
            f'Y-Travel: {y_travel:.2f}m | Velocity: {self.drone_velocity_y:.2f}m/s'
        )
        
        # Save map if enabled
        if self.save_map and self.save_counter % 100 == 0:  # Save every 100 frames
            self.save_map_image()
        self.save_counter += 1
        
        return self.image,
    
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
        
        # Calculate new limits centered on mouse position
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
