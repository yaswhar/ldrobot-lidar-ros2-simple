#!/usr/bin/env python3
"""
LDLidar Visualization Node
A lightweight, interactive polar plot visualization for LaserScan data.
Designed for headless/low-resource systems like Raspberry Pi 4.

Features:
- Real-time polar plot of laser scan data
- Color-coded by range (closer = red, farther = blue)
- Interactive: pan with left-click, zoom with scroll wheel
- Auto-adjusts to lidar parameters (range and angle limits)
- Adds 5% margin for better visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive display
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import math

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('ldlidar_visualizer')
        
        # Declare parameters
        self.declare_parameter('lidar.scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('point_size', 5.0)  # Smaller points for better resolution
        self.declare_parameter('colormap', 'jet_r')  # jet_r = red (close) to blue (far)
        
        # Get parameters
        scan_topic = self.get_parameter('lidar.scan_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        self.point_size = self.get_parameter('point_size').value
        self.colormap = self.get_parameter('colormap').value
        
        # Initialize data storage
        self.latest_scan = None
        self.scan_params = {
            'angle_min': 0.0,
            'angle_max': 2 * np.pi,
            'range_min': 0.0,
            'range_max': 12.0
        }
        
        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.get_logger().info(f'LDLidar Visualizer started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info('Controls: Left-click point=show data, Left-click empty=clear, Right-click drag=pan, Scroll=zoom')
        
        # Setup matplotlib figure
        self.setup_plot()
        
    def scan_callback(self, msg):
        """Store the latest scan data"""
        self.latest_scan = msg
        
        # Update scan parameters from first message
        if self.scan_params['angle_min'] == 0.0 and self.scan_params['angle_max'] == 2 * np.pi:
            self.scan_params['angle_min'] = msg.angle_min
            self.scan_params['angle_max'] = msg.angle_max
            self.scan_params['range_min'] = msg.range_min
            self.scan_params['range_max'] = msg.range_max
            self.get_logger().info(
                f'Scan parameters detected: '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m, '
                f'angle=[{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}]°'
            )
            # Update plot limits with 5% margin
            self.update_plot_limits()
    
    def setup_plot(self):
        """Setup the matplotlib cartesian plot with polar grid"""
        # Create figure and regular axis with tighter layout
        self.fig = plt.figure(figsize=(9, 9))
        self.ax = self.fig.add_subplot(111)
        
        # Set equal aspect ratio for proper circular display
        self.ax.set_aspect('equal')
        
        # Use tight layout to reduce empty space
        self.fig.tight_layout(pad=2.0)
        
        # Initial plot limits (will be updated when first scan arrives)
        max_range = 12.0 * 1.05
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        
        # Setup colormap for range visualization
        self.norm = Normalize(vmin=0, vmax=12.0)
        self.cmap = plt.get_cmap(self.colormap)
        self.sm = ScalarMappable(norm=self.norm, cmap=self.cmap)
        
        # Add colorbar with smaller padding
        self.cbar = plt.colorbar(self.sm, ax=self.ax, pad=0.05, shrink=0.9)
        self.cbar.set_label('Range (m)', rotation=270, labelpad=15)
        
        # Initialize scatter plot with picker enabled for click detection
        self.scatter = self.ax.scatter([], [], c=[], s=self.point_size, 
                                       cmap=self.cmap, norm=self.norm,
                                       alpha=0.8, edgecolors='none',
                                       picker=True)  # Enable picking for click events
        
        # Add title with less padding
        self.title = self.ax.set_title('LDLidar Visualization\nWaiting for data...', 
                                       pad=10, fontsize=12, fontweight='bold')
        
        # Draw polar grid
        self.draw_polar_grid(max_range)
        
        # Labels
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        
        # Enable interactive features
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)  # Use pick_event for point selection
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)  # Handle clicks (left=clear, right=pan)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)  # Handle release for panning
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)  # Handle mouse motion for panning
        
        # Store latest scan data for display
        self.latest_x = np.array([])
        self.latest_y = np.array([])
        self.latest_ranges = np.array([])
        self.latest_angles = np.array([])
        
        # Flag to track if a pick event occurred
        self.point_was_picked = False
        
        # Pan state
        self.panning = False
        self.pan_start = None
        
    def draw_polar_grid(self, max_range):
        """Draw polar grid lines (circles and radial lines)"""
        # Draw concentric circles
        num_circles = 6
        for i in range(1, num_circles + 1):
            r = (max_range / num_circles) * i
            circles = plt.Circle((0, 0), r, fill=False, color='gray',
                               alpha=0.3, linewidth=0.5, linestyle='--')
            self.ax.add_patch(circles)
            # Add range labels
            self.ax.text(0, r, f'{r:.1f}m', ha='center', va='bottom', 
                        fontsize=8, color='gray', alpha=0.7)
        
        # Draw radial lines (every 30 degrees) within sensor angle range
        # Rotate grid by 90° so 0° is at top, and negate X for clockwise rotation
        angle_min_deg = math.degrees(self.scan_params['angle_min'])
        angle_max_deg = math.degrees(self.scan_params['angle_max'])

        # Find the first 30° increment within or after angle_min
        start_angle = int(np.ceil(angle_min_deg / 30.0)) * 30
        
        # Generate angles from start to angle_max in 30° steps
        sensor_angle_deg = start_angle
        while sensor_angle_deg <= angle_max_deg:
            # Transform: rotate by 90° and negate for clockwise
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
        
        # Also draw the boundary lines at exact min/max angles if they don't coincide with grid lines
        for boundary_angle_deg in [angle_min_deg, angle_max_deg]:
            display_angle_rad = np.radians(90 - boundary_angle_deg)
            x = [0, max_range * np.cos(display_angle_rad)]
            y = [0, max_range * np.sin(display_angle_rad)]
            self.ax.plot(x, y, color='red', alpha=0.5, linewidth=1.0, linestyle='-')
        
        # Draw origin marker
        self.ax.plot(0, 0, 'k+', markersize=10, markeredgewidth=2)
        
    def update_plot_limits(self):
        """Update plot limits based on scan parameters with 5% margin"""
        margin = 0.05
        
        # Range limit with margin
        range_max = self.scan_params['range_max'] * (1.0 + margin)
        self.ax.set_xlim(-range_max, range_max)
        self.ax.set_ylim(-range_max, range_max)
        
        # Redraw polar grid with new limits
        self.ax.clear()
        self.draw_polar_grid(range_max)
        
        # Recreate scatter plot after clearing with picker enabled
        self.scatter = self.ax.scatter([], [], c=[], s=self.point_size, 
                                       cmap=self.cmap, norm=self.norm,
                                       alpha=0.8, edgecolors='none',
                                       picker=True)  # Enable picking
        
        # Update colormap range
        self.norm = Normalize(vmin=self.scan_params['range_min'], 
                             vmax=self.scan_params['range_max'])
        self.sm.set_norm(self.norm)
        self.scatter.set_norm(self.norm)
        self.cbar.update_normal(self.sm)
        
    def update_plot(self, frame):
        """Update the plot with latest scan data"""
        if self.latest_scan is None:
            return self.scatter,
        
        scan = self.latest_scan
        
        # Extract angles and ranges
        num_points = len(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_points)
        ranges = np.array(scan.ranges)
        
        # Filter out invalid points (NaN, inf, out of range)
        valid_mask = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        valid_angles = angles[valid_mask]
        valid_ranges = ranges[valid_mask]
        
        # Convert polar to cartesian coordinates
        # Rotate by 90° so sensor 0° appears at top, and negate X for clockwise (mirror)
        # This makes: 0°=top, 90°=right, 180°=bottom, 270°=left (clockwise)
        adjusted_angles = np.pi/2 - valid_angles  # 90° rotation and negate for CW
        x = valid_ranges * np.cos(adjusted_angles)
        y = valid_ranges * np.sin(adjusted_angles)
        
        # Store for click functionality
        self.latest_x = x
        self.latest_y = y
        self.latest_ranges = valid_ranges
        self.latest_angles = np.degrees(valid_angles)  # Store in degrees for display
        
        # Update scatter plot
        if len(x) > 0:
            self.scatter.set_offsets(np.c_[x, y])
            self.scatter.set_array(valid_ranges)
            
            # Update title with stats
            self.title.set_text(
                f'LDLidar Visualization\n'
                f'Points: {len(valid_ranges)}/{num_points} | '
                f'Min: {np.min(valid_ranges):.2f}m | '
                f'Max: {np.max(valid_ranges):.2f}m'
            )
        else:
            self.scatter.set_offsets(np.empty((0, 2)))
            self.title.set_text('LDLidar Visualization\nNo valid data')
        
        return self.scatter,
    
    def on_scroll(self, event):
        """Handle zoom with scroll wheel"""
        if event.inaxes != self.ax:
            return
        
        # Get current limits
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        
        # Zoom factor
        zoom_factor = 1.2 if event.button == 'down' else 0.8
        
        # Calculate new limits centered on current view
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
    
    def on_pick(self, event):
        """Handle pick event when a point is clicked"""
        # Set flag to indicate a point was picked
        self.point_was_picked = True
        
        # Check if the picked artist is our scatter plot
        if event.artist == self.scatter:
            ind = event.ind[0]  # Get the index of the clicked point
            x_data = self.latest_x[ind]
            y_data = self.latest_y[ind]
            range_data = self.latest_ranges[ind]
            angle_data = self.latest_angles[ind]
            
            label_text = f'({range_data:.2f}m, {angle_data:.1f}°)'
            
            # Remove previous annotations if any
            for ann in self.ax.findobj(lambda x: isinstance(x, plt.Annotation)):
                ann.remove()
            
            # Add a new annotation
            self.ax.annotate(label_text, (x_data, y_data),
                            xytext=(20, 20), textcoords='offset points',
                            bbox=dict(boxstyle="round,pad=0.5", fc="yellow", alpha=0.9),
                            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.2'))
            self.fig.canvas.draw_idle()  # Redraw the canvas
    
    def on_click(self, event):
        """Handle click events - left click clears annotations, right click starts panning"""
        if event.inaxes != self.ax:
            return
        
        # Left click - clear annotations if no point was picked
        if event.button == 1:
            # Check if a pick event happened (flag is set by on_pick which fires first)
            if not self.point_was_picked:
                # Remove all annotations
                for ann in self.ax.findobj(lambda x: isinstance(x, plt.Annotation)):
                    ann.remove()
                self.fig.canvas.draw_idle()
            
            # Reset the flag for next click
            self.point_was_picked = False
        
        # Right click - start panning
        elif event.button == 3:
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
        
        # Update limits (subtract because we're moving the view, not the data)
        self.ax.set_xlim(x_min - dx, x_max - dx)
        self.ax.set_ylim(y_min - dy, y_max - dy)
        
        # Update pan start to current position (in the new coordinate system)
        self.pan_start = (event.xdata - dx, event.ydata - dy)
        
        self.fig.canvas.draw_idle()
    
    def run(self):
        """Run the visualization"""
        # Setup animation
        interval_ms = int(1000.0 / self.update_rate)
        # Use full redraws (blit=False) so annotation show/hide works reliably
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
    
    visualizer = LidarVisualizer()
    
    try:
        # Run the visualizer in a separate thread
        import threading
        ros_thread = threading.Thread(target=lambda: rclpy.spin(visualizer), daemon=True)
        ros_thread.start()
        
        # Run the matplotlib event loop (blocking)
        visualizer.run()
        
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
