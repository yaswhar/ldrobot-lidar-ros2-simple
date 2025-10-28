#!/usr/bin/env python3
"""
LDLidar Statistics Analyzer Node

Analyzes laser scan data from LDLidar and computes statistics:
- Minimum distance and its corresponding angle
- Maximum distance and its corresponding angle
- Average distance across all valid points

The statistics are logged to terminal and written to a timestamped text file
at a configurable rate.

Author: Auto-generated for Raspberry Pi LDLidar ROS2 project
License: Apache License 2.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from datetime import datetime
import os


class LidarStatsAnalyzer(Node):
    def __init__(self):
        super().__init__('ldlidar_stats_analyzer')
        
        # Declare parameters
        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('update_rate', 1.0)  # Hz, default 1 second
        self.declare_parameter('log_file_path', os.path.expanduser('~/Desktop'))
        
        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        # Expand user (~) in provided path to get an absolute directory
        log_file_path = os.path.expanduser(self.get_parameter('log_file_path').value)

        # Initialize data storage
        self.latest_scan = None
        self.row_number = 0
        # Track whether we've logged receipt of the first scan
        self._first_scan_received = False
        
        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        # Create output file with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Create and initialize the log file (with a safe fallback)
        try:
            os.makedirs(log_file_path, exist_ok=True)
        except Exception as e:
            self.get_logger().warn(f'Could not create directory "{log_file_path}": {e}. Falling back to /tmp')
            log_file_path = '/tmp'

        self.log_filename = os.path.join(log_file_path, f'ldlidar_stats_{timestamp}.txt')
        try:
            with open(self.log_filename, 'w') as f:
                f.write('LDLidar Statistics Log\n')
                f.write(f'Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write('Format: Row - min: (dist, angle) - max: (dist, angle) - average distance\n')
                f.write('='*80 + '\n')
            self.get_logger().info(f'Log file created: {self.log_filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to create log file "{self.log_filename}": {e}. Disabling file logging.')
            self.log_filename = None
        
        # Create timer for statistics computation
        timer_period = 1.0 / self.update_rate  # seconds
        self.timer = self.create_timer(timer_period, self.compute_and_log_stats)
        
        self.get_logger().info('LDLidar Statistics Analyzer started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Log file: {self.log_filename}')
        
    def scan_callback(self, msg):
        """Store the latest scan data"""
        self.latest_scan = msg
        if not self._first_scan_received:
            self._first_scan_received = True
            try:
                num_points = len(msg.ranges)
            except Exception:
                num_points = -1
            self.get_logger().info(f'Received first scan: points={num_points}, angle_min={math.degrees(msg.angle_min):.1f}°, angle_max={math.degrees(msg.angle_max):.1f}°')
    
    def compute_and_log_stats(self):
        """Compute statistics and log to terminal and file"""
        if self.latest_scan is None:
            self.get_logger().warn('No scan data received yet', throttle_duration_sec=5.0)
            return
        
        scan = self.latest_scan
        
        # Extract angles and ranges
        num_points = len(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_points)
        ranges = np.array(scan.ranges)
        
        # Filter out invalid points (NaN, inf, out of range)
        valid_mask = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        valid_angles = angles[valid_mask]
        valid_ranges = ranges[valid_mask]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn('No valid scan points', throttle_duration_sec=5.0)
            return
        
        # Compute statistics
        min_idx = np.argmin(valid_ranges)
        max_idx = np.argmax(valid_ranges)
        
        min_dist = valid_ranges[min_idx]
        min_angle = math.degrees(valid_angles[min_idx])
        
        max_dist = valid_ranges[max_idx]
        max_angle = math.degrees(valid_angles[max_idx])
        
        avg_dist = np.mean(valid_ranges)
        
        # Increment row number
        self.row_number += 1
        
        # Format the output string
        output_str = (
            f'{self.row_number} - '
            f'min: ({min_dist:.2f}m, {min_angle:.1f}°) - '
            f'max: ({max_dist:.2f}m, {max_angle:.1f}°) - '
            f'avg: {avg_dist:.2f}m'
        )
        
        # Log to terminal
        self.get_logger().info(output_str)
        
        # Write to file
        if self.log_filename is not None:
            try:
                with open(self.log_filename, 'a') as f:
                    f.write(output_str + '\n')
                    try:
                        f.flush()
                        os.fsync(f.fileno())
                    except Exception:
                        # If fsync is unavailable or fails, that's non-fatal
                        pass
            except Exception as e:
                self.get_logger().error(f'Failed to write to log file: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    stats_analyzer = LidarStatsAnalyzer()
    
    try:
        rclpy.spin(stats_analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        # Write closing message to log file
        if stats_analyzer.log_filename is not None:
            try:
                with open(stats_analyzer.log_filename, 'a') as f:
                    f.write('='*80 + '\n')
                    f.write(f'Ended: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                stats_analyzer.get_logger().info(f'Log file closed: {stats_analyzer.log_filename}')
            except Exception as e:
                stats_analyzer.get_logger().error(f'Failed to close log file: {e}')
        
        stats_analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
