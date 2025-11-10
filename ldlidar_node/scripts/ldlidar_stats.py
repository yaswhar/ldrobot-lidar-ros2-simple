#!/usr/bin/env python3
"""
LDLidar Statistics Publisher Node

Analyzes laser scan data from LDLidar and computes statistics:
- Minimum distance and its corresponding angle
- Maximum distance and its corresponding angle
- Average distance across all valid points

The statistics are published to /ldlidar_stats topic and logged to terminal.

Author: Auto-generated for Raspberry Pi LDLidar ROS2 project
License: Apache License 2.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import math
from datetime import datetime
import os


class LidarStatsPublisher(Node):
    def __init__(self):
        super().__init__('ldlidar_stats_publisher')
        
        # Declare parameters
        self.declare_parameter('lidar.scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('update_rate', 1.0)  # Hz, default 1 second
        
        # Get parameters
        scan_topic = self.get_parameter('lidar.scan_topic').value
        self.update_rate = self.get_parameter('update_rate').value

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
        
        # Create publisher for statistics
        self.stats_publisher = self.create_publisher(
            String,
            '/ldlidar_stats',
            10)
        
        # Create timer for statistics computation
        timer_period = 1.0 / self.update_rate  # seconds
        self.timer = self.create_timer(timer_period, self.compute_and_publish_stats)
        
        self.get_logger().info('LDLidar Statistics Publisher started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing to: /ldlidar_stats')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        
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
    
    def compute_and_publish_stats(self):
        """Compute statistics and publish to topic"""
        if self.latest_scan is None:
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
        
        # Publish to topic
        msg = String()
        msg.data = output_str
        self.stats_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    stats_publisher = LidarStatsPublisher()
    
    try:
        rclpy.spin(stats_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        stats_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
