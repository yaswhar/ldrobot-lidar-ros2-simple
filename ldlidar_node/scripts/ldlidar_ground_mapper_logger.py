#!/usr/bin/env python3
"""
LDLidar Ground Mapper Logger Node

Logs complete point cloud data from LaserScan topic for topological mapping.
Saves position-indexed scan data with all points (distance, angle) for later map reconstruction.

Log Format:
- Header: Configuration parameters (drone velocity, scan rates, angle crops, etc.)
- Body: Position-indexed scans with all valid points (distance, angle pairs)

This logger is designed to work with the ground mapper visualization,
providing complete data for offline topological map generation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import os
from datetime import datetime
import math


class GroundMapperLogger(Node):
    def __init__(self):
        super().__init__('ldlidar_ground_mapper_logger')
        
        # Declare parameters - read from shared /** namespace
        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('drone_velocity', 1.0)
        self.declare_parameter('lidar.angle_crop_min', 0.0)
        self.declare_parameter('lidar.angle_crop_max', 360.0)
        self.declare_parameter('lidar.range_max', 12.0)
        
        # Node-specific parameters from ground_mapper_logger namespace
        self.declare_parameter('log_output_dir', '~/shahrokhi/logs')
        self.declare_parameter('log_interval', 0.1)
        
        # Get shared parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.drone_velocity = self.get_parameter('drone_velocity').value
        self.angle_crop_min = self.get_parameter('lidar.angle_crop_min').value
        self.angle_crop_max = self.get_parameter('lidar.angle_crop_max').value
        self.range_max = self.get_parameter('lidar.range_max').value
        
        # Get node-specific parameters
        log_output_dir = self.get_parameter('log_output_dir').value
        self.log_interval = self.get_parameter('log_interval').value
        
        # Initialize tracking variables
        self.offset_distance = 0.0  # Current offset distance based on velocity
        self.last_timestamp = None
        self.first_scan_received = False
        self.last_log_time = None
        
        # Scan parameters (will be updated from first message)
        self.scan_params = {
            'angle_min': 0.0,
            'angle_max': 2 * np.pi,
            'range_min': 0.0,
            'range_max': 12.0,
            'scan_time': 0.0,
            'angle_increment': 0.0
        }
        
        # Setup log file
        self.log_file = None
        self.setup_log_file(log_output_dir)
        
        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.get_logger().info(f'Ground Mapper Logger started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Drone velocity: {self.drone_velocity} m/s')
        self.get_logger().info(f'Log interval: {self.log_interval} s ({1.0/self.log_interval:.1f} Hz)')
        self.get_logger().info(f'Log file: {self.log_file}')
        
    def setup_log_file(self, log_output_dir):
        """Create log file with header"""
        try:
            # Expand user path and create output directory if it doesn't exist
            output_dir = os.path.expanduser(log_output_dir)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                self.get_logger().info(f'Created directory: {output_dir}')
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'ground_mapper_log_{timestamp}.txt'
            self.log_file = os.path.join(output_dir, filename)
            
            # Create file with header
            with open(self.log_file, 'w') as f:
                f.write('=' * 80 + '\n')
                f.write('LDLidar Ground Mapper Data Log\n')
                f.write('=' * 80 + '\n')
                f.write(f'Log Created: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write(f'Drone Velocity: {self.drone_velocity} m/s\n')
                f.write(f'Log Interval: {self.log_interval} s ({1.0/self.log_interval:.1f} Hz)\n')
                f.write('\nNote: Scan parameters will be updated after first scan is received.\n')
                f.write('=' * 80 + '\n\n')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create log file: {str(e)}')
            self.log_file = None
    
    def write_scan_params_to_log(self):
        """Write scan parameters to log file (called after first scan)"""
        if self.log_file is None:
            return
        
        try:
            with open(self.log_file, 'a') as f:
                f.write('SCAN PARAMETERS\n')
                f.write('-' * 80 + '\n')
                f.write(f'Angle Range: [{math.degrees(self.scan_params["angle_min"]):.2f}°, '
                       f'{math.degrees(self.scan_params["angle_max"]):.2f}°]\n')
                f.write(f'Angle Crop: [{self.angle_crop_min:.2f}°, {self.angle_crop_max:.2f}°]\n')
                f.write(f'Angle Increment: {math.degrees(self.scan_params["angle_increment"]):.4f}°\n')
                f.write(f'Range Limits: [{self.scan_params["range_min"]:.3f}m, '
                       f'{self.scan_params["range_max"]:.3f}m]\n')
                f.write(f'Range Max (Config): {self.range_max:.3f}m\n')
                f.write(f'Scan Time: {self.scan_params["scan_time"]:.4f}s '
                       f'({1.0/self.scan_params["scan_time"]:.1f} Hz)\n')
                f.write('-' * 80 + '\n\n')
                
                f.write('DATA FORMAT\n')
                f.write('-' * 80 + '\n')
                f.write('Each scan entry format:\n')
                f.write('  POSITION: x=<distance_traveled_in_meters>\n')
                f.write('  TIMESTAMP: <seconds_since_start>\n')
                f.write('  POINTS: <count>\n')
                f.write('  <distance_m> <angle_deg>\n')
                f.write('  <distance_m> <angle_deg>\n')
                f.write('  ...\n')
                f.write('-' * 80 + '\n\n')
                
                f.write('BEGIN DATA\n')
                f.write('=' * 80 + '\n\n')
                f.flush()
                
        except Exception as e:
            self.get_logger().error(f'Failed to write scan parameters: {str(e)}')
    
    def scan_callback(self, msg):
        """Process scan and log data at specified interval"""
        # Get timestamp in seconds
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Update scan parameters from first message
        if not self.first_scan_received:
            self.scan_params['angle_min'] = msg.angle_min
            self.scan_params['angle_max'] = msg.angle_max
            self.scan_params['range_min'] = msg.range_min
            self.scan_params['range_max'] = msg.range_max
            self.scan_params['scan_time'] = msg.scan_time if msg.scan_time > 0 else 0.1
            self.scan_params['angle_increment'] = msg.angle_increment
            self.last_timestamp = current_timestamp
            self.last_log_time = current_timestamp
            self.first_scan_received = True
            
            # Write scan parameters to log
            self.write_scan_params_to_log()
            
            self.get_logger().info(
                f'First scan received: '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m, '
                f'angle=[{math.degrees(msg.angle_min):.1f}°, {math.degrees(msg.angle_max):.1f}°]'
            )
        
        # Calculate time delta and update offset
        if self.last_timestamp is not None:
            delta_t = current_timestamp - self.last_timestamp
            delta_offset = self.drone_velocity * delta_t
            self.offset_distance += delta_offset
        
        self.last_timestamp = current_timestamp
        
        # Check if it's time to log (based on log_interval)
        if self.last_log_time is None or (current_timestamp - self.last_log_time) >= self.log_interval:
            self.log_scan_data(msg, current_timestamp)
            self.last_log_time = current_timestamp
    
    def log_scan_data(self, msg, timestamp):
        """Log scan data to file"""
        if self.log_file is None:
            return
        
        try:
            # Extract scan data
            num_points = len(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
            ranges = np.array(msg.ranges)
            
            # Filter valid points
            valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            valid_angles = angles[valid_mask]
            valid_ranges = ranges[valid_mask]
            
            # Calculate time since start
            if hasattr(self, 'first_timestamp'):
                time_since_start = timestamp - self.first_timestamp
            else:
                self.first_timestamp = timestamp
                time_since_start = 0.0
            
            # Write to log file
            with open(self.log_file, 'a') as f:
                f.write(f'POSITION: x={self.offset_distance:.4f}m\n')
                f.write(f'TIMESTAMP: {time_since_start:.3f}s\n')
                f.write(f'POINTS: {len(valid_ranges)}\n')
                
                # Write all valid points (distance, angle)
                for distance, angle in zip(valid_ranges, valid_angles):
                    angle_deg = math.degrees(angle)
                    f.write(f'{distance:.4f} {angle_deg:.2f}\n')
                
                f.write('\n')  # Blank line between scans
                f.flush()  # Ensure data is written immediately
                
        except Exception as e:
            self.get_logger().error(f'Failed to log scan data: {str(e)}')
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        if self.log_file is not None:
            try:
                with open(self.log_file, 'a') as f:
                    f.write('=' * 80 + '\n')
                    f.write('END DATA\n')
                    f.write('=' * 80 + '\n')
                    f.write(f'Log Ended: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                    f.write(f'Total Distance Traveled: {self.offset_distance:.4f}m\n')
                self.get_logger().info(f'Log file closed: {self.log_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to close log file: {str(e)}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    logger = GroundMapperLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
