#!/usr/bin/env python3
"""
LDLidar Statistics Logger Node

Subscribes to /ldlidar_stats topic and logs statistics to:
- Terminal (for real-time visibility)
- Text file (timestamped, persisted to disk)

This node works in tandem with ldlidar_stats.py (publisher).

Author: Auto-generated for Raspberry Pi LDLidar ROS2 project
License: Apache License 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import os
import sys


class LidarStatsLogger(Node):
    def __init__(self):
        super().__init__('ldlidar_stats_logger')
        
        # Declare parameters
        self.declare_parameter('log_file_path', '~/shahrokhi')
        
        # Get parameters and expand user path
        log_file_path = os.path.expanduser(self.get_parameter('log_file_path').value)
        
        # Create log directory
        try:
            os.makedirs(log_file_path, exist_ok=True)
            print(f'Log directory: {log_file_path}')
        except Exception as e:
            print(f'Could not create directory "{log_file_path}": {e}. Falling back to /tmp')
            log_file_path = '/tmp'
        
        # Create timestamped log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(log_file_path, f'ldlidar_stats_{timestamp}.txt')
        
        try:
            with open(self.log_filename, 'w') as f:
                f.write('LDLidar Statistics Log\n')
                f.write(f'Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write('Format: Row - min: (dist, angle) - max: (dist, angle) - avg: dist\n')
                f.write('='*80 + '\n')
            print(f'Log file: {self.log_filename}')
        except Exception as e:
            print(f'Failed to create log file: {e}')
            self.log_filename = None
        
        # Print header to terminal
        print('='*80)
        print('LDLidar Statistics Logger')
        print(f'Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        print('Format: Row - min: (dist, angle) - max: (dist, angle) - avg: dist')
        print('='*80)
        sys.stdout.flush()
        
        # Subscribe to statistics topic
        self.subscription = self.create_subscription(
            String,
            '/ldlidar_stats',
            self.stats_callback,
            10)
    
    def stats_callback(self, msg):
        """Log received statistics to terminal and file"""
        stats_data = msg.data
        
        # Print to terminal (clean output, no ROS logger formatting)
        print(stats_data)
        sys.stdout.flush()
        
        # Log to file
        if self.log_filename is not None:
            try:
                with open(self.log_filename, 'a') as f:
                    f.write(stats_data + '\n')
                    f.flush()
                    os.fsync(f.fileno())
            except Exception as e:
                print(f'Error writing to log file: {e}', file=sys.stderr)


def main(args=None):
    rclpy.init(args=args)
    
    logger = LidarStatsLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        # Write closing message
        print('='*80)
        print(f'Ended: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        
        # Write closing message to log file
        if logger.log_filename is not None:
            try:
                with open(logger.log_filename, 'a') as f:
                    f.write('='*80 + '\n')
                    f.write(f'Ended: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                print(f'Log file closed: {logger.log_filename}')
            except Exception as e:
                print(f'Failed to close log file: {e}', file=sys.stderr)
        
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
