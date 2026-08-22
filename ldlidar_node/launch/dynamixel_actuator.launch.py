#!/usr/bin/env python3
"""
Launch the dynamixel_actuator_node (HARDWARE LAYER only).

Independently launchable -- it has NO notion of modes, geometry or IK. It only
listens on /joint_goal and publishes /joint_states.

    ros2 launch ldlidar_node dynamixel_actuator.launch.py
    ros2 launch ldlidar_node dynamixel_actuator.launch.py \
        params_file:=/mnt/host_desktop/dynamixel_actuator_param.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ldlidar_node')
    default_params = os.path.join(pkg_dir, 'params', 'dynamixel_actuator.yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the dynamixel_actuator params YAML')

    params_file = LaunchConfiguration('params_file')

    actuator_node = Node(
        package='ldlidar_node',
        executable='dynamixel_actuator_node.py',
        name='dynamixel_actuator',
        output='screen',
        parameters=[params_file],
        ros_arguments=['--log-level', 'info'])

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(actuator_node)
    return ld
