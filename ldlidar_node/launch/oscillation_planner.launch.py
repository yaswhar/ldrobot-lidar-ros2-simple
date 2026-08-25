#!/usr/bin/env python3
"""
Launch the oscillation_planner_node (MOTION-PLANNING LAYER only).

Independently launchable. The motion is a continuous elliptical move+tilt loop,
repeated for the laps configured in oscillation_planner.yaml (lap_durations_s),
each lap independently timed. To change speeds/laps, edit the YAML or pass a
different params file:

    ros2 launch ldlidar_node oscillation_planner.launch.py
    ros2 launch ldlidar_node oscillation_planner.launch.py \
        params_file:=/mnt/host_desktop/oscillation_planner_param.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ldlidar_node')
    default_params = os.path.join(pkg_dir, 'params', 'oscillation_planner.yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the oscillation_planner params YAML')

    params_file = LaunchConfiguration('params_file')

    planner_node = Node(
        package='ldlidar_node',
        executable='oscillation_planner_node.py',
        name='oscillation_planner',
        output='screen',
        parameters=[params_file],
        ros_arguments=['--log-level', 'info'])

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(planner_node)
    return ld
