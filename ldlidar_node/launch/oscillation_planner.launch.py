#!/usr/bin/env python3
"""
Launch the oscillation_planner_node (MOTION-PLANNING LAYER only).

Independently launchable. The 'mode' argument (1|2|3) lives ONLY here -- the
actuator has no notion of modes.

    ros2 launch ldlidar_node oscillation_planner.launch.py mode:=2
    ros2 launch ldlidar_node oscillation_planner.launch.py mode:=3 repeat:=5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ldlidar_node')
    default_params = os.path.join(pkg_dir, 'params', 'oscillation_planner.yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the oscillation_planner params YAML')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode', default_value='2',
        description='Speed mode: 1=slow(6.0s) 2=normal(4.5s) 3=fast(3.0s)')

    declare_repeat_cmd = DeclareLaunchArgument(
        'repeat', default_value='1',
        description='Number of full round trips (1 = single round trip)')

    params_file = LaunchConfiguration('params_file')
    mode = LaunchConfiguration('mode')
    repeat = LaunchConfiguration('repeat')

    planner_node = Node(
        package='ldlidar_node',
        executable='oscillation_planner_node.py',
        name='oscillation_planner',
        output='screen',
        parameters=[params_file, {
            'mode': ParameterValue(mode, value_type=int),
            'repeat': ParameterValue(repeat, value_type=int),
        }],
        ros_arguments=['--log-level', 'info'])

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_repeat_cmd)
    ld.add_action(planner_node)
    return ld
