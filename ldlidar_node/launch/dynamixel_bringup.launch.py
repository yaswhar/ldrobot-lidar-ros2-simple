#!/usr/bin/env python3
"""
Combined bringup: dynamixel_actuator_node + oscillation_planner_node.

Convenience for bench testing. The two nodes remain SEPARATE processes coupled
only by the /joint_goal topic -- this file just starts both. Swapping the
planner for a lidar_planner_node later needs no change to the actuator.

    ros2 launch ldlidar_node dynamixel_bringup.launch.py mode:=2
    ros2 launch ldlidar_node dynamixel_bringup.launch.py mode:=3 repeat:=3 \
        actuator_params_file:=/mnt/host_desktop/dynamixel_actuator_param.yaml \
        planner_params_file:=/mnt/host_desktop/oscillation_planner_param.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ldlidar_node')
    actuator_default = os.path.join(pkg_dir, 'params', 'dynamixel_actuator.yaml')
    planner_default = os.path.join(pkg_dir, 'params', 'oscillation_planner.yaml')

    declare_actuator_params = DeclareLaunchArgument(
        'actuator_params_file', default_value=actuator_default,
        description='dynamixel_actuator params YAML')
    declare_planner_params = DeclareLaunchArgument(
        'planner_params_file', default_value=planner_default,
        description='oscillation_planner params YAML')
    declare_mode = DeclareLaunchArgument(
        'mode', default_value='2',
        description='Speed mode: 1=slow(6.0s) 2=normal(4.5s) 3=fast(3.0s)')
    declare_repeat = DeclareLaunchArgument(
        'repeat', default_value='1', description='Number of full round trips')

    actuator_params = LaunchConfiguration('actuator_params_file')
    planner_params = LaunchConfiguration('planner_params_file')
    mode = LaunchConfiguration('mode')
    repeat = LaunchConfiguration('repeat')

    actuator_node = Node(
        package='ldlidar_node',
        executable='dynamixel_actuator_node.py',
        name='dynamixel_actuator',
        output='screen',
        parameters=[actuator_params])

    planner_node = Node(
        package='ldlidar_node',
        executable='oscillation_planner_node.py',
        name='oscillation_planner',
        output='screen',
        parameters=[planner_params, {
            'mode': ParameterValue(mode, value_type=int),
            'repeat': ParameterValue(repeat, value_type=int),
        }])

    # Start the planner a couple of seconds after the actuator so the actuator
    # is subscribed and ready before the first /joint_goal is published.
    delayed_planner = TimerAction(period=3.0, actions=[planner_node])

    ld = LaunchDescription()
    ld.add_action(declare_actuator_params)
    ld.add_action(declare_planner_params)
    ld.add_action(declare_mode)
    ld.add_action(declare_repeat)
    ld.add_action(actuator_node)
    ld.add_action(delayed_planner)
    return ld
