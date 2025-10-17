#!/usr/bin/env python3
"""
Launch file for LDLidar with visualization

Launches:
1. Lifecycle manager
2. Robot state publisher
3. LDLidar component node (in container)
4. Visualization node

This is similar to ldlidar_simple.launch.py but adds the visualization node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('ldlidar_node')
    
    # Paths to configuration files
    params_file = os.path.join(pkg_dir, 'params', 'ldlidar.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ldlidar_descr.urdf.xml')
    lifecycle_nodes = ['ldlidar_node']

    # Launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for the ldlidar node'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Launch configuration variables
    params_file_launch = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ldlidar_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file).read()
        }]
    )

    # Component container for the lidar node
    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='ldlidar_container',
        output='screen'
    )

    # Load the lidar component into the container
    load_composable_nodes = LoadComposableNodes(
        target_container='ldlidar_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_component',
                plugin='ldlidar::LdLidarComponent',
                name='ldlidar_node',
                parameters=[params_file_launch, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # Log info
    log_info = LogInfo(msg='* Loading node: ldlidar_node in container: /ldlidar_container')

    # Visualization node
    visualizer_node = Node(
        package='ldlidar_node',
        executable='ldlidar_visualizer.py',
        name='ldlidar_visualizer',
        output='screen',
        parameters=[{
            'scan_topic': '/ldlidar_node/scan',
            'update_rate': 10.0,
            'point_size': 20.0,
            'colormap': 'jet_r'  # red (close) to blue (far)
        }]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add nodes
    ld.add_action(log_info)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    ld.add_action(visualizer_node)

    return ld
