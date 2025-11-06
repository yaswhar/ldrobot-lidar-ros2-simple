#!/usr/bin/env python3
"""
Launch file for LDLidar with Ground Mapper and Statistics Logger

Launches:
1. Lifecycle manager
2. Robot state publisher
3. LDLidar component node (in container)
4. Ground mapper node (for topological mapping)
5. Statistics publisher node (publishes to /ldlidar_stats topic)
6. Statistics logger GUI node (logs statistics with pause/resume functionality)

This launch file combines ground mapping with statistics logging,
ideal for monitoring lidar performance while building topological maps.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        description='Full path to the ROS2 parameters file to use for both ldlidar and ground mapper'
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

    declare_drone_velocity_cmd = DeclareLaunchArgument(
        'drone_velocity',
        default_value='1.0',
        description='Drone velocity in any direction (m/s)'
    )

    declare_stats_update_rate_cmd = DeclareLaunchArgument(
        'stats_update_rate',
        default_value='10.0',
        description='Statistics update rate in Hz (default: 10.0 Hz to match ground mapper)'
    )

    # Launch configuration variables
    params_file_launch = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    drone_velocity = LaunchConfiguration('drone_velocity')
    stats_update_rate = LaunchConfiguration('stats_update_rate')

    # Lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
            {'bond_timeout': 0.0}  # Disable bond connection
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

    # Ground mapper node
    ground_mapper_node = Node(
        package='ldlidar_node',
        executable='ldlidar_ground_mapper.py',
        name='ground_mapper',
        output='screen',
        parameters=[
            params_file_launch,
            {'drone_velocity': drone_velocity}
        ]
    )

    # Ground mapper logger node
    # Logs complete point cloud data for offline topological map generation
    ground_mapper_logger_node = Node(
        package='ldlidar_node',
        executable='ldlidar_ground_mapper_logger.py',
        name='ground_mapper_logger',
        output='screen',
        parameters=[
            params_file_launch,
            {'drone_velocity': drone_velocity}
        ]
    )

    # Statistics publisher node
    # Update rate set to match ground mapper (10 Hz) for synchronized monitoring
    stats_publisher_node = Node(
        package='ldlidar_node',
        executable='ldlidar_stats.py',
        name='ldlidar_stats_publisher',
        output='screen',
        parameters=[
            params_file_launch,
            {'update_rate': stats_update_rate}
        ]
    )

    # Statistics logger node with Tkinter GUI
    # Optimized for Raspberry Pi - uses built-in Tkinter (no extra packages needed)
    # Creates a pop-up window with pause/clear controls and automatic memory management
    stats_logger_node = Node(
        package='ldlidar_node',
        executable='ldlidar_stats_logger_gui.py',
        name='ldlidar_stats_logger',
        output='screen',
        parameters=[params_file_launch]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_drone_velocity_cmd)
    ld.add_action(declare_stats_update_rate_cmd)

    # Add nodes
    ld.add_action(log_info)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    ld.add_action(ground_mapper_node)
    ld.add_action(ground_mapper_logger_node)
    ld.add_action(stats_publisher_node)
    ld.add_action(stats_logger_node)

    return ld
