# Copyright 2022 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

"""
Simple launch file for Raspberry Pi 4 deployment.
This launches the LDLidar with lifecycle manager for auto-recovery and monitoring.
No visualization tools (RViz2) included - suitable for headless operation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate simple launch description for Raspberry Pi deployment."""
    
    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml'
    )

    # Lifecycle manager node - handles auto-recovery and state management
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config_path]
    )

    # Include LDLidar launch with lifecycle support
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node'
        }.items()
    )

    # Build launch description
    ld = LaunchDescription()
    
    # Add lifecycle manager for auto-recovery
    ld.add_action(lc_mgr_node)
    
    # Add LDLidar node
    ld.add_action(ldlidar_launch)

    return ld
