#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import glob
from datetime import datetime

def generate_launch_description():
    # Get the scan file to load
    index_from_last = LaunchConfiguration('index_from_last')
    
    package_dir = get_package_share_directory('my_spray_controller')
    rviz_config_file = os.path.join(package_dir, 'config', 'spray_controller_config_pico.rviz')
    scan_directory_path = '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans'

    
    return LaunchDescription([
        # Start RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        ),
        
        # Start pointcloud loader node
        Node(
            package='my_spray_controller',
            executable='pointcloud_loader_node',
            name='pointcloud_loader_node',
            parameters=[
                {'scan_directory': scan_directory_path},    # abs path
                {'index_from_last': 0},
                {'filter_left_distance': 1.0},          # -1 = no filter
                {'filter_right_distance': 1.0},         # -1 = no filter
                {'filter_tolerance': 0.5}
            ],
            output='screen'
        )
    ])