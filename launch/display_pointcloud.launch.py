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
    # index_from_last = LaunchConfiguration('index_from_last')
    
    package_dir = get_package_share_directory('my_spray_controller')
    rviz_config_file = os.path.join(package_dir, 'config', 'spray_controller_config_pico.rviz')
    # scan_directory_path = '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans'

    
    return LaunchDescription([
        # Start RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        ),

        DeclareLaunchArgument(
            'scan_directory',
            default_value='/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans',
            description='Optional: whatever path the scans should be loaded from'
        ),

        DeclareLaunchArgument(
            'index_from_last',
            default_value='1',
            description='Optional: 1 = last Data, 2 = second last, ...'
        ),

        DeclareLaunchArgument(
            'filter_left_distance',
            default_value='1.0',
            description='Optional: Points to keep around left distance \nSet to -1 for keeping all left Points. '
        ),

        DeclareLaunchArgument(
            'filter_right_distance',
            default_value='1.0',
            description='Optional: Points to keep around right distance \nSet to -1 for keeping all left Points. '
        ),

        DeclareLaunchArgument(
            'filter_tolerance',
            default_value='0.25',
            description='Optional: Tolerance range for points to keep around left&right distance'
        ),

         DeclareLaunchArgument(
            'cyclic_publish',
            default_value='True',
            description='Optional: Cyclic publishing of /pointcloud'
        ),
        
        # Start pointcloud loader node
        Node(
            package='my_spray_controller',
            executable='pointcloud_loader_node',
            name='pointcloud_loader_node',
            parameters=[{
                'scan_directory': LaunchConfiguration('scan_directory'),    # abs path
                'index_from_last': LaunchConfiguration('index_from_last'),
                'filter_left_distance': LaunchConfiguration('filter_left_distance'),          # -1 = no filter
                'filter_right_distance': LaunchConfiguration('filter_right_distance'),         # -1 = no filter
                'filter_tolerance': LaunchConfiguration('filter_tolerance'),
                'cyclic_publish': LaunchConfiguration('cyclic_publish'),
            }],
            output='screen'
        )
    ])