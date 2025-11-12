from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory for RViz config
    pkg_share = get_package_share_directory('my_spray_controller')
    rviz_config_path = str(Path(pkg_share) / 'config' / 'spray_controller_config_pico.rviz')
    scan_output_path = '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans'

    # sllidar launch file
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a3_launch.py'
            )
        ]),
        launch_arguments={
            # 'scan_mode': 'Stability',           # for use outdoors or inside with sun light
            'scan_mode': 'Sensitivity',         # for use indoors with non natural light
            # 'serial_port': '/dev/ttyUSB0',
        }.items()
    )

    return LaunchDescription([
        # SLLIDAR
        # sllidar_launch,

        # Joy Node for Xbox Controller
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Xbox Controller Node from my_rovo_controller
        Node(
            package='my_rovo_controller',
            executable='xbox_rovo_controller',
            name='xbox_rovo_controller',
            output='screen',
        ),

        # Spray Coordinator Node
        Node(
            package='my_spray_controller',
            executable='spray_coordinator_node',
            name='spray_coordinator_node',
            output='screen',
        ),

        # Flow Sensor Node
        Node(
            package='my_spray_controller',
            executable='flow_sensor_node',
            name='flow_sensor_node',
            output='screen',
            parameters=[
                {'publish_flow_rate': 2.0}
            ]
        ),

        # Pressure Node
        Node(
            package='my_spray_controller',
            executable='pressure_node',
            name='pressure_node',
            output='screen',
            parameters=[
                {'publish_pressure_rate': 2.0}
            ]
        ),

        # # BNO085 IMU Node
        # Node(
        #     package='my_spray_controller',
        #     executable='bno085_node',
        #     name='bno085_node',
        #     output='screen',
        #     parameters=[
        #         {'publish_orientation_rate': 2.0}
        #     ]
        # ),

        # Valve Control Node
        Node(
            package='my_spray_controller',
            executable='valve_control_node',
            name='valve_control_node',
            output='screen',
        ),

        # # Scanner Node (Lidar)
        # Node(
        #     package='my_spray_controller',
        #     executable='scanner_node',
        #     name='scanner_node',
        #     output='screen',
        #     parameters=[
        #         {'lidar_height': 1.0},              # Height of lidar above ground
        #         {'lateral_distance': 1.2},          # Distance to foliage wall (left/right)
        #         {'grid_height_min': 0.4},           # Minimum height for grid (40cm)
        #         {'grid_height_max': 2.0},           # Maximum height for grid (2m)
        #         {'grid_length': 1.0},               # Grid length in driving direction (1m)
        #         {'scan_history_distance': 1000.0},  # Keep last X meters of scans (set very high to keep all)
        #         {'save_to_file': False},
        #         {'output_directory': scan_output_path}
        #     ]
        # ),

        # Relais Controller Node
        Node(
            package='my_spray_controller',
            executable='relais_node',
            name='relais_node',
            output='screen',
            parameters=[
                {'main_pump_init': 'off'},
                {'transfer_pump_init': 'off'}, # 'off', 'fw', or 'psm'
                {'valve_init': 'off'} # 'off', 'fw', or 'psm'
            ]
        ),

        # Propeller Controller Node (C++ executable)
        Node(
            package='my_hardware_pwm_controller',
            executable='propeller_controller_node',
            name='propeller_controller_node',
            output='screen',
        ),

        # Pump Controller Node (C++ executable)
        Node(
            package='my_hardware_pwm_controller',
            executable='pump_controller_node',
            name='pump_controller_node',
            output='screen',
        ),

        # # RViz2 Visualization
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_path],
        # ),
    ])