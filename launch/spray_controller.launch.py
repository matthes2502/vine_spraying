from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    # Get package share directory for RViz config
    pkg_share = FindPackageShare('my_spray_controller').find('my_spray_controller')
    rviz_config_path = str(Path(pkg_share) / 'config' / 'spray_controller_config.rviz')

    return LaunchDescription([
        # Joy Node for Xbox Controller
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     output='screen',
        # ),

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

        # BNO085 IMU Node
        Node(
            package='my_spray_controller',
            executable='bno085_node',
            name='bno085_node',
            output='screen',
            parameters=[
                {'publish_orientation_rate': 2.0}
            ]
        ),

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
        #         {'lateral_distance': 1.2},
        #         {'grid_height_min': 0.4},
        #         {'grid_height_max': 2.0},
        #         {'grid_length': 1.0},
        #         {'scan_history_distance': 1000.0},
        #         {'save_to_file': False},
        #         {'output_directory': './lidar_scans'}
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