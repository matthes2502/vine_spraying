from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_spray_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        # RViz configs
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.rviz')),
        # LiDAR scans directory
        (os.path.join('share', package_name, 'lidar_scans'), 
         glob('lidar_scans/*') if os.path.exists('lidar_scans') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthes',
    maintainer_email='schmma10@thu.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spray_coordinator_node = my_spray_controller.spray_coordinator:main",
            "flow_sensor_node = my_spray_controller.publish_flow_rate:main",
            "pressure_node = my_spray_controller.publish_pressure:main",
            "bno085_node = my_spray_controller.publish_orientation:main",
            "valve_control_node = my_spray_controller.valve_controller:main",
            "scanner_node = my_spray_controller.leaf_wall_scanner:main",
            "relais_node = my_spray_controller.relais_controller:main",
            "pointcloud_loader_node = my_spray_controller.pointcloud_loader:main",
            # "pump_test_node = my_spray_controller.pump_test_node:main",
            "rplidar_complete_test_node = my_spray_controller.rplidar_tester_complete:main",
            "show_pointcloud_node = my_spray_controller.show_saved_pointcloud:main",
        ],
    },
)
