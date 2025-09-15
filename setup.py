from setuptools import find_packages, setup

package_name = 'my_spray_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "flow_sensor_node = my_spray_controller.publish_flow_rate:main",
            "pressure_node = my_spray_controller.publish_pressure:main",
            "bno085_node = my_spray_controller.publish_orientation:main",
            "pump_test_node = my_spray_controller.pump_test_node:main",
        ],
    },
)
