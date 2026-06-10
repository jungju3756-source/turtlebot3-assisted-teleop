import os
from glob import glob
from setuptools import setup

package_name = 'assisted_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smHan22',
    maintainer_email='smh7729@gmail.com',
    description='VFH-lite + 4-Way Guard Stop assisted teleoperation for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assisted_teleop_bridge = assisted_teleop.assisted_teleop_bridge:main',
            'tof_sensor_node = assisted_teleop.tof_sensor_node:main',
            'depth_pointcloud_node = assisted_teleop.depth_pointcloud_node:main',
            'distance_marker_node = assisted_teleop.distance_marker_node:main',
        ],
    },
)
