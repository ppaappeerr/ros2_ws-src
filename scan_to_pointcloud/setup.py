from setuptools import setup
import os
from glob import glob

package_name = 'scan_to_pointcloud'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='p',
    maintainer_email='p@todo.todo',
    description='Converts 2D laser scans to 3D point clouds using IMU data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_to_pointcloud = scan_to_pointcloud.scan_to_pointcloud_node:main',
            'accumulated_pointcloud = scan_to_pointcloud.accumulated_pointcloud_node:main',
            'simple_accumulator = scan_to_pointcloud.simple_accumulator:main',
            'pointcloud_accumulator = scan_to_pointcloud.pointcloud_accumulator_node:main',
        ],
    },
)
