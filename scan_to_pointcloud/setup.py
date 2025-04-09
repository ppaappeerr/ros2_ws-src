from setuptools import setup
import os
from glob import glob

package_name = 'scan_to_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
