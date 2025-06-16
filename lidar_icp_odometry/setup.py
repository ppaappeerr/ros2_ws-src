from setuptools import setup
import os
from glob import glob

package_name = 'lidar_icp_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/lidar_icp_odometry']),
                ('share/lidar_icp_odometry/launch', ['launch/odometry_launch.py']),
               ],
    install_requires=['setuptools','scikit-learn', 'numpy', 'ros2-numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ICP based odometry estimation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_odom_node = lidar_icp_odometry.icp_odom_node:main',
            'voxel_z_clip = lidar_icp_odometry.voxel_z_clip:main',
        ],
        'launch_frontend.launch_extension': [
            'pylaunch = launch',
        ]
    }
)
