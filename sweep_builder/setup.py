# sweep_builder/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'sweep_builder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=[
        'setuptools', 
        'tf_transformations',
        'numpy',      # 🔥 추가
        'open3d'      # 🔥 추가
    ],
    zip_safe=True,
    maintainer='jaden',
    maintainer_email='you@example.com',
    description='2D LiDAR + IMU 누적 3D 포인트클라우드 발행 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leveled_scan_node = sweep_builder.leveled_scan_node:main',
            'lidar_imu_to_pointcloud = sweep_builder.lidar_imu_to_pointcloud:main',
            'lidar_imu_filtered_to_pointcloud = sweep_builder.lidar_imu_filtered_to_pointcloud:main',
            'ground_aligned_scan_node = sweep_builder.ground_aligned_scan_node:main',
            'sweep_accmulator_node = sweep_builder.sweep_accumulator_node:main',
            'point_preprocessor_node = sweep_builder.point_preprocessor_node:main',
            'sweep_xy_filter_node = sweep_builder.sweep_xy_filter_node:main',
            'sweep_node = sweep_builder.sweep_node:main',
            'accumulator_node = sweep_builder.accumulator_node:main',
            'preproc_front_filter_node = sweep_builder.preproc_front_filter_node:main',
            'lidar_imu_to_pointcloud_front = sweep_builder.lidar_imu_filtered_to_pointcloud_front:main',
            'dynamic_obstacle_processor = sweep_builder.dynamic_obstacle_processor:main',  # 🔥 추가
        ],
    },
)
