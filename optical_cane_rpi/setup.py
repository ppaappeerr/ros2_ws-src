import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'optical_cane_rpi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='p',
    maintainer_email='whdgnl098@gmail.com',
    description='Optical cane for RPi with SLLIDAR and MPU9250',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = optical_cane_rpi.sensor_fusion_node:main',
            'sweep_node = optical_cane_rpi.sweep_node:main',
            'accumulator_node = optical_cane_rpi.accumulator_node:main',
            'angle_sweeper_node = optical_cane_rpi.angle_sweeper_node:main',
        ],
    },
)
