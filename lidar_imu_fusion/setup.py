from setuptools import find_packages, setup

package_name = 'lidar_imu_fusion'

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
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='라이다와 IMU 센서 데이터 통합 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_tf_publisher = lidar_imu_fusion.imu_tf_publisher:main',
            'imu_odom_publisher = lidar_imu_fusion.imu_odom_publisher:main'
        ],
    },
)