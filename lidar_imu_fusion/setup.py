from setuptools import setup
import os
from glob import glob

package_name = 'lidar_imu_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 실행 파일 명시적으로 복사
        (os.path.join('lib', package_name), [
            os.path.join(package_name, 'imu_tf_publisher.py'),
            os.path.join(package_name, 'imu_odom_publisher.py') # 이 줄 추가
        ]),
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
