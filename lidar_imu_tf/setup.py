from setuptools import find_packages, setup

package_name = 'lidar_imu_tf'

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
    maintainer='p',
    maintainer_email='p@todo.todo',
    description='TF publisher using IMU',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_tf_publisher = lidar_imu_tf.imu_tf_publisher:main'
        ],
    },
)
