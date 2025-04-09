from setuptools import find_packages, setup

package_name = 'mpu6050_py'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_node = mpu6050_py.mpu6050_node:main',
            'mpu6050_calibrate = mpu6050_py.mpu6050_calibrate:main'
        ],
    },
)
