from setuptools import find_packages, setup

package_name = 'mpu9250_py'

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
            'mpu9250_node = mpu9250_py.mpu9250_node:main',
            'mpu9250_calibrate = mpu9250_py.mpu9250_calibrate:main',
            ''
        ],
    },
)
