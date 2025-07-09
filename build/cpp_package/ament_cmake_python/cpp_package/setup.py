from setuptools import find_packages
from setuptools import setup

setup(
    name='cpp_package',
    version='0.0.1',
    packages=find_packages(
        include=('cpp_package', 'cpp_package.*')),
)
