from setuptools import find_packages
from setuptools import setup

setup(
    name='simple_arm_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('simple_arm_interfaces', 'simple_arm_interfaces.*')),
)
