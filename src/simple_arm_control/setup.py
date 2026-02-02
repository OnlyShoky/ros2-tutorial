from setuptools import find_packages, setup

package_name = 'simple_arm_control'

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
    maintainer='shoky',
    maintainer_email='leozul@hotmail.es',
    description='Control nodes for simple robot arm',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper_server = simple_arm_control.gripper_server:main',
            'gripper_client = simple_arm_control.gripper_client:main',
        ],
    },
)
