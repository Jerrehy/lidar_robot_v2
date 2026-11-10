import os
from glob import glob

from setuptools import setup

package_name = 'lidarbot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lidarbot',
    maintainer_email='dev@example.com',
    description='Минимальный набор аппаратных узлов для робота на Raspberry Pi 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_driver = lidarbot_hardware.imu_driver:main',
            'pan_tilt_driver = lidarbot_hardware.pan_tilt_driver:main',
            'motor_driver = lidarbot_hardware.motor_driver:main',
            'system_status_node = lidarbot_hardware.system_status_node:main',
        ],
    },
)
