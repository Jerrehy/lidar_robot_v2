#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch-файл стадии «Минимум».

Запускает:
- sllidar_node;
- v4l2_camera;
- imu_driver;
- pan_tilt_driver;
- motor_driver;
- system_status_node.

На этой стадии не запускаются:
- SLAM;
- Nav2;
- автономная навигация;
- веб-интерфейс;
- планировщики миссий.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lidarbot_hardware')

    lidar_params = os.path.join(pkg_share, 'config', 'lidar.yaml')
    camera_params = os.path.join(pkg_share, 'config', 'camera.yaml')
    imu_params = os.path.join(pkg_share, 'config', 'imu.yaml')
    # pan_tilt_params = os.path.join(pkg_share, 'config', 'pan_tilt.yaml')
    motor_params = os.path.join(pkg_share, 'config', 'motors.yaml')
    status_params = os.path.join(pkg_share, 'config', 'status.yaml')

    # Лидар
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[lidar_params],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    # Камера
    #
    # image_transport должен обеспечить в том числе:
    # /camera/image_raw/compressed
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[camera_params],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    # IMU
    imu_node = Node(
        package='lidarbot_hardware',
        executable='imu_driver',
        name='imu_driver',
        output='screen',
        parameters=[imu_params],
    )

    # Пан-тилт
    #pan_tilt_node = Node(
    #    package='lidarbot_hardware',
    #    executable='pan_tilt_driver',
    #    name='pan_tilt_driver',
    #    output='screen',
    #    parameters=[pan_tilt_params],
    #)

    # Безопасный драйвер моторов
    motor_node = Node(
        package='lidarbot_hardware',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[motor_params],
    )

    # Состояние системы
    status_node = Node(
        package='lidarbot_hardware',
        executable='system_status_node',
        name='system_status_node',
        output='screen',
        parameters=[status_params],
    )

    return LaunchDescription([
        sllidar_node,
        camera_node,
        imu_node,
        #pan_tilt_node,
        motor_node,
        status_node,
    ])
