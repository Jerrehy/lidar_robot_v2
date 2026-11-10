#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Базовая проверка системы:
- есть ли обязательные топики;
- есть ли ожидаемые узлы;
- публикуются ли данные;
- есть ли основные устройства.
"""

import os
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Imu, CompressedImage
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray


# Все обязательные топики стадии «Минимум»
ALL_TOPICS = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/camera/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    #'/pan_tilt_cmd': 'geometry_msgs/msg/Vector3',
    #'/pan_tilt_state': 'geometry_msgs/msg/Vector3',
    '/odom_raw': 'nav_msgs/msg/Odometry',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/system_status': 'diagnostic_msgs/msg/DiagnosticArray',
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
}

# Топики, для которых проверяем не только наличие, но и приход данных
DATA_TOPICS = {
    '/scan': LaserScan,
    '/imu/data': Imu,
    '/camera/image_raw/compressed': CompressedImage,
    #'/pan_tilt_state': Vector3,
    '/odom_raw': Odometry,
    '/system_status': DiagnosticArray,
    '/diagnostics': DiagnosticArray,
}

EXPECTED_NODES = [
    '/sllidar_node',
    '/camera',
    '/imu_driver',
    #'/pan_tilt_driver',
    '/motor_driver',
    '/system_status_node',
]

DEVICE_HINTS = [
    '/dev/video0',
    '/dev/ttyUSB0',
    '/dev/i2c-1',
]


def main():
    errors = 0
    warnings = 0

    rclpy.init()
    node = rclpy.create_node('bot_doctor')

    topic_names = dict(node.get_topic_names_and_types())
    node_names = node.get_node_names()

    print('=== Проверка топиков ===')
    for topic, expected_type in ALL_TOPICS.items():
        if topic in topic_names:
            print(f'[OK   ] {topic} существует')
        else:
            print(f'[ERROR] {topic} отсутствует')
            errors += 1

    print()
    print('=== Проверка узлов ===')
    for expected_node in EXPECTED_NODES:
        if expected_node in node_names:
            print(f'[OK   ] {expected_node} найден')
        else:
            print(f'[ERROR] {expected_node} не найден')
            errors += 1

    print()
    print('=== Проверка устройств ===')
    for device in DEVICE_HINTS:
        if os.path.exists(device):
            print(f'[OK   ] {device} существует')
        else:
            print(f'[WARN ] {device} не найден')
            warnings += 1

    print()
    print('=== Проверка потока данных ===')

    received = {topic: False for topic in DATA_TOPICS.keys()}

    for topic, msg_type in DATA_TOPICS.items():
        if topic not in topic_names:
            continue

        def make_callback(topic_name):
            def callback(msg):
                received[topic_name] = True
            return callback

        node.create_subscription(
            msg_type,
            topic,
            make_callback(topic),
            qos_profile_sensor_data
        )

    wait_seconds = 3.0
    end_time = time.monotonic() + wait_seconds

    print(f'Ожидание данных в течение {wait_seconds:.1f} секунд...')

    while rclpy.ok() and time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    print()

    for topic in DATA_TOPICS.keys():
        if topic not in topic_names:
            print(f'[ERROR] {topic}: топик отсутствует')
            errors += 1
        elif received[topic]:
            print(f'[OK   ] {topic}: данные приходят')
        else:
            print(f'[ERROR] {topic}: данные не приходят')
            errors += 1

    print()
    print('=== Итог ===')
    print(f'Ошибки: {errors}')
    print(f'Предупреждения: {warnings}')

    node.destroy_node()
    rclpy.shutdown()

    if errors > 0:
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
