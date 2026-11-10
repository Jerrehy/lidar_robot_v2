#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Простой вывод состояния системы из /system_status.
"""

import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray


LEVEL_NAMES = {
    0: 'OK',
    1: 'WARN',
    2: 'ERROR',
    3: 'STALE',
}


def main():
    rclpy.init()

    node = rclpy.create_node('bot_status')

    received = []

    def callback(msg: DiagnosticArray):
        received.append(msg)

    node.create_subscription(
        DiagnosticArray,
        '/system_status',
        callback,
        10
    )

    start = time.monotonic()
    timeout_sec = 3.0

    while rclpy.ok() and time.monotonic() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            break

    if not received:
        print('ERROR: не получено /system_status за 3 секунды.')
        print('Проверьте, запущена ли система: bot up-min')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = received[-1]

    stamp = msg.header.stamp
    print(f"System status time: {stamp.sec}.{stamp.nanosec:09d}")
    print()

    for status in msg.status:
        level = LEVEL_NAMES.get(status.level, 'UNKNOWN')
        print(f"[{level:5}] {status.name}: {status.message}")

        for value in status.values:
            print(f"         {value.key}: {value.value}")

        print()

    node.destroy_node()
    rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
