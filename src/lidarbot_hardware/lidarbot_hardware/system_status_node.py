#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Узел состояния системы.

Собирает простую диагностику:
- наличие данных с основных топиков;
- примерную частоту публикации;
- CPU;
- RAM;
- температуру;
- состояние E-stop, если доступно.

Публикует:
- /diagnostics
- /system_status

Пан-тилт отключен на стадии «Минимум».
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan, Imu, CompressedImage
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class SystemStatusNode(Node):
    def __init__(self):
        super().__init__('system_status_node')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('topic_timeout_sec', 2.0)

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('estop_topic', '/estop_state')

        self.declare_parameter('thermal_zone', '/sys/class/thermal/thermal_zone0/temp')

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.topic_timeout_sec = float(self.get_parameter('topic_timeout_sec').value)

        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 1.0

        if self.topic_timeout_sec <= 0.0:
            self.topic_timeout_sec = 2.0

        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.estop_topic = self.get_parameter('estop_topic').value

        self.thermal_zone = self.get_parameter('thermal_zone').value

        # Издательства
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        self.system_status_pub = self.create_publisher(
            DiagnosticArray,
            '/system_status',
            10
        )

        # Статистика по топикам
        # Пан-тилт отключен на стадии «Минимум»
        self.stats = {
            'lidar': {
                'topic': self.scan_topic,
                'type': LaserScan,
                'count': 0,
                'hz': 0.0,
                'last_time': None,
            },
            'camera': {
                'topic': self.camera_topic,
                'type': CompressedImage,
                'count': 0,
                'hz': 0.0,
                'last_time': None,
            },
            'imu': {
                'topic': self.imu_topic,
                'type': Imu,
                'count': 0,
                'hz': 0.0,
                'last_time': None,
            },
            'motor_odom': {
                'topic': self.odom_topic,
                'type': Odometry,
                'count': 0,
                'hz': 0.0,
                'last_time': None,
            },
        }

        # Подписки
        for key, info in self.stats.items():
            self.create_subscription(
                info['type'],
                info['topic'],
                self._make_callback(key),
                qos_profile_sensor_data
            )

        # E-stop
        self.estop_state = None
        self.estop_last_time = None

        self.create_subscription(
            Bool,
            self.estop_topic,
            self._estop_callback,
            10
        )

        # CPU
        self._cpu_prev = None
        self._cpu_percent = 0.0

        # RAM
        self._mem_used_percent = 0.0

        # Temperature
        self._temperature_c = float('nan')

        self._last_publish_time = self.get_clock().now()

        self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)

        self.get_logger().info('system_status_node запущен (пан-тилт отключен)')

    def _make_callback(self, key: str):
        """Создает callback для подсчета сообщений конкретного топика."""
        def callback(msg):
            now = self.get_clock().now()
            self.stats[key]['count'] += 1
            self.stats[key]['last_time'] = now
        return callback

    def _estop_callback(self, msg: Bool):
        """Прием состояния E-stop."""
        self.estop_state = bool(msg.data)
        self.estop_last_time = self.get_clock().now()

    def _read_cpu(self):
        """Читает /proc/stat и считает приблизительную загрузку CPU."""
        try:
            with open('/proc/stat', 'r', encoding='utf-8') as f:
                line = f.readline()

            parts = line.split()
            if parts[0] != 'cpu':
                return

            values = list(map(int, parts[1:9]))

            idle = values[3] + values[4]
            total = sum(values)

            if self._cpu_prev is not None:
                prev_total, prev_idle = self._cpu_prev
                total_diff = total - prev_total
                idle_diff = idle - prev_idle

                if total_diff > 0:
                    self._cpu_percent = 100.0 * (1.0 - idle_diff / total_diff)
                else:
                    self._cpu_percent = 0.0

            self._cpu_prev = (total, idle)

        except Exception:
            self._cpu_percent = 0.0

    def _read_mem(self):
        """Читает /proc/meminfo и считает использование RAM."""
        try:
            mem_total = None
            mem_available = None

            with open('/proc/meminfo', 'r', encoding='utf-8') as f:
                for line in f:
                    if line.startswith('MemTotal:'):
                        mem_total = int(line.split()[1])
                    elif line.startswith('MemAvailable:'):
                        mem_available = int(line.split()[1])

            if mem_total and mem_available is not None:
                used = mem_total - mem_available
                self._mem_used_percent = 100.0 * used / mem_total
            else:
                self._mem_used_percent = 0.0

        except Exception:
            self._mem_used_percent = 0.0

    def _read_temperature(self):
        """Читает температуру из теплового сенсора."""
        try:
            if not os.path.exists(self.thermal_zone):
                self._temperature_c = float('nan')
                return

            with open(self.thermal_zone, 'r', encoding='utf-8') as f:
                raw = int(f.read().strip())

            self._temperature_c = raw / 1000.0

        except Exception:
            self._temperature_c = float('nan')

    def _age_seconds(self, last_time):
        """Возраст последнего сообщения в секундах."""
        if last_time is None:
            return float('inf')

        now = self.get_clock().now()
        age = (now - last_time).nanoseconds * 1e-9
        return max(0.0, age)

    def _make_topic_status(self, name: str, key: str) -> DiagnosticStatus:
        """Формирует DiagnosticStatus для конкретного топика."""
        info = self.stats[key]
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = info['topic']

        age = self._age_seconds(info['last_time'])

        status.values.append(KeyValue(key='topic', value=str(info['topic'])))
        status.values.append(KeyValue(key='hz', value=f"{info['hz']:.2f}"))
        status.values.append(KeyValue(key='age_sec', value=f"{age:.2f}"))

        if info['last_time'] is not None and age < self.topic_timeout_sec:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'нет свежих данных'

        return status

    def _make_motor_status(self) -> DiagnosticStatus:
        """Формирует статус моторов. Использует /odom_raw и /estop_state."""
        status = DiagnosticStatus()
        status.name = 'motors'
        status.hardware_id = self.odom_topic

        info = self.stats['motor_odom']
        age = self._age_seconds(info['last_time'])

        status.values.append(KeyValue(key='topic', value=str(info['topic'])))
        status.values.append(KeyValue(key='hz', value=f"{info['hz']:.2f}"))
        status.values.append(KeyValue(key='age_sec', value=f"{age:.2f}"))

        if self.estop_state is None:
            status.values.append(KeyValue(key='estop', value='unknown'))
        else:
            status.values.append(KeyValue(key='estop', value='ON' if self.estop_state else 'OFF'))

        if info['last_time'] is not None and age < self.topic_timeout_sec:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'нет /odom_raw'

        return status

    def _make_system_status(self) -> DiagnosticStatus:
        """Формирует статус вычислительной системы."""
        status = DiagnosticStatus()
        status.name = 'system'
        status.hardware_id = 'raspberry_pi4'

        status.values.append(KeyValue(key='cpu_percent', value=f"{self._cpu_percent:.1f}"))
        status.values.append(KeyValue(key='ram_used_percent', value=f"{self._mem_used_percent:.1f}"))

        if self._temperature_c == self._temperature_c:  # not NaN
            status.values.append(KeyValue(key='temperature_c', value=f"{self._temperature_c:.1f}"))
        else:
            status.values.append(KeyValue(key='temperature_c', value='unknown'))

        level = DiagnosticStatus.OK
        messages = []

        if self._cpu_percent > 90.0:
            level = max(level, DiagnosticStatus.WARN)
            messages.append('высокая CPU')

        if self._mem_used_percent > 90.0:
            level = max(level, DiagnosticStatus.WARN)
            messages.append('высокая RAM')

        if self._temperature_c == self._temperature_c and self._temperature_c > 80.0:
            level = max(level, DiagnosticStatus.WARN)
            messages.append('высокая температура')

        status.level = level
        status.message = ', '.join(messages) if messages else 'OK'

        return status

    def _timer_callback(self):
        """Периодическая публикация диагностики."""
        now = self.get_clock().now()
        dt = (now - self._last_publish_time).nanoseconds * 1e-9

        if dt <= 0.0:
            dt = 1.0 / self.publish_rate_hz

        # Частота топиков
        for key, info in self.stats.items():
            info['hz'] = info['count'] / dt
            info['count'] = 0

        self._last_publish_time = now

        # Системные метрики
        self._read_cpu()
        self._read_mem()
        self._read_temperature()

        # Формирование массива диагностики
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()

        diag.status.append(self._make_topic_status('lidar', 'lidar'))
        diag.status.append(self._make_topic_status('camera', 'camera'))
        diag.status.append(self._make_topic_status('imu', 'imu'))
        # Пан-тилт отключен на стадии «Минимум»
        diag.status.append(self._make_motor_status())
        diag.status.append(self._make_system_status())

        self.diagnostics_pub.publish(diag)
        self.system_status_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = SystemStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
