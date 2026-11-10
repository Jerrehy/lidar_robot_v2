#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Безопасный драйвер моторов для стадии «Минимум».

Особенности:
- по умолчанию реальное движение запрещено;
- движение разрешается только параметром motion_allowed;
- дополнительно движение блокируется аварийной остановкой;
- есть watchdog: если нет /cmd_vel дольше заданного времени, остановка;
- публикуется /odom_raw, но не публикуется TF odom -> base_link;
- реальная аппаратная часть моторов здесь сделана заглушкой.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def clamp(value: float, low: float, high: float) -> float:
    """
    Ограничение значения.
    """
    return max(low, min(high, value))


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Параметры безопасности
        self.declare_parameter('motion_allowed', False)
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('estop_active_default', True)
        self.declare_parameter('watchdog_timeout_sec', 0.4)

        # Ограничения скорости
        self.declare_parameter('max_linear_x', 0.2)
        self.declare_parameter('max_angular_z', 0.5)

        # Параметры одометрии
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('odom_frame', 'odom_raw')
        self.declare_parameter('base_frame', 'base_link')

        # Параметры модели колес для диагностического расчета
        self.declare_parameter('wheel_separation_m', 0.20)
        self.declare_parameter('wheel_radius_m', 0.033)

        # Чтение параметров
        self.motion_allowed = bool(self.get_parameter('motion_allowed').value)
        self.hardware_enabled = bool(self.get_parameter('hardware_enabled').value)
        self.estop_active = bool(self.get_parameter('estop_active_default').value)
        self.watchdog_timeout_sec = float(self.get_parameter('watchdog_timeout_sec').value)

        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.wheel_separation_m = float(self.get_parameter('wheel_separation_m').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)

        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 10.0

        if self.watchdog_timeout_sec <= 0.0:
            self.get_logger().warn('watchdog_timeout_sec <= 0, использую 0.4 с')
            self.watchdog_timeout_sec = 0.4

        # Текущее состояние
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_timer_time = now

        self._last_motor_command = None
        self._last_warn_time = 0.0

        # QoS для состояния E-stop
        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.estop_state_pub = self.create_publisher(Bool, '/estop_state', estop_qos)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        self.estop_service = self.create_service(
            SetBool,
            '/set_estop',
            self._set_estop_callback
        )

        self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)

        self._publish_estop_state()

        self.get_logger().info(
            f'motor_driver запущен: motion_allowed={self.motion_allowed}, '
            f'hardware_enabled={self.hardware_enabled}, '
            f'estop_active={self.estop_active}'
        )

        if self.motion_allowed:
            self.get_logger().warn(
                'motion_allowed=true! Разрешено диагностическое движение. '
                'Убедитесь, что робот находится в безопасном месте.'
            )
        else:
            self.get_logger().info(
                'Движение запрещено: motion_allowed=false. '
                'Моторы не будут выполнять команды /cmd_vel.'
            )

    def _cmd_vel_callback(self, msg: Twist):
        """
        Прием команд скорости.
        Команды учитываются только если движение разрешено и E-stop выключен.
        """
        now = self.get_clock().now()

        if not self.motion_allowed:
            self._warn_throttled('Команда /cmd_vel игнорируется: motion_allowed=false')
            return

        if self.estop_active:
            self._warn_throttled('Команда /cmd_vel игнорируется: активен E-stop')
            return

        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        if not math.isfinite(vx) or not math.isfinite(wz):
            self.get_logger().warn('Получена некорректная команда /cmd_vel: NaN или inf')
            return

        vx = clamp(vx, -self.max_linear_x, self.max_linear_x)
        wz = clamp(wz, -self.max_angular_z, self.max_angular_z)

        self.cmd_vx = vx
        self.cmd_wz = wz
        self.last_cmd_time = now

        if self.hardware_enabled:
            self._apply_motor_commands(vx, wz)

    def _set_estop_callback(self, request, response):
        """
        Сервис /set_estop.
        true — включить аварийную остановку,
        false — выключить аварийную остановку.
        """
        new_state = bool(request.data)
        self.estop_active = new_state

        if self.estop_active:
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0

            if self.hardware_enabled:
                self._apply_motor_commands(0.0, 0.0)

            self.get_logger().warn('E-stop включен')
            response.message = 'E-stop enabled'
        else:
            if not self.motion_allowed:
                self.get_logger().warn(
                    'E-stop выключен, но motion_allowed=false, '
                    'движение все равно запрещено'
                )
            else:
                self.get_logger().info('E-stop выключен')

            response.message = 'E-stop disabled'

        self._publish_estop_state()

        response.success = True
        return response

    def _publish_estop_state(self):
        """
        Публикует текущее состояние E-stop.
        """
        msg = Bool()
        msg.data = self.estop_active
        self.estop_state_pub.publish(msg)

    def _warn_throttled(self, text: str, period_sec: float = 5.0):
        """
        warning без спама.
        """
        now = time.monotonic()
        if now - self._last_warn_time > period_sec:
            self._last_warn_time = now
            self.get_logger().warn(text)

    def _apply_motor_commands(self, vx: float, wz: float):
        """
        Заглушка реальной отправки команд на драйвер моторов.

        Здесь нужно заменить код на реальную передачу команд
        в ваш контроллер моторов.

        На стадии «Минимум» мы только логируем расчетные значения.
        """
        if self.wheel_radius_m <= 0.0:
            self.get_logger().warn('wheel_radius_m <= 0, расчет колес невозможен')
            return

        # Дифференциальная кинематика:
        # left = (v - w * L / 2) / r
        # right = (v + w * L / 2) / r
        left_wheel_rad_s = (vx - wz * self.wheel_separation_m / 2.0) / self.wheel_radius_m
        right_wheel_rad_s = (vx + wz * self.wheel_separation_m / 2.0) / self.wheel_radius_m

        command = (round(left_wheel_rad_s, 3), round(right_wheel_rad_s, 3))

        if command != self._last_motor_command:
            self._last_motor_command = command
            self.get_logger().info(
                f'Motor command: left={left_wheel_rad_s:.3f} rad/s, '
                f'right={right_wheel_rad_s:.3f} rad/s'
            )

            # TODO: здесь должна быть реальная отправка команд
            # в драйвер моторов. Без subprocess и без прямого GPIO,
            # если используется внешний контроллер.

    def _timer_callback(self):
        """
        Периодическая обработка watchdog и публикация /odom_raw.
        """
        now = self.get_clock().now()

        dt = (now - self.last_timer_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            # Защита от скачков времени
            dt = 1.0 / self.publish_rate_hz

        self.last_timer_time = now

        motion_active = self.motion_allowed and not self.estop_active

        if motion_active:
            cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9

            if cmd_age > self.watchdog_timeout_sec:
                if self.cmd_vx != 0.0 or self.cmd_wz != 0.0:
                    self.get_logger().warn(
                        f'Watchdog: нет /cmd_vel {cmd_age:.2f} с, остановка'
                    )

                self.cmd_vx = 0.0
                self.cmd_wz = 0.0

                if self.hardware_enabled:
                    self._apply_motor_commands(0.0, 0.0)
            else:
                # Интегрирование команды только в диагностическом режиме.
                # Это не честная одометрия по энкодерам,
                # а сырая оценка по команде.
                self.x += self.cmd_vx * math.cos(self.theta) * dt
                self.y += self.cmd_vx * math.sin(self.theta) * dt
                self.theta += self.cmd_wz * dt

                # Нормализация угла
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        else:
            if self.cmd_vx != 0.0 or self.cmd_wz != 0.0:
                self.cmd_vx = 0.0
                self.cmd_wz = 0.0

                if self.hardware_enabled:
                    self._apply_motor_commands(0.0, 0.0)

        self._publish_odometry(now)

    def _publish_odometry(self, now):
        """
        Публикация /odom_raw.
        TF odom -> base_link намеренно не публикуется.
        """
        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion из yaw
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        if self.motion_allowed and not self.estop_active:
            odom.twist.twist.linear.x = self.cmd_vx
            odom.twist.twist.angular.z = self.cmd_wz
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0

        # Простые диагональные ковариации
        for i in range(6):
            odom.pose.covariance[i * 6 + i] = 0.01
            odom.twist.covariance[i * 6 + i] = 0.01

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
