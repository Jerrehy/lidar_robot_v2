#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Узел пан-тилт механизма через gpiozero + lgpio с оптимизацией для программного ШИМ.

Минимизирует дрожание сервоприводов на пинах без аппаратного ШИМ.
"""

import math
import os
import time
import warnings
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# Подавляем warning о программном ШИМ
warnings.filterwarnings("ignore", message=".*To reduce servo jitter.*")

try:
    from gpiozero import Servo
    from gpiozero.pins.lgpio import LGPIOFactory
except ImportError:
    Servo = None
    LGPIOFactory = None


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        low, high = high, low
    return max(low, min(high, value))


class PanTiltDriver(Node):
    def __init__(self):
        super().__init__('pan_tilt_driver')

        # Параметры
        self.declare_parameter('pan_gpio_pin', 5)
        self.declare_parameter('tilt_gpio_pin', 6)
        self.declare_parameter('gpio_chip', 0)
        self.declare_parameter('use_gpio', True)

        self.declare_parameter('pan_min_angle', 0.0)
        self.declare_parameter('pan_max_angle', 180.0)
        self.declare_parameter('tilt_min_angle', 0.0)
        self.declare_parameter('tilt_max_angle', 180.0)

        self.declare_parameter('pan_default_angle', 90.0)
        self.declare_parameter('tilt_default_angle', 90.0)

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('dead_zone_deg', 1.0)  # Мертвая зона в градусах
        self.declare_parameter('command_rate_limit_hz', 5.0)  # Макс частота обновления серво

        # Чтение параметров
        self.pan_gpio_pin = int(self.get_parameter('pan_gpio_pin').value)
        self.tilt_gpio_pin = int(self.get_parameter('tilt_gpio_pin').value)
        self.gpio_chip = int(self.get_parameter('gpio_chip').value)
        self.use_gpio = bool(self.get_parameter('use_gpio').value)

        self.pan_min_angle = float(self.get_parameter('pan_min_angle').value)
        self.pan_max_angle = float(self.get_parameter('pan_max_angle').value)
        self.tilt_min_angle = float(self.get_parameter('tilt_min_angle').value)
        self.tilt_max_angle = float(self.get_parameter('tilt_max_angle').value)

        self.pan_angle = float(self.get_parameter('pan_default_angle').value)
        self.tilt_angle = float(self.get_parameter('tilt_default_angle').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 10.0

        self.dead_zone_deg = float(self.get_parameter('dead_zone_deg').value)
        self.command_rate_limit_hz = float(self.get_parameter('command_rate_limit_hz').value)

        # Начальные ограничения
        self.pan_angle = clamp(self.pan_angle, self.pan_min_angle, self.pan_max_angle)
        self.tilt_angle = clamp(self.tilt_angle, self.tilt_min_angle, self.tilt_max_angle)

        # Состояние для сглаживания
        self._target_pan = self.pan_angle
        self._target_tilt = self.tilt_angle
        self._last_update_time = 0.0
        self._lock = threading.Lock()

        # Инициализация gpiozero
        self._servo_pan = None
        self._servo_tilt = None
        self._gpio_initialized = False

        if self.use_gpio and Servo is not None and LGPIOFactory is not None:
            self._init_gpiozero()
        else:
            if not self.use_gpio:
                self.get_logger().info('GPIO отключен. Режим заглушки.')
            elif Servo is None or LGPIOFactory is None:
                self.get_logger().warn('gpiozero или lgpio не установлены. Режим заглушки.')

        # Подписки и публикации
        self.command_sub = self.create_subscription(
            Vector3,
            '/pan_tilt_cmd',
            self._command_callback,
            10
        )

        self.state_pub = self.create_publisher(
            Vector3,
            '/pan_tilt_state',
            10
        )

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_state)

        # Таймер для обновления сервоприводов (с ограниченной частотой)
        if self.command_rate_limit_hz > 0:
            self.create_timer(1.0 / self.command_rate_limit_hz, self._update_servos)

        self.get_logger().info(
            f'pan_tilt_driver запущен: pan_pin={self.pan_gpio_pin}, '
            f'tilt_pin={self.tilt_gpio_pin}, chip={self.gpio_chip}'
        )
        self.get_logger().info(
            f'Начальное положение: pan={self.pan_angle:.1f}, tilt={self.tilt_angle:.1f}'
        )
        self.get_logger().info(
            f'Оптимизация: dead_zone={self.dead_zone_deg}°, '
            f'rate_limit={self.command_rate_limit_hz} Hz'
        )

    def _angle_to_servo_value(self, angle: float, min_angle: float, max_angle: float) -> float:
        if max_angle <= min_angle:
            return 0.0
        normalized = (angle - min_angle) / (max_angle - min_angle)
        normalized = clamp(normalized, 0.0, 1.0)
        return normalized * 2.0 - 1.0

    def _init_gpiozero(self):
        """Инициализация gpiozero с автоматическим определением чипа."""
        try:
            factory = None
            working_chip = None
            
            chips_to_try = [self.gpio_chip, 0, 4, 1, 2, 3]
            
            for chip_num in chips_to_try:
                try:
                    factory = LGPIOFactory(chip=chip_num)
                    working_chip = chip_num
                    break
                except Exception:
                    continue
            
            if factory is None:
                raise RuntimeError('Не удалось открыть gpiochip')

            self._servo_pan = Servo(
                self.pan_gpio_pin,
                min_pulse_width=0.5 / 1000,
                max_pulse_width=2.5 / 1000,
                pin_factory=factory
            )
            
            self._servo_tilt = Servo(
                self.tilt_gpio_pin,
                min_pulse_width=0.5 / 1000,
                max_pulse_width=2.5 / 1000,
                pin_factory=factory
            )

            # Установка начального положения
            self._servo_pan.value = self._angle_to_servo_value(
                self.pan_angle, self.pan_min_angle, self.pan_max_angle
            )
            self._servo_tilt.value = self._angle_to_servo_value(
                self.tilt_angle, self.tilt_min_angle, self.tilt_max_angle
            )

            # Повышение приоритета процесса для уменьшения дрожания
            try:
                os.nice(-10)  # Повышаем приоритет
            except PermissionError:
                self.get_logger().warn('Не удалось повысить приоритет процесса (нет прав)')

            self._gpio_initialized = True
            self.get_logger().info(
                f'gpiozero (LGPIO) инициализирован на чипе {working_chip}'
            )
        except Exception as e:
            self._gpio_initialized = False
            self.get_logger().error(f'Ошибка инициализации gpiozero: {e}')

    def _command_callback(self, msg: Vector3):
        """Прием команды /pan_tilt_cmd."""
        try:
            pan_cmd = float(msg.x)
            tilt_cmd = float(msg.y)
        except Exception:
            self.get_logger().warn('Некорректная команда пан-тилт')
            return

        if not math.isfinite(pan_cmd) or not math.isfinite(tilt_cmd):
            self.get_logger().warn('Некорректная команда: NaN или inf')
            return

        pan_cmd = clamp(pan_cmd, self.pan_min_angle, self.pan_max_angle)
        tilt_cmd = clamp(tilt_cmd, self.tilt_min_angle, self.tilt_max_angle)

        with self._lock:
            # Обновляем целевые углы
            self._target_pan = pan_cmd
            self._target_tilt = tilt_cmd

    def _update_servos(self):
        """Периодическое обновление сервоприводов с проверкой мертвой зоны."""
        if not self._gpio_initialized:
            return

        with self._lock:
            pan_diff = abs(self._target_pan - self.pan_angle)
            tilt_diff = abs(self._target_tilt - self.tilt_angle)

            # Проверяем мертвую зону
            if pan_diff > self.dead_zone_deg or tilt_diff > self.dead_zone_deg:
                old_pan = self.pan_angle
                old_tilt = self.tilt_angle

                self.pan_angle = self._target_pan
                self.tilt_angle = self._target_tilt

                if self.pan_angle != old_pan or self.tilt_angle != old_tilt:
                    self._servo_pan.value = self._angle_to_servo_value(
                        self.pan_angle, self.pan_min_angle, self.pan_max_angle
                    )
                    self._servo_tilt.value = self._angle_to_servo_value(
                        self.tilt_angle, self.tilt_min_angle, self.tilt_max_angle
                    )

    def _publish_state(self):
        """Публикация текущего состояния."""
        msg = Vector3()
        msg.x = float(self.pan_angle)
        msg.y = float(self.tilt_angle)
        msg.z = 0.0
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Завершение pan_tilt_driver...')
        if self._gpio_initialized:
            try:
                if self._servo_pan is not None:
                    self._servo_pan.detach()
                if self._servo_tilt is not None:
                    self._servo_tilt.detach()
            except Exception as e:
                self.get_logger().warn(f'Ошибка при отключении: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
