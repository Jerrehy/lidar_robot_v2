#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Драйвер IMU Waveshare 10 DOF (ICM-20948) для стадии «Минимум».

Поддерживает:
- Акселерометр (±2g по умолчанию)
- Гироскоп (±250 dps по умолчанию)
- Температура

Магнитометр и барометр на этой стадии не используются.
Главная цель — стабильно публиковать /imu/data.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import smbus
except ImportError:
    smbus = None


# Регистры ICM-20948
REG_WHO_AM_I = 0x00          # Bank 0
REG_PWR_MGMT_1 = 0x06        # Bank 0
REG_PWR_MGMT_2 = 0x07        # Bank 0
REG_INT_PIN_CFG = 0x0F       # Bank 0
REG_ACCEL_SMPLRT_DIV1 = 0x10 # Bank 2
REG_ACCEL_SMPLRT_DIV2 = 0x11 # Bank 2
REG_ACCEL_CONFIG = 0x14      # Bank 2
REG_GYRO_SMPLRT_DIV = 0x00   # Bank 2
REG_GYRO_CONFIG_1 = 0x01     # Bank 2

# Регистры данных (Bank 0)
REG_ACCEL_XOUT_H = 0x2D
REG_TEMP_OUT_H = 0x39
REG_GYRO_XOUT_H = 0x33

# Константы
ICM20948_WHO_AM_I_VALUE = 0xEA
ACCEL_RANGE_G = 2.0          # ±2g
GYRO_RANGE_DPS = 250.0       # ±250 dps
ACCEL_SENSITIVITY = 16384.0  # LSB/g для ±2g
GYRO_SENSITIVITY = 131.0     # LSB/dps для ±250 dps


def int16_from_bytes(high: int, low: int) -> int:
    """Преобразует два байта в знаковый int16."""
    value = (high << 8) | low
    if value >= 32768:
        value -= 65536
    return value


class ICM20948Driver:
    """Низкоуровневый драйвер ICM-20948."""

    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        self._current_bank = -1

    def _select_bank(self, bank: int):
        """Переключение банка регистров (0-3)."""
        if bank != self._current_bank:
            self.bus.write_byte_data(self.address, 0x7F, bank & 0x03)
            self._current_bank = bank

    def _read_reg(self, bank: int, reg: int) -> int:
        """Чтение одного регистра."""
        self._select_bank(bank)
        return self.bus.read_byte_data(self.address, reg)

    def _write_reg(self, bank: int, reg: int, value: int):
        """Запись одного регистра."""
        self._select_bank(bank)
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def _read_block(self, bank: int, reg: int, length: int) -> list:
        """Чтение блока регистров."""
        self._select_bank(bank)
        return self.bus.read_i2c_block_data(self.address, reg, length)

    def check_who_am_i(self) -> bool:
        """Проверка WHO_AM_I регистра."""
        try:
            who = self._read_reg(0, REG_WHO_AM_I)
            return who == ICM20948_WHO_AM_I_VALUE
        except Exception:
            return False

    def reset(self):
        """Сброс устройства."""
        # Бит 7 в PWR_MGMT_1 - сброс
        self._write_reg(0, REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)  # Ждем сброса
        
        # Ожидание готовности
        for _ in range(10):
            try:
                val = self._read_reg(0, REG_PWR_MGMT_1)
                if (val & 0x80) == 0:
                    break
            except Exception:
                pass
            time.sleep(0.01)

    def init(self):
        """Инициализация датчика."""
        # Выход из режима сна
        # Бит 6 (SLEEP) = 0, Бит 0 (CLKSEL) = 1 (auto clock)
        self._write_reg(0, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.01)

        # Включение акселерометра и гироскопа
        # Все биты = 0 (включены)
        self._write_reg(0, REG_PWR_MGMT_2, 0x00)
        time.sleep(0.01)

        # Настройка акселерометра (Bank 2)
        # ACCEL_CONFIG: DLPCFG=0 (448Hz), AFS_SEL=0 (±2g)
        self._write_reg(2, REG_ACCEL_CONFIG, 0x00)
        
        # Sample rate делитель для акселерометра
        # ODR = 1125Hz / (1 + ACCEL_SMPLRT_DIV)
        # Для 100Hz: DIV = 10 (примерно)
        self._write_reg(2, REG_ACCEL_SMPLRT_DIV1, 0x00)
        self._write_reg(2, REG_ACCEL_SMPLRT_DIV2, 0x0A)

        # Настройка гироскопа (Bank 2)
        # GYRO_CONFIG_1: DLPF=0 (196.6Hz), GYRO_FS_SEL=0 (±250 dps)
        self._write_reg(2, REG_GYRO_CONFIG_1, 0x00)
        
        # Sample rate делитель для гироскопа
        # ODR = 1125Hz / (1 + GYRO_SMPLRT_DIV)
        self._write_reg(2, REG_GYRO_SMPLRT_DIV, 0x0A)

    def read_accel_gyro(self):
        """
        Чтение акселерометра и гироскопа.
        Возвращает: (ax, ay, az, gx, gy, gz) в м/с² и рад/с
        """
        # Чтение 12 байт: акселерометр (6) + гироскоп (6)
        data = self._read_block(0, REG_ACCEL_XOUT_H, 12)
        
        # Акселерометр (big-endian)
        ax_raw = int16_from_bytes(data[0], data[1])
        ay_raw = int16_from_bytes(data[2], data[3])
        az_raw = int16_from_bytes(data[4], data[5])
        
        # Гироскоп
        gx_raw = int16_from_bytes(data[6], data[7])
        gy_raw = int16_from_bytes(data[8], data[9])
        gz_raw = int16_from_bytes(data[10], data[11])
        
        # Преобразование в физические единицы
        ax = (ax_raw / ACCEL_SENSITIVITY) * 9.80665  # м/с²
        ay = (ay_raw / ACCEL_SENSITIVITY) * 9.80665
        az = (az_raw / ACCEL_SENSITIVITY) * 9.80665
        
        gx = (gx_raw / GYRO_SENSITIVITY) * math.pi / 180.0  # рад/с
        gy = (gy_raw / GYRO_SENSITIVITY) * math.pi / 180.0
        gz = (gz_raw / GYRO_SENSITIVITY) * math.pi / 180.0
        
        return ax, ay, az, gx, gy, gz


class ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        # Параметры
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        # Калибровка в физических единицах:
        # gyro_offset — рад/с,
        # accel_offset — м/с^2.
        self.declare_parameter('gyro_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('accel_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('accel_scale', [1.0, 1.0, 1.0])

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.gyro_offset = self._get_vec3_param('gyro_offset')
        self.accel_offset = self._get_vec3_param('accel_offset')
        self.gyro_scale = self._get_vec3_param('gyro_scale')
        self.accel_scale = self._get_vec3_param('accel_scale')

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn('publish_rate_hz <= 0, использую 50.0 Hz')
            self.publish_rate_hz = 50.0

        self._bus = None
        self._imu = None
        self._initialized = False
        self._last_init_time = 0.0
        self._last_error_log = 0.0
        self._error_count = 0

        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

        self._init_hardware()

        self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)

        self.get_logger().info(
            f'IMU driver запущен: ICM-20948, '
            f'i2c_bus={self.i2c_bus}, address=0x{self.i2c_address:02X}'
        )

    def _get_vec3_param(self, name: str):
        """Возвращает список из 3 float."""
        value = self.get_parameter(name).value
        default = [1.0, 1.0, 1.0] if 'scale' in name else [0.0, 0.0, 0.0]

        if value is None:
            return default

        try:
            if len(value) != 3:
                self.get_logger().warn(f'Параметр {name} должен иметь длину 3')
                return default
            return [float(value[0]), float(value[1]), float(value[2])]
        except Exception:
            self.get_logger().warn(f'Некорректный параметр {name}, использую {default}')
            return default

    def _init_hardware(self):
        """Инициализирует I2C и ICM-20948."""
        self._last_init_time = time.monotonic()

        if smbus is None:
            self.get_logger().error(
                'Модуль smbus не установлен. '
                'Выполните: sudo apt install python3-smbus'
            )
            self._initialized = False
            return

        try:
            # Создание шины I2C
            if self._bus is None:
                self._bus = smbus.SMBus(self.i2c_bus)

            # Создание драйвера
            self._imu = ICM20948Driver(self._bus, self.i2c_address)

            # Проверка WHO_AM_I
            if not self._imu.check_who_am_i():
                raise RuntimeError(
                    f'ICM-20948 не найден по адресу 0x{self.i2c_address:02X}. '
                    f'Ожидалось WHO_AM_I=0x{ICM20948_WHO_AM_I_VALUE:02X}'
                )

            # Сброс устройства
            self.get_logger().info('Выполняется сброс ICM-20948...')
            self._imu.reset()
            
            # Инициализация
            self.get_logger().info('Инициализация ICM-20948...')
            self._imu.init()

            self._initialized = True
            self._error_count = 0
            self.get_logger().info('ICM-20948 успешно инициализирован')

        except Exception as e:
            self._initialized = False
            self._imu = None
            self.get_logger().error(f'Ошибка инициализации IMU: {e}')

    def _apply_calibration(self, accel, gyro):
        """Применяет калибровку: value = value * scale - offset"""
        ax = accel[0] * self.accel_scale[0] - self.accel_offset[0]
        ay = accel[1] * self.accel_scale[1] - self.accel_offset[1]
        az = accel[2] * self.accel_scale[2] - self.accel_offset[2]

        gx = gyro[0] * self.gyro_scale[0] - self.gyro_offset[0]
        gy = gyro[1] * self.gyro_scale[1] - self.gyro_offset[1]
        gz = gyro[2] * self.gyro_scale[2] - self.gyro_offset[2]

        return [ax, ay, az], [gx, gy, gz]

    def _timer_callback(self):
        """Периодическое чтение датчика и публикация /imu/data."""
        now_mono = time.monotonic()

        if not self._initialized:
            # Пробуем переинициализировать не чаще, чем раз в 5 секунд
            if now_mono - self._last_init_time > 5.0:
                self.get_logger().warn('Повторная инициализация IMU')
                self._init_hardware()
            return

        try:
            # Чтение данных
            ax, ay, az, gx, gy, gz = self._imu.read_accel_gyro()

            # Применение калибровки
            accel_calibrated, gyro_calibrated = self._apply_calibration(
                [ax, ay, az],
                [gx, gy, gz]
            )

            # Публикация
            self._publish_imu(gyro_calibrated, accel_calibrated)
            
            # Сброс счетчика ошибок при успешном чтении
            self._error_count = 0

        except Exception as e:
            self._error_count += 1
            
            # Логируем ошибку не чаще раза в 5 секунд
            if now_mono - self._last_error_log > 5.0:
                self._last_error_log = now_mono
                self.get_logger().error(
                    f'Ошибка чтения IMU (попытка {self._error_count}): {e}'
                )

            # Если ошибок слишком много, переинициализируем
            if self._error_count > 10:
                self.get_logger().warn('Слишком много ошибок, переинициализация...')
                self._initialized = False

    def _publish_imu(self, gyro, accel):
        """Публикует сообщение Imu."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])

        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])

        # Ориентация не публикуется
        msg.orientation_covariance[0] = -1

        # Простые диагональные ковариации
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
