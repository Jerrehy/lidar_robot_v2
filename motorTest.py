import RPi.GPIO as GPIO
import time

# Определение пинов (нумерация BCM)
ML_Ctrl = 26     # Пин управления направлением левого мотора (A)
ML_PWM = 27      # Пин ШИМ (скорости) левого мотора (A)
MR_Ctrl = 24     # Пин управления направлением правого мотора (B)
MR_PWM = 25      # Пин ШИМ (скорости) правого мотора (B)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def init():
    # Настройка ВСЕХ пинов на выход (включая PWM пины!)
    GPIO.setup(ML_Ctrl, GPIO.OUT)
    GPIO.setup(ML_PWM, GPIO.OUT)    # <-- ДОБАВЛЕНО: обязательно для PWM
    GPIO.setup(MR_Ctrl, GPIO.OUT)
    GPIO.setup(MR_PWM, GPIO.OUT)    # <-- ДОБАВЛЕНО: обязательно для PWM
    
    # Инициализация ШИМ для пинов скорости (частота 100 Гц - стандарт для моторов)
    pwm_ml = GPIO.PWM(ML_PWM, 100)
    pwm_mr = GPIO.PWM(MR_PWM, 100)
    
    # Запуск ШИМ с 0% заполнением (моторы остановлены)
    pwm_ml.start(0)
    pwm_mr.start(0)
    
    return pwm_ml, pwm_mr

# Вспомогательная функция: преобразует значение Arduino (0-255) 
# в процент заполнения ШИМ Raspberry Pi (0-100)
def set_pwm_speed(pwm_obj, arduino_speed_value):
    duty_cycle = (arduino_speed_value / 255.0) * 100.0
    # Ограничиваем значение диапазоном от 0.0 до 100.0
    pwm_obj.ChangeDutyCycle(min(100.0, max(0.0, duty_cycle)))

try:
    pwm_left, pwm_right = init()
    
    while True:
        # Вперед (front)
        GPIO.output(ML_Ctrl, GPIO.HIGH)
        set_pwm_speed(pwm_left, 55)
        GPIO.output(MR_Ctrl, GPIO.HIGH)
        set_pwm_speed(pwm_right, 55)
        time.sleep(2.0)
        
        # Назад (back)
        GPIO.output(ML_Ctrl, GPIO.LOW)
        set_pwm_speed(pwm_left, 200)
        GPIO.output(MR_Ctrl, GPIO.LOW)
        set_pwm_speed(pwm_right, 200)
        time.sleep(2.0)
        
        # Влево (left)
        GPIO.output(ML_Ctrl, GPIO.LOW)
        set_pwm_speed(pwm_left, 200)
        GPIO.output(MR_Ctrl, GPIO.HIGH)
        set_pwm_speed(pwm_right, 55)
        time.sleep(2.0)
        
        # Вправо (right)
        GPIO.output(ML_Ctrl, GPIO.HIGH)
        set_pwm_speed(pwm_left, 55)
        GPIO.output(MR_Ctrl, GPIO.LOW)
        set_pwm_speed(pwm_right, 200)
        time.sleep(2.0)
        
        # Стоп (stop)
        GPIO.output(ML_Ctrl, GPIO.LOW)
        set_pwm_speed(pwm_left, 0)
        GPIO.output(MR_Ctrl, GPIO.LOW)
        set_pwm_speed(pwm_right, 0)
        time.sleep(2.0)

except KeyboardInterrupt:
    # При нажатии Ctrl+C останавливаем моторы перед выходом
    set_pwm_speed(pwm_left, 0)
    set_pwm_speed(pwm_right, 0)
    print("\nПрограмма остановлена пользователем.")

finally:
    # Очистка настроек GPIO при завершении программы
    GPIO.cleanup()
