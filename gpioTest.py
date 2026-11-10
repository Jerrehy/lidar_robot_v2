import RPi.GPIO as GPIO
import time

servoPin1 = 6
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def init():
    GPIO.setup(servoPin1, GPIO.OUT)
    
def servoPulse(servoPin, myangle):
    pulsewidth = (myangle*11) + 500  # The pulse width
    GPIO.output(servoPin,GPIO.HIGH)
    time.sleep(pulsewidth/1000000.0)
    GPIO.output(servoPin,GPIO.LOW)
    time.sleep(20.0/1000 - pulsewidth/1000000.0) # The cycle of 20 ms

try:
    init()
    while True:
        # servoPulse(servoPin1, 90)
        
        for i in range(0,180):
            servoPulse(servoPin1, i)

        for i in range(0,180):
            i = 180 - i
            servoPulse(servoPin1, i)
        
except KeyboardInterrupt:
    pass
GPIO.cleanup()
