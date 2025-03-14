import RPi.GPIO as GPIO # type: ignore
import time

# Set pin 21 as servo1 output
SERVO_1_PIN = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_1_PIN, GPIO.OUT)

# Setup PWM at 50Hz
SERVO_1 = GPIO.PWM(SERVO_1_PIN, 50)
SERVO_1.start(0)

def servo_write_angle(angle):
    """Writes angle +-90 degrees to servo"""
    angle = max(min(angle, 90), -90)  # Clamp angle between -90 and 90
    duty_cycle = angle / 18 + 2
    SERVO_1.ChangeDutyCycle(duty_cycle)

while True:
    servo_write_angle(-90)
    time.sleep(1)
    servo_write_angle(90)
    time.sleep(1)