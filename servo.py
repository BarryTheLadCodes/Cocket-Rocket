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
    # Assuming a 50Hz PWM frequency (20ms period)
    # 1ms pulse width corresponds to 5% duty cycle
    # 2ms pulse width corresponds to 10% duty cycle
    min_duty = 5
    max_duty = 10
    # Convert angle from -90 to 90 degrees to 0 to 180 degrees
    angle = angle + 90
    duty_cycle = min_duty + (angle / 180.0) * (max_duty - min_duty)
    return duty_cycle

while True:
    servo_write_angle(-90)
    time.sleep(1)
    servo_write_angle(90)
    time.sleep(1)