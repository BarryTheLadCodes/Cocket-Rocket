from gpiozero import AngularServo  # type: ignore
import time

# Set pin 40 as servo1 output
SERVO_1_PIN = 40
servo_1 = AngularServo(17, min_angle=-90, max_angle=90)
while True:
    angle = -90
    servo_1.angle = angle
    time.sleep(0.1)
    for n in range (18):
        angle += 10
        servo_1.angle = angle
        time.sleep(0.1)